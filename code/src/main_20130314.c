/**************************************************************************//**
 * @file     Smpl_DrvPWM.c
 * @version  V1.00
 * $Revision: 11 $
 * $Date: 12/08/09 7:27p $ 
 * @brief    M051 Series PWM Generator and Capture Timer Driver Sample Code
 *
 * @note
 * Copyright (C) 2011 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "M051Series.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Macro, type and constant definitions                                                                    */
/*---------------------------------------------------------------------------------------------------------*/

#define PLLCON_SETTING      SYSCLK_PLLCON_50MHz_XTAL
#define PLL_CLOCK           50000000


/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
//volatile uint8_t g_u8PWMCount = 1;
//volatile uint16_t g_u16Frequency;
//volatile uint32_t g_u32Pulse = 0;
unsigned char pwm0_count=0;
unsigned char pwm1_count=0;
unsigned char linght_gray0=0;
unsigned char linght_gray1=0;
/* Assume PWM output frequency is 523Hz and duty ratio is 60%, user can calculate PWM settings by follows.
   PWM clock source frequency = __XTAL = 12000000 in the sample code.
   (CNR+1) = PWM clock source frequency/prescaler/clock source divider/PWM output frequency 
           = 12000000/2/1/523 = 11472 < 65536  (Note: If calculated value is larger than 65536, user should increase prescale value.)
   CNR = 11471 =>g_au16ScaleCnr[0] = 11471
   duty ratio = 60% = (CMR+1)/(CNR+1) ==> CMR = (CNR+1)*60/100-1 = 11472*60/100-1
   CMR = 6882 =>g_au16ScaleCmr[0] = 6882
*/ 
unsigned int Brightness[21]={0,2,3,5,9,16,28,49,84,147,256,446,776,1351,2353,4096,7131,12417,21619,37640,65535};

/**
 * @brief       PWMA IRQ Handler
 *
 * @param       None
 *
 * @return      None
 *
 * @details     ISR to handle PWMA interrupt event
 */
void PWMA_IRQHandler(void)
{    
    /*In this sample code, user will not use PWMA channel 0~3 PIIR interrupt and CAPIF capture interrupt. 
      Defined following code as 0 for reducing interrupt execution time and code size. */  
	  uint32_t u32PwmIntFlag,u32CapIntFlag;
	/*==============PWM==================*/
	 u32PwmIntFlag = PWMA->PIIR;
    
    /* PWMB channel 0 PWM timer interrupt */ 
    if (u32PwmIntFlag & PWM_PIIR_PWMIF0_Msk)
    {
        PWMA->PIIR = PWM_PIIR_PWMIF0_Msk; 
		pwm0_count++;
        //PWM_PwmIRQHandler();
        if(pwm0_count>=30)
        {
			pwm0_count=0;
			PWMA->CMR0=Brightness[linght_gray0];
			linght_gray0++;
			if(linght_gray0>20)
				linght_gray0=0;
		}
    }  
	if (u32PwmIntFlag & PWM_PIIR_PWMIF1_Msk)
    {
        PWMA->PIIR = PWM_PIIR_PWMIF1_Msk; 
		pwm1_count++;
        //PWM_PwmIRQHandler();
        if(pwm1_count>=30)
        {
			pwm1_count=0;
			//PWMA->CMR1=Brightness[linght_gray1];
			PWMA->CMR1=Brightness[19];
			linght_gray1++;
			if(linght_gray1>20)
				linght_gray1=0;
		}
    } 


	/*===============capture=================*/
	u32CapIntFlag = PWMA->CCR2;
    /* PWMB channel 2 Capture interrupt */
    if (u32CapIntFlag & PWM_CCR2_CAPIF2_Msk) 
    {
		P36=1;
		PWMA->CCR2 |= PWM_CCR2_CAPIF2_Msk;
		if(u32CapIntFlag & PWM_CCR2_CFLRI2_Msk) //FALLING 
		{
			PWMA->CCR2 |=PWM_CCR2_CFLRI2_Msk; //CLEAR FLAG
			//P36=1;
		}
		if(u32CapIntFlag & PWM_CCR2_CRLRI2_Msk) //rsing
		{
			PWMA->CCR2 |=PWM_CCR2_CRLRI2_Msk; //CLEAR FLAG
			//P36=1;
		}
    }
	
}

/**
 * @brief       PWMB IRQ Handler
 *
 * @param       None
 *
 * @return      None
 *
 * @details     ISR to handle PWMB interrupt event
 */
 /*
void PWMB_IRQHandler(void)
{    
}
/*



/*--------------------------------------------------------------------------------------*/
/* Capture function to calculate the input waveform information                         */
/* u32Count[4] : Keep the internal counter value when input signal rising / falling     */
/*               happens                                                                */
/*                                                                                      */
/* time    A    B     C     D                                                           */
/*           ___   ___   ___   ___   ___   ___   ___   ___                              */
/*      ____|   |_|   |_|   |_|   |_|   |_|   |_|   |_|   |_____                        */
/* index              0 1   2 3                                                         */
/*                                                                                      */
/* The capture internal counter down count from 0x10000, and reload to 0x10000 after    */
/* input signal falling happens (Time B/C/D)                                            */
/*--------------------------------------------------------------------------------------*/                              
void CalPeriodTime(PWM_T *PWM, uint32_t u32Ch)
{
    uint16_t u32Count[4];
    uint32_t u32i;
    uint16_t u16RisingTime, u16FallingTime, u16HighPeroid, u16LowPeroid, u16TotalPeroid;
    
    /* Clear Capture Falling Indicator (Time A) */
    _PWM_CLR_CAP_FALLING_INDICATOR(PWM, u32Ch);
    
    /* Wait for Capture Falling Indicator  */
    while(_PWM_GET_CAP_FALLING_INDICATOR(PWM, u32Ch)==0);    
    /* Clear Capture Falling Indicator (Time B)*/
    _PWM_CLR_CAP_FALLING_INDICATOR(PWM, u32Ch);
    
    u32i = 0;
    
    while (u32i < 4)
    {
        /* Wait for Capture Falling Indicator */
        while(_PWM_GET_CAP_FALLING_INDICATOR(PWM, u32Ch)==0);
        /* Clear Capture Falling Indicator */
        _PWM_CLR_CAP_FALLING_INDICATOR(PWM, u32Ch);

        /* Clear Capture Rising Indicator */
        _PWM_CLR_CAP_RISING_INDICATOR(PWM, u32Ch);
                
        /* Get Capture Falling Latch Counter Data */
        u32Count[u32i++] = _PWM_GET_CAP_FALLING_LATCH_VALUE(PWM, u32Ch);
        
        /* Wait for Capture Rising Indicator */
        while(_PWM_GET_CAP_RISING_INDICATOR(PWM, u32Ch)==0);
        
        /* Clear Capture Rising Indicator */
        _PWM_CLR_CAP_RISING_INDICATOR(PWM, u32Ch);
        
        /* Get Capture Rising Latch Counter Data */
        u32Count[u32i++] = _PWM_GET_CAP_RISING_LATCH_VALUE(PWM, u32Ch);
    }   
    
    u16RisingTime = u32Count[1];
    
    u16FallingTime = u32Count[0];
    
    u16HighPeroid = u32Count[1] - u32Count[2];
    
    u16LowPeroid = 0x10000 - u32Count[1];
    
    u16TotalPeroid = 0x10000 - u32Count[2];
        
   
}


void SYS_Init(void)
{
/*---------------------------------------------------------------------------------------------------------*/
/* Init System Clock                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable Internal RC clock */
    SYSCLK->PWRCON |= SYSCLK_PWRCON_IRC22M_EN_Msk;

    /* Waiting for IRC22M clock ready */
    SYS_WaitingForClockReady(SYSCLK_CLKSTATUS_IRC22M_STB_Msk);

    /* Switch HCLK clock source to Internal RC */
    SYSCLK->CLKSEL0 = SYSCLK_CLKSEL0_HCLK_IRC22M;

    /* Set PLL to power down mode and PLL_STB bit in CLKSTATUS register will be cleared by hardware.*/
    SYSCLK->PLLCON |= SYSCLK_PLLCON_PD_Msk;

    /* Enable external 12MHz XTAL, internal 22.1184MHz */
    SYSCLK->PWRCON |= SYSCLK_PWRCON_XTL12M_EN_Msk | SYSCLK_PWRCON_IRC22M_EN_Msk;

    /* Enable PLL and Set PLL frequency */        
    SYSCLK->PLLCON = PLLCON_SETTING;
    
    /* Waiting for clock ready */
    SYS_WaitingForClockReady(SYSCLK_CLKSTATUS_PLL_STB_Msk | SYSCLK_CLKSTATUS_XTL12M_STB_Msk | SYSCLK_CLKSTATUS_IRC22M_STB_Msk);

    /* Switch HCLK clock source to PLL, STCLK to HCLK/2 */
    SYSCLK->CLKSEL0 = SYSCLK_CLKSEL0_STCLK_HCLK_DIV2 | SYSCLK_CLKSEL0_HCLK_PLL;

    /* Enable IP clock */        
    SYSCLK->APBCLK = SYSCLK_APBCLK_PWM01_EN_Msk;
  
    /* IP clock source */
    SYSCLK->CLKSEL1 = SYSCLK_CLKSEL1_PWM01_XTAL | SYSCLK_CLKSEL1_PWM23_XTAL;
    
    /* Reset PWMB channel0~channel3 */                    
    SYS->IPRSTC2 = SYS_IPRSTC2_PWM03_RST_Msk;                    
    SYS->IPRSTC2 = 0;      

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    //SystemCoreClockUpdate(); 
    PllClock        = PLL_CLOCK;            // PLL
    SystemCoreClock = PLL_CLOCK / 1;        // HCLK
    CyclesPerUs     = PLL_CLOCK / 1000000;  // For SYS_SysTickDelay()

/*---------------------------------------------------------------------------------------------------------*/
/* Init I/O Multi-function                                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
 
    /* Set P2 multi-function pins for PWMA Channel0~3  */
    SYS->P2_MFP = SYS_MFP_P20_PWM0|SYS_MFP_P21_PWM1|SYS_MFP_P22_PWM2;

    /* Lock protected registers */
    SYS_LockReg();
}

void open_pwm(char channel)
{
	 /* Assume PWM output frequency is 523Hz and duty ratio is 60%, user can calculate PWM settings by follows.
        duty ratio = (CMR+1)/(CNR+1)
        cycle time = CNR+1
        High level = CMR+1
        PWM clock source frequency = __XTAL = 12000000
        (CNR+1) = PWM clock source frequency/prescaler/clock source divider/PWM output frequency 
               = 12000000/2/1/523 = 11472 
        (Note: CNR is 16 bits, so if calculated value is larger than 65536, user should increase prescale value.)
        CNR = 11471
        duty ratio = 60% ==> (CMR+1)/(CNR+1) = 60% ==> CMR = (CNR+1)*0.6-1 = 11472*60/100-1
        CMR = 6882
        Prescale value is 1 : prescaler= 2
        Clock divider is PWM_CSR_DIV1 : clock divider =1             
       */  
        char pwm_channel;
	    if(channel==0)
			pwm_channel=PWM_CH0;
		else
			pwm_channel=PWM_CH1;
        /*Set Pwm mode*/
        _PWM_SET_TIMER_AUTO_RELOAD_MODE(PWMA,pwm_channel);  
                            
        /*Set PWM Timer clock prescaler*/
       _PWM_SET_TIMER_PRESCALE(PWMA,pwm_channel,1); // Divided by 2  
                                                 
       /*Set PWM Timer clock divider select*/
       _PWM_SET_TIMER_CLOCK_DIV(PWMA,pwm_channel,PWM_CSR_DIV1); 
        
       if(pwm_channel==PWM_CH0) 
       {
       	 /*Set PWM Timer duty*/
         //PWMA->CMR0 = g_au16ScaleCmr[0];
         PWMA->CMR0 = 0;                                    
         /*Set PWM Timer period*/
         PWMA->CNR0 = 0xffff;
       }
	   else
	   {
       	 /*Set PWM Timer duty*/
         //PWMA->CMR0 = g_au16ScaleCmr[0];
         PWMA->CMR1 = 0;                                    
         /*Set PWM Timer period*/
         PWMA->CNR1 = 0xffff;
       }
       /* Enable PWM Output pin */
      _PWM_ENABLE_PWM_OUT(PWMA,pwm_channel); 

      /* Enable Timer period Interrupt */
      _PWM_ENABLE_TIMER_PERIOD_INT(PWMA,pwm_channel);
                        
      /* Enable PWMB NVIC */
      NVIC_EnableIRQ((IRQn_Type)(PWMA_IRQn)); 
                                                            
      /* Enable PWM Timer */
      _PWM_ENABLE_TIMER(PWMA,pwm_channel); 
}

void close_pwm(char channel)
{
	   /*--------------------------------------------------------------------------------------*/
       /* Stop PWM Timer (Recommended procedure method 2)                                      */
       /* Set PWM Timer counter as 0, When interrupt request happen, disable PWM Timer         */          
       /* Set PWM Timer counter as 0 in Call back function                                     */                              
       /*--------------------------------------------------------------------------------------*/                          
		char pwm_channel;
	    if(channel==1)
			pwm_channel=PWM_CH1;
		else
			pwm_channel=PWM_CH0;
        /* Disable PWMB NVIC */
        NVIC_DisableIRQ((IRQn_Type)(PWMA_IRQn)); 

        /* Wait until PWMB channel 0 Timer Stop */  
		if(channel==1)
			while(PWMA->PDR1!=0);
		else	
        	while(PWMA->PDR0!=0);
                                                                                        
        /* Disable the PWM Timer */
        _PWM_DISABLE_TIMER(PWMA,pwm_channel); 
                        
        /* Disable PWM Output pin */
       _PWM_DISABLE_PWM_OUT(PWMA, pwm_channel);                         

    	/* Disable PWMB clock engine */
      //  SYSCLK->APBCLK&=(~SYSCLK_APBCLK_PWM01_EN_Msk);  
}



/*****************************/
//open PWMA capture 2
void open_capture(void)
{
	//pwm timer2 clk=HCLK/2/1
	PWMA->PPR |= PWM_PPR_CP23(150);  //set every step is 0.2ms  1hz ~ 5khz
	PWMA->CSR |= PWM_CSR_CSR2(PWM_CSR_DIV16);
	PWMA->PCR |= PWM_PCR_CH2EN_ENABLE |PWM_PCR_CH2MOD_AUTO_RELOAD;
	PWMA->CCR2 |= PWM_CCR2_CFLRI2_Msk | PWM_CCR2_CRLRI2_Msk | PWM_CCR2_CAPCH2EN_Msk | 
					PWM_CCR2_CFL_IE2_Msk |PWM_CCR2_CRL_IE2_Msk;
	PWMA->CNR2 = 10000;
	PWMA->CMR2=0;
	//PWMA->POE &= 0x0b;  //DISABLE POE PWM2 PIN
	PWMA->PIER |= PWM_PIER_PWMIE2_Msk;
	
    NVIC_EnableIRQ((IRQn_Type)(PWMA_IRQn));
	while(PWMA->PDR2 ==0);
	PWMA->CAPENR = 0x04;
	
}
/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main (void)
{   
	unsigned int i=0;
	/* Init System, IP clock and multi-function I/O */
    SYS_Init(); //In the end of SYS_Init() will issue SYS_LockReg() to lock protected register. If user want to write protected register, please issue SYS_UnlockReg() to unlock protected register.       
	_GPIO_SET_PIN_MODE(P0,1,GPIO_PMD_OUTPUT);
	_GPIO_SET_PIN_MODE(P3,6,GPIO_PMD_OUTPUT);
	open_pwm(0);
	open_pwm(1);
	open_capture();
	while(1)
	{
		for(i=0;i<65535;i++);
		for(i=0;i<65535;i++);
		for(i=0;i<65535;i++);
		for(i=0;i<65535;i++);
		for(i=0;i<65535;i++);
		for(i=0;i<65535;i++);
		for(i=0;i<65535;i++);
		for(i=0;i<65535;i++);
		for(i=0;i<65535;i++);
		for(i=0;i<65535;i++);
		//P36=1;
		for(i=0;i<65535;i++);
		for(i=0;i<65535;i++);
		for(i=0;i<65535;i++);
		for(i=0;i<65535;i++);
		for(i=0;i<65535;i++);
		for(i=0;i<65535;i++);
		for(i=0;i<65535;i++);
		for(i=0;i<65535;i++);
		for(i=0;i<65535;i++);
		for(i=0;i<65535;i++);
		//P36=0;	
	}
	return 0;  
}




