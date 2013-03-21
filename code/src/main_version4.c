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
#define LIGHT_RESISTANCES  P24
#define CAPTURE_ALL_TIME    4
#define DAY  1
#define NIGHT 0
#define CAPTURE_TEST
/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
unsigned char pwm0_count=0;
unsigned char pwm1_count=0;
unsigned char linght_gray0=0;
unsigned char linght_gray1=0;
unsigned int    timer0_100ms_count=0;

char capture_start_enable=0;
char capture_first_edge=2;  //0:falling 1:resing
char capture_lose_flag=0;
char capture_wave_count=0;

char light_flag=DAY; //default is day

unsigned int capture_value_array[CAPTURE_ALL_TIME]; //use save capture value form interrput 

unsigned int Brightness[21]={0,2,3,5,9,16,28,49,84,147,256,446,776,1351,2353,4096,7131,12417,21619,37640,65535};



void capture_interrupt_save_data(char egde_type);

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
        
		pwm0_count++;
        //PWM_PwmIRQHandler();
        if(pwm0_count>=20)
        {
			pwm0_count=0;
			if(linght_gray0<20)
			{	
				PWMA->CMR0=Brightness[linght_gray0];
				linght_gray0++;
			}
			
			
		}
		PWMA->PIIR |= PWM_PIIR_PWMIF0_Msk; 
    }  
   if (u32PwmIntFlag & PWM_PIIR_PWMIF1_Msk)
    {
       
		pwm1_count++;
        //PWM_PwmIRQHandler();
        if(pwm1_count>=20)
        {
			pwm1_count=0;
			if(linght_gray1<20)
			{	
				PWMA->CMR1=Brightness[linght_gray1];
				linght_gray1++;
			}

		}
		 PWMA->PIIR |= PWM_PIIR_PWMIF1_Msk; 
    } 

   if(u32PwmIntFlag & PWM_PIIR_PWMIF2_Msk)
   {	
   	
	if(capture_start_enable && (capture_wave_count!=0) ) //capture timeout clear capture pamater
	{
		capture_first_edge=2;
		capture_lose_flag=0;
		capture_wave_count=0;
	}
	PWMA->PIIR |= PWM_PIIR_PWMIF2_Msk;
   }

   if(u32PwmIntFlag & PWM_PIIR_PWMIF3_Msk)
   {	
   		PWMA->PIIR |= PWM_PIIR_PWMIF3_Msk;
   }
	/*===============capture=================*/
	u32CapIntFlag = PWMA->CCR2;
    /* PWMB channel 2 Capture interrupt */
    if (u32CapIntFlag & PWM_CCR2_CAPIF2_Msk) 
    {
		PWMA->CCR2 |= PWM_CCR2_CAPIF2_Msk;
		if(u32CapIntFlag & PWM_CCR2_CFLRI2_Msk) //FALLING 
		{
			
			//P36=1;
			capture_interrupt_save_data(0);
			PWMA->CCR2 |=PWM_CCR2_CFLRI2_Msk; //CLEAR FLAG
			
		}
		if(u32CapIntFlag & PWM_CCR2_CRLRI2_Msk) //rsing
		{
			
			//P36=0;
			capture_interrupt_save_data(1);
			PWMA->CCR2 |=PWM_CCR2_CRLRI2_Msk; //CLEAR FLAG
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
*/

void TMR0_IRQHandler(void)
{
	 TIMER0->TISR=1;
	 timer0_100ms_count++;
}

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
#if 0
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
#endif

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
    SYSCLK->PWRCON |= SYSCLK_PWRCON_XTL12M_EN_Msk | SYSCLK_PWRCON_IRC22M_EN_Msk |SYSCLK_PWRCON_IRC10K_EN_Msk;

    /* Enable PLL and Set PLL frequency */        
    SYSCLK->PLLCON = PLLCON_SETTING;
    
    /* Waiting for clock ready */
    SYS_WaitingForClockReady(SYSCLK_CLKSTATUS_PLL_STB_Msk | SYSCLK_CLKSTATUS_XTL12M_STB_Msk | SYSCLK_CLKSTATUS_IRC22M_STB_Msk |SYSCLK_CLKSTATUS_IRC10K_STB_Msk);

    /* Switch HCLK clock source to PLL, STCLK to HCLK/2 */
    SYSCLK->CLKSEL0 = SYSCLK_CLKSEL0_STCLK_HCLK_DIV2 | SYSCLK_CLKSEL0_HCLK_PLL;

    /* Enable IP clock */        
    SYSCLK->APBCLK = SYSCLK_APBCLK_PWM01_EN_Msk | SYSCLK_APBCLK_PWM23_EN_Msk | SYSCLK_APBCLK_TMR0_EN_Msk |SYSCLK_APBCLK_UART0_EN_Msk |SYSCLK_APBCLK_WDT_EN_Msk;
  
    /* IP clock source */
    SYSCLK->CLKSEL1 = SYSCLK_CLKSEL1_PWM01_XTAL | SYSCLK_CLKSEL1_PWM23_XTAL |SYSCLK_CLKSEL1_TMR0_XTAL |SYSCLK_CLKSEL1_UART_PLL | SYSCLK_CLKSEL1_WDT_IRC10K;

    /* Reset PWMB channel0~channel3  and Reset TIMER0*/                    
    SYS->IPRSTC2 = SYS_IPRSTC2_PWM03_RST_Msk | SYS_IPRSTC2_TMR0_RST_Msk |SYS_IPRSTC2_UART0_RST_Msk;                    
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
    //SYS->P2_MFP = SYS_MFP_P20_PWM0|SYS_MFP_P21_PWM1|SYS_MFP_P22_PWM2;
     SYS->P3_MFP = SYS_MFP_P30_RXD0 | SYS_MFP_P31_TXD0 ;
     SYS->P2_MFP = SYS_MFP_P22_PWM2;
    /* Lock protected registers */
    SYS_LockReg();
}

/*---------------------------------------------------------------------------------------------------------*/
/* Init UART                                                                                               */
/*---------------------------------------------------------------------------------------------------------*/

void UART0_Init(void)
{
    /* Configure UART0 and set UART0 Baudrate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_DIV_MODE2(PLL_CLOCK, 115200);
    _UART_SET_DATA_FORMAT(UART0, UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1);
}



/*
	PWM3 for test capture
*/
#ifdef CAPTURE_TEST
void open_pwm3(void)
{
	
		SYS_UnlockReg();
		SYS->P2_MFP |= SYS_MFP_P23_PWM3;
		SYS_LockReg();
	
        /*Set Pwm mode*/
        _PWM_SET_TIMER_AUTO_RELOAD_MODE(PWMA,PWM_CH3);  
                            
        /*Set PWM Timer clock prescaler*/
       _PWM_SET_TIMER_PRESCALE(PWMA,PWM_CH3,150); // Divided by 2  
                                                 
       /*Set PWM Timer clock divider select*/
       _PWM_SET_TIMER_CLOCK_DIV(PWMA,PWM_CH3,PWM_CSR_DIV1); 
        
         PWMA->CMR3 = 4000;                                    
         /*Set PWM Timer period*/
         PWMA->CNR3 = 40000;
      
       /* Enable PWM Output pin */
      _PWM_ENABLE_PWM_OUT(PWMA,PWM_CH3); 

      /* Enable Timer period Interrupt */
      _PWM_ENABLE_TIMER_PERIOD_INT(PWMA,PWM_CH3);
                        
      /* Enable PWMB NVIC */
      NVIC_EnableIRQ((IRQn_Type)(PWMA_IRQn)); 
                                                            
      /* Enable PWM Timer */
      _PWM_ENABLE_TIMER(PWMA,PWM_CH3); 
}
#endif







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
		{	
			pwm_channel=PWM_CH0;
			SYS_UnlockReg();
			SYS->P2_MFP |= SYS_MFP_P20_PWM0;
			SYS_LockReg();
		}
		else
		{	
			pwm_channel=PWM_CH1;
			SYS_UnlockReg();
			SYS->P2_MFP |= SYS_MFP_P21_PWM1;
			SYS_LockReg();
		}
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
         PWMA->CMR1 = 0;
         //PWMA->CMR1 = Brightness[19];                                    
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
		{	
			pwm_channel=PWM_CH1;
			PWMA->PIER &= ~PWM_PIER_PWMIE1_Msk;
		}
		else
		{	
			pwm_channel=PWM_CH0;
			PWMA->PIER &= ~PWM_PIER_PWMIE0_Msk;
		}

        	/* Disable PWMB NVIC */
			
        	//NVIC_DisableIRQ((IRQn_Type)(PWMA_IRQn)); 

       	 /* Wait until PWMB channel 0 Timer Stop */  
		if(channel==1)
		{
			while(PWMA->PDR1!=0);
			linght_gray1=0;
		}
		else	
		{
        		while(PWMA->PDR0!=0);
			linght_gray0=0;
		}                                                                           
        /* Disable the PWM Timer */
        _PWM_DISABLE_TIMER(PWMA,pwm_channel); 
                        
        /* Disable PWM Output pin */
       _PWM_DISABLE_PWM_OUT(PWMA, pwm_channel);                         
   
   
	if(channel==1)
    {	
		SYS_UnlockReg();	
		SYS->P2_MFP |= SYS_MFP_P21_GPIO;
		SYS_LockReg();
		_GPIO_SET_PIN_MODE(P2,1,GPIO_PMD_OUTPUT);
		P21=0;
	}
	else
	{	
		SYS_UnlockReg();	
		SYS->P2_MFP |= SYS_MFP_P20_GPIO;
		SYS_LockReg();
		_GPIO_SET_PIN_MODE(P2,0,GPIO_PMD_OUTPUT);
		P20=0;
	}	
   
}



/*****************************/
//open PWMA capture 2
void open_capture(void)
{
	//pwm timer2 clk=HCLK/2/1
	PWMA->PPR |= PWM_PPR_CP23(150);  //set every step is 0.2ms  1hz ~ 5khz
	PWMA->CSR |= PWM_CSR_CSR2(PWM_CSR_DIV16);
	PWMA->PCR |= PWM_PCR_CH2EN_Msk | PWM_PCR_CH2MOD_AUTO_RELOAD;
	PWMA->CCR2 |= PWM_CCR2_CFLRI2_Msk | PWM_CCR2_CRLRI2_Msk | PWM_CCR2_CAPCH2EN_Msk | 
					PWM_CCR2_CFL_IE2_Msk |PWM_CCR2_CRL_IE2_Msk;
	//PWMA->CNR2 = 10000;
	PWMA->CNR2 = 0xffff;
	PWMA->CMR2=0;
	//PWMA->POE &= 0x0b;  //DISABLE POE PWM2 PIN
	PWMA->PIER |= PWM_PIER_PWMIE2_Msk;
	
       NVIC_EnableIRQ((IRQn_Type)(PWMA_IRQn));
	while(PWMA->PDR2 ==0);
	PWMA->CAPENR = 4;
	
}


void open_timer0(void) //100ms interrupt once
{
	NVIC_EnableIRQ(TMR0_IRQn);
	TIMER0->TCMPR = 4800;
	TIMER0->TCSR = 0x680100F9;
	timer0_100ms_count=0;
}

void close_timer0(void)
{
	
	TIMER0->TCSR=5;
	TIMER0->TCMPR = 0;
	NVIC_DisableIRQ((IRQn_Type)(TMR0_IRQn)); 
}



/*
	use for copy capture data form register in capture interrupt funcation
	0: falling 1:resing
*/

void capture_interrupt_save_data(char egde_type)
{
	if(capture_start_enable && (capture_wave_count<CAPTURE_ALL_TIME))
	{
		if(capture_lose_flag)  //have lose first interrupt value
		{
			if(capture_first_edge==2)
			{	
				capture_first_edge=egde_type;
				capture_wave_count=0;
			}
			if(egde_type) //resing
			{
				//P36=0;
				capture_value_array[capture_wave_count]=PWMA->CRLR2;
			}
			else //falling
			{
				//P36=1;
				capture_value_array[capture_wave_count]=PWMA->CFLR2;
			}
			capture_wave_count++;
			if(capture_wave_count>=CAPTURE_ALL_TIME)
			{
				capture_start_enable=0;
			}
		}
		else
		{
			capture_lose_flag=1; // lose the first capture signal
		}
	}
}



void clean_capture_flag(void)
{
	capture_start_enable=0;
	capture_first_edge=2;
	capture_lose_flag=0;
	capture_wave_count=0;
}


void open_capture_flag(void)
{
	capture_start_enable=1;
	capture_first_edge=2;
	capture_lose_flag=0;
	capture_wave_count=0;
}

/*
	calculate capture wave frequency
	return 0: 0~100HZ pepoer action
		   -1: other 
*/
int calculate_capture_frequency(void)
{
	unsigned int  high_level_time0,low_level_time0, high_level_time1,low_level_time1;
	unsigned int  average_frequency;
	printf("capture_first_flag is %d\n",capture_first_edge);
	printf("cva0 is %d\n",capture_value_array[0]);
	printf("cva1 is %d\n",capture_value_array[1]);
	printf("cva2 is %d\n",capture_value_array[2]);
	printf("cva3 is %d\n",capture_value_array[3]);
	if(capture_first_edge) //rasing 0.2ms setp
	{
		low_level_time0=0xffff-capture_value_array[0];
		high_level_time0=0xffff-capture_value_array[1];
		low_level_time1=0xffff-capture_value_array[2];
		high_level_time1=0xffff-capture_value_array[3];
	}
	else //falling
	{
		high_level_time0=0xffff-capture_value_array[0];
		low_level_time0=0xffff-capture_value_array[1];
		high_level_time1=0xffff-capture_value_array[2];
		low_level_time1=0xffff-capture_value_array[3];
	}
	printf("high_level_time0 is %d\n",high_level_time0);
	printf("low_level_time0 is %d\n",low_level_time0);
	printf("high_level_time1 is %d\n",high_level_time1);
	printf("low_level_time1 is %d\n",low_level_time1);
	average_frequency= ((unsigned long)high_level_time0+low_level_time0+high_level_time1+low_level_time1)/2;  //0.2ms every cnt
	printf("average_frequency is %d\n",average_frequency);
	if((average_frequency<=10000) && (average_frequency>10)) //0~500Hz
	{	
		//P36=0;
		return 0;
	}
	
	return -1;
	//return 0;
}


void reset_wdt(void)
{
	SYS_UnlockReg();
	_WDT_RESET();
	SYS_LockReg();
}
/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main (void)
{   
	unsigned int i=0;
       SYS_Init();  
	   
	UART0_Init();
	printf("This is M051 UART0 Test\n");
	P20=0;P21=0;
	_GPIO_SET_PIN_MODE(P3,6,GPIO_PMD_OUTPUT);
	open_pwm3();
	open_capture(); //open capture funcation
	SYS_UnlockReg();
	WDT->WTCR =  WDT_WTCR_COMMON(5) | WDT_WTCR_WTRE_Msk;
	SYS_LockReg();	
	while(1)
	{
		if(LIGHT_RESISTANCES) //day
		{
			if(light_flag!=DAY) //night turn to day need close LED 
			{
				light_flag=DAY;
				reset_wdt();
				close_pwm(0);
				close_pwm(1);
				clean_capture_flag(); //day close capture
			}
		}
		else  //night
		{
			if(light_flag!=NIGHT) //day turn to night
			{
				light_flag=NIGHT;
				reset_wdt();
				open_pwm(0);  //open led0
				//open_pwm(1);
				if(!capture_start_enable)
					open_capture_flag();
			}
			else
			{
				if(capture_wave_count>=CAPTURE_ALL_TIME) //capture compelet once time
				{
					
					if(calculate_capture_frequency()==0) //have someone pass by, led1 light
					{
						open_pwm(1);
						open_timer0();
						open_capture_flag();
						while(timer0_100ms_count<50) // 3s
						{
							if(capture_wave_count>=CAPTURE_ALL_TIME) 
							{
								if(calculate_capture_frequency()==0) //have peope
								{
									timer0_100ms_count=0;
								}
								open_capture_flag();
							}
							if(LIGHT_RESISTANCES) //day
							{	
								open_capture_flag();
								break;
							}
							for(i=0;i<1000;i++);
							reset_wdt();
						}
						close_timer0();
						close_pwm(1);
					}
					else
						open_capture_flag();
				}
			}
			
		}
		//delay some time for check light resistance	
		for(i=0;i<1000;i++);
		reset_wdt();
	}
	//return 0;  
}






