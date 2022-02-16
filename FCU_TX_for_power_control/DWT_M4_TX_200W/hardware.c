
#include <stdio.h>

#include "hardware.h"
#include "WPCQiTx.h"
#include "Pwm.h"
#include "Coil.h"

#define PLL_CLOCK       192000000

extern uint8_t  dcdc_enable;

/****************************************************************************************************/
/*																									*/
/*			SYS_vInitSetting																				*/ 
/*																									*/
/*																									*/
/****************************************************************************************************/
void SYS_vInitSetting ( void )
{
	VBUS_SW_EN			= 0;
	Q_V_SUPPLY_EN		= 0;				
	BLE_EN					= 0;
	OVP_CTL 				= 1;
	Q_GATE_DR_SUPPLY_EN 		= 0; // Gate driver supply enable
	DCDC_EN 		= 0;
	dcdc_enable 	= 0;

 	printf("\nStart MPU\n");


	TIMER_EnableInt(TIMER1);

	EPWM_EnablePeriodInt(EPWM0, 4, 0); 
	NVIC_DisableIRQ(EPWM0P2_IRQn);	  /* Enable PWMA NVIC interrupt */

/**********************************************************************************/
/* KYJ TEST USING CH0 AND ZERO INTERRUPT, PWM WAVE WILL HAPPEN AS  L_H_L_HHH_L_H */
/**********************************************************************************/

#ifdef USE_ECAP
	NVIC_EnableIRQ(ECAP0_IRQn);
//#ifndef Q_MEASURE 		
//	ECAP0_vStart();
//#endif
#endif
#ifdef ECAP_Q_FACTOR
	NVIC_EnableIRQ(ECAP1_IRQn);
#endif
	NVIC_EnableIRQ(TMR1_IRQn);

	TIMER_Start(TIMER1);

}

/****************************************************************************************************/
/*																									*/
/*			CLK_vInit																				*/ 
/*																									*/
/*																									*/
/****************************************************************************************************/
void CLK_vInit(void)
{
	/* Unlock protected registers */
	SYS_UnlockReg();

	/* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
	PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

	/* Enable External XTAL (4~24 MHz) */
	CLK->PWRCTL |= CLK_PWRCTL_HXTEN_Msk; // XTAL12M (HXT) Enabled

	/* Waiting for 12MHz clock ready */
	CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

//	/* Switch HCLK clock source to XTAL */
//	CLK->CLKSEL0 &= ~CLK_CLKSEL0_HCLKSEL_Msk;
//	CLK->CLKSEL0 |= CLK_CLKSEL0_HCLKSEL_HXT;

	/* Set core clock as PLL_CLOCK from PLL */
	CLK_SetCoreClock(PLL_CLOCK);
	
	/* Set PCLK0 = PCLK1 = HCLK/1 */
	CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV1 | CLK_PCLKDIV_APB1DIV_DIV1);
	
	/* Enable IP module clock */
	CLK_EnableModuleClock(EPWM0_MODULE);
	
	/* EPWM clock frequency is set double to PCLK: select EPWM module clock source as PLL */
	CLK_SetModuleClock(EPWM0_MODULE, CLK_CLKSEL2_EPWM0SEL_PLL, (uint32_t)NULL);

	/* Enable IP module clock */
	CLK_EnableModuleClock(EPWM1_MODULE);
	
	/* EPWM clock frequency is set double to PCLK: select EPWM module clock source as PLL */
	CLK_SetModuleClock(EPWM1_MODULE, CLK_CLKSEL2_EPWM1SEL_PLL, (uint32_t)NULL);

#ifdef USE_ECAP
	/* Enable ECAP0 module clock */ 	 
	CLK_EnableModuleClock(ECAP0_MODULE);
#endif
#ifdef ECAP_Q_FACTOR
	/* Enable ECAP1 module clock */ 	 
	CLK_EnableModuleClock(ECAP1_MODULE);
#endif
	/* Enable TIMER0 module clock */    
	CLK_EnableModuleClock(TMR0_MODULE);

	/* Select Timer 0 clock source from HXT */
	CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HXT, 0);

	/* Enable TIMER1 module clock */    
	CLK_EnableModuleClock(TMR1_MODULE);

	/* Select Timer 1 clock source from HXT */
	CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1SEL_PCLK0, 0);


    /* Enable UART clock */
    //CLK_EnableModuleClock(UART0_MODULE | UART2_MODULE | UART4_MODULE);

//  CLK_EnableModuleClock(UART0_MODULE);
//  CLK_EnableModuleClock(UART4_MODULE);

	/* Enable IP clock */
	CLK->APBCLK0 |= (CLK->APBCLK0 & ~ CLK_APBCLK0_UART0CKEN_Msk ) | CLK_APBCLK0_UART0CKEN_Msk;   // UART0 Clock Enable
	CLK->APBCLK0 |= (CLK->APBCLK0 & ~ CLK_APBCLK0_UART4CKEN_Msk ) | CLK_APBCLK0_UART4CKEN_Msk;	 // UART4 Clock Enable

    /* Select UART clock source from HXT */
    //CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HXT, CLK_CLKDIV0_UART0(1));
  CLK->CLKSEL1 = (CLK->CLKSEL1 & ~CLK_CLKSEL1_UART0SEL_Msk) | (0x0 << CLK_CLKSEL1_UART0SEL_Pos);

    /* Select UART clock source from HXT */
    //CLK_SetModuleClock(UART4_MODULE, CLK_CLKSEL3_UART4SEL_HXT, CLK_CLKDIV4_UART4(1));
	CLK->CLKSEL3 = (CLK->CLKSEL3 & ~CLK_CLKSEL3_UART4SEL_Msk) | (0x0 << CLK_CLKSEL3_UART4SEL_Pos);

	/* Enable EADC module clock */		
	CLK_EnableModuleClock(EADC_MODULE); 	 
	
	/* EADC clock source is 96MHz, set divider to 8, EADC clock is 96/8 MHz */ 	 
	/* EADC clock source is 96MHz, set divider to 8, EADC clock is 192/8 MHz */ 	 
	CLK_SetModuleClock(EADC_MODULE, 0, CLK_CLKDIV0_EADC(1));

	/* Update System Core Clock */
	/* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
	SystemCoreClockUpdate();

	/* Lock protected registers */
	SYS_LockReg();

}

/****************************************************************************************************/
/*																									*/
/*			GPIO_vInit																				*/ 
/*																									*/
/*																									*/
/****************************************************************************************************/
void GPIO_vInit(void)
{

/*===========================  EPWM 0, 1 =====================================================*/
/*EPWM0 0,1,2,3,4,5 => PA5,PA4,PA3,PA2,PA1,PA0 */
	SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA5MFP_Msk | SYS_GPA_MFPL_PA4MFP_Msk | SYS_GPA_MFPL_PA3MFP_Msk | SYS_GPA_MFPL_PA2MFP_Msk | SYS_GPA_MFPL_PA1MFP_Msk | SYS_GPA_MFPL_PA0MFP_Msk);
	SYS->GPA_MFPL |= ( /*SYS_GPA_MFPL_PA0MFP_BPWM0_CH0 | SYS_GPA_MFPL_PA1MFP_BPWM0_CH1 |*/SYS_GPA_MFPL_PA1MFP_EPWM0_CH4 | SYS_GPA_MFPL_PA0MFP_EPWM0_CH5);
	SYS->GPA_MFPL |= ( SYS_GPA_MFPL_PA2MFP_EPWM0_CH3 | SYS_GPA_MFPL_PA3MFP_EPWM0_CH2 );
	GPIO_SetMode(PA, BIT4, GPIO_MODE_OUTPUT); 	 
	GPIO_SetMode(PA, BIT5, GPIO_MODE_OUTPUT); 	 

/*EPWM1 0,1,2,3,4,5 => PC5,PC4,PC3,PC2,PC1,PC0 */
	SYS->GPC_MFPL &= ~(SYS_GPC_MFPL_PC5MFP_Msk | SYS_GPC_MFPL_PC4MFP_Msk | SYS_GPC_MFPL_PC3MFP_Msk | SYS_GPC_MFPL_PC2MFP_Msk | SYS_GPC_MFPL_PC1MFP_Msk | SYS_GPC_MFPL_PC0MFP_Msk ); 	 
	SYS->GPC_MFPL |= ( /*SYS_GPC_MFPL_PC5MFP_EPWM1_CH0 | SYS_GPC_MFPL_PC4MFP_EPWM1_CH1 |*/ SYS_GPC_MFPL_PC1MFP_EPWM1_CH4| SYS_GPC_MFPL_PC0MFP_EPWM1_CH5);

	GPIO_SetMode(PC, BIT2, GPIO_MODE_OUTPUT); 	 
	GPIO_SetMode(PC, BIT3, GPIO_MODE_OUTPUT); 	 
	GPIO_SetMode(PC, BIT4, GPIO_MODE_OUTPUT); 	 
	GPIO_SetMode(PC, BIT5, GPIO_MODE_OUTPUT); 	

/*===========================  UART 0, 2, 4 =====================================================*/

	/* Set GPA PA6, PA7  multi-function pins for UART0 RXD and TXD */
	SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA6MFP_Msk | SYS_GPA_MFPL_PA7MFP_Msk);
	SYS->GPA_MFPL |= (SYS_GPA_MFPL_PA6MFP_UART0_RXD | SYS_GPA_MFPL_PA7MFP_UART0_TXD);

	/* Set GPA PF5, PF4  multi-function pins for UART2 RXD and TXD */
	//SYS->GPF_MFPL &= ~(SYS_GPF_MFPL_PF5MFP_Msk | SYS_GPF_MFPL_PF4MFP_Msk);
	//SYS->GPF_MFPL |= (SYS_GPF_MFPL_PF5MFP_UART2_RXD | SYS_GPF_MFPL_PF4MFP_UART2_TXD);

	/* Set GPA PA13, PA12  multi-function pins for UART4 RXD and TXD */
	SYS->GPA_MFPH &= ~(SYS_GPA_MFPH_PA13MFP_Msk | SYS_GPA_MFPH_PA12MFP_Msk);
	SYS->GPA_MFPH |= (SYS_GPA_MFPH_PA13MFP_UART4_RXD | SYS_GPA_MFPH_PA12MFP_UART4_TXD);

/*===========================  TMx_EXT  =====================================================*/
#ifndef USE_ECAP
  /* Set PB.15 for ECAP0_IC0*/    
	SYS->GPB_MFPH = (SYS->GPB_MFPH & ~SYS_GPB_MFPH_PB15MFP_Msk) |SYS_GPB_MFPH_PB15MFP_TM0_EXT;
	
  /* Set PB.14 for ECAP1_IC0*/    
	SYS->GPB_MFPH = (SYS->GPB_MFPH & ~SYS_GPB_MFPH_PB14MFP_Msk) |SYS_GPB_MFPH_PB14MFP_TM1_EXT;
#endif
/*============================	GPIO ==================================================== */


	/* Configure PB.3 as Output mode and PD.8 as Input mode then close it */    

//	GPIO_SetMode(PB, BIT4, GPIO_MODE_OUTPUT);    
	GPIO_SetMode(PF, BIT7, GPIO_MODE_OUTPUT);    
	GPIO_SetMode(PF, BIT8, GPIO_MODE_OUTPUT);    
	GPIO_SetMode(PF, BIT9, GPIO_MODE_OUTPUT);    
	PF7=PF8=PF9=0;

//	GPIO_SetMode(PB, BIT4, GPIO_MODE_OUTPUT);    
/*    LED  */
	GPIO_SetMode(PE, BIT3, GPIO_MODE_OUTPUT);    
	GPIO_SetMode(PE, BIT4, GPIO_MODE_OUTPUT);    
	GPIO_SetMode(PE, BIT5, GPIO_MODE_OUTPUT);    

	GPIO_SetMode(PE, BIT10, GPIO_MODE_OUTPUT);
//yy.lee : QF reset
	GPIO_SetMode(PF, BIT11, GPIO_MODE_OUTPUT);

//	PE10 = 1;

	GPIO_SetMode(PC, BIT6, GPIO_MODE_OUTPUT);    
	GPIO_SetMode(PC, BIT7, GPIO_MODE_OUTPUT);    


/* for debug  */
	GPIO_SetMode(PG, BIT11, GPIO_MODE_OUTPUT);    
	GPIO_SetMode(PG, BIT12, GPIO_MODE_OUTPUT);		
	GPIO_SetMode(PG, BIT13, GPIO_MODE_OUTPUT);		
	GPIO_SetMode(PG, BIT14, GPIO_MODE_OUTPUT);		
	GPIO_SetMode(PG, BIT15, GPIO_MODE_OUTPUT);		

	/* for Q control  Gate driver supply enable */
	GPIO_SetMode(PG, BIT4, GPIO_MODE_OUTPUT);		

	/* for FO sel  */
	GPIO_SetMode(PH, BIT8, GPIO_MODE_OUTPUT);    
	GPIO_SetMode(PH, BIT9, GPIO_MODE_OUTPUT);    
	GPIO_SetMode(PH, BIT10, GPIO_MODE_OUTPUT);    
	GPIO_SetMode(PH, BIT11, GPIO_MODE_OUTPUT);    

	GPIO_SetMode(PH, BIT4, GPIO_MODE_OUTPUT);
	PH4 = 0;


/*  OVP_CTL  */
	GPIO_SetMode(PB, BIT12, GPIO_MODE_OUTPUT);    
//  PB = 1; 

	/* Set PB.0 ~ PB.3 to input mode */
	PB->MODE &= ~(GPIO_MODE_MODE0_Msk | GPIO_MODE_MODE1_Msk | GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk |\
								GPIO_MODE_MODE4_Msk | GPIO_MODE_MODE5_Msk | GPIO_MODE_MODE6_Msk | GPIO_MODE_MODE7_Msk |\
								GPIO_MODE_MODE8_Msk | GPIO_MODE_MODE9_Msk | GPIO_MODE_MODE10_Msk | GPIO_MODE_MODE11_Msk );

	GPIO_SetMode(PB, BIT8, GPIO_MODE_INPUT);    
	GPIO_SetMode(PB, BIT9, GPIO_MODE_INPUT);    


	/* Configure the GPB0 - GPB11 ADC analog input pins.  */    
	SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB0MFP_Msk | SYS_GPB_MFPL_PB1MFP_Msk | \
											SYS_GPB_MFPL_PB2MFP_Msk | SYS_GPB_MFPL_PB3MFP_Msk | \
											SYS_GPB_MFPL_PB4MFP_Msk | SYS_GPB_MFPL_PB5MFP_Msk | \
											SYS_GPB_MFPL_PB6MFP_Msk | SYS_GPB_MFPL_PB7MFP_Msk | \
											SYS_GPB_MFPH_PB8MFP_Msk | SYS_GPB_MFPH_PB9MFP_Msk | \
											SYS_GPB_MFPH_PB10MFP_Msk | SYS_GPB_MFPH_PB11MFP_Msk );    
	
	SYS->GPB_MFPL |= (SYS_GPB_MFPL_PB0MFP_EADC0_CH0 | SYS_GPB_MFPL_PB1MFP_EADC0_CH1 | \
										SYS_GPB_MFPL_PB2MFP_EADC0_CH2 | SYS_GPB_MFPL_PB3MFP_EADC0_CH3 | \
										SYS_GPB_MFPL_PB4MFP_EADC0_CH4 | SYS_GPB_MFPL_PB5MFP_EADC0_CH5 | \
										SYS_GPB_MFPL_PB6MFP_EADC0_CH6 | SYS_GPB_MFPL_PB7MFP_EADC0_CH7 | \
										SYS_GPB_MFPH_PB8MFP_EADC0_CH8 | SYS_GPB_MFPH_PB9MFP_EADC0_CH9 | \
										SYS_GPB_MFPH_PB10MFP_EADC0_CH10 | SYS_GPB_MFPH_PB11MFP_EADC0_CH11 );
	
	/* Disable the GPB0 - GPB3 digital input path to avoid the leakage current. */    
	GPIO_DISABLE_DIGITAL_PATH(PB, BIT0|BIT1|BIT2|BIT3|BIT4|BIT5|BIT6|BIT7|\
																BIT8|BIT9|BIT10|BIT11	);

	/* Set reference voltage to external pin (3.3V) */		
	SYS_SetVRef(SYS_VREFCTL_VREF_PIN);

#ifdef USE_ECAP
/*============================	ECAP0_IC0 ==================================================== */
	/* Set PA.10	for ECAP0_IC0*/ 	
//	SYS->GPA_MFPH = (SYS->GPA_MFPH & ~SYS_GPA_MFPH_PA10MFP_Msk) |SYS_GPA_MFPH_PA10MFP_ECAP0_IC0;

//	/* Set	PE.8 for ECAP0_IC0*/	
	SYS->GPE_MFPH = (SYS->GPE_MFPH & ~SYS_GPE_MFPH_PE8MFP_Msk) | SYS_GPE_MFPH_PE8MFP_ECAP0_IC0;
#endif
//	GPIO_SetMode(PD, BIT8, GPIO_MODE_INPUT);    

#ifdef ECAP_Q_FACTOR
	/* Set PE.13	for ECAP1_IC0*/ 	
	SYS->GPE_MFPH = (SYS->GPE_MFPH & ~SYS_GPE_MFPH_PE13MFP_Msk) |SYS_GPE_MFPH_PE13MFP_ECAP1_IC0;
#endif



 
}


/****************************************************************************************************/
/*																									*/
/*						UART4_vInit																	*/ 
/*																									*/
/*																									*/
/****************************************************************************************************/
void UART4_vInit(void)
{
    /* Reset UART0 */    
	SYS_ResetModule(UART4_RST);

    UART_Open(UART4, 115200);

	/* Set RX Trigger Level = 8 */   
	UART4->FIFO = (UART4->FIFO &~ UART_FIFO_RFITL_Msk) | UART_FIFO_RFITL_8BYTES;
    /* Set Timeout time 0x3E bit-time */    
	UART_SetTimeoutCnt(UART4, 0x3E);    
	/* enable uart */    
	UART_EnableInt(UART4, UART_INTEN_RDAIEN_Msk /*| UART_INTEN_THREIEN_Msk |*/
									/*UART_INTEN_RXTOIEN_Msk | UART_INTEN_TOCNTEN_Msk*/);  
}

/****************************************************************************************************/
/*																									*/
/*						UART0_vInit																	*/ 
/*																									*/
/*																									*/
/****************************************************************************************************/
void UART0_vInit(void)
{
    /* Reset UART0 */    
	SYS_ResetModule(UART0_RST);

	UART_Open(UART0, 115200);
    /* Set RX Trigger Level = 8 */
	UART0->FIFO = (UART0->FIFO &~ UART_FIFO_RFITL_Msk) | UART_FIFO_RFITL_8BYTES;		
	UART0->FIFO |= UART_FIFO_RTSTRGLV_8BYTES; 	 

	/* Set Timeout time 0x3E bit-time */ 	 
	UART_SetTimeoutCnt(UART0, 0x3E); 	 

	/* enable uart */		
	UART_EnableInt(UART0, UART_INTEN_RDAIEN_Msk /*| UART_INTEN_THREIEN_Msk*/ |
									UART_INTEN_RXTOIEN_Msk | UART_INTEN_TOCNTEN_Msk);		

}


/************************************************************************************************************
*    TIMER 0 interrupt subroutine
************************************************************************************************************/
/****************************************************************************************************/
/*																									*/
/*						PWM_vInit																	*/ 
/*																									*/
/*																									*/
/*																									*/
/*  EPWM0 channel 0 waveform of this sample shown below:													*/
/*																									*/
/*   |<-		 CNR + 1  clk	          ->|  CNR + 1 = 399 + 1 CLKs												*/
/*				    |<-CMR+1 clk ->|  CMR + 1 = 199 + 1 CLKs											*/
/*								|<-   CNR + 1  ->|	CNR + 1 = 99 + 1 CLKs								*/
/*										  |<CMR+1>|	CMR + 1 = 39 + 1 CLKs								*/
/* __			    ______________		  _______													*/
/*   |______200_____|	   200		|____60__|	 40  |_____EPWM waveform									*/
/*																									*/ 
/*Configure EPWM0 channel 0 init period and duty. 															*/
/*Period is PLL / (prescaler * (CNR + 1))																	*/
/*Duty ratio = (CMR + 1) / (CNR + 1)																		*/
/*Period = 192 MHz / (4 * (199 + 1)) = 240000 Hz															*/
/*Duty ratio = (99 + 1) / (199 + 1) = 50%																	*/
/*																									*/
/*																									*/
/****************************************************************************************************/
// PWM WING TEST USING AND GATE CH4 WITH CH0 
void PWM_vInit(void)
{			   
//		EPWM_ConfigOutputChannel(EPWM0, 0, 240000, 50);  
//		EPWM_ConfigOutputChannel(EPWM1, 0, 240000, 50);
		EPWM_ConfigOutputChannel(EPWM0, 2, 240000, 50);  //Voltage control
		EPWM_ConfigOutputChannel(EPWM0, 3, 240000, 50);  //Voltage control
    	EPWM_ConfigOutputChannel(EPWM0, 4, 240000, 50);
   		EPWM_ConfigOutputChannel(EPWM1, 4, 240000, 50);
        /* Init configuaration set for EPWM0 CH4 & CH 5 & EPWM1 CH4 & CH 5 */
		EPWM_SET_CNR(EPWM0, 4, TIMERA_TICKS_110KHZ);	// priod
		EPWM_SET_CMR(EPWM0, 4, TIMERA_TICKS_110KHZ- TIMERA_TICKS_110KHZ/2); // cmpdata n :  0 

		EPWM_SET_CNR(EPWM0, 5, TIMERA_TICKS_110KHZ);
		EPWM_SET_CMR(EPWM0, 5, TIMERA_TICKS_110KHZ - TIMERA_TICKS_110KHZ/2);  // cmpdata n : 1

		EPWM_SET_CNR(EPWM1, 4, TIMERA_TICKS_110KHZ);	// priod
		EPWM_SET_CMR(EPWM1, 4, TIMERA_TICKS_110KHZ - TIMERA_TICKS_110KHZ/2); // cmpdata n :  0 

		EPWM_SET_CNR(EPWM1, 5, TIMERA_TICKS_110KHZ);
		EPWM_SET_CMR(EPWM1, 5, TIMERA_TICKS_110KHZ - TIMERA_TICKS_110KHZ/2);  // cmpdata n : 1
		
		EPWM_SET_ALIGNED_TYPE(EPWM0, EPWM_CH_4_MASK | EPWM_CH_5_MASK, EPWM_CENTER_ALIGNED);
		EPWM_SET_ALIGNED_TYPE(EPWM1, EPWM_CH_4_MASK | EPWM_CH_5_MASK, EPWM_CENTER_ALIGNED);
        /* KYJ : 										  zero point, 		   compare up point, 	period(center) point,       compare down point*/
		EPWM_SET_OUTPUT_LEVEL(EPWM0, EPWM_CH_4_MASK, EPWM_OUTPUT_LOW, EPWM_OUTPUT_HIGH, EPWM_OUTPUT_NOTHING, EPWM_OUTPUT_NOTHING);  // n    여기는 도닌거.
		EPWM_SET_OUTPUT_LEVEL(EPWM0, EPWM_CH_5_MASK, EPWM_OUTPUT_LOW, EPWM_OUTPUT_NOTHING, EPWM_OUTPUT_NOTHING, EPWM_OUTPUT_LOW); //m

		EPWM_SET_OUTPUT_LEVEL(EPWM1, EPWM_CH_4_MASK, EPWM_OUTPUT_LOW, EPWM_OUTPUT_HIGH, EPWM_OUTPUT_NOTHING, EPWM_OUTPUT_NOTHING);  // n    여기는 도닌거.
		EPWM_SET_OUTPUT_LEVEL(EPWM1, EPWM_CH_5_MASK, EPWM_OUTPUT_LOW, EPWM_OUTPUT_NOTHING, EPWM_OUTPUT_NOTHING, EPWM_OUTPUT_LOW); //m

        /* Set EPWM mode to complementary */
		EPWM_ENABLE_COMPLEMENTARY_MODE(EPWM0);
		EPWM_ENABLE_COMPLEMENTARY_MODE(EPWM1);

		/* Set sync */
		EPWM_ENABLE_TIMER_SYNC(EPWM0, /*EPWM_CH_0_MASK | EPWM_CH_1_MASK |*/ EPWM_CH_4_MASK | EPWM_CH_5_MASK, EPWM_SSCTL_SSRC_EPWM0);
		EPWM_ENABLE_TIMER_SYNC(EPWM1, /*EPWM_CH_0_MASK | EPWM_CH_1_MASK |*/ EPWM_CH_4_MASK | EPWM_CH_5_MASK, EPWM_SSCTL_SSRC_EPWM1);

        /* Set EPWM1 out put to invert mode */
		EPWM_ENABLE_OUTPUT_INVERTER(EPWM1,/*EPWM_CH_0_MASK | EPWM_CH_1_MASK |*/ EPWM_CH_4_MASK | EPWM_CH_5_MASK);


        /* Init configuaration set for EPWM0 CH 2& CH3 using voltage control */
		EPWM_SET_CNR(EPWM0, 2, TIMERA_TICKS_120KHZ);	// priod
		EPWM_SET_CMR(EPWM0, 2, TIMERA_TICKS_120KHZ/2); // cmpdata n :  0 
		
		EPWM_SET_CNR(EPWM0, 3, TIMERA_TICKS_120KHZ);	// priod
		EPWM_SET_CMR(EPWM0, 3, TIMERA_TICKS_120KHZ/2); // cmpdata n :  0 

		EPWM_SET_ALIGNED_TYPE(EPWM0, EPWM_CH_2_MASK , EPWM_CENTER_ALIGNED);
		EPWM_SET_ALIGNED_TYPE(EPWM0, EPWM_CH_3_MASK , EPWM_CENTER_ALIGNED);

		EPWM_SET_OUTPUT_LEVEL(EPWM0, EPWM_CH_2_MASK, EPWM_OUTPUT_LOW, EPWM_OUTPUT_HIGH, EPWM_OUTPUT_NOTHING, EPWM_OUTPUT_LOW);  // n    여기는 도닌거.
		EPWM_SET_OUTPUT_LEVEL(EPWM0, EPWM_CH_3_MASK, EPWM_OUTPUT_LOW, EPWM_OUTPUT_HIGH, EPWM_OUTPUT_NOTHING, EPWM_OUTPUT_LOW); //m

		SYS_UnlockReg();			
//		EPWM_EnableDeadZone(EPWM0, 0, 0xf);		
		EPWM_EnableDeadZone(EPWM0, 4, 0x9);		
		EPWM_EnableDeadZone(EPWM1, 4, 0x9);		
		SYS_LockReg();


		EPWM_EnableOutput(EPWM0, /*EPWM_CH_0_MASK | EPWM_CH_1_MASK |*/ EPWM_CH_3_MASK | EPWM_CH_4_MASK | EPWM_CH_5_MASK );
        EPWM_EnableOutput(EPWM1, /*EPWM_CH_0_MASK | EPWM_CH_1_MASK |*/ EPWM_CH_4_MASK | EPWM_CH_5_MASK );

		PA3=PA4=PA5=1; 	 
		PC2=PC3=PC4=PC5=1; 	 

		
		EPWM_Start(EPWM0, /*EPWM_CH_0_MASK | EPWM_CH_1_MASK |*/ EPWM_CH_2_MASK | EPWM_CH_3_MASK | EPWM_CH_4_MASK | EPWM_CH_5_MASK );
		EPWM_Start(EPWM1, /*EPWM_CH_0_MASK | EPWM_CH_1_MASK |*/ EPWM_CH_4_MASK | EPWM_CH_5_MASK );

}


#ifdef ECAP_Q_FACTOR
/****************************************************************************************************/
/*																									*/
/*						ECAP1_vInit																	*/ 
/*																									*/
/*																									*/
/****************************************************************************************************/
void ECAP1_vInit(void)
{

  /* Compare value set */
	ECAP_SET_CNT_CMP(ECAP1, ECAP1_TIMEOUT); // CNTCMP  120000 /48M = 2.5ms

	/* Enable ECAP0*/
	ECAP_Open(ECAP1, ECAP_COMPARE_FUNCTION /*ECAP_DISABLE_COMPARE*/);		// CTL0

  /* noise filter  */
	ECAP_SET_NOISE_FILTER_CLKDIV(ECAP1, ECAP_NOISE_FILTER_CLKDIV_2);  // ((ecap)->CTL0 = ((ecap)->CTL0 & ~ECAP_CTL0_NFCLKSEL_Msk)|(u32ClkSel))

	/* Enable ECAP0 Input Channel 0  PA.10 or PE.8 */		
	ECAP_ENABLE_INPUT_CHANNEL(ECAP1, ECAP_CTL0_IC0EN_Msk);

	/* Enable ECAP0 source from IC0 */		
	ECAP_SEL_INPUT_SRC(ECAP1, ECAP_IC0, ECAP_CAP_INPUT_SRC_FROM_IC);

	ECAP1->CTL0 |= ECAP_CTL0_CAPIEN0_Msk;

	ECAP_ENABLE_CMP_MATCH_INT(ECAP1);	// ((ecap)->CTL0 |= ECAP_CTL0_CMPIEN_Msk)

	ECAP1->CTL0 |= ECAP_CTL0_CMPCLREN_Msk;

	ECAP_ENABLE_CMP(ECAP1);	// ((ecap)->CTL0 |= ECAP_CTL0_CMPEN_Msk)
	
	ECAP_ENABLE_CNT(ECAP1); // ((ecap)->CTL0 |= ECAP_CTL0_CAPEN_Msk)

	/* Select IC0 detect rising edge */ 	 
	ECAP_SEL_CAPTURE_EDGE(ECAP1, ECAP_IC0, ECAP_RISING_EDGE);	// CTL1	

  /* PCLK: 192 MHz / 4   :  48 MHz */
  /* ECAP_SEL_TIMER_CLK_DIV(ECAP1, ECAP_CAPTURE_TIMER_CLKDIV_4); */
  /* PCLK: 192 MHz / 1   :  192 MHz */
	ECAP_SEL_TIMER_CLK_DIV(ECAP1, ECAP_CAPTURE_TIMER_CLKDIV_1);
	
	/* Select Reload function */		
	ECAP_SET_CNT_CLEAR_EVENT(ECAP1, (ECAP_CTL1_CAP0RLDEN_Msk /*|ECAP_CTL1_CAP1RLDEN_Msk*/));

	ECAP1->CTL0 |= ECAP_CTL0_CMPCLREN_Msk;


	/* Init & clear ECAP interrupt status flags */		
	ECAP_CLR_CAPTURE_FLAG(ECAP1, ECAP_STATUS_CAPTF0_Msk | ECAP_STATUS_CAPCMPF_Msk | ECAP_STATUS_CAPOVF_Msk ); /* ((ecap)->STATUS = (u32Mask))*/


}

/****************************************************************************************************/
/*																									*/
/*						ECAP1_vInit																	*/ 
/*																									*/
/*																									*/
/****************************************************************************************************/
void ECAP1_vStart(void)
{
	/* ECAP_CNT starts up-counting */ 	
//	ECAP_DISABLE_CMP_MATCH_INT
	ECAP_CNT_START(ECAP1);  /* ((ecap)->CTL0 |= ECAP_CTL0_CNTEN_Msk)*/
	NVIC_EnableIRQ(ECAP1_IRQn);
}

/****************************************************************************************************/
/*																									*/
/*						ECAP1_vInit																	*/ 
/*																									*/
/*																									*/
/****************************************************************************************************/
void ECAP1_vStop(void)
{
	/* ECAP_CNT stop up-counting */ 	
	ECAP_CNT_STOP(ECAP1);  /* ((ecap)->CTL0 &= ~ECAP_CTL0_CNTEN_Msk)*/
	NVIC_DisableIRQ(ECAP1_IRQn);
}


#endif

#ifdef USE_ECAP
/****************************************************************************************************/
/*																									*/
/*						ECAP0_vInit																	*/ 
/*																									*/
/*																									*/
/****************************************************************************************************/
void ECAP0_vInit(void)
{

    /* Compare value set */
	ECAP_SET_CNT_CMP(ECAP0, ECAP0_TIMEOUT); // CNTCMP  120000 /48M = 2.5ms

	/* Enable ECAP0*/
	ECAP_Open(ECAP0, ECAP_COMPARE_FUNCTION /*ECAP_DISABLE_COMPARE*/);		// CTL0

    /* noise filter  */
	ECAP_SET_NOISE_FILTER_CLKDIV(ECAP0, ECAP_NOISE_FILTER_CLKDIV_2);  // ((ecap)->CTL0 = ((ecap)->CTL0 & ~ECAP_CTL0_NFCLKSEL_Msk)|(u32ClkSel))

	/* Enable ECAP0 Input Channel 0  PA.10 or PE.8 */		
	ECAP_ENABLE_INPUT_CHANNEL(ECAP0, ECAP_CTL0_IC0EN_Msk);

	/* Enable ECAP0 source from IC0 */		
	ECAP_SEL_INPUT_SRC(ECAP0, ECAP_IC0, ECAP_CAP_INPUT_SRC_FROM_IC);

	ECAP0->CTL0 |= ECAP_CTL0_CAPIEN0_Msk;

	ECAP_ENABLE_CMP_MATCH_INT(ECAP0);	// ((ecap)->CTL0 |= ECAP_CTL0_CMPIEN_Msk)

	ECAP0->CTL0 |= ECAP_CTL0_CMPCLREN_Msk;

	ECAP_ENABLE_CMP(ECAP0);	// ((ecap)->CTL0 |= ECAP_CTL0_CMPEN_Msk)
	
	ECAP_ENABLE_CNT(ECAP0); // ((ecap)->CTL0 |= ECAP_CTL0_CAPEN_Msk)

	/* Select IC0 detect rising edge */ 	 
	ECAP_SEL_CAPTURE_EDGE(ECAP0, ECAP_IC0, ECAP_RISING_FALLING_EDGE);	// CTL1	

    /* PCLK: 192 MHz / 4   :  48 MHz */
	ECAP_SEL_TIMER_CLK_DIV(ECAP0, ECAP_CAPTURE_TIMER_CLKDIV_4);
	
	/* Select Reload function */		
	ECAP_SET_CNT_CLEAR_EVENT(ECAP0, (ECAP_CTL1_CAP0RLDEN_Msk /*|ECAP_CTL1_CAP1RLDEN_Msk*/));

	ECAP0->CTL0 |= ECAP_CTL0_CMPCLREN_Msk;


	/* Init & clear ECAP interrupt status flags */		
	ECAP_CLR_CAPTURE_FLAG(ECAP0, ECAP_STATUS_CAPTF0_Msk | ECAP_STATUS_CAPCMPF_Msk | ECAP_STATUS_CAPOVF_Msk ); // ((ecap)->STATUS = (u32Mask))


}

/****************************************************************************************************/
/*																									*/
/*						ECAP0_vStart																	*/ 
/*																									*/
/*																									*/
/****************************************************************************************************/
void ECAP0_vStart(void)
{
	/* ECAP_CNT starts up-counting */ 	
	ECAP_CNT_START(ECAP0);  // ((ecap)->CTL0 |= ECAP_CTL0_CNTEN_Msk)
}

/****************************************************************************************************/
/*																									*/
/*						ECAP0_vStop																	*/ 
/*																									*/
/*																									*/
/****************************************************************************************************/
void ECAP0_vStop(void)
{
	/* ECAP_CNT stop up-counting */ 	
	ECAP_CNT_STOP(ECAP0);  // ((ecap)->CTL0 &= ~ECAP_CTL0_CNTEN_Msk)
}

#endif

/****************************************************************************************************/
/*																									*/
/*						TMR1_vInit																	*/ 
/*																									*/
/*																									*/
/****************************************************************************************************/
void TMR1_vInit(void)
{
	/* Enable Timer0 event counter input and external capture function */ 	 
	TIMER_Open(TIMER1, TIMER_PERIODIC_MODE , 400);	// 400Hz : 2.5ms
	
	TIMER_SET_PRESCALE_VALUE(TIMER1, 31);		//divided by ( PSC + 1 )  192MHz /4  48MHz / 8  : 6MHz
	
	TIMER_SET_CMP_VALUE(TIMER1, 6000);		//  1000Hz, 1ms
	
	/* Timer time-out interrupt enable or disabel */
	TIMER_EnableInt(TIMER1); 	 	
}



/****************************************************************************************************/
/*																									*/
/*						ADC_vInit																	*/ 
/*																									*/
/*																									*/
/****************************************************************************************************/
void ADC_vInit(void)
{

	/* Set input mode as single-end and enable the A/D converter */ 					 
	EADC_Open(EADC, EADC_CTL_DIFFEN_SINGLE_END); 		

  /* Disable the GPB0 digital input path to avoid the leakage current. */
  GPIO_DISABLE_DIGITAL_PATH(PB, BIT0);
	/* Configure the sample 0 module for analog input channel 0 and enable ADINT0 trigger source */						
	EADC_ConfigSampleModule(EADC, 0, EADC_ADINT0_TRIGGER, 0); 					 
	
  /* Disable the GPB1 digital input path to avoid the leakage current. */
  GPIO_DISABLE_DIGITAL_PATH(PB, BIT1);
	/* Configure the sample 1 module for analog input channel 1 and enable ADINT0 trigger source */						
	EADC_ConfigSampleModule(EADC, 1, EADC_ADINT0_TRIGGER, 1); 					 
	
  /* Disable the GPB2 digital input path to avoid the leakage current. */
  GPIO_DISABLE_DIGITAL_PATH(PB, BIT2);
	/* Configure the sample 2 module for analog input channel 2 and enable ADINT0 trigger source */						
	EADC_ConfigSampleModule(EADC, 2, EADC_ADINT0_TRIGGER, 2); 					 
	
  /* Disable the GPB3 digital input path to avoid the leakage current. */
  GPIO_DISABLE_DIGITAL_PATH(PB, BIT3);
	/* Configure the sample 3 module for analog input channel 3 and enable ADINT0 trigger source */						
	EADC_ConfigSampleModule(EADC, 3, EADC_ADINT0_TRIGGER, 3);
	
  /* Disable the GPB4 digital input path to avoid the leakage current. */
  GPIO_DISABLE_DIGITAL_PATH(PB, BIT4);
	/* Configure the sample 4 module for analog input channel 3 and enable ADINT0 trigger source */						
	EADC_ConfigSampleModule(EADC, 4, EADC_ADINT0_TRIGGER, 4);
	
  /* Disable the GPB5 digital input path to avoid the leakage current. */
  GPIO_DISABLE_DIGITAL_PATH(PB, BIT5);
	/* Configure the sample 5 module for analog input channel 3 and enable ADINT0 trigger source */						
	EADC_ConfigSampleModule(EADC, 5, EADC_ADINT0_TRIGGER, 5);
	
  /* Disable the GPB6 digital input path to avoid the leakage current. */
  GPIO_DISABLE_DIGITAL_PATH(PB, BIT6);
	/* Configure the sample 6 module for analog input channel 3 and enable ADINT0 trigger source */						
	EADC_ConfigSampleModule(EADC, 6, EADC_ADINT0_TRIGGER, 6);
	
  /* Disable the GPB7 digital input path to avoid the leakage current. */
  GPIO_DISABLE_DIGITAL_PATH(PB, BIT7);
	/* Configure the sample 7 module for analog input channel 3 and enable ADINT0 trigger source */						
	EADC_ConfigSampleModule(EADC, 7, EADC_ADINT0_TRIGGER, 7);
	
  /* Disable the GPB8 digital input path to avoid the leakage current. */
  GPIO_DISABLE_DIGITAL_PATH(PB, BIT8);
	/* Configure the sample 8 module for analog input channel 3 and enable ADINT0 trigger source */						
	EADC_ConfigSampleModule(EADC, 8, EADC_ADINT0_TRIGGER, 8);
	
  /* Disable the GPB9 digital input path to avoid the leakage current. */
  GPIO_DISABLE_DIGITAL_PATH(PB, BIT9);
	/* Configure the sample 9 module for analog input channel 3 and enable ADINT0 trigger source */						
	EADC_ConfigSampleModule(EADC, 9, EADC_ADINT0_TRIGGER, 9);
	
  /* Disable the GPB10 digital input path to avoid the leakage current. */
  GPIO_DISABLE_DIGITAL_PATH(PB, BIT10);
	/* Configure the sample 10 module for analog input channel 3 and enable ADINT0 trigger source */						
	EADC_ConfigSampleModule(EADC, 10, EADC_ADINT0_TRIGGER, 10);
	
  /* Disable the GPB11 digital input path to avoid the leakage current. */
  GPIO_DISABLE_DIGITAL_PATH(PB, BIT11);
	/* Configure the sample 11 module for analog input channel 3 and enable ADINT0 trigger source */						
	EADC_ConfigSampleModule(EADC, 11, EADC_ADINT0_TRIGGER, 11);


  /* Clear the A/D ADINT0 interrupt flag for safe */            
	EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF0_Msk);
	
	SYS_SetVRef(SYS_VREFCTL_VREF_3_0V);
}





/****************************************************************************************************/
/*																									*/
/*						SYS_vInit																	*/ 
/*																									*/
/*																									*/
/****************************************************************************************************/
void SYS_vInit(void)
{
	CLK_vInit();
	GPIO_vInit();
	PWM_vInit();
#ifdef USE_ECAP
	ECAP0_vInit();
#else
//	TMR0_CAP_init(); // Timer0
#endif
#ifdef ECAP_Q_FACTOR
	ECAP1_vInit();
#endif

//	TMR0_vInit();
	TMR1_vInit();
	ADC_vInit();
	UART0_vInit();
	UART4_vInit();
	SYS_vInitSetting();
}



