#include "application.h"
#include "Pwm.h"
#include "Pid.h"

uint8_t  dcdc_enable = 0;



#ifdef VOLTAGE_CONTROL
/****************************************************************************************************/
/*																									*/
/*						PWM_vStartBuckbooster															       */ 
/*																									*/
/*																									*/
/****************************************************************************************************/
void PWM_vInitBuckbooster( uint16_t period_ticks, uint16_t duty_ticks_high_side)
{
  PWM_vUpdateBuckbooster(period_ticks, duty_ticks_high_side);
}

/****************************************************************************************************/
/*																									*/
/*						PWM_vStartBuckbooster															       */ 
/*																									*/
/*																									*/
/****************************************************************************************************/
void PWM_vStartBuckbooster(void)
{
	SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA3MFP_Msk | SYS_GPA_MFPL_PA2MFP_Msk );
	SYS->GPA_MFPL |= ( SYS_GPA_MFPL_PA2MFP_EPWM0_CH3 | SYS_GPA_MFPL_PA3MFP_EPWM0_CH2 );

	EPWM_Start(EPWM0, EPWM_CH_2_MASK | EPWM_CH_3_MASK );
	EPWM_EnableOutput(EPWM0, EPWM_CH_3_MASK );
	DCDC_EN = 1;
	dcdc_enable = 1;


}
/****************************************************************************************************/
/*																									*/
/*						PWM_vStopBuckbooster															       */ 
/*																									*/
/*																									*/
/****************************************************************************************************/

void PWM_vStopBuckbooster(void)
{
	DCDC_EN = 0;
	dcdc_enable = 0;

	/* Disable PWM Output path for PWMA channel 1 */
	EPWM_DisableOutput(EPWM0, EPWM_CH_2_MASK | EPWM_CH_3_MASK );
	EPWM_Stop(EPWM0, EPWM_CH_2_MASK | EPWM_CH_3_MASK );
    GPIO_SetMode(PA, BIT2, GPIO_MODE_OUTPUT);	
    GPIO_SetMode(PA, BIT3, GPIO_MODE_OUTPUT);	
	PA2 = PA3 = 1; //KYJ: Changed the  DCDC ENABLE PIN value to high from low

}
/****************************************************************************************************/
/*																									*/
/*						PWM_vUpdateBuckbooster															       */ 
/*																									*/
/*																									*/
/****************************************************************************************************/
void PWM_vUpdateBuckbooster(uint16_t period_ticks, uint16_t duty_ticks_high_side) // ex 297, 149
{	
	EPWM_SET_CNR(EPWM0, 2, period_ticks);	// priod
	EPWM_SET_CMR(EPWM0, 2, duty_ticks_high_side); // cmpdata n :	0 

	EPWM_SET_CNR(EPWM0, 3, period_ticks);	// priod
	EPWM_SET_CMR(EPWM0, 3, duty_ticks_high_side); // cmpdata n :	0 
}
#endif

/****************************************************************************************************/
/*																									*/
/*		PWM_vInitCoil																							*/ 
/*																									*/
/*																									*/
/****************************************************************************************************/
void  PWM_vInitCoil( uint16_t period_ticks, uint16_t duty_ticks_high_side) /* set duty 50% */
{
    WPCQi_setPreFrequencyHandleCEP(FREQ_KHZ_PING_FIX16);
	PWM_vUpdateCoil(period_ticks, duty_ticks_high_side);
}
/****************************************************************************************************/
/*																									*/
/*		PWM_vStartCoil																							*/ 
/*																									*/
/*																									*/
/****************************************************************************************************/
void PWM_vStartCoil(void)
{
/*EPWM0 0,1,2,3,4,5 => PA5,PA4,PA3,PA2,PA1,PA0 */
	SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA5MFP_Msk | SYS_GPA_MFPL_PA4MFP_Msk | SYS_GPA_MFPL_PA3MFP_Msk | SYS_GPA_MFPL_PA2MFP_Msk | SYS_GPA_MFPL_PA1MFP_Msk | SYS_GPA_MFPL_PA0MFP_Msk);
	SYS->GPA_MFPL |= ( /*SYS_GPA_MFPL_PA5MFP_EPWM0_CH0 | SYS_GPA_MFPL_PA4MFP_EPWM0_CH1 |*/ SYS_GPA_MFPL_PA1MFP_EPWM0_CH4 | SYS_GPA_MFPL_PA0MFP_EPWM0_CH5);
#ifndef VOLTAGE_CONTROL
	GPIO_SetMode(PA, BIT2, GPIO_MODE_OUTPUT);
	GPIO_SetMode(PA, BIT3, GPIO_MODE_OUTPUT); 	 
#else
	SYS->GPA_MFPL |= ( SYS_GPA_MFPL_PA2MFP_EPWM0_CH3 | SYS_GPA_MFPL_PA3MFP_EPWM0_CH2 );
#endif
	GPIO_SetMode(PA, BIT4, GPIO_MODE_OUTPUT);	 
	GPIO_SetMode(PA, BIT5, GPIO_MODE_OUTPUT);	 

/*EPWM1 0,1,2,3,4,5 => PC5,PC4,PC3,PC2,PC1,PC0 */
	SYS->GPC_MFPL &= ~(SYS_GPC_MFPL_PC5MFP_Msk | SYS_GPC_MFPL_PC4MFP_Msk | SYS_GPC_MFPL_PC3MFP_Msk | SYS_GPC_MFPL_PC2MFP_Msk | SYS_GPC_MFPL_PC1MFP_Msk | SYS_GPC_MFPL_PC0MFP_Msk ); 	 
	SYS->GPC_MFPL |= ( /*SYS_GPC_MFPL_PC5MFP_EPWM1_CH0 | SYS_GPC_MFPL_PC4MFP_EPWM1_CH1 |*/ SYS_GPC_MFPL_PC1MFP_EPWM1_CH4| SYS_GPC_MFPL_PC0MFP_EPWM1_CH5);

	GPIO_SetMode(PC, BIT2, GPIO_MODE_OUTPUT); 	 
	GPIO_SetMode(PC, BIT3, GPIO_MODE_OUTPUT); 	 
	GPIO_SetMode(PC, BIT4, GPIO_MODE_OUTPUT); 	 
	GPIO_SetMode(PC, BIT5, GPIO_MODE_OUTPUT); 	

	EPWM_Start(EPWM0, EPWM_CH_0_MASK | EPWM_CH_1_MASK | EPWM_CH_4_MASK | EPWM_CH_5_MASK );
	EPWM_Start(EPWM1, EPWM_CH_0_MASK | EPWM_CH_1_MASK | EPWM_CH_4_MASK | EPWM_CH_5_MASK );
	Q_GATE_DR_SUPPLY_EN =0;
}

/****************************************************************************************************/
/*																									*/
/*			PWM_vStopCoil																						*/ 
/*																									*/
/*																									*/
/****************************************************************************************************/
void PWM_vStopCoil(void)
{
	EPWM_Stop(EPWM0, EPWM_CH_0_MASK | EPWM_CH_1_MASK |EPWM_CH_4_MASK | EPWM_CH_5_MASK );
	EPWM_Stop(EPWM1, EPWM_CH_0_MASK | EPWM_CH_1_MASK |EPWM_CH_4_MASK | EPWM_CH_5_MASK );

/*EPWM0 0,1,2,3,4,5 => PA5,PA4,PA3,PA2,PA1,PA0 */
	SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA5MFP_Msk | SYS_GPA_MFPL_PA4MFP_Msk | SYS_GPA_MFPL_PA3MFP_Msk | SYS_GPA_MFPL_PA2MFP_Msk | SYS_GPA_MFPL_PA1MFP_Msk | SYS_GPA_MFPL_PA0MFP_Msk);
#ifndef VOLTAGE_CONTROL
	GPIO_SetMode(PA, BIT2, GPIO_MODE_OUTPUT);
	GPIO_SetMode(PA, BIT3, GPIO_MODE_OUTPUT); 	 
#else
//	SYS->GPA_MFPL |= ( SYS_GPA_MFPL_PA2MFP_EPWM0_CH3 | SYS_GPA_MFPL_PA3MFP_EPWM0_CH2 );
	GPIO_SetMode(PA, BIT2, GPIO_MODE_OUTPUT);
	GPIO_SetMode(PA, BIT3, GPIO_MODE_OUTPUT); 	 
#endif
	GPIO_SetMode(PA, BIT0, GPIO_MODE_OUTPUT); // EPWM0 5  PA0	 
	GPIO_SetMode(PA, BIT1, GPIO_MODE_OUTPUT); // EPWM0 4  PA1	 

	GPIO_SetMode(PA, BIT4, GPIO_MODE_OUTPUT); 	 
	GPIO_SetMode(PA, BIT5, GPIO_MODE_OUTPUT); 

/*EPWM1 0,1,2,3,4,5 => PC5,PC4,PC3,PC2,PC1,PC0 */
	SYS->GPC_MFPL &= ~(SYS_GPC_MFPL_PC5MFP_Msk | SYS_GPC_MFPL_PC4MFP_Msk | SYS_GPC_MFPL_PC3MFP_Msk | SYS_GPC_MFPL_PC2MFP_Msk | SYS_GPC_MFPL_PC1MFP_Msk | SYS_GPC_MFPL_PC0MFP_Msk ); 	 

	GPIO_SetMode(PC, BIT0, GPIO_MODE_OUTPUT); // EPWM1 5  PC0
	GPIO_SetMode(PC, BIT1, GPIO_MODE_OUTPUT); // EPWM1 4  PC1	 

	GPIO_SetMode(PC, BIT2, GPIO_MODE_OUTPUT); 	 
	GPIO_SetMode(PC, BIT3, GPIO_MODE_OUTPUT); 	 
	GPIO_SetMode(PC, BIT4, GPIO_MODE_OUTPUT); 	 
	GPIO_SetMode(PC, BIT5, GPIO_MODE_OUTPUT); 	
	Q_GATE_DR_SUPPLY_EN = 1;


}

/****************************************************************************************************/
/*																									*/
/*			PWM_vUpdateCoil																						*/ 
/*																									*/
/*																									*/
/****************************************************************************************************/

void PWM_vUpdateCoil(uint16_t period_ticks, uint16_t duty_ticks_high_side) // ex 297, 149
{

	EPWM_SET_CNR(EPWM0, 4, period_ticks);	// priod
	EPWM_SET_CMR(EPWM0, 4, duty_ticks_high_side); // cmpdata n :	0 

	EPWM_SET_CNR(EPWM0, 5, period_ticks);
	EPWM_SET_CMR(EPWM0, 5, duty_ticks_high_side);	// cmpdata n : 1

	EPWM_SET_CNR(EPWM1, 4, period_ticks);	// priod
	EPWM_SET_CMR(EPWM1, 4, duty_ticks_high_side); // cmpdata n :	0 

	EPWM_SET_CNR(EPWM1, 5, period_ticks);
	EPWM_SET_CMR(EPWM1, 5, duty_ticks_high_side);	// cmpdata n : 1
	
}
  

