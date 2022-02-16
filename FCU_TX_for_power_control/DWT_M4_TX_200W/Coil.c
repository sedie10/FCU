#include "Coil.h"
#include "Hardware.h"
//#include "NuMicro.h"


Pwm_Bridge_Mode_State mode_state=half_bridge;
static WPCQiTxCoil_t usesCoil;


/****************************************************************************************************/
/*																									*/
/*		QiTxBalancedCoil_vselect																			*/ 
/*																									*/
/*																									*/
/****************************************************************************************************/
void QiTxBalancedCoil_vselect( uint8_t fo_bal_coil_selection )
{
	switch ( fo_bal_coil_selection )
	{
		case FO_BAL_COIL_SEL_0:
			FO_BAL_COIL_DRV0_SEL = 1;
			FO_BAL_COIL_DRV1_SEL = 0;
			FO_BAL_COIL_DRV2_SEL = 0;
			FO_BAL_COIL_DRV3_SEL = 0;
			break;

		case FO_BAL_COIL_SEL_1:
			FO_BAL_COIL_DRV0_SEL = 0;
			FO_BAL_COIL_DRV1_SEL = 1;
			FO_BAL_COIL_DRV2_SEL = 0;
			FO_BAL_COIL_DRV3_SEL = 0;
			break;

		case FO_BAL_COIL_SEL_2:
			FO_BAL_COIL_DRV0_SEL = 0;
			FO_BAL_COIL_DRV1_SEL = 0;
			FO_BAL_COIL_DRV2_SEL = 1;
			FO_BAL_COIL_DRV3_SEL = 0;
			break;

		case FO_BAL_COIL_SEL_3:
			FO_BAL_COIL_DRV0_SEL = 0;
			FO_BAL_COIL_DRV1_SEL = 0;
			FO_BAL_COIL_DRV2_SEL = 0;
			FO_BAL_COIL_DRV3_SEL = 1;
			break;

		case FO_BAL_COIL_SEL_NONE:
			FO_BAL_COIL_DRV0_SEL = 0;
			FO_BAL_COIL_DRV1_SEL = 0;
			FO_BAL_COIL_DRV2_SEL = 0;
			FO_BAL_COIL_DRV3_SEL = 0;
				break;
	}
}

/****************************************************************************************************/
/*																									*/
/*		QiTxCoil_vSelect																						*/ 
/*																									*/
/*																									*/
/****************************************************************************************************/

void QiTxCoil_vSelect(WPCQiTxCoil_t coil_selection )
{
    usesCoil = coil_selection;
	
	switch ( usesCoil )
	{
		case COIL_SEL_0:
			COIL_DRV0_SEL = 1;
			COIL_DRV1_SEL = 0;
			COIL_DRV2_SEL = 0;
			break;

		case COIL_SEL_1:
			COIL_DRV0_SEL = 0;
			COIL_DRV1_SEL = 1;
			COIL_DRV2_SEL = 0;
			break;

		case COIL_SEL_2:
			COIL_DRV0_SEL = 0;
			COIL_DRV1_SEL = 0;
			COIL_DRV2_SEL = 1;
			break;

		case COIL_SEL_01:
			COIL_DRV0_SEL = 1;
			COIL_DRV1_SEL = 1;
			COIL_DRV2_SEL = 0;
			break;

		case COIL_SEL_12:
			COIL_DRV0_SEL = 0;
			COIL_DRV1_SEL = 1;
			COIL_DRV2_SEL = 1;
			break;

		case COIL_SEL_02:
			COIL_DRV0_SEL = 1;
			COIL_DRV1_SEL = 0;
			COIL_DRV2_SEL = 1;
			break;

		case COIL_SEL_012:
			COIL_DRV0_SEL = 1;
			COIL_DRV1_SEL = 1;
			COIL_DRV2_SEL = 1;
			break;

		case COIL_SEL_NONE:
			COIL_DRV0_SEL = 0;
			COIL_DRV1_SEL = 0;
			COIL_DRV2_SEL = 0;
			break;
	}
}

/*----------------------------------------------------------------------------------------------------------*/
/*      																										*/ 	
/*      																										*/
/*      function : QiTxCoil_vInit																							*/
/*       																										*/
/* 																											*/
/*----------------------------------------------------------------------------------------------------------*/

void QiTxCoil_vInit ( void )
{
	QiTxCoil_vSelect(COIL_SEL_NONE);
}

/*----------------------------------------------------------------------------------------------------------*/
/*      																										*/ 	
/*      																										*/
/*      function : QiTxCoil_vGetCurrentCoil																							*/
/*       																										*/
/* 																											*/
/*----------------------------------------------------------------------------------------------------------*/
WPCQiTxCoil_t QiTxCoil_vGetCurrentCoil ( void )
{
    return usesCoil;
}
/*----------------------------------------------------------------------------------------------------------*/
/*      																										*/ 	
/*      																										*/
/*      function : getNextCoilToUse																							*/
/*       																										*/
/* 																											*/
/*----------------------------------------------------------------------------------------------------------*/

WPCQiTxCoil_t QiTxCoil_vGetNextCoil ( void )
{
	WPCQiTxCoil_t nextCoil;

    if( usesCoil == COIL_SEL_NONE )
		nextCoil = COIL_SEL_1;
	else if ( usesCoil == COIL_SEL_1)
		nextCoil = COIL_SEL_2;
	else
		nextCoil = COIL_SEL_1;
	//printf("coil =%d\n\r", whichCoil);
	nextCoil = COIL_SEL_1;
	return nextCoil;
}
/*----------------------------------------------------------------------------------------------------------*/
/*      																										*/ 	
/*      																										*/
/*      function : setWhichCoilWillUse																							*/
/*       																										*/
/* 																											*/
/*----------------------------------------------------------------------------------------------------------*/

void QiTxCoil_vSetCurrentCoil( WPCQiTxCoil_t coil )
{
	usesCoil = coil;
}

/*----------------------------------------------------------------------------------------------------------*/
/*      																										*/ 	
/*      																										*/
/*      function : setWhichCoilWillUse																							*/
/*       																										*/
/* 																											*/
/*----------------------------------------------------------------------------------------------------------*/

uint8_t QiTxCoil_vCheckCoil( WPCQiTxCoil_t coil, uint8_t powerClass )
{
   uint8_t result = 0;
   
   if( coil == COIL_SEL_1 )
   {
      if( powerClass == 0 )
	  	result = 1;
	  else if ( powerClass == 1 )
	  	result = 0;
	  else result = 1;
   }
   else if( coil == COIL_SEL_2 )
   {
      if( powerClass == 0 )
	  	result = 0;
	  else if ( powerClass == 1 )
	  	result = 0; // 1 test.,,,, small coil
	  else
	  	result = 1;
   }
   return result;
}

/****************************************************************************************************/
/*																									*/
/*		QiTx_vPowerControl																			*/ 
/*																									*/
/*																									*/
/****************************************************************************************************/

void QiTx_vPowerControl( onoff_t onoff)
{
	VBUS_SW_EN = onoff;
}

/****************************************************************************************************/
/*																									*/
/*		VBUS_SW_EN																						*/ 
/*																									*/
/*																									*/
/****************************************************************************************************/
void half_bridge_set(void)
{

	mode_state=half_bridge;
}
/****************************************************************************************************/
/*																									*/
/*		VBUS_SW_EN																						*/ 
/*																									*/
/*																									*/
/****************************************************************************************************/
void full_bridge_set(void)
{
//	if(qi_major_version>1)
	{
		/* Enable PWM Output path for PWMA channel 1 */
//		SYS->GPA_MFP |= SYS_GPA_MFP_PA13_Msk; // set gpio13 to PWM1 mode
	}
//	PWMA->POE |= PWM_POE_POE0_Msk | PWM_POE_POE1_Msk;
}

// PWM period/duty support
/****************************************************************************************************/
/*																									*/
/*		VBUS_SW_EN																						*/ 
/*																									*/
/*																									*/
/****************************************************************************************************/
void set_bridge(Pwm_Bridge_Mode_State mstate)
{
    
	if(mstate !=mode_state)
	{
		mode_state=mstate;
		if(mode_state == half_bridge)
		{
			half_bridge_set();
		}
		else
		{
			full_bridge_set();
		}
	}
    
}



