#ifndef _COIL_H
#define _COIL_H

#include "Targetdef.h"

/********************************************************************************************************/
/*																										*/
/*					Define the structure																*/
/*																										*/
/*																										*/
/********************************************************************************************************/

typedef enum
{
    POWER_OFF = 0,
	POWER_ON,		
}onoff_t;


typedef enum
{
    COIL_SEL_0 =	0,
	COIL_SEL_1,
	COIL_SEL_2,
	COIL_SEL_01,
	COIL_SEL_12,
	COIL_SEL_02,
	COIL_SEL_012,
	COIL_SEL_NONE
} WPCQiTxCoil_t;

typedef enum
{
  FO_BAL_COIL_SEL_0 =	0,
	FO_BAL_COIL_SEL_1,
	FO_BAL_COIL_SEL_2,
	FO_BAL_COIL_SEL_3,
	FO_BAL_COIL_SEL_NONE
} FO_BAL_Coil_selection;

typedef enum 
{
  half_bridge,          // half bridge control
  full_bridge          // full bridge control
    
} Pwm_Bridge_Mode_State;


void QiTxBalancedCoil_vselect( uint8_t fo_bal_coil_selection );
void QiTxCoil_vSelect(WPCQiTxCoil_t coil_selection );
extern void QiTx_vPowerControl(onoff_t onoff);
extern void half_bridge_set(void);

extern void full_bridge_set(void);

extern void set_bridge(Pwm_Bridge_Mode_State mstate);
/*----------------------------------------------------------------------------------------------------------*/
/*      																										*/ 	
/*      																										*/
/*      function : QiTxCoil_vInit																							*/
/*       																										*/
/* 																											*/
/*----------------------------------------------------------------------------------------------------------*/

void QiTxCoil_vInit ( void );

/*----------------------------------------------------------------------------------------------------------*/
/*      																										*/ 	
/*      																										*/
/*      function : QiTxCoil_vGetCurrentCoil																							*/
/*       																										*/
/* 																											*/
/*----------------------------------------------------------------------------------------------------------*/
WPCQiTxCoil_t QiTxCoil_vGetCurrentCoil ( void );
/*----------------------------------------------------------------------------------------------------------*/
/*      																										*/ 	
/*      																										*/
/*      function : getNextCoilToUse																							*/
/*       																										*/
/* 																											*/
/*----------------------------------------------------------------------------------------------------------*/

WPCQiTxCoil_t QiTxCoil_vGetNextCoil ( void );

/*----------------------------------------------------------------------------------------------------------*/
/*      																										*/ 	
/*      																										*/
/*      function : setWhichCoilWillUse																							*/
/*       																										*/
/* 																											*/
/*----------------------------------------------------------------------------------------------------------*/

void QiTxCoil_vSetCurrentCoil( WPCQiTxCoil_t coil );
/*----------------------------------------------------------------------------------------------------------*/
/*      																										*/ 	
/*      																										*/
/*      function : QiTxCoil_vCheckCoil																							*/
/*       																										*/
/* 																											*/
/*----------------------------------------------------------------------------------------------------------*/

uint8_t QiTxCoil_vCheckCoil( WPCQiTxCoil_t coil, uint8_t powerClass );

#endif
