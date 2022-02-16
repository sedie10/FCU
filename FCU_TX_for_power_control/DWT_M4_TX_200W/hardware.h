#ifndef _HARDWARE_H
#define _HARDWARE_H

#include "NuMicro.h"

#define ECAP0_TIMEOUT   		34000  			/* 2.5ms */
#define ECAP1_TIMEOUT   		3840  			/*  @ 50kHz , 192Mhz */
#define ECAP1_TIMER_FREQ 		192000000


#define FO_BAL_COIL_DRV0_SEL									PH11
#define FO_BAL_COIL_DRV1_SEL 									PH10
#define FO_BAL_COIL_DRV2_SEL 									PH9
#define FO_BAL_COIL_DRV3_SEL 									PH8

#define COIL_DRV0_SEL 											PF7
#define COIL_DRV1_SEL 											PF8
#define COIL_DRV2_SEL 											PF9

#define VBUS_SW_EN												PC7

#define Q_V_SUPPLY_EN											PC6									
#define Q_MEASURE_RST											PF11
#define Q_GATE_DR_SUPPLY_EN                                     PG4


// BLE EN
#define BLE_EN													PE10

// OVP_CTL
#define OVP_CTL													PB12

// Q gate driver supply ctl
#define Q_SUPPLY_EN												PG4

#define DCDC_EN													PH4

void SYS_vInit(void);
void CLK_vInit(void);
void GPIO_vInit(void);
void UART_vInit(void);
void TMR0_CAP_init(void);
void ECAP0_vInit(void);
void ECAP1_vInit(void);

void TMR0_vInit(void);
void TMR1_vInit(void);
void ECAP0_vStart(void);
void ECAP0_vStop(void);
void ECAP1_vStart(void);
void ECAP1_vStop(void);

void PWM_vInit(void);
void Pwm_Work_Q_Measure(void);
void ADC_vInit(void);

void SYS_vInitSetting ( void );


#endif


