#ifndef TARGETDEF_H
#define TARGETDEF_H


#if defined   ( __CC_ARM   ) 
#define __ramfunc __attribute__ ((section(".ramfunc")))
#else
#define __ramfunc
#endif


#define inline  __inline

typedef unsigned char		uint8_t;
typedef unsigned short int 	uint16_t;
typedef unsigned int 		uint32_t;
typedef float				fix16_t;

/********************************************************************************************************/
/*																										*/
/*					Define the function feature																*/
/*																										*/
/*																										*/
/********************************************************************************************************/
#define KR_BUILD
#define EPP_NEG

#define CONFIG_EPP

#define USE_96_CENTER_ALIGN_MHZ


#if (defined(KR_BUILD))
#define POWER_NOISY_FILTER
  #if( defined(POWER_NOISY_FILTER))
  #define FILTER_MAX_COUNTER      400
  #endif
#define NEW_METHOD_DUTY
#define NEW_FILTER
#define IGNORE_EPT_NO_RESPONSE
#define CONFIG_FORCE_GUI
#define NEW_QI_CONTROL
#endif

/*Using epwm channel 4 & 5 */
#define USE_EPWM45
/*Using the ecap feature */
#define USE_ECAP


#define QI_TX
#ifdef QI_TX
//#define USB_PD
#define ASK_COMM_BIT_DEBUG
//#define FSK_COMM_BIT_DEBUG
//#define Q_MEASURE
//#define FO_BALANCED_MEASURE
#ifdef Q_MEASURE
#define Q_FACTOR_DEBUG
//#define FOD
//#define Q_FACTOR_VBUS_SET_DEBUG
#ifdef USE_ECAP
#define ECAP_Q_FACTOR
#endif
#endif

#define VOLTAGE_CONTROL
//#define FREQ_CONTROL

#define OVER_VOLTAGE_CONTROL
#endif

#define VBUS_24V_APP
#define HALF_BRIDGE

#define BLE_HANDSHAKING    

/********************************************************************************************************/
/*																										*/
/*					Define the feature flage																	*/
/*																										*/
/*																										*/
/********************************************************************************************************/
#define DEBUG_FSK_PRN					0
#define DEBUG_ASK_BIT_PRN				0
#define DEBUG_ASK_PRN					0
#define DEBUG_QI_STATE_PRN				1
#define DEBUG_QI_Q_MEAS_PRN				0
#define DEBUG_QI_CEP_PRN				0
#define DEBUG_QI_BLE_CEP_PRN			0
#define DEBUG_QI_FO_MEAS_BY_BAL_PRN		0
#define DEBUG_QI_PWR_CONTRACT_PRN		1
#define DEBUG_BLE_PRN                   0

#endif // TARGETDEF_H

