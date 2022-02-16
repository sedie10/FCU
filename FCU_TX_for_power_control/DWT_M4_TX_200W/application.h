// application.h
#ifndef APPLICATION_H
#define APPLICATION_H

#include "NuMicro.h"
#include "WPCQiTx.h"
#include "app_flash_config.h"
#include "pid.h"
#include "hardware.h"
#include "targetdef.h"
#include "Adc.h"

/********************************************************************************************************/
/*																										*/
/*					Define the structure																*/
/*																										*/
/*																										*/
/********************************************************************************************************/
// pwm mode functions


#if defined( BLE_HANDSHAKING )

typedef enum {

	QI_COMMTYPE_COIL = 0,
	QI_COMMTYPE_BLE,
	QI_COMMTYPE_NFC,
	QI_COMMTYPE_MAX

}qiCommType_t;

#endif



/********************************************************************************************************/
/*																										*/
/*					Define the function feature																*/
/*																										*/
/*																										*/
/********************************************************************************************************/

#define debug_fsk_print(args ...) if (DEBUG_FSK_PRN) printf( args)
#define debug_ask_bit_print(args ...) if (DEBUG_ASK_BIT_PRN) printf( args)
#define debug_ask_print(args ...) if (DEBUG_ASK_PRN) printf( args)
#define debug_qi_state_print(args ...) if (DEBUG_QI_STATE_PRN) printf( args)
#define debug_qi_Q_measure_print(args ...) if (DEBUG_QI_Q_MEAS_PRN) printf( args)
#define debug_qi_cep_print(args ...) if (DEBUG_QI_CEP_PRN) printf( args)
#define debug_qi_BLE_state_print(args ...) if (DEBUG_QI_BLE_CEP_PRN) printf( args)
#define debug_qi_FO_measure_print(args ...) if (DEBUG_QI_FO_MEAS_BY_BAL_PRN) printf( args)
#define debug_qi_contract_print(args ...) if (DEBUG_QI_PWR_CONTRACT_PRN) printf( args)
#define debug_ble_print(args ...) if (DEBUG_BLE_PRN) printf( args)

#if defined( BLE_HANDSHAKING )
#define BLE_RES_ACK		    			0xFF
#define BLE_RES_NAK						0x00
#define BLE_RES_ND						0x55
#endif

/********************************************************************************************************/
/*																										*/
/*					Define the MCU Peripheral																*/
/*																										*/
/*																										*/
/********************************************************************************************************/




/********************************************************************************************************/
/*																										*/
/*					Define the extern variable																	*/
/*																										*/
/*																										*/
/********************************************************************************************************/

extern FlashConfigData flash_config_ram;
extern FlashConfigData *flash_config_flash;


extern uint32_t EFF_THRESHOLD;
extern uint16_t VCOUNTS_ANALOG_SELECTION_THRESH;
extern uint16_t TIMERA_LED_TICKS;                               
extern uint16_t TIMERA_TED_TICKS; 
extern uint8_t FW_VERSION_DEBUG;

extern const int POWER_OF_TEN[];

#if defined( BLE_HANDSHAKING )
extern qiCommType_t commType;
extern uint8_t wait_ble_connection;
#endif



/********************************************************************************************************/
/*																										*/
/*					Define the function																		*/
/*																										*/
/*																										*/
/********************************************************************************************************/
static inline __ramfunc uint16_t vsense_analog_counts(void)   { return ADC_Convert(Q_RESONANCE_VOLT);}
static inline __ramfunc uint16_t vsense_digital_counts(void)    { return ADC_Convert(Q_RESONANCE_VOLT);}
static inline __ramfunc uint16_t isense_counts(void)             { return ADC_Convert(CHANNEL_CUR);}
static inline __ramfunc uint16_t vbus_counts(void)               { return ADC_Convert(CHANNEL_VBUS);}
static inline __ramfunc uint16_t rth_counts(void)                 { return ADC_Convert(CHANNEL_TMP);}
static inline __ramfunc uint16_t overvolt_counts(void)      			{ return ADC_Convert(CHANNEL_OVERVOLT);}
static inline __ramfunc uint16_t q_peak_hold_counts(void)  		{ return ADC_Convert(Q_PEAK_HOLD_DATA);}
static inline __ramfunc uint16_t fo_balanced_counts(void)   { return ADC_Convert(FO_BALANCED_V);}

/********************************************************************************************************/
/*																										*/
/*					Define the extern function																		*/
/*																										*/
/*																										*/
/********************************************************************************************************/


extern __ramfunc void efficiency_reset(void);                              /*!< Resets efficiency state machine */
extern __ramfunc int efficiency_low(int i_sense_counts, int v_sense_counts);   /*!< Returns 1 if the efficiency is too low (FOD) */

extern __ramfunc int power_calibration_phase(int i_sense_counts, int vbus_counts, int mode);


extern void vdroop_control(int vin_counts);                              /*!< Control loop for Vin */




#endif // APPLICATION_H

