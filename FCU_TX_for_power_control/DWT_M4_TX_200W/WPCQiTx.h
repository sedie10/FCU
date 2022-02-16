#ifndef _QI_H
#define _QI_H

#include "Targetdef.h"
#include "application.h"



#define DIGITAL_PING_PERIOD          	 10	/* digital ping after the number of analog ping fail */


#define T_MIN_DELAY_MS                    5                                     /*!< T(min)delay in ms */
#define T_MAX_DELAY_MS                    205                                   /*!< T(max)delay in ms */

#define T_IDCFG_RESPOND                   4										/*3~10ms*/
#define T_LOOP_TIME_MS                    1                                     /*!< Tloop in ms */
#define T_NEGO_DELAY_MS                   4

#define T_NEGO_TIMEOUT_MS                 350	/*350*/
#define T_FOD_TIMEOUT_MS                  10000
#define T_PREAMBLE_TIMEOUT				  4
#define T_RESPONSE_MS						            4             /*!< Tresponse in ms */


//#define T_ODI_MS                            1000          /*!< Todi limit in ms */
//#define T_PING_MS                           65            /*!< Tping limit in ms */
#define T_PING_MS                           105            /*!< Tping limit in ms */
//#define T_FIRST_MS													20						/*!< Tfirst limit in ms */
#define T_FIRST_MS                          25            /*!< Tfirst limit in ms */
//#define T_NEXT_MS                           95            /*!< Tnext limit in ms */
#define T_NEXT_MS                           105            /*!< Tnext limit in ms */
#define T_MAX_MS                            170           /*!< Tmax limit in ms */
#define T_TIMEOUT_MS                        1500          /*!< Ttimeout limit in ms */
#define T_POWER_MS                          23000         /*!< Tpower limit in ms */
//#define T_INTERNAL_MS                       1500//350     /*!< Ttimeout limit in ms from the second time */
#define T_INTERNAL_MS                       T_TIMEOUT_MS//350     /*!< Ttimeout limit in ms from the second time */
//#define T_INTERNAL_MS                       6500//350     /*!< Ttimeout limit in ms from the second time */
#define T_RECEIVED_MS                       24000//4000   /*!< Tpower limit in ms */
#define T_ACTIVE_MS                         30            /*!< Tactive in ms */
#define T_SETTLE_MS                         5             /*!< Tsettle in ms */
#define T_SILENT_MS                         6             /*!< Tsilent in ms */
#define T_CONTROL_MS                        25            /*!< Tcontrol in ms */


typedef enum
{
  end_tx_unknown,
  end_tx_charge_complete,
  end_tx_internal_fault,
  end_tx_over_temperature,
  end_tx_over_voltage,
  end_tx_over_current,
  end_tx_battery_failure,
  end_tx_reconfigure,
  end_tx_no_response,
  end_tx_renegotiate,
  end_tx_negotiation_failure,
  end_tx_restart_power_transfer
    
} End_Transfer_Code;

#if 0// check later
typedef enum
{
	QiRx_EndPowerTransferReason_Unknown = 0,
	QiRx_EndPowerTransferReason_ChargeComplete,
	QiRx_EndPowerTransferReason_InternalFault,
	QiRx_EndPowerTransferReason_OverTemperature,
	QiRx_EndPowerTransferReason_OverVoltage,
	QiRx_EndPowerTransferReason_OverCurrent,
	QiRx_EndPowerTransferReason_BatteryFailure,
	QiRx_EndPowerTransferReason_ReConfigure,
	QiRx_EndPowerTransferReason_NoResponse,
	QiRx_EndPowerTransferReason_NegotiationFailure = 0x0A,
	QiRx_EndPowerTransferReason_Fod,
	QiRx_EndPowerTransferReason_Max = 0xff
}WPCQi_EndPowerTransferReason_t;
#endif


typedef enum
{
  // Ping phase
  
  WPCQi_SignalStrengthPacket_H 				= 0x01,
  WPCQi_EndPowerTransferPacket_H,  
  WPCQi_ControlErrorPacet_H,
  WPCQi_8BitReceivedPowerPacket_H,
  WPCQi_ChargeStatusPacket_H,
  WPCQi_PowerContronHoldOffPacket_H,
  WPCQi_GeneralRequestPacket_H,

  WPCQi_RenegotiatePacket_H					= 0x09,
  
  WPCQi_proprietary_1 						= 0x18,
  WPCQi_proprietary_2,

  WPCQi_SpecificRequestPacket_H             = 0x20,
  WPCQi_ReceiverPowerExtendPacket_H,
  WPCQi_FODStatusPacket_H,

  WPCQi_proprietary_3 						= 0x28,
  WPCQi_proprietary_4,
  
  WPCQi_24BitReceivedPowerPacket_H			= 0x31,

  WPCQi_proprietary_5                       = 0x38,
  WPCQi_proprietary_6 						= 0x48,

  WPCQi_ConfigurationPacket_H				= 0x51,

  WPCQi_WPID_MostSignificantBitsPacket_H	= 0x54, 					
  WPCQi_WPID_LeastSignificantBitsPacket_H 	= 0x55, 					

  WPCQi_proprietary_7 						= 0x58,

  WPCQi_ProprietaryOOBInfoPacket_H			= 0x68,

  WPCQi_IdetificationPacket_H				= 0x71,

  WPCQi_proprietary_9 						= 0x78,

  WPCQi_ExtendedIdentificationPacket_H		= 0x81,

  WPCQi_proprietary_10 						= 0x84,
  WPCQi_proprietary_11 						= 0xA4,
  WPCQi_proprietary_12 						= 0xC4,
  WPCQi_proprietary_13 						= 0xE2,
  WPCQi_proprietary_14 						= 0xF2

}WPCQi_Header_t;


typedef enum
{

  WPCQi_SpecificRequest_EndNegotiationPacker_H 			= 0x0,
  WPCQi_SpecificRequest_GuaranteedPowerPacket_H,
  WPCQi_SpecificRequest_ReceivedPowerPacketTypePacket_H,
  WPCQi_SpecificRequest_FSKParameterPacket_H,
  WPCQi_SpecificRequest_MaxiumPowerPacket_H,
  WPCQi_SpecificRequest_ReservedPacket_H 				= 0xEF,
  WPCQi_SpecificRequest_ProprietaryPacket_H				= 0xFF,

}WPCQiSpecificRequestPacketHeader_t;


typedef enum
{
  WPCQi_GeneralRequest_TransmitterIdentificationPacket_H = 0x30, 
  WPCQi_GeneralRequest_TransmitterCapabilityPacket_H, 

}WPCQiGeneralRequestPacketHeader_t;


typedef enum 
{

  QI_INVALID_S = 0, 
  QI_RESET_S,       
  QI_RESET_WAIT_S,     
  QI_MEASURE_S,      
  QI_FO_BALANCED_S,  
  QI_SELECTION_S,    
  QI_PINGWAIT_SIGNAL_STRENGTH_S,
  QI_IDCFG_WAIT_ID_S, 
  QI_IDGFG_WAIT_EXID_S, 
  QI_IDCFG_WAIT_CONFIG_S, 
  QI_IDCFG_RESPOND_S, 
  QI_NEGO_WAIT_ID_S,
  QI_NEGO_ID_RESPOND_S, 
  QI_NEGO_WAIT_CFG_S,
  QI_NEGO_CFG_RESPOND_S,
  QI_NEGO_WAIT_MP_S, 
  QI_NEGO_MP_RESPOND_S, 
  QI_NEGO_WAIT_GP_S, 
  QI_NEGO_GP_RESPOND_S, 
  QI_NEGO_WAIT_SRP_S,
  QI_NEGO_SRP_RESPOND_S, 
  QI_NEGO_WAIT_END_S, 
  QI_NEGO_END_RESPOND_S, 
  QI_NEGO_WAIT_PACKAGE_S, 
  QI_CALIBRATION_S, 
  QI_POWER_TRANSFER_S,
  QI_STATE_MACHINE_DELAY_S,
  QI_WAIT_SPEAKER_DONE_S, 
  QI_LATCH_OFF_S, 
  QI_WAIT_PAD_REMOVAL_S,
  QI_WAIT_PAD_REMOVAL_QUICK_S, 
  QI_WAIT_PAD_REMOVAL_NO_TIMEOUT_S, 
  QI_WAIT_RESTART_CHARGING_S, 
  QI_BLE_WAIT_PAIRING_S,
  QI_TXSTATE_GOTO_ERROR_S,
    
} WPCQiTx_State_t;

typedef struct {
  uint8_t guaranteeed_power;
  uint8_t max_power;
  uint8_t received_power;
  uint8_t modulation_depth;
  uint8_t count;
}qi_Tx_Power_Contract; 
// External Data declarations

typedef enum
{
    RESET_POWER_ON = 0x00,

	RESET_QVALUE_IS_NOT_GOOD,

    RESET_RX_PACKET_ERROR = 0x10,	
	RESET_RX_PACKET_RECEVING_TIMEOUT,
	RESET_RX_PACKET_SEQUENCE_ERROR,

	RESET_DIGITALPING_COUNTOVER = 0x20,
	RESET_DIGITALPING_TIMEOUT,
	RESET_DIGITALPING_FIRST_TIMEOUT,

	RESET_IDCFG_MAJORVERION_ERROR = 0x30,
	RESET_IDCFG_IDMESSAGE_SEQUENCE_ERROR,
	RESET_IDCFG_IDMESSAGE_RECEIVING_TIMEOUT,
	RESET_IDCFG_IDMESSAGE_TIMEOUT,	
	RESET_IDCFGWAIT_STATE_ERROR,
	RESET_IDCFG_EXIDMESSAGE_SEQUENCE_ERROR,
	RESET_IDCFG_EXIDMESSAGE_RECEIVING_TIMEOUT,	
	RESET_IDCFG_EXIDMESSAGE_TIMEOUT,
    RESET_IDCFG_MAXPWR_ERROR,
	RESET_IDCFG_HOLDOFF_TIMEOUT,

	RESET_BLE_COMMUNICATION_ERROR = 0x40,

	RESET_NEGO_MESSAGE_TIMEOUT = 0x50,
	RESET_NEGO_SEQUENCE_ERROR,
	RESET_NEGO_PROCESS_ERROR,

	RESET_TRANSEFER_CEP_PACKET_TIMEOUT = 0x60,
	RESET_TRANSEFER_POWERPACKET_TIMEOUT,
	RESET_TRANSEFER_RXPOWER_ERROR,
	RESET_TRANSEFER_RECEIVE_POWEREND,
	RESET_TRANSEFER_INVALID_PACKET_ERROR,

	RESET_FOD_REMOVE_WAIT_TIMEOUT = 0x80,
	RESET_FOD_REMOVE_APING_COUNTOVER,
	RESET_FOD_REMOVE_NO_TIME_COUNTOVER,

	RESET_COIL_SELECTION_MISMATCH = 0x90,
    RESET_TX_LOOP_DELAY,
	RESET_TX_STATE_DEFAULT,
	
	RESET_SYSTEM_UNDER_VOLTAGE = 0xA0,
	RESET_SYSTEM_OVER_VOLTAGE,
	
}qitxStateResetReason_t;


extern int uart_comm_deep_sleep_emulate;                /*!< emulating deep sleep mode */

extern int efficiency;                              /*!< Last calculated efficiency */

extern uint8_t qi_end_transfer_code;                    /*!< end transfer code */
extern uint8_t qi_signal_strength;                      /*!< unscaled signal strength */
extern signed char qi_control_error_value;              /*!< unscaled control error value */
extern fix16_t qi_control_error_value_f16;          /*!< Last measured cell current in amperes in fix16_t format */
extern uint32_t qi_received_power;                       /*!< unscaled received power */
extern uint8_t qi_charge_status;                        /*!< charge status */
extern uint8_t qi_holdoff_time_ms;                      /*!< unscaled holdoff time in ms */
extern uint8_t qi_power_class;                          /*!< power class */
extern uint8_t qi_maximum_power;                        /*!< maximum power */
extern uint8_t qi_oob_flag;
extern uint8_t qi_prop;                                 /*!< power transfer property */
extern uint8_t qi_count;                                /*!< number of optional configuration packets */
extern uint8_t qi_window_size;                          /*!< number of 4ms slots to average power */
extern uint8_t qi_window_offset;                        /*!< interval between averaging window and received power packet, in number of 4ms slots */
extern uint8_t qi_neg;//add
extern uint8_t qi_FSKPolarity;//add
extern uint8_t qi_FSKDepth;//add
extern uint8_t qi_FSKReserved;

extern uint8_t qi_oob;
extern uint8_t qi_oobMacAddr[6];

extern uint8_t qi_FSKPolarity_20;//add
extern uint8_t qi_FSKDepth_20;//add
extern uint8_t qi_recevied_20;
extern uint8_t qi_FSKReserved_20;

extern uint8_t qi_guaranteed_class;//add
extern uint8_t qi_guaranteed_reserved;
extern uint8_t qi_guaranteed_power;//add
extern uint8_t qi_potential_class;//add
extern uint8_t qi_potential_power;//add
extern uint8_t qi_major_version;                        /*!< major version of the wireless power specification complies */
extern uint8_t qi_minor_version;                        /*!< minor version of the wireless power specification complies */
extern uint16_t qi_manufacturer_code;                   /*!< manufacturer ID */
extern uint8_t qi_ext;                                  /*!< set to 1 if using extended device identifier to identify power receiver */
extern uint32_t qi_basic_device_identifier;             /*!< part of device identification */
extern uint8_t qi_extended_device_identifier[8];        /*!< optional part of device identification */
extern int qi_packet_first_bit_timer_ms;                /*!< Number of milli-seconds elapsed since first bit of the current packet received */
extern int qi_packet_last_bit_timer_ms;                 /*!< Number of milli-seconds elapsed since last bit of the last packet received */

extern uint8_t	qi_received_power_mode;
extern uint8_t	qi_maximum_power_reserved;

extern uint8_t 	qi_wpid55_incorrect_checksum;


extern uint8_t qi_quality_factor_mode;
extern uint8_t qi_ref_quality_factor;
extern uint8_t qi_fod_status_reserved;
extern int qi_comm_message_received;                    /*!< non-zero if valid comm packet received */
extern uint8_t qi_last_packet_header;                   /*!< Last valid packet header received */
extern uint8_t qi_tx_last_packet_header; //add
extern uint8_t qi_tx_received_power_header;//add
extern uint8_t qi_message_length;                       /*!< Number of bytes in message payload after last header byte received */
extern uint8_t neg_change_count;
extern fix16_t qi_coil_current_f16;                     /*!< Last measured coil current in amperes in fix16_t format */
extern fix16_t qi_control_error_value_f16;               /*!< Last measured cell current in amperes in fix16_t format */

extern uint8_t qi_device_charging;				/*!< Set to non-zero when device is charging */
extern uint8_t qi_device_charged;				/*!< Set to non-zero when device is finished charging */
extern int qi_receiving_start_bit_message;                  /*!< Set to non-zero when qi comm state machine is receiving a message */

extern uint8_t qi_foreign_object_found;                     /*!< Set to non-zero when foreign object has been found */

extern uint8_t qi_comm_enabled;                             /*!< Set to non-zero if communications receive is enabled */

extern uint8_t voltage_droop_flag;                          /*!< Set to non-zero when Vbus voltage-droop is detected */

#ifdef OVER_VOLTAGE_CONTROL
extern uint8_t coil_over_voltage_flag;                          /*!< Set to non-zero when coil over voltage is detected */
#endif

extern uint8_t over_temp_flag;                              /*!< Set to non-zero when the temperature is over the threshold */

extern uint8_t is_control;

extern int over_current_flag;                           /*!< Set to non-zero if system in over-current */

extern uint8_t efficiency_low_flag;                         /*!< Set to non-zero when the efficiency is lower than threshold */

extern uint16_t pwm_duty_ticks;                         /*!< Number of timer ticks for coil PWM duty cycle */

extern fix16_t min_freq_kHz_f16; 

extern int qi_low_power_mode;                           /*!< Set to non-zero if low-power mode active */

extern int lp_filter_hys;                               /*!< Adaptive hysteresis value to be using */

extern uint8_t dynamic_hysteresis;                          /*!< Current value of dynamic hysteresis */
extern uint8_t collecting_dynamic_hysteresis;               /*!< Set to 1 when collecting dynamic hysteresis */
extern int lp_alpha;                                    /*!< LP_ALPHA constant used for firmware LP filter */

extern int num_comm_errors;                             /*!< Number of communication errors while processing packet */

extern WPCQiTx_State_t state;                                 /*!< Main Qi state machine state */

#ifdef DYNAMIC_DIGITAL_PING_POWER
extern int digital_ping_freq;
extern int digital_ping_duty;
#endif

extern uint8_t de_debug_msg[10]; 
extern uint8_t de_debug_msg_length;

extern qi_Tx_Power_Contract g_st_power_contract;//add

// External function declarations

extern void WPCQi_TxReset(void);               /*!< Resets Qi state machine. If flag set, enter wait remove from pad state after */
extern void qi_state_machine_latch_off(void);           /*!< Latches off state machine (for error states that aren't recoverable) */
extern void WPCQiTx_Delay(long ms);             /*!< Number of milliseconds to delay state machine processing */
extern void qi_comm_state_machine_reset(void);          /*!< Resets qi communication state machine */
extern void qi_comm_message_state_machine_reset(void);  /*!< Resets qi communication message state  machine */
extern void WPCQiTx_WaitPadRemoval(void);     /*!< Declare FOD, and latch off power transfer until item removed from charge pad */

extern void qi_lp_filter_reset(void);                   /*!< Resets the LP filter value */
extern void dynamic_hysteresis_reset(void);             /*!< Resets dynamic hysteresis value */
extern __ramfunc void WPCQi_Tx(void);           /*!< Main qi state machine */
extern __ramfunc void ASK_vDecode(uint32_t period );      /*!< qi Communications state machine */


extern void reset_freq_machine(void);
extern void freq_control_machine(void);


#endif // QI_H
 
 
