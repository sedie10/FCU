#include <stdio.h>
#include "NuMicro.h"
#include "wwdt.h"
#include "hardware.h"
#include "application.h"
#include "WPCQiTx.h"
#include "led.h"
#include "modulation.h"
#include "math.h"
#include "WPCQiTx_Standby.h"
#include "WPCQiTx_TxState.h"
#include "Fsk.h"
#include "Pwm.h"
#include "Coil.h"
#include "Ble.h"
#include "Isr.h"
#include "Pid.h"



#define _15W_MAX_POWER							15
#define _30W_MAX_POWER             				30
#define _200W_MAX_POWER							200
#define MAX_POWER                  				_200W_MAX_POWER            /*!< The maximum power */
#define GUARANTEEED_POWER_CHANGE_FLAG        	0x01
#define MAX_POWER_CHANGE_FLAG                	0x02
#define RECEIVED_POWER_CHANGE_FLAG           	0x04
#define MODULATION_DEPTH_CHANGE_FLAG         	0x08

#define RECEIVED_POWER_NORMAL_ACK   			0x00
#define RECEIVED_POWER_LIGHT_LOAD   			0x01
#define RECEIVED_POWER_CONNECTED_LOAD   		0x02
#define RECEIVED_POWER_NORMAL_NO_ACK   			0x04

#define PI  									3.141592653
#define MATHING_CAP								0.15
#define MATHING_CAP2							0.247
#define FET_ON_R                				0.013

#define Q_DRH10_PA1         					PA1
#define Q_DRL10_PA0       						PA0
#define Q_DRH11_PC1       						PC1
#define Q_DRL11_PC0        						PC0

#define Q_SELF_RESONANCE_NUM  					81
#define ADC_Q_NUM  								200
#define Q_MEASURE_AVG_NUM       				3



qi_Tx_Power_Contract g_st_power_contract;//add
const int POWER_OF_TEN[5] = {1, 10, 100, 1000,10000};
const uint8_t COUNT_BIT_ONE_IN_BYTE[16] = {0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4};
uint32_t t_control_error_ms = T_TIMEOUT_MS;
uint32_t t_received_power_ms = T_POWER_MS;
WPCQiTx_State_t state = QI_RESET_S;
static WPCQiTx_State_t state_after_delay = QI_RESET_S;
static WPCQi_FskState_t control_loop_state = FSK_IDLE_S;
fix16_t qi_coil_current_f16 = 0;                        /*!< Last measured cell current in amperes in fix16_t format */
fix16_t qi_control_error_value_f16 = 0;                        /*!< Last measured cell current in amperes in fix16_t format */
uint8_t qi_device_charging = 0;				/*!< Set to non-zero when device is charging */
uint8_t qi_device_charged = 0;				/*!< Set to non-zero when device is finished charging */
uint8_t qi_device_recharging = 0;                           /*!< Set when device is 're-charging' (after EPT:fully charged, but still on pad) */
uint8_t qi_device_unknown=0;                                /*!< Set to non-zero when got end power transfer unknown packet */
uint8_t qi_foreign_object_found = 0;                        /*!< Set to non-zero when foreign object has been found */
uint8_t qi_comm_enabled = 0;                                /*!< Set to non-zero if communications receive is enabled */
uint8_t is_control = 0;
uint8_t voltage_droop_flag = 0;  
/*!< Set to non-zero when the Vbus voltage is less than threshold */
#ifdef OVER_VOLTAGE_CONTROL
uint8_t coil_over_voltage_flag = 0;
#endif
uint8_t over_temp_flag = 0;                                 /*!< Set to non-zero when the Rth is greater than threshold */
uint8_t efficiency_low_flag = 1;                            /*!< Set to non-zero when the efficiency calculated is lower than threshold */
uint8_t dynamic_hysteresis = 0;                             /*!< Current value of dynamic hysteresis */
uint8_t collecting_dynamic_hysteresis = 0;                  /*!< Set to 1 when collecting dynamic hysteresis */
static uint8_t change_modulation_type = 0;                  /*!< Change modulation at next digital ping */
uint16_t pwm_duty_ticks = 0;                            /*!< Number of timer ticks for coil PWM duty cycle */
static long delay_counter = 0;

volatile int pwmFreq = TIMERA_TICKS_110KHZ;

#define RING_BUFFER_SIZE        4
static int i_ring_buffer[RING_BUFFER_SIZE];             /*!< Ring buffer */


static uint8_t i_ring_buffer_index = 0;                     /*!< Ring buffer index */
int received_endNego = 0;
int received_holdoffcmd = 0;
uint8_t	qi_wpid55_incorrect_checksum;
uint8_t mac_idx = 0;

#if defined( BLE_HANDSHAKING )
qiCommType_t commType = QI_COMMTYPE_COIL;
#endif


uint8_t prev_isBLE =0;
uint8_t transfer_test_data_error = 0;

WPCQiTxCoil_t coil_sel_num 			=	COIL_SEL_NONE;
WPCQiTxCoil_t prev_coil_sel_num 	=	COIL_SEL_NONE;
WPCQiTxCoil_t next_coil_sel_num 	=	COIL_SEL_NONE;



unsigned int q_peak_hold_temp,q_peak_hold_temp_min,q_peak_hold_temp_max;
int q_factor_test_min_value_time;
uint16_t q_adc[Q_SELF_RESONANCE_NUM+1]; 
uint32_t q_self_resonance_time_array[Q_SELF_RESONANCE_NUM];
uint16_t q_self_resonance_time_idx = 0;
uint8_t q_self_resonance_time_new = 0;
uint8_t q_self_resonance_test_finished = 0;
uint8_t q_self_resonance_adc_num = 0;
uint32_t freq_meausre_t,freq_meausre_f;
uint32_t freq_meausre,freq_meausre_avg;
uint8_t qi_oob_Max_Addr_send_to_ble = 0;

#ifdef APING_TEST
	static int num_contiguous_a_ping_fail = 0;                /*!< Number of times that analog ping has failed to detect device in a row */

	uint16_t threshold;
#endif
static long counter_ms = 0;
extern fix16_t light_load_tx_power, connected_load_tx_power;
extern fix16_t light_load_received_power,connected_load_received_power;
qitxStateResetReason_t reasonReset = RESET_POWER_ON;


#ifdef Q_MEASURE	
#define Q_TH_RATIO 					45
#define Q_TH_REF_ADC 				1345

uint8_t q_measure_state =0;

uint16_t q_max_value 				= 0;
uint16_t q_max_value_idx 		= 0;
uint16_t q_min_value 				= 0;
uint16_t q_min_value_idx 		= 0;
uint16_t q_th_value 				= 0;
uint16_t q_th_ref_value 		= Q_TH_REF_ADC;
static uint8_t qi_measured_quality_factor_avg;
float big_coil_ref_q_f[8][2] = { {169.15, 77.29},{175.91, 82.82},
	                                	{171.29, 86.49},{180.04, 88.60},
	                                	{180.58, 90.04},{186.17, 90.91},
	                                	{183.55, 91.43},{209.19, 92.44/*92.44*/}	};

float small_coil_ref_q_f[8][2] = { {163.66, 90.66},{177.14, 94.53},
	                                		{176.15, 97.02},{187.09, 100.34},
	                                		{191.03, 100.63},{199.30, 101.09},
	                                		{91.43, 102.51},{229.90, 102.89}	};

#define big_coil_q_th_ratio    		0.97
#define small_coil_q_th_ratio    	0.97


static	double q_temp,q_temp_q,q_temp_q2[3][Q_MEASURE_AVG_NUM+2],q_temp_l[3][Q_MEASURE_AVG_NUM+2],q_temp_r[3][Q_MEASURE_AVG_NUM+2],q_temp_f[3][Q_MEASURE_AVG_NUM+2];
static uint8_t q_num_cnt = 0,q_coil_sel = 0;
static float big_coil_ref_q_th = 0,small_coil_ref_q_th = 0;
static float coil_ref_q_slp = 0;
uint16_t /*Q_temp,*/Q_temp2;
#endif

static int qi_num_rx_optional_packets = 0;				  /*!< Number of optional config packets received in ID/CFG state */

int qi_packet_last_bit_timer_ms = 0;              /*!< Number of ms from last bit of last received message */
static int power_xfer_delay_ms = 0; 					  /*!< Number of us expired when entering power transfer (for delay) */
static int control_error_pkt_ms = 0;					  /*!< Number of milli-seconds since last CEP */
static int received_power_pkt_ms = 0;					  /*!< Number of milli-seconds since last RXPP */
static int modulation_error_pkt_ms = 0; 				  /*!< Number of milli-seconds since last CEP and modulation switch */
static int cep_count = 0;
static int charge_terminate_counter = 0;				  /*!< Number of times receiver terminated charge (no resp, reconfig, chg comlpete) before pad removal */
static int num_digital_ping_attempts = 0;				  /*!< Number of times digital ping attempted without commuincation before timeout */


uint16_t delay_ms = 0;
FlashConfigData flash_config_ram;
FlashConfigData *flash_config_flash;

extern void efficiency_init(void);


/*----------------------------------------------------------------------------------------------------------*/
/*      																										*/ 	
/*      																										*/
/*      function : WPCQi_TxReset																			*/
/*       																										*/
/* 																											*/
/*----------------------------------------------------------------------------------------------------------*/
void WPCQiTx_setWaitTime(int frclk_ticks)
{
	delay_ms = frclk_ticks;
}

/*----------------------------------------------------------------------------------------------------------*/
/*      																										*/ 	
/*      																										*/
/*      function : WPCQi_TxReset																			*/
/*       																										*/
/* 																											*/
/*----------------------------------------------------------------------------------------------------------*/
void WPCQi_TxReset(void)
{ 
  state = QI_RESET_S;
  reasonReset = RESET_BLE_COMMUNICATION_ERROR;
}

/*----------------------------------------------------------------------------------------------------------*/
/*      																										*/ 	
/*      																										*/
/*      function : WPCQiTx_Delay																			*/
/*       																										*/
/* 																											*/
/*----------------------------------------------------------------------------------------------------------*/
void WPCQiTx_Delay(long ms)
{
  state = QI_STATE_MACHINE_DELAY_S;
  
  delay_counter = ms;
  
}  

/*----------------------------------------------------------------------------------------------------------*/
/*      																										*/ 	
/*      																										*/
/*      function : WPCQiTx_WaitPadRemoval																			*/
/*       																										*/
/* 																											*/
/*----------------------------------------------------------------------------------------------------------*/
void WPCQiTx_WaitPadRemoval(void)
{
  PWM_vStopCoil();
    
  qi_device_charging = 0;
    
  qi_foreign_object_found = 1;

	led_state_machine();      // Force LED state machine to run   
  
  state =QI_WAIT_PAD_REMOVAL_S;
}

/*----------------------------------------------------------------------------------------------------------*/
/*      																										*/ 	
/*      																										*/
/*      function : WPCQiTx_WaitPadRemovalLatch																			*/
/*       																										*/
/* 																											*/
/*----------------------------------------------------------------------------------------------------------*/
void WPCQiTx_WaitPadRemovalLatch(void)
{
  qi_device_charging = 0;

	led_state_machine();      // Force LED state machine to run
  
  state =QI_WAIT_PAD_REMOVAL_NO_TIMEOUT_S; 

}

/*----------------------------------------------------------------------------------------------------------*/
/*      																										*/ 	
/*      																										*/
/*      function : WPCQiTx_vInitPower																			*/
/*       																										*/
/* 																											*/
/*----------------------------------------------------------------------------------------------------------*/
void WPCQiTx_vInitPower(uint8_t max_power, uint8_t FSKdepth)
{
    g_st_power_contract.count = 0;
    g_st_power_contract.received_power = WPCQi_8BitReceivedPowerPacket_H;
    g_st_power_contract.guaranteeed_power = 5;
    g_st_power_contract.max_power = max_power;
    g_st_power_contract.modulation_depth = FSKdepth;
}

/*----------------------------------------------------------------------------------------------------------*/
/*      																										*/ 	
/*      																										*/
/*      function : WPCQiTx_vSetNegoPower																			*/
/*       																										*/
/* 																											*/
/*----------------------------------------------------------------------------------------------------------*/

void WPCQiTx_vSetNegoPower(void)
{ 
    if(g_st_power_contract.count & RECEIVED_POWER_CHANGE_FLAG)   
        g_st_power_contract.received_power = qi_tx_received_power_header;
    
    if(g_st_power_contract.count & GUARANTEEED_POWER_CHANGE_FLAG)   
        g_st_power_contract.guaranteeed_power = (qi_guaranteed_power >> 1) * POWER_OF_TEN[qi_guaranteed_class];
    
    if(g_st_power_contract.count & MAX_POWER_CHANGE_FLAG)   
        g_st_power_contract.max_power = (qi_maximum_power >> 1) * POWER_OF_TEN[qi_power_class];
    
    if(g_st_power_contract.count & MODULATION_DEPTH_CHANGE_FLAG)   
        g_st_power_contract.modulation_depth = qi_FSKDepth;

		debug_qi_contract_print("\r\n RP:%d GP:%d MP:%d MD:%d",g_st_power_contract.received_power,
																														g_st_power_contract.guaranteeed_power,
																														g_st_power_contract.max_power,
																														g_st_power_contract.modulation_depth);
		
}   
 
 /****************************************************************************************************/
 /* 																								 */
 /* 		 WPCQiTx_powerTransferInit																 */ 
 /* 																								 */
 /* 																								 */
 /****************************************************************************************************/
void WPCQiTx_powerTransferInit(void)
{
    t_control_error_ms = T_TIMEOUT_MS;
    t_received_power_ms = T_POWER_MS;
}

 
#ifdef NEW_FILTER
#define VCOUNT_SAMPLE_PERIOD_MS             800
#define VCOUNT_PEAK_THRESHOLD               150
/****************************************************************************************************/
/*																									*/
/*			WPCQiTx_vCountSampleMachine																*/ 
/*																									*/
/*																									*/
/****************************************************************************************************/

void WPCQiTx_vCountSampleMachine(uint16_t value)
{
  static int counter=0;
  static int min,max;
  if(counter==0)
  {
    min=0x3FF;
    max=0x00;
  } 
  else if(counter <VCOUNT_SAMPLE_PERIOD_MS)
  {
    if(value >max)
    {
       max=value;       
    }
    if(min >value)
    {
       min=value; 
    }
  }
  else
  {
//    vcount_max=max;
//    vcount_min=min;
    if(max >min + VCOUNT_PEAK_THRESHOLD)
    {
//      receiver_removed=0;
    }
    else
    {
//      receiver_removed=1;
    }
    counter=0;
    return;
  }
  counter++;
}
#endif  


//static WPCQiTx_FskResponse_t qi_tx_data; 




#ifdef FO_BALANCED_MEASURE
/****************************************************************************************************/
/*																									*/
/*			WPCQiTx_ProcBalanceCoilMeasure																*/ 
/*																									*/
/*																									*/
/****************************************************************************************************/
#define FO_BAL_MEASUR_DLY_T   3000			

void WPCQiTx_ProcBalanceCoilMeasure ( void )
{
	int i,j;
	uint32_t fo_balaned_value_temp = 0;
	uint32_t fo_balaned_value_0[3] = {0};
	uint32_t fo_balaned_value_1[3] = {0};
	uint32_t fo_balaned_value_2[3] = {0};
	uint32_t fo_balaned_value_3[3] = {0};

	PWM_vUpdateBuckbooster( TIMERA_TICKS_100KHZ, TIMERA_TICKS_100KHZ*185/DCDC_PWM_RATIO_200);
	PWM_vStartBuckbooster();

	PWM_vUpdateCoil(TIMERA_DIGITAL_PING_FREQ,TIMERA_DIGITAL_PING_V_CNTRL_DUTY);

//			coil_sel_num = COIL_SEL_1;

	QiTxCoil_vSelect( coil_sel_num );

	PWM_vStartCoil();

	QiTxBalancedCoil_vselect(FO_BAL_COIL_SEL_NONE);

	CLK_SysTickDelay(5000);	//10ms
	
	QiTxBalancedCoil_vselect(FO_BAL_COIL_SEL_0);
	CLK_SysTickDelay(FO_BAL_MEASUR_DLY_T);	//40ms
	
	for(i = 0; i<10 ; i++ )
	{
		fo_balaned_value_temp += fo_balanced_counts();
	}
	
	CLK_SysTickDelay(FO_BAL_MEASUR_DLY_T);	//2ms
	
	for( j = 0 ; j < 3 ; j++ )
	{
		QiTxBalancedCoil_vselect(FO_BAL_COIL_SEL_0);
		CLK_SysTickDelay(FO_BAL_MEASUR_DLY_T);  //2ms
		fo_balaned_value_temp =0;
		for(i = 0; i<10 ; i++ )
		{
			fo_balaned_value_temp += fo_balanced_counts();
		}
		fo_balaned_value_0[j] = fo_balaned_value_temp /10;
		
		QiTxBalancedCoil_vselect(FO_BAL_COIL_SEL_1);
		CLK_SysTickDelay(FO_BAL_MEASUR_DLY_T);  //1ms
		fo_balaned_value_temp =0;
		for(i = 0; i<10 ; i++ )
		{
			fo_balaned_value_temp += fo_balanced_counts();
		}
		fo_balaned_value_1[j] = fo_balaned_value_temp /10;

		QiTxBalancedCoil_vselect(FO_BAL_COIL_SEL_2);
		CLK_SysTickDelay(FO_BAL_MEASUR_DLY_T);  //1ms
		fo_balaned_value_temp =0;
		for(i = 0; i<10 ; i++ )
		{
			fo_balaned_value_temp += fo_balanced_counts();
		}
		fo_balaned_value_2[j] = fo_balaned_value_temp /10;

		QiTxBalancedCoil_vselect(FO_BAL_COIL_SEL_3);
		CLK_SysTickDelay(FO_BAL_MEASUR_DLY_T);  //1ms
		fo_balaned_value_temp =0;
		for(i = 0; i<10 ; i++ )
		{
			fo_balaned_value_temp += fo_balanced_counts();
		}
		fo_balaned_value_3[j] = fo_balaned_value_temp /10;

//				debug_qi_FO_measure_print("\r\n Fo %d   B0:%d   B1:%d   B2:%d   B3:%d ", 
//								j,fo_balaned_value_0[j],fo_balaned_value_1[j],fo_balaned_value_2[j],fo_balaned_value_3[j]);
	}

//			VBUS_SW_EN = 0;  // VBUS PMOS FET enable.
	QiTx_vPowerControl(POWER_OFF);
	PWM_vStopBuckbooster();
	PWM_vStopCoil();
//			CLK_SysTickDelay(100000);	//100ms
	WPCQiTx_setWaitTime(RTC_0P5S_PERIOD_TICKS);
	
	fo_balaned_value_temp = fo_balaned_value_0[0] + fo_balaned_value_0[1] + fo_balaned_value_0[2];
	fo_balaned_value_0[0] = fo_balaned_value_temp / 3;//  - 44;

	fo_balaned_value_temp = fo_balaned_value_1[0] + fo_balaned_value_1[1] + fo_balaned_value_1[2];
	fo_balaned_value_1[0] = fo_balaned_value_temp / 3;//  - 16 ;

	fo_balaned_value_temp = fo_balaned_value_2[0] + fo_balaned_value_2[1] + fo_balaned_value_2[2];
	fo_balaned_value_2[0] = fo_balaned_value_temp / 3;//  + 5;

	fo_balaned_value_temp = fo_balaned_value_3[0] + fo_balaned_value_3[1] + fo_balaned_value_3[2];
	fo_balaned_value_3[0] = fo_balaned_value_temp / 3;// + 58;

	
	fo_balaned_value_temp = fo_balaned_value_0[0] + fo_balaned_value_1[0] + fo_balaned_value_2[0] + fo_balaned_value_3[0];
	fo_balaned_value_temp = fo_balaned_value_temp/4;
//			debug_qi_FO_measure_print("\r\n fo B0:%d B1:%d B2:%d B3:%d ", fo_balaned_value_0-FO_BAL_OFFSET,fo_balaned_value_1-FO_BAL_OFFSET,fo_balaned_value_2-FO_BAL_OFFSET,fo_balaned_value_3-FO_BAL_OFFSET);
	debug_qi_FO_measure_print("\r\n Fo avg:%d B0:%d   B1:%d   B2:%d   B3:%d ", 
					fo_balaned_value_temp,fo_balaned_value_0[0] - fo_balaned_value_temp,fo_balaned_value_1[0] - fo_balaned_value_temp,fo_balaned_value_2[0] - fo_balaned_value_temp,fo_balaned_value_3[0] - fo_balaned_value_temp);

	ECAP_CLR_CAPTURE_FLAG(ECAP1, ECAP_STATUS_CAPTF0_Msk | ECAP_STATUS_CAPCMPF_Msk | ECAP_STATUS_CAPOVF_Msk);		

}
#endif



#ifdef Q_MEASURE			
/****************************************************************************************************/
/*																									*/
/*			WPCQiTx_ProcCoilMeasure																*/ 
/*																									*/
/*																									*/
/****************************************************************************************************/
WPCQiTx_State_t WPCQiTx_ProcCoilMeasure ( void )
{


	WPCQiTx_State_t retState = QI_MEASURE_S;
	
	if((q_measure_state == 0)&&(q_self_resonance_test_finished == 0))
	{
#ifdef APING_TEST
		threshold = flash_config_ram.analog_ping_threshold;

		temp = analog_ping_test(threshold);
		if ((temp == 0) && (++num_contiguous_a_ping_fail <= DIGITAL_PING_PERIOD))
		{
			charge_terminate_counter = 0;		 

			WPCQiTx_setWaitTime(RTC_0P25S_PERIOD_TICKS);
		}
		else
#endif					
		{
			if(q_coil_sel ==0 )
				quality_factor_measure_test_vbus_set( COIL_SEL_1 );
			else if(q_coil_sel ==1 )
				quality_factor_measure_test_vbus_set( COIL_SEL_2 );
		}
	}
	else if ((q_self_resonance_test_finished == 1) ||(q_self_resonance_adc_num > 50)/* &&(q_measure_state == 1)*/)
	{
#ifdef ECAP_Q_FACTOR
		ECAP1_vStop();
#endif
		q_max_value = 0;
		q_temp_f[q_coil_sel][q_num_cnt] = freq_meausre_avg;
		q_temp = freq_meausre_avg * 2 * PI;
		if( q_coil_sel == 0 )
			q_temp = q_temp* q_temp * MATHING_CAP / 1000000;
		else if ( q_coil_sel == 1 )
			q_temp = q_temp* q_temp * MATHING_CAP2 / 1000000;
		q_temp = 1000000 / q_temp;
		q_temp_l[q_coil_sel][q_num_cnt] = q_temp;
		debug_qi_Q_measure_print("\r\n SRF :%d L:%.2f ",freq_meausre_avg,q_temp_l[q_coil_sel][q_num_cnt] );
//				debug_qi_Q_measure_print("\r\n");
		for( i= 1;i< Q_SELF_RESONANCE_NUM  ;i++)
		{
			if(( q_max_value == 0) &&(q_adc[i]< 4095)&&(q_adc[i]> q_adc[i+1]))
			{
				q_max_value = q_adc[i];
				q_max_value_idx = i;
				q_th_value = ( q_max_value - q_th_ref_value ) * Q_TH_RATIO / 100;
			}
			else if ( ( q_max_value != 0 ) && (q_min_value == 0 ))
			{
				if( q_adc[i] < ( q_th_value + q_th_ref_value ) )
				{
					q_min_value = q_adc[i];
					q_min_value_idx = i;
				}
			}
#if 0
			debug_qi_Q_measure_print("%d ",q_adc[i]);
			if((i%10) ==0)
				debug_qi_Q_measure_print("\r\n");
#endif					
		}
		NVIC_DisableIRQ(TMR1_IRQn);
		TIMER_ClearIntFlag(TIMER1);
#if 0
{
	int q_loop = 0;
	debug_qi_Q_measure_print("\r\n srf_time \r\n " );
	for(q_loop =2 ; q_loop < Q_SELF_RESONANCE_NUM ; q_loop++ )
	{
		debug_qi_Q_measure_print(" %d  ",q_self_resonance_time_array[q_loop]  );
		if(( (q_loop+2) % 10  ) == 0 )
			debug_qi_Q_measure_print("\r\n " );
	}
}
#endif


/*				jj=(double)log((double)2.71);						*/
/*																					 */
/*			Q = ( L / R ) 2* PI * 100kHz									*/
/*																								*/
/*				(  1/ Freq	) * 2 * PI * 100kHz 							*/
/*				------------------------							*/
/*						 2* ln( Vn /Vm )											*/

		q_temp =((double) ((q_min_value_idx - q_max_value_idx )*PI * 2 * 100000)) / (double)freq_meausre_avg;
//				debug_qi_Q_measure_print("\r\n q temp 1: %.6f ",q_temp);
//				q_temp = q_temp * PI * 2 * 100000;
//				debug_qi_Q_measure_print("\r\n q temp 2: %.2f ",q_temp);
		q_temp = q_temp /( log ((q_max_value-q_th_ref_value)/(q_min_value-q_th_ref_value)));
//				debug_qi_Q_measure_print("\r\n q temp 3: %.2f ",q_temp);
		q_temp_q =	q_temp / 2.0  ;
		q_temp_r[q_coil_sel][q_num_cnt] = q_temp_q /(PI * 2 * 100000);
		
		q_temp_r[q_coil_sel][q_num_cnt] = q_temp_l[q_coil_sel][q_num_cnt] / q_temp_r[q_coil_sel][q_num_cnt]/1000000;

		q_temp_r[q_coil_sel][q_num_cnt] = q_temp_r[q_coil_sel][q_num_cnt] - FET_ON_R;

		q_temp_q2[q_coil_sel][q_num_cnt] = (PI * 2 * 100000) * (q_temp_l[q_coil_sel][q_num_cnt] /1000000)/q_temp_r[q_coil_sel][q_num_cnt];
		
		debug_qi_Q_measure_print("\r\n Rs: %.6f QF %.6f",q_temp_r[q_coil_sel][q_num_cnt],q_temp_q2[q_coil_sel][q_num_cnt]);
		
		qi_measured_quality_factor_avg = (uint8_t)(q_temp_q) ;
		debug_qi_Q_measure_print("\r\n qrv:%d Qo:%d QMa:%d QMaI:%d QMi:%d QMiI:%d QF: %.2f %d ",
							 q_th_ref_value,q_th_value,q_max_value,q_max_value_idx,q_min_value,q_min_value_idx,q_temp_q ,qi_measured_quality_factor_avg);

		q_self_resonance_test_finished = 0;
		q_measure_state =0;

		NVIC_DisableIRQ(TMR1_IRQn);
		TIMER_ClearIntFlag(TIMER1);
		CLK_SysTickDelay(3000);  //5ms
//				VBUS_SW_EN = 1;  // VBUS PMOS FET enable.
		QiTx_vPowerControl(POWER_ON);

		TIMER_SET_CMP_VALUE(TIMER1, 6000);		//	 1ms
		NVIC_EnableIRQ(TMR1_IRQn);
#ifdef FO_BALANCED_MEASURE
		retState = QI_FO_BALANCED_S;// fo balanced; 
#else
		if (q_num_cnt >= ( Q_MEASURE_AVG_NUM - 1) )
		{
//					debug_qi_Q_measure_print("\r\n Rs: %.2f QF: %.2f ls: %.2f fs: %.2f ",
			 if(q_coil_sel == 0 )
				 printf("\r\n Big ");
			 else if(q_coil_sel == 1 )
				 printf("\r\n small ");
			 printf(" Q Rs: %.2f QF: %.2f ls: %.2f fs: %.2f ",
					(q_temp_r[q_coil_sel][0]+q_temp_r[q_coil_sel][1]+q_temp_r[q_coil_sel][2])*1000/3, 
					(q_temp_q2[q_coil_sel][3]= (q_temp_q2[q_coil_sel][0]+q_temp_q2[q_coil_sel][1]+q_temp_q2[q_coil_sel][2])/3),
					(q_temp_l[q_coil_sel][3]= (q_temp_l[q_coil_sel][0]+q_temp_l[q_coil_sel][1]+q_temp_l[q_coil_sel][2])/3),
					(q_temp_f[q_coil_sel][3]= (q_temp_f[q_coil_sel][0]+q_temp_f[q_coil_sel][1]+q_temp_f[q_coil_sel][2])/3));
		  q_num_cnt = 0;

			if( q_coil_sel >= 1 )
			{
				if (( q_temp_q2[0][3] > 13) && ( q_temp_q2[1][3] > 13 ))
				{
					if ((( q_temp_q2[0][3] > 100) && ( q_temp_q2[1][3] > 100 ))&&((q_temp_l[1][3] > 13.0) &&(q_temp_l[0][3] > 22.5) )) /*  on  PC 1 coil to coil 4mm */
					{
						next_coil_sel_num = COIL_SEL_1;
					
					}
					else if((( q_temp_q2[0][3] > 100 ) && ( q_temp_q2[1][3] > 100 ))&&((q_temp_l[1][3] < 12.0) &&(q_temp_l[0][3] > 22.5) )) /*	on	PC 1 coil to coil 9mm */
					{
						next_coil_sel_num = COIL_SEL_1;
					
					}
					else if((( q_temp_q2[0][3] > 100 ) && ( q_temp_q2[1][3] > 100 ))&&((q_temp_l[1][3] < 11.5) &&(q_temp_l[0][3] > 21.5) )) /*	on	PC 1 coil to coil 14mm */
					{
						next_coil_sel_num = COIL_SEL_1;
					
					}
					else if((( q_temp_q2[0][3] < 100 ) && ( q_temp_q2[1][3] > 13 ))&&(q_temp_l[1][3] > 13.0)) /* on smart phone, PC0 */
					{
						next_coil_sel_num = COIL_SEL_2;
					
					}
					else  
					{
						next_coil_sel_num = next_coil_sel_num;							
					}

					{
						big_coil_ref_q_th = 0;
						small_coil_ref_q_th = 0;
						coil_ref_q_slp = 0;

						for( i = 1 ; i < 9 ; i++ )
						{
							if( i == 8)
							{
								coil_ref_q_slp =(big_coil_ref_q_f[i-1][0] - big_coil_ref_q_f[i-2][0]) / (big_coil_ref_q_f[i-1][1] - big_coil_ref_q_f[i-2][1]);
								big_coil_ref_q_th = big_coil_ref_q_f[i-1][0] + ((q_temp_f[0][3]/1000.0)- big_coil_ref_q_f[i-1][1]  )*coil_ref_q_slp;
								big_coil_ref_q_th = big_coil_ref_q_th * (float)big_coil_q_th_ratio;
								i=9;
							}
							else if( (q_temp_f[0][3]/1000.0) < big_coil_ref_q_f[i][1] )
							{
								coil_ref_q_slp =(big_coil_ref_q_f[i][0] - big_coil_ref_q_f[i-1][0]) / (big_coil_ref_q_f[i][1] - big_coil_ref_q_f[i-1][1]);
								big_coil_ref_q_th = big_coil_ref_q_f[i-1][0] + ((q_temp_f[0][3]/1000.0)- big_coil_ref_q_f[i-1][1]  )*coil_ref_q_slp;
								big_coil_ref_q_th = big_coil_ref_q_th * (float)big_coil_q_th_ratio;
								i=9;
							}
						}

						for( i = 1 ; i < 8 ; i++ )
						{
							if( (q_temp_f[1][3]/1000.0) < small_coil_ref_q_f[i][1] )
							{
								coil_ref_q_slp =(small_coil_ref_q_f[i][0] - small_coil_ref_q_f[i-1][0]) / (small_coil_ref_q_f[i][1] - small_coil_ref_q_f[i-1][1]);
								small_coil_ref_q_th = small_coil_ref_q_f[i-1][0] + ((q_temp_f[1][3]/1000.0) - small_coil_ref_q_f[i-1][1] )*coil_ref_q_slp;
								small_coil_ref_q_th = small_coil_ref_q_th * (float)small_coil_q_th_ratio;
								i=8;
							}
						}
						printf("\r\n Big Q th : %.2f  small Q th : %.2f ",big_coil_ref_q_th,small_coil_ref_q_th);
					}

					if(( q_temp_q2[0][3]  < big_coil_ref_q_th ) && (q_temp_q2[1][3] > 150.0 ))
					{
						printf("\r\n Bad QF  !!!!!!! ");
						WPCQiTx_setWaitTime(RTC_1P5S_PERIOD_TICKS);
						retState = QI_RESET_S;
					}
					else
					{
						ECAP_CLR_CAPTURE_FLAG(ECAP1, ECAP_STATUS_CAPTF0_Msk | ECAP_STATUS_CAPCMPF_Msk | ECAP_STATUS_CAPOVF_Msk);		
						ECAP0_vStart();

						PWM_vUpdateBuckbooster( TIMERA_TICKS_100KHZ, TIMERA_TICKS_100KHZ*DCDC_PWM_APING/DCDC_PWM_RATIO_200);
						PWM_vStartBuckbooster();
						CLK_SysTickDelay(100000);  //100ms
						retState = QI_SELECTION_S;
					}
				}
				else
				{
					printf("\r\n Bad QF  !!!!!!! ");
					WPCQiTx_setWaitTime(RTC_1P5S_PERIOD_TICKS);
					retState = QI_RESET_S;							
					reasonReset = RESET_QVALUE_IS_NOT_GOOD;
				}
			}
			else
				q_coil_sel++;
		}
		else
		{
			q_num_cnt++;
		}
#endif
		counter_ms = 0;
	}
    return retState;
}
#endif
/****************************************************************************************************/
/*																									*/
/*			WPCQiTx_ProcSelections																*/ 
/*																									*/
/*																									*/
/****************************************************************************************************/
WPCQiTx_State_t WPCQiTx_ProcSelections ( void )
{
    WPCQiTx_State_t retState = QI_SELECTION_S;
#ifdef APING_TEST

	threshold = flash_config_ram.analog_ping_threshold;
	temp = analog_ping_test(threshold);

	if ((temp == 0) && (++num_contiguous_a_ping_fail <= DIGITAL_PING_PERIOD))
	{
		charge_terminate_counter = 0;	   

		WPCQiTx_setWaitTime(RTC_0P25S_PERIOD_TICKS);
	}
	else
#endif
	{
		 coil_sel_num = QiTxCoil_vGetNextCoil();
		 debug_qi_state_print("coil sel %d\r\n",coil_sel_num);

		// Advance to digital ping states
#ifdef VOLTAGE_CONTROL
		 if( coil_sel_num == COIL_SEL_2)
			PWM_vUpdateBuckbooster( TIMERA_TICKS_100KHZ, TIMERA_TICKS_100KHZ*DCDC_PWM_PING_C0/DCDC_PWM_RATIO_200);
		 else
			PWM_vUpdateBuckbooster( TIMERA_TICKS_100KHZ, TIMERA_TICKS_100KHZ*DCDC_PWM_PING/DCDC_PWM_RATIO_200);
		 
		 PWM_vStartBuckbooster();
		 qi_comm_enabled = 1;
#endif
		QiTx_vPowerControl(POWER_ON);
		if (change_modulation_type) //	&& (num_contiguous_a_ping_fail != 0))
		{
			advanceModulation();
			change_modulation_type = 0;
#ifdef DYNAMIC_DIGITAL_PING_POWER
			freq_control_machine();
#endif       
		}
		// Clear analog ping count
#ifdef APING_TEST
		num_contiguous_a_ping_fail = 0;
#endif
		// Set AS0SEQ0 to Vsense_digital
		resetDynamicHysteresis();
		// Apply power to the active coil and advance to wait for signal strength
		// packet

#ifdef DYNAMIC_DIGITAL_PING_POWER
		PWM_vUpdateCoil(digital_ping_freq,digital_ping_duty);
#else

#ifdef VOLTAGE_CONTROL
		PWM_vUpdateCoil(TIMERA_DIGITAL_PING_FREQ,TIMERA_DIGITAL_PING_V_CNTRL_DUTY);
			
#else
		PWM_vUpdateCoil(TIMERA_DIGITAL_PING_FREQ,TIMERA_DIGITAL_PING_DUTY);
#endif
//				set_bridge(half_bridge);
#endif

		QiTxCoil_vSelect( coil_sel_num );

		PWM_vStartCoil();

		retState = QI_PINGWAIT_SIGNAL_STRENGTH_S;

		counter_ms = 0;

		num_digital_ping_attempts++;

	}

	counter_ms = 0;
	
	return retState;
}
/****************************************************************************************************/
/*																									*/
/*			WPCQiTx_ProcSignalStrength																*/ 
/*																									*/
/*																									*/
/****************************************************************************************************/
WPCQiTx_State_t WPCQiTx_ProcSignalStrength ( void )
{
    WPCQiTx_State_t retState = QI_PINGWAIT_SIGNAL_STRENGTH_S;
	if (qi_comm_message_received)
	{

		qi_comm_message_received = 0;	  

		if (qi_last_packet_header == WPCQi_SignalStrengthPacket_H)
		{

			num_digital_ping_attempts = 0;

			debug_qi_state_print("SS received\r\n");

			BLE_EN =1;
			NVIC_EnableIRQ(UART0_IRQn);

			retState = QI_IDCFG_WAIT_ID_S;
		}
		else
		{
			// Last packet received was not signal strength, so restart state machine

			PWM_vStopCoil();

			// Got invalid packet type during digital ping, go to delay
			// state and then restart

			WPCQiTx_setWaitTime(RTC_1P5S_PERIOD_TICKS);

			retState = QI_RESET_S;
			
			reasonReset = RESET_RX_PACKET_SEQUENCE_ERROR;
		}

		counter_ms = 0;
	}
	else
	{
		// Check to see if start bit of packet has been received for a message

		if (!qi_receiving_start_bit_message)  
		{	 

			// See if we have attempted to receive comm more than 5 times in a row

			if (num_digital_ping_attempts > 3)
			{
				num_digital_ping_attempts = 0;

				PWM_vStopCoil();

				retState = QI_RESET_S;
				
				reasonReset = RESET_DIGITALPING_COUNTOVER;


				WPCQiTx_setWaitTime(RTC_1P5S_PERIOD_TICKS);

			}
			else if (counter_ms > T_PING_MS) //If more than Tping (65ms) has elapsed since current was stable
			{

				change_modulation_type = 1;

				PWM_vStopCoil();


				WPCQiTx_setWaitTime(RTC_0P25S_PERIOD_TICKS);

				reasonReset = RESET_DIGITALPING_TIMEOUT;

				retState = QI_RESET_S;
			}
		}
		else
		{
			// start of packet has been received. If more than Tfirst (20ms) has elapsed
			// since the beginning of the packet, reset state machine

			if (qi_packet_first_bit_timer_ms > T_FIRST_MS)
			{
				PWM_vStopCoil();

				debug_qi_state_print("\r\n SS don't received fisrt bit timer time over ");

				WPCQiTx_setWaitTime(RTC_1P5S_PERIOD_TICKS);

				retState = QI_RESET_S;
				reasonReset = RESET_DIGITALPING_FIRST_TIMEOUT;
			}
		}
	}
	return retState;
}

/****************************************************************************************************/
/*																									*/
/*			WPCQiTx_ProcWaitID																*/ 
/*																									*/
/*																									*/
/****************************************************************************************************/
WPCQiTx_State_t WPCQiTx_ProcWaitID ( void )
{
    WPCQiTx_State_t retState = QI_IDCFG_WAIT_ID_S;
	if (qi_comm_message_received)
	{
		qi_comm_message_received = 0;	  
	
		if (qi_last_packet_header == WPCQi_IdetificationPacket_H)
		{
			// Check for version 1 or 2
			WPCQi_InitFrequencyHandleCEP();
	
	
			if (qi_major_version != 1 && qi_major_version != 2)/*1*/
			{
				PWM_vStopCoil();
	
				debug_qi_state_print("\r\n idcfg_wait_id out of major verseion ");
	
				WPCQiTx_setWaitTime(RTC_1P5S_PERIOD_TICKS);
	
				retState = QI_RESET_S;
				
				reasonReset = RESET_IDCFG_MAJORVERION_ERROR;
			}
			else if (qi_ext == 1)
				retState = QI_IDGFG_WAIT_EXID_S;
			else
				retState = QI_IDCFG_WAIT_CONFIG_S;
	
			qi_num_rx_optional_packets = 0;
		}
		else
		{
			// Last packet received was not identification, so turn off coil
			// and reset state machine
	
			PWM_vStopCoil();
	
			debug_qi_state_print("\r\n idcfg_wait_id don't message received ");
	
			WPCQiTx_setWaitTime(RTC_1P5S_PERIOD_TICKS);
	
			retState = QI_RESET_S;//reset;
			
			reasonReset = RESET_IDCFG_IDMESSAGE_SEQUENCE_ERROR;
		}
	
		counter_ms = 0;
	}
	else
	{
		if (qi_packet_last_bit_timer_ms >= T_NEXT_MS)
		{
			// Power off coil and revert to selection phase
	
			PWM_vStopCoil();
	
			debug_qi_state_print("\r\n idcfg_wait_id don't received packet last bit time over ");
	
			WPCQiTx_setWaitTime(RTC_1P5S_PERIOD_TICKS);
	
			retState = QI_RESET_S; 	
			reasonReset = RESET_IDCFG_IDMESSAGE_TIMEOUT;
		}
	
		// Check to see if start of packet has been received for a message
	
		if (!qi_receiving_start_bit_message)
		{	 
			// Check to see if more than Tnext ms have elapsed since last bit of last packet
	
	
		}
		else
		{
	
			// We have started to receive a packet, but it hasn't been fully received
			// yet. Make sure that not more than Tmax ms have elapsed since start of packet
	
			if (qi_packet_first_bit_timer_ms > T_MAX_MS)
			{
				// Power off coil and revert to selection phase
	
				PWM_vStopCoil();
	
				WPCQiTx_setWaitTime(RTC_1P5S_PERIOD_TICKS);
	
				retState = QI_RESET_S;//reset;
				reasonReset = RESET_IDCFG_IDMESSAGE_RECEIVING_TIMEOUT;
			}
		}
	}
	return retState;
}
/****************************************************************************************************/
/*																									*/
/*			WPCQiTx_ProcWaitExID																*/ 
/*																									*/
/*																									*/
/****************************************************************************************************/
WPCQiTx_State_t WPCQiTx_ProcWaitExID ( void )
{
    WPCQiTx_State_t retState = QI_IDGFG_WAIT_EXID_S;
	
	if (qi_comm_message_received)
	{
		qi_comm_message_received = 0;
		if (qi_last_packet_header == WPCQi_ExtendedIdentificationPacket_H)
		{
			retState = QI_IDCFG_WAIT_CONFIG_S;
			counter_ms = 0;
		}
		else
		{
			// Last packet received was not extended ID packet, so turn off coil
			// and reset state machine
			PWM_vStopCoil();
			WPCQiTx_setWaitTime(RTC_1P5S_PERIOD_TICKS);
			retState = QI_RESET_S;
			reasonReset= RESET_IDCFG_EXIDMESSAGE_SEQUENCE_ERROR;
		}
	}
	else
	{
		if (qi_packet_last_bit_timer_ms >= T_NEXT_MS)
		{
			// Power off coil and revert to selection phase
			PWM_vStopCoil();
			WPCQiTx_setWaitTime(RTC_1P5S_PERIOD_TICKS);
			retState = QI_RESET_S;
			reasonReset= RESET_IDCFG_EXIDMESSAGE_TIMEOUT;
		}

		// Check to see if start of packet has been received for a message

		if (!qi_receiving_start_bit_message)
		{
			// We have started to receive a packet, but it hasn't been fully received
			// yet. Make sure that not more than Tmax ms have elapsed since start of packet

			if (qi_packet_first_bit_timer_ms > T_MAX_MS)
			{
				// Power off coil and revert to selection phase
				PWM_vStopCoil();
				retState = QI_RESET_S;//reset;
				reasonReset= RESET_IDCFG_EXIDMESSAGE_RECEIVING_TIMEOUT;
			}
		}
	}
	return retState;

}
/****************************************************************************************************/
/*																									*/
/*			WPCQiTx_ProcWaitConfig																*/ 
/*																									*/
/*																									*/
/****************************************************************************************************/
WPCQiTx_State_t WPCQiTx_ProcWaitConfig ( void )
{
    WPCQiTx_State_t retState = QI_IDCFG_WAIT_CONFIG_S;
    uint8_t ic_max_power;

	if (qi_comm_message_received)
	{
		qi_comm_message_received = 0;	 

		switch (qi_last_packet_header)
		{
			// Valid packets at this point are power control holdoff, and any
			// proprietary packets, or configuration packet

			case WPCQi_PowerContronHoldOffPacket_H:
				//edwardkwon
				received_holdoffcmd = 1;
				
				if ((qi_holdoff_time_ms < T_MIN_DELAY_MS) ||
				(qi_holdoff_time_ms > T_MAX_DELAY_MS))
				{
					PWM_vStopCoil();

					debug_qi_state_print("\r\n idcfg_wait_config out of holdoff time ");

					WPCQiTx_setWaitTime(RTC_1P5S_PERIOD_TICKS);

					retState = QI_RESET_S;
					reasonReset= RESET_IDCFG_HOLDOFF_TIMEOUT;

					counter_ms = 0;
					break;
				}
				else 
				{
					retState = QI_IDCFG_WAIT_CONFIG_S;
					break;
				}


			case WPCQi_proprietary_1:
			case WPCQi_proprietary_2:
			case WPCQi_proprietary_3:
			case WPCQi_proprietary_4:
			case WPCQi_proprietary_5:
			case WPCQi_proprietary_6:
			case WPCQi_proprietary_7:
			case WPCQi_proprietary_9:
			case WPCQi_proprietary_10:
			case WPCQi_proprietary_11:
			case WPCQi_proprietary_12:
			case WPCQi_proprietary_13:
			case WPCQi_proprietary_14:

				qi_num_rx_optional_packets++;
                return QI_TXSTATE_GOTO_ERROR_S;

			case WPCQi_ConfigurationPacket_H:

				// Check to make sure count of optional packets, power class (version),
				// holdoff time, max power > max_power/2 [10 for 5W]

				ic_max_power = (qi_maximum_power >> 1) * POWER_OF_TEN[qi_power_class];
				if(
//					if ((qi_count != qi_num_rx_optional_packets) ||
				((qi_power_class != 0) && (qi_power_class != 1)) || (ic_max_power > MAX_POWER))
				{
					// Remove power and revert to selection

					PWM_vStopCoil();

					debug_qi_state_print("\r\n WPCQi_ConfigurationPacket_H out of max power ");

					WPCQiTx_setWaitTime(RTC_1P5S_PERIOD_TICKS);

					qi_power_class = 0;
					qi_holdoff_time_ms = T_MIN_DELAY_MS;
					qi_maximum_power = 0;
					qi_count = 0;

					retState = QI_RESET_S;
					reasonReset= RESET_IDCFG_MAXPWR_ERROR;
				}
				else
				{
				    if(  QiTxCoil_vCheckCoil(coil_sel_num, qi_power_class ) == 1 )
					//if(((qi_power_class == 0)&&(( coil_sel_num == COIL_SEL_1)))/*||((qi_power_class == 1)&&(( coil_sel_num == COIL_SEL_2)))*/)
					{
						PWM_vStopCoil();
						
						debug_qi_state_print("\r\n mismatch coil sel ");
						
						WPCQiTx_setWaitTime(RTC_1P5S_PERIOD_TICKS);
						
						qi_power_class = 0;
						qi_holdoff_time_ms = T_MIN_DELAY_MS;
						qi_maximum_power = 0;
						qi_count = 0;
						
						retState = QI_RESET_S;
						
						reasonReset= RESET_COIL_SELECTION_MISMATCH;

					}
					else
					{
					// Reset timers
						control_error_pkt_ms = 0;					  /*!< Milli-seconds since last CEP */
						received_power_pkt_ms = 0;					  /*!< Milli-seconds since last RXPP */
						modulation_error_pkt_ms = 0;				  /*!< Milli-seconds since last CEP since modulation switch */

						control_loop_state = FSK_IDLE_S;

						WPCQiTx_vInitPower(ic_max_power, qi_FSKDepth);

						/* Check request for power negotiation */
#ifdef EPP_NEG
						if(qi_neg)
						{	
#ifndef CONFIG_EPP
							retState = QI_IDCFG_RESPOND_S;
#else
							WPCQiTxFskLoopState = FSK_WAIT_S;
							WPCQiFskResponse = FSK_ACK_RESPONSE;
							if ( qi_oob_flag == 0 )
								retState = QI_NEGO_WAIT_ID_S;
							else
								retState = QI_IDCFG_WAIT_CONFIG_S;
#endif
							counter_ms = 0;
						}
						else
#endif								
						{ // For BPP
							qi_device_charging = 1;  

							retState = QI_POWER_TRANSFER_S;//power_transfer;
							debug_qi_state_print("\r\n enter power transfer");

							WPCQiTx_powerTransferInit();

							efficiency_init();
							
							g_st_power_contract.received_power = WPCQi_8BitReceivedPowerPacket_H;
						}
					}
				}

				break;

#if defined(BLE_HANDSHAKING)
			case WPCQi_ProprietaryOOBInfoPacket_H:

				// Reset timers
				control_error_pkt_ms = 0;										/*!< Milli-seconds since last CEP */
				received_power_pkt_ms = 0;										/*!< Milli-seconds since last RXPP */
				modulation_error_pkt_ms = 0;									/*!< Milli-seconds since last CEP since modulation switch */

				control_loop_state = FSK_IDLE_S;
#ifdef EPP_NEG
				if(qi_neg)
				{	
					WPCQiTxFskLoopState = FSK_WAIT_S;
					WPCQiFskResponse = FSK_ACK_RESPONSE;
					retState = QI_NEGO_WAIT_ID_S;
					counter_ms = 0;

					if( qi_oob_Max_Addr_send_to_ble == 0 )
					{
					   sendMacAddToBle();
					   debug_qi_BLE_state_print("\r\nM=%x:%x:%x:%x:%x:%x,%d",
																						qi_oobMacAddr[0],
																						qi_oobMacAddr[1],
																						qi_oobMacAddr[2],
																						qi_oobMacAddr[3],
																						qi_oobMacAddr[4],
																						qi_oobMacAddr[5],commType /*isBLE*/);
					   qi_oob_Max_Addr_send_to_ble =1;
					}
				}
				else
#endif								
				{ // For BPP
					qi_device_charging = 1;  

					retState = QI_POWER_TRANSFER_S;
					debug_qi_state_print("\r\n enter power transfer");

					WPCQiTx_powerTransferInit();

					efficiency_init();
					
					g_st_power_contract.received_power = WPCQi_8BitReceivedPowerPacket_H;
				}

				break;
#endif

			default:

				// Invalid packet received, stop power and revert to seletion

				PWM_vStopCoil();

				debug_qi_state_print("\r\n default state ");

				retState = QI_RESET_S;
				reasonReset= RESET_IDCFGWAIT_STATE_ERROR;

				WPCQiTx_setWaitTime(RTC_1P5S_PERIOD_TICKS);

		}

		counter_ms = 0;
	}
	else
	{
		if ((qi_packet_last_bit_timer_ms >= T_NEXT_MS) && (received_holdoffcmd != 1)) 
		{
			received_holdoffcmd = 0;
			
			// Power off coil and revert to selection phase 			
			
			PWM_vStopCoil();
			
			debug_qi_state_print("\r\n qi_packet_last_bit_timer_ms: %d	 received_holdoffcmd :%d",qi_packet_last_bit_timer_ms, received_holdoffcmd );

			WPCQiTx_setWaitTime(RTC_1P5S_PERIOD_TICKS);

			retState = QI_RESET_S;//reset; 	
			
			reasonReset = RESET_RX_PACKET_ERROR;
		}

		// Check to see if start of packet has been received for a message

		if (qi_receiving_start_bit_message)
		{
			// We have started to receive a packet, but it hasn't been fully received
			// yet. Make sure that not more than Tmax ms have elapsed since start of packet

			if (qi_packet_first_bit_timer_ms > T_MAX_MS)
			{
				// Power off coil and revert to selection phase

				PWM_vStopCoil();

				retState = QI_RESET_S;
				
				reasonReset= RESET_RX_PACKET_RECEVING_TIMEOUT;
			}
		}
	}
	return retState;
}

/****************************************************************************************************/
/*																									*/
/*			WPCQiTx_ProcNegoResponse																*/ 
/*																									*/
/*																									*/
/****************************************************************************************************/
WPCQiTx_State_t WPCQiTx_ProcNegoResponse ( void )
{
    WPCQiTx_State_t retState = QI_NEGO_ID_RESPOND_S;
	uint8_t n_max_power;
	uint8_t max_power;

	if(qi_last_packet_header == WPCQi_FODStatusPacket_H) //0x22
	{
#ifdef Q_MEASURE			
	

		if( coil_sel_num == COIL_SEL_1)
			qi_measured_quality_factor_avg = (uint16_t)q_temp_q2[0][3];

		else  if( coil_sel_num == COIL_SEL_2)
			qi_measured_quality_factor_avg = (uint16_t)q_temp_q2[1][3];
		

		if( coil_sel_num == COIL_SEL_1)
		{
			Q_temp2 = (uint16_t) big_coil_ref_q_th;
		}
		else if( coil_sel_num == COIL_SEL_2)
		{
			Q_temp2 = (uint16_t) 14.0;
		
		}
		debug_qi_state_print("\r\n coil: %d Q Meas:%d q_ref:%d	q_ref_th:%d \r\n",coil_sel_num,qi_measured_quality_factor_avg, qi_ref_quality_factor ,Q_temp2);

#ifdef FOD
		if( qi_measured_quality_factor_avg < Q_temp2 )
		{
			debug_qi_state_print("\r\n coil: %d Bad Q !!!!!!!! \r\n",coil_sel_num);

			PWM_vStopCoil();
			WPCQiTx_setWaitTime(RTC_1P5S_PERIOD_TICKS);
			retState = QI_RESET_S;
			WPCQiTxFskLoopState = FSK_IDLE_S;
		}
		else
#endif				
#endif
		if(counter_ms > T_NEGO_DELAY_MS)
		{

			if((qi_quality_factor_mode == 0) && (qi_fod_status_reserved == 0))
				WPCQiFskResponse = FSK_ACK_RESPONSE;
			else
				WPCQiFskResponse = FSK_ND_RESPONSE;
			WPCQiTxFskLoopState = FSK_WAIT_S;
			counter_ms = 0;
			retState = QI_NEGO_WAIT_ID_S;
		}

	}
	else if(qi_last_packet_header == WPCQi_GeneralRequestPacket_H) //0x07
	{
		if(counter_ms > T_NEGO_DELAY_MS)
		{
			if(qi_tx_last_packet_header == WPCQi_GeneralRequest_TransmitterIdentificationPacket_H)
			{
				WPCQiTxFskLoopState = FSK_WAIT_S;
				WPCQiFskResponse = FSK_IDENTIFICATE_PACKET;
				//debug_qi_state_print("\n\rP:07 30");
			}
			else if(qi_tx_last_packet_header == WPCQi_GeneralRequest_TransmitterCapabilityPacket_H)  
			{
				WPCQiTxFskLoopState = FSK_WAIT_S;
				WPCQiFskResponse = FSK_CAPABILITY_PACKET;
			}
			else if(qi_tx_last_packet_header == 0xFF)  
			{
				WPCQiTxFskLoopState = FSK_WAIT_S;
				WPCQiFskResponse = FSK_NA_RESPONSE;
			}

			counter_ms = 0;
			retState = QI_NEGO_WAIT_ID_S;
		}
	}
	else if(qi_last_packet_header == WPCQi_SpecificRequestPacket_H) //0x20
	{ 
		if(counter_ms > T_IDCFG_RESPOND)
		{
			counter_ms = 0;
			switch(qi_tx_last_packet_header)
			{
				case WPCQi_SpecificRequest_EndNegotiationPacker_H: 
					if(neg_change_count == 0)
					{
						WPCQiTxFskLoopState = FSK_WAIT_S;
						WPCQiFskResponse = FSK_NAK_RESPONSE;
						g_st_power_contract.count = 0;
						retState = QI_NEGO_WAIT_ID_S;
	  				}
	 				 else
//						if(COUNT_BIT_ONE_IN_BYTE[g_st_power_contract.count & 0x0F] == neg_change_count)
					{
						WPCQiTxFskLoopState = FSK_WAIT_S;
						WPCQiFskResponse = FSK_ACK_RESPONSE;
						WPCQiTx_vSetNegoPower();
						retState = QI_CALIBRATION_S;
#if defined(BLE_HANDSHAKING)
						if( qi_oob_flag == 0x01 )
						{
							ECAP0_vStop();
						}
#endif								
						qi_device_charging = 1;

						WPCQiTx_powerTransferInit();
						
						efficiency_init();

						received_endNego = 1;
					}
					break;

				case WPCQi_SpecificRequest_GuaranteedPowerPacket_H: 
					n_max_power = qi_guaranteed_power / 2.0;
					if(qi_guaranteed_reserved != 0x00)
					{
						WPCQiTxFskLoopState = FSK_WAIT_S;
						WPCQiFskResponse = FSK_ND_RESPONSE;
						retState = QI_NEGO_WAIT_ID_S;
						break;
					}

					if(n_max_power > qi_potential_power)
					{
						WPCQiTxFskLoopState = FSK_WAIT_S;
						WPCQiFskResponse = FSK_NAK_RESPONSE;
					}
					else 
					{
						WPCQiTxFskLoopState = FSK_WAIT_S;
						WPCQiFskResponse = FSK_ACK_RESPONSE;					
						if (n_max_power != g_st_power_contract.guaranteeed_power) 
							g_st_power_contract.count |= GUARANTEEED_POWER_CHANGE_FLAG;
						else 
							g_st_power_contract.count &= ~GUARANTEEED_POWER_CHANGE_FLAG;				
					}
					retState = QI_NEGO_WAIT_ID_S;
				    break;
						
				case WPCQi_SpecificRequest_ReceivedPowerPacketTypePacket_H: 

					if(qi_tx_received_power_header == 0x04)
					{
						WPCQiTxFskLoopState = FSK_WAIT_S;
						WPCQiFskResponse = FSK_NAK_RESPONSE;	
					}
					else
					{
						WPCQiTxFskLoopState = FSK_WAIT_S;
						WPCQiFskResponse = FSK_ACK_RESPONSE;					
					}
					retState = QI_NEGO_WAIT_ID_S;
					break;
				
				case WPCQi_SpecificRequest_FSKParameterPacket_H: 
					if(qi_FSKReserved_20 != 0)
					{
						WPCQiTxFskLoopState = FSK_WAIT_S;
						WPCQiFskResponse = FSK_ND_RESPONSE;	
					}
					else
					{
						WPCQiTxFskLoopState = FSK_WAIT_S;
						WPCQiFskResponse = FSK_ACK_RESPONSE;					
					}

					if (qi_tx_received_power_header != g_st_power_contract.modulation_depth)
						g_st_power_contract.count |= MODULATION_DEPTH_CHANGE_FLAG;
					else
						g_st_power_contract.count &= ~MODULATION_DEPTH_CHANGE_FLAG;

					retState = QI_NEGO_WAIT_ID_S;

					break;
						
				case WPCQi_SpecificRequest_MaxiumPowerPacket_H: 
					max_power = (qi_maximum_power >> 1) * POWER_OF_TEN[qi_power_class]; 
					if((qi_maximum_power_reserved != 0) || (max_power >= MAX_POWER)) 
					{
						WPCQiTxFskLoopState = FSK_WAIT_S;
						WPCQiFskResponse = FSK_ND_RESPONSE;	
					}
					else
					{ 
						WPCQiTxFskLoopState = FSK_WAIT_S;
						WPCQiFskResponse = FSK_ACK_RESPONSE;					
						if (n_max_power != g_st_power_contract.max_power)
						{
							g_st_power_contract.count |= MAX_POWER_CHANGE_FLAG;
						} 
						else 
						{
							g_st_power_contract.count &= ~MAX_POWER_CHANGE_FLAG;
						}
					}
					retState = QI_NEGO_WAIT_ID_S;
					break;

				case WPCQi_SpecificRequest_ReservedPacket_H: 
					WPCQiTxFskLoopState = FSK_WAIT_S;
					WPCQiFskResponse = FSK_ND_RESPONSE;	
					retState = QI_NEGO_WAIT_ID_S;
					break;
					
				case WPCQi_SpecificRequest_ProprietaryPacket_H: 
					WPCQiTxFskLoopState = FSK_WAIT_S;
					WPCQiFskResponse = FSK_ND_RESPONSE;	
					retState = QI_NEGO_WAIT_ID_S;
					break;
					
				default:
					break;
						
			}
		}
	}
	else if(qi_last_packet_header == WPCQi_ControlErrorPacet_H)
	{
		qi_device_charging = 1;  
		retState = QI_CALIBRATION_S;
		efficiency_init();
	}
	else if(qi_last_packet_header == WPCQi_EndPowerTransferPacket_H)
	{ 
		if(qi_end_transfer_code == end_tx_reconfigure)
		{
			retState = QI_IDCFG_WAIT_CONFIG_S;
			qi_num_rx_optional_packets = 0;
			counter_ms = 0;
		}
		else if(qi_end_transfer_code != end_tx_renegotiate)
		{
			debug_qi_state_print("\r\n (qi_end_transfer_code != end_tx_renegotiate)");
			PWM_vStopCoil();
			retState = QI_RESET_S;
			reasonReset= RESET_NEGO_SEQUENCE_ERROR;
		}
	}
	else if(qi_last_packet_header == WPCQi_EndPowerTransferPacket_H)
	{
		if(qi_end_transfer_code == 0x0A)
			PWM_vStopCoil();
	}
	else if(qi_last_packet_header == WPCQi_WPID_MostSignificantBitsPacket_H)
	{
		WPCQiTx_FskSendNd();
	}
	else if(qi_last_packet_header == WPCQi_WPID_LeastSignificantBitsPacket_H)
	{
		qi_wpid55_incorrect_checksum = 0;
		if(counter_ms > T_NEGO_DELAY_MS)
		{
			WPCQiTxFskLoopState = FSK_WAIT_S;
			WPCQiFskResponse = FSK_ND_RESPONSE;	

			counter_ms = 0;
			retState = QI_NEGO_WAIT_ID_S;
		}	
	}
	else  
	{
		if(counter_ms > T_NEGO_DELAY_MS)
		{

			WPCQiTxFskLoopState = FSK_WAIT_S;
			WPCQiFskResponse = FSK_ND_RESPONSE;	
			retState = QI_NEGO_WAIT_ID_S;
		}
	}

    return retState;
}


/****************************************************************************************************/
/*																									*/
/*			WPCQiTx_ProcCoilMeasure																*/ 
/*																									*/
/*																									*/
/****************************************************************************************************/
WPCQiTx_State_t WPCQiTx_ProcPowerTransfer ( void )
{


    WPCQiTx_State_t retState = QI_POWER_TRANSFER_S;
	int i,temp,temp_v,ret;
#ifdef OVER_VOLTAGE_CONTROL
	int temp_ov;	
#endif

	control_error_pkt_ms += T_LOOP_TIME_MS;
	received_power_pkt_ms += T_LOOP_TIME_MS;
	modulation_error_pkt_ms += T_LOOP_TIME_MS;

    if( commType == QI_COMMTYPE_COIL && control_error_pkt_ms == 800 && (qi_oob_flag == 0x01) )//KYJ ADD oob flag condition
    {
#if defined(BLE_HANDSHAKING)
		sendMacAddToBle();
#endif
    }
    else 
		if( commType == QI_COMMTYPE_BLE && prev_isBLE == 0 )
	    {
		    prev_isBLE = 1;
	        //debug_qi_state_print("\r\n ====> connected");
	    }

		if (( (received_endNego != 1) && (qi_neg)) || ((commType == QI_COMMTYPE_COIL )&&(modulation_error_pkt_ms > 5000)))			
	    {
	        retState = QI_RESET_S;
	        reasonReset = RESET_NEGO_PROCESS_ERROR;
	        received_endNego = 0;
	        if( commType == QI_COMMTYPE_COIL )debug_qi_state_print("\r\n ====> is not connected");
	        return retState;
	    }

		// Check for 500ms with no CEP and switch modes
		if (modulation_error_pkt_ms > 300)
		{
			advanceModulation();
			modulation_error_pkt_ms = 0;
		}
#if (defined(POWER_NOISY_FILTER) && defined(NEW_FILTER))
//			WPCQiTx_vCountSampleMachine(vsense_digital_counts()); 
#endif
		// Check for timeouts and handle received packet

#if 1	// 1-> 0 keti lim 220117
		if ((control_error_pkt_ms > t_control_error_ms) || (received_power_pkt_ms > t_received_power_ms))
		{
			PWM_vStopCoil();
			debug_qi_state_print("\r\n coil pwm disable num_sequential_timeout_inc	\r\n Prec(23s) : %d cep(1.5s) : %d ",received_power_pkt_ms,control_error_pkt_ms);

			/* Noisy charger filter */
#ifdef POWER_NOISY_FILTER    
#if 0
			if ((receiver_removed ==0) && (num_sequential_timeout_inc<FILTER_MAX_COUNTER)) 
#else
//orig				if (num_comm_errors >= 1 && num_sequential_timeout_inc<FILTER_MAX_COUNTER)
			if (1)
#endif       
			{
				control_error_pkt_ms = received_power_pkt_ms = 0;
				num_comm_errors = 0;
			}
#else
			if ((receiver_removed ==0)) 
			{
				control_error_pkt_ms = received_power_pkt_ms = 0;
				num_comm_errors = 0;
			}
#endif
			//else
			{
				aping_last=1024;

				retState = QI_RESET_S;
				
				if(control_error_pkt_ms > t_control_error_ms) reasonReset = RESET_TRANSEFER_CEP_PACKET_TIMEOUT;
				else reasonReset = RESET_TRANSEFER_POWERPACKET_TIMEOUT;

				qi_device_charging = 0;

				led_state_machine();      
				WPCQiTx_setWaitTime(RTC_1P5S_PERIOD_TICKS);

				return retState;

			}
		}
#endif
		// Collect current into ring buffer

		if (!qi_receiving_start_bit_message)
		{
			temp = i_ring_buffer[i_ring_buffer_index++] = isense_counts();

			if (i_ring_buffer_index == RING_BUFFER_SIZE)
				i_ring_buffer_index = 0;
		}
		
#ifdef OVER_VOLTAGE_CONTROL
		temp_ov = overvolt_counts();
#endif

		// Check to see if we have received a packet

		if (qi_comm_message_received)
		{
			qi_comm_message_received = 0;     

			switch (qi_last_packet_header)
			{
				// Check for valid packet types
        		case WPCQi_ProprietaryOOBInfoPacket_H:
            		control_error_pkt_ms = 0;
            		modulation_error_pkt_ms = 0;
            		break;
					
				case WPCQi_ControlErrorPacet_H:
					// Reset delay counter, to allow power to settle before regualation
					printf("\r\nFreq : %d\r\n", pwmFreq);
					power_xfer_delay_ms = 0;

					// Reset timer for control error packet received

					control_error_pkt_ms = 0;

					// Reset timer for control error packet received since last modulation mode change

					modulation_error_pkt_ms = 0;

					// Set control state machine to wait tdelay
					if(is_control == 1)
						control_loop_state = FSK_WAIT_S;

					++cep_count;

					num_comm_errors = 0;


					t_control_error_ms = T_INTERNAL_MS;

//					if (cep_count == 1) 
					if(1)	//modify by keti lim
					{
//							if ( g_st_power_contract.max_power < EPP_MAX_POWER_15W )
						if ( 1 )
						{
#ifdef VOLTAGE_CONTROL
							if( coil_sel_num == COIL_SEL_2)
							{
								PWM_vInitBuckbooster( TIMERA_TICKS_100KHZ, TIMERA_TICKS_100KHZ*DCDC_PWM_POWER_TRANSFER_15W_INIT/DCDC_PWM_RATIO_200);
								WPCQi_InitVoltageHandleCEP(DCDC_PWM_POWER_TRANSFER_15W_INIT);
							}
							else
							{
								PWM_vInitBuckbooster( TIMERA_TICKS_100KHZ, TIMERA_TICKS_100KHZ*DCDC_PWM_POWER_TRANSFER_INIT/DCDC_PWM_RATIO_200);
								WPCQi_InitVoltageHandleCEP(DCDC_PWM_POWER_TRANSFER_INIT);
							}
								
							if( coil_sel_num == COIL_SEL_2)
								PWM_vInitCoil(TIMERA_DIGITAL_PING_FREQ, TIMERA_DIGITAL_PING_V_CNTRL_DUTY); /* set duty 50% */
							else
								PWM_vInitCoil(pwmFreq, pwmFreq>>1); /* set duty 50% */ //pwmFreq -> modify by keti lim
								
#else
							PWM_vInitCoil(TIMERA_DIGITAL_PING_FREQ, TIMERA_DIGITAL_PING_DUTY   ); /* set duty 50% */
#endif
							set_bridge(full_bridge);
						}
						else
						{
#ifdef VOLTAGE_CONTROL
							WPCQi_InitVoltageHandleCEP(DCDC_PWM_POWER_TRANSFER_15W_INIT);

							PWM_vInitBuckbooster( TIMERA_TICKS_100KHZ, TIMERA_TICKS_100KHZ*DCDC_PWM_POWER_TRANSFER_15W_INIT/DCDC_PWM_RATIO_200);
							PWM_vInitCoil(TIMERA_DIGITAL_PING_FREQ, TIMERA_DIGITAL_PING_V_CNTRL_DUTY); /* set duty 50% */
#else
							PWM_vInitCoil(TIMERA_DIGITAL_PING_FREQ, TIMERA_DIGITAL_PING_DUTY); /* set duty 12.5% */
#endif
							set_bridge(half_bridge);
							debug_qi_state_print("\r\n max power %d set up half bridge  cep %d ",g_st_power_contract.max_power,control_error_pkt_ms );
						}
					}
					is_control = 1;				
				    break;

				case WPCQi_24BitReceivedPowerPacket_H:
					received_power_pkt_ms = 0;
					t_received_power_ms = T_RECEIVED_MS;
					
					printf("\r\nADC val = %d\r\n", qi_received_power);
					if(qi_received_power > 880){// modify by keti lim
						printf("freq -> 145KHz\r\n");
						pwmFreq = TIMERA_TICKS_145KHZ;
					}else{
						printf("freq -> 110KHz\r\n");
						pwmFreq = TIMERA_TICKS_110KHZ;
					}
					// Calculate average current from ring buffer
					temp = 0;
					for (i = 0; i < RING_BUFFER_SIZE; i++)
						temp += i_ring_buffer[i];
					
					temp /= RING_BUFFER_SIZE;
					temp_v = vbus_counts();
#ifdef OVER_VOLTAGE_CONTROL
					if ( g_st_power_contract.max_power > EPP_MAX_POWER_15W )
						temp = temp * MAXIMUM_VOLTAGE_RATIO / 100 + MAXIMUM_VOLTAGE_OFFSET_30W;
					else
						temp = temp * MAXIMUM_VOLTAGE_RATIO / 100 + MAXIMUM_VOLTAGE_OFFSET_15W;
						
					if( temp_ov > 4095 )
					{
						coil_over_voltage_flag = 1;
						debug_qi_state_print("\r\n OVP I:%d  V:%d OV:%d flg:%d",temp,temp_v,temp_ov,coil_over_voltage_flag);
					}
					else
						coil_over_voltage_flag = 0;
#endif
					if(qi_recevied_20 == 1)
					{
						qi_FSKPolarity = qi_FSKPolarity_20;
						fsk_delta = fsk_delta_20;
						qi_recevied_20 = 0;					
					}
					
					if(qi_received_power_mode <= RECEIVED_POWER_CONNECTED_LOAD)
					{
						if((qi_control_error_value > 1) || \
							(qi_control_error_value < -1))
						{
							WPCQiTxFskLoopState = FSK_WAIT_S;
							WPCQiFskResponse = FSK_NAK_RESPONSE;
						}
						else
						{
							WPCQiTxFskLoopState = FSK_WAIT_S;
							WPCQiFskResponse = FSK_ACK_RESPONSE;

						}
					}
					else if(qi_received_power_mode == RECEIVED_POWER_NORMAL_NO_ACK)
					{
						WPCQiTxFskLoopState = FSK_IDLE_S;
                        retState = QI_TXSTATE_GOTO_ERROR_S;
		                return retState;
					}
					else if(qi_received_power_mode != RECEIVED_POWER_NORMAL_NO_ACK)
					{
						WPCQiTxFskLoopState = FSK_WAIT_S;
						WPCQiFskResponse = FSK_ND_RESPONSE;
                        retState = QI_TXSTATE_GOTO_ERROR_S;
		                return retState;
					}

					break;

				case WPCQi_8BitReceivedPowerPacket_H:
					
					if(qi_received_power > 0xF000)
					{
						PWM_vStopCoil();
						retState = QI_RESET_S;
						reasonReset = RESET_TRANSEFER_RXPOWER_ERROR;
						debug_qi_state_print("\r\n higher received power 0xF000");
						return retState;
					}
					// Reset timer for RXPP received 

					received_power_pkt_ms = 0;
					t_received_power_ms = T_RECEIVED_MS;
					// Calculate average current from ring buffer

					temp = 0;
					for (i = 0; i < RING_BUFFER_SIZE; i++)
						temp += i_ring_buffer[i];
					temp /= RING_BUFFER_SIZE;
					temp_v = vbus_counts();

					debug_qi_state_print("\r\n I : %d   V :  %d",temp,temp_v);

					// yy.lee need calibration phase. spec.: 5.1.2.5 calibration phase-epp only
					// mode : 0x00 : response requested 
					// mode : 0x01 : light load,response requested
					// mode : 0x02 : connected-load calibration valure, response requested
					// mode : 0x04 : Normal value no response requested
					if( state == QI_CALIBRATION_S )
					{
						if(( ( qi_received_power_mode == RECEIVED_POWER_LIGHT_LOAD )||(qi_received_power_mode == RECEIVED_POWER_CONNECTED_LOAD))&&(qi_neg == 1))
						{
							if(qi_received_power_mode == RECEIVED_POWER_LIGHT_LOAD)
								ret = power_calibration_phase(temp, temp_v,qi_received_power_mode);
							else if(qi_received_power_mode == RECEIVED_POWER_CONNECTED_LOAD)
								ret = power_calibration_phase(temp, temp_v,qi_received_power_mode);

							if(ret == 0)
							{
								WPCQiTxFskLoopState = FSK_WAIT_S;
								WPCQiFskResponse = FSK_ACK_RESPONSE;
								break;
							}
							else
							{
								WPCQiTxFskLoopState = FSK_WAIT_S;
								WPCQiFskResponse = FSK_NAK_RESPONSE;					
							}
						}
						else if(( qi_received_power_mode == 0x00 )&&(qi_neg == 1))
						{
							WPCQiTxFskLoopState = FSK_WAIT_S;
							WPCQiFskResponse = FSK_ACK_RESPONSE;

							retState = QI_POWER_TRANSFER_S;
							debug_qi_state_print("\r\n enter power transfer");

						}
						else 
						{
								retState = QI_CALIBRATION_S;
						}
					}
					else
				    {		
					 	if(( qi_received_power_mode == 0x00 )&&(qi_neg == 1))
						{
							WPCQiTxFskLoopState = FSK_WAIT_S;
							WPCQiFskResponse = FSK_ACK_RESPONSE;
		      			}
		      			else
						{
							WPCQiTxFskLoopState = FSK_WAIT_S;
							WPCQiFskResponse = FSK_NAK_RESPONSE;		
						}


						ret = efficiency_low(temp, temp_v);
				    }
					break;

				case WPCQi_EndPowerTransferPacket_H:

					// Qi ver 1.2.1 has no "no response" packet.
#ifdef IGNORE_EPT_NO_RESPONSE
					if (qi_end_transfer_code == end_tx_no_response)
					return QI_RESET_S;
#endif
					// If we got a reconfigure message, keep operating point and revert
					// to ID and configuration

					if (qi_end_transfer_code == end_tx_reconfigure)
					{
						retState = QI_IDCFG_WAIT_ID_S;

						qi_num_rx_optional_packets = 0;

						counter_ms = 0;

						return retState;
					}
					else if (qi_end_transfer_code == end_tx_renegotiate)
					{
						retState = QI_NEGO_WAIT_PACKAGE_S;

						counter_ms = 0;

						return retState;
					} 
					//        else if (qi_end_transfer_code == end_tx_restart_power_transfer)
					//        {
					////           state = neg_wait_package;
					////          
					////           counter_ms = 0;
					//          
					////           return;
					//        }         
					else 
					{

						debug_qi_state_print("\r\n coil disable WPCQi_EndPowerTransferPacket_H ");

						PWM_vStopCoil();

						if (qi_end_transfer_code == end_tx_charge_complete)
						{
							++charge_terminate_counter;
							delay_counter = 30000;
							retState = QI_STATE_MACHINE_DELAY_S;
						}
						else if ((qi_end_transfer_code == end_tx_internal_fault) ||
						(qi_end_transfer_code == end_tx_battery_failure) ||
						(qi_end_transfer_code == end_tx_over_current ))
						{
							// delay_counter = 30000;
							//  state = state_machine_delay;
							debug_qi_state_print("\r\n WPCQiTx_WaitPadRemoval");
							WPCQiTx_WaitPadRemoval();

							//  qi_state_machine_latch_off();
						}   
						else
						{
						  if (qi_end_transfer_code == end_tx_no_response)
						  {

							retState = QI_WAIT_PAD_REMOVAL_NO_TIMEOUT_S;
						  }
						  else
						  {
							retState = QI_RESET_S;
							reasonReset = RESET_TRANSEFER_RECEIVE_POWEREND;
						  }
						}

						qi_device_charging = 0;
						qi_device_charged = 1;
						counter_ms = 0;
						led_state_machine();     
						WPCQiTx_setWaitTime(RTC_1P5S_PERIOD_TICKS);
                        
                        retState = QI_TXSTATE_GOTO_ERROR_S;
		                return retState;
					}
					
				case WPCQi_RenegotiatePacket_H:
					retState = QI_IDCFG_RESPOND_S;            
					counter_ms = 0;
					break;
				case WPCQi_ChargeStatusPacket_H:
				case WPCQi_proprietary_1:
				case WPCQi_proprietary_2:
				case WPCQi_proprietary_3:
				case WPCQi_proprietary_4:
				case WPCQi_proprietary_5:
				case WPCQi_proprietary_6:
				case WPCQi_proprietary_7:
				case WPCQi_proprietary_9:
				case WPCQi_proprietary_10:
				case WPCQi_proprietary_11:
				case WPCQi_proprietary_12:
				case WPCQi_proprietary_13:
				case WPCQi_proprietary_14:

					break;

				default:

					// Invalid packet type - reset state machinease

					debug_qi_state_print("\r\n Invalid packet type %d ",qi_last_packet_header);

					PWM_vStopCoil();//*((uint32_t *)0xD0000000)=4;while(1);
					qi_device_charging = 0;
					led_state_machine();      // Force LED state machine to run
					WPCQiTx_setWaitTime(RTC_1P5S_PERIOD_TICKS);
					retState = QI_RESET_S;
					
					reasonReset = RESET_TRANSEFER_INVALID_PACKET_ERROR;

					break;

			}

			counter_ms = 0;
		}

		switch (control_loop_state)
		{
			case FSK_IDLE_S:

				break;

			case FSK_WAIT_S:
				if (power_xfer_delay_ms >= qi_holdoff_time_ms)
				{
					// Run control loop one time with new CEP
					qi_coil_current_f16 = (float)(isense_counts()/*-adc_offset_isense*/) * (float)ADC_GAIN;
					if (qi_coil_current_f16 < 0)
					qi_coil_current_f16 = 0x0;
					
					WPCQi_HandleControlErrorPacket(1);
					control_loop_state = FSK_RUN_S;
					power_xfer_delay_ms = 0;
				}
				else
					power_xfer_delay_ms += T_LOOP_TIME_MS;
				break;

			case FSK_RUN_S:

				power_xfer_delay_ms += T_LOOP_TIME_MS;
				if (power_xfer_delay_ms >= T_ACTIVE_MS)
				{
					control_loop_state = FSK_IDLE_S;
				}
				WPCQi_HandleControlErrorPacket(0);
				break;

	}
    return retState;
}

/****************************************************************************************************/
/*																									*/
/*			WPCQiTx_ProcCoilMeasure																*/ 
/*																									*/
/*																									*/
/****************************************************************************************************/
WPCQiTx_State_t WPCQiTx_ProcFODHandle ( WPCQiTx_State_t state )
{
    WPCQiTx_State_t retState = state;
	static int contiguous_analog_ping_test_fails = 0; 

	switch( state )
	{
		case QI_LATCH_OFF_S:
		case QI_WAIT_PAD_REMOVAL_S:


			if ((counter_ms % 500) == 100)
			{

				if (counter_ms >= T_FOD_TIMEOUT_MS)
				{
					retState = QI_RESET_S;//reset;
					
					reasonReset = RESET_FOD_REMOVE_WAIT_TIMEOUT;
				}
				else if(analog_ping_test(flash_config_ram.analog_ping_threshold ) == 0)
				{
					// If this happened 3 times in a row, go to delay state

					if (++contiguous_analog_ping_test_fails >= 3)
					{
						retState = QI_RESET_S;//reset;
						reasonReset = RESET_FOD_REMOVE_APING_COUNTOVER;

						contiguous_analog_ping_test_fails = 0;
					}
				}
				else
				contiguous_analog_ping_test_fails = 0;

			}

			break;

		case QI_WAIT_PAD_REMOVAL_NO_TIMEOUT_S:

			if (analog_ping_test(flash_config_ram.analog_ping_threshold) == 0)//flash_config_ram.analog_ping_threshold
			{
				// If this happened 3 times in a row, go to delay state

				if (++contiguous_analog_ping_test_fails >= 3)
				{
					retState = QI_RESET_S;//reset;
					
					reasonReset = RESET_FOD_REMOVE_NO_TIME_COUNTOVER;
					contiguous_analog_ping_test_fails = 0;
				}
			}
			else
				contiguous_analog_ping_test_fails = 0;

			WPCQiTx_setWaitTime(RTC_1P5S_PERIOD_TICKS);

			break;

		case QI_STATE_MACHINE_DELAY_S:

			if (counter_ms == delay_counter)
			{
				retState = QI_RESET_S;
				reasonReset = RESET_TX_LOOP_DELAY;

				if (over_temp_flag && (rth_counts() > ADC_AD5_RTH_HYSTERESIS))
					over_temp_flag = 0;
			}

			break;

		case QI_WAIT_SPEAKER_DONE_S://wait_speaker_done:

//			if (!speaker_active)
			if (0)
			{
				retState = state_after_delay;

				counter_ms = 0;

				WPCQiTx_setWaitTime(RTC_1P5S_PERIOD_TICKS);
			}

			break;
	
	}
	return retState;
}

/*----------------------------------------------------------------------------------------------------------*/
/*      																										*/ 	
/*      																										*/
/*      function : WPCQi_Tx																		*/
/*      																										*/
/*         This routine must be called every 4kHz in order to sample the coil voltage  										*/
/* 	        to read messages from the target device.																       */
/*      																										*/
/*      																										*/
/*----------------------------------------------------------------------------------------------------------*/
__ramfunc void WPCQi_Tx(void)
{
#ifdef APING_TEST
	int temp,temp_v,ret;
#endif
	int i;
	
	

	switch (state)
	{

		case QI_RESET_S://reset:
			printf("QiTx Reset 0x%x\r\n", reasonReset);
            reasonReset = RESET_POWER_ON;
            app_flash_config_set_defaults();			

			for (i = 0; i < RING_BUFFER_SIZE; i++)
				i_ring_buffer[i] = 0;  
			counter_ms = 0;                             // Reset ms counter

			PWM_vStopCoil();                         // Just in case, disable coil
			PWM_vStopBuckbooster();
			QiTxBalancedCoil_vselect(FO_BAL_COIL_SEL_NONE);
			qi_comm_message_state_machine_reset();      // Reset message level state machine
			qi_comm_state_machine_reset();              // Reset communications state machines
			WPCQi_InitHandleCEP();                         // Reset PID controller history

			qi_comm_enabled = 0;
			qi_device_charging = 0;
			qi_device_charged = 0;
			qi_foreign_object_found = 0;
			efficiency_low_flag = 0;
			transfer_test_data_error = 0;
			qi_oob_Max_Addr_send_to_ble = 0;
#ifdef Q_MEASURE			
			q_num_cnt = 0;
			q_coil_sel = 0;
#endif
			commType = QI_COMMTYPE_COIL;
			qi_oob_flag = 0;					
			prev_isBLE = 0;
#ifdef APING_TEST
			num_contiguous_a_ping_fail = 0;
#endif
			BLE_EN =0;

			QiTx_vPowerControl(POWER_OFF);

			
#ifdef QI_TX
			reset_loop_counters();
#endif
			control_error_pkt_ms = 0;

			received_power_pkt_ms = 0;

			cep_count = 0;
			
			qi_neg = 0;
			qi_major_version=0;
		    led_state_machine();
			efficiency_reset();
			num_comm_errors = 0;
#ifdef DYNAMIC_DIGITAL_PING_POWER
			reset_freq_machine();
#endif
#if 0
			if(vbus_counts() >=  ADC_VBUS_UV &&  vbus_counts() <=ADC_VBUS_OV )
			{
				voltage_droop_flag=0;        
				// No break - go right into selection
			}
			else
				break;

			if (over_temp_flag && (rth_counts() < ADC_AD5_RTH_HYSTERESIS))
				break;
			else
				over_temp_flag=0;   
#endif			
			WPCQiTx_vInitPower(5, 0);  /* Create default power contract for 5W receiver */

#ifdef Q_MEASURE			
			ECAP1_vStop();
			q_max_value_idx = 0;
			q_min_value = 0;
			q_min_value_idx = 0;
			q_th_value = 0;
			state = QI_MEASURE_S;   
#else
			ECAP0_vStart();
			state = QI_SELECTION_S; 
#endif
            break;  



	
#ifdef Q_MEASURE			
		case QI_MEASURE_S:
            state = WPCQiTx_ProcCoilMeasure();			
            break;  
#endif



#ifdef FO_BALANCED_MEASURE
		case QI_FO_BALANCED_S:
		
		    WPCQiTx_ProcBalanceCoilMeasure();
			ECAP0_vStart();
			state = QI_SELECTION_S;
			counter_ms = 0;
			break;	
#endif		


		case QI_SELECTION_S:
		    state = WPCQiTx_ProcSelections();
			break;


		case QI_PINGWAIT_SIGNAL_STRENGTH_S:

			// See if we have received a valid signal strength packet
			state = WPCQiTx_ProcSignalStrength();
			break;




		case QI_IDCFG_WAIT_ID_S:

            state = WPCQiTx_ProcWaitID();
			break;

		case QI_IDGFG_WAIT_EXID_S:

			state = WPCQiTx_ProcWaitExID();
			break;


		case QI_IDCFG_WAIT_CONFIG_S:

			// See if we have completed receiving a message
			state = WPCQiTx_ProcWaitConfig();
#ifdef CONFIG_EPP
			WPCQiTx_TxState();
#endif
			break;


		case QI_IDCFG_RESPOND_S:
			if(counter_ms > T_IDCFG_RESPOND)
			{
				WPCQiTx_FskSendAck();
				state = QI_NEGO_WAIT_ID_S;
				counter_ms = 0;
			}

			break;


		case QI_NEGO_WAIT_ID_S:
#if defined( BLE_HANDSHAKING )
			if(counter_ms > T_NEGO_TIMEOUT_MS *2)
#else
			if(counter_ms > T_NEGO_TIMEOUT_MS)
#endif
			{
					counter_ms = 0;
					//PWM_vStopCoil();
					debug_qi_state_print("\r\n (counter_ms > T_NEGO_TIMEOUT_MS)");
					//state = QI_RESET_S;
					//reasonReset= RESET_NEGO_MESSAGE_TIMEOUT;
					break;
			}
			
			if(qi_comm_message_received)
			{
				counter_ms = 0;
				qi_comm_message_received = 0;
				state = QI_NEGO_ID_RESPOND_S;
			}
#ifdef CONFIG_EPP
			WPCQiTx_TxState();
#endif
			break;


		case QI_NEGO_ID_RESPOND_S:

	        state = WPCQiTx_ProcNegoResponse();
#ifdef CONFIG_EPP
			WPCQiTx_TxState();
#endif
			break;


	    case QI_CALIBRATION_S:
			transfer_test_data_error = 1;
			printf("  enter PT state\r\n");
			state = QI_CALIBRATION_S;

		case QI_POWER_TRANSFER_S:
	        state = WPCQiTx_ProcPowerTransfer();
#ifdef CONFIG_EPP
			WPCQiTx_TxState();
#endif
			break;


		case QI_LATCH_OFF_S:

			//   break;

		case QI_WAIT_PAD_REMOVAL_S:

			// Wait 0.5 second, then do analog ping
			state = WPCQiTx_ProcFODHandle(state);

			break;

		case QI_WAIT_PAD_REMOVAL_NO_TIMEOUT_S:
			state = WPCQiTx_ProcFODHandle(state);
			break;

		case QI_STATE_MACHINE_DELAY_S:
			state = WPCQiTx_ProcFODHandle(state);
			break;

		case QI_WAIT_SPEAKER_DONE_S:
			state = WPCQiTx_ProcFODHandle(state);
			break;
			
		case QI_TXSTATE_GOTO_ERROR_S:
			
			goto state_end;
		
			

	}

	state_end:

	// See if we have terminated charge three times in a row, to 
	// latch off until pad removal

	if (charge_terminate_counter >= 3)
	{
		charge_terminate_counter = 0;

		PWM_vStopCoil();//*((uint32_t *)0xD0000000)=5;while(1);

		WPCQiTx_WaitPadRemovalLatch();

		counter_ms = 0; 
	}

	// Increment timer by 1

	counter_ms += T_LOOP_TIME_MS;
	qi_packet_first_bit_timer_ms += T_LOOP_TIME_MS;

	// Only increment last bit timer if we are not receiving a message
	if (!qi_receiving_start_bit_message)
		qi_packet_last_bit_timer_ms += T_LOOP_TIME_MS;
}



