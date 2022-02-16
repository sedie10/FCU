#include <stdio.h>
#include "NuMicro.h"
#include "WPCQiTx.h"
#include "modulation.h"
#include "Ask.h"
#include "Fsk.h"

static WPCQiTX_AskDecodeState_t AskDecodeState = ASKDECODE_PREAMBLE_HUNT_EDGE_S;                /*!< Bit-level state machine state */
static uint8_t qi_bit_num_consecutive_ones = 0;                               	/*!< Number of consecutive ones when hunting for start bit */
static uint8_t qi_bit_num_half_ones = 0;                                      	/*!< Number of half-period transitions for ones when hunting for start bit */
static uint8_t qi_bit_num_unsure_zero = 0;                                      /*!< Number of half-period transitions for ones when hunting for start bit */
static WPCQiTX_AskBitDecodeState_t askBitDecodeState = ASKBITDECODE_START_S;                  /*!< Byte-level state machine state */
static uint8_t qi_byte_message_byte_count = 0;                                	/*!< Number of received bytes */
static uint8_t qi_byte_bit_position = 0;                                      	/*!< Bit position when shifting in data byte */
static uint8_t qi_byte_num_ones = 0;                                          	/*!< Number of ones in current byte being received */
static uint8_t qi_byte_rx_byte = 0;                                       		/*!< Value of current received byte */
static uint8_t temp_qi_byte_rx_byte = 0;																			/*!< Value of current received byte */
int num_comm_errors = 0;                                                  		/*!< Number of comm errors while processing communications */
int num_start_bit = 0; 
static uint8_t qi_bit_glitch_flag = 0;  
static int32_t prev_period_delta = 0;
int byte_hunting_stop_cnt;
#define PREAMBLE_DEBUG
#ifdef ASK_COMM_BIT_DEBUG
uint16_t period_a[201]; 
int16_t prev_period_dlt[201]; 
uint8_t period_a_cnt = 0;
#endif
uint8_t data_tmp;
uint8_t rx_message_buf[30] = {0};
extern uint8_t transfer_test_data_error;
uint8_t qi_rx_data_buf[100] ={0}; 
extern uint8_t qi_rx_header;
uint8_t qi_rx_buffer[27];
uint8_t qi_rx_header;

extern WPCQiTx_State_t state;
uint8_t qi_end_transfer_code = 0;
uint8_t qi_signal_strength = 0;                   /*!< unscaled signal strength */
signed char qi_control_error_value = 0;                  /*!< unscaled control error value */
uint32_t qi_received_power = 0;                    /*!< unscaled received power */
uint8_t qi_charge_status = 0;                     /*!< charge status */
uint8_t qi_holdoff_time_ms = T_MIN_DELAY_MS;      /*!< unscaled holdoff time in ms */
uint8_t qi_power_class = 1;                       /*!< power class */
uint8_t qi_maximum_power_reserved = 0;
uint8_t qi_maximum_power = 3;                     /*!< maximum power */
uint8_t qi_prop = 0;                              /*!< power transfer property */
uint8_t qi_oob_flag = 0;                              /*!< power transfer property */
uint8_t qi_count = 0;                             /*!< number of optional configuration packets */
uint8_t qi_window_size = 0;                       /*!< number of 4ms slots to average power */
uint8_t qi_window_offset = 0;                     /*!< interval between averaging window and received power packet, in number of 4ms slots */
uint8_t qi_neg=0;//add
uint8_t qi_FSKPolarity = 0;
uint8_t qi_FSKDepth = 0;
uint8_t qi_FSKReserved = 0;
uint8_t qi_FSKPolarity_20 = 0;
uint8_t qi_FSKDepth_20 = 0;
uint8_t qi_FSKReserved_20 = 0;
uint8_t qi_recevied_20 = 0;
uint8_t qi_oob = 0;
uint8_t qi_oobMacAddr[6] = {0x00,};
uint8_t qi_received_power_mode = 0;
uint8_t qi_guaranteed_class=1;
uint8_t qi_guaranteed_reserved= 0;
uint8_t qi_guaranteed_power=10;
uint8_t qi_potential_class=1;
uint8_t qi_potential_power= QI_POTENTIAL_POWER;
uint8_t qi_major_version = 0;                     /*!< major version of the wireless power specification complies */
uint8_t qi_minor_version = 0;                     /*!< minor version of the wireless power specification complies */
uint16_t qi_manufacturer_code = 0;                /*!< manufacturer ID */
uint8_t qi_ext = 0;                               /*!< set to 1 if using extended device identifier to identify power receiver */
uint32_t qi_basic_device_identifier = 0;          /*!< part of device identification */
uint8_t qi_extended_device_identifier[8];         /*!< optional part of device identification */


int qi_packet_first_bit_timer_ms = 0;             /*!< Number of ms from first bit of current received message */
int qi_receiving_start_bit_message;                         /*!< Set to non-zero when qi comm state machine is receiving a message */
int qi_comm_message_received;                     /*!< Set to non-zero if correctly formatted packet received */
uint8_t neg_change_count=0;
uint8_t qi_ref_quality_factor;
uint8_t qi_quality_factor_mode;
uint8_t qi_fod_status_reserved;
uint8_t qi_last_packet_header;                    /*!< last header byte of the last packet received */
uint8_t qi_tx_last_packet_header; 
uint8_t qi_tx_received_power_header=WPCQi_8BitReceivedPowerPacket_H;
uint8_t qi_message_length;                        /*!< Number of bytes in message payload after last header byte received */
const uint8_t G_QI_FSK_DELTA[4] = {4, 7, 13, 25}; 
static int rx_message_byte_count = 0;
uint8_t wait_ble_connection = 0;
WPCQiTX_AskByteDecodeState_t askByteDecodeState = ASKBYTEDECODE_START_S;


/* Function Prototypes */
static void ASK_vBitDecode(uint8_t BitValue);                        		 	/*!< Byte state machine */
static int ASK_vPacketDecode(void); 
__ramfunc void ASK_vByteDecode(uint8_t Byte_Data);     						/*!< qi Message state machine */
extern void qi_comm_message_state_machine_reset(void);                    /*!< Reset qi Message state machine */

/*----------------------------------------------------------------------------------------------------------*/
/*      																										*/ 	
/*      																										*/
/*      function : qi_comm_state_machine_reset																					*/
/*       																										*/
/* 																											*/
/*----------------------------------------------------------------------------------------------------------*/
void qi_comm_state_machine_reset(void)
{
  
  AskDecodeState = ASKDECODE_PREAMBLE_HUNT_EDGE_S;
  askBitDecodeState = ASKBITDECODE_START_S;

  qi_bit_num_consecutive_ones = 0;
  qi_bit_num_half_ones = 0;
  qi_byte_bit_position = 0;
  qi_byte_num_ones = 0;
  qi_byte_rx_byte = 0;
  temp_qi_byte_rx_byte =0;
  qi_byte_message_byte_count = 0; 
  qi_receiving_start_bit_message = 0;
  qi_comm_message_received = 0;
  qi_packet_last_bit_timer_ms = 0;
  prev_period_delta = 0;
  collecting_dynamic_hysteresis = 0;
  qi_bit_num_unsure_zero = 0;  
  lp_filter_hys = resetHysteresis();
}
/*----------------------------------------------------------------------------------------------------------*/
/*      																										*/ 	
/*      																										*/
/*      function : qi_comm_message_init																					*/
/*       																										*/
/* 																											*/
/*----------------------------------------------------------------------------------------------------------*/

void qi_comm_message_init( void)
{
    uint8_t i;
	
	qi_rx_header = 0x0;
	for( i = 0; i<27; i++ )
		qi_rx_buffer[i] = 0x0;	

	rx_message_byte_count = 0;
}

/*----------------------------------------------------------------------------------------------------------*/
/*      																										*/ 	
/*      																										*/
/*      function : qi_comm_message_state_machine_reset																					*/
/*       																										*/
/* 																											*/
/*----------------------------------------------------------------------------------------------------------*/

void qi_comm_message_state_machine_reset(void)
{
  askByteDecodeState = ASKBYTEDECODE_START_S;
  qi_packet_first_bit_timer_ms = 0;
  qi_packet_last_bit_timer_ms = 0;
  
  qi_device_charged = 0;
  qi_guaranteed_power = 10;
  
  qi_comm_message_received = 0;
} 



/****************************************************************************************************/
/*																									*/
/*						ASK_vDecode																	*/ 
/*																									*/
/*	Detected edge transition. There are three cases:															*/
/*	Logic 0: 400us <= qi_bit_edge_transition_counts <= 550us (15 - 22 counts)									*/
/*	Logic 1: 175 <= qi_bit_edge_transition_counts <= 325us (7 - 13 counts)										*/
/*	 Noise:   other																						*/
/****************************************************************************************************/
void ASK_vDecode( uint32_t period )
{
	static int logic_level_prev = 0;
	static int logic_level = 0;
	static int32_t period_delta = 0;
#ifdef ASK_COMM_BIT_DEBUG
	static uint8_t jj = 0;
#endif
	// 
	// Low-level COMM protocol
	//


	if (logic_level_prev == 0)
		logic_level = 1;
	else
		logic_level = 0;

	switch (AskDecodeState)
	{

		case ASKDECODE_PREAMBLE_HUNT_EDGE_S:

			if (logic_level != logic_level_prev)
			{       

#ifdef ASK_COMM_BIT_DEBUG
				if(jj > 9 ) jj = 0;
				period_a[jj] = period;
				prev_period_dlt[jj++] =prev_period_delta;
#endif

	      		if ((period > FULL_500U_PERIOD_VALUE_LOW) && (period <= FULL_500U_PERIOD_VALUE_UP)) // 250us - 500us
				{
						// Got zero - check to see if previous consecutive ones are the right
						// number to declare a start bit

#ifdef PREAMBLE_DEBUG
					if ( period < FULL_500U_PERIOD_VALUE_LOW_15P )
					{
						qi_bit_num_unsure_zero = 1; 	
						period_delta = period - HALF_250U_PERIOD_VALUE;
					}
					else
#endif
					{
					    if (qi_bit_num_consecutive_ones >= 4)//4 
					    {
							// Got start bit
							AskDecodeState = ASKDECODE_DATA_HUNT_EDGE_S;         

							period_delta = period - FULL_500U_PERIOD_VALUE;

							qi_bit_glitch_flag = 0; 

							logic_level_prev = logic_level;

							qi_receiving_start_bit_message = 1;

							qi_packet_first_bit_timer_ms = 0;

							qi_bit_num_consecutive_ones = 0;
							qi_bit_num_half_ones = 0;

							askBitDecodeState = ASKBITDECODE_START_S;  //yy.lee debug
#ifdef PREAMBLE_DEBUG
							if(( qi_bit_num_unsure_zero == 1 ) &&( ( period + period_delta ) > FULL_500U_PERIOD_VALUE_UP))
							{
								qi_bit_num_unsure_zero = 0;
								ASK_vBitDecode(0); 
							}
							qi_bit_num_unsure_zero = 0;
#endif
							ASK_vBitDecode(0);
#ifdef ASK_COMM_BIT_DEBUG
							period_a[10] = jj;
							jj =11;
							for ( jj = 11 ; jj< 201 ;jj++)
							    period_a[jj] = 0;
							period_a_cnt = 11;
#endif
							return;
						}
				     }
						// Got zero, but not enough ones preceding it to be a start bit

				     qi_bit_num_consecutive_ones = 0;
				     qi_bit_num_half_ones = 0;
	      
	      	      }
#ifdef PREAMBLE_DEBUG
			      else if ((period < HALF_250U_PERIOD_VALUE_LOW) && (( qi_bit_num_unsure_zero == 1 )))
			      {
				     qi_bit_num_unsure_zero = 0;
	                 ++qi_bit_num_consecutive_ones;
	                 qi_bit_num_half_ones = 0;
			      }
#endif
	              else if ((period > HALF_250U_PERIOD_VALUE_LOW) && (period <= HALF_250U_PERIOD_VALUE_UP)) // 125us - 250us  // + 4를 합.   7 ~ 14
	              { 
			 	      if (++qi_bit_num_half_ones == 2)
	                  {
	                    // Log this as a "1"
	                       ++qi_bit_num_consecutive_ones;
	                       qi_bit_num_half_ones = 0;
	                  }
	              }
	              else
	              {
	                  if (qi_bit_num_consecutive_ones >= 4)//4   normal // ????
	                  {
	                     ++num_comm_errors;
				      }
	                  // Noise - not zero or one
	                  qi_bit_num_consecutive_ones = 0;
	                  qi_bit_num_half_ones = 0;
	              }
      
                  AskDecodeState = ASKDECODE_PREAMBLE_HUNT_EDGE_S;

            }
            logic_level_prev = logic_level;    
            break;

	
        case ASKDECODE_DATA_HUNT_EDGE_S:
    
#ifdef ASK_COMM_BIT_DEBUG
			period_a[period_a_cnt] = period;
			prev_period_dlt[period_a_cnt] =prev_period_delta;
		
			period_a_cnt++;
			if(period_a_cnt > 200)	period_a_cnt = 200;
#endif    

		    if (logic_level != logic_level_prev)
    		{
				logic_level_prev = logic_level;
				if( ( askBitDecodeState == ASKBITDECODE_STOP_S) && (qi_bit_num_half_ones == 1) )
				{
						qi_bit_num_half_ones = 0;
						ASK_vBitDecode(1);
						return;
				}
      
			if ( ( period <= HALF_250U_PERIOD_VALUE_LOW )/*&& (askBitDecodeState != ASKBITDECODE_STOP_S)*/)
			{
				if ( qi_bit_glitch_flag == 0 )
				{
					if (((( period_delta + period ) - ( HALF_250U_PERIOD_VALUE_LOW) > 0) ) && (qi_bit_num_half_ones == 0))
					{
						period_delta = period_delta + period;
						qi_bit_glitch_flag = 1; 
								
						qi_bit_num_unsure_zero = 0;
					}
					else if ((( period_delta + period ) > HALF_250U_PERIOD_VALUE_LOW ) && (qi_bit_num_half_ones == 1))
					{
						qi_bit_num_half_ones = 0;									
						qi_bit_num_unsure_zero = 0;
						ASK_vBitDecode(1);
					}
					else
					if ((( period_delta + period ) > HALF_250U_PERIOD_VALUE_LOW ) && (qi_bit_num_unsure_zero == 1))
					{
						qi_bit_num_unsure_zero = 0;
						qi_bit_num_half_ones = 0;									
						ASK_vBitDecode(1);
					}
					else				
						qi_bit_glitch_flag = 1; 
				}
				else
				{
					qi_bit_glitch_flag = 0; 
					period_delta = period_delta + period;
				}
			}
			else
				if ((period > HALF_250U_PERIOD_VALUE_LOW ) && (period <= HALF_250U_PERIOD_VALUE_UP)) // 125us - 250us  
				{
		        // logic 1
					if( period_delta < (-HALF_250U_PERIOD_VALUE) )
					{
						period_delta = period + period_delta;
						qi_bit_glitch_flag = 1;
						if(qi_bit_num_unsure_zero == 1)
						{
							qi_bit_num_unsure_zero = 0;
							ASK_vBitDecode(0);
						}
						return;
					}
					period_delta = period - HALF_250U_PERIOD_VALUE;
		         
					if ((qi_bit_num_unsure_zero == 0) && (askBitDecodeState == ASKBITDECODE_START_S))
					{
						period_delta = period - FULL_500U_PERIOD_VALUE;
						ASK_vBitDecode(0);
						return;
					}
					else if ((qi_bit_num_unsure_zero == 1) && (askBitDecodeState == ASKBITDECODE_START_S))
					{
						ASK_vBitDecode(0);
						qi_bit_num_unsure_zero = 0;
						qi_bit_num_half_ones = 1;
						return;
					}
					else if (qi_bit_num_unsure_zero == 1)
					{
						if( period_delta < 0 )
						{
							ASK_vBitDecode(1);
							qi_bit_num_unsure_zero = 0;
						}
						else
						{
							ASK_vBitDecode(0);
							qi_bit_num_unsure_zero = 0;
							qi_bit_num_half_ones = 1;
						}
						return;
					}
					else
					if (qi_bit_num_half_ones == 1)
					{
						qi_bit_num_half_ones = 0;
						ASK_vBitDecode(1);
						return;
					}
					else
					{
					  qi_bit_num_half_ones = 1;
					}
		        
		      }
		      else if((period > FULL_500U_PERIOD_VALUE_LOW) && (period <= FULL_500U_PERIOD_VALUE_UP))
		      {
		// logic 0
				if (period_delta < (-HALF_250U_PERIOD_VALUE))
				{
					prev_period_delta = period_delta;
					period_delta = period + period_delta - FULL_500U_PERIOD_VALUE;
					qi_bit_num_unsure_zero = 1;	
					return;
				}
				else if ((period_delta < ( FULL_500U_PERIOD_VALUE - FULL_500U_PERIOD_VALUE_LOW_15P) ) && (qi_bit_num_unsure_zero == 1))
				{
					prev_period_delta = period_delta;
					period_delta = period + period_delta - HALF_250U_PERIOD_VALUE;
					qi_bit_num_unsure_zero = 0;
					ASK_vBitDecode(0);
					if( period_delta < 0 )
						qi_bit_num_half_ones = 1;	
					else
					{
						period_delta = period_delta - HALF_250U_PERIOD_VALUE;
						if( (period_delta > -GLITCH_150U_PERIOD_VALUE)  && (period_delta  < GLITCH_150U_PERIOD_VALUE))
						{
							period_delta = 0;
							ASK_vBitDecode(0);
						}
						else
							qi_bit_num_unsure_zero = 1;
					}
					return;
				}

				period_delta = period - FULL_500U_PERIOD_VALUE;
				if ( ( period > FULL_500U_PERIOD_VALUE_LOW_15P) && (qi_bit_num_half_ones == 1))
				{
					prev_period_delta = period_delta;
					period_delta = period - period_delta -FULL_500U_PERIOD_VALUE - HALF_250U_PERIOD_VALUE;
					qi_bit_num_half_ones = 0 ;
					if(( qi_byte_message_byte_count == 0 ) && (qi_byte_bit_position < 2))
						ASK_vBitDecode(1);
					else
						ASK_vBitDecode(0);
					ASK_vBitDecode(0);
					return;
				}
				else if ( ( period < FULL_500U_PERIOD_VALUE_LOW_15P ) && (qi_bit_num_half_ones == 0))
				{
					qi_bit_num_unsure_zero = 1;		
					period_delta = period - FULL_500U_PERIOD_VALUE;
					return;
				}
				else if ( ( period < FULL_500U_PERIOD_VALUE_LOW_15P ) && (qi_bit_num_half_ones == 1))
				{
					prev_period_delta = period_delta;
					period_delta = period - HALF_250U_PERIOD_VALUE;
					qi_bit_num_half_ones = 0 ;
					ASK_vBitDecode(1);
					return;
				}
		        else
		        {
					qi_bit_num_half_ones = 0;
					if (qi_bit_num_unsure_zero == 1)
					{
						ASK_vBitDecode(0);
						qi_bit_num_unsure_zero = 0;
					}
					ASK_vBitDecode(0);
					return;
		        }
		      }
     
			  else if ( ( period > FULL_500U_PERIOD_VALUE_UP) && ( period < ( FULL_500U_PERIOD_VALUE + HALF_250U_PERIOD_VALUE_UP) ))
			  {
					prev_period_delta = period_delta;
					if(qi_bit_num_half_ones == 1 )
					{
						period_delta = period - period_delta  - FULL_500U_PERIOD_VALUE - HALF_250U_PERIOD_VALUE;
						qi_bit_num_half_ones = 0;
						ASK_vBitDecode(0);

						ASK_vBitDecode(0);
			  			qi_bit_num_consecutive_ones = 0;
					}
					else
					{
						period_delta = period - FULL_500U_PERIOD_VALUE * 2;
						ASK_vBitDecode(1);
						qi_bit_num_unsure_zero = 0;
					
						ASK_vBitDecode(0);

			  			qi_bit_num_consecutive_ones = 0;
						qi_bit_num_half_ones = 0;
					}
		      }
			  else if ( period > ( FULL_500U_PERIOD_VALUE_UP + FULL_500U_PERIOD_VALUE) )
			  {
					if(askBitDecodeState == ASKBITDECODE_STOP_S)
					{
						qi_bit_num_half_ones = 0;
						ASK_vBitDecode(1);

					}
					else
					{
#ifdef ASK_COMM_BIT_DEBUG
						debug_ask_print("\r\nASK	bit time over time:%d, qi_byte_state :%d qi_bit_num_half_ones %d",period,askBitDecodeState,qi_bit_num_half_ones);
#endif
						AskDecodeState = ASKDECODE_PREAMBLE_HUNT_EDGE_S;
						qi_bit_num_consecutive_ones = 0;
						qi_bit_num_half_ones = 0;
						qi_comm_state_machine_reset();
						qi_comm_message_state_machine_reset();
					}
		    }
			else
			{
				AskDecodeState = ASKDECODE_PREAMBLE_HUNT_EDGE_S;
				qi_bit_num_consecutive_ones = 0;
				qi_bit_num_half_ones = 0;
				qi_comm_state_machine_reset();
				qi_comm_message_state_machine_reset();
			}
    }
    break;
  }
}

/****************************************************************************************************/
/*																									*/
/*						ASK_vDecode 															*/ 
/*																									*/
/*																									*/
/****************************************************************************************************/
__ramfunc void ASK_vBitDecode( uint8_t BitValue)
{
	static int num_message_bytes = 0;
#if (defined ( ASK_COMM_BIT_DEBUG ))
	uint8_t jj = 0;
#endif
	

	switch (askBitDecodeState)  
	{

		case ASKBITDECODE_START_S:
			// Start bit has to be a 0
    		if (BitValue == 0)
			{
				askBitDecodeState = ASKBITDECODE_DATA_S;

				qi_byte_num_ones = 0;
				qi_byte_rx_byte = 0;
				temp_qi_byte_rx_byte =0;
				qi_byte_bit_position = 0;
			}
			else
			{
  			++num_comm_errors;
				debug_ask_print("\r\nASK byte hunting startbit err received 1  ");
#ifdef ASK_COMM_BIT_DEBUG
				debug_ask_bit_print("\r\n ASK bit time");
				for(jj =0 ; jj < qi_byte_message_byte_count ; jj++)
				{
					CLK_SysTickDelay(15);
					debug_ask_bit_print("  %d : 0x%x",jj,qi_rx_data_buf[jj]);
				}

				for( jj =0 ; jj < 201 ;jj++ )
				{
					CLK_SysTickDelay(25);
					debug_ask_bit_print("\r\n%d  %d",period_a[jj],prev_period_dlt[jj]);
					if(period_a[jj] == 0) break;
				}
#endif
				qi_comm_state_machine_reset();
				qi_comm_message_state_machine_reset();

			}

			break;

		case ASKBITDECODE_DATA_S:
			// Keep track of number of ones for parity calculation later
			// and shift bits into data byte	

		    if (BitValue)
			{
				qi_byte_num_ones++;
				qi_byte_rx_byte |= (1L << qi_byte_bit_position);
			}
			
			qi_byte_bit_position++;

			// Check if we are done receiving data

			if (qi_byte_bit_position == 8)
			{
				askBitDecodeState = ASKBITDECODE_PARITY_S;

				qi_byte_bit_position = 0;
#ifdef ASK_COMM_BIT_DEBUG
				qi_rx_data_buf[qi_byte_message_byte_count]=qi_byte_rx_byte;
#endif
			}

			break;

		case ASKBITDECODE_PARITY_S:
			
			// Odd parity: even number of ones: parity == 1
			if (( qi_byte_num_ones & 1) == (BitValue & 1))
			{
#ifdef COMM_BIT_DEBUG
				if (debug_tracking.ctr_num_msg_parity_error > 10 )
				{
					num_comm_errors = num_comm_errors;	  
					period_a[period_a_cnt] = 0xff;
				}
#endif	
				if(transfer_test_data_error == 1)
				{
					debug_ask_print("\r\nASK parity bit error D : 0x%x 0x%x  cnt :0x%x p:0x%x  ",qi_byte_rx_byte, temp_qi_byte_rx_byte, qi_byte_message_byte_count,BitValue);
#ifdef ASK_COMM_BIT_DEBUG
					debug_ask_bit_print("\r\n ASK bit time");
					for(jj =0 ; jj < qi_byte_message_byte_count ; jj++)
					{
						CLK_SysTickDelay(15);
						debug_ask_bit_print("  %d : 0x%x",jj,qi_rx_data_buf[jj]);
					}

					for( jj =0 ; jj < 201 ;jj++ )
					{
						CLK_SysTickDelay(25);
						debug_ask_bit_print("\r\n%d  %d",period_a[jj],prev_period_dlt[jj]);
						if(period_a[jj] == 0) break;
					}
					jj = 0;
#endif
				}
				
				qi_comm_state_machine_reset();
				qi_comm_message_state_machine_reset();
				++num_comm_errors;
			}
			else
			{
				askBitDecodeState = ASKBITDECODE_STOP_S;
				byte_hunting_stop_cnt = 0;
			}
			break;

		case ASKBITDECODE_STOP_S:
			if (BitValue != 1)
			{
					debug_ask_print("\r\nASK stop bit error D : 0x%x 0x%x  num :0x%x p:0x%x  ",qi_byte_rx_byte, temp_qi_byte_rx_byte, qi_byte_num_ones,BitValue);
#ifdef ASK_COMM_BIT_DEBUG
					debug_ask_bit_print("\r\n ASK bit time");
					for(jj =0 ; jj < qi_byte_message_byte_count ; jj++)
					{
						CLK_SysTickDelay(15);
						debug_ask_bit_print("  %d : 0x%x",jj,qi_rx_data_buf[jj]);
					}

					for( jj =0 ; jj < 201 ;jj++ )
					{
						CLK_SysTickDelay(25);
						debug_ask_bit_print("\r\n%d  %d",period_a[jj],prev_period_dlt[jj]);
						if(period_a[jj] == 0) break;
					}
					jj = 0;
#endif
				++num_comm_errors;
				qi_comm_state_machine_reset();
				qi_comm_message_state_machine_reset();
			}
			else
			{
				if (qi_byte_message_byte_count == 0) // header byte
				{
					
				
					if (qi_byte_rx_byte <= 0x1F)                  // 0x00 <= data <= 0x1F
						num_message_bytes = 1;
					else if (qi_byte_rx_byte <= 0x7F)             // 0x20 < data <= 0x7F
						num_message_bytes = 2 + ((qi_byte_rx_byte - 32) >> 4);
					else if (qi_byte_rx_byte <= 0xDF)             // 0x80 < data <= 0xDF
						num_message_bytes = 8 + ((qi_byte_rx_byte - 128) >> 3);
					else                                          // 0xE0 < data <= 0xFF
						num_message_bytes = 20 + ((qi_byte_rx_byte - 224) >> 2);

					askBitDecodeState = ASKBITDECODE_START_S;

					qi_byte_message_byte_count++;

					num_message_bytes++; // compensate for checksum byte

					qi_message_length = num_message_bytes;

				}
				else
				{

					if (qi_byte_message_byte_count == num_message_bytes) // message payload + checksum
					{
						// Note: can't call reset function, since byte count will be reset....
						qi_byte_message_byte_count = 0;

						askBitDecodeState = ASKBITDECODE_START_S;

						AskDecodeState = ASKDECODE_PREAMBLE_HUNT_EDGE_S;
						
						prev_period_delta = 0;

						qi_packet_last_bit_timer_ms = 0;

						qi_bit_num_consecutive_ones = 0;
						qi_bit_num_half_ones = 0;

						qi_receiving_start_bit_message = 0;

					}
					else
					{
						askBitDecodeState = ASKBITDECODE_START_S;

						qi_byte_message_byte_count++;

					}

				}

				ASK_vByteDecode(qi_byte_rx_byte);

			}

			break;

	}

}
/****************************************************************************************************/
/*																									*/
/*						ASK_vByteDecode 															       */ 
/*																									*/
/*																									*/
/****************************************************************************************************/

__ramfunc void ASK_vByteDecode(uint8_t Byte_Data)
{ 
//	static int rx_message_byte_count = 0;
	int i;
	uint8_t checksum = 0; 
    
	switch (askByteDecodeState)
	{

		case ASKBYTEDECODE_START_S:
    
            qi_rx_header = Byte_Data;
            qi_last_packet_header = Byte_Data;

			askByteDecodeState = ASKBYTEDECODE_DATA_S;

			rx_message_byte_count = 0;

			break;

		case ASKBYTEDECODE_DATA_S:

			qi_rx_buffer[rx_message_byte_count++] = Byte_Data;

			if (rx_message_byte_count == (qi_message_length - 1)) // length includes checksum
				askByteDecodeState = ASKBYTEDECODE_CHECKSUM_S;

			break;

		case ASKBYTEDECODE_CHECKSUM_S:

			checksum = qi_rx_header;

			for (i = 0; i < (qi_message_length-1); i++){
				checksum ^= qi_rx_buffer[i];
			}
			
    		if (checksum != Byte_Data)
			{
			}
			else if (ASK_vPacketDecode() != 0)
			{
			}
			else
			{
				qi_last_packet_header = qi_rx_header;
				qi_comm_message_received = 1;
				
				printf("C: %02X ",qi_rx_header );
				for (i = 0; i < (qi_message_length-1); i++){
					printf("%02X ", qi_rx_buffer[i]);
				}
				printf("\n\r");
			}
			// Reset parameters

			rx_message_byte_count = 0;

			// Reset the last bit timer

			qi_packet_last_bit_timer_ms = 0;

			// Reset state machine

			askByteDecodeState = ASKBYTEDECODE_START_S;

            qi_comm_message_init(); //KYJ TEST
			break;
	}
}

/****************************************************************************************************/
/*																									*/
/*						ASK_vPacketDecode 															       */ 
/*																									*/
/*																									*/
/****************************************************************************************************/
static inline int ASK_vPacketDecode(void)
{
  int i;

  switch (qi_rx_header)
  {
  
  case WPCQi_SignalStrengthPacket_H:
    
    qi_signal_strength = qi_rx_buffer[0];
   
    break;
    
  case WPCQi_EndPowerTransferPacket_H:
  
    qi_end_transfer_code = qi_rx_buffer[0];
    
    break;
    
  case WPCQi_PowerContronHoldOffPacket_H:
    
    qi_holdoff_time_ms = qi_rx_buffer[0];
    
    break;
    
  case WPCQi_ConfigurationPacket_H:
    qi_power_class = qi_rx_buffer[0] >> 6;
    qi_maximum_power = qi_rx_buffer[0] & 0x3F; // 수정 필요.
	qi_oob_flag = (qi_rx_buffer[2] >> 4 ) & 0x01; 
    qi_prop = qi_rx_buffer[2] >> 7;
    qi_count = qi_rx_buffer[2] & 0x07;
    qi_window_size = qi_rx_buffer[3] >> 3;
    qi_window_offset = qi_rx_buffer[3] & 0x07;    
    qi_neg=qi_rx_buffer[4]>>7;
    qi_FSKPolarity=(qi_rx_buffer[4]>>6) & 0x01;
    qi_FSKDepth=((qi_rx_buffer[4]>>4) & 0x03); 
    fsk_delta= G_QI_FSK_DELTA[qi_FSKDepth];
    break;

  case WPCQi_IdetificationPacket_H:
    qi_major_version = qi_rx_buffer[0] >> 4;
    qi_minor_version = qi_rx_buffer[0] & 0x0F;
    qi_manufacturer_code = ((uint16_t)qi_rx_buffer[1] << 8) | (uint16_t)qi_rx_buffer[2];
    qi_ext = qi_rx_buffer[3] >> 7;
    qi_basic_device_identifier = ((uint32_t)((qi_rx_buffer[3] & 0x7F) << 24) |
                                  (uint32_t)(qi_rx_buffer[4] << 16) |
                                  (uint32_t)(qi_rx_buffer[5] << 8) |
                                  (uint32_t)(qi_rx_buffer[6]));
    break;
    
  case WPCQi_ExtendedIdentificationPacket_H:
    
    for (i = 0; i < 8; i++)
      qi_extended_device_identifier[i] = qi_rx_buffer[i];
     
    break;
    
  case WPCQi_ControlErrorPacet_H:
    qi_control_error_value = (signed char)qi_rx_buffer[0]; 
    
    break;
    
  case WPCQi_8BitReceivedPowerPacket_H:
    
    qi_received_power = (uint32_t)qi_rx_buffer[0];
    qi_received_power <<= 8;
    break;

  case WPCQi_24BitReceivedPowerPacket_H:
    qi_received_power_mode = qi_rx_buffer[0] & 0x07;
    qi_received_power = qi_rx_buffer[1];/*2 bytes  Preceive=(qi_rx_buffer[1:0]/32768)*(maximum power/2)*pow(10,power class)*/
    qi_received_power <<= 8;
    qi_received_power |=qi_rx_buffer[2];
    break;    
  case WPCQi_ChargeStatusPacket_H:
    
    qi_charge_status = qi_rx_buffer[0];
    
    break;
  case WPCQi_RenegotiatePacket_H:
    break;

  case WPCQi_GeneralRequestPacket_H: 
      if(qi_rx_buffer[0] == WPCQi_GeneralRequest_TransmitterIdentificationPacket_H)
      { 
          qi_tx_last_packet_header = WPCQi_GeneralRequest_TransmitterIdentificationPacket_H; 
      }
      else if(qi_rx_buffer[0] == WPCQi_GeneralRequest_TransmitterCapabilityPacket_H)
      {
          qi_tx_last_packet_header = WPCQi_GeneralRequest_TransmitterCapabilityPacket_H; 
      }
	  else if(qi_rx_buffer[0] == 0xFF)
			qi_tx_last_packet_header = 0xFF;  

  break;
  
  case WPCQi_FODStatusPacket_H:
    qi_quality_factor_mode=qi_rx_buffer[0] & 0x03;
    qi_ref_quality_factor=qi_rx_buffer[1];
    break; 
	
  case WPCQi_SpecificRequestPacket_H: //0x20
      if(qi_rx_buffer[0] == WPCQi_SpecificRequest_GuaranteedPowerPacket_H)
      { 
		  qi_guaranteed_reserved = (qi_rx_buffer[1]>>6) & 0x03;
          qi_guaranteed_power = qi_rx_buffer[1] & 0x3F;
      }
      else if(qi_rx_buffer[0] == WPCQi_SpecificRequest_EndNegotiationPacker_H)
      {
          neg_change_count = qi_rx_buffer[1]; 
      }
      else if(qi_rx_buffer[0] == WPCQi_SpecificRequest_ReceivedPowerPacketTypePacket_H)
      {
          qi_tx_received_power_header = qi_rx_buffer[1];
      }
      else if(qi_rx_buffer[0] == WPCQi_SpecificRequest_FSKParameterPacket_H)
      {
			qi_FSKReserved_20 = (qi_rx_buffer[1]>>3) & 0x1F;
			qi_FSKPolarity_20 = (qi_rx_buffer[1]>>2) & 0x01;
			qi_FSKDepth_20 = qi_rx_buffer[1] & 0x03; 
			fsk_delta_20 = G_QI_FSK_DELTA[qi_FSKDepth_20];
			qi_recevied_20 = 1;
      }
      else if(qi_rx_buffer[0] == WPCQi_SpecificRequest_MaxiumPowerPacket_H)
      {
          //qi_power_class = qi_rx_buffer[1] >> 6;
          qi_maximum_power_reserved = qi_rx_buffer[1] >> 6;
          qi_maximum_power = qi_rx_buffer[1] & 0x3F; // max value : 63
      }
      qi_tx_last_packet_header = qi_rx_buffer[0];

        break;
	case WPCQi_ProprietaryOOBInfoPacket_H:
		qi_oobMacAddr[0] = qi_rx_buffer[0];
		qi_oobMacAddr[1] = qi_rx_buffer[1];
		qi_oobMacAddr[2] = qi_rx_buffer[2];
		qi_oobMacAddr[3] = qi_rx_buffer[3];
		qi_oobMacAddr[4] = qi_rx_buffer[4];
		qi_oobMacAddr[5] = qi_rx_buffer[5];
		break;

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
    
    return 1;
  }
  
  return 0;
}
