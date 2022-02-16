#include "WPCQiTx_TxState.h"
#include "Fsk.h"
#include "WPCQiTx.h"
#include "Ble.h"

WPCQi_FskState_t WPCQiTxFskLoopState = FSK_IDLE_S;
WPCQiTx_FskResponse_t WPCQiFskResponse;


/****************************************************************************************************/
/*																									*/
/*			WPCQiTx_FskSendIdentification																              */ 
/*																									*/
/*																									*/
/****************************************************************************************************/
void WPCQiTx_FskSendIdentification( void )
{

    g_qi_tx_buffer[0] = pwrxmitter_identification;
    g_qi_tx_buffer[1] = (qi_major_version << 4) | qi_minor_version;
    g_qi_tx_buffer[2] = qi_manufacturer_code >> 8;
    g_qi_tx_buffer[3] = qi_manufacturer_code & 0xFF;
    g_qi_tx_buffer[4] = g_qi_tx_buffer[0] ^ g_qi_tx_buffer[1] ^ g_qi_tx_buffer[2] ^ g_qi_tx_buffer[3];
    g_qi_tx_size = 5;

    Fsk_vInit(QI_TX_INFOR);
}

/****************************************************************************************************/
/*																									*/
/*			WPCQiTx_FskSendConfiguration																              */ 
/*																									*/
/*																									*/
/****************************************************************************************************/
void WPCQiTx_FskSendConfiguration(void)
{
	g_qi_tx_buffer[0] = pwrxmitter_capability;
	g_qi_tx_buffer[1] = (0x01 << 6) | QI_GUARANTEED_POWER_C0;
	g_qi_tx_buffer[2] = (0x01 << 6) | (QI_POTENTIAL_POWER_C0);
	g_qi_tx_buffer[3] = 0; //edwardkwon : reserved + WPID + not res sens
	g_qi_tx_buffer[4] = g_qi_tx_buffer[0] ^ g_qi_tx_buffer[1] ^ g_qi_tx_buffer[2] ^ g_qi_tx_buffer[3] ;
	g_qi_tx_size = 5;

    Fsk_vInit(QI_TX_INFOR);
}
/****************************************************************************************************/
/*																									*/
/*			WPCQiTx_FskSendNotAvailable																              */ 
/*																									*/
/*																									*/
/****************************************************************************************************/
void WPCQiTx_FskSendNotAvailable(void)
{
	g_qi_tx_buffer[0] = pwrxmitter_data_not_available;
	g_qi_tx_buffer[1] = 0x00;
	g_qi_tx_buffer[2] = g_qi_tx_buffer[0] ^ g_qi_tx_buffer[1];
	g_qi_tx_size = 3;

	Fsk_vInit(QI_TX_INFOR);
}

/****************************************************************************************************/
/*																									*/
/*			WPCQiTx_FskSendAck																              */ 
/*																									*/
/*																									*/
/****************************************************************************************************/
void WPCQiTx_FskSendAck(void)
{ 
	Fsk_vInit(QI_TX_ACK);  
}

/****************************************************************************************************/
/*																									*/
/*			WPCQiTx_FskSendNack																              */ 
/*																									*/
/*																									*/
/****************************************************************************************************/
void WPCQiTx_FskSendNack(void)
{ 
    Fsk_vInit(QI_TX_NACK);
}


/****************************************************************************************************/
/*																									*/
/*			WPCQiTx_FskSendNd																              */ 
/*																									*/
/*																									*/
/****************************************************************************************************/
void WPCQiTx_FskSendNd( void )
{ 
    Fsk_vInit(QI_TX_ND);
}
/****************************************************************************************************/
/*																									*/
/*			WPCQiTx_setFskState																              */ 
/*																									*/
/*																									*/
/****************************************************************************************************/
void WPCQiTx_setFskInfo( WPCQi_FskState_t state, WPCQiTx_FskResponse_t response )
{
	WPCQiTxFskLoopState = state;
	WPCQiFskResponse = response;
}
/****************************************************************************************************/
/*																									*/
/*			WPCQiTx_TxState																              */ 
/*																									*/
/*																									*/
/****************************************************************************************************/
void WPCQiTx_TxState(void)
{
	switch (WPCQiTxFskLoopState)
	{
		case FSK_IDLE_S:

			break;

		case FSK_WAIT_S:

			if (qi_packet_last_bit_timer_ms >= T_RESPONSE_MS)
			{
				WPCQiTxFskLoopState = FSK_RUN_S;

				if (WPCQiFskResponse == FSK_NA_RESPONSE)
				{
#if defined(BLE_HANDSHAKING)
                    if( commType == QI_COMMTYPE_BLE&&  qi_oob_flag == QI_COMMTYPE_BLE )
                    {
					    sendNotavailableToBle();
                    }
					else
					{
						WPCQiTx_FskSendNotAvailable();
					}
#else
					WPCQiTx_FskSendNotAvailable();
#endif
					debug_fsk_print("\r\nNA");
				}
				else if (WPCQiFskResponse == FSK_IDENTIFICATE_PACKET)
				{
#if defined(BLE_HANDSHAKING) 
					debug_ble_print("\nid_packet %d %d %d",commType,qi_oob_flag, wait_ble_connection);
                    if( commType == QI_COMMTYPE_COIL&&  qi_oob_flag == QI_COMMTYPE_BLE )
                    {
                       if( ++wait_ble_connection>20 ) WPCQi_TxReset();
					
                       break;
                    }
                    if( commType == QI_COMMTYPE_BLE &&  qi_oob_flag == QI_COMMTYPE_BLE )
					{
					    sendIdentificationToBle();
					}
					else if (qi_oob_flag != QI_COMMTYPE_BLE)
					{
						WPCQiTx_FskSendIdentification();
					}
#else
					WPCQiTx_FskSendIdentification();
#endif
					debug_fsk_print("\r\nID");
				}
				else if (WPCQiFskResponse == FSK_CAPABILITY_PACKET)
				{
#if defined(BLE_HANDSHAKING)
                    if( commType == QI_COMMTYPE_BLE&&  qi_oob_flag == 0x01 )
                    {
					    sendConfigurationToBle();
                    }
					else
					{
						WPCQiTx_FskSendConfiguration();
					}
#else
					WPCQiTx_FskSendConfiguration();
#endif					
					debug_fsk_print("\r\nCON");
				}
				else if (WPCQiFskResponse == FSK_ACK_RESPONSE)
				{
                    if( commType == QI_COMMTYPE_BLE &&  qi_oob_flag == 0x01 ) sendResponseToBle(BLE_RES_ACK);
					else	WPCQiTx_FskSendAck();
					debug_fsk_print("\r\nACK");
				}
				else if (WPCQiFskResponse == FSK_NAK_RESPONSE)
				{
                    if( commType == QI_COMMTYPE_BLE &&  qi_oob_flag == 0x01 ) sendResponseToBle(BLE_RES_NAK);
					else WPCQiTx_FskSendNack();
					debug_fsk_print("\r\nNAK");
				}
				else if (WPCQiFskResponse == FSK_ND_RESPONSE)
				{
                    if( commType == QI_COMMTYPE_BLE &&  qi_oob_flag == 0x01 ) sendResponseToBle(BLE_RES_ND);
	                else WPCQiTx_FskSendNd();
					debug_fsk_print("\r\nND");
				}				
			}
			break;

		case FSK_RUN_S:
			if (qi_packet_last_bit_timer_ms >= T_ACTIVE_MS)
			{
				WPCQiTxFskLoopState = FSK_IDLE_S;
				WPCQiFskResponse = FSK_ND_RESPONSE;
			}
			break;
		}
}


