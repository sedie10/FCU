#ifndef WPCQITX_TXSTATE_H
#define WPCQITX_TXSTATE_H
 
 
/****************************************************************************************************/
/*																									*/
/*			Typedef																			              */ 
/*																									*/
/*																									*/
/****************************************************************************************************/

typedef enum 
{
  FSK_IDLE_S,                // No activity
  FSK_WAIT_S,           // Wait for Tdelay to expire, then run contol with new CEP
  FSK_RUN_S,           // Run control 
} WPCQi_FskState_t;

typedef enum 
{
  FSK_ND_RESPONSE, 
  FSK_ACK_RESPONSE,                
  FSK_NAK_RESPONSE, 
  FSK_NA_RESPONSE, 
  FSK_IDENTIFICATE_PACKET, 
  FSK_CAPABILITY_PACKET, 

}WPCQiTx_FskResponse_t;



/****************************************************************************************************/
/*																									*/
/*			External functions																              */ 
/*																									*/
/*																									*/
/****************************************************************************************************/
void WPCQiTx_setFskInfo( WPCQi_FskState_t state, WPCQiTx_FskResponse_t response );
void WPCQiTx_TxState(void);
void WPCQiTx_FskSendIdentification(void);
void WPCQiTx_FskSendConfiguration(void);
void WPCQiTx_FskSendAck(void);
void WPCQiTx_FskSendNack(void); 
void WPCQiTx_FskSendNd(void);
void WPCQiTx_FskSendNotAvailable(void);


extern WPCQi_FskState_t WPCQiTxFskLoopState;
extern WPCQiTx_FskResponse_t WPCQiFskResponse;
#endif
