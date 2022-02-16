#ifndef _FSK_H
#define _FSK_H

#include "Targetdef.h"


#define HALF_BIT               256 
#define FULL_BIT               512
extern unsigned char fsk_delta, fsk_delta_20;
#define QI_TX_BUFFER_SIZE      32

typedef enum 
{
    FBIT0 = 0,
    FBIT1 = 1
}BitValue;

typedef enum 
{ 
    QI_TX_START = 0,
    QI_TX_INFOR = 0,
    QI_TX_DATA,
    QI_TX_PARITY,
    QI_TX_STOP,
    QI_TX_END_PACKET,
    QI_TX_ACK,
    QI_TX_NACK,
    QI_TX_ND,
    QI_TX_NEXT
}qi_Tx_State;


typedef enum
{
  end_negotiation,
  guaranteed_power,
  received_power_packet_type,
  fsk_parameters,
  maximum_power
    
} specific_request_field;

typedef enum
{
  // Negotiation phase
  
  pwrxmitter_data_not_available =			0x00,
  pwrxmitter_identification 		=			0x30,
  pwrxmitter_capability 				=			0x31,
  
} qi_Tx_Header_Type;


void Fsk_vStart(uint16_t bit_machine);
void Fsk_vInit(qi_Tx_State mes_type);

extern uint8_t g_qi_tx_buffer[QI_TX_BUFFER_SIZE];  /* buffer contains messages Power Transmitter responds to Power Receiver */
extern uint8_t g_qi_tx_size;                   /* byte number of message Power Transmitter responds to Power Receiver */



#endif
