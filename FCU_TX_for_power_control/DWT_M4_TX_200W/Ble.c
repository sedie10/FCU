#include <string.h>
#include "application.h"
#include "modulation.h"
#include "WPCQiTx.h"
#include "Fsk.h"

#if defined( BLE_HANDSHAKING )
#define UARTBLE_MAX_SIZE 		256
#define UART_MESSAGE_SIZE		128 //KYJ 8->128

uint8_t uart_buffer[UART_MESSAGE_SIZE];
static int uart_byte_count = 0;

uint8_t buf_UARTBLE[UARTBLE_MAX_SIZE];
uint16_t q_UARTBLE_wr = 0, q_UARTBLE_rd = 0;

#define SOH							0x01
#define	EOT							0X04

#define _CR    						0x0D
#define _LF    						0x0A
#define BLE_CMD_HEADER  			0x7E
#define BLE_CMD_EX_HEADER  			0x7D
#define BLE_CMD_ETX  				0x7F
#define BLE_CMD_REP                 0x7C
#define BLE_INDICATOR_MAC    		0x01
#define BLE_INDICATOR_CONNECT  		0x10

const uint8_t BLE_QI_FSK_DELTA[4] = {4, 7, 13, 25}; 
static uint8_t uartCmdReading = 0;
extern void qi_comm_message_init( void);

static uint8_t sendIdentificationCnt = 0;

/*----------------------------------------------------------------------------------------------------------*/
/*      																										*/ 	
/*      																										*/
/*      function : dleDel																							*/
/*       																										*/
/* 																											*/
/*----------------------------------------------------------------------------------------------------------*/
uint16_t dleDel(uint8_t *dst, uint8_t *src, uint16_t length)
{
   uint16_t i, dstlen=0;

   for (i=0; i<length; i++)
   {
      switch (src[i])
      {
      case 0x10:
         i++;
         dst[dstlen] = src[i] ^ 0x20;
         break;
      default:
         dst[dstlen] = src[i];
         break;
      }

      dstlen++;
   }

   return dstlen;
}
/*----------------------------------------------------------------------------------------------------------*/
/*      																										*/ 	
/*      																										*/
/*      function : dleAdd																							*/
/*       																										*/
/* 																											*/
/*----------------------------------------------------------------------------------------------------------*/

uint16_t dleAdd(uint8_t *dst, uint8_t *src, uint16_t length)
{
   uint16_t i, dstlen=0;

   for (i=0; i<length; i++)
   {
      switch (src[i])
      {
      case 0x01:
      case 0x04:
      case 0x10:
         dst[dstlen] = 0x10;
         dstlen++;
         dst[dstlen] = src[i] ^ 0x20;
         break;

      default:
         dst[dstlen] = src[i];
         break;
      }

      dstlen++;
   }

   return dstlen;
}
/*----------------------------------------------------------------------------------------------------------*/
/*      																										*/ 	
/*      																										*/
/*      function : sendDataToBle																					*/
/*       																										*/
/* 																											*/
/*----------------------------------------------------------------------------------------------------------*/

void sendDataToBle ( uint8_t *data, uint8_t size)
{
    uint8_t i, ch, length =0, chksum = 0;	
	uint8_t /*txBuf[16] = {0,},*/ tmpBuf[16] = {0,};
#if 0
    txBuf[0] = SOH;
    length = dleAdd(txBuf+1, data, size );
	txBuf[length+1] = EOT;
	length += 2;

	for( i = 0; i<length; i++ )
	{
	   
	   ch = txBuf[i];
       debug_ble_print("\n sendDataToBle ch=0x%x", ch);
	   UART_WRITE(UART0,ch);
	   
	}
#else
	length = size;

	memcpy( tmpBuf, data, length );

	chksum = tmpBuf[1];
	for ( i =2; i< length; i++ )
	   chksum ^= tmpBuf[i];

	tmpBuf[length++] = chksum;
	tmpBuf[length++] = BLE_CMD_ETX;

	for( i = 0; i<length; i++ )
	{	   
	   ch = tmpBuf[i];
	   UART_WRITE(UART0,ch);
	}


#endif
}
/*----------------------------------------------------------------------------------------------------------*/
/*      																										*/ 	
/*      																										*/
/*      function : sendMacAddToBle																				*/
/*       																										*/
/* 																											*/
/*----------------------------------------------------------------------------------------------------------*/

void sendMacAddToBle( void )
{
    uint8_t txBuf[10]={0,}, len = 9;
    txBuf[0] = BLE_CMD_HEADER;
	txBuf[1] = 0x68;
	txBuf[2] = 0x06;
	
    memcpy( &txBuf[3], qi_oobMacAddr, sizeof(qi_oobMacAddr));
	sendDataToBle(txBuf, len);
	debug_ble_print("\r\nRX MAC [0x%x,0x%x,0x%x,0x%x,0x%x,0x%x]", qi_oobMacAddr[0], qi_oobMacAddr[1], qi_oobMacAddr[2], qi_oobMacAddr[3], qi_oobMacAddr[4], qi_oobMacAddr[5]);

	wait_ble_connection = 0;	
}
/*----------------------------------------------------------------------------------------------------------*/
/*      																										*/ 	
/*      																										*/
/*      function : sendResponseToBle																			*/
/*       																										*/
/* 																											*/
/*----------------------------------------------------------------------------------------------------------*/


void sendResponseToBle ( uint8_t type )
{
	uint8_t txBuf[8] = {0,};

	debug_ble_print("\nBLE_S:ACK/NAK/ND=%d, CMD=%d", type, qi_tx_last_packet_header );    
    txBuf[0] = BLE_CMD_EX_HEADER;
    txBuf[1] = BLE_CMD_REP;
	txBuf[2] = type;
	sendDataToBle(txBuf, 3);
}

/*----------------------------------------------------------------------------------------------------------*/
/*      																										*/ 	
/*      																										*/
/*      function : sendNotavailableToBle																			*/
/*       																										*/
/* 																											*/
/*----------------------------------------------------------------------------------------------------------*/

void sendNotavailableToBle( void )
{
    uint8_t txBuf[10]={0,}, len = 0;
	txBuf[0] = BLE_CMD_EX_HEADER;
	txBuf[1] = pwrxmitter_data_not_available;
	txBuf[2] = 0x00;
	len = 3;

	sendDataToBle(txBuf, len);
	debug_ble_print("\nBLE_S:Notavailable");
}

/*----------------------------------------------------------------------------------------------------------*/
/*      																										*/ 	
/*      																										*/
/*      function : sendIdentificationToBle																			*/
/*       																										*/
/* 																											*/
/*----------------------------------------------------------------------------------------------------------*/

void sendIdentificationToBle( void )
{
    uint8_t txBuf[10]={0,}, len = 0;

    txBuf[0] = BLE_CMD_EX_HEADER;
    txBuf[1] = pwrxmitter_identification;
    txBuf[2] = (qi_major_version << 4) | qi_minor_version;
    txBuf[3] = qi_manufacturer_code >> 8;
    txBuf[4] = qi_manufacturer_code & 0xFF;
    len = 5;

//	qi_comm_message_init();
	if( sendIdentificationCnt <=5 )
	{
		;  //wait
	}
	if( sendIdentificationCnt <= 15 )
	{
		sendIdentificationCnt++;
		sendDataToBle(txBuf, len);  //send data to RX using the BLE
	}
	else
	{
		sendIdentificationCnt = 0;
		WPCQi_TxReset();
 	}
 		
	debug_ble_print("\nBLE_S:Power Transmitter Identification %d", sendIdentificationCnt);
}
/*----------------------------------------------------------------------------------------------------------*/
/*      																										*/ 	
/*      																										*/
/*      function : sendConfigurationToBle																			*/
/*       																										*/
/* 																											*/
/*----------------------------------------------------------------------------------------------------------*/

void sendConfigurationToBle( void )
{
    uint8_t txBuf[10]={0,}, len = 0;

    txBuf[0] = BLE_CMD_EX_HEADER;
	txBuf[1] = pwrxmitter_capability;
	txBuf[2] = (qi_guaranteed_class << 6) | QI_GUARANTEED_POWER;
	txBuf[3] = (qi_potential_class << 6) | (QI_POTENTIAL_POWER);
	txBuf[4] = 0;
	len = 5;

	sendDataToBle(txBuf, len);
	debug_ble_print("\nBLE_S:Power Transmitter Capability");
}

/*----------------------------------------------------------------------------------------------------------*/
/*      																										*/ 	
/*      																										*/
/*      function : uart0BuffInit																					*/
/*       																										*/
/* 																											*/
/*----------------------------------------------------------------------------------------------------------*/

void uart0BuffInit( void )
{
    unsigned short i;
	printf("B: ");
	for( i=0; i <uart_byte_count-1; i++ )
		printf("%02X ", uart_buffer[i+1]);
	printf("\n\r");
	uart_byte_count = 0;
	memset( uart_buffer, 0, sizeof(uart_buffer));
}

/*----------------------------------------------------------------------------------------------------------*/
/*      																										*/ 	
/*      																										*/
/*      function : buildChkSum																					*/
/*       																										*/
/* 																											*/
/*----------------------------------------------------------------------------------------------------------*/

uint8_t buildChkSum( uint8_t *data, uint8_t length)
{
	uint8_t chksum = 0, i, tmpBuff[10] = {0,};
	memcpy( tmpBuff, data, length );
	chksum = tmpBuff[1];
	
	for( i = 1; i<length-2; i++ )
	{
	   chksum ^= tmpBuff[1+i];
//	   debug_ble_print("\n %d, %d", chksum, tmpBuff[1+i]);
	}
	return chksum;
}

/*----------------------------------------------------------------------------------------------------------*/
/*      																										*/ 	
/*      																										*/
/*      function : 	dataLengthCheck																				*/
/*       																										*/
/* 																											*/
/*----------------------------------------------------------------------------------------------------------*/

uint8_t dataLengthCheck(uint8_t length)
{
    uint8_t cmd, subCmd, negSpecCmd=0, cmdlength =0;

		cmd = uart_buffer[0];
		subCmd = uart_buffer[1];
	
		switch( cmd )
		{
			case BLE_CMD_HEADER:
				switch( subCmd )
				{
				
					case BLE_INDICATOR_CONNECT:
						return 1;
                        //cmdlength = 2;
						//break;
	
					case BLE_INDICATOR_MAC:
						break;
				}
	
				break;
			
			case BLE_CMD_EX_HEADER:
				
				switch( subCmd )
				{
					case WPCQi_ControlErrorPacet_H:  /*0x03*/
                        cmdlength = 4;
						break;
	
					case WPCQi_8BitReceivedPowerPacket_H: /*0x04*/
					case WPCQi_FODStatusPacket_H:
                        cmdlength = 5;
						break;
	
					case WPCQi_24BitReceivedPowerPacket_H: /*0x31*/
                        cmdlength = 6;
						break;
	
	
					case WPCQi_GeneralRequestPacket_H:
						negSpecCmd = uart_buffer[2];
						switch( negSpecCmd )
						{
							case WPCQi_GeneralRequest_TransmitterCapabilityPacket_H:
							case WPCQi_GeneralRequest_TransmitterIdentificationPacket_H:
							case 0xff:
								cmdlength = 4;
								break;
						}
						break;
						
					case WPCQi_SpecificRequestPacket_H:					
						negSpecCmd = uart_buffer[2];
						switch(negSpecCmd)
						{
							case WPCQi_SpecificRequest_GuaranteedPowerPacket_H:
							case WPCQi_SpecificRequest_EndNegotiationPacker_H:
							case WPCQi_SpecificRequest_ReceivedPowerPacketTypePacket_H:
							case WPCQi_SpecificRequest_FSKParameterPacket_H:
							case WPCQi_SpecificRequest_MaxiumPowerPacket_H:
								cmdlength = 5;
								break;
						}
						break;
	
				}
				break;
	
			
			 default:			
				break;
		}
        if( length > cmdlength )
        {
            debug_ble_print("\r\n BLE data borken:l=%d, %d, %d, %d",length, uart_buffer[0],uart_buffer[1],uart_buffer[2] );
			uart_byte_count = length = cmdlength;
        }		
		return (cmdlength == length );
}

/*----------------------------------------------------------------------------------------------------------*/
/*      																										*/ 	
/*      																										*/
/*      function : UART0_Proc																					*/
/*       																										*/
/* 																											*/
/*----------------------------------------------------------------------------------------------------------*/

void UART0_Proc( uint8_t data )
{

//	uint16_t length = 0;
//	uint8_t tmpBuff[256] = {0,};

	uint8_t cmd = 0, subCmd = 0, negSpecCmd = 0;

    if( qi_oob_flag != 0x01 ) return;

		printf("%02X \r\n", data); 
		//debug_ble_print("\r\n UART C=0x%x", data);

	switch( data )
	{

	   case BLE_CMD_HEADER: //SOH:
	   case BLE_CMD_EX_HEADER:
	   	   if( !uartCmdReading )
	   	   {
		     uart_byte_count = 0;
		     memset( uart_buffer, 0, sizeof(uart_buffer));
			 
			 uart_buffer[uart_byte_count++] = data;
			 if( uart_byte_count > UART_MESSAGE_SIZE ) uart_byte_count = 0;   
			 uartCmdReading = 1;
	   	   }
		   else
		   {
			   uart_buffer[uart_byte_count++] = data;
			   if( uart_byte_count > UART_MESSAGE_SIZE ) uart_byte_count = 0;	  
		   }
		   return;
		   
		case BLE_CMD_ETX://EOT:
		
		   if( uartCmdReading )
		   {
			   if( dataLengthCheck(uart_byte_count)){
					uartCmdReading = 0;
					
			   }
			     
			   else
			   {
				   uart_buffer[uart_byte_count++] = data;
				   if( uart_byte_count > UART_MESSAGE_SIZE ) uart_byte_count = 0;	  
			   	   debug_ble_print("\nBLECMDETX %x:%x,%d", uart_buffer[0], uart_buffer[1],uart_byte_count);
	               return;
			   }
			   break;
		   }
		   else
		   	return;

	    default: 
		   if( uartCmdReading )
		   {
			   uart_buffer[uart_byte_count++] = data;
			   if( uart_byte_count > UART_MESSAGE_SIZE ) uart_byte_count = 0; 	
		   }
		   return;

	}

	if( uart_byte_count <= 0 ) return;

//	length = dleDel( tmpBuff, uart_buffer,uart_byte_count );
//    memcpy( tmpBuff, uart_buffer, uart_byte_count);


	cmd = uart_buffer[0];
	subCmd = uart_buffer[1];
	
    if( subCmd != WPCQi_ControlErrorPacet_H )
	{
	    if( cmd == BLE_CMD_EX_HEADER ) 
	    {
	        if( uart_buffer[uart_byte_count-1] != buildChkSum(uart_buffer, uart_byte_count))
	       {
		       debug_ble_print("\r\n Chksum wrong !! %d",uart_buffer[uart_byte_count-1]);
		       uart0BuffInit();
		       return;
	       }
	    }
    }

	switch( cmd )
	{
		case BLE_CMD_HEADER:
			
			
			switch( subCmd )
			{
			
				case BLE_INDICATOR_CONNECT:
					commType = QI_COMMTYPE_BLE;
					debug_ble_print("\r\n BLE CONNECTED");
					wait_ble_connection = 0;
					uart0BuffInit();
					break;

				case BLE_INDICATOR_MAC:
					uart0BuffInit();
					break;
			}

			break;
		
		case BLE_CMD_EX_HEADER:
			
			switch( subCmd )
			{
				case WPCQi_ControlErrorPacet_H:  /*0x03*/
					qi_packet_last_bit_timer_ms = 0;	   
					qi_control_error_value = (signed char)uart_buffer[2];
					qi_comm_message_received = 1;
					qi_last_packet_header = WPCQi_ControlErrorPacet_H;
					debug_ble_print("\nBLE_R:03 CEP:%d", (signed char)qi_control_error_value);	  
					uart0BuffInit();
					break;

				case WPCQi_8BitReceivedPowerPacket_H: /*0x04*/
					
					qi_packet_last_bit_timer_ms = 0;	   
					qi_received_power = (uint32_t)uart_buffer[2];
					qi_received_power <<= 8;
					qi_comm_message_received = 1;
					qi_last_packet_header = WPCQi_8BitReceivedPowerPacket_H;
					debug_ble_print("\nBLE_R:8-bit Received Power Packet:%d",qi_received_power);	  
					uart0BuffInit();
					break;

				case WPCQi_24BitReceivedPowerPacket_H: /*0x31*/
					
					qi_packet_last_bit_timer_ms = 0;
					qi_received_power_mode = uart_buffer[2] & 0x07;
					qi_received_power = ((uart_buffer[3]<<8) | uart_buffer[4]);
					
					qi_comm_message_received = 1;
					qi_last_packet_header = WPCQi_24BitReceivedPowerPacket_H;
					debug_ble_print("\nBLE_R:31 %d %d %d",qi_received_power, uart_buffer[3], uart_buffer[4]);	 
					uart0BuffInit();
					break;

				case WPCQi_FODStatusPacket_H:
					qi_comm_message_received = 1;			
					qi_last_packet_header = WPCQi_FODStatusPacket_H;
					
					qi_packet_last_bit_timer_ms = 0;
					qi_quality_factor_mode=uart_buffer[2] & 0x03;
					qi_ref_quality_factor=uart_buffer[3];
					debug_ble_print("\nBLE_R:FOD Status:%d,%d", qi_quality_factor_mode,qi_ref_quality_factor );   
					uart0BuffInit();
					break;

                case WPCQi_GeneralRequestPacket_H:
					negSpecCmd = uart_buffer[2];
					switch( negSpecCmd )
					{
						case WPCQi_GeneralRequest_TransmitterCapabilityPacket_H:
							qi_comm_message_received = 1;			
							qi_last_packet_header = WPCQi_GeneralRequestPacket_H;
				
							qi_packet_last_bit_timer_ms = 0;				
							qi_tx_last_packet_header = WPCQi_GeneralRequest_TransmitterCapabilityPacket_H; 
							debug_ble_print("\nBLE_R:General Request-Power Transmitter Capability");	  
							uart0BuffInit();
							break;

						case WPCQi_GeneralRequest_TransmitterIdentificationPacket_H:
							
							qi_comm_message_received = 1;			
							qi_last_packet_header = WPCQi_GeneralRequestPacket_H;
				
							qi_packet_last_bit_timer_ms = 0;			
							qi_tx_last_packet_header = WPCQi_GeneralRequest_TransmitterIdentificationPacket_H; 
							debug_ble_print("\nBLE_R:General Request-Power Transmitter Identification");	  
							uart0BuffInit();
							break;
						case 0xff:
							
							qi_comm_message_received = 1;			
							qi_last_packet_header = WPCQi_GeneralRequestPacket_H;
				
							qi_packet_last_bit_timer_ms = 0;			
							qi_tx_last_packet_header = 0xFF; 
							debug_ble_print("\nBLE_R:General Request-0xff");	  
							uart0BuffInit();
							break;
					}
					break;
					
				case WPCQi_SpecificRequestPacket_H:					
					negSpecCmd = uart_buffer[2];
					switch(negSpecCmd)
					{
						case WPCQi_SpecificRequest_GuaranteedPowerPacket_H:
							
							qi_comm_message_received = 1;			
							qi_last_packet_header = WPCQi_SpecificRequestPacket_H;
							qi_packet_last_bit_timer_ms = 0;
							qi_guaranteed_reserved = (uart_buffer[3]>>6) & 0x03;
							qi_guaranteed_power = uart_buffer[3] & 0x3F;
							qi_tx_last_packet_header = WPCQi_SpecificRequest_GuaranteedPowerPacket_H;
							debug_ble_print("\nBLE_R:Specific Request-Guaranteed Power:%d,%d",qi_guaranteed_reserved, qi_guaranteed_power);	  
							uart0BuffInit();
							break;

						case WPCQi_SpecificRequest_EndNegotiationPacker_H:
							
							qi_packet_last_bit_timer_ms = 0;
							neg_change_count = uart_buffer[3];	
							qi_tx_last_packet_header = WPCQi_SpecificRequest_EndNegotiationPacker_H;
							
							qi_comm_message_received = 1;			
							qi_last_packet_header = WPCQi_SpecificRequestPacket_H;
							debug_ble_print("\nBLE_R:Specific Request-End Negotiation:%d",neg_change_count);	  
							uart0BuffInit();
							break;

						case WPCQi_SpecificRequest_ReceivedPowerPacketTypePacket_H:
				
							qi_packet_last_bit_timer_ms = 0;
							qi_tx_received_power_header = uart_buffer[3];
							
							qi_tx_last_packet_header = WPCQi_SpecificRequest_ReceivedPowerPacketTypePacket_H;
				
							qi_comm_message_received = 1;			
							qi_last_packet_header = WPCQi_SpecificRequestPacket_H;
							debug_ble_print("\nBLE_R:Specific Request-Received Power Packet Type");	  
							uart0BuffInit();
							break;

						case WPCQi_SpecificRequest_FSKParameterPacket_H:
							qi_comm_message_received = 1;			
							qi_last_packet_header = WPCQi_SpecificRequestPacket_H;
							qi_packet_last_bit_timer_ms = 0;
							qi_FSKReserved_20 = (uart_buffer[3]>>3) & 0x1F;
							qi_FSKPolarity_20 = (uart_buffer[3]>>2) & 0x01;
							qi_FSKDepth_20 = uart_buffer[3] & 0x03; 
							fsk_delta_20 = BLE_QI_FSK_DELTA[qi_FSKDepth_20];
							qi_recevied_20 = 1;
							
							qi_tx_last_packet_header = WPCQi_SpecificRequest_FSKParameterPacket_H;
							debug_ble_print("\nBLE_R:Specific Request-FSK Parameters:%d,%d,%d",qi_FSKReserved_20,qi_FSKPolarity_20,qi_FSKDepth_20);	  
							uart0BuffInit();
							break;
						case WPCQi_SpecificRequest_MaxiumPowerPacket_H:
							qi_comm_message_received = 1;			
							qi_last_packet_header = WPCQi_SpecificRequestPacket_H;
				
							qi_packet_last_bit_timer_ms = 0;
							qi_maximum_power_reserved = uart_buffer[3] >> 6;
							qi_maximum_power = uart_buffer[3] & 0x3F; // max value : 63
							
							qi_tx_last_packet_header = WPCQi_SpecificRequest_MaxiumPowerPacket_H;
							debug_ble_print("\nBLE_R:Specific Request-Maximum Power:%d,%d",qi_maximum_power_reserved,qi_maximum_power);	  
							uart0BuffInit();
							break;
					}
			    	break;

			}
			break;

		
		 default:			
			debug_ble_print("\r\nBLE wrong cmd=0x%x, sub=%d",cmd, subCmd);
			uart0BuffInit();
			break;
	}
	

}
/*----------------------------------------------------------------------------------------------------------*/
/*      																										*/ 	
/*      																										*/
/*      function : buildChkSum																					*/
/*       																										*/
/* 																											*/
/*----------------------------------------------------------------------------------------------------------*/
uint8_t getString_UART0( void )
{
  uint8_t ch = 0x00;

  while( q_UARTBLE_rd == q_UARTBLE_wr );

  ch = buf_UARTBLE[q_UARTBLE_rd];
  if(++q_UARTBLE_rd >= UARTBLE_MAX_SIZE ) q_UARTBLE_rd = 0;

  return( ch );
}

/*----------------------------------------------------------------------------------------------------------*/
/*      																										*/ 	
/*      																										*/
/*      function : buildChkSum																					*/
/*       																										*/
/* 																											*/
/*----------------------------------------------------------------------------------------------------------*/
uint8_t getStatus_UART0(void)
{
  return( q_UARTBLE_rd != q_UARTBLE_wr );
}

/*----------------------------------------------------------------------------------------------------------*/
/*      																										*/ 	
/*      																										*/
/*      function : buildChkSum																					*/
/*       																										*/
/* 																											*/
/*----------------------------------------------------------------------------------------------------------*/
void UART0_IRQHandler(void)
{

	uint32_t u32IntSts = UART0->INTSTS;

    if((u32IntSts & UART_INTSTS_RDAINT_Msk) || (u32IntSts & UART_INTSTS_RXTOINT_Msk))
    {
    
	    buf_UARTBLE[q_UARTBLE_wr] = UART_READ(UART0); 
		if( ++q_UARTBLE_wr >= UARTBLE_MAX_SIZE ) q_UARTBLE_wr = 0;
    }
}

#endif

