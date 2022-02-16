
#include <stdio.h>
#include <string.h>
#include "BLE.h"
//#include "board.h"
#include "NuMicro.h"

uint8_t uart_buffer[128];

int UartBufferCnt = 0;

extern uint8_t BleComplete;

void sendDataToBle(uint8_t *data, uint8_t size)
{
    uint8_t i, ch, len =0, chksum = 0;	
	uint8_t /*txBuf[16] = {0,},*/ Buf[16] = {0,};

	len = size;

	memcpy( Buf, data, len );

	chksum = Buf[1];
	for ( i =2; i< len; i++ )
	   chksum ^= Buf[i];

	Buf[len++] = chksum;
	Buf[len++] = 0x7F;
	//printf("0x%02X ", Buf[2]);
	//printf("To BLE : ");
	for( i = 0; i<len; i++ )
	{	   

	   ch = Buf[i];
		 //printf("0x%02X ", ch);
	   UART_WRITE(UART2,ch);
	}
	//printf("\r\n");
}


void uart2Bufferinit( void )
{
    unsigned short i;
	printf("B: ");
	for( i=0; i <UartBufferCnt-1; i++ )
		printf("%02X ", uart_buffer[i+1]);
	printf("\n\r");
	UartBufferCnt = 0;
	memset( uart_buffer, 0, sizeof(uart_buffer));
}

void BLEenable(void){

    UART2->IER |= UART_IER_RDA_IEN_Msk;
    MyDelay(200);
    NVIC_EnableIRQ(UART02_IRQn);
	UartBufferCnt = 0;
    BleComplete = 0;
    return;
}

void BLEdisable(void){

	UART2->IER &= ~UART_IER_RDA_IEN_Msk;
	MyDelay(200);
	NVIC_DisableIRQ(UART02_IRQn);
	return;
}