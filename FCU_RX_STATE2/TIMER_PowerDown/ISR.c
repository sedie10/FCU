#include <stdio.h>
//#include "board.h"
#include "NuMicro.h"
#include "ISR.h"
#include "ASK.h"
#include "FSK.h"
#include "BLE.h"

extern uint32_t timerCount;
extern uint8_t AskStart;
extern uint8_t uart_buffer[128];

extern char tmpBuf[100];
extern char ch;
extern int i, j;
extern int length;
extern void AskOut();
extern uint8_t ByteCnt;
extern uint8_t InFskTransfer;

volatile uint32_t tmpcnt = 0;

uint32_t FskCnt = 0;
uint32_t FskStartCnt = 0;
uint32_t FskEndCnt = 0;
uint8_t HalfBitCnt = 0;
uint8_t FullBitCnt = 0;
uint8_t BleComplete = 0;

extern int UartBufferCnt;
extern uint8_t AskHeader;
void TMR0_IRQHandler(void){
    //timer0 interrupt handler
		
    if(TIMER_GetIntFlag(TIMER0) == 1){

        if(AskStart){
            AskOut();
            TIMER_ClearIntFlag(TIMER0);
    	}
	}
}

void GPAB_IRQHandler(void){
    //GPIO PA, PB interrupt handler
    if(GPIO_GET_INT_FLAG(PB, BIT3)){

        if(!ByteCnt){
            InFskTransfer = 1; //second chain
        }

        if(!FskCnt){
					//FskStartCnt = TIMER_GetCounter(TIMER0);
					FskStartCnt = TIMER0->TDR;
				}
            

        FskCnt ++;
        
        if(FskCnt == HALFBIT){
            //FskEndCnt = TIMER_GetCounter(TIMER0);
						FskEndCnt = TIMER0->TDR;
            HalfBitCnt = FskEndCnt - FskStartCnt;
            FskStartCnt = FskEndCnt;
        }else if(FskCnt == FULLBIT){
            //FskEndCnt = TIMER_GetCounter(TIMER0);
						FskEndCnt = TIMER0->TDR;
            FullBitCnt = FskEndCnt - FskStartCnt;
            FskStartCnt = 0;
            FskEndCnt = 0;
            FskCnt = 0;
            FskBitUpdate(HalfBitCnt, FullBitCnt);
        }
        GPIO_CLR_INT_FLAG(PB, BIT3);


    }else{
        PA -> ISRC = PA -> ISRC;
        PB -> ISRC = PB -> ISRC;
    }
}

void UART02_IRQHandler(void){
		
	int b;
	uint8_t dat = 0xFF;
	uint32_t intsts = UART2->ISR;
	
	if(intsts & UART_ISR_RDA_INT_Msk){
		printf("\r\nB : ");
			
        while(UART_IS_RX_READY(UART2)){
                dat = UART_READ(UART2);
                uart_buffer[UartBufferCnt] = dat;
                printf("0x%02X ", uart_buffer[UartBufferCnt]);
                UartBufferCnt++;
        }
        printf("\r\n");
	}
	//BLEdisable();
	
	BleComplete = 1;

}
		
