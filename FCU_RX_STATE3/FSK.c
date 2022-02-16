#include <stdio.h>
#include "FSK.h"
//#include "board.h"
#include "NuMicro.h"

uint8_t FskByte = 0;
uint8_t ByteCnt = 0;
uint8_t FskParity = 0;
uint8_t FskBuffer[8];
uint8_t FskBufferCnt = 0;
uint8_t InFskTransfer = 0;

extern uint8_t qi_FSKDepth;



void FskBitUpdate(int halfbit, int fullbit){
    int diff = halfbit - fullbit;
    uint8_t bit;
		
    if(diff < 0){
        diff *= -1;
    }

    if(qi_FSKDepth == 0){
        if((2 < diff) || (diff < 5)) // 72M x 30.25ns < diff < 72M x 63.25ns
            bit = 1;
        else if(diff < 3)
            bit = 0;
    }else if(qi_FSKDepth == 1){
        if((4 < diff) || (diff < 7)) // 72M x 61.50ns < diff < 72M x 94.50ns
            bit = 1;
		else if(diff < 5)
            bit = 0;
    }else if(qi_FSKDepth == 2){
        if((8 < diff) || (diff < 12)) // 72M x 124.00ns < diff < 72M x 157.00ns
            bit = 1;
		else if(diff < 9)
            bit = 0;
    }else if(qi_FSKDepth == 3){
        if((17 < diff) || (diff < 21)) // 72M x 249.00ns < diff < 72M x 282.00ns
            bit = 1;
		else if(diff < 18)
            bit = 0;
    }

    if(ByteCnt == 0){
				
        if(bit) //start bit must be 0 /  start bit = 1 -> don't start FSK read byte
		{
			//PA9=0;
            return;
		}
    }else if(ByteCnt == 9){
			
        if(bit != FskParity){ //parity bit error -> initialize
            FskParity = 0;
            ByteCnt = 0;
            FskByte = 0;
            return;
        }
    }else if(ByteCnt == 10){
        if(bit == 1){ //stop bit must be 1
            //FskBuffer[FskBufferCnt++] = FskByte;
            FskBuffer[0] = FskByte;
        }else{
            FskParity = 0;
            ByteCnt = 0;
            FskByte = 0;
            InFskTransfer = 0;
            //NVIC_DisableIRQ(GPAB_IRQn);
            return;
        }
    }else if((ByteCnt > 0) && (ByteCnt < 9)){
        FskByte = FskByte | (bit << (ByteCnt - 1));
			
    }

    ByteCnt++;

}

void WaitFSK(){
    
    if(!ByteCnt){
        InFskTransfer = 1; //first chain
        //NVIC_EnableIRQ(GPAB_IRQn);
    }

    while(InFskTransfer);

    return;
}