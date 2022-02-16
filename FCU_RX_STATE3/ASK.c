#include <stdio.h>
#include "ASK.h"
//#include "board.h"
#include "NuMicro.h"




uint32_t AskState = 0;
uint8_t ByteOutCnt = 0;
uint8_t Parity = 0;
uint8_t InAskTransfer = 0;  // transfering ASK or Not
uint8_t AskBuffer[8];
uint8_t AskBufferCnt = 0;
uint32_t AskCnt = 0;
uint8_t AskHeader = 0;
uint8_t AskDataCnt = 0;
uint8_t AskCheckSum = 0;

extern int i;

extern uint32_t timerCount;
extern uint8_t AskStart;

extern uint8_t qi_signal_strength;
extern uint8_t qi_end_transfer_code;
extern uint8_t qi_holdoff_time_ms;
extern uint8_t qi_power_class;
extern uint8_t qi_maximum_power;
extern uint8_t qi_prop;
extern uint8_t qi_oob_flag;
extern uint8_t qi_count;
extern uint8_t qi_window_size;
extern uint8_t qi_window_offset;
extern uint8_t qi_neg;
extern uint8_t qi_FSKPolarity;
extern uint8_t qi_FSKDepth;
extern uint8_t qi_major_version;
extern uint8_t qi_minor_version;
extern uint16_t qi_manufacturer_code;
extern uint8_t qi_ext;
extern uint32_t qi_basic_device_identifier;
extern uint64_t qi_extended_device_identifier;
extern uint8_t qi_control_error_value;
extern uint16_t qi_received_power;
extern uint8_t qi_received_power_mode;
extern uint8_t qi_charge_status;
extern uint8_t qi_tx_last_packet_header;
extern uint8_t qi_quality_factor_mode;
extern uint8_t qi_ref_quality_factor;
extern uint8_t Subcmd;
extern uint8_t MacAddr[6];


void PB14Toggle(){
    if(PB14)
        PB14 = 0;
    else
        PB14 = 1;

    return;
}

void AskOut(){

    if(AskState == ASK_PREAM_STATE){
        
        PB14Toggle();
        if(AskCnt++ > PREAMCNT){
		    AskCnt = 0;
            AskState = ASK_HEADER_STATE;
		}
 
    }else if(AskState == ASK_HEADER_STATE){

        AskByteOut(AskHeader);
        if(InAskTransfer == 0){
            AskState = ASK_DATA_STATE;
            AskCheckSum = AskHeader;
        }
    }else if(AskState == ASK_DATA_STATE){

        AskByteOut(AskBuffer[AskBufferCnt]);
        if(InAskTransfer == 0){
            AskCheckSum ^= AskBuffer[AskBufferCnt++];
            if((AskBufferCnt == AskDataCnt))
                AskState = ASK_CHECKSUM_STATE;
        }

    }else if(AskState == ASK_CHECKSUM_STATE){
        AskByteOut(AskCheckSum);
		if(InAskTransfer == 0){
            AskBufferCnt = 0;
            AskStart = 0;
            timerCount = 0;  
            AskState = ASK_PREAM_STATE;
            NVIC_DisableIRQ(TMR0_IRQn);
        }
    }else{
        AskStart = 0;
        AskState = 0;
        AskBufferCnt = 0;
        InAskTransfer = 0;
        timerCount = 0;
    }



}

void AskByteOut(uint8_t Data){

    if(ByteOutCnt%2 == 0){
        PB14Toggle();
        if(ByteOutCnt == 0){
            InAskTransfer = 1;
        }
        
    }else{
        if(ByteOutCnt == 1){
            //start bit 0 -> Do nothing
					
        }else if(ByteOutCnt == 19){
            //parity bit
            if(!Parity){
                PB14Toggle();
            }
        }else if(ByteOutCnt == 21){
            //end bit 1
            PB14Toggle();
            InAskTransfer = 0;
            ByteOutCnt = 0;
            Parity = 0;
            timerCount = 0;
						if(AskHeader==0x68){
						
						}
            return;
        }else{
					
            if(Data >> (((ByteOutCnt - 3)/2)) & 0x01){ //bit 1
                PB14Toggle();
                Parity ^= 1;
									
            }
        }
    }

    ByteOutCnt++;

    return;
}

void DataPacketStart(uint8_t header){
		
    switch(header){
			
        case 0x01:
            AskBuffer[0] = qi_signal_strength;
            AskDataCnt = 1;
            break;
        case 0x02:
            AskBuffer[0] = qi_end_transfer_code;
            AskDataCnt = 1;
            break;
        case 0x06:
            AskBuffer[0] = qi_holdoff_time_ms;
            AskDataCnt = 1;
            break;
        case 0x51:
            AskBuffer[0] = (qi_power_class & 0x03) << 6;
            AskBuffer[0] = AskBuffer[0] | (qi_maximum_power & 0x3F);
            AskBuffer[1] = 0x00;
            AskBuffer[2] = (qi_prop & 0x01) << 7;
            AskBuffer[2] = AskBuffer[2] | ((qi_oob_flag & 0x01) << 4);
            AskBuffer[2] = AskBuffer[2] | (qi_count & 0x07);
            AskBuffer[3] = (qi_window_size & 0x1F) << 3;
            AskBuffer[3] = AskBuffer[3] | (qi_window_offset & 0x07);
            AskBuffer[4] = (qi_neg & 0x01) << 7;
            AskBuffer[4] = AskBuffer[4] | ((qi_FSKPolarity & 0x01) << 6);
            AskBuffer[4] = AskBuffer[4] | ((qi_FSKDepth & 0x03) << 4);
            AskDataCnt = 5;
            break;
        case 0x71:
            AskBuffer[0] = (qi_major_version & 0x0F) << 4;
            AskBuffer[0] = AskBuffer[0] | (qi_minor_version & 0x0F);
            AskBuffer[1] = qi_manufacturer_code >> 8;
            AskBuffer[2] = qi_manufacturer_code & 0xFF;
            AskBuffer[3] = qi_ext << 7;
            AskBuffer[3] = AskBuffer[3] | ((qi_basic_device_identifier >> 24) & 0x7F);
            AskBuffer[4] = (qi_basic_device_identifier >> 16) & 0xFF;
            AskBuffer[5] = (qi_basic_device_identifier >> 8) & 0xFF;
            AskBuffer[6] = qi_basic_device_identifier & 0xFF;
            AskDataCnt = 7;
            break;
        case 0x81:
            AskBuffer[0] = (qi_extended_device_identifier >> 56) & 0xFF;
            AskBuffer[1] = (qi_extended_device_identifier >> 48) & 0xFF;
            AskBuffer[2] = (qi_extended_device_identifier >> 40) & 0xFF;
            AskBuffer[3] = (qi_extended_device_identifier >> 32) & 0xFF;
            AskBuffer[4] = (qi_extended_device_identifier >> 24) & 0xFF;
            AskBuffer[5] = (qi_extended_device_identifier >> 16) & 0xFF;
            AskBuffer[6] = (qi_extended_device_identifier >> 8) & 0xFF;
            AskBuffer[7] = qi_extended_device_identifier & 0xFF;
            AskDataCnt = 8;
            break;
        case 0x03:
            AskBuffer[0] = qi_control_error_value;
            AskDataCnt = 1;
            break;
        case 0x04:
            AskBuffer[0] = (qi_received_power >> 8) & 0xFF;
            AskDataCnt = 1;
            break;
        case 0x31:
            AskBuffer[0] = qi_received_power_mode & 0x07;
            AskBuffer[1] = qi_received_power >> 8;
            AskBuffer[2] = qi_received_power & 0xFF;
            AskDataCnt = 3;
            break;
        case 0x05:
            AskBuffer[0] = qi_charge_status;
            AskDataCnt = 1;
            break;
        case 0x09:
            AskBuffer[0] = 0;
            AskDataCnt = 1;
            break;
        case 0x07:
						
            AskBuffer[0] = 0x30;
            //AskBuffer[0] = 0x31;
            //AskBuffer[0] = 0xFF;
            AskDataCnt = 1;
            break;
        case 0x22:
            AskBuffer[0] = qi_quality_factor_mode & 0x03;
            AskBuffer[1] = qi_ref_quality_factor;
            AskDataCnt = 2;
            break;
        case 0x20:
            break;
        case 0x68:
					
						//printf("MAC : ");
						
						for(i=0; i<6; i++){
								AskBuffer[i] = MacAddr[i];
							//printf("%02X : %02X \r\n",AskBuffer[i], MacAddr[i]);
						}
						//printf("\r\n");
            AskDataCnt = 6;
						
            break;
        default:
            break;
    }


		NVIC_EnableIRQ(TMR0_IRQn);
    AskStart = 1;
		TIMER0->TCSR = 0x6A000000;
    return;

}

void MyDelay(int count){
		int timeTMP=TIMER0->TDR;
	
		while(TIMER0->TDR < timeTMP + count){
			
		}
		
		return;
}