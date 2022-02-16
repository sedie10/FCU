/*
FCU RX 200W 
*/

#include <stdio.h>
#include <string.h>
//#include "board.h"
#include "NuMicro.h"
#include "ASK.h"
#include "FSK.h"
#include "ISR.h"
#include "BLE.h"

#define PLL_CLOCK           72000000    //72MHz(MAX)
#define UART0_BAUDRATE      115200      //COM
#define UART2_BAUDRATE      115200      //BLE

#define ACK                 0xFF
#define NACK                0x00
#define ND                  0x55

#define ADC_CHANNEL_VRECT   0
#define ADC_CHANNEL_ISENSE  1
#define ADC_CHANNEL_TEMP    2
#define ADC_CHANNEL_FOD     3

extern void TMR0_IRQHandler(void);

extern uint8_t FskBuffer[8];
extern uint8_t FskBufferCnt;

extern uint8_t uart_buffer[128];
extern int UartBufferCnt;

extern uint8_t BleComplete;

uint8_t BleBuffer[100];

void SYS_Init1(void);
void SYS_Init2(void);
void UART0_Init(void);
void UART1_Init(void);
void UART2_Init(void);
void StateMachine1(void);
void StateMachine2(void);
void StateMachine3(void);
void BLEbufferInit();


int32_t ReadADC(uint32_t u32Channel);

char tmpBuf[100];
char ch;
int i, j;
int length = 100;
uint8_t CmdReadMacAddr[2] = {0x7E, 0x01};
uint8_t Cmd0731[3] = {0x7D, 0x07, 0x31};
uint8_t CEPbuffer[4] = {0x7D, 0x03, 0x00};

volatile uint32_t timerCount = 0;
volatile uint8_t AskStart = 0;
uint8_t qi_signal_strength = 0x10;
uint8_t qi_end_transfer_code = 0;
uint8_t qi_holdoff_time_ms = 0;
uint8_t qi_power_class = 0;
uint8_t qi_maximum_power = 0;
uint8_t qi_prop = 0;
uint8_t qi_oob_flag = 0;
uint8_t qi_count = 0;
uint8_t qi_window_size = 0;
uint8_t qi_window_offset = 0;
uint8_t qi_neg = 0;
uint8_t qi_FSKPolarity = 0;
uint8_t qi_FSKDepth = 0;
uint8_t qi_major_version = 0;
uint8_t qi_minor_version = 0;
uint16_t qi_manufacturer_code = 0;
uint8_t qi_ext = 0;
uint32_t qi_basic_device_identifier = 0;
uint64_t qi_extended_device_identifier = 0;
uint8_t qi_control_error_value = 0;
uint16_t qi_received_power = 0;
uint8_t qi_received_power_mode = 0;
uint8_t qi_charge_status = 0;
uint8_t qi_tx_last_packet_header = 0;
uint8_t qi_quality_factor_mode = 0;
uint8_t qi_ref_quality_factor = 0;
uint8_t Subcmd = 0;
uint8_t MacAddr[6];

extern uint8_t AskHeader;

int main(void){
	
	int a, b;
	
    //register setting
    //SYS_UnlockReg();
    SYS->REGWRPROT = 0x59;
    SYS->REGWRPROT = 0x16;
    SYS->REGWRPROT = 0x88;
	
    SYS_Init1();
    
    //SYS_LockReg();
    SYS->REGWRPROT = 0x00;

    //GPIO SET MODE
    PA -> PMD = (PB -> PMD & ~(0x3 << 14)) | (0x1UL << 14);
    PB -> PMD = (PB -> PMD & ~(0x3 << 26)) | (0x1UL << 26);
    // tiemr0 = 72MHz / (1+prescaler)
    TIMER_SET_PRESCALE_VALUE(TIMER0, 0);
    TIMER_SET_CMP_VALUE(TIMER0, 0xFFFFFF);

    //TIMER_EnableEventCounter(TIMER0, TIMER_COUNTER_RISING_EDGE);
    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 4000);
    TIMER_EnableInt(TIMER0);

    /*timer0 start*/
    TIMER_Start(TIMER0);
		AskHeader = 0x01;
    DataPacketStart(AskHeader);


//-------------------------------------------------------------------
    SYS_UnlockReg();
    SYS_Init2();
    SYS_LockReg();

    //UART init
    UART0_Init();
	UART_WRITE(UART0, '0');
    UART2_Init();

		
	printf("\r\ncounter1 : %d \r\n", TIMER0->TDR);

    /*GPIO setting and interrupt*/

    //FSK interrupt
    GPIO_SetMode(PB, BIT3, GPIO_PMD_INPUT);
    GPIO_EnableInt(PB, 3, GPIO_INT_RISING); // interrupt with rising edge

    //GPIO_EnableInt(PB, 3, GPIO_INT_BOTH_EDGE); // interrupt with rising edge and falling edge
    NVIC_EnableIRQ(GPAB_IRQn);


    GPIO_SetMode(PA, BIT9, GPIO_PMD_OUTPUT); // for test - green LED , active low

    while(AskStart == 1){
         
    }
		
    while(1){
        StateMachine2();
    }

}

void SYS_Init1(void){

     /* Enable Internal RC 22.1184MHz clock */
    CLK_EnableXtalRC(CLK_PWRCON_OSC22M_EN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_OSC22M_STB_Msk);
	
    /*Switch HCLK clock source to Internal RC and HCLK source divide 1*/
    CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_HIRC, CLK_CLKDIV_HCLK(1));

	    /* Enable external XTAL 12MHz clock */
    CLK_EnableXtalRC(CLK_PWRCON_XTL12M_EN_Msk);//add

    /* Waiting for external XTAL clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_XTL12M_STB_Msk);//add

    /*Set core clock as PLL_CLOCK form PLL*/
    CLK_SetCoreClock(PLL_CLOCK);

    /*Enable Timer0 module clock*/
    CLK_EnableModuleClock(TMR0_MODULE);

    /*Select Timer0 module clock source*/
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0_S_HCLK, NULL);
    
    /*Enable ADC module clock*/
    //CLK_EnableModuleClock(ADC_MODULE);

    /*select ADC module clock source : 22.1184 / 7 MHz*/
    //CLK_SetModuleClock(ADC_MODULE, CLK_CLKSEL1_ADC_S_HIRC, CLK_CLKDIV_ADC(7));

    /*
    ** GPIO setting(IO multi function)
    */


    /*Set PB multi-function pins for TM0*/
    SYS->GPB_MFP &= ~SYS_GPB_MFP_PB15_Msk;
    SYS->GPB_MFP |= SYS_GPB_MFP_PB15_TM0;
    SYS->ALT_MFP &= ~SYS_ALT_MFP_PB15_Msk;
    SYS->ALT_MFP |= SYS_ALT_MFP_PB15_TM0;

    SYS->GPB_MFP &= ~SYS_GPB_MFP_PB8_Msk;
    SYS->GPB_MFP |= SYS_GPB_MFP_PB8_TM0;
    SYS->ALT_MFP &= ~SYS_ALT_MFP_PB8_Msk;
    SYS->ALT_MFP |= SYS_ALT_MFP_PB8_TM0;


}

void SYS_Init2(void){
	

	
	/*Enable UART module clock*/
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(UART2_MODULE);

    /*Select UART module clock source*/
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_HXT, CLK_CLKDIV_UART(1));
    CLK_SetModuleClock(UART2_MODULE, CLK_CLKSEL1_UART_S_HXT, CLK_CLKDIV_UART(1));
	
	
	/*Set PB multi-function pins for UART0 RXD, TXD*/
    SYS->GPB_MFP &= ~(SYS_GPB_MFP_PB0_Msk|SYS_GPB_MFP_PB1_Msk);
    SYS->GPB_MFP |= SYS_GPB_MFP_PB0_UART0_RXD | SYS_GPB_MFP_PB1_UART0_TXD;

    /*Set PB multi-function pins for UART2 RXD, TXD*/
    SYS->GPB_MFP &= ~(SYS_GPB_MFP_PB9_Msk|SYS_GPB_MFP_PB10_Msk);
    SYS->GPB_MFP |= (SYS_GPB_MFP_PB9_UART2_TXD | SYS_GPB_MFP_PB10_UART2_RXD);
	
	SYS->ALT_MFP &= ~SYS_ALT_MFP_PB9_TM1;
    SYS->ALT_MFP |= SYS_ALT_MFP_PB9_UART2_TXD;
		
	SYS->ALT_MFP &= ~SYS_ALT_MFP_PB10_TM2;
    SYS->ALT_MFP |= SYS_ALT_MFP_PB10_UART2_RXD;
	
}

void UART0_Init(void){

    SYS_ResetModule(UART0_RST);

    UART_Open(UART0, UART0_BAUDRATE);

}

void UART1_Init(void){

    SYS_ResetModule(UART1_RST);

    UART_Open(UART1, 115200);

}

void UART2_Init(void){

    SYS_ResetModule(UART2_RST);

    UART_Open(UART2, UART2_BAUDRATE);

}

void StateMachine1(void){//BPP

//identification transfer
    AskHeader = 0x71;
    
    qi_major_version = 0x01;
    qi_minor_version = 0x02;
    qi_manufacturer_code = 0x1D99;
    qi_ext = 0;
    qi_basic_device_identifier = 0x00010021;
    
    DataPacketStart(AskHeader);
    
    while(AskStart == 1){
         
    }
    
    /*
    memset(tmpBuf, 0, 100);
		sprintf(tmpBuf, "[%d] 0x71\r\n", j++);
    print();
	*/	
//configuration transfer
    AskHeader = 0x51;
    
    qi_power_class = 0x01;
    qi_maximum_power = 0x28;
    qi_prop = 0;
		qi_oob_flag = 0x00;
    qi_count = 0;
    qi_window_offset = 0x04;
    qi_window_size = 0x10;
    qi_neg = 0;
    qi_FSKPolarity = 0x00;
    qi_FSKDepth = 0x03;

    DataPacketStart(AskHeader);

    while(AskStart == 1){
         
    }
/*
    memset(tmpBuf, 0, 100);
		sprintf(tmpBuf, "[%d] 0x51\r\n", j++);
    print();
*/		
// power transfer request

// power transfer -> LED  *****for test

    AskHeader = 0x03;

    qi_control_error_value = 0x20;

    DataPacketStart(AskHeader);

    while(AskStart == 1){
         
    }

    /*
		memset(tmpBuf, 0, 100);
		sprintf(tmpBuf, "[%d] 0x03\r\n", j++);
    print();
    */



    while(1);
}

void StateMachine2(void){

    int i, b;

    //Get BLE MAC Address
    BLEenable();
    sendDataToBle(CmdReadMacAddr, sizeof(CmdReadMacAddr));


//identification transfer
    AskHeader = 0x71;
    
    qi_major_version = 0x01;
    qi_minor_version = 0x02;
    qi_manufacturer_code = 0x1D99;
    qi_ext = 0;
    qi_basic_device_identifier = 0x00010021;
    
    DataPacketStart(AskHeader);
	while(AskStart == 1){
         
    }

//configuration transfer
    AskHeader = 0x51;

    qi_power_class = 0x01;
    qi_maximum_power = 0x28;
    qi_prop = 0;
	qi_oob_flag = 0x01;
    qi_count = 0;
    qi_window_offset = 0x04;
    qi_window_size = 0x10;
    qi_neg = 1;
    qi_FSKPolarity = 0x00;
    qi_FSKDepth = 0x03;

    DataPacketStart(AskHeader);
    while(AskStart == 1){

    }

//transmit MAC ADDR		
    AskHeader = 0x68;

	//while(BleComplete != 1)		
				
    for(i=0; i<6; i++){
				MacAddr[i] = uart_buffer[i+2];
    }
		BleComplete = 0;    	
		MyDelay(3000);
		UartBufferCnt=0;
    DataPacketStart(AskHeader);
		//BLEenable();
    while(AskStart == 1){

    }
		
//wait ACK

		TIMER_Delay(TIMER0, 10000);
    AskHeader = 0x07;
		DataPacketStart(AskHeader);
		while(AskStart == 1){
				 
		}
		
		while(uart_buffer[1] != 0x10);
		
		UartBufferCnt = 0;
		BleComplete = 0;
		while(1){
			sendDataToBle(Cmd0731, sizeof(Cmd0731));		
			if(uart_buffer[1] == 0x31){
				break;
			}
			TIMER_Delay(TIMER0, 100000);
		}
		
//send specific request 0x20 0x00
		TIMER_Delay(TIMER0, 100000);
		UartBufferCnt = 0;
		BleComplete = 0;
		BleBuffer[0] = 0x7D;
    BleBuffer[1] = 0x20;
    BleBuffer[2] = 0x00;
    BleBuffer[3] = 0x01;
		sendDataToBle(BleBuffer, 4);	

//send control error packet
		TIMER_Delay(TIMER0, 100000);
/*		
		BleBuffer[0] = 0x7D;
    BleBuffer[1] = 0x03;
    BleBuffer[2] = 0x00;
*/

		sendDataToBle(CEPbuffer, sizeof(CEPbuffer));
		

		
		BleBuffer[0] = 0x7D;
		BleBuffer[1] = 0x31;
		BleBuffer[2] = 0x28;
		BleBuffer[3] = 0x00;
		BleBuffer[4] = 0x01;
		i=0;
		while(1){
			if(i%20 == 0){
				sendDataToBle(BleBuffer, 5);
			}else{
				sendDataToBle(CEPbuffer, sizeof(CEPbuffer));
				while(1){
					if(uart_buffer[1] == 0x7C){
						break;
					}
					TIMER_Delay(TIMER0, 100000);
				}
			}
			i++;
			TIMER_Delay(TIMER0, 500000);
		}
}

void StateMachine3(void){

    int32_t ADCVal_VRECT = 0;
    int32_t ADCVal_ISENSE = 0;

//identification transfer
    AskHeader = 0x71;
    
    qi_major_version = 0x01;
    qi_minor_version = 0x02;
    qi_manufacturer_code = 0x1D99;
    qi_ext = 0;
    qi_basic_device_identifier = 0x00010021;
    
    DataPacketStart(AskHeader);
	while(AskStart == 1){
         
    }

//configuration transfer
    AskHeader = 0x51;

    qi_power_class = 0x01;
    qi_maximum_power = 0x28;
    qi_prop = 0;
		qi_oob_flag = 0x01;
    qi_count = 0;
    qi_window_offset = 0x04;
    qi_window_size = 0x10;
    qi_neg = 1;
    qi_FSKPolarity = 0x00;
    qi_FSKDepth = 0x03;


		
    DataPacketStart(AskHeader);
    while(AskStart == 1){
         
    }


//wait ACK
    WaitFSK();
		
    if(FskBuffer[0] != ACK)
      return;
//PA8=0;
//transmit MAC ADDR
    AskHeader = 0x68;
    uart2Bufferinit();
    sendDataToBle(CmdReadMacAddr, sizeof(CmdReadMacAddr));
    while(uart_buffer[UartBufferCnt] != 0x7F); // wait BLE EOT
    memcpy(MacAddr, &uart_buffer[UartBufferCnt-7], 6);
    uart2Bufferinit();

    DataPacketStart(AskHeader);
    while(AskStart == 1){
         
    }
//wait ACK
    WaitFSK();

    if(FskBuffer[0] != ACK)
      return;

//send request 0x07 0x30 (Identification)
    AskHeader = 0x07;
    
    DataPacketStart(AskHeader);
    while(AskStart == 1){
         
    }
//receive BLE Uart
    while(uart_buffer[UartBufferCnt] != 0x7F);
    uart2Bufferinit();

//send request 0x07 0x31 (Configuration)
    BleBuffer[0] = 0x7D;
    BleBuffer[1] = 0x07;
    BleBuffer[2] = 0x31;
    sendDataToBle(BleBuffer, 3);
    BLEbufferInit();

//receive BLE Uart
    while(uart_buffer[UartBufferCnt] != 0x7F);
    uart2Bufferinit();

//send specific request 0x20 0x00
    BleBuffer[0] = 0x7D;
    BleBuffer[1] = 0x20;
    BleBuffer[2] = 0x00;
    BleBuffer[3] = 1;
    sendDataToBle(BleBuffer, 4);
    BLEbufferInit();

//receive BLE Uart
    while(uart_buffer[UartBufferCnt] != 0x7F);
    uart2Bufferinit();

//send error packet for power transfer
    BleBuffer[0] = 0x7D;
    BleBuffer[1] = 0x03;
    BleBuffer[2] = 0x00;
    sendDataToBle(BleBuffer, 3);
    BLEbufferInit();

    while(1){
        /*wait 1s time and receive BLE Uart?*/
        /****************/
        /****************/

        //Read ADC Data
        ADCVal_VRECT = ReadADC(ADC_CHANNEL_VRECT);
        ADCVal_ISENSE = ReadADC(ADC_CHANNEL_ISENSE);

        //send BLE
        BleBuffer[0] = 0x7D;
        BleBuffer[1] = 0x31;
        BleBuffer[2] = 0x00;
        #if 1
        BleBuffer[3] = 0x00;
        BleBuffer[4] = 0x00;

        //print
        memset(tmpBuf, 0, 100);
        sprintf(tmpBuf, "VRECT : %d\r\n", ADCVal_VRECT);
				length = sizeof(tmpBuf);
        for( i = 0; i<length; i++ )
        {	   
            ch = tmpBuf[i];
            UART_WRITE(UART0,ch);
        }

        #else
        BleBuffer[3] = ;
        BleBuffer[4] = ;
        #endif
        sendDataToBle(BleBuffer, 5);
        BLEbufferInit();

        /*wait ack*/
        while(uart_buffer[UartBufferCnt] != 0x7F);
        uart2Bufferinit();

    }

}

int32_t ReadADC(uint32_t u32Channel){

    int32_t ConversionData = 0;

/*ADC power ON*/
    ADC_POWER_ON(ADC);

/*ADC OPEN*/
    ADC_Open(ADC, ADC_ADCR_DIFFEN_SINGLE_END, ADC_ADCR_ADMD_CONTINUOUS, 0xF);

/*Clear INT, for safe - not need*/
    ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT);

/*Start Analog to Digital conversion*/
    ADC_START_CONV(ADC);

/*Wait conversion*/
    while(!ADC_GET_INT_FLAG(ADC, ADC_ADF_INT));

/*Clear INT, for safe - not need*/
    ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT);

/*Get conversion data*/
    ConversionData = ADC_GET_CONVERSION_DATA(ADC, u32Channel);

/*Stop conversion*/
    ADC_STOP_CONV(ADC);

    return ConversionData;

}


void BLEbufferInit(){
    memset(BleBuffer, 0, sizeof(BleBuffer));
    return;
}