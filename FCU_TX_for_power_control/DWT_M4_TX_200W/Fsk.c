#include "Fsk.h"
#include "WPCQiTx.h"

uint8_t g_qi_tx_buffer[QI_TX_BUFFER_SIZE];  /* buffer contains messages Power Transmitter responds to Power Receiver */
uint8_t g_qi_tx_size = 0;                   /* byte number of message Power Transmitter responds to Power Receiver */

static uint8_t g_qi_tx_bit_index = 0;              /* index of bit in current sending byte */
static uint8_t g_qi_tx_byte_index = 0;             /* index of byte in current buffer */

static uint8_t g_qi_bit_value = 0;                 /* buffer contains value of one bit of a byte ( 1 or 0) */
static uint8_t g_tx_parity = 0;                    /* parity bit value */

static uint16_t g_periods=0;                         /* get the period of timer A(period of Power Signal) to start modulating FSK */

extern uint16_t g_qi_tx_time_counter;
unsigned char fsk_delta, fsk_delta_20=2;
 
static qi_Tx_State g_qi_tx_state;					/* is used to be bit state when we modulate FSK signal */

/****************************************************************************************************/
/*																									*/
/*			Fsk_vStop																              */ 
/*																									*/
/*																									*/
/****************************************************************************************************/
static inline void Fsk_vStop(void)
{ 

	EPWM0->PERIOD[0] = EPWM0->PERIOD[1] = EPWM1->PERIOD[0] = EPWM1->PERIOD[1] = EPWM0->PERIOD[4] = EPWM0->PERIOD[5] = EPWM1->PERIOD[4] = EPWM1->PERIOD[5] = g_periods;    
	NVIC_DisableIRQ(EPWM0P2_IRQn);	  /* Enable PWMA NVIC interrupt */
	NVIC_EnableIRQ(TMR0_IRQn);
}

/****************************************************************************************************/
/*																									*/
/*			Fsk_vInit																              */ 
/*																									*/
/*																									*/
/****************************************************************************************************/
void Fsk_vInit(qi_Tx_State mes_type)
{ 
    g_qi_tx_state = mes_type; 
    g_qi_tx_byte_index = 0;
    g_qi_tx_bit_index = 0;
    g_qi_tx_time_counter = FULL_BIT - 1;
	g_periods = EPWM0->PERIOD[4];
	
    NVIC_DisableIRQ(TMR0_IRQn);
    NVIC_EnableIRQ(EPWM0P2_IRQn);    /* Enable PWMA NVIC interrupt */
}  

 
 /****************************************************************************************************/
 /* 																								 */
 /* 		 Fsk_vUpdate																			   */ 
 /* 																								 */
 /* 																								 */
 /****************************************************************************************************/
 void Fsk_vUpdate()
 {
		 if( EPWM0->PERIOD[4] == g_periods)
		 {
				 if(qi_FSKPolarity == 1)
				 {
						 EPWM0->PERIOD[0] = EPWM0->PERIOD[1] = EPWM1->PERIOD[0] = EPWM1->PERIOD[1] = EPWM0->PERIOD[4] = EPWM0->PERIOD[5] = EPWM1->PERIOD[4] = EPWM1->PERIOD[5] = g_periods + fsk_delta;//G_QI_FSK_DELTA[qi_FSKDepth];//11;//G_QI_FSK_DELTA[qi_FSKDepth]; 
				 }
				 else
						 EPWM0->PERIOD[0] = EPWM0->PERIOD[1] = EPWM1->PERIOD[0] = EPWM1->PERIOD[1]=EPWM0->PERIOD[4] = EPWM0->PERIOD[5] = EPWM1->PERIOD[4] = EPWM1->PERIOD[5] = g_periods - fsk_delta;//G_QI_FSK_DELTA[qi_FSKDepth];//11;//G_QI_FSK_DELTA[qi_FSKDepth];	 
		 }
		 else
				 EPWM0->PERIOD[0] = EPWM0->PERIOD[1] = EPWM1->PERIOD[0] = EPWM1->PERIOD[1]= EPWM0->PERIOD[4] = EPWM0->PERIOD[5] = EPWM1->PERIOD[4] = EPWM1->PERIOD[5] = g_periods; 
 
 }


/****************************************************************************************************/
/*																									*/
/*			Fsk_dataEndCheck 																			  */ 
/*																									*/
/*																									*/
/****************************************************************************************************/
static inline void Fsk_dataEndCheck( void )
{
    g_qi_tx_state = QI_TX_START; 
    if (++g_qi_tx_byte_index == g_qi_tx_size) 
        g_qi_tx_state = QI_TX_END_PACKET; 
}
  

/****************************************************************************************************/
/*																									*/
/*			Fsk_vStart 																			  */ 
/*																									*/
/*																									*/
/****************************************************************************************************/
void Fsk_vStart(uint16_t bit_machine)
{
     
    switch(bit_machine) 
    { 
	    case HALF_BIT:                    /* 256 periods after start bit */
	    {
	        switch (g_qi_tx_state) 
	        {
	        
	        case QI_TX_START: 
	            g_qi_tx_state = QI_TX_DATA;
	            g_tx_parity  = 0;
	            g_qi_tx_bit_index = 0;          
	            break;
				
	        case QI_TX_DATA:

	            g_qi_bit_value = (g_qi_tx_buffer[g_qi_tx_byte_index] >> g_qi_tx_bit_index) & 0x1;
	            g_tx_parity ^= g_qi_bit_value;

	            if((BitValue)g_qi_bit_value == FBIT1) 
	                Fsk_vUpdate();

	            if (++g_qi_tx_bit_index == 8) /* number of bits in one byte */
	            {  
	                g_qi_tx_state = QI_TX_PARITY; 
	                g_qi_tx_bit_index = 0;
	            }             
	            break;
				
	        case QI_TX_PARITY:
	            if((BitValue)g_tx_parity == FBIT1) 
	            {
	                Fsk_vUpdate();
	            }

	            g_qi_tx_state = QI_TX_STOP; 
	            break;
				
	        case QI_TX_STOP:
	            Fsk_vUpdate(); 
	            g_qi_tx_state = QI_TX_NEXT;            
	            break;
				
	        case QI_TX_END_PACKET:
	            Fsk_vStop();
	            break;
				
	        case QI_TX_ACK:
	            Fsk_vUpdate();
	            g_qi_tx_bit_index++;            
	            break;
				
	        case QI_TX_NACK:
	            g_qi_tx_bit_index++;            
	            break;
				
	        case QI_TX_ND:
	            if((BitValue)g_tx_parity == FBIT0) 
	                Fsk_vUpdate();
				
	            g_tx_parity ^= 0x1;
	            g_qi_tx_bit_index++;            
	            break;

			default:
	            break;     
	        } 
    	} 
    	break;
		
    case FULL_BIT:         /* 512 periods of Power Signal */
        Fsk_vUpdate();

        if (g_qi_tx_state == QI_TX_NEXT) 
            Fsk_dataEndCheck();
    
        else if (g_qi_tx_bit_index == 9) /* end of pattern message */
            Fsk_vStop();
        break;
		
    default:      
        break;
    }
}  


