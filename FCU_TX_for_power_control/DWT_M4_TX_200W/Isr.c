#include "Isr.h"
#include "NuMicro.h"
#include "WPCQiTx.h"
#include "Fsk.h"
#include "led.h"
#include "Pwm.h"

uint16_t g_qi_tx_time_counter = 0;   /* the variable is used to counter number of periods of Timer A( period of Power  Signal) */
static int ticks_1ms = 0;
static int ticks_250ms = 0;
unsigned int Rth_threshold_buf, Rth_HYS_threshold_buf;
extern uint16_t delay_ms;
static uint16_t /*rth_value,*/ vbus_value;

#ifdef Q_MEASURE			
extern __ramfunc uint8_t quality_factor_measure_test(void);       /*!< Returns non-zero if something conducting is on pad */
extern uint8_t q_measure_state;
#endif

extern uint8_t q_self_resonance_test_finished;
extern uint8_t dcdc_enable;
extern qitxStateResetReason_t reasonReset;

static uint8_t qitx_UvCount = 0;
static uint8_t qitx_OvCount = 0;

#ifdef USE_ECAP
/****************************************************************************************************/
/*																									*/
/*						ECAP0_IRQHandler																	*/ 
/*																									*/
/*																									*/
/****************************************************************************************************/
void ECAP0_IRQHandler(void)
{    
	uint32_t u32Status;

	/* Get input Capture status */    
	u32Status = ECAP_GET_INT_STATUS(ECAP0);   
	
	/* Check input capture channel 0 flag */    
	if((u32Status & ECAP_STATUS_CAPTF0_Msk) == ECAP_STATUS_CAPTF0_Msk)    
	{        
		/* Clear input capture channel 0 flag */        
//		ECAP_CLR_CAPTURE_FLAG(ECAP0, ECAP_STATUS_CAPTF0_Msk);    
		ECAP_CLR_CAPTURE_FLAG(ECAP0, ECAP_STATUS_CAPTF0_Msk | ECAP_STATUS_CAPCMPF_Msk | ECAP_STATUS_CAPOVF_Msk);    
		/* Get input capture counter hold value */        
		ASK_vDecode((uint32_t)(ECAP0->HLD0 & 0x00FFFFFF));
	}    
	
	/* Check input capture compare-match flag */    
	else if((u32Status & ECAP_STATUS_CAPCMPF_Msk) == ECAP_STATUS_CAPCMPF_Msk)    
	{        
		/* Clear input capture compare-match flag */        
		ECAP_CLR_CAPTURE_FLAG(ECAP0,ECAP_STATUS_CAPCMPF_Msk);    

		ASK_vDecode((uint32_t)( ECAP0_TIMEOUT & 0x00FFFFFF));
	}   
	/* Check input capture overflow flag */
	if((u32Status & ECAP_STATUS_CAPOVF_Msk) == ECAP_STATUS_CAPOVF_Msk)
	{
			/* Clear input capture overflow flag */
			ECAP_CLR_CAPTURE_FLAG(ECAP0,ECAP_STATUS_CAPOVF_Msk);
	}	
}
#endif



#ifdef ECAP_Q_FACTOR

extern void quality_factor_measure_freq(uint32_t q_resonance_time );


/****************************************************************************************************/
/*																									*/
/*						ECAP1_IRQHandler																*/ 
/*																									*/
/*																									*/
/****************************************************************************************************/

void ECAP1_IRQHandler(void)
{    
	uint32_t u32Status;

	/* Get input Capture status */    
	u32Status = ECAP_GET_INT_STATUS(ECAP1);   
	
	/* Check input capture channel 0 flag */    
	if((u32Status & ECAP_STATUS_CAPTF0_Msk) == ECAP_STATUS_CAPTF0_Msk)    
	{        
		/* Clear input capture channel 0 flag */        
//		ECAP_CLR_CAPTURE_FLAG(ECAP0, ECAP_STATUS_CAPTF0_Msk);    
		ECAP_CLR_CAPTURE_FLAG(ECAP1, ECAP_STATUS_CAPTF0_Msk | ECAP_STATUS_CAPCMPF_Msk | ECAP_STATUS_CAPOVF_Msk);    
		/* Get input capture counter hold value */        
		quality_factor_measure_freq((uint32_t)(ECAP1->HLD0 & 0x00FFFFFF));
	}    
	
	/* Check input capture compare-match flag */    
	else if((u32Status & ECAP_STATUS_CAPCMPF_Msk) == ECAP_STATUS_CAPCMPF_Msk)    
	{        
		/* Clear input capture compare-match flag */        
		ECAP_CLR_CAPTURE_FLAG(ECAP1,ECAP_STATUS_CAPCMPF_Msk);    

		quality_factor_measure_freq((uint32_t)( ECAP1_TIMEOUT & 0x00FFFFFF));
	}   
	/* Check input capture overflow flag */
	if((u32Status & ECAP_STATUS_CAPOVF_Msk) == ECAP_STATUS_CAPOVF_Msk)
	{
			/* Clear input capture overflow flag */
			ECAP_CLR_CAPTURE_FLAG(ECAP1,ECAP_STATUS_CAPOVF_Msk);
	}	
}

#endif

/****************************************************************************************************/
/*																									*/
/*						TMR0_IRQHandler																*/ 
/*																									*/
/*																									*/
/****************************************************************************************************/
void TMR0_IRQHandler(void)
{
#if 0 //orig	
//    if(TIMER_GetCaptureIntFlag(TIMER0) == 1) 
	{
		/* Clear Timer0 capture interrupt flag */
		TIMER_ClearCaptureIntFlag(TIMER0);
		TIMER0->TCSR |= TIMER_TCSR_CRST_Msk;
		TIMER0->TCSR |= TIMER_TCSR_CEN_Msk;
		qi_comm_state_machine((uint32_t)(TIMER0->TCAP & 0x00FFFFFF));
    }
#endif
//	static int cnt = 0; 	 
//	static uint32_t t0; 	 
	TIMER_ClearCaptureIntFlag(TIMER0);		

//	TIMER0->TCSR |= TIMER_TCSR_CRST_Msk;
//	TIMER0->TCSR |= TIMER_TCSR_CEN_Msk;

//	t0 = TIMER_GetCaptureData(TIMER0);				
//	qi_comm_state_machine((uint32_t)(TIMER0->TCAP & 0x00FFFFFF));
//	qi_comm_state_machine((uint32_t)(t0 & 0x00FFFFFF));

}


/*---------------------------------------------------------------------------------------------------------*/
/* Define global variables and constants                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint32_t g_u32AdcIntFlag;

/****************************************************************************************************/
/*																									*/
/*						ADC_IRQHandler																*/ 
/*																									*/
/*																									*/
/****************************************************************************************************/

void ADC_IRQHandler(void)
{
#if 1 //orig
    g_u32AdcIntFlag = 1;
//    ADC->ADSR = ADC_ADSR_ADF_Msk;      /* clear the A/D conversion flag */
#endif //orig	
}

static volatile 	 uint32_t 	 g_u32AdcIntFlag,g_u32COVNUMFlag;

/****************************************************************************************************/
/*																									*/
/*						EADC00_IRQHandler																*/ 
/*																									*/
/*																									*/
/****************************************************************************************************/

void EADC00_IRQHandler(void)
{    
	/* Clear the A/D ADINT1 interrupt flag */    
	EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF0_Msk);    
	g_u32AdcIntFlag = 1;
}

/****************************************************************************************************/
/*																									*/
/*						EPWM0P2_IRQHandler															*/ 
/*																									*/
/*																									*/
/****************************************************************************************************/

void EPWM0P2_IRQHandler(void)
{
//	EPWM_ClearZeroIntFlag(EPWM0, 4);
	EPWM_ClearPeriodIntFlag(EPWM0, 4);
	g_qi_tx_time_counter++; 
	if(g_qi_tx_time_counter == HALF_BIT) 
	{    
		Fsk_vStart(HALF_BIT);
	} 
	else if(g_qi_tx_time_counter == FULL_BIT) 
	{ 
		g_qi_tx_time_counter = 0;
		Fsk_vStart(FULL_BIT);
	}  
}
/****************************************************************************************************/
/*																									*/
/*						reset_loop_counters															       */ 
/*																									*/
/*																									*/
/****************************************************************************************************/
void reset_loop_counters(void)
{
//	ticks_25us = 0;       
	ticks_1ms = 0;
	ticks_250ms = 0;
}

/****************************************************************************************************/
/*																									*/
/*						TMR1_IRQHandler															       */ 
/*																									*/
/*																									*/
/****************************************************************************************************/

__ramfunc void TMR1_IRQHandler(void) // 1000us
{
//	uint16_t rth_value, vbus_value;
//  int vin_counts;

#ifdef CONFIG_USE_UART 
  if (uart_comm_deep_sleep_emulate)
  {
      return;
  }
#endif
#ifdef Q_MEASURE			
	if((q_measure_state == 1 )/* &&( q_self_resonance_test_finished == 0 )*/)
	{
//		qi_measured_quality_factor[0] = quality_factor_measure_test();
		quality_factor_measure_test();
		TIMER_ClearIntFlag(TIMER1);
		return;
	}
#endif
  {
	if(delay_ms > 0 ) 
	{
		delay_ms = delay_ms - 1;
		/* Clear Timer1 time-out interrupt flag */
		TIMER_ClearIntFlag(TIMER1);
		return;
	}
	else
		delay_ms =0;
		// Check Vbus
		vbus_value = vbus_counts();
#if 0		
		if( vbus_value < ADC_VBUS_UV )
		{
		    if( ++qitx_UvCount > 10 )
		    {
				qi_device_charging = 0;
				PWM_vStopCoil();
				debug_qi_state_print("\r\n UV coil pwm disable %d",vbus_value);
				voltage_droop_flag = 1;
				state =QI_RESET_S;//reset;
				reasonReset = RESET_SYSTEM_UNDER_VOLTAGE;
				voltage_droop_flag = 1;
		    }
		}
		else
#endif			
		{
			qitx_UvCount=0;
		}
		if ((vbus_value > ADC_VBUS_OV) && (dcdc_enable == 1))
		{
		    if( ++qitx_OvCount > 10 )
		    {
				qi_device_charging = 0;
				PWM_vStopCoil();
				debug_qi_state_print("\r\n OV coil pwm disable %d",vbus_value);
				voltage_droop_flag = 1;
				state =QI_RESET_S;//reset;
				reasonReset = RESET_SYSTEM_OVER_VOLTAGE;
				voltage_droop_flag = 1;
		    }
		}
		else
		{
		    qitx_OvCount = 0;
		}
		if( qitx_UvCount == 0 && qitx_OvCount == 0 ) voltage_droop_flag = 0;
#if 0		
		//if (vbus_value < ADC_VBUS_UV || vbus_value > ADC_VBUS_OV || icurrent_value > ADC_ICCUR_OC)
		if ((vbus_value < ADC_VBUS_UV || vbus_value > ADC_VBUS_OV) && (dcdc_enable == 1))
		{
			//UV or OV
			qi_device_charging = 0;
			PWM_vStopCoil();
			debug_qi_state_print("\r\n UV or OV coil pwm disable %d",vbus_value);
			voltage_droop_flag = 1;
			state =QI_RESET_S;//reset;
			
		    if (vbus_value < ADC_VBUS_UV) reasonReset = RESET_SYSTEM_UNDER_VOLTAGE;
			else reasonReset = RESET_SYSTEM_OVER_VOLTAGE;
		}else
			voltage_droop_flag = 0;
#endif
		// Every 250ms
		if (++ticks_1ms == 250)
		{  
			ticks_1ms = 0;

			// Every 1s
			if (++ticks_250ms == 4)
			{
				ticks_250ms = 0;
				// Check temperature
#ifdef OT_LIMIT_POWER
        ot_control(rth_counts());
#endif
//				if (!over_temp_flag && (rth_value < ADC_AD5_RTH_THRESHOLD))
					if (0)
				{
					over_temp_flag = 1;

					qi_device_charging = 0;

					PWM_vStopCoil();

					WPCQiTx_Delay(15000);
				}

			}

			led_state_machine();
		}


		WPCQi_Tx();
	}
  /* Clear Timer1 time-out interrupt flag */
  TIMER_ClearIntFlag(TIMER1);

}

