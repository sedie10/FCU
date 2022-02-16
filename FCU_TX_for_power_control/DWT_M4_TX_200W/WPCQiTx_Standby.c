#include "WPCQiTx_Standby.h"
#include "math.h"
#include "Adc.h"
#include "Application.h"
#include "Pwm.h"
#include "Coil.h"

int aping_thresh_last = 0;
int q_factor_measured_value_temp;
int aping_last = 1024;

/****************************************************************************************************/
/*																									*/
/*			analog_ping_test																*/ 
/*																									*/
/*																									*/
/****************************************************************************************************/
int analog_ping_test(uint16_t threshold)
{ 
	int i;
  // Compensate threshold for Vin  

   aping_thresh_last = threshold - ((ADC_VBUS_24 - vbus_counts())>>1); 

// Enable coil at selection operating point
#ifdef ANALOG_PING_HALF
   coil_pwm_half_enable();
#else  
 // PWM_vStartCoil();
#endif  
  // Disable coil
   PWM_vStopCoil();
#ifdef VOLTAGE_CONTROL
   PWM_vStartBuckbooster();

   PWM_vUpdateBuckbooster( TIMERA_TICKS_100KHZ, TIMERA_TICKS_100KHZ*DCDC_PWM_APING/DCDC_PWM_RATIO_200);
#endif
//  generate 1 cycle pule for coil Quality factor

    PA12 = 1;
	for (i = 0;i < 200;i++)
	{
			__nop();
	}
// 7us delay

    // ADC Conversion  Store v_counts

	aping_last = vsense_analog_counts();

	for (i = 0;i < 400;i++)
	{
			__nop();
	}
    PA12 = 0;

  // Look for Vsense to drop below threshold for 19.5us

    if (aping_last > aping_thresh_last)
    {
        return 1;
    }
  
    if (aping_last <= aping_thresh_last)
        return 0;

    return 0;
}

#ifdef Q_MEASURE	
/****************************************************************************************************/
/*																									*/
/*			quality_factor_measure_test_vbus_set																*/ 
/*																									*/
/*																									*/
/****************************************************************************************************/

uint8_t quality_factor_measure_test_vbus_set(uint8_t fod_coil_sel)
{
	int q_loop;
	uint16_t q_adc_ref_temp = 0;
	uint16_t q_adc_ref_sum = 0;
	


//	NVIC_DisableIRQ(ECAP0_IRQn);
		
	NVIC_DisableIRQ(TMR1_IRQn);
	
    TIMER_ClearIntFlag(TIMER1);
	
	TIMER_SET_CMP_VALUE(TIMER1, 30);		//   5us
	
	q_measure_state = 1;

	ECAP0_vStop();
	
	for( q_loop = 0 ; q_loop < Q_SELF_RESONANCE_NUM+1 ; q_loop++ )
	{
		q_self_resonance_time_array[q_loop] = 0;
	}
	q_self_resonance_time_idx =	0;
	q_self_resonance_time_new =	0;
	q_self_resonance_test_finished = 0;

	for(q_loop = 0 ; q_loop< Q_SELF_RESONANCE_NUM+1 ; q_loop++)	{	q_adc[q_loop] =	0;	}

//	VBUS_SW_EN = 0;
	QiTx_vPowerControl(POWER_OFF);
	CLK_SysTickDelay(100000);  //100ms

	Q_V_SUPPLY_EN = 1;
	Q_GATE_DR_SUPPLY_EN = 1;

	PWM_vStopCoil();
	CLK_SysTickDelay(100000);  //100ms

	Q_DRH10_PA1		= 0;
	Q_DRL10_PA0		= 0;
	Q_DRH11_PC1		= 0;
	Q_DRL11_PC0		= 0;
	
	

//	coil_sel( coil_sel_num );
	QiTxCoil_vSelect( fod_coil_sel );
	
#if 1
//	Q_DRH11_PC1		= 1;
	Q_DRH11_PC1		= 0;
	Q_DRL10_PA0 	= 1;
//need to delay...	& QF value init
	CLK_SysTickDelay(0xffffffff);  //100ms
	CLK_SysTickDelay(0xffffffff);  //100ms
	CLK_SysTickDelay(0xffffffff);  //100ms
	GPIO_SetMode(PF, BIT11, GPIO_MODE_OUTPUT);
	Q_MEASURE_RST = 1;
	CLK_SysTickDelay(1000);  //1ms
	Q_MEASURE_RST = 0;
	GPIO_SetMode(PF, BIT11, GPIO_MODE_INPUT);
	CLK_SysTickDelay(1000);  //1ms
	for( q_loop = 0 ; q_loop <10 ; q_loop++ )
	{
		q_adc_ref_temp = q_peak_hold_counts();
		q_adc_ref_sum += q_adc_ref_temp;
		CLK_SysTickDelay(10);  //10us
	}
	q_th_ref_value = q_adc_ref_sum /10;
//	q_th_ref_value = 1360;
	Q_V_SUPPLY_EN = 0;
//	Q_DRH11_PC1		= 0;
	Q_DRL11_PC0		= 1;
#else
	Q_DRL11_PC0 	= 1;
	Q_DRH10_PA1		= 1;
#endif
//	CLK_SysTickDelay(1000);  //1ms
	q_measure_state = 1;
	q_self_resonance_adc_num =0;
	
#ifdef ECAP_Q_FACTOR
	ECAP1_vStart();
#endif	
	NVIC_EnableIRQ(TMR1_IRQn);
	return (uint8_t) 1;
}

/*   measure EADC0_CH0 : Q_PEAK_HOLD_DATA */

/****************************************************************************************************/
/*																									*/
/*			quality_factor_measure_freq																*/ 
/*																									*/
/*																									*/
/****************************************************************************************************/

/*    Freq calculation      average time / timer frequency  */
void quality_factor_measure_freq( uint32_t q_resonance_time )
{
//	uint32_t freq_meausre,freq_meausre_avg;
	uint32_t	q_loop;
		
	freq_meausre_avg =0;
	
	q_self_resonance_time_array[q_self_resonance_time_idx] = q_resonance_time;
	q_self_resonance_time_new = 1;
	if((q_self_resonance_time_idx >= (Q_SELF_RESONANCE_NUM-1)) || (q_resonance_time > (ECAP1_TIMEOUT-1)))
	{
#ifdef ECAP_Q_FACTOR
		ECAP1_vStop();
#endif
		for( q_loop = 2 ; q_loop < q_self_resonance_time_idx ; q_loop++)
		{

			freq_meausre_avg += q_self_resonance_time_array[q_loop];
		}
		if(q_resonance_time > (ECAP1_TIMEOUT-1))
		{
			freq_meausre_avg -= q_self_resonance_time_array[q_loop];
			q_self_resonance_time_idx -=1;
		}
		freq_meausre_avg = freq_meausre_avg / ( q_self_resonance_time_idx -2);
		freq_meausre_avg = ECAP1_TIMER_FREQ / freq_meausre_avg;
		q_self_resonance_test_finished = 1;
//		debug_qi_Q_measure_print("\r\n freq test finished 1 res tim %d  idx %d ",q_resonance_time,q_self_resonance_time_idx);
		q_self_resonance_time_idx = 0;
	}
	q_self_resonance_time_idx++;
	if(q_self_resonance_time_idx >= Q_SELF_RESONANCE_NUM )
	{
#ifdef ECAP_Q_FACTOR
		ECAP1_vStop();
#endif
		q_self_resonance_time_idx =0;
		q_self_resonance_test_finished = 1;
		q_measure_state = 0;
		debug_qi_Q_measure_print("\r\n freq test finished 2 ");
	}
}

/****************************************************************************************************/
/*																									*/
/*			quality_factor_measure_test																*/ 
/*																									*/
/*																									*/
/****************************************************************************************************/
__ramfunc uint8_t quality_factor_measure_test(void) 
{
//	int i,q_loop,q_loop_min_max;
	q_factor_measured_value_temp = 0;
//	q_loop = 0;
	if( q_self_resonance_time_new == 1 )
	{	
		q_self_resonance_time_new = 0;
		q_peak_hold_temp_min = 0;
		q_peak_hold_temp_max = 0;
		q_self_resonance_adc_num =0;
	}
	else
		q_self_resonance_adc_num++;
	if( q_self_resonance_adc_num > ( Q_SELF_RESONANCE_NUM -1 ) )
	{
		q_self_resonance_adc_num = Q_SELF_RESONANCE_NUM;
	}

	q_peak_hold_temp = q_peak_hold_counts();
	if(q_peak_hold_temp_min > q_peak_hold_temp )
	{
		q_peak_hold_temp_min = q_peak_hold_temp;
	}
	else if (q_peak_hold_temp_max < q_peak_hold_temp)
	{
		q_peak_hold_temp_max = q_peak_hold_temp;
	}

	q_adc[q_self_resonance_time_idx] = q_peak_hold_temp_max; 
	if(( q_self_resonance_test_finished == 1 ) ||(q_self_resonance_adc_num > ( Q_SELF_RESONANCE_NUM -1 )))
	{
		Q_DRH10_PA1		= 0;
		Q_DRL10_PA0		= 0;
		Q_DRH11_PC1		= 0;
		Q_DRL11_PC0		= 0;
		Q_V_SUPPLY_EN = 0;
		q_factor_measured_value_temp =q_adc[q_self_resonance_time_idx];

		NVIC_DisableIRQ(TMR1_IRQn);
		TIMER_ClearIntFlag(TIMER1);
		q_measure_state = 0;
		if( q_self_resonance_test_finished == 0 )
			q_self_resonance_test_finished = 1;
//		state = selection;
//		q_self_resonance_test_finished = 0;		
		TIMER_SET_CMP_VALUE(TIMER1, 6000);		//	 1ms
		NVIC_EnableIRQ(TMR1_IRQn);
#ifdef ECAP_Q_FACTOR
		ECAP1_vStop();
#endif
//		NVIC_DisableIRQ(ECAP1_IRQn);

		ECAP0_vStart();
//		NVIC_EnableIRQ(ECAP0_IRQn);

//		jj=(double)log((double)2.71);

/*      Q = ( L / R ) 2* PI * 100kHz                  */
/*                                                                     */
/*        (  1/ Freq  ) * 2 * PI * 100kHz             */
/*        ------------------------             */
/*             2* ln( Vn /Vm )                                */
	}
	return (uint8_t)( q_factor_measured_value_temp);
}
#endif	


