#include <stdio.h>
#include "NuMicro.h"
#include "Pid.h"
#include "application.h"
#include "Pwm.h"
#include "Coil.h"

#ifdef VOLTAGE_CONTROL
static fix16_t freq_prev_kHz_f16 = FREQ_KHZ_PING_FIX16;          /*!< Previous coil switching frequency in kHz (default 175kHz) */
static fix16_t voltage_prev_volt_f16 = DCDC_PWM_INIT;          /*!< Previous coil switching frequency in kHz (default 175kHz) */
#else
static fix16_t I_prev_f16 = 0;                          /*!< Previous I term in fix16_t format */
static fix16_t freq_prev_kHz_f16 = FREQ_KHZ_PING_FIX16;          /*!< Previous coil switching frequency in kHz (default 175kHz) */
#endif
fix16_t min_freq_kHz_f16 = FREQ_KHZ_MIN_FIX16;   /*!< Minimum Fs used for control */

fix16_t freq_cutoff = FREQ_KHZ_CUTOFF_FIX16;
fix16_t duty_cutoff= MP_DUTY_CUTOFF_THRESHOLD;
static fix16_t duty_kp = DUTY_KP_FIX16;
#ifdef FREQ_CONTROL
static fix16_t freq_kp = FREQ_KP_FIX16;
#endif
#ifdef VOLTAGE_CONTROL
static fix16_t volt_kp = VOLT_KP_FIX16;
#endif
#ifdef FREQ_CONTROL
static fix16_t freq_ki = FREQ_KI_FIX16;
static fix16_t freq_mi = FREQ_MI_FIX16;
static fix16_t freq_mpid = FREQ_MPID_FIX16;
#endif

#ifdef DYNAMIC_DIGITAL_PING_POWER
typedef struct
{
	int freq;
	int duty;
	uint8_t prev_freq;
	uint8_t next;
}freq_cfg_state;

freq_cfg_state freqs[4] ={{TIMERA_TICKS_145KHZ,TIMERA_TICKS_145KHZ>>1,145,1},
			  			  {TIMERA_TICKS_135KHZ,TIMERA_TICKS_135KHZ>>1,135,2},
			              {TIMERA_TICKS_127_7KHZ,TIMERA_TICKS_127_7KHZ>>1,127,3},
			              {TIMERA_TICKS_110KHZ,TIMERA_TICKS_110KHZ>>1,110,0}};


freq_cfg_state *freq_state;

int digital_ping_freq=TIMERA_TICKS_175KHZ;
int digital_ping_duty=TIMERA_TICKS_175KHZ>>1;
#endif

uint8_t ot_limit_flag;                               /*!< Set to non-zero if over-temperature power limit mode is not working */

fix16_t P_f16, I_f16, PID_f16, freq_kHz_f16, scale_f16,ss; /*!< P, I, PID terms, Fs in kHz and scale in fix16_t format */
fix16_t dcdc_update_value =0;
uint16_t dcdc_update_value_int =0;
extern uint8_t qi_oob_Max_Addr_send_to_ble;

extern uint8_t coil_sel_num;
#ifdef NEW_METHOD_DUTY 
static __ramfunc void set_pwm_from_virt_freq(fix16_t freq_kHz_f16,int Update_cep);
#else
static __ramfunc void set_pwm_from_virt_freq(fix16_t freq_kHz_f16);
#endif


#ifdef DYNAMIC_DIGITAL_PING_POWER

/****************************************************************************************************/
/*																									*/
/*			reset_freq_machine																*/ 
/*																									*/
/*																									*/
/****************************************************************************************************/
void reset_freq_machine(void)
{ 
	freq_state =&freqs[0];
}
/****************************************************************************************************/
/*																									*/
/*			freq_control_machine																*/ 
/*																									*/
/*																									*/
/****************************************************************************************************/
void freq_control_machine(void)
{
	static char counter=0;
	if(counter++<5)
		return;
  counter=0;
  freq_state=&freqs[freq_state->next];
	freq_prev_kHz_f16 =freq_state->prev_freq <<16;
	digital_ping_freq=freq_state->freq;
	digital_ping_duty =freq_state->freq>>1;
	
}				
#endif

/****************************************************************************************************/
/*																									*/
/*			WPCQi_setPreFrequencyHandleCEP																*/ 
/*																									*/
/*																									*/
/****************************************************************************************************/
void WPCQi_setPreFrequencyHandleCEP (fix16_t frequency )
{
    freq_prev_kHz_f16 = frequency;
}
/****************************************************************************************************/
/*																									*/
/*			WPCQi_InitVoltageHandleCEP																*/ 
/*																									*/
/*																									*/
/****************************************************************************************************/
void WPCQi_InitVoltageHandleCEP(fix16_t init_voltage_value )
{
	voltage_prev_volt_f16 = init_voltage_value;
}



/****************************************************************************************************/
/*																									*/
/*			WPCQi_InitHandleCEP																*/ 
/*																									*/
/*																									*/
/****************************************************************************************************/
void WPCQi_InitFrequencyHandleCEP(void)
{
//	if (g_st_power_contract.max_power > 5)
	if (g_st_power_contract.max_power > BPP_MAX_POWER_10W)
    {
        freq_cutoff = FREQ_KHZ_CUTOFF_FIX16;
        duty_cutoff = MP_DUTY_CUTOFF_THRESHOLD;
        duty_kp     = DUTY_KP_FIX16;
    
#ifdef FREQ_CONTROL
        freq_kp     = FREQ_KP_FIX16;
        freq_ki     = FREQ_KI_FIX16;
        freq_mi     = FREQ_MI_FIX16;
        freq_mpid   = FREQ_MPID_FIX16;
#endif
    }
    else
    {
        freq_cutoff = FREQ_KHZ_LCUTOFF_FIX16;
        duty_cutoff = LP_DUTY_CUTOFF_THRESHOLD;
        duty_kp     = DUTY_LKP_FIX16;
    
#ifdef FREQ_CONTROL
        freq_kp     = FREQ_LKP_FIX16;
        freq_ki     = FREQ_LKI_FIX16;
        freq_mi     = FREQ_LMI_FIX16;
        freq_mpid   = FREQ_LMPID_FIX16;   
#endif
    }
 
}


/****************************************************************************************************/
/*																									*/
/*			WPCQi_InitHandleCEP																*/ 
/*																									*/
/*																									*/
/****************************************************************************************************/
void WPCQi_InitHandleCEP(void)
{
  is_control = 0;
#ifdef VOLTAGE_CONTROL
//  freq_prev_kHz_f16 = FREQ_KHZ_145_FIX16;
	freq_prev_kHz_f16 = FREQ_KHZ_PING_FIX16;					/*!< Previous coil switching frequency in kHz */
#else
	I_prev_f16 = 0;
	freq_prev_kHz_f16 = FREQ_KHZ_PING_FIX16;
#endif
  voltage_droop_flag = 0;

#ifdef OVER_VOLTAGE_CONTROL
	coil_over_voltage_flag = 0;
#endif

  min_freq_kHz_f16 = FREQ_KHZ_MIN_FIX16;
#ifdef VOLTAGE_CONTROL
#ifndef Q_MEASURE			
  PWM_vStartBuckbooster();
  voltage_prev_volt_f16 = DCDC_PWM_APING;		  /*!< Previous coil switching frequency in kHz (default 175kHz) */
  PWM_vUpdateBuckbooster( TIMERA_TICKS_100KHZ, TIMERA_TICKS_100KHZ*voltage_prev_volt_f16/DCDC_PWM_RATIO_200);
#else
#endif

#endif
}





/****************************************************************************************************/
/*																									*/
/*			WPCQi_HandleControlErrorPacket																*/ 
/*																									*/
/*																									*/
/****************************************************************************************************/
__ramfunc void WPCQi_HandleControlErrorPacket(int update_cep)
{
	static fix16_t coil_current_f16;                         /* new coil current , !< in amperes in fix16_t format */
	fix16_t error_f16;                                      /*!< CEP errror in fix16_t format */

	if((min_freq_kHz_f16 >= (fix16_t)FREQ_KHZ_MIN_FIX16) && (qi_control_error_value == 0))
	{
	 if( qi_oob_Max_Addr_send_to_ble == 1 )
		 debug_qi_cep_print("\r\n cep :%d ",qi_control_error_value);
	return;
	}
	  
	qi_control_error_value_f16 = (float)qi_control_error_value;
	if( coil_sel_num == COIL_SEL_2)
	{
	    qi_control_error_value_f16 = qi_control_error_value_f16/(float)(128.0);
	}
	else
	{
		qi_control_error_value_f16 = qi_control_error_value_f16/(float)(32.0);
	}
	coil_current_f16 = (float)1.0 + qi_control_error_value_f16;
	coil_current_f16 = qi_coil_current_f16 * coil_current_f16;
    error_f16 = coil_current_f16 - qi_coil_current_f16;

#ifdef VOLTAGE_CONTROL
#ifdef OVER_VOLTAGE_CONTROL
	if ( ( coil_over_voltage_flag == 1 ) && ( qi_control_error_value > 0 ) )
	{
		voltage_prev_volt_f16 = voltage_prev_volt_f16 /* + (fix16_t)10*/;
	}
	else
#endif		
	{
		P_f16 = volt_kp  * error_f16;
	    voltage_prev_volt_f16 = voltage_prev_volt_f16 - P_f16;
	}
	if( coil_sel_num == COIL_SEL_2)
	{
		if(voltage_prev_volt_f16 < DCDC_PWM_MIN_C0 )
			voltage_prev_volt_f16 = DCDC_PWM_MIN_C0;
		else if(voltage_prev_volt_f16 > DCDC_PWM_MAX_C0 )
			voltage_prev_volt_f16 = DCDC_PWM_MAX_C0;
	}
	else
	{
	if(voltage_prev_volt_f16 < DCDC_PWM_MIN )
		voltage_prev_volt_f16 = DCDC_PWM_MIN;
	else if(voltage_prev_volt_f16 > DCDC_PWM_MAX )
		voltage_prev_volt_f16 = DCDC_PWM_MAX;
	}
	dcdc_update_value = ((fix16_t)TIMERA_TICKS_100KHZ) * voltage_prev_volt_f16 / (fix16_t)DCDC_PWM_RATIO_200;
	dcdc_update_value_int = (uint16_t)dcdc_update_value;
	if( qi_oob_Max_Addr_send_to_ble == 1 )
		debug_qi_cep_print("\r\n V set PWM f :%.1f D: %d  cep :%d  I %.2f ovp %d ",voltage_prev_volt_f16,dcdc_update_value_int,qi_control_error_value, qi_coil_current_f16,coil_over_voltage_flag);

	PWM_vUpdateBuckbooster(TIMERA_TICKS_100KHZ,dcdc_update_value_int );
#else  
	  // P = Kp * error
	  P_f16 = freq_kp/*FREQ_KP_FIX16*/ * error_f16;
	  // I = I previous + Ki * error * Tinner
	  //
	  // Limit I to: -MI <= I <= MI
	  
	  I_f16 =  freq_ki/*FREQ_KI_FIX16*/ * error_f16;
	  I_f16 =  I_f16 * TINNER_FIX16;

	  I_f16 += I_prev_f16;
	  
	  if (I_f16 < -freq_mi/*FREQ_MI_FIX16*/)
	    I_f16 = -freq_mi;//FREQ_MI_FIX16;
	  else if (I_f16 > freq_mi/*FREQ_MI_FIX16*/)
	    I_f16 = freq_mi;//FREQ_MI_FIX16;
	  
	  // Calculate PID = P + I
	  //
	  // Limit PID to -MPID <= PID <= MPID
	  
	  PID_f16 =  P_f16 + I_f16;  
	  if (PID_f16 < -freq_mpid/*FREQ_MPID_FIX16*/)
	    PID_f16 = -freq_mpid;//FREQ_MPID_FIX16;
	  else if (PID_f16 > freq_mpid/*FREQ_MPID_FIX16*/)
	    PID_f16 = freq_mpid;//FREQ_MPID_FIX16;
	  
	  // New frequency = last_frequency - scale * PID
	  //
	  // Make sure to limit to min/max allowable frequencies
	  
	  if (freq_prev_kHz_f16 < FREQ_KHZ_115_FIX16)
	    scale_f16 = FREQ_SV_110KHZ_LTE_FS_LT_115KHZ_FIX16; // 1.5
	  else if (freq_prev_kHz_f16 < FREQ_KHZ_120_FIX16)
	    scale_f16 = FREQ_SV_115KHZ_LTE_FS_LT_120KHZ_FIX16; // 2.0 
	  else if (freq_prev_kHz_f16 < FREQ_KHZ_135_FIX16)
	    scale_f16 = FREQ_SV_120KHZ_LTE_FS_LT_135KHZ_FIX16; // 3.0
	  else if (freq_prev_kHz_f16 < FREQ_KHZ_140_FIX16)
	    scale_f16 = FREQ_SV_135KHZ_LTE_FS_LT_140KHZ_FIX16; // 5.0
	  else if (freq_prev_kHz_f16 < FREQ_KHZ_145_FIX16)
	    scale_f16 = FREQ_SV_140KHZ_LTE_FS_LT_145KHZ_FIX16; // 10.0
	  else if (freq_prev_kHz_f16 < FREQ_KHZ_205_FIX16)
	    scale_f16 = FREQ_SV_145KHZ_LTE_FS_LT_205KHZ_FIX16; // 16.0
	  else
	    scale_f16 = FREQ_SV_205KHZ_LTE_FS_LTE_430KHZ_FIX16; // 32.0
	  
	  freq_kHz_f16 = (float)scale_f16 * (float)PID_f16;

	  freq_kHz_f16 = freq_prev_kHz_f16 - freq_kHz_f16;

		if (freq_kHz_f16 > FREQ_KHZ_MAX_VIRT_FIX16)
			freq_kHz_f16 = FREQ_KHZ_MAX_VIRT_FIX16;
		else if (freq_kHz_f16 < min_freq_kHz_f16)
			freq_kHz_f16 = min_freq_kHz_f16;

		// Set the new PWM based on this virtual frequency
#ifdef NEW_METHOD_DUTY 
		set_pwm_from_virt_freq((fix16_t)freq_kHz_f16,update_cep);
#else
		set_pwm_from_virt_freq((fix16_t)freq_kHz_f16);
#endif

		freq_prev_kHz_f16 = freq_prev_kHz_f16;   

	// Store virtual frequency for next loop iteration
	freq_prev_kHz_f16 = freq_kHz_f16;   

	// Store previous I term for the PI controller
#ifndef VOLTAGE_CONTROL
	I_prev_f16 = I_f16;
#endif
#endif
}

__ramfunc void vdroop_control(int vin_counts)
{ 
//  // See if we should be adjust down
//  static fix16_t new_freq_kHz_f16 = FREQ_KHZ_MIN_FIX16;   /*!< New Fs used for droop correction */
//
//  if (vin_counts < APP_VDROOP_REF)
//  {
//    voltage_droop_flag = 1;
//  
//    new_freq_kHz_f16 = fix16_mul(VDROOP_DOWN_KP_FIX16, new_freq_kHz_f16);
//    
//    if (new_freq_kHz_f16 > VDROOP_MIN_FREQ_FIX16)
//      new_freq_kHz_f16 = VDROOP_MIN_FREQ_FIX16;
//    
//    set_pwm_from_virt_freq(new_freq_kHz_f16);
//    
//    min_freq_kHz_f16 = new_freq_kHz_f16;
//    
//  }
}

#ifdef NEW_METHOD_DUTY 
__ramfunc void set_pwm_from_virt_freq(fix16_t freq_kHz_f16,int update_cep)
#else
__ramfunc void set_pwm_from_virt_freq(fix16_t freq_kHz_f16)
#endif
{
	fix16_t temp_f16;
	fix16_t pwm_duty_scale;
  fix16_t act_kHz_f16;
  int pwm_period_ticks;
	
  // Cap maximum virtual frequency, and min in case of overflow
  if (freq_kHz_f16 > (fix16_t)FREQ_KHZ_MAX_VIRT_FIX16)
    freq_kHz_f16 = FREQ_KHZ_MAX_VIRT_FIX16;
  else if (freq_kHz_f16 < min_freq_kHz_f16)
    freq_kHz_f16 = min_freq_kHz_f16;
  
  // If the frequency is less than the taper threshold, calculate new duty
  // cycle for light load processing

  if (freq_kHz_f16 > (fix16_t)FREQ_KHZ_CUTOFF_FIX16)
  {
		// duty = Fvirtual - Fcutoff
		// duty = max(duty, 0)
		// duty = duty * Kp_duty
		// duty = max(50% - duty, 10%)

		temp_f16 = freq_kHz_f16 - (fix16_t)FREQ_KHZ_CUTOFF_FIX16;

		freq_kHz_f16 = FREQ_KHZ_CUTOFF_FIX16;

		//    if (temp_f16 < 0)
		//      temp_f16 = 0;

		temp_f16 = temp_f16 * duty_kp/*DUTY_KP_FIX16*/;
		// 0.5 - temp_f16   
		if (temp_f16 > (fix16_t)DUTY_CUTOFF_THRESHOLD )
			temp_f16 = DUTY_CUTOFF_THRESHOLD;

		temp_f16 = (fix16_t)DUTY_CUTOFF_THRESHOLD - temp_f16;

		if (temp_f16 < (fix16_t)MIN_DUTY) //0x1999
			temp_f16 = MIN_DUTY;
		pwm_duty_scale = temp_f16;
  }
  else
  {
    // Use 50% duty cycle 
    pwm_duty_scale = DUTY_CUTOFF_THRESHOLD;
  }
  
  // Cap frequency
  if (freq_kHz_f16 > (fix16_t)FREQ_KHZ_CUTOFF_FIX16)
    act_kHz_f16 = FREQ_KHZ_CUTOFF_FIX16;
  else if (freq_kHz_f16 < (fix16_t)FREQ_KHZ_MIN_FIX16)
    act_kHz_f16 = FREQ_KHZ_MIN_FIX16;
  else
    act_kHz_f16 = freq_kHz_f16;
  
  // Convert frequency to period register value

  temp_f16 = (fix16_t)(55 * TIMERA_TICKS_110KHZ);   // scale by 2 //  55 * 327 = 17985
  temp_f16 = (fix16_t)(temp_f16 /act_kHz_f16 );  // temp_f16 / act_kHz_f16  17985/ 135 = 133.2
  pwm_period_ticks = (int)((temp_f16 * (float)2 ) + 1);  // 148.29 * 2 + 0.5 = 266.9..

   // Calculate duty ticks
   
   pwm_duty_ticks = (int)((pwm_period_ticks * pwm_duty_scale)); // 266.9* 0.5 = 133.2
   PWM_vUpdateCoil(pwm_period_ticks, pwm_duty_ticks);

}




void mul_pwm_from_virt_freq(fix16_t freq_gain_f16)
{
//yy.lee debug   set_pwm_from_virt_freq(fix16_mul_new_16_16(freq_prev_kHz_f16, freq_gain_f16));
//   set_pwm_from_virt_freq( freq_prev_kHz_f16 * freq_gain_f16);

}


#if 0
void vdroop_control(int vin_counts)
{
  int vin_error;
  //fix16_t freq_kHz_f16;
static   int clear_vdroop_ticks = 0;

  vin_error = fix16_to_int(flash_config_ram.vdroop_ref) - vin_counts;
  
  if ((vin_error > 0) /*&& (freq_kHz_f16 < freq_prev_kHz_f16)*/)
  {
    //debug2_on();
    min_freq_kHz_f16 = fix16_add(freq_prev_kHz_f16, fix16_mul_new_16_16(fix16_from_int(vin_error),flash_config_ram.vdroop_Kp));
    if (min_freq_kHz_f16 > flash_config_ram.vdroop_max)
    {
      min_freq_kHz_f16 = flash_config_ram.vdroop_max;
    }
    set_pwm_from_virt_freq(min_freq_kHz_f16);
 
    
    freq_prev_kHz_f16 = min_freq_kHz_f16;
    
    if (voltage_droop_flag == 0)
    {
      voltage_droop_flag = 1;
      debug_tracking.ctr_num_vdroop++;
    }
    clear_vdroop_ticks = 0;
   
  }
  else
  {
    min_freq_kHz_f16 = FREQ_KHZ_MIN_FIX16;
    if (++clear_vdroop_ticks >= 20000)
    {
      voltage_droop_flag = 0;
      /*
      if (!ot_limit_flag)
        min_freq_kHz_f16 = FREQ_KHZ_MIN_FIX16;
      */
    }
  }
}

#endif


 void ot_control(uint16_t rth_value)
{
  int ot_ctrl_count = 0;
  int prev_rth_value;
  
  /* enter power limit mode if temperature is around 10C lower than the threshold */
  if (!ot_limit_flag && (rth_value < (flash_config_ram.rth_ot_threshold + 55)))
  {
    ot_ctrl_count = 0;
    ot_limit_flag = 1;
//    debug_tracking.ctr_num_ot_ctrl = 0;
    prev_rth_value = 1023;
  }
  
  /* keep power limit mode if temperature is higher than the threshold + 63 (10C lower than threshold) */
  if (ot_limit_flag && (rth_value < (flash_config_ram.rth_ot_hysteresis_threshood + 63)))
  {
    if (ot_ctrl_count == 0)
    {
      if ((rth_value < prev_rth_value) && (state == QI_POWER_TRANSFER_S/*power_transfer*/) 
	  	/*&& (debug_tracking.ctr_num_ot_ctrl < 2)*/
	  	)
      {
        //min_freq_kHz_f16 = fix16_mul_new_16_16(freq_prev_kHz_f16, 0x10CCC); // 5% pwm freq increase 
        //min_freq_kHz_f16 = fix16_mul_new_16_16(freq_prev_kHz_f16, 0x10F5C); // 6% pwm freq increase 
        min_freq_kHz_f16 = (fix16_t)(freq_prev_kHz_f16 * (fix16_t)1.07); // 7% pwm freq increase yy.lee debug  ????
        freq_prev_kHz_f16 = min_freq_kHz_f16;
        set_pwm_from_virt_freq(min_freq_kHz_f16,1);
//        debug_tracking.ctr_num_ot_ctrl++;
      }
      prev_rth_value = rth_value;
    }
    if (++ot_ctrl_count == 60)
    {
      ot_ctrl_count = 0; // every 60secs
    }
  }
  /* release power limit mode if temperature is lower than threshold + 30 (5C lower than threshold) */
  else if (ot_limit_flag && (rth_value >= flash_config_ram.rth_ot_hysteresis_threshood + 30))  // OT released
  {
    ot_limit_flag = 0;
//    debug_tracking.ctr_num_ot_ctrl = 0;
    if (!voltage_droop_flag)
      min_freq_kHz_f16 = FREQ_KHZ_MIN_FIX16;
  }
}



