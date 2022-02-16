#include <stdio.h>
#include "NuMicro.h"
#include "application.h"
#include "modulation.h"

#define MAX_CONSECUTIVE_FAILURES        3
#define EFF_Z           05                     /*!< no load Isense counts */
#define MAX_EFFICIENCY 1000                    /* 0x000A0000 */    /*!< 1000% */

static int consecutive_faliures = 0;
extern int adc_offset_isense;


//uint32_t FOD_EFF_THRESHOLD= 0.65 /*0x0000A666*/; 
uint32_t FOD_EFF_THRESHOLD= 65 /*0x0000A666*/; 
int  eff_ratio= 3 /*0x00030000*/;

void efficiency_init(void)
{
//	eff_ratio =  qi_maximum_power;
//	eff_ratio =  ( eff_ratio *  POWER_OF_TEN[qi_power_class]);	   
//	eff_ratio=  1000 / eff_ratio ;
}
__ramfunc void efficiency_reset(void)
{
  consecutive_faliures = 0;
}

/*!
 *  Efficiency low calculation
 *
 *  This routine is called to determine if the transfer efficiency is too low,
 *  which could indicate that there is a foreign object on the charge pad, 
 *  between the transmitter and receiver coils.
 *
 *  i_sense_counts - Isense in ADC counts, unscaled for ENOS
 *  
 *  returns - 
 *     0 if efficiency is not too low
 *     1 if efficiency is below threshold
 */

int efficiency = 0;
//fix16_t efficiency_threshold;

__ramfunc int efficiency_low(int i_sense_counts, int vbus_counts)
{
//  fix16_t temp, bin_max;
  int /*index,*/ temp_int;
    
  // Make minimum RxPP value 5, so calculation will work for 
  // some receivers
  //if (qi_received_power < 0x200)
  //   return 0;
  
  if (qi_received_power < 1280) //0.2W @ 5W, 0.6W @ 15W
    qi_received_power = 1280;   
  
  // If this is a version 1.0 receiver, make the efficiency threshold
  // 70% of what it typically is
#if 0  
  if (qi_minor_version == 0)
  {
    //efficiency_threshold = fix16_mul_new_16_16(flash_config_ram.efficiency_threshold, 0x00008000);  /*!< 50% */
	  efficiency_threshold = flash_config_ram.efficiency_threshold /2.0 ;
	}
	else
	{
	  efficiency_threshold = flash_config_ram.efficiency_threshold;  
	}
#endif

  /*   Calculate raw efficiency   */
  temp_int = vbus_counts;  
	
  temp_int = temp_int * 10 / qi_maximum_power;  

  temp_int = temp_int * (i_sense_counts - adc_offset_isense - EFF_Z);

  if(temp_int <1 ) temp_int = 1;

  efficiency =  qi_received_power * 100 / temp_int ;

  if (efficiency < 1) efficiency = 1;
  
//  else
//    efficiency = temp / efficiency;

  // Isense should never be railed here, if it is stop 
  // immediately

//yy.lee debug orig  if (i_sense_counts == 1023)
  if (i_sense_counts > OVER_CURRENT)
  {
    consecutive_faliures = 0;
    return 1;
  }
  else if ((efficiency < flash_config_ram.efficiency_threshold ))// || 
 //     ((efficiency > MAX_EFFICIENCY) && (qi_received_power > 2560 ))) // 2560 : 1.17W @ 15W , 0.39W @ 5W
  {
    if (++consecutive_faliures >= MAX_CONSECUTIVE_FAILURES)
    {
      consecutive_faliures = 0;
      return 1;
    }
  }
  else
    consecutive_faliures = 0;

  return 0; // ok

}



// yy.lee need calibration phase. spec.: 5.1.2.5 calibration phase-epp only
// mode : 0x01 : light load, 0x02
//Preceived = (qi_rx_buffer[1:0]/32768)*(maximum power/2)*pow(10,power class)
// yy.lee need calibration phase. spec.: 5.1.2.5 calibration phase-epp only
// mode : 0x00 : response requested 
// mode : 0x01 : light load,response requested
// mode : 0x02 : connected-load calibration valure, response requested
// mode : 0x04 : Normal value no response requested

fix16_t /*temp_is,temp_vs,*/measured_tx_power;
fix16_t light_load_tx_power, connected_load_tx_power;
fix16_t light_load_received_power,connected_load_received_power, max_power;


__ramfunc int power_calibration_phase(int i_sense_counts, int vbus_counts, int mode)
{
//  fix16_t temp, bin_max;
//  int index, temp_int;
//  g_st_power_contract.max_power;
	int temp_int;




//  temp_is = i_sense_counts;
//  temp_vs = vbus_counts;
//  measured_tx_power = temp_is * temp_vs ;

	temp_int = vbus_counts;  

	temp_int = temp_int * 10 / qi_maximum_power;	

	temp_int = temp_int * (i_sense_counts - adc_offset_isense - EFF_Z);	

	measured_tx_power = (fix16_t)temp_int;

  if( qi_received_power_mode == 0x01 )
	{
    light_load_tx_power = measured_tx_power;  
		light_load_received_power = 	qi_received_power;
	}
  else if(qi_received_power_mode == 0x02)
	{
    connected_load_tx_power = measured_tx_power;
		connected_load_received_power = 	qi_received_power;
	}
  return 0;
}


