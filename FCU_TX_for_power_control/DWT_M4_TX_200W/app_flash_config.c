#include "application.h"
#include "App_flash_config.h"

/****************************************************************************************************/
/*																									*/
/*			ledCntrol																              			*/ 
/*																									*/
/*																									*/
/****************************************************************************************************/
void app_flash_config_set_defaults( void )
{

	flash_config_ram.analog_ping_threshold = APP_APING_THRESH;

	flash_config_ram.vbus_ov_threshold = APP_VBUS_OV;

	flash_config_ram.rth_ot_threshold = APP_RTH_THRESHOLD;
	flash_config_ram.rth_ot_hysteresis_threshood = APP_RTH_HYS_THRESHOLD;

	flash_config_ram.vdroop_ref = APP_VDROOP_REF;
	flash_config_ram.vdroop_Kp = APP_VDROOP_KP;
	flash_config_ram.vdroop_max = APP_VDROOP_MAX;

	flash_config_ram.efficiency_threshold = APP_EFF_THRESH;
	flash_config_ram.flash_valid_key = FLASH_VALID_KEY;
  
}



