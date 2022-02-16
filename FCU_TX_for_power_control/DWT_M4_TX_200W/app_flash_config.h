#ifndef APP_FLASH_CONFIG_H
#define APP_FLASH_CONFIG_H

#include "application.h"

typedef float               fix16_t;


#define FLASH_VALID_KEY                 0xBEEFDEAD


// Analog ping
#if (defined(KR_BUILD))
#define APP_APING_THRESH                345
#else
#define APP_APING_THRESH                250
#endif

#define APP_OC_THRESH                   12


// Vdroop
#define APP_VDROOP_REF                  (ADC_VBUS_VDROOP << 16)     // Vdroop PID reference in fix16_t
#define APP_VDROOP_KP                   0x000a0000
#define APP_VDROOP_MAX                  (275 << 16)

// UV
#define APP_VBUS_UV                     ADC_VBUS_UV


// OV
#define APP_VBUS_OV                     ADC_VBUS_OV

// Thermistor
#define APP_RTH_THRESHOLD               71             /* 95C*/
#define APP_RTH_HYS_THRESHOLD           71 


// PingPong modulation

#define APP_PP_V_DEFAULT_HYS            20               
#define APP_PP_V_LP_ALPHA               FIX16_3K
#define APP_PP_V_HYS_SCALE              12

#define APP_PP_I_DEFAULT_HYS            3               
#define APP_PP_I_LP_ALPHA               FIX16_2K
#define APP_PP_I_HYS_SCALE              10

// Efficiency
#define APP_EFF_THRESH                  65;     /* 65% */


typedef struct 
{
  uint16_t analog_ping_threshold;       /*!< Analog ping threshold ADC counts */
  fix16_t vdroop_ref;                   /*!< Vdroop PID reference */
  fix16_t vdroop_Kp;                    /*!< Vdroop Kp gain */
  fix16_t vdroop_max;                   /*!< Vdroop max output */
  uint16_t vbus_ov_threshold;           /*!< Vbus OV threshold */
  uint16_t rth_ot_threshold;            /*!< Rth OT threshold */
  uint16_t rth_ot_hysteresis_threshood; /*!< Rth OT hysteresis threshold */
  int32_t efficiency_threshold;         /*!< FOD threshold */
  uint32_t flash_valid_key;             /*!< Key */
  uint32_t flash_checksum;              /*!< Checksum */
  
} FlashConfigData;


/****************************************************************************************************/
/*																									*/
/*			ledCntrol																              			*/ 
/*																									*/
/*																									*/
/****************************************************************************************************/
extern void app_flash_config_set_defaults(void);

#endif // APP_FLASH_CONFIG

