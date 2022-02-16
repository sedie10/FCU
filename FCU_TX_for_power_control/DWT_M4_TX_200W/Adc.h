#ifndef _ADC_H
#define _ADC_H

#include "Targetdef.h"
#include "NuMicro.h"

#define CHANNEL_VBUS                             0
#define CHANNEL_CUR                              1
#define CHANNEL_TMP                              2
#define Q_RESONANCE_VOLT                         3
#define CHANNEL_OVERVOLT                         4
#define Q_PEAK_HOLD_DATA                         5
#define FO_BALANCED_V                            6



#ifdef QI_TX
#ifdef OVER_VOLTAGE_CONTROL
#if 0 // Vref: 5V
#define MAXIMUM_VOLTAGE_RATIO    40
#define MAXIMUM_VOLTAGE_OFFSET_30W   ( 274 - 15 )
//#define MAXIMUM_VOLTAGE_OFFSET   ( 274 - 100 )
#define MAXIMUM_VOLTAGE_OFFSET_15W   ( 274 - 110 )
#else
#define MAXIMUM_VOLTAGE_RATIO    40
#define MAXIMUM_VOLTAGE_OFFSET_30W   518  /*( 274 - 15 )*/
#define MAXIMUM_VOLTAGE_OFFSET_15W   328  /*( 274 - 110 )*/
#endif
#endif

#endif


#define _VREF_2_5V      0
#define _VREF_3_0V      1
#define _VREF_5_0V      0


#define DCDC_PWM_Q_MEAS             				200
//#define DCDC_PWM_POWER_TRANSFER_15W_INIT    190
//#define DCDC_PWM_POWER_TRANSFER_15W_INIT    183
//#define DCDC_PWM_POWER_TRANSFER_15W_INIT    170
//#define DCDC_PWM_POWER_TRANSFER_INIT    		170
//#define DCDC_PWM_POWER_TRANSFER_15W_INIT    165
//#define DCDC_PWM_POWER_TRANSFER_INIT    		165
#define DCDC_PWM_POWER_TRANSFER_15W_INIT    190
#define DCDC_PWM_POWER_TRANSFER_INIT    		170
//#define DCDC_PWM_PING               				183
//#define DCDC_PWM_PING               				170
//yy.lee
//#define DCDC_PWM_PING               				165
#define DCDC_PWM_PING               				180
#define DCDC_PWM_PING_C0               			185
#define DCDC_PWM_APING											192
#define DCDC_PWM_200W_TEST						165

//#define DCDC_PWM_INIT               				190
#define DCDC_PWM_INIT               				DCDC_PWM_PING
#define DCDC_PWM_MAX                				(200 - 8 )
//#define DCDC_PWM_MAX                				(200 - 3 )
#define DCDC_PWM_MAX_C0                				(200 - 0 )
#define DCDC_PWM_MIN  											(200 - 198 )
#define DCDC_PWM_MIN_C0  											(200 - 40 )
//#define DCDC_PWM_RATIO_100            			200
#define DCDC_PWM_RATIO_200            			200


//#define QI_GUARANTEED_POWER_C0									0x3C   /* 30W */
//#define QI_POTENTIAL_POWER_C0									0x3C   /* 30W */
#define QI_GUARANTEED_POWER_C0									0x28   /* 30W */
#define QI_POTENTIAL_POWER_C0									0x28   /* 30W */
//#define QI_GUARANTEED_POWER_C0									0x1F   /* 15W */
//#define QI_POTENTIAL_POWER_C0									0x1F   /* 15W */
#define QI_GUARANTEED_POWER									0x28   /* 200W */
#define QI_POTENTIAL_POWER									0x28   /* 200W */


//#ifdef VBUS_24V_APP
// VBUS_24V
// R1 : 51k, 10K,

//#define ADC_GAIN                                0.0012
//#define ADC_GAIN                                0.00233
//#define ADC_GAIN                                0.000233
//#define ADC_GAIN                                0.000117
// Rsense : 0.02, Gain : 37.9, Vref : 5V, ADC resolution 10bit
//#define ADC_GAIN                                0.00644
//#define ADC_GAIN                                0.00488
// Rsense : 0.02, Gain : 37.9, Vref : 2.5V, ADC resolution 10bit
//#define ADC_GAIN                                0.00644
#define ADC_GAIN                                0.00322

#define ADC_AD5_RTH_THRESHOLD                   91             /*85C */
#define ADC_AD5_RTH_HYSTERESIS                  91             /*85C */

#if _VREF_5_0V  // V ref  :  5V
#define ADC_VBUS_3                               96             /* 3V */
#define ADC_VBUS_4                              130             /* 4V */
#define ADC_VBUS_5                              164             /* 5V */
#define ADC_VBUS_6                              198             /* 6V */
#define ADC_VBUS_7                              231             /* 7V */
#define ADC_VBUS_8                              265             /* 8V */
#define ADC_VBUS_9                              299             /* 9V */
#define ADC_VBUS_10                             333             /* 10V */
#define ADC_VBUS_11                             366             /* 11V */
#define ADC_VBUS_12                             400             /* 12V */
#define ADC_VBUS_13                             434             /* 13V */
#define ADC_VBUS_14                             468             /* 14V */
#define ADC_VBUS_15                             502             /* 15V */
#define ADC_VBUS_16                             536             /* 16V */
#define ADC_VBUS_17                             571             /* 17V */
#define ADC_VBUS_18                             604             /* 18V */
#define ADC_VBUS_19                             638             /* 19V */
#define ADC_VBUS_20                             671             /* 20V */  
#define ADC_VBUS_21                             705             /* 21V */
#define ADC_VBUS_22                             739             /* 22V */
#define ADC_VBUS_23                             772             /* 23V */
#define ADC_VBUS_24                             806             /* 24V */
#define ADC_VBUS_25                             839             /* 25V */
#define ADC_VBUS_26                             873             /* 26V */
#define ADC_VBUS_27                             906             /* 27V */  
#define ADC_VBUS_28                             940             /* 28V */ 
#define ADC_VBUS_29                             970             /* 28V */ 
#elif( _VREF_2_5V )   
// V ref  : 2.5V  51k : 2.7k
#define ADC_VBUS_2                              165             /* 2V */
#define ADC_VBUS_3                              247             /* 3V */
#define ADC_VBUS_4                              330             /* 4V */
#define ADC_VBUS_5                              412             /* 5V */
#define ADC_VBUS_6                              494             /* 6V */
#define ADC_VBUS_7                              577             /* 7V */
#define ADC_VBUS_8                              659             /* 8V */
#define ADC_VBUS_9                              741             /* 9V */
#define ADC_VBUS_10                             824             /* 10V */
#define ADC_VBUS_11                             906             /* 11V */
#define ADC_VBUS_12                             989             /* 12V */
#define ADC_VBUS_13                            1071             /* 13V */
#define ADC_VBUS_14                            1153             /* 14V */
#define ADC_VBUS_15                            1236             /* 15V */
#define ADC_VBUS_16                            1318             /* 16V */
#define ADC_VBUS_17                            1400             /* 17V */
#define ADC_VBUS_18                            1483             /* 18V */
#define ADC_VBUS_19                            1565             /* 19V */
#define ADC_VBUS_20                            1648             /* 20V */  
#define ADC_VBUS_21                            1730             /* 21V */
#define ADC_VBUS_22                            1812             /* 22V */
#define ADC_VBUS_23                            1895             /* 23V */
#define ADC_VBUS_24                            1977             /* 24V */
#define ADC_VBUS_25                            2059             /* 25V */
#define ADC_VBUS_26                            2142             /* 26V */
#define ADC_VBUS_27                            2224             /* 27V */  
#define ADC_VBUS_28                            2307             /* 28V */ 
#define ADC_VBUS_29                            2389             /* 29V */ 
#define ADC_VBUS_30                            2471             /* 30V */
#define ADC_VBUS_31                            2554             /* 31V */  
#define ADC_VBUS_32                            2636             /* 32V */
#define ADC_VBUS_33                            2718             /* 33V */
#define ADC_VBUS_34                            2801             /* 34V */
#define ADC_VBUS_35                            2883             /* 35V */
#define ADC_VBUS_36                            2966             /* 36V */
#define ADC_VBUS_37                            3048             /* 37V */
#define ADC_VBUS_38                            3130             /* 38V */  
#define ADC_VBUS_39                            3213             /* 39V */ 
#define ADC_VBUS_40                            3295             /* 40V */ 
#define ADC_VBUS_41                            3377             /* 41V */ 
#define ADC_VBUS_42                            3460             /* 42V */ 
#define ADC_VBUS_43                            3542             /* 43V */ 
#define ADC_VBUS_44                            3625             /* 44V */ 
#define ADC_VBUS_45                            3707             /* 45V */ 
#define ADC_VBUS_46                            3789             /* 46V */ 
#define ADC_VBUS_47                            3872             /* 47V */ 
#define ADC_VBUS_48                            3954             /* 48V */ 
#define ADC_VBUS_49                            4037             /* 49V */ 
#define ADC_VBUS_50                            4119             /* 50V */ 
#else
// V ref  : 3.0V  51k : 2.7k
#define ADC_VBUS_2                              137             /* 2V */
#define ADC_VBUS_3                              206             /* 3V */
#define ADC_VBUS_4                              275             /* 4V */
#define ADC_VBUS_5                              343             /* 5V */
#define ADC_VBUS_6                              412             /* 6V */
#define ADC_VBUS_7                              481             /* 7V */
#define ADC_VBUS_8                              549             /* 8V */
#define ADC_VBUS_9                              618             /* 9V */
#define ADC_VBUS_10                             686             /* 10V */
#define ADC_VBUS_11                             755             /* 11V */
#define ADC_VBUS_12                             824             /* 12V */
#define ADC_VBUS_13                             892             /* 13V */
#define ADC_VBUS_14                             961             /* 14V */
#define ADC_VBUS_15                            1030             /* 15V */
#define ADC_VBUS_16                            1098             /* 16V */
#define ADC_VBUS_17                            1167             /* 17V */
#define ADC_VBUS_18                            1236             /* 18V */
#define ADC_VBUS_19                            1304             /* 19V */
#define ADC_VBUS_20                            1373             /* 20V */  
#define ADC_VBUS_21                            1442             /* 21V */
#define ADC_VBUS_22                            1510             /* 22V */
#define ADC_VBUS_23                            1579             /* 23V */
#define ADC_VBUS_24                            1648             /* 24V */
#define ADC_VBUS_25                            1716             /* 25V */
#define ADC_VBUS_26                            1785             /* 26V */
#define ADC_VBUS_27                            1853             /* 27V */  
#define ADC_VBUS_28                            1922             /* 28V */ 
#define ADC_VBUS_29                            1991             /* 29V */ 
#define ADC_VBUS_30                            2059             /* 30V */
#define ADC_VBUS_31                            2128             /* 31V */  
#define ADC_VBUS_32                            2197             /* 32V */
#define ADC_VBUS_33                            2265             /* 33V */
#define ADC_VBUS_34                            2334             /* 34V */
#define ADC_VBUS_35                            2403             /* 35V */
#define ADC_VBUS_36                            2471             /* 36V */
#define ADC_VBUS_37                            2540             /* 37V */
#define ADC_VBUS_38                            2609             /* 38V */  
#define ADC_VBUS_39                            2677             /* 39V */ 
#define ADC_VBUS_40                            2746             /* 40V */ 
#define ADC_VBUS_41                            2815             /* 41V */ 
#define ADC_VBUS_42                            2883             /* 42V */ 
#define ADC_VBUS_43                            2952             /* 43V */ 
#define ADC_VBUS_44                            3021             /* 44V */ 
#define ADC_VBUS_45                            3089             /* 45V */ 
#define ADC_VBUS_46                            3158             /* 46V */ 
#define ADC_VBUS_47                            3226             /* 47V */ 
#define ADC_VBUS_48                            3295             /* 48V */ 
#define ADC_VBUS_49                            3364             /* 49V */ 
#define ADC_VBUS_50                            3432             /* 50V */ 
#define ADC_VBUS_51                            3501             /* 51V */ 
#define ADC_VBUS_52                            3570             /* 52V */ 
#define ADC_VBUS_53                            3638             /* 53V */ 
#define ADC_VBUS_54                            3707             /* 54V */ 
#define ADC_VBUS_55                            3776             /* 50V */ 
#define ADC_VBUS_56                            3844             /* 51V */ 
#define ADC_VBUS_57                            3913             /* 52V */ 
#define ADC_VBUS_58                            3982             /* 53V */ 
#define ADC_VBUS_59                            4050             /* 54V */ 
#endif

#define NOMINAL_VBUS                           ADC_VBUS_24
#define ADC_VBUS_UV                            ADC_VBUS_2
#define ADC_VBUS_OV                            4096


#define _30W_COIL_OV							130
#define _28W_COIL_OV							100
#define _11W_COIL_OV							70

#define ADC_COIL_OV                             _30W_COIL_OV

#define ADC_VBUS_VDROOP                         ADC_VBUS_19
#define ADC_VBUS_VDROOP_RESET                   ADC_VBUS_19

#if 0   // Vref: 5V
// 4.542 lsb unit
#define APP_CURRENT_HC 440  /* 2A */
#define APP_CURRENT_OC 500  /* 2.27A */
#else  // Vref: 2.5V
// 4.542 lsb unit
#define APP_CURRENT_HC 880  /* 2A */
#define APP_CURRENT_OC 1000  /* 2.27A */
#endif


//vbus_ov_threshold
#define VBUS_OV_THRESHOLD APP_VBUS_OV
//flash_config_ram.vbus_uv_threshold
#define VBUS_UV_THRESHOLD APP_VBUS_UV

#define HIGH_CURRENT      APP_CURRENT_HC
#define OVER_CURRENT      APP_CURRENT_OC

//#define TIMERA_TICKS_100KHZ             100.000

/****************************************************************************************************/
/*																									*/
/*			ADC_Convert																				*/ 
/*																									*/
/*																									*/
/****************************************************************************************************/
uint16_t ADC_Convert(uint8_t channel);


#endif
