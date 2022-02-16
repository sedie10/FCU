#ifndef _PWM_H
#define _PWM_H

#include "NuMicro.h"
#include "Targetdef.h"



/*
 * HCLK/ACLK definitions
 */
   
/**********************    FCLK = 80MHz **************************/

#define NUM_WAIT_STATES                 FLASH_WSTATE_HCLK_LTE_25MHZ

#ifdef USE_40MHZ
#define TIMERA_TICKS_100KHZ             400                         /*!< Number of ticks for 100kHz for Timer A (ACLK /1) */
#define TIMERA_TICKS_110KHZ             364	                        /*!< Number of ticks for 110kHz for Timer A (ACLK /1) */
#define TIMERA_TICKS_120KHZ             333	                        /*!< Number of ticks for 120kHz for Timer A (ACLK /1) */
#define TIMERA_TICKS_130KHZ             308	                        /*!< Number of ticks for 130kHz for Timer A (ACLK /1) */
#define TIMERA_TICKS_135KHZ             296	                        /*!< Number of ticks for 135kHz for Timer A (ACLK /1) */
#define TIMERA_TICKS_140KHZ             286	                        /*!< Number of ticks for 140kHz for Timer A (ACLK /1) */
#define TIMERA_TICKS_145KHZ             276
#define TIMERA_TICKS_150KHZ             267	                        /*!< Number of ticks for 150kHz for Timer A (ACLK /1) */
#define TIMERA_TICKS_155KHZ             258
#define TIMERA_TICKS_160KHZ             250	                        /*!< Number of ticks for 160kHz for Timer A (ACLK /1) */
#define TIMERA_TICKS_165KHZ             242 
#define TIMERA_TICKS_170KHZ             235
#define TIMERA_TICKS_175KHZ             228                         /*!< Number of ticks for 175kHz for Timer A (ACLK /1) */
#define TIMERA_TICKS_180KHZ             222                         /*!< Number of ticks for 180kHz for Timer A (ACLK /1) */
#define TIMERA_TICKS_205KHZ             195                         /*!< Number of ticks for 205kHz for Timer A (ACLK /1) */
#elif (defined(USE_36MHZ))
#define TIMERA_TICKS_100KHZ             360                         /*!< Number of ticks for 100kHz for Timer A (ACLK /1) */
#define TIMERA_TICKS_110KHZ             327	                        /*!< Number of ticks for 110kHz for Timer A (ACLK /1) */
#define TIMERA_TICKS_117_7KHZ           327	                        /*!< Number of ticks for 117.7kHz for Timer A (ACLK /1) */
#define TIMERA_TICKS_120KHZ             300	                        /*!< Number of ticks for 120kHz for Timer A (ACLK /1) */
#define TIMERA_TICKS_128KHZ             279	                        /*!< Number of ticks for 128.8kHz for Timer A (ACLK /1) */
#define TIMERA_TICKS_130KHZ             277	                        /*!< Number of ticks for 130kHz for Timer A (ACLK /1) */
#define TIMERA_TICKS_135KHZ             266	                        /*!< Number of ticks for 135kHz for Timer A (ACLK /1) */
#define TIMERA_TICKS_140KHZ             257	                        /*!< Number of ticks for 140kHz for Timer A (ACLK /1) */
#define TIMERA_TICKS_145KHZ             248
#define TIMERA_TICKS_150KHZ             240	                        /*!< Number of ticks for 150kHz for Timer A (ACLK /1) */
#define TIMERA_TICKS_155KHZ             232
#define TIMERA_TICKS_160KHZ             225	                        /*!< Number of ticks for 160kHz for Timer A (ACLK /1) */
#define TIMERA_TICKS_165KHZ             218 
#define TIMERA_TICKS_170KHZ             212
#define TIMERA_TICKS_175KHZ             206                         /*!< Number of ticks for 175kHz for Timer A (ACLK /1) */
#define TIMERA_TICKS_180KHZ             200                         /*!< Number of ticks for 180kHz for Timer A (ACLK /1) */
#define TIMERA_TICKS_205KHZ             175                         /*!< Number of ticks for 205kHz for Timer A (ACLK /1) */
#elif (defined(USE_96_CENTER_ALIGN_MHZ))
#define TIMERA_TICKS_100KHZ             960                         /*!< Number of ticks for 100kHz for Timer A (ACLK /1) */
#define TIMERA_TICKS_110KHZ             872 //KYJ TEST	                        /*!< Number of ticks for 110kHz for Timer A (ACLK /1) */
#define TIMERA_TICKS_120KHZ             800	                        /*!< Number of ticks for 120kHz for Timer A (ACLK /1) */
#define TIMERA_TICKS_127_7KHZ           752	                        /*!< Number of ticks for 117.7kHz for Timer A (ACLK /1) */
#define TIMERA_TICKS_130KHZ             739	                        /*!< Number of ticks for 130kHz for Timer A (ACLK /1) */
#define TIMERA_TICKS_135KHZ             711	                        /*!< Number of ticks for 135kHz for Timer A (ACLK /1) */
#define TIMERA_TICKS_140KHZ             686	                        /*!< Number of ticks for 140kHz for Timer A (ACLK /1) */
#define TIMERA_TICKS_145KHZ             662
#define TIMERA_TICKS_150KHZ             640	                        /*!< Number of ticks for 150kHz for Timer A (ACLK /1) */
#define TIMERA_TICKS_155KHZ             620
#define TIMERA_TICKS_160KHZ             600	                        /*!< Number of ticks for 160kHz for Timer A (ACLK /1) */
#define TIMERA_TICKS_165KHZ             582 
#define TIMERA_TICKS_170KHZ             565
#define TIMERA_TICKS_175KHZ             549                         /*!< Number of ticks for 175kHz for Timer A (ACLK /1) */
#define TIMERA_TICKS_180KHZ             534                         /*!< Number of ticks for 180kHz for Timer A (ACLK /1) */
#define TIMERA_TICKS_205KHZ             468                         /*!< Number of ticks for 205kHz for Timer A (ACLK /1) */
#endif


#define DUTY_DIVIDE                     1  /*  12.5% */
#define TIMERA_DIGITAL_PING_FREQ        				TIMERA_TICKS_145KHZ
#define TIMERA_DIGITAL_PING_V_CNTRL_DUTY        (TIMERA_DIGITAL_PING_FREQ >> 1 )
#define TIMERA_DIGITAL_PING_DUTY        				(TIMERA_TICKS_110KHZ >> DUTY_DIVIDE )

#define BPP_MAX_POWER_5W                5
#define BPP_MAX_POWER_10W               10
#define EPP_MAX_POWER_15W               15
#define EPP_MAX_POWER_30W               30

#define RTC_1P5S_PERIOD_TICKS          1500 /* 0x16E360 */                       /*!< Number of ticks for 1.5s RTC @ 4MHz FRCLK, /4 */
#define RTC_0P25S_PERIOD_TICKS         250 /* 0x03d090 */                       /*!< Number of ticks for 250ms RTC @ 4MHz FRCLK, /4 */
#define RTC_0P5S_PERIOD_TICKS						500 /* 0x03d090 */                       /*!< Number of ticks for 500ms RTC @ 4MHz FRCLK, /4 */

#ifdef VOLTAGE_CONTROL
extern void PWM_vStartBuckbooster(void);
extern void PWM_vStopBuckbooster(void);
extern void PWM_vUpdateBuckbooster(uint16_t period_ticks, uint16_t duty_ticks_high_side);
extern void PWM_vInitBuckbooster( uint16_t period_ticks, uint16_t duty_ticks_high_side);
#endif

extern void PWM_vUpdateCoil(uint16_t period_ticks, uint16_t duty_ticks_high_side);
extern void PWM_vStopCoil(void);
extern void PWM_vStartCoil(void);
extern void PWM_vInitCoil( uint16_t period_ticks, uint16_t duty_ticks_high_side);

#endif
