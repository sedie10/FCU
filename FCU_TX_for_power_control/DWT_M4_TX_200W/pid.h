#ifndef PID_H
#define PID_H

#include "Targetdef.h"


typedef float               fix16_t;

#define FREQ_KP_FIX16                            1         /*!< Fs Proporational gain:  */

#define FREQ_KI_FIX16                            0.05         /*!< Fs Integral gain:       0.05mA^-1ms^-1    */ 
#define FREQ_MI_FIX16                            3000         /*!< Fs Integral term limit: 3000              */
#define FREQ_MPID_FIX16                          20000         /*!< Fs PID output limit:    20000             */
#define DUTY_KP_FIX16                            0.014         /*!< Duty Proporational gain: 0.0014 */
#ifdef VOLTAGE_CONTROL
#define VOLT_KP_FIX16                            0.1
#endif
#define FREQ_LKP_FIX16                           8         /*!< Fs Proporational gain:0x50000:                 5mA^-1            */
#define FREQ_LKI_FIX16                           0.05         /*!< Fs Integral gain:0x051E                        0.02mA^-1ms^-1    */ 
#define FREQ_LMI_FIX16                           3000         /*!< Fs Integral term limit:0x03E80000              1,000              */
#define FREQ_LMPID_FIX16                         20000         /*!< Fs PID output limit:0x27100000                 10,000             */

#define DUTY_LKP_FIX16                           0.0015         /*!< Duty Proporational gain: 0.001535 */

#define FREQ_KHZ_LCUTOFF_FIX16                   145.000         /*!< 91 145 */
#define FREQ_KHZ_CUTOFF_FIX16                    145.000         /*!< 180 */
#define FREQ_KHZ_MAX_VIRT_FIX16                  430.000         /*!< Maximum virtual frequency (430 kHz) */
#define DUTY_CUTOFF_THRESHOLD                    0.5           /* duty cutoff threshold  */

#define TINNER_FIX16                             0.001         /*!< Tinner (Tctl):          1ms               */
#define ONE_DIV_TINNER_FIX16                     1000         /*!< 1/Tinner (Fctl):        1000Hz            */ 

#define MIN_DUTY																	0.007


#if 0
#define FREQ_SV_110KHZ_LTE_FS_LT_115KHZ_FIX16     1.2          /*!< Scaling factor (110kHz <= Fs < 140kHz: 1.5 */
#define FREQ_SV_115KHZ_LTE_FS_LT_120KHZ_FIX16     1.3          /*!< Scaling factor (110kHz <= Fs < 140kHz: 1.5 */
#define FREQ_SV_120KHZ_LTE_FS_LT_135KHZ_FIX16     1.5          /*!< Scaling factor (110kHz <= Fs < 140kHz: 1.5 */
#define FREQ_SV_135KHZ_LTE_FS_LT_140KHZ_FIX16     1.7          /*!< Scaling factor (110kHz <= Fs < 140kHz: 1.5 */
#define FREQ_SV_140KHZ_LTE_FS_LT_145KHZ_FIX16     2.0          /*!< Scaling factor (110kHz <= Fs < 140kHz: 1.5 */
#define FREQ_SV_145KHZ_LTE_FS_LT_205KHZ_FIX16     2.5          /*!< Scaling factor (140kHz <= Fs < 160kHz: 2 */
#define FREQ_SV_205KHZ_LTE_FS_LTE_430KHZ_FIX16    4.0          /*!< Scaling factor (160kHz <= Fs < 180kHz: 3 */
#else
#define FREQ_SV_110KHZ_LTE_FS_LT_115KHZ_FIX16     1.0          /*!< Scaling factor (110kHz <= Fs < 140kHz: 1.5 */
#define FREQ_SV_115KHZ_LTE_FS_LT_120KHZ_FIX16     1.1          /*!< Scaling factor (110kHz <= Fs < 140kHz: 1.5 */
#define FREQ_SV_120KHZ_LTE_FS_LT_135KHZ_FIX16     1.2          /*!< Scaling factor (110kHz <= Fs < 140kHz: 1.5 */
#define FREQ_SV_135KHZ_LTE_FS_LT_140KHZ_FIX16     1.3          /*!< Scaling factor (110kHz <= Fs < 140kHz: 1.5 */
#define FREQ_SV_140KHZ_LTE_FS_LT_145KHZ_FIX16     1.4          /*!< Scaling factor (110kHz <= Fs < 140kHz: 1.5 */
#define FREQ_SV_145KHZ_LTE_FS_LT_205KHZ_FIX16     1.5          /*!< Scaling factor (140kHz <= Fs < 160kHz: 2 */
#define FREQ_SV_205KHZ_LTE_FS_LTE_430KHZ_FIX16    1.6          /*!< Scaling factor (160kHz <= Fs < 180kHz: 3 */
#endif

#define FREQ_SV_110KHZ_HTE_FS_LT_115KHZ_FIX16     1.5          /*!< Scaling factor (110kHz <= Fs < 115kHz: 1.5 */
#define FREQ_SV_115KHZ_HTE_FS_LT_120KHZ_FIX16     2.0          /*!< Scaling factor (115kHz <= Fs < 120kHz: 2 */
#define FREQ_SV_120KHZ_HTE_FS_LT_135KHZ_FIX16     3.0         /*!< Scaling factor (120kHz <= Fs < 135kHz: 3 */
#define FREQ_SV_135KHZ_HTE_FS_LT_140KHZ_FIX16     5.0          /*!< Scaling factor (135kHz <= Fs <= 140kHz: 5 */
#define FREQ_SV_140KHZ_HTE_FS_LT_145KHZ_FIX16     10.0          /*!< Scaling factor (140kHz <= Fs <= 145kHz: 10 */
#define FREQ_SV_145KHZ_HTE_FS_LT_205KHZ_FIX16     16.0          /*!< Scaling factor (145kHz <= Fs <= 205kHz: 16 */
#define FREQ_SV_205KHZ_HTE_FS_LTE_430KHZ_FIX16    64.0          /*!< Scaling factor (205kHz <= Fs <= 430kHz: 64 */

#ifdef USE_MIN_100KHZ
#define FREQ_KHZ_MIN_FIX16              110.000          /*!< Freq(min):6E 110kHz */
#else
#define FREQ_KHZ_MIN_FIX16              110.000          /*!< Freq(min): 110kHz */   
#endif
#define FREQ_KHZ_MAX_FIX16              145.000          /*!< Freq(max): 145kHz */

#define FREQ_KHZ_MAX_MIN_FIX16          180.000          /*!< Freq(max minimum): 180kHz */

#define FREQ_KHZ_110_FIX16                        110.000          /*!< 110kHz in fix16_t format */
#define FREQ_KHZ_115_FIX16                        115.000          /*!< 115kHz in fix16_t format */
#define FREQ_KHZ_117_7_FIX16                      117.700          /*!< 117.7kHz in fix16_t format */
#define FREQ_KHZ_120_FIX16                        120.000          /*!< 120kHz in fix16_t format */
#define FREQ_KHZ_128_FIX16                        128.800          /*!< 128.8kHz in fix16_t format */
#define FREQ_KHZ_135_FIX16                        135.000          /*!< 135kHz in fix16_t format */
#define FREQ_KHZ_140_FIX16                        140.000          /*!< 140kHz in fix16_t format */
#define FREQ_KHZ_145_FIX16                        145.000          /*!< 145kHz in fix16_t format */
#define FREQ_KHZ_160_FIX16                        160.000          /*!< 160kHz in fix16_t format */
#define FREQ_KHZ_180_FIX16                        180.000          /*!< 180kHz in fix16_t format */
#define FREQ_KHZ_205_FIX16                        205.000          /*!< 205kHz in fix16_t format */


#ifdef VOLTAGE_CONTROL
#define FREQ_KHZ_PING_FIX16                     	FREQ_KHZ_110_FIX16 /*!< 140kHz in fix16_t format */
#else
#define FREQ_KHZ_PING_FIX16                     	FREQ_KHZ_205_FIX16 /*!< 205kHz in fix16_t format */
#endif
#define VDROOP_DOWN_KP_FIX16                    	1.75  /*0x0001C000*/           /*!< 1.1 when adjusting frequency up (reduce power) */

#define VDROOP_MIN_FREQ_FIX16                    (250 << 16)          /*!< Max Minimum virtual frequency to use when drooping */

#define MP_DUTY_CUTOFF_THRESHOLD             			0.5 /* 0x00008000 */

#define LP_DUTY_CUTOFF_THRESHOLD             			0.5 /* 0x00008000 */


extern void WPCQi_InitHandleCEP(void);
extern __ramfunc void WPCQi_HandleControlErrorPacket(int update_cep);         /*!< Runs the control loop. update_cep set when new CEP packet has arrived */
extern void WPCQi_InitFrequencyHandleCEP(void);

#ifdef VOLTAGE_CONTROL
extern void WPCQi_InitVoltageHandleCEP(fix16_t init_voltage_value );
#endif

void WPCQi_setPreFrequencyHandleCEP (fix16_t frequency );

#endif // PID_H
