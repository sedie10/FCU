#ifndef _WPCQITX_STANDBY_H
#define _WPCQITX_STANDBY_H

#include "Targetdef.h"

extern int aping_last;                                  /*!< Last sampled analog ping voltage */


int analog_ping_test(uint16_t threshold);
void quality_factor_measure_freq( uint32_t q_resonance_time );
#ifdef Q_MEASURE			
uint8_t quality_factor_measure_test_vbus_set(uint8_t fod_coil_sel);
__ramfunc uint8_t quality_factor_measure_test(void); 
#endif
#endif
