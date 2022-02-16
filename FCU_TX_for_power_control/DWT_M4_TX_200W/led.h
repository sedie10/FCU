#ifndef _LED_H
#define _LED_H

#include "Targetdef.h"

#define LED_RED                         						PE5
#define LED_GREEN                       						PE4
#define LED_BLUE                         						PE3

extern void led_state_machine(void);

#endif
