#include "led.h"
#include "WPCQiTx.h"


#define LED_BLUE_BLINKING				0x80
#define LED_BLUE_ON						0x40
#define LED_GREEN_BLINK_FAST			0x20
#define LED_GREEN_BLINK_SLOW			0x10
#define LED_GREEN_ON					0x08
#define LED_RED_BLINK_FAST				0x04
#define LED_RED_BLINK_SLOW				0x02
#define LED_RED_ON						0x01

#define LED_RED_GREEN_OFF				0xC0
#define LED_BLUE_OFF					0x3F

#define LED_FAST_BLINK_MASK             0xA4
#define LED_SLOW_BLINK_MASK             0x12


// LED State Table
typedef enum
{
	LED_IDLE_S = 0,
	LED_CHARGING_S,
	LED_CHARGE_DONE_S,
	LED_WARNING_S,
	LED_FAULT_S,
	LED_STATE_MAX_S
}ledState_t;

static ledState_t ledState = LED_IDLE_S;
static ledState_t lastLedState = LED_STATE_MAX_S;

/****************************************************************************************************/
/*																									*/
/*			ledCntrol																              			*/ 
/*																									*/
/*																									*/
/****************************************************************************************************/
static inline __ramfunc void ledCntrol(uint8_t red,uint8_t green, uint8_t blue)
{ 

	LED_RED 	= red; 
	LED_GREEN 	= green;
	LED_BLUE 	= blue;
}

/****************************************************************************************************/
/*																									*/
/*			led_state_machine																              	*/ 
/*																									*/
/*																									*/
/****************************************************************************************************/

__ramfunc void led_state_machine(void)
{

    static uint16_t led_ticks = 0;
	uint8_t led_mask = 0x00;
	static uint8_t last_mask = 0x00;
	
	if (over_temp_flag || qi_foreign_object_found )
		ledState = LED_WARNING_S;	
	else if (voltage_droop_flag)
		ledState = LED_FAULT_S;
	else if (qi_device_charging)
		ledState = LED_CHARGING_S;
	else if(qi_device_charged)
		ledState = LED_CHARGE_DONE_S;
	else
		ledState = LED_IDLE_S;

	if (ledState != lastLedState)
	{
		led_ticks = 0;
		lastLedState = ledState;
		led_mask &= LED_RED_GREEN_OFF;
	}
	
    led_ticks++;


    switch(ledState )
    {

		case LED_IDLE_S:
			led_mask &= LED_RED_GREEN_OFF;
			break;
			
		case LED_CHARGING_S:			

			led_mask |= LED_RED_ON;
			break;
			
		case LED_CHARGE_DONE_S:
			led_mask |= LED_GREEN_ON;
			break;
			
		case LED_WARNING_S:
			if( led_ticks % 4 < 2 )
			    led_mask |= LED_RED_ON;	
			else
			    led_mask &= (uint8_t)(~LED_RED_ON);	
			break;
			
		case LED_FAULT_S:
			if( led_ticks % 8 < 4)
			    led_mask |= LED_RED_ON;	
			else
			    led_mask &= (uint8_t)(~LED_RED_ON);	
			break;
			
		default:
			break;
			
    }
	
    if( commType == QI_COMMTYPE_BLE )
    {   
	    led_mask |= LED_BLUE_ON;
    }
	else if( qi_oob_flag == 0x01 )
	{
		if( led_ticks % 2 )
			led_mask |= LED_BLUE_ON; 
		else
			led_mask &= (uint8_t)(~LED_BLUE_ON); 
	}
	else
	{
	    led_mask &= LED_BLUE_OFF;
	}
	
    if( last_mask != led_mask )
    {
        ledCntrol((led_mask & LED_RED_ON ? 0:1),(led_mask & LED_GREEN_ON ? 0:1), (led_mask & LED_BLUE_ON ? 0:1));

		last_mask = led_mask;
	}
}

