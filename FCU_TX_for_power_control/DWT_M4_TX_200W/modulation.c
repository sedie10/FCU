#include "application.h"
#include "modulation.h"

#define NUM_PING_PONG                   4

#define LP_ALPHA_1K                    0.13574219 /* 0x22c0 */
#define LP_ALPHA_2K                    0.23904419 /* 0x3d32 */
#define LP_ALPHA_3K                    0.32029724 /* 0x51ff */
#define LP_ALPHA_4K                    0.38586426 /* 0x62C8 */
#define LP_ALPHA_5K                    0.43989563 /* 0x709d */

typedef struct
{
	ModulationType mod_type;
	int default_hys;
	fix16_t lp_alpha;
	int hys_scale_factor;
	ModulationState next_state;
} PingPongEntry;
 
typedef struct 
{
	ModulationState starting_state;
	PingPongEntry pp[NUM_PING_PONG];
} ModulationEntry;

ModulationState starting_mod_state;
PingPongEntry selected_mod_state[NUM_PING_PONG];

const ModulationEntry mod_states[1] = 
  { PingPong2, { { VoltagePeak, 5, LP_ALPHA_5K, 8, PingPong2 }, { VoltagePeak, 5, LP_ALPHA_3K, 12, PingPong4 }, { IsensePhase, 5, LP_ALPHA_2K, 4, PingPong4 }, { IsensePhase, 3, LP_ALPHA_2K, 10, PingPong2 } } };

// Globals

ModulationState modulation_state;                               /*!< Current type of modulation being used */
ModulationType modulation_type;                                 /*!< Current type of modulation being used (V or I) */
int hysteresis_scale_factor;                                    /*!< Current hysteresis scale factor */

// Local functions

static void changeModes(ModulationState new_state);

void initModulation(void) // 사용하는 데가 없어요...
{
  
	int i, index = 0;

	starting_mod_state = mod_states[index].starting_state;

	for (i = 0; i < NUM_PING_PONG; i++)
	{
		selected_mod_state[i].mod_type = mod_states[index].pp[i].mod_type;
		selected_mod_state[i].default_hys = mod_states[index].pp[i].default_hys;
		selected_mod_state[i].lp_alpha = mod_states[index].pp[i].lp_alpha;
		selected_mod_state[i].hys_scale_factor = mod_states[index].pp[i].hys_scale_factor;
		selected_mod_state[i].next_state = mod_states[index].pp[i].next_state;
	}
  
}

__ramfunc int resetHysteresis(void)
{
	return selected_mod_state[modulation_state].default_hys;
}

__ramfunc void resetDynamicHysteresis(void)
{
	dynamic_hysteresis = 0;
}
    
__ramfunc void resetModulation(void)
{
	changeModes(starting_mod_state);
}

__ramfunc int limitLpFilterHys(int raw_hys)
{
	raw_hys = (raw_hys * selected_mod_state[modulation_state].hys_scale_factor) >> 4;

	if (raw_hys < selected_mod_state[modulation_state].default_hys)
		raw_hys = selected_mod_state[modulation_state].default_hys;

	return raw_hys;
}

__ramfunc void changeModes(ModulationState new_state)
{
	resetDynamicHysteresis();

	modulation_state = new_state;

	modulation_type = selected_mod_state[new_state].mod_type;
	lp_filter_hys = selected_mod_state[new_state].default_hys;
	lp_alpha = selected_mod_state[new_state].lp_alpha;
	hysteresis_scale_factor = selected_mod_state[new_state].hys_scale_factor;
  
#if 0
  
  switch (modulation_state)
  {
  case PingPong1:
    
    led_off();
    
    break;
    
  case PingPong2:
    
    led_on_red();
    
    break;
    
  case PingPong3:
    
    led_on_green();
    
    break;
    
  case PingPong4:
    
    led_on_orange();
    
    break;

  }
  
#endif
  
}
    
__ramfunc void advanceModulation(void)
{
	modulation_state = selected_mod_state[modulation_state].next_state;

	changeModes(modulation_state);  
}
