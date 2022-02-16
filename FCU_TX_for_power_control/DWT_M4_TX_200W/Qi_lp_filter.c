#include "WPCQiTx.h"
#include "modulation.h"

int lp_alpha;

// Filter output history

static int counts_filter_prev = 0;

int lp_filter_hys;                 /* voltage */

__ramfunc void qi_lp_filter_reset(void)
{
  //if (modulation_type == VoltagePeak)
  //  counts_filter_prev = vsense_digital_counts();
  //else
   // counts_filter_prev = isense_counts();
}

__ramfunc int qi_lp_filter(int logic_level_prev)
{
	fix16_t counts_filter;
	int threshold;
	int counts, v_counts, i_counts, temp;

	// Take ADC sample

	v_counts = vsense_digital_counts();
	i_counts = isense_counts();

	if (modulation_type == VoltagePeak)
		counts = v_counts;
	else
		counts = i_counts;

	// Low-pass filter on ADC reading to maintain mid-point

//	counts_filter = fix16_from_int(counts - counts_filter_prev);
//	counts_filter = fix16_mul(counts_filter, lp_alpha);               
//	counts_filter = fix16_to_int(counts_filter + 0x8000) + counts_filter_prev;

	counts_filter = (float)(counts - counts_filter_prev);
	counts_filter = (float)(counts_filter * lp_alpha);               
	counts_filter = (int32_t)((counts_filter + 0x8000) + counts_filter_prev);

	counts_filter_prev = counts_filter;

	// Hysteresis

	if (logic_level_prev == 0)
		threshold = counts_filter_prev + lp_filter_hys;
	else 
		threshold = counts_filter_prev - lp_filter_hys;

	if (collecting_dynamic_hysteresis)
	{    

		// Calculate adaptive hysteresis

		/* Lowpass filter using int32 only */
		/* y(i) = y(i-1) + alpha * (x(i) - y(i-1)), where alpha = dt / (tau + dt) and tau = RC */

		/* with c = (tau+dt)/dt follows: 
		* y(i)*c = y(i-1)*c + x(i) - y(i-1) 
		*
		* following corner frequencies can be achieved 
		* with td = 25e-6s, and c = 2^x for easy division using right shift tau = c*dt - dt
		*     c   tau            f_c
		*      
		*  2^4    16    0.375ms  2667Hz
		*  2^5    32    0.775ms  1290Hz   
		*  2^6    64    1.575ms   635Hz
		*  2^7   128    3.175ms   315Hz 
		*  2^8   256    6.375ms   157Hz***
		*  2^9   512   12.775ms    78Hz
		*/ 

		// Filter input: abs(adc_counts - filter_output) * scale

		temp = (counts_filter - counts);
		if (temp < 0) 
			temp = -temp;

		// Only accumulate dynamic hysteresis if abs (counts-counts_filter) > (dynamic_hysteresis/16*50%)

		if (temp > (dynamic_hysteresis >> 5)) // / 16 / 2
			dynamic_hysteresis += (temp - (dynamic_hysteresis >> 4));
	}

	// Determine logic level from filter comparison

	if (counts < threshold)
	{
		temp = 0;
	}else{
		temp = 1;
	}

	return temp;

}

