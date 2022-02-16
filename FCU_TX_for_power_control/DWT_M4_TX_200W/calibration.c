#include "application.h"
 
int adc_offset_vsense_digital = 512;
int adc_offset_isense = 512;

void system_calibrate(void)
{
	int counts_sum_vsense_digital = 0;
	int counts_sum_isense = 0;
	volatile int i;
	volatile int j;

	// Run a bunch of samples at zero current to calculate offset

	for (i = 0; i < 16; i++)
	{
		// Delay

		for (j = 0; j < 20; j++);

		// Collect sample

		counts_sum_vsense_digital += vsense_digital_counts();
		counts_sum_isense += isense_counts();
	}

	adc_offset_vsense_digital = counts_sum_vsense_digital / 16;
	adc_offset_isense = counts_sum_isense / 16;

}

