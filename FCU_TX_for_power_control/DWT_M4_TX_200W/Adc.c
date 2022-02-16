#include "Adc.h"

uint32_t adc_test =0;

/****************************************************************************************************/
/*																									*/
/*			ADC_Convert																				*/ 
/*																									*/
/*																									*/
/****************************************************************************************************/
uint16_t ADC_Convert(uint8_t channel)
{	

	uint8_t ch;

	switch(channel)
	{
		case CHANNEL_TMP:  //TEMP
			ch = 5;
		break;
		case CHANNEL_CUR:  //AMP_I_OUT
			ch = 4;
		break;
		case CHANNEL_VBUS:  //VPA
			ch = 3;
		break;
		case Q_RESONANCE_VOLT:  // Q_RESONANCE_VOLT
			ch = 1;
		break;
		case CHANNEL_OVERVOLT:  // VCOIl_OV
			ch = 8;
		break;

		case FO_BALANCED_V:  // VCOIl_OV
			ch = 9;
		break;
		
		case Q_PEAK_HOLD_DATA:  // VCOIl_OV
			ch = 0;
		break;
		
	}

	/* Enable the sample module 7 interrupt */            
	EADC_ENABLE_INT(EADC, BIT0);
	
	//Enable sample module  A/D ADINT0 interrupt.            
	EADC_ENABLE_SAMPLE_MODULE_INT(EADC, 0,  BIT0<<ch);
	
	//Enable sample module 7 interrupt.            

//	NVIC_EnableIRQ(EADC00_IRQn);
	/* Reset the ADC indicator and trigger sample module 7 to start A/D conversion */       

	//g_u32AdcIntFlag = 0;            
	
	EADC_START_CONV(EADC, BIT0<<ch);            
//	__WFI();
	
	/* Disable the sample module  interrupt */            
	EADC_DISABLE_SAMPLE_MODULE_INT(EADC, 0, BIT0<<ch);
	
	/* Wait conversion done */            
	while(EADC_GET_DATA_VALID_FLAG(EADC, (BIT0<<ch)) != (BIT0<<ch));
//  while(g_u32AdcIntFlag == 0);	

//	NVIC_DisableIRQ(EADC00_IRQn);
	EADC_DISABLE_INT(EADC, BIT0);
//	EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF0_Msk);    
	/* Get the conversion result of the sample module */        
	adc_test = ( EADC->DAT[ch] ) & ( 0x0fff );
	if(ch == 3 )
	{
		adc_test = ( EADC->DAT[3] ) & ( 0x0fff );		
		return adc_test;
	}
	else
	{
		adc_test = ( EADC->DAT[ch] ) & ( 0x0fff );
		return adc_test; // 12bits
	}
}

