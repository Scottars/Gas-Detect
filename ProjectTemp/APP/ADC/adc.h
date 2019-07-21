#ifndef _adc_H
#define _adc_H
#include "stm32f10x.h"

void adc10_init();
	void adc11_init();
		void adc12_init();
u16 Get_ADC_Value(u8 channe,u8 times);
float *AD_Conversion(void);


float AD_Conversion_1479A(int times);

float AD_Conversion_627D(int times);

float AD_Conversion_025D(int times);

#endif
