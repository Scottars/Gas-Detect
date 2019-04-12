#ifndef _adc_H
#define _adc_H
#include "stm32f10x.h"
void adc_init(void);
u16 Get_ADC_Value(u8 channe,u8 times);
float *AD_Conversion(void);


float AD_Conversion_1479A(void);

float AD_Conversion_627D(void);

float AD_Conversion_025D(void);

#endif
