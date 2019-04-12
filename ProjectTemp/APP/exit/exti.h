#ifndef _exti_H
#define _exti_H
#include "stm32f10x.h"

#include "printf.h"
#define k_left GPIO_Pin_2  //K1 PE2

void exti_init(void);  //外部中断初始化

void exti_disable(void);  //外部中断初始化

void Timing_Signal_init(void);

void Timing_Signal_Check(void); // get the timing signal is here or not  

#endif 
