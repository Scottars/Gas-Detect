#ifndef _exti_H
#define _exti_H
#include "stm32f10x.h"

#include "printf.h"


void exti_init(void);  //外部中断初始化

void exti_disable(void);  //外部中断初始化

void Timing_Signal_init(void);

void Timing_Signal_Check(void); // get the timing signal is here or not  

extern u8 Normal_Puff_RunningMode;
extern u8 PEV_1479A_ControlMode;

#endif 
