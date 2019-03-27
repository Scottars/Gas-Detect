#ifndef _beep_H
#define _beep_H
#include "stm32f10x.h"
#include "public.h"
#define BZ GPIO_Pin_5 //PB5	 ¶¨Òå¶Ë¿ÚPB5
void sound1(void);
void sound2(void);
void BEEP_Init(void);

#endif
