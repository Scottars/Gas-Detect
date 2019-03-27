#ifndef _smg_H
#define _smg_H
#include "stm32f10x.h"
#include "systick.h"

#define smg_duan (GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7)//PC0~PC7 

void smg_init(void);//数码管初始化
void static_smg_display(void);	//静态数码管显示

#endif
