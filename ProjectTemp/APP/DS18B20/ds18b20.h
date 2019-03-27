#ifndef _DS18B20_H
#define _DS18B20_H
#include "stm32f10x.h"
#include "systick.h"
#define dq (GPIO_Pin_11) //PG11
void ds18b20_init(void);
void DQOUTINT(void);	 //输出配置
void DQININT(void);	 //输入配置
void ds18b20init(void);
void ds18b20wr(u8 dat);
u8 DS18b20rd(void);
u8 readtemp(void);
void RCCINIT_DS18B20(void);

//全部变量定义
extern u8 a,temp;

#endif
