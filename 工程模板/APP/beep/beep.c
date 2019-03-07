#include "beep.h"

/*******************************************************************************
* 函 数 名         : BEEP_Init
* 函数功能		   : 蜂鸣器端口初始化函数	   通过改变频率控制声音变化
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void BEEP_Init()	  //端口初始化
{
	GPIO_InitTypeDef GPIO_InitStructure;	//声明一个结构体变量，用来初始化GPIO

	SystemInit();
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);   /* 开启GPIO时钟 */

	/*  配置GPIO的模式和IO口 */
	GPIO_InitStructure.GPIO_Pin=BZ;		//选择你要设置的IO口
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;		  //设置推挽输出模式
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;	  //设置传输速率
	GPIO_Init(GPIOB,&GPIO_InitStructure); 	 /* 初始化GPIO */
}

/*******************************************************************************
* 函 数 名         : sound1
* 函数功能		   : 蜂鸣器报警函数
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void sound1()  //救护车报警
{
	u32 i=5000;
	while(i--)			   //产生一段时间的PWM波，使蜂鸣器发声
	{
		GPIO_SetBits(GPIOB,BZ);		 //IO口输出高电平
		delay(i);
		GPIO_ResetBits(GPIOB,BZ);	  //IO口输出低电平
		delay(i--);
	}	
}

/*******************************************************************************
* 函 数 名         : sound2
* 函数功能		   : 蜂鸣器报警函数	  通过改变频率控制声音变化
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void sound2()  //电动车报警
{
	u32 i=1000;
	while(i--)					//产生一段时间的PWM波，使蜂鸣器发声
	{
		GPIO_SetBits(GPIOB,BZ);	   //IO口输出高电平
		delay(i);
		GPIO_ResetBits(GPIOB,BZ);	//IO口输出低电平
		delay(i--);
	}	
}

