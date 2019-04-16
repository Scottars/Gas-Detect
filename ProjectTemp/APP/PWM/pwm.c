#include "pwm.h"

/*******************************************************************************
* 函 数 名         : pwm_init
* 函数功能		   : IO端口及TIM3初始化函数	   
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void pwm_init(u16 pre, u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;   //声明一个结构体变量，用来初始化GPIO

	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;//声明一个结构体变量，用来初始化定时器

	TIM_OCInitTypeDef TIM_OCInitStructure;//根据TIM_OCInitStruct中指定的参数初始化外设TIMx

	/* 开启时钟 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);

	/*  配置GPIO的模式和IO口 */
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_7;    //TIM3 PC7
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;//复用推挽输出
	GPIO_Init(GPIOC,&GPIO_InitStructure);

	//Fpwm = 72M / ((arr+1)*(psc+1))(??:Hz)
	//duty circle = TIM3->CCR1 / arr(??:%)

	//TIM2定时器初始化
	TIM_TimeBaseInitStructure.TIM_Period = pre;	   //不分频,PWM 频率=72000/900=80Khz//设置自动重装载寄存器周期的值  计数的次数的设置
	TIM_TimeBaseInitStructure.TIM_Prescaler = psc;//设置用来作为TIMx时钟频率预分频值，100Khz计数频率
	TIM_TimeBaseInitStructure.TIM_ClockDivision =TIM_CKD_DIV1;//设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;	//TIM向上计数模式
	TIM_TimeBaseInit(TIM3, & TIM_TimeBaseInitStructure);
	
	
	/*
	  关于这部分的说明
		pwm_init(900,0);	 //PWM初始 pre = 900   psc=0		
						Fpwm = 72M / ((arr+1)*(psc+1)
	//duty circle = TIM3->CCR1 / arr(??:%)

		TIM_SetCompare2(TIM2, ti);//设置TIMx捕获比较2寄存器值	   //ti 0-65535  占空比		
	*/
	//注意公式： Fpwm = 72M / ((arr+1)*(psc+1))(??:Hz)

	GPIO_PinRemapConfig(GPIO_FullRemap_TIM3,ENABLE);//改变指定管脚的映射	//pC7

	//PWM初始化	  //根据TIM_OCInitStruct中指定的参数初始化外设TIMx
	
	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;   //PWM 1 2 是本身波形的不同
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;//PWM输出使能
	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_Low;     //震荡的极性是不同的

	TIM_OC2Init(TIM3,&TIM_OCInitStructure);
	//注意此处初始化时TIM_OC2Init而不是TIM_OCInit，否则会出错。因为固件库的版本不一样。
	//PS 通道1 2 3 4  就是通过这个OC2 OC1 来指定的

	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);//使能或者失能TIMx在CCR2上的预装载寄存器
					
	

}
/*******************************************************************************
* Function name  :  pwm_enable
* Description  : to enable pwm
* Input : None
* Output  :  None
* Return Value :  None
* Attention: we do not want pwm to open any time, so we want to use it when we need to 
*******************************************************************************/

void pwm_enable()
{
	TIM_Cmd(TIM3,ENABLE);//使能或者失能TIMx外设

}
