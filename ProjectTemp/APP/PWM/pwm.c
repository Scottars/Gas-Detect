#include "pwm.h"

/*******************************************************************************
* �� �� ��         : pwm_init
* ��������		   : IO�˿ڼ�TIM3��ʼ������	   
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
void pwm_init(u16 pre, u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;   //����һ���ṹ�������������ʼ��GPIO

	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;//����һ���ṹ�������������ʼ����ʱ��

	TIM_OCInitTypeDef TIM_OCInitStructure;//����TIM_OCInitStruct��ָ���Ĳ�����ʼ������TIMx

	/* ����ʱ�� */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);

	/*  ����GPIO��ģʽ��IO�� */
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_7;    //TIM3 PC7
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;//�����������
	GPIO_Init(GPIOC,&GPIO_InitStructure);

	//Fpwm = 72M / ((arr+1)*(psc+1))(??:Hz)
	//duty circle = TIM3->CCR1 / arr(??:%)

	//TIM2��ʱ����ʼ��
	TIM_TimeBaseInitStructure.TIM_Period = pre;	   //����Ƶ,PWM Ƶ��=72000/900=80Khz//�����Զ���װ�ؼĴ������ڵ�ֵ  �����Ĵ���������
	TIM_TimeBaseInitStructure.TIM_Prescaler = psc;//����������ΪTIMxʱ��Ƶ��Ԥ��Ƶֵ��100Khz����Ƶ��
	TIM_TimeBaseInitStructure.TIM_ClockDivision =TIM_CKD_DIV1;//����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;	//TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM3, & TIM_TimeBaseInitStructure);
	
	
	/*
	  �����ⲿ�ֵ�˵��
		pwm_init(900,0);	 //PWM��ʼ pre = 900   psc=0		
						Fpwm = 72M / ((arr+1)*(psc+1)
	//duty circle = TIM3->CCR1 / arr(??:%)

		TIM_SetCompare2(TIM2, ti);//����TIMx����Ƚ�2�Ĵ���ֵ	   //ti 0-65535  ռ�ձ�		
	*/
	//ע�⹫ʽ�� Fpwm = 72M / ((arr+1)*(psc+1))(??:Hz)

	GPIO_PinRemapConfig(GPIO_FullRemap_TIM3,ENABLE);//�ı�ָ���ܽŵ�ӳ��	//pC7s

	//PWM��ʼ��	  //����TIM_OCInitStruct��ָ���Ĳ�����ʼ������TIMx
	
	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;   //PWM 1 2 �Ǳ����εĲ�ͬ
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;//PWM���ʹ��
	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_Low;     //�𵴵ļ����ǲ�ͬ��

	TIM_OC2Init(TIM3,&TIM_OCInitStructure);
	//ע��˴���ʼ��ʱTIM_OC2Init������TIM_OCInit������������Ϊ�̼���İ汾��һ����
	//PS ͨ��1 2 3 4  ����ͨ�����OC2 OC1 ��ָ����

	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);//ʹ�ܻ���ʧ��TIMx��CCR2�ϵ�Ԥװ�ؼĴ���
					
	

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
	TIM_Cmd(TIM3,ENABLE);//ʹ�ܻ���ʧ��TIMx����

}
