#include "exti.h"

/*******************************************************************************
* �� �� ��         : exti_init
* ��������         : �ⲿ�ж�2�˿ڳ�ʼ������
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
void exti_init()  //�ⲿ�жϳ�ʼ��
{
    GPIO_InitTypeDef GPIO_InitStructure;

    EXTI_InitTypeDef EXTI_InitStructure;

    NVIC_InitTypeDef NVIC_InitStructure;

    /* ����GPIOʱ�� */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);  //
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);

    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;  //input mode
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_Init(GPIOB,&GPIO_InitStructure);

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource1);//set io and extiline

    /* �����ⲿ�жϵ�ģʽ */
    EXTI_InitStructure.EXTI_Line=EXTI_Line1;
    EXTI_InitStructure.EXTI_Mode=EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    /* ����NVIC���� */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;    //��EXTI2��ȫ���ж�
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; //��ռ���ȼ�Ϊ0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;        //��Ӧ���ȼ�Ϊ0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;           //ʹ��
    NVIC_Init(&NVIC_InitStructure);


	printf("we initialize the exti\r\n");
}
