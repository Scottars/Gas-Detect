#include "exti.h"
#include "printf.h"

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
void exti_disable()  //�ⲿ�жϳ�ʼ��
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
    EXTI_InitStructure.EXTI_LineCmd = DISABLE;
    EXTI_Init(&EXTI_InitStructure);

    /* ����NVIC���� */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;    //��EXTI2��ȫ���ж�
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; //��ռ���ȼ�Ϊ0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;        //��Ӧ���ȼ�Ϊ0
    NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;           //ʹ��
    NVIC_Init(&NVIC_InitStructure);


    printf("we disable the \r\n");
}

/*******************************************************************************
* Function name  : Timing_Signal_init
* Description  : Intial the gpio pin , in order to read the pin voltage to judge the signal is here or not 
* Input : None
* Output  :  None
* Return Value :  None
* Attention: In the Timing mode, we initialize the gpio to input down so we 
		judge the signal is here or not to decide if we should open the puff mode 

*******************************************************************************/

void Timing_Signal_init()
{

    GPIO_InitTypeDef GPIO_InitStructure;    //����һ���ṹ�������������ʼ��GPIO

    SystemInit();   //ϵͳʱ�ӳ�ʼ��

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE); /* ����GPIOʱ�� */

    /*  ����GPIO��ģʽ��IO�� */
    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_8;  //ѡ����Ҫ���õ�IO��
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPD;   //floating in , the voltage is fully up to the outside circut
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;   //���ô�������
    GPIO_Init(GPIOA,&GPIO_InitStructure); /* ��ʼ��GPIO */

}
/*******************************************************************************
* Function name  :  Timing_Signal_Check
* Description  : Timing check if the signal is here or not 
* Input : None
* Output  :  None
* Return Value :  None
* Attention: we judge the signal is PA8 pin, which is five voltage torlerance

*******************************************************************************/

void Timing_Signal_Check()  //�ⲿ�жϳ�ʼ��
{


    if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_8)==1)
    {
        Normal_Puff_RunningMode=0xff;
        printf("Timing signal is here \r\n");
    }
    else
    {

        
            Normal_Puff_RunningMode=0x00;

            
        
    }
}

