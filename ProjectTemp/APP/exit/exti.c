#include "exti.h"
#include "printf.h"

/*******************************************************************************
* 函 数 名         : exti_init
* 函数功能         : 外部中断2端口初始化函数
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/

void exti_init()  //外部中断初始化
{
    GPIO_InitTypeDef GPIO_InitStructure;

    EXTI_InitTypeDef EXTI_InitStructure;

    NVIC_InitTypeDef NVIC_InitStructure;

    /* 开启GPIO时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);  //
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);

    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;  //input mode
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_Init(GPIOB,&GPIO_InitStructure);

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource1);//set io and extiline

    /* 设置外部中断的模式 */
    EXTI_InitStructure.EXTI_Line=EXTI_Line1;
    EXTI_InitStructure.EXTI_Mode=EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    /* 设置NVIC参数 */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;    //打开EXTI2的全局中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; //抢占优先级为0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;        //响应优先级为0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;           //使能
    NVIC_Init(&NVIC_InitStructure);


    printf("we initialize the exti\r\n");
}
void exti_disable()  //外部中断初始化
{
    GPIO_InitTypeDef GPIO_InitStructure;

    EXTI_InitTypeDef EXTI_InitStructure;

    NVIC_InitTypeDef NVIC_InitStructure;

    /* 开启GPIO时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);  //
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);

    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;  //input mode
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_Init(GPIOB,&GPIO_InitStructure);

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource1);//set io and extiline

    /* 设置外部中断的模式 */
    EXTI_InitStructure.EXTI_Line=EXTI_Line1;
    EXTI_InitStructure.EXTI_Mode=EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = DISABLE;
    EXTI_Init(&EXTI_InitStructure);

    /* 设置NVIC参数 */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;    //打开EXTI2的全局中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; //抢占优先级为0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;        //响应优先级为0
    NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;           //使能
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

    GPIO_InitTypeDef GPIO_InitStructure;    //声明一个结构体变量，用来初始化GPIO

    SystemInit();   //系统时钟初始化

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE); /* 开启GPIO时钟 */

    /*  配置GPIO的模式和IO口 */
    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_8;  //选择你要设置的IO口
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPD;   //floating in , the voltage is fully up to the outside circut
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;   //设置传输速率
    GPIO_Init(GPIOA,&GPIO_InitStructure); /* 初始化GPIO */

}
/*******************************************************************************
* Function name  :  Timing_Signal_Check
* Description  : Timing check if the signal is here or not 
* Input : None
* Output  :  None
* Return Value :  None
* Attention: we judge the signal is PA8 pin, which is five voltage torlerance

*******************************************************************************/

void Timing_Signal_Check()  //外部中断初始化
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

