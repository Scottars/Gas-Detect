

#include "adc.h"
#include "systick.h"

/*******************************************************************************
* 函 数 名         : adc_init
* 函数功能         : IO端口时钟初始化函数
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void adc10_init()
{
    GPIO_InitTypeDef GPIO_InitStructure;
    ADC_InitTypeDef ADC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC|RCC_APB2Periph_AFIO|RCC_APB2Periph_ADC1,ENABLE);

    RCC_ADCCLKConfig(RCC_PCLK2_Div6);//12M  最大14M 设置ADC时钟（ADCCLK）

    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0;//ADC
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AIN; //模拟输入
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_Init(GPIOC,&GPIO_InitStructure);


    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &ADC_InitStructure);

    //设置指定ADC的规则组通道，设置它们的转化顺序和采样时间
    ADC_RegularChannelConfig(ADC1,ADC_Channel_10,1,ADC_SampleTime_239Cycles5);

    ADC_Cmd(ADC1,ENABLE);

    ADC_ResetCalibration(ADC1);//重置指定的ADC的校准寄存器
    while(ADC_GetResetCalibrationStatus(ADC1));//获取ADC重置校准寄存器的状态

    ADC_StartCalibration(ADC1);//开始指定ADC的校准状态
    while(ADC_GetCalibrationStatus(ADC1));//获取指定ADC的校准程序

    ADC_SoftwareStartConvCmd(ADC1, ENABLE);//使能或者失能指定的ADC的软件转换启动功能


}


void adc11_init()
{
    GPIO_InitTypeDef GPIO_InitStructure;
    ADC_InitTypeDef ADC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC|RCC_APB2Periph_AFIO|RCC_APB2Periph_ADC1,ENABLE);

    RCC_ADCCLKConfig(RCC_PCLK2_Div6);//12M  最大14M 设置ADC时钟（ADCCLK）

    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_1;//ADC
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AIN; //模拟输入
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_Init(GPIOC,&GPIO_InitStructure);


    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &ADC_InitStructure);

    //设置指定ADC的规则组通道，设置它们的转化顺序和采样时间
    ADC_RegularChannelConfig(ADC1,ADC_Channel_11,1,ADC_SampleTime_239Cycles5);

    ADC_Cmd(ADC1,ENABLE);

    ADC_ResetCalibration(ADC1);//重置指定的ADC的校准寄存器
    while(ADC_GetResetCalibrationStatus(ADC1));//获取ADC重置校准寄存器的状态

    ADC_StartCalibration(ADC1);//开始指定ADC的校准状态
    while(ADC_GetCalibrationStatus(ADC1));//获取指定ADC的校准程序

    ADC_SoftwareStartConvCmd(ADC1, ENABLE);//使能或者失能指定的ADC的软件转换启动功能


}
void adc12_init()
{
    GPIO_InitTypeDef GPIO_InitStructure;
    ADC_InitTypeDef ADC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC|RCC_APB2Periph_AFIO|RCC_APB2Periph_ADC1,ENABLE);

    RCC_ADCCLKConfig(RCC_PCLK2_Div6);//12M  最大14M 设置ADC时钟（ADCCLK）

    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_2;//ADC
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AIN; //模拟输入
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_Init(GPIOC,&GPIO_InitStructure);


    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &ADC_InitStructure);

    //设置指定ADC的规则组通道，设置它们的转化顺序和采样时间
    ADC_RegularChannelConfig(ADC1,ADC_Channel_12,1,ADC_SampleTime_239Cycles5);

    ADC_Cmd(ADC1,ENABLE);

    ADC_ResetCalibration(ADC1);//重置指定的ADC的校准寄存器
    while(ADC_GetResetCalibrationStatus(ADC1));//获取ADC重置校准寄存器的状态

    ADC_StartCalibration(ADC1);//开始指定ADC的校准状态
    while(ADC_GetCalibrationStatus(ADC1));//获取指定ADC的校准程序

    ADC_SoftwareStartConvCmd(ADC1, ENABLE);//使能或者失能指定的ADC的软件转换启动功能


}





/*******************************************************************************
* Function name  : Get_ADC_Value
* Description  : Get the result of ad conversion which concludes three channels to be choosed
* Input : the channel you want get, and the sample times you want  ,get the mean of these times
* Output  :  the ad result 
* Return Value :  ad result(which is related to the reference)
* Attention: we can only choose channel10 11 12 which mean pc0 pc1 pc2

*******************************************************************************/
u16 Get_ADC_Value(u8 channel,u8 times)
{
    u8 t;  //times 用来实现多次采样的平均值~

    u32 temp=0;

    ADC_RegularChannelConfig(ADC1,channel,1,ADC_SampleTime_239Cycles5);
    for (t=0; t<times; t++)
    {
        ADC_SoftwareStartConvCmd(ADC1, ENABLE);// 软件触发
        while (!ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC));  //??????   本身也有延迟，实际上如果
        temp += ADC_GetConversionValue(ADC1);        //用来读取我们的转换值。
        delay_ms(5);//this delay that we can delete

    }

    return temp/(t);



}
float *AD_Conversion()
{
    u8 i;
    ////////////////////AD part/////////////////////////
    u16 dacval;
    float voltage[3];
    u16 AD_Channel_Select[3]= {ADC_Channel_10,ADC_Channel_11,ADC_Channel_12}; //定义通道选择数组
    u32 AD_Channel_10_Value,AD_Channel_11_Value,AD_Channel_12_Value;


    AD_Channel_10_Value = Get_ADC_Value(ADC_Channel_10,1);
    AD_Channel_11_Value = Get_ADC_Value(ADC_Channel_11,1);
    AD_Channel_12_Value = Get_ADC_Value(ADC_Channel_12,1);
    voltage[0] = (float)AD_Channel_10_Value*(3.3/4096);
    voltage[1] = (float)AD_Channel_11_Value*(3.3/4096);
    voltage[2] = (float)AD_Channel_12_Value*(3.3/4096);
    //  printf("Voltage0 is %f\r\n",voltage[0]);
//printf("Voltage1 is %f\r\n",voltage[1]);

    //  printf("Voltage2 is %f\r\n",voltage[2]);
    //返回voltage 数组

    return voltage ;
}


/*******************************************************************************
* Function name  : AD_Conversion_1479A
* Description  : Get the result of AD_Conversion_1479A
* Input : none
* Output  : the float value  of 1479A channel--channel 10--pc0
* Return Value :  ad voltage (which is related to the reference)
* Attention: the reference voltage value is 3.3V

*******************************************************************************/


float AD_Conversion_1479A()
{

    float voltage;
    ////////////////////AD part/////////////////////////

    u32 AD_Channel_10_Value,AD_Channel_11_Value,AD_Channel_12_Value;
    ADC_DeInit(ADC1);

    adc10_init();
    AD_Channel_10_Value = Get_ADC_Value(ADC_Channel_10,10);
    voltage = (float)AD_Channel_10_Value*(0.0008056640625);

  //  printf("AD conversion mid  1479A is %f\r\n",voltage);

    //返回voltage 数组

    return voltage ;
}
/*******************************************************************************
* Function name  : AD_Conversion_627D
* Description  : Get the result of AD_Conversion_627D
* Input : none
* Output  : the float value  of 627D channel--channel 11--pc1
* Return Value :  ad voltage (which is related to the reference)
* Attention: the reference voltage value is 3.3V

*******************************************************************************/


float AD_Conversion_627D()
{
    float voltage;
    ////////////////////AD part/////////////////////////

    u32 AD_Channel_10_Value,AD_Channel_11_Value,AD_Channel_12_Value;
    ADC_DeInit(ADC1);
    adc11_init();
    AD_Channel_11_Value = Get_ADC_Value(ADC_Channel_11,10);
    voltage = (float)AD_Channel_11_Value*(0.0008056640625);

    //printf("AD conversion mid  627Dis %f\r\n",voltage);

    //返回voltage 数组
    return voltage ;
}

/*******************************************************************************
* Function name  : AD_Conversion_025D
* Description  : Get the result of AD_Conversion_025D
* Input : none
* Output  : the float value  of 025D channel--channel 12--pc2
* Return Value :  ad voltage (which is related to the reference)
* Attention: the reference voltage value is 3.3V

*******************************************************************************/

float AD_Conversion_025D()
{
    float voltage;
    ////////////////////AD part/////////////////////////

    u32 AD_Channel_10_Value,AD_Channel_11_Value,AD_Channel_12_Value;

    ADC_DeInit(ADC1);
    adc12_init();

    AD_Channel_12_Value = Get_ADC_Value(ADC_Channel_12,10);

    voltage = (float)AD_Channel_12_Value*(0.0008056640625);
    
    //printf("AD conversion mid  025D is %f\r\n",voltage);

    //返回voltage 数组

    return voltage ;
}
