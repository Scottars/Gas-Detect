

#include "adc.h"
#include "systick.h"


/*******************************************************************************
* �� �� ��         : adc_init
* ��������         : IO�˿�ʱ�ӳ�ʼ������
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
volatile uint16_t ADC_ConvertedValue[10][3];

void dma_test()
{

	GPIO_InitTypeDef GPIO_InitStructure;
    ADC_InitTypeDef ADC_InitStructure;
		DMA_InitTypeDef DMA_InitStructure;
	
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);
	
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC|RCC_APB2Periph_AFIO|RCC_APB2Periph_ADC1,ENABLE);

    RCC_ADCCLKConfig(RCC_PCLK2_Div6);//12M  ???14M ????ADC????ADCCLK??

  
		DMA_DeInit(DMA1_Channel1);    //将通道一寄存器设为默认值
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(ADC1->DR);//该参数用以定义DMA外设基地址
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&ADC_ConvertedValue;//该参数用以定义DMA内存基地址(转换结果保存的地址)
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;//该参数规定了外设是作为数据传输的目的地还是来源，此处是作为来源
    DMA_InitStructure.DMA_BufferSize = 3*10;//定义指定DMA通道的DMA缓存的大小,单位为数据单位。这里也就是ADCConvertedValue的大小
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//设定外设地址寄存器递增与否,此处设为不变 Disable
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//用来设定内存地址寄存器递增与否,此处设为递增，Enable
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;//数据宽度为16位
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;//数据宽度为16位
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular; //工作在循环缓存模式
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;//DMA通道拥有高优先级 分别4个等级 低、中、高、非常高
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;//使能DMA通道的内存到内存传输
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);//根据DMA_InitStruct中指定的参数初始化DMA的通道

    DMA_Cmd(DMA1_Channel1, ENABLE);//启动DMA通道一

	
	
	  GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0 |GPIO_Pin_1 | GPIO_Pin_2;//ADC
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AIN; //???????
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_Init(GPIOC,&GPIO_InitStructure);

		
	
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 3;
    ADC_Init(ADC1, &ADC_InitStructure);

    //???????ADC??????????????????????????????????
   
		//RCC_ADCCLKConfig(RCC_PCLK2_Div8);//配置ADC时钟，为PCLK2的8分频，即9Hz  这个上面就已经进行定义了
		//ADC_RegularChannelConfig(ADC1,ADC_Channel_10,1,ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1,ADC_Channel_10,1,ADC_SampleTime_239Cycles5);//通道一转换结果保存到ADCConvertedValue[0~10][0]
	ADC_RegularChannelConfig(ADC1,ADC_Channel_11,2,ADC_SampleTime_239Cycles5);//通道二转换结果保存到ADCConvertedValue[0~10][1]
	ADC_RegularChannelConfig(ADC1,ADC_Channel_12,3,ADC_SampleTime_239Cycles5);//通道三转换结果保存到ADCConvertedValue[0~10][2]



		ADC_DMACmd(ADC1,ENABLE);
    ADC_Cmd(ADC1,ENABLE);

    ADC_ResetCalibration(ADC1);//?????????ADC??У??????
    while(ADC_GetResetCalibrationStatus(ADC1));//???ADC????У??????????

    ADC_StartCalibration(ADC1);//??????ADC??У???
    while(ADC_GetCalibrationStatus(ADC1));//??????ADC??У?????

	ADC_SoftwareStartConvCmd(ADC1, ENABLE);//??????????????ADC???????????????


}


float AD_Conversion_1479A_DMA()
{
	
	
	int sum;
	u8 i,j;
	float ADValue_1479A;//用来保存经过转换得到的电压值
	i=0;
	sum=0;
	
	for(j=0;j<10;j++)
	{
		sum +=ADC_ConvertedValue[j][i];
	//	printf("Inside：。%f\r\n",ADC_ConvertedValue[j][i]);
	}
	ADValue_1479A=(float)sum/(10*4096)*3.3;//求平均值并转换成电压值
	printf("The current ADValue_1479A value =%f\r\n",ADValue_1479A);
}


float AD_Conversion_627D_DMA()
{
	
	int sum;
	u8 i,j;
	float ADValue_627D;//用来保存经过转换得到的电压值
	i=0;
	sum=0;
	
	for(j=0;j<10;j++)
	{
		sum +=ADC_ConvertedValue[j][i];
	//	printf("Inside：。%f\r\n",ADC_ConvertedValue[j][i]);
	}
	ADValue_627D=(float)sum/(10*4096)*3.3;//求平均值并转换成电压值
	printf("The currentADValue_627D value =%f\r\n",ADValue_627D);
		
}


float AD_Conversion_025D_DMA()
{
	
	int sum;
	u8 i,j;
	float ADValue_025D;//用来保存经过转换得到的电压值
	i=0;
	sum=0;
	
	for(j=0;j<10;j++)
	{
		sum +=ADC_ConvertedValue[j][i];
	//	printf("Inside：。%f\r\n",ADC_ConvertedValue[j][i]);
	}
	ADValue_025D=(float)sum/(10*4096)*3.3;//求平均值并转换成电压值
	printf("The current ADValue_025D value =%f\r\n",ADValue_025D);
}







/*******************************************************************************
* �� �� ��         : adc_init
* ��������         : IO�˿�ʱ�ӳ�ʼ������
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
void adc10_init()
{
    GPIO_InitTypeDef GPIO_InitStructure;
    ADC_InitTypeDef ADC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC|RCC_APB2Periph_AFIO|RCC_APB2Periph_ADC1,ENABLE);

    RCC_ADCCLKConfig(RCC_PCLK2_Div6);//12M  ���14M ����ADCʱ�ӣ�ADCCLK��

    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0;//ADC
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AIN; //ģ������
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_Init(GPIOC,&GPIO_InitStructure);


    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &ADC_InitStructure);

    //����ָ��ADC�Ĺ�����ͨ�����������ǵ�ת��˳��Ͳ���ʱ��
    ADC_RegularChannelConfig(ADC1,ADC_Channel_10,1,ADC_SampleTime_239Cycles5);

    ADC_Cmd(ADC1,ENABLE);

    ADC_ResetCalibration(ADC1);//����ָ����ADC��У׼�Ĵ���
    while(ADC_GetResetCalibrationStatus(ADC1));//��ȡADC����У׼�Ĵ�����״̬

    ADC_StartCalibration(ADC1);//��ʼָ��ADC��У׼״̬
    while(ADC_GetCalibrationStatus(ADC1));//��ȡָ��ADC��У׼����

    ADC_SoftwareStartConvCmd(ADC1, ENABLE);//ʹ�ܻ���ʧ��ָ����ADC�����ת����������


}


void adc11_init()
{
    GPIO_InitTypeDef GPIO_InitStructure;
    ADC_InitTypeDef ADC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC|RCC_APB2Periph_AFIO|RCC_APB2Periph_ADC1,ENABLE);

    RCC_ADCCLKConfig(RCC_PCLK2_Div6);//12M  ���14M ����ADCʱ�ӣ�ADCCLK��

    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_1;//ADC
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AIN; //ģ������
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_Init(GPIOC,&GPIO_InitStructure);


    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &ADC_InitStructure);

    //����ָ��ADC�Ĺ�����ͨ�����������ǵ�ת��˳��Ͳ���ʱ��
    ADC_RegularChannelConfig(ADC1,ADC_Channel_11,1,ADC_SampleTime_239Cycles5);

    ADC_Cmd(ADC1,ENABLE);

    ADC_ResetCalibration(ADC1);//����ָ����ADC��У׼�Ĵ���
    while(ADC_GetResetCalibrationStatus(ADC1));//��ȡADC����У׼�Ĵ�����״̬

    ADC_StartCalibration(ADC1);//��ʼָ��ADC��У׼״̬
    while(ADC_GetCalibrationStatus(ADC1));//��ȡָ��ADC��У׼����

    ADC_SoftwareStartConvCmd(ADC1, ENABLE);//ʹ�ܻ���ʧ��ָ����ADC�����ת����������


}
void adc12_init()
{
    GPIO_InitTypeDef GPIO_InitStructure;
    ADC_InitTypeDef ADC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC|RCC_APB2Periph_AFIO|RCC_APB2Periph_ADC1,ENABLE);

    RCC_ADCCLKConfig(RCC_PCLK2_Div6);//12M  ���14M ����ADCʱ�ӣ�ADCCLK��

    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_2;//ADC
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AIN; //ģ������
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_Init(GPIOC,&GPIO_InitStructure);


    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &ADC_InitStructure);

    //����ָ��ADC�Ĺ�����ͨ�����������ǵ�ת��˳��Ͳ���ʱ��
    ADC_RegularChannelConfig(ADC1,ADC_Channel_12,1,ADC_SampleTime_239Cycles5);

    ADC_Cmd(ADC1,ENABLE);

    ADC_ResetCalibration(ADC1);//����ָ����ADC��У׼�Ĵ���
    while(ADC_GetResetCalibrationStatus(ADC1));//��ȡADC����У׼�Ĵ�����״̬

    ADC_StartCalibration(ADC1);//��ʼָ��ADC��У׼״̬
    while(ADC_GetCalibrationStatus(ADC1));//��ȡָ��ADC��У׼����

    ADC_SoftwareStartConvCmd(ADC1, ENABLE);//ʹ�ܻ���ʧ��ָ����ADC�����ת����������


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
    u8 t;  //times ����ʵ�ֶ�β�����ƽ��ֵ~

    u32 temp=0;

    ADC_RegularChannelConfig(ADC1,channel,1,ADC_SampleTime_239Cycles5);
    for (t=0; t<times; t++)
    {
        ADC_SoftwareStartConvCmd(ADC1, ENABLE);// �������
        while (!ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC));  //??????   ����Ҳ���ӳ٣�ʵ�������
        temp += ADC_GetConversionValue(ADC1);        //������ȡ���ǵ�ת��ֵ��
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
    u16 AD_Channel_Select[3]= {ADC_Channel_10,ADC_Channel_11,ADC_Channel_12}; //����ͨ��ѡ������
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
    //����voltage ����

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


float AD_Conversion_1479A(times)

{

    float voltage;
    ////////////////////AD part/////////////////////////

    u32 AD_Channel_10_Value,AD_Channel_11_Value,AD_Channel_12_Value;
    ADC_DeInit(ADC1);

    adc10_init();
    AD_Channel_12_Value = Get_ADC_Value(ADC_Channel_12,times);
    voltage = (float)AD_Channel_12_Value*(0.0008056640625);

  //  printf("AD conversion mid  1479A is %f\r\n",voltage);

    //����voltage ����

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


float AD_Conversion_627D(times)
{
    float voltage;
    ////////////////////AD part/////////////////////////

    u32 AD_Channel_10_Value,AD_Channel_11_Value,AD_Channel_12_Value;
    ADC_DeInit(ADC1);
    adc11_init();
    AD_Channel_11_Value = Get_ADC_Value(ADC_Channel_11,times);
    voltage = (float)AD_Channel_11_Value*(0.0008056640625);

    //printf("AD conversion mid  627Dis %f\r\n",voltage);

    //����voltage ����
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

float AD_Conversion_025D(times)
{
    float voltage;
    ////////////////////AD part/////////////////////////

    u32 AD_Channel_10_Value,AD_Channel_11_Value,AD_Channel_12_Value;

    ADC_DeInit(ADC1);
    adc12_init();

    AD_Channel_10_Value = Get_ADC_Value(ADC_Channel_10,times);

    voltage = (float)AD_Channel_10_Value*(0.0008056640625);
    
    //printf("AD conversion mid  025D is %f\r\n",voltage);

    //����voltage ����

    return voltage ;
}

