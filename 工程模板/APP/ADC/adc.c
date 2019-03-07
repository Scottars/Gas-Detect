

#include "adc.h"
#include "systick.h"

/*******************************************************************************
* �� �� ��         : adc_init
* ��������		   : IO�˿�ʱ�ӳ�ʼ������	   
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
void adc_init()
{
	GPIO_InitTypeDef GPIO_InitStructure; 
	ADC_InitTypeDef ADC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO|RCC_APB2Periph_ADC1,ENABLE);

	RCC_ADCCLKConfig(RCC_PCLK2_Div6);//12M  ���14M ����ADCʱ�ӣ�ADCCLK�����ʹ�õ�ʱ�ӵ����ʱ14M ����Ϊ����߲����ľ���

	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0;//ADC  PA1��
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AIN;	//ģ������
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOC,&GPIO_InitStructure);


	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;    //����ADC ģʽ �� ˫��ģʽ 
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;      //���Ե�ͨ  ��ת�� ���Զ�ͨ��ת��
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;     //֧�ֵ���ת��������ת��  �ر�����ɨ��  ��ÿ�α����������
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; // ADC�Ĵ�����ʽ
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;      //���뷽ʽ
	ADC_InitStructure.ADC_NbrOfChannel = 1;      //ʹ��ͨ�������������ʹ���˶�ͨ�� ǰ��Ķ�ͨ����ѡ��ťҲҪ�趨λenable
	ADC_Init(ADC1, &ADC_InitStructure);
	
	
	ADC_Cmd(ADC1,ENABLE);	  //����ʹ��ADC

//����ADC��λУ׼
	ADC_ResetCalibration(ADC1);//����ָ����ADC��У׼�Ĵ���
	while(ADC_GetResetCalibrationStatus(ADC1));//��ȡADC����У׼�Ĵ�����״̬
	//ADCУ׼
	ADC_StartCalibration(ADC1);//��ʼָ��ADC��У׼״̬
	while(ADC_GetCalibrationStatus(ADC1));//��ȡָ��ADC��У׼����
	
//����ָ��ADC�Ĺ�����ͨ�����������ǵ�ת��˳��Ͳ���ʱ��
	ADC_RegularChannelConfig(ADC1,ADC_Channel_10,1,ADC_SampleTime_239Cycles5);
	                       //ͨ��ѡ��    ͨ������    ����ʱ�䣨ADC���в�������

	ADC_SoftwareStartConvCmd(ADC1, ENABLE);//ʹ�ܻ���ʧ��ָ����ADC�����ת����������   ��ʼ

}


u16 Get_ADC_Value(u8 channel,u8 times)
{
	u8 t;
	u32 temp=0;
	
//?���ù����飬������˳������,ADC1   channel0  ���õ��ǹ�����
	ADC_RegularChannelConfig(ADC1,channel,1,ADC_SampleTime_239Cycles5);
	for (t=0;t<times;t++)
	{
		ADC_SoftwareStartConvCmd(ADC1, ENABLE);// �������
		while (!ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC));  //??????   ����Ҳ���ӳ٣�ʵ�������
		temp += ADC_GetConversionValue(ADC1);		 //������ȡ���ǵ�ת��ֵ��
		delay_ms(5);

	}
	return temp/(t+1) ;
}

