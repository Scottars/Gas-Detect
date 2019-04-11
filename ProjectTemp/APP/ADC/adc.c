

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
/////////ͨ��10////////////////////////
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0;//ADC  PA1��  PA2��
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

//////////////////////AD1 ͨ��11////////////////////////////////////////
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_1;//ADC  PA1��  PA2��
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




//////////////////////AD1 ͨ��12////////////////////////////////////////
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_2;//ADC  PA1��  PA2��
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
	u8 t;  //times ����ʵ�ֶ�β�����ƽ��ֵ~ 
	
	u32 temp=0;
	
	ADC_RegularChannelConfig(ADC1,channel,1,ADC_SampleTime_239Cycles5);
	for (t=0;t<times;t++)
		{
			ADC_SoftwareStartConvCmd(ADC1, ENABLE);// �������
			while (!ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC));  //??????   ����Ҳ���ӳ٣�ʵ�������
			temp += ADC_GetConversionValue(ADC1);		 //������ȡ���ǵ�ת��ֵ��
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
	u16 AD_Channel_Select[3]={ADC_Channel_10,ADC_Channel_11,ADC_Channel_12};//����ͨ��ѡ������
	u32 AD_Channel_10_Value,AD_Channel_11_Value,AD_Channel_12_Value;

		//for (i=0;i<10;i=i+1)
		//{
		
		//	printf("The target dac boltage is : %f\n",dacval*3.3/3300);	
		
			AD_Channel_10_Value = Get_ADC_Value(ADC_Channel_10,1); 
			AD_Channel_11_Value = Get_ADC_Value(ADC_Channel_11,1);
			AD_Channel_12_Value = Get_ADC_Value(ADC_Channel_12,1);
			voltage[0] = (float)AD_Channel_10_Value*(3.3/4096);
			voltage[1] = (float)AD_Channel_11_Value*(3.3/4096);
			voltage[2] = (float)AD_Channel_12_Value*(3.3/4096);
			//printf("Voltage0 is %f\r\n",voltage[0]);
			//printf("Voltage1 is %f\r\n",voltage[1]);
			//printf("Voltage2 is %f\r\n",voltage[2]);
			//����voltage ����
		
	//	}
		return voltage ;
	
}