

#include "adc.h"
#include "systick.h"

/*******************************************************************************
* 函 数 名         : adc_init
* 函数功能		   : IO端口时钟初始化函数	   
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void adc_init()
{
	GPIO_InitTypeDef GPIO_InitStructure; 
	ADC_InitTypeDef ADC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO|RCC_APB2Periph_ADC1,ENABLE);

	RCC_ADCCLKConfig(RCC_PCLK2_Div6);//12M  最大14M 设置ADC时钟（ADCCLK）这个使用的时钟的最大时14M 所以为了提高采样的精度
/////////通道10////////////////////////
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0;//ADC  PA1口  PA2口
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AIN;	//模拟输入
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOC,&GPIO_InitStructure);


	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;    //独立ADC 模式 和 双重模式 
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;      //可以单通  道转换 可以多通道转换
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;     //支持单次转换和连续转换  关闭连续扫描  就每次必须软件触发
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; // ADC的触发方式
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;      //对齐方式
	ADC_InitStructure.ADC_NbrOfChannel = 1;      //使用通道的数量，如果使用了多通道 前面的多通道的选择按钮也要设定位enable
	ADC_Init(ADC1, &ADC_InitStructure);
	
	ADC_Cmd(ADC1,ENABLE);	  //用来使能ADC
//进行ADC复位校准
	ADC_ResetCalibration(ADC1);//重置指定的ADC的校准寄存器
	while(ADC_GetResetCalibrationStatus(ADC1));//获取ADC重置校准寄存器的状态
	//ADC校准
	ADC_StartCalibration(ADC1);//开始指定ADC的校准状态
	while(ADC_GetCalibrationStatus(ADC1));//获取指定ADC的校准程序
	
//设置指定ADC的规则组通道，设置它们的转化顺序和采样时间
	ADC_RegularChannelConfig(ADC1,ADC_Channel_10,1,ADC_SampleTime_239Cycles5);
	                       //通道选择    通道数量    采样时间（ADC）有采样周期
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);//使能或者失能指定的ADC的软件转换启动功能   开始

//////////////////////AD1 通道11////////////////////////////////////////
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_1;//ADC  PA1口  PA2口
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AIN;	//模拟输入
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOC,&GPIO_InitStructure);


	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;    //独立ADC 模式 和 双重模式 
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;      //可以单通  道转换 可以多通道转换
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;     //支持单次转换和连续转换  关闭连续扫描  就每次必须软件触发
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; // ADC的触发方式
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;      //对齐方式
	ADC_InitStructure.ADC_NbrOfChannel = 1;      //使用通道的数量，如果使用了多通道 前面的多通道的选择按钮也要设定位enable
	ADC_Init(ADC1, &ADC_InitStructure);
	
	ADC_Cmd(ADC1,ENABLE);	  //用来使能ADC
//进行ADC复位校准
	ADC_ResetCalibration(ADC1);//重置指定的ADC的校准寄存器
	while(ADC_GetResetCalibrationStatus(ADC1));//获取ADC重置校准寄存器的状态
	//ADC校准
	ADC_StartCalibration(ADC1);//开始指定ADC的校准状态
	while(ADC_GetCalibrationStatus(ADC1));//获取指定ADC的校准程序
	
//设置指定ADC的规则组通道，设置它们的转化顺序和采样时间
	ADC_RegularChannelConfig(ADC1,ADC_Channel_11,1,ADC_SampleTime_239Cycles5);
	                       //通道选择    通道数量    采样时间（ADC）有采样周期
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);//使能或者失能指定的ADC的软件转换启动功能   开始




//////////////////////AD1 通道12////////////////////////////////////////
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_2;//ADC  PC2
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AIN;	//模拟输入
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOC,&GPIO_InitStructure);


	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;    //独立ADC 模式 和 双重模式 
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;      //可以单通  道转换 可以多通道转换
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;     //支持单次转换和连续转换  关闭连续扫描  就每次必须软件触发
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; // ADC的触发方式
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;      //对齐方式
	ADC_InitStructure.ADC_NbrOfChannel = 1;      //使用通道的数量，如果使用了多通道 前面的多通道的选择按钮也要设定位enable
	ADC_Init(ADC1, &ADC_InitStructure);
	
	ADC_Cmd(ADC1,ENABLE);	  //用来使能ADC
//进行ADC复位校准
	ADC_ResetCalibration(ADC1);//重置指定的ADC的校准寄存器
	while(ADC_GetResetCalibrationStatus(ADC1));//获取ADC重置校准寄存器的状态
	//ADC校准
	ADC_StartCalibration(ADC1);//开始指定ADC的校准状态
	while(ADC_GetCalibrationStatus(ADC1));//获取指定ADC的校准程序
	
//设置指定ADC的规则组通道，设置它们的转化顺序和采样时间
	ADC_RegularChannelConfig(ADC1,ADC_Channel_12,1,ADC_SampleTime_239Cycles5);
	                       //通道选择    通道数量    采样时间（ADC）有采样周期
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);//使能或者失能指定的ADC的软件转换启动功能   开始






}


u16 Get_ADC_Value(u8 channel,u8 times)
{
	u8 t;  //times 用来实现多次采样的平均值~ 
	
	u32 temp=0;
	
	ADC_RegularChannelConfig(ADC1,channel,1,ADC_SampleTime_239Cycles5);
	for (t=0;t<times;t++)
		{
			ADC_SoftwareStartConvCmd(ADC1, ENABLE);// 软件触发
			while (!ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC));  //??????   本身也有延迟，实际上如果
			temp += ADC_GetConversionValue(ADC1);		 //用来读取我们的转换值。
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
	u16 AD_Channel_Select[3]={ADC_Channel_10,ADC_Channel_11,ADC_Channel_12};//定义通道选择数组
	u32 AD_Channel_10_Value,AD_Channel_11_Value,AD_Channel_12_Value;

			AD_Channel_10_Value = Get_ADC_Value(ADC_Channel_10,1); 
			AD_Channel_11_Value = Get_ADC_Value(ADC_Channel_11,1);
			AD_Channel_12_Value = Get_ADC_Value(ADC_Channel_12,1);
			voltage[0] = (float)AD_Channel_10_Value*(3.3/4096);
			voltage[1] = (float)AD_Channel_11_Value*(3.3/4096);
			voltage[2] = (float)AD_Channel_12_Value*(3.3/4096);
		//	printf("Voltage0 is %f\r\n",voltage[0]);
//printf("Voltage1 is %f\r\n",voltage[1]);
			
		//	printf("Voltage2 is %f\r\n",voltage[2]);
			//返回voltage 数组	

		return voltage ;	
}

float AD_Conversion_1479A()
{
		
			float voltage;
	////////////////////AD part/////////////////////////
	
	u32 AD_Channel_10_Value,AD_Channel_11_Value,AD_Channel_12_Value;

			AD_Channel_10_Value = Get_ADC_Value(ADC_Channel_10,1); 
	
			voltage = (float)AD_Channel_10_Value*(0.0008056640625);
	
			//printf("AD conversion mid  1479A is %f\r\n",voltage);
		
			//返回voltage 数组	

		return voltage ;	
}

float AD_Conversion_627D()
{
		
			float voltage;
	////////////////////AD part/////////////////////////
	
	u32 AD_Channel_10_Value,AD_Channel_11_Value,AD_Channel_12_Value;

			AD_Channel_11_Value = Get_ADC_Value(ADC_Channel_11,1); 
	
			voltage = (float)AD_Channel_11_Value*(0.0008056640625);
	
			//printf("AD conversion mid  627Dis %f\r\n",voltage);
		
			//返回voltage 数组	

		return voltage ;	
}
float AD_Conversion_025D()
{
		
			float voltage;
	////////////////////AD part/////////////////////////
	
	u32 AD_Channel_10_Value,AD_Channel_11_Value,AD_Channel_12_Value;

			AD_Channel_12_Value = Get_ADC_Value(ADC_Channel_12,1); 
	
			voltage = (float)AD_Channel_12_Value*(0.0008056640625);
	
			//printf("AD conversion mid  025D is %f\r\n",voltage);
		
			//返回voltage 数组	

		return voltage ;	
}