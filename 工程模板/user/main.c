/*******************************************************************************
*                 
*                 		       普中科技
--------------------------------------------------------------------------------
* 实 验 名		 : ADC转换实验
* 实验说明       : 通过printf打印AD检测数据 ，调节AD模块旁边的电位器 在串口助手上即可输出电压，
					见文件内截图
* 连接方式       : 
* 注    意		 : 	所用函数在头文件.c文件内
*******************************************************************************/


#include "public.h"
#include "printf.h"
#include "adc.h"
#include "systick.h"
#include "dac.h"    //实现ad 转换
#include "led.h"

#include "gui.h"
#include "test.h"
//#include "delay.h"
#include "lcd.h"
#include "IOState.h"

#include "w5500.h"

/****************************************************************************
* Function Name  : main
* Description    : Main program.
* Input          : None
* Output         : None
* Return         : None
****************************************************************************/
unsigned char ADdataA[7];
extern unsigned int W5500_Send_Delay_Counter;
//unsigned char ADdataB[7];
//unsigned char A1;
//unsigned char A2;



int main()
{	
		u8 i,j,k;		
	////////////////////AD part/////////////////////////
	u16 dacval;
	
	float voltage[3];
	u16 AD_Channel_Select[3]={ADC_Channel_10,ADC_Channel_11,ADC_Channel_12};//定义通道选择数组
	u32 AD_Channel_10_Value,AD_Channel_11_Value,AD_Channel_12_Value;
	
	///////////////////////Valve Gas PArt///////	
	u8 *ValveState_Value;
	u8 ValveSet_Value[12]={0,0,0,0,0,0,0,0,0,0,0,0};  //来自网络传入过来的信号
	

	///////////////////////液晶屏初始化过程///////////////////////
	
	SystemInit();//初始化RCC 设置系统主频为72MHZ
	delay_init(72);	     //延时初始化
	LCD_Init();	   //液晶屏初始化
	
	
	//////////////////////////////////////////////////////////////
		
	LED_Init(); //初始化显示灯的部分  仅仅在调试的时候使用到了
	
	
	
	////////////////////////////////////////////
	//ADC模块使用了三个片上AD PC0 PC1 PC2  分别是 AD10 11 12 通道
	adc_init();	 //ADC初始化
	
	/////////////////////////////
	//用来进行串口通讯的，printf 实现了重定向
	printf_init(); //printf初始化
	
	/////////////////////////////////////////
	//DAC 部分的初始化，我们采用的是DAC1  也就是PA4 口的输出
	Dac1_Init();				//DAC初始化
	DAC_SetChannel1Data(DAC_Align_12b_R, 0);//初始值为0	 


////////////////////
//电光转换部分的引脚的的初始化
	ValveState_Init(); //供气阀门GPIO口的部分初始化
	
	
	
	
	
	
	
	
	
	//////////////////网络初始化//////////////////////
/*********************************************
说明：  PC
						IP：192.168.1.199
						端口号：4001 未定，通过测试选用什么端口号都能够实现
						
						测试的时候的目标的IP和目标的端口号使用的就是w5500的IP与端口号

				w5500：
						IP：192.168.1.198
						端口号：5000


**********************************************/	

		System_Initialization();	//STM32系统初始化函数(初始化STM32时钟及外设)
		Load_Net_Parameters();		//装载网络参数	
		W5500_Hardware_Reset();		//硬件复位W5500
		W5500_Initialization();		//W5500初始货配置
	
	
	
	

//界面初始化//

	GridLayer();  //显示屏的网络框架层

	Gas_StateLayer();//显示屏 供气阀状态初始化部分
	
		
	while(1)
	{
		
	
	//	Gas_StateLayerUpdate(ValveStateValue);
		/*
		//////////////////////屏幕tests 部分/////////////////////////////
		main_test(); 		//测试主界面
		Test_Color();  		//简单刷屏填充测试
		Test_FillRec();		//GUI矩形绘图测试
		Test_Circle(); 		//GUI画圆测试
		Test_Triangle();    //GUI三角形绘图测试
		English_Font_test();//英文字体示例测试
		Chinese_Font_test();//中文字体示例测试
		Pic_test();			//图片显示示例测试
		Rotate_Test();   //旋转显示测试
		//如果不带触摸，或者不需要触摸功能，请注释掉下面触摸屏测试项
		//Touch_Test();		//触摸屏手写测试  
			
		//////////////////////////////////////////////////////
			
		*/
		
	
				
		//网络传输
		
			
		
		//读取IO口的数据，用来更新我们的状态裂变，或者可以采取中断的方式
		//简单地方式就是每次走过这个地方把所有的IO的数据全都使用一下，还有一种方案就是通过中断？   中断还是应当采取外部中断吗？  显然不是
		//
		
		
		/*************************供气阀门部分程序处理	**********************************************/
	
  		ValveStateChange(ValveSet_Value);//网络信号传入过来的开断信息，然后程序实现自动设置对应的IO的高低电平
	
		
  		ValveState_Value=Gas_State_Read(); //函数实现读取IO口的高低电平值
		
				
		/***********************************************************************************************/
		
		
		
		/****************************************网络处理通讯处理********************************************/
		
		W5500_Socket_Set();//W5500端口初始化配置

		if(W5500_Interrupt)//处理W5500中断		
		{LED0=0;
			W5500_Interrupt_Process();//W5500中断处理程序框架
		}
		if((S0_Data & S_RECEIVE) == S_RECEIVE)//如果Socket0接收到数据
		{
			S0_Data&=~S_RECEIVE;
			Process_Socket_Data(0);//W5500接收并发送接收到的数据
		}
		else if(W5500_Send_Delay_Counter >= 500)//定时发送字符串
		{
			if(S0_State == (S_INIT|S_CONN))
			{
				S0_Data&=~S_TRANSMITOK;
				memcpy(Tx_Buffer, "\r\n THis is the data i want to test\r\n", 23);	
				Write_SOCK_Data_Buffer(0, Tx_Buffer, 23);//指定Socket(0~7)发送数据处理,端口0发送23字节数据
			}
			W5500_Send_Delay_Counter=0;
		}
		
		
		/*******************************************************************************************************/
	
		
		
		
			j++;
			if(j%20==0)
		{		led_display(); //LED2 
			
			
		}


		
		for (i=0;i<10;i=i+1)
		{
			led_display();
	
			//i=1500;
			dacval=i*200;
			Dac1_Set_Vol(dacval);//设置DAC值	
		
			printf("The target dac boltage is : %f\n",dacval*3.3/3300);	
			
			
			
			
			
			
			
			AD_Channel_10_Value = Get_ADC_Value(ADC_Channel_10,20);
			AD_Channel_11_Value = Get_ADC_Value(ADC_Channel_11,20);
			AD_Channel_12_Value = Get_ADC_Value(ADC_Channel_12,20);
			voltage[0] = (float)AD_Channel_10_Value*(3.3/4096);
			voltage[1] = (float)AD_Channel_11_Value*(3.3/4096);
			voltage[2] = (float)AD_Channel_12_Value*(3.3/4096);
			printf("Voltage0 is %f\n",voltage[0]);
			printf("Voltage1 is %f\n",voltage[1]);
			printf("Voltage2 is %f\n",voltage[2]);
			ADC_LCD_Out(voltage);
		}
	
		
		
		
		
		
		
					
	}			
}


