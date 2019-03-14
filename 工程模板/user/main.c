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

float *voltage;
	
	char *ValveValue_Status;
	char *ValveValue_Status_LCD;
	

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
	
	
	
//界面初始化//

	GridLayer();  //显示屏的网络框架层

	Gas_StateLayer();//显示屏 供气阀状态初始化部分
	
	
	
	
	
	
	
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
		
		/****************************************网络处理通讯处理********************************************/
		
		W5500_Socket_Set();//W5500端口初始化配置

		if(W5500_Interrupt)//处理W5500中断		
		{
			//LED0=0;   												//LED0      指示的是网络部分的信号的传输
			W5500_Interrupt_Process();//W5500中断处理程序框架
		}
		if((S0_Data & S_RECEIVE) == S_RECEIVE)//如果Socket0接收到数据
		{
			S0_Data&=~S_RECEIVE;
			Process_Socket_Data(0);//W5500接收并发送接收到的数据
			//对于传下来的整个的接收的数据包，我们有两种方法进行考虑，考虑1：将寄存器中的数据分出来，然后在主函数中进行各种的调用情况
			//考虑二：直接在数据处理部分就直接调用我们的进行实际的操作部分。
		}
		
		
		/*****************供气检测部分**************************/
		
		//IO口的数据回读部分
		ValveValue_Status=Gas_State_Read(); //函数实现读取IO口的高低电平值
		ValveValue_Status_LCD = Gas_State_Read_LCD();
		//IO数据串口显示
		/*for(i=0;i<12;i++)
		{
			printf("StatusData:%s\n",ValveValue_Status_LCD[i]);
		}*/
		printf("%c",ValveValue_Status[0]);
		printf("%c",ValveValue_Status[1]);
		//IO数据 显示屏刷新
			
		Gas_StateLayerUpdate(ValveValue_Status_LCD);
		
		/*******************************************************************************************************/
	
		
		voltage=AD_Conversion();
		ADC_LCD_Out(voltage);

		
		
		
					
	}			
}


