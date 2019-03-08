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

/****************************************************************************
* Function Name  : main
* Description    : Main program.
* Input          : None
* Output         : None
* Return         : None
****************************************************************************/
unsigned char ADdataA[7];
//unsigned char ADdataB[7];
//unsigned char A1;
//unsigned char A2;


int main()
{	
	
	
	u16 dacval;
	u16 value;
	float voltage[3];
	u16 AD_Channel_Select[3]={ADC_Channel_10,ADC_Channel_11,ADC_Channel_12};//定义通道选择数组
	u32 AD_Channel_10_Value,AD_Channel_11_Value,AD_Channel_12_Value;
	
	u8 i,j;		
	///////////////////////液晶屏初始化过程///////////////////////
	
	SystemInit();//初始化RCC 设置系统主频为72MHZ
	delay_init(72);	     //延时初始化
	LCD_Init();	   //液晶屏初始化
	
	
	//////////////////////////////////////////////////////////////
		
	LED_Init();
	adc_init();	 //ADC初始化
	printf_init(); //printf初始化
	
	Dac1_Init();				//DAC初始化
	
	DAC_SetChannel1Data(DAC_Align_12b_R, 0);//初始值为0	    
	
	
	GridLayer();

	Gas_StateLayer();
		
	while(1)
	{
		
		//Gas_StateLayerUpdate();
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
		
		//界面初始化//

		
		//读取IO口的数据，用来更新我们的状态裂变，或者可以采取中断的方式
		//简单地方式就是每次走过这个地方把所有的IO的数据全都使用一下，还有一种方案就是通过中断？   中断还是应当采取外部中断吗？  显然不是
		//
		//Gas_State_Read();
			
		
		
		//网络传输
		
		
		
		
			j++;
			if(j%20==0)
		{		led_display();
			
			
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
			ADC_Out(voltage);
		}
	
		
		
		
		
		
		
					
	}			
}


