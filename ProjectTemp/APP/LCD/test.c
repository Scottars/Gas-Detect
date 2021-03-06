//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//测试硬件：单片机STM32F103RCT6,正点原子MiniSTM32开发板,主频72MHZ，晶振12MHZ
//QDtech-TFT液晶驱动 for STM32 IO模拟
//xiao冯@ShenZhen QDtech co.,LTD
//公司网站:www.qdtft.com
//淘宝网站：http://qdtech.taobao.com
//wiki技术网站：http://www.lcdwiki.com
//我司提供技术支持，任何技术问题欢迎随时交流学习
//固话(传真) :+86 0755-23594567 
//手机:15989313508（冯工） 
//邮箱:lcdwiki01@gmail.com    support@lcdwiki.com    goodtft@163.com 
//技术支持QQ:3002773612  3002778157
//技术交流QQ群:324828016
//创建日期:2018/08/09
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 深圳市全动电子技术有限公司 2018-2028
//All rights reserved
/****************************************************************************************************
//=========================================电源接线================================================//
//     LCD模块                STM32单片机
//      VCC          接        DC5V/3.3V      //电源
//      GND          接          GND          //电源地
//=======================================液晶屏数据线接线==========================================//
//本模块默认数据总线类型为SPI总线
//     LCD模块                STM32单片机    
//    SDI(MOSI)      接          PB15         //液晶屏SPI总线数据写信号
//    SDO(MISO)      接          PB14         //液晶屏SPI总线数据读信号，如果不需要读，可以不接线
//=======================================液晶屏控制线接线==========================================//
//     LCD模块 					      STM32单片机 
//       LED         接          PB9          //液晶屏背光控制信号，如果不需要控制，接5V或3.3V
//       SCK         接          PB13         //液晶屏SPI总线时钟信号   
//      DC/RS        接          PB10         //液晶屏数据/命令控制信号
//       RST         接          PB12         //液晶屏复位控制信号
//       CS          接          PB11         //液晶屏片选控制信号
//=========================================触摸屏触接线=========================================//
//如果模块不带触摸功能或者带有触摸功能，但是不需要触摸功能，则不需要进行触摸屏接线
//	   LCD模块                STM32单片机 
//      T_IRQ        接          PC10         //触摸屏触摸中断信号
//      T_DO         接          PC2          //触摸屏SPI总线读信号
//      T_DIN        接          PC3          //触摸屏SPI总线写信号
//      T_CS         接          PC13         //触摸屏片选控制信号
//      T_CLK        接          PC0          //触摸屏SPI总线时钟信号
**************************************************************************************************/	
 /* @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, QD electronic SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
**************************************************************************************************/	
#include "lcd.h"
#include "systick.h"
#include "gui.h"
#include "test.h"
#include "touch.h"
#include "key.h" 
#include "led.h"
#include "pic.h"
#include "printf.h"
/*
#include "stdio.h"
#include "string.h"*/

//========================variable==========================//
u16 ColorTab[5]={RED,GREEN,BLUE,YELLOW,BRED};//定义颜色数组
//=====================end of variable======================//

/*****************************************************************************
 * @name       :void DrawTestPage(u8 *str)
 * @date       :2018-08-09 
 * @function   :Drawing test interface
 * @parameters :str:the start address of the Chinese and English strings
 * @retvalue   :None
******************************************************************************/ 


void DrawTestPage(u8 *str)
{

//绘制固定栏up
LCD_Clear(WHITE);
LCD_Fill(0,0,lcddev.width,20,BLUE);
//绘制固定栏down
LCD_Fill(0,lcddev.height-20,lcddev.width,lcddev.height,BLUE);
POINT_COLOR=WHITE;
Gui_StrCenter(0,2,WHITE,BLUE,str,16,1);//居中显示
Gui_StrCenter(0,lcddev.height-18,WHITE,BLUE,"http://www.lcdwiki.com",16,1);//居中显示
//绘制测试区域
//LCD_Fill(0,20,lcddev.width,lcddev.height-20,WHITE);
}

/*******************************************************************************
* Function name  :  ADC_LCD_Out
* Description  : update the adc part
* Input : the adc part voltage(Pressure,we need to display) , the target value we need to set 
* Output  :  None
* Return Value :  None
* Attention: the input is tuple

*******************************************************************************/

void ADC_LCD_Out(float voltage[3],float Set_Voltage[3])
{
	
//u8 TEXT_Buffer[19]={"FLASH SPI TEST OK!"};
//	DrawTestPage("全动电子综合测试程序");	
//	Gui_StrCenter(0,30,RED,BLUE,"全动电子",16,1);//居中显示
//	Gui_StrCenter(0,60,RED,BLUE,"综合测试程序",16,1);//居中显示	
	char str[40];
	u8 i;
	char description[19] = {"V"};

	for(i=0;i<3;i++)
	{

	sprintf(str,"%f%.20s",voltage[i],description);
	

	//sprintf(str,"%.3s%.3s",s1,s2); 将多个字符串连接起来
	//Gui_StrCenter(0,90,WHITE,RED,str,16,1);//居中显示
	Show_Str(130,56+i*20,BLACK,WHITE,str,12,0);
	//delay_ms(10);		
	//delay_ms(10);
	}

	for(i=0;i<3;i++)
	{
	sprintf(str,"%f%.20s",Set_Voltage[i],description);
		//sprintf(str,"%.3s%.3s",s1,s2); 将多个字符串连接起来
	//Gui_StrCenter(0,90,WHITE,RED,str,16,1);//居中显示
	Show_Str(60,56+i*20,BLACK,WHITE,str,12,0);
	//delay_ms(10);		
	//delay_ms(10);
	}
}
/*******************************************************************************
* Function name  :  GridLayer
* Description  : draw the basic page we put on the screen
* Input :None 
* Output  :  None
* Return Value :  None
* Attention: we do this only to make refresh the page

*******************************************************************************/

void GridLayer()
{	
	int i;
	//LCD_Clear(WHITE);   we have clear to white in the intial function
	for (i=1;i<16;i++)
	{
		if(i<=2)
		{
			LCD_DrawLine(0,i*26, 280, i*26);
		}	else if(i<=6 && i>2)
		{
			LCD_DrawLine(0,i*20+12, 250, i*20+12);
		}
		else if(i==7)
		{
			LCD_DrawLine(0,i*20+12+6, 250, i*20+12+6);
		}
			else 
		{
			LCD_DrawLine(0,i*20+18, 250, i*20+18);
		}
		//printf("hello");
	}
	LCD_DrawLine(120,26, 120, 132);  //画屏幕上的竖线  竖线位置 120 从左到右
	LCD_DrawLine(120,158, 120, 298); //画屏幕上的竖线
	
	
	//Main menu set
	//Gas Supply And Dectection System
	Show_Str(10,5,BLACK,WHITE,"Gas Suppply And Dectection System",12,0);//显示   mdde 0  表示 no overlay
	//Set Value   Measure
	Show_Str(60,33,BLACK,WHITE,"Set Value",12,1);
	Show_Str(140,33,BLACK,WHITE,"Measure Value",12,1);
	//Device  Show
	Show_Str(0,56,BLACK,YELLOW," 1479A :",12,0);
	Show_Str(0,78,BLACK,YELLOW," 627D  :",12,0);
	Show_Str(0,98,BLACK,YELLOW,"CDG025D:",12,0);
	Show_Str(0,118,BLACK,YELLOW,"   PV  :",12,0);

///////////////GAS Value State	/////////////////////////
	Show_Str(80,139,BLACK,WHITE,"Gas Value State",12,0);
	
	Show_Str(10,162,BLACK,YELLOW,"No.1",12,0);
	Show_Str(10,182,BLACK,YELLOW,"No.2",12,0);
	Show_Str(10,202,BLACK,YELLOW,"No.3",12,0);
	Show_Str(10,222,BLACK,YELLOW,"No.4",12,0);
	Show_Str(10,242,BLACK,YELLOW,"No.5",12,0);
	Show_Str(10,262,BLACK,YELLOW,"No.6",12,0);
	Show_Str(10,282,BLACK,YELLOW,"No.7",12,0);
	Show_Str(10,302,BLACK,YELLOW,"No.8",12,0);
	
	Show_Str(130,162,BLACK,YELLOW,"No.9",12,0);
	Show_Str(130,182,BLACK,YELLOW,"No.10",12,0);
	Show_Str(130,202,BLACK,YELLOW,"No.11",12,0);
	Show_Str(130,222,BLACK,YELLOW,"No.12",12,0);
	Show_Str(130,242,BLACK,YELLOW,"No.13",12,0);
	Show_Str(130,262,BLACK,YELLOW,"No.14",12,0);
	Show_Str(130,282,BLACK,YELLOW,"No.15",12,0);
	Show_Str(130,302,BLACK,YELLOW,"No.16",12,0);
}

/*******************************************************************************
* Function name  :  Gas_StateLayer
* Description  : draw the state of valve state, it only shows the default value 
* Input :None 
* Output  :  None
* Return Value :  None
* Attention: 

*******************************************************************************/

void Gas_StateLayer(void )
{
	
	u8 i=0;
	//LCD_Fill(0,20,lcddev.width,lcddev.height-20,WHITE);
	for(i=0;i<8;i++)
	{
		//if()
			gui_circle(180,168+i*20,ColorTab[0],6,1);
		gui_circle(50,168+i*20,ColorTab[0],6,1);
		
	}
	
	//delay_ms(1500);  //In order to make it faster
}
/*******************************************************************************
* Function name  :  Gas_StateLayerUpdate
* Description  : if we want to update the status, we need to call this funtion to make this change
* Input :None 
* Output  :  None
* Return Value :  None
* Attention:  the input is 12 tuple,so that we can judge it one by one 

*******************************************************************************/

void Gas_StateLayerUpdate(u8 Value_State[16] )
{
	//传入一个数组，用来指示更新各个灯的状态
	u8 i=0;
	//LCD_Fill(0,20,lcddev.width,lcddev.height-20,WHITE);
	for(i=0;i<8;i++)
	{
		if(Value_State[i]==1)
		{
			gui_circle(50,168+i*20,ColorTab[1],6,1);
			Show_Str(65,162+i*20,BLACK,YELLOW,"ON ",12,0);
		
		}
		else
		{
		gui_circle(50,168+i*20,ColorTab[0],6,1);
			Show_Str(65,162+i*20,BLACK,YELLOW,"OFF",12,0);
		}
			
	}
		for(i=8;i<16;i++)
	{
		if(Value_State[i]==1)
		{
			gui_circle(180,168+(i-8)*20,ColorTab[1],6,1);
			Show_Str(195,162+(i-8)*20,BLACK,YELLOW,"ON ",12,0);
		}
		else
		{
			gui_circle(180,168+(i-8)*20,ColorTab[0],6,1);
			Show_Str(195,162+(i-8)*20,BLACK,YELLOW,"OFF",12,0);
		}

		
	}
		
	delay_ms(1500);
}




/*****************************************************************************
 * @name       :void main_test(void)
 * @date       :2018-08-09 
 * @function   :Drawing the main Interface of the Comprehensive Test Program
 * @parameters :None
 * @retvalue   :None
******************************************************************************/
void main_test(void)
{
	DrawTestPage("全动电子综合测试程序__wangsai");	
	Gui_StrCenter(0,30,RED,BLUE,"全动电zi",16,1);//居中显示
	Gui_StrCenter(0,60,RED,BLUE,"综合测试程序",16,1);//居中显示	
	Gui_StrCenter(0,90,GREEN,BLUE,"3.2\" ILI9341 240X320",16,1);//居中显示
	Gui_StrCenter(0,120,BLUE,BLUE,"Pzx@QDtech 2018-08-20",16,1);//居中显示
	delay_ms(1500);		
	delay_ms(1500);
}



/*****************************************************************************
 * @name       :void Test_Color(void)
 * @date       :2018-08-09 
 * @function   :Color fill test(white,black,red,green,blue)
 * @parameters :None
 * @retvalue   :None
******************************************************************************/
void Test_Color(void)
{
	//DrawTestPage("测试1:纯色填充测试");
	LCD_Fill(0,0,lcddev.width,lcddev.height,WHITE);
	Show_Str(20,30,BLUE,YELLOW,"BL Test",16,1);delay_ms(800);
	LCD_Fill(0,0,lcddev.width,lcddev.height,RED);
	Show_Str(20,30,BLUE,YELLOW,"RED ",16,1);delay_ms(800);
	LCD_Fill(0,0,lcddev.width,lcddev.height,GREEN);
	Show_Str(20,30,BLUE,YELLOW,"GREEN ",16,1);delay_ms(800);
	LCD_Fill(0,0,lcddev.width,lcddev.height,BLUE);
	Show_Str(20,30,RED,YELLOW,"BLUE ",16,1);delay_ms(800);
}

/*****************************************************************************
 * @name       :void Test_FillRec(void)
 * @date       :2018-08-09 
 * @function   :Rectangular display and fill test
								Display red,green,blue,yellow,pink rectangular boxes in turn,
								1500 milliseconds later,
								Fill the rectangle in red,green,blue,yellow and pink in turn
 * @parameters :None
 * @retvalue   :None
******************************************************************************/
void Test_FillRec(void)
{
	u8 i=0;
	DrawTestPage("测试2:GUI矩形填充测试");
	LCD_Fill(0,20,lcddev.width,lcddev.height-20,WHITE);
	for (i=0; i<5; i++) 
	{
		POINT_COLOR=ColorTab[i];
		LCD_DrawRectangle(lcddev.width/2-80+(i*15),lcddev.height/2-80+(i*15),lcddev.width/2-80+(i*15)+60,lcddev.height/2-80+(i*15)+60); 
	}
	delay_ms(1500);	
	LCD_Fill(0,20,lcddev.width,lcddev.height-20,WHITE); 
	for (i=0; i<5; i++) 
	{
		POINT_COLOR=ColorTab[i];
		LCD_DrawFillRectangle(lcddev.width/2-80+(i*15),lcddev.height/2-80+(i*15),lcddev.width/2-80+(i*15)+60,lcddev.height/2-80+(i*15)+60); 
	}
	delay_ms(1500);
}

/*****************************************************************************
 * @name       :void Test_Circle(void)
 * @date       :2018-08-09 
 * @function   :circular display and fill test
								Display red,green,blue,yellow,pink circular boxes in turn,
								1500 milliseconds later,
								Fill the circular in red,green,blue,yellow and pink in turn
 * @parameters :None
 * @retvalue   :None
******************************************************************************/
void Test_Circle(void)
{
	u8 i=0;
	DrawTestPage("测试3:GUI画圆填充测试");
	LCD_Fill(0,20,lcddev.width,lcddev.height-20,WHITE);
	for (i=0; i<5; i++)  
		gui_circle(lcddev.width/2-80+(i*25),lcddev.height/2-50+(i*25),ColorTab[i],30,0);
	delay_ms(1500);	
	LCD_Fill(0,20,lcddev.width,lcddev.height-20,WHITE); 
	for (i=0; i<5; i++) 
	  	gui_circle(lcddev.width/2-80+(i*25),lcddev.height/2-50+(i*25),ColorTab[i],30,1);
	delay_ms(1500);
}

/*****************************************************************************
 * @name       :void English_Font_test(void)
 * @date       :2018-08-09 
 * @function   :English display test
 * @parameters :None
 * @retvalue   :None
******************************************************************************/
void English_Font_test(void)
{
	DrawTestPage("测试5:英文显示测试");
	Show_Str(10,30,BLUE,YELLOW,"6X12:abcdefghijklmnopqrstuvwxyz0123456789",12,0);
	Show_Str(10,45,BLUE,YELLOW,"6X12:ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789",12,1);
	Show_Str(10,60,BLUE,YELLOW,"6X12:~!@#$%^&*()_+{}:<>?/|-+.",12,0);
	Show_Str(10,80,BLUE,YELLOW,"8X16:abcdefghijklmnopqrstuvwxyz0123456789",16,0);
	Show_Str(10,100,BLUE,YELLOW,"8X16:ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789",16,1);
	Show_Str(10,120,BLUE,YELLOW,"8X16:~!@#$%^&*()_+{}:<>?/|-+.",16,0); 
	delay_ms(1200);
}

/*****************************************************************************
 * @name       :void Test_Triangle(void)
 * @date       :2018-08-09 
 * @function   :triangle display and fill test
								Display red,green,blue,yellow,pink triangle boxes in turn,
								1500 milliseconds later,
								Fill the triangle in red,green,blue,yellow and pink in turn
 * @parameters :None
 * @retvalue   :None
******************************************************************************/
void Test_Triangle(void)
{
	u8 i=0;
	DrawTestPage("测试4:GUI Triangle填充测试");
	LCD_Fill(0,20,lcddev.width,lcddev.height-20,WHITE);
	for(i=0;i<5;i++)
	{
		POINT_COLOR=ColorTab[i];
		Draw_Triangel(lcddev.width/2-80+(i*20),lcddev.height/2-20+(i*15),lcddev.width/2-50-1+(i*20),lcddev.height/2-20-52-1+(i*15),lcddev.width/2-20-1+(i*20),lcddev.height/2-20+(i*15));
	}
	delay_ms(1500);	
	LCD_Fill(0,20,lcddev.width,lcddev.height-20,WHITE); 
	for(i=0;i<5;i++)
	{
		POINT_COLOR=ColorTab[i];
		Fill_Triangel(lcddev.width/2-80+(i*20),lcddev.height/2-20+(i*15),lcddev.width/2-50-1+(i*20),lcddev.height/2-20-52-1+(i*15),lcddev.width/2-20-1+(i*20),lcddev.height/2-20+(i*15));
	}
	delay_ms(1500);
}

/*****************************************************************************
 * @name       :void Chinese_Font_test(void)
 * @date       :2018-08-09 
 * @function   :chinese display test
 * @parameters :None
 * @retvalue   :None
******************************************************************************/
void Chinese_Font_test(void)
{	
	DrawTestPage("测试6:中文显示测试");
	Show_Str(10,30,BLUE,YELLOW,"16X16:全动电子技术有限公司欢迎您",16,0);
	Show_Str(10,50,BLUE,YELLOW,"16X16:Welcome全动电子",16,0);
	Show_Str(10,70,BLUE,YELLOW,"24X24:深圳市中文测试",24,1);
	Show_Str(10,100,BLUE,YELLOW,"32X32:字体测试",32,1);
	delay_ms(1200);
}

/*****************************************************************************
 * @name       :void Pic_test(void)
 * @date       :2018-08-09 
 * @function   :picture display test
 * @parameters :None
 * @retvalue   :None
******************************************************************************/
void Pic_test(void)
{
	DrawTestPage("测试7:图片显示测试");
	//LCD_Fill(0,20,lcddev.width,lcddev.height-20,WHITE);
	Gui_Drawbmp16(30,30,gImage_qq);
	Show_Str(30+12,75,BLUE,YELLOW,"QQ",16,1);
	Gui_Drawbmp16(90,30,gImage_qq);
	Show_Str(90+12,75,BLUE,YELLOW,"QQ",16,1);
	Gui_Drawbmp16(150,30,gImage_qq);
	Show_Str(150+12,75,BLUE,YELLOW,"QQ",16,1);
	delay_ms(1200);
}

/*****************************************************************************
 * @name       :void Rotate_Test(void)
 * @date       :2018-08-09 
 * @function   :rotate test
 * @parameters :None
 * @retvalue   :None
******************************************************************************/
void Rotate_Test(void)
{
	u8 i=0;
	u8 *Direction[4]={"Rotation:0","Rotation:90","Rotation:180","Rotation:270"};
	
	for(i=0;i<4;i++)
	{
	LCD_direction(i);
	DrawTestPage("测试8:屏幕旋转测试");
	Show_Str(20,30,BLUE,YELLOW,Direction[i],16,1);
	Gui_Drawbmp16(30,50,gImage_qq);
	delay_ms(1000);delay_ms(1000);
	}
	LCD_direction(USE_HORIZONTAL);
}

/*****************************************************************************
 * @name       :void Touch_Test(void)
 * @date       :2018-08-09 
 * @function   :touch test
 * @parameters :None
 * @retvalue   :None
******************************************************************************/
/*void Touch_Test(void)
{
	u8 key;
	u16 i=0;
	u16 j=0;
	u16 colorTemp=0;
	TP_Init();
	KEY_Init();
	LED_Init();
	DrawTestPage("测试9:Touch(按KEY0校准)      ");
	LCD_ShowString(lcddev.width-24,0,16,"RST",1);//显示清屏区域
	POINT_COLOR=RED;
	LCD_Fill(lcddev.width-52,2,lcddev.width-50+20,18,RED); 
		while(1)
	{
	 	key=KEY_Scan();
		tp_dev.scan(0); 		 
		if(tp_dev.sta&TP_PRES_DOWN)			//触摸屏被按下
		{	
		 	if(tp_dev.x<lcddev.width&&tp_dev.y<lcddev.height)
			{	
				if(tp_dev.x>(lcddev.width-24)&&tp_dev.y<16)
				{
					DrawTestPage("测试9:Touch(按KEY0校准)      ");//清除
					LCD_ShowString(lcddev.width-24,0,16,"RST",1);//显示清屏区域
					POINT_COLOR=colorTemp;
					LCD_Fill(lcddev.width-52,2,lcddev.width-50+20,18,POINT_COLOR); 
				}
				else if((tp_dev.x>(lcddev.width-60)&&tp_dev.x<(lcddev.width-50+20))&&tp_dev.y<20)
				{
				LCD_Fill(lcddev.width-52,2,lcddev.width-50+20,18,ColorTab[j%5]); 
				POINT_COLOR=ColorTab[(j++)%5];
				colorTemp=POINT_COLOR;
				delay_ms(10);
				}

				else TP_Draw_Big_Point(tp_dev.x,tp_dev.y,POINT_COLOR);		//画图	  			   
			}
		}else delay_ms(10);	//没有按键按下的时候 	    
		if(key==1)	//KEY_RIGHT按下,则执行校准程序
		{

			LCD_Clear(WHITE);//清屏
		    TP_Adjust();  //屏幕校准 
			TP_Save_Adjdata();	 
			DrawTestPage("测试9:Touch(按KEY0校准)      ");
			LCD_ShowString(lcddev.width-24,0,16,"RST",1);//显示清屏区域
			POINT_COLOR=colorTemp;
			LCD_Fill(lcddev.width-52,2,lcddev.width-50+20,18,POINT_COLOR); 
		}
		i++;
		if(i==30)
		{
			i=0;
		//	LED0=!LED0;
			//break;
		}
	}   
}
*/



