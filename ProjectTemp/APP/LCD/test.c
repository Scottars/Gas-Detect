//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//����Ӳ������Ƭ��STM32F103RCT6,����ԭ��MiniSTM32������,��Ƶ72MHZ������12MHZ
//QDtech-TFTҺ������ for STM32 IOģ��
//xiao��@ShenZhen QDtech co.,LTD
//��˾��վ:www.qdtft.com
//�Ա���վ��http://qdtech.taobao.com
//wiki������վ��http://www.lcdwiki.com
//��˾�ṩ����֧�֣��κμ������⻶ӭ��ʱ����ѧϰ
//�̻�(����) :+86 0755-23594567 
//�ֻ�:15989313508���빤�� 
//����:lcdwiki01@gmail.com    support@lcdwiki.com    goodtft@163.com 
//����֧��QQ:3002773612  3002778157
//��������QQȺ:324828016
//��������:2018/08/09
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������ȫ�����Ӽ������޹�˾ 2018-2028
//All rights reserved
/****************************************************************************************************
//=========================================��Դ����================================================//
//     LCDģ��                STM32��Ƭ��
//      VCC          ��        DC5V/3.3V      //��Դ
//      GND          ��          GND          //��Դ��
//=======================================Һ���������߽���==========================================//
//��ģ��Ĭ��������������ΪSPI����
//     LCDģ��                STM32��Ƭ��    
//    SDI(MOSI)      ��          PB15         //Һ����SPI��������д�ź�
//    SDO(MISO)      ��          PB14         //Һ����SPI�������ݶ��źţ��������Ҫ�������Բ�����
//=======================================Һ���������߽���==========================================//
//     LCDģ�� 					      STM32��Ƭ�� 
//       LED         ��          PB9          //Һ������������źţ��������Ҫ���ƣ���5V��3.3V
//       SCK         ��          PB13         //Һ����SPI����ʱ���ź�   
//      DC/RS        ��          PB10         //Һ��������/��������ź�
//       RST         ��          PB12         //Һ������λ�����ź�
//       CS          ��          PB11         //Һ����Ƭѡ�����ź�
//=========================================������������=========================================//
//���ģ�鲻���������ܻ��ߴ��д������ܣ����ǲ���Ҫ�������ܣ�����Ҫ���д���������
//	   LCDģ��                STM32��Ƭ�� 
//      T_IRQ        ��          PC10         //�����������ж��ź�
//      T_DO         ��          PC2          //������SPI���߶��ź�
//      T_DIN        ��          PC3          //������SPI����д�ź�
//      T_CS         ��          PC13         //������Ƭѡ�����ź�
//      T_CLK        ��          PC0          //������SPI����ʱ���ź�
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
u16 ColorTab[5]={RED,GREEN,BLUE,YELLOW,BRED};//������ɫ����
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

//���ƹ̶���up
LCD_Clear(WHITE);
LCD_Fill(0,0,lcddev.width,20,BLUE);
//���ƹ̶���down
LCD_Fill(0,lcddev.height-20,lcddev.width,lcddev.height,BLUE);
POINT_COLOR=WHITE;
Gui_StrCenter(0,2,WHITE,BLUE,str,16,1);//������ʾ
Gui_StrCenter(0,lcddev.height-18,WHITE,BLUE,"http://www.lcdwiki.com",16,1);//������ʾ
//���Ʋ�������
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
//	DrawTestPage("ȫ�������ۺϲ��Գ���");	
//	Gui_StrCenter(0,30,RED,BLUE,"ȫ������",16,1);//������ʾ
//	Gui_StrCenter(0,60,RED,BLUE,"�ۺϲ��Գ���",16,1);//������ʾ	
	char str[40];
	u8 i;
	char description[19] = {"V"};

	for(i=0;i<3;i++)
	{

	sprintf(str,"%f%.20s",voltage[i],description);
	

	//sprintf(str,"%.3s%.3s",s1,s2); ������ַ�����������
	//Gui_StrCenter(0,90,WHITE,RED,str,16,1);//������ʾ
	Show_Str(130,56+i*20,BLACK,WHITE,str,12,0);
	//delay_ms(10);		
	//delay_ms(10);
	}

	for(i=0;i<3;i++)
	{
	sprintf(str,"%f%.20s",Set_Voltage[i],description);
		//sprintf(str,"%.3s%.3s",s1,s2); ������ַ�����������
	//Gui_StrCenter(0,90,WHITE,RED,str,16,1);//������ʾ
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
	LCD_DrawLine(120,26, 120, 132);  //����Ļ�ϵ�����  ����λ�� 120 ������
	LCD_DrawLine(120,158, 120, 298); //����Ļ�ϵ�����
	
	
	//Main menu set
	//Gas Supply And Dectection System
	Show_Str(10,5,BLACK,WHITE,"Gas Suppply And Dectection System",12,0);//��ʾ   mdde 0  ��ʾ no overlay
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
	//����һ�����飬����ָʾ���¸����Ƶ�״̬
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
	DrawTestPage("ȫ�������ۺϲ��Գ���__wangsai");	
	Gui_StrCenter(0,30,RED,BLUE,"ȫ����zi",16,1);//������ʾ
	Gui_StrCenter(0,60,RED,BLUE,"�ۺϲ��Գ���",16,1);//������ʾ	
	Gui_StrCenter(0,90,GREEN,BLUE,"3.2\" ILI9341 240X320",16,1);//������ʾ
	Gui_StrCenter(0,120,BLUE,BLUE,"Pzx@QDtech 2018-08-20",16,1);//������ʾ
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
	//DrawTestPage("����1:��ɫ������");
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
	DrawTestPage("����2:GUI����������");
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
	DrawTestPage("����3:GUI��Բ������");
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
	DrawTestPage("����5:Ӣ����ʾ����");
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
	DrawTestPage("����4:GUI Triangle������");
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
	DrawTestPage("����6:������ʾ����");
	Show_Str(10,30,BLUE,YELLOW,"16X16:ȫ�����Ӽ������޹�˾��ӭ��",16,0);
	Show_Str(10,50,BLUE,YELLOW,"16X16:Welcomeȫ������",16,0);
	Show_Str(10,70,BLUE,YELLOW,"24X24:���������Ĳ���",24,1);
	Show_Str(10,100,BLUE,YELLOW,"32X32:�������",32,1);
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
	DrawTestPage("����7:ͼƬ��ʾ����");
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
	DrawTestPage("����8:��Ļ��ת����");
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
	DrawTestPage("����9:Touch(��KEY0У׼)      ");
	LCD_ShowString(lcddev.width-24,0,16,"RST",1);//��ʾ��������
	POINT_COLOR=RED;
	LCD_Fill(lcddev.width-52,2,lcddev.width-50+20,18,RED); 
		while(1)
	{
	 	key=KEY_Scan();
		tp_dev.scan(0); 		 
		if(tp_dev.sta&TP_PRES_DOWN)			//������������
		{	
		 	if(tp_dev.x<lcddev.width&&tp_dev.y<lcddev.height)
			{	
				if(tp_dev.x>(lcddev.width-24)&&tp_dev.y<16)
				{
					DrawTestPage("����9:Touch(��KEY0У׼)      ");//���
					LCD_ShowString(lcddev.width-24,0,16,"RST",1);//��ʾ��������
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

				else TP_Draw_Big_Point(tp_dev.x,tp_dev.y,POINT_COLOR);		//��ͼ	  			   
			}
		}else delay_ms(10);	//û�а������µ�ʱ�� 	    
		if(key==1)	//KEY_RIGHT����,��ִ��У׼����
		{

			LCD_Clear(WHITE);//����
		    TP_Adjust();  //��ĻУ׼ 
			TP_Save_Adjdata();	 
			DrawTestPage("����9:Touch(��KEY0У׼)      ");
			LCD_ShowString(lcddev.width-24,0,16,"RST",1);//��ʾ��������
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



