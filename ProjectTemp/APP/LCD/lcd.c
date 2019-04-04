//////////////////////////////////////////////////////////////////////////////////	 
//±æ≥Ã–Ú÷ªπ©—ßœ∞ π”√£¨Œ¥æ≠◊˜’ﬂ–Ìø…£¨≤ªµ√”√”⁄∆‰À¸»Œ∫Œ”√Õæ
//≤‚ ‘”≤º˛£∫µ•∆¨ª˙STM32F103RCT6,’˝µ„‘≠◊”MiniSTM32ø™∑¢∞Â,÷˜∆µ72MHZ£¨æß’Ò12MHZ
//QDtech-TFT“∫æß«˝∂Ø for STM32 IOƒ£ƒ‚
//xiao∑Î@ShenZhen QDtech co.,LTD
//π´ÀæÕ¯’æ:www.qdtft.com
//Ã‘±¶Õ¯’æ£∫http://qdtech.taobao.com
//wikiºº ıÕ¯’æ£∫http://www.lcdwiki.com
//Œ“ÀæÃ·π©ºº ı÷ß≥÷£¨»Œ∫Œºº ıŒ Ã‚ª∂”≠ÀÊ ±Ωª¡˜—ßœ∞
//πÃª∞(¥´’Ê) :+86 0755-23594567 
// ÷ª˙:15989313508£®∑Îπ§£© 
//” œ‰:lcdwiki01@gmail.com    support@lcdwiki.com    goodtft@163.com 
//ºº ı÷ß≥÷QQ:3002773612  3002778157
//ºº ıΩª¡˜QQ»∫:324828016
//¥¥Ω®»’∆⁄:2018/08/09
//∞Ê±æ£∫V1.0
//∞Ê»®À˘”–£¨µ¡∞Ê±ÿæø°£
//Copyright(C) …Ó€⁄ –»´∂ØµÁ◊”ºº ı”–œﬁπ´Àæ 2018-2028
//All rights reserved
/****************************************************************************************************
//=========================================ÁîµÊ∫êÊé•Á∫ø================================================//
//     LCDÊ®°Âùó                STM32ÂçïÁâáÊú∫
-
//      VCC          Êé•        DC5V/3.3V      //ÁîµÊ∫ê    						   -----same
//      GND          Êé•          GND          //ÁîµÊ∫êÂú∞       						---- same
//=======================================Ê∂≤Êô∂Â±èÊï∞ÊçÆÁ∫øÊé•Á∫ø==========================================//
//Êú¨Ê®°ÂùóÈªòËÆ§Êï∞ÊçÆÊÄªÁ∫øÁ±ªÂûã‰∏∫SPIÊÄªÁ∫ø
//     LCDÊ®°Âùó                STM32ÂçïÁâáÊú∫    
//    SDI(MOSI)      Êé•          PB15         //Ê∂≤Êô∂Â±èSPIÊÄªÁ∫øÊï∞ÊçÆÂÜô‰ø°Âè∑
//    SDO(MISO)      Êé•          PB14         //Ê∂≤Êô∂Â±èSPIÊÄªÁ∫øÊï∞ÊçÆËØª‰ø°Âè∑ÔºåÂ¶ÇÊûú‰∏çÈúÄË¶ÅËØªÔºåÂèØ‰ª•‰∏çÊé•Á∫ø
//=======================================Ê∂≤Êô∂Â±èÊéßÂà∂Á∫øÊé•Á∫ø==========================================//
//     LCDÊ®°Âùó 					      STM32ÂçïÁâáÊú∫ 
//       LED         Êé•          PB9          //Ê∂≤Êô∂Â±èËÉåÂÖâÊéßÂà∂‰ø°Âè∑ÔºåÂ¶ÇÊûú‰∏çÈúÄË¶ÅÊéßÂà∂ÔºåÊé•5VÊàñ3.3V   
//       SCK         Êé•          PB13         //Ê∂≤Êô∂Â±èSPIÊÄªÁ∫øÊó∂Èíü‰ø°Âè∑    ----
//      DC/RS        Êé•          PB10         //Ê∂≤Êô∂Â±èÊï∞ÊçÆ/ÂëΩ‰ª§ÊéßÂà∂‰ø°Âè∑
//       RST         Êé•          PB12         //Ê∂≤Êô∂Â±èÂ§ç‰ΩçÊéßÂà∂‰ø°Âè∑
//       CS          Êé•          PB11         //Ê∂≤Êô∂Â±èÁâáÈÄâÊéßÂà∂‰ø°Âè∑   ----Scottar: me too
//=========================================Ëß¶Êë∏Â±èËß¶Êé•Á∫ø=========================================//
//Â¶ÇÊûúÊ®°Âùó‰∏çÂ∏¶Ëß¶Êë∏ÂäüËÉΩÊàñËÄÖÂ∏¶ÊúâËß¶Êë∏ÂäüËÉΩÔºå‰ΩÜÊòØ‰∏çÈúÄË¶ÅËß¶Êë∏ÂäüËÉΩÔºåÂàô‰∏çÈúÄË¶ÅËøõË°åËß¶Êë∏Â±èÊé•Á∫ø------ we donnot use these lines
//	   LCDÊ®°Âùó                STM32ÂçïÁâáÊú∫ 
//      T_IRQ        Êé•          PC10         //Ëß¶Êë∏Â±èËß¶Êë∏‰∏≠Êñ≠‰ø°Âè∑
//      T_DO         Êé•          PC2          //Ëß¶Êë∏Â±èSPIÊÄªÁ∫øËØª‰ø°Âè∑
//      T_DIN        Êé•          PC3          //Ëß¶Êë∏Â±èSPIÊÄªÁ∫øÂÜô‰ø°Âè∑
//      T_CS         Êé•          PC13         //Ëß¶Êë∏Â±èÁâáÈÄâÊéßÂà∂‰ø°Âè∑
//      T_CLK        Êé•          PC0          //Ëß¶Êë∏Â±èSPIÊÄªÁ∫øÊó∂Èíü‰ø°Âè∑
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
#include "stdlib.h"
#include "systick.h" 
#include "spi.h"

	   
//π‹¿ÌLCD÷ÿ“™≤Œ ˝
//ƒ¨»œŒ™ ˙∆¡
_lcd_dev lcddev;

//ª≠± —’…´,±≥æ∞—’…´
u16 POINT_COLOR = 0x0000,BACK_COLOR = 0xFFFF;  
u16 DeviceCode;	 

/*****************************************************************************
 * @name       :void LCD_WR_REG(u8 data)
 * @date       :2018-08-09 
 * @function   :Write an 8-bit command to the LCD screen
 * @parameters :data:Command value to be written
 * @retvalue   :None
******************************************************************************/
void LCD_WR_REG(u8 data)
{ 
   LCD_CS_CLR;     
	 LCD_RS_CLR;	  
   SPI_WriteByte(SPI2,data);
   LCD_CS_SET;	
}

/*****************************************************************************
 * @name       :void LCD_WR_DATA(u8 data)
 * @date       :2018-08-09 
 * @function   :Write an 8-bit data to the LCD screen
 * @parameters :data:data value to be written
 * @retvalue   :None
******************************************************************************/
void LCD_WR_DATA(u8 data)
{
   LCD_CS_CLR;
	 LCD_RS_SET;
   SPI_WriteByte(SPI2,data);
   LCD_CS_SET;
}

/*****************************************************************************
 * @name       :void LCD_WriteReg(u8 LCD_Reg, u16 LCD_RegValue)
 * @date       :2018-08-09 
 * @function   :Write data into registers
 * @parameters :LCD_Reg:Register address
                LCD_RegValue:Data to be written
 * @retvalue   :None
******************************************************************************/
void LCD_WriteReg(u8 LCD_Reg, u16 LCD_RegValue)
{	
	LCD_WR_REG(LCD_Reg);  
	LCD_WR_DATA(LCD_RegValue);	    		 
}	   

/*****************************************************************************
 * @name       :void LCD_WriteRAM_Prepare(void)
 * @date       :2018-08-09 
 * @function   :Write GRAM
 * @parameters :None
 * @retvalue   :None
******************************************************************************/	 
void LCD_WriteRAM_Prepare(void)
{
	LCD_WR_REG(lcddev.wramcmd);
}	 

/*****************************************************************************
 * @name       :void Lcd_WriteData_16Bit(u16 Data)
 * @date       :2018-08-09 
 * @function   :Write an 16-bit command to the LCD screen
 * @parameters :Data:Data to be written
 * @retvalue   :None
******************************************************************************/	 
void Lcd_WriteData_16Bit(u16 Data)
{	
   LCD_CS_CLR;
   LCD_RS_SET;  
   SPI_WriteByte(SPI2,Data>>8);
	 SPI_WriteByte(SPI2,Data);
   LCD_CS_SET;
}

/*****************************************************************************
 * @name       :void LCD_DrawPoint(u16 x,u16 y)
 * @date       :2018-08-09 
 * @function   :Write a pixel data at a specified location
 * @parameters :x:the x coordinate of the pixel
                y:the y coordinate of the pixel
 * @retvalue   :None
******************************************************************************/	
void LCD_DrawPoint(u16 x,u16 y)
{
	LCD_SetCursor(x,y);//…Ë÷√π‚±ÍŒª÷√ 
	Lcd_WriteData_16Bit(POINT_COLOR); 
}

/*****************************************************************************
 * @name       :void LCD_Clear(u16 Color)
 * @date       :2018-08-09 
 * @function   :Full screen filled LCD screen
 * @parameters :color:Filled color
 * @retvalue   :None
******************************************************************************/	
void LCD_Clear(u16 Color)
{
  unsigned int i,m;  
	LCD_SetWindows(0,0,lcddev.width-1,lcddev.height-1);   
	LCD_CS_CLR;
	LCD_RS_SET;
	for(i=0;i<lcddev.height;i++)
	{
    for(m=0;m<lcddev.width;m++)
    {	
			Lcd_WriteData_16Bit(Color);
		}
	}
	 LCD_CS_SET;
} 

/*****************************************************************************
 * @name       :void LCD_GPIOInit(void)
 * @date       :2018-08-09 
 * @function   :Initialization LCD screen GPIO
 * @parameters :None
 * @retvalue   :None
******************************************************************************/	
void LCD_GPIOInit(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;	      
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB ,ENABLE);	// πƒ‹GPIOB ±÷”
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9| GPIO_Pin_10| GPIO_Pin_11| GPIO_Pin_12; //GPIOB9,10,11,12
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   //Õ∆ÕÏ ‰≥ˆ
	GPIO_Init(GPIOB, &GPIO_InitStructure);//≥ı ºªØ
}

/*****************************************************************************
 * @name       :void LCD_RESET(void)
 * @date       :2018-08-09 
 * @function   :Reset LCD screen
 * @parameters :None
 * @retvalue   :None
******************************************************************************/	
void LCD_RESET(void)
{
	LCD_RST_CLR;
	delay_ms(100);	
	LCD_RST_SET;
	delay_ms(50);
}

/*****************************************************************************
 * @name       :void LCD_RESET(void)
 * @date       :2018-08-09 
 * @function   :Initialization LCD screen
 * @parameters :None
 * @retvalue   :None
******************************************************************************/	 	 
void LCD_Init(void)
{  
	SPI2_Init(); //”≤º˛SPI2≥ı ºªØ
	LCD_GPIOInit();//LCD GPIO≥ı ºªØ										 
 	LCD_RESET(); //LCD ∏¥Œª
//************* ILI9341≥ı ºªØ**********//	
LCD_WR_REG(0xCF);  
	LCD_WR_DATA(0x00); 
	LCD_WR_DATA(0xC1); 
	LCD_WR_DATA(0X30); 
	LCD_WR_REG(0xED);  
	LCD_WR_DATA(0x64); 
	LCD_WR_DATA(0x03); 
	LCD_WR_DATA(0X12); 
	LCD_WR_DATA(0X81); 
	LCD_WR_REG(0xE8);  
	LCD_WR_DATA(0x85); 
	LCD_WR_DATA(0x10); 
	LCD_WR_DATA(0x7A); 
	LCD_WR_REG(0xCB);  
	LCD_WR_DATA(0x39); 
	LCD_WR_DATA(0x2C); 
	LCD_WR_DATA(0x00); 
	LCD_WR_DATA(0x34); 
	LCD_WR_DATA(0x02); 
	LCD_WR_REG(0xF7);  
	LCD_WR_DATA(0x20); 
	LCD_WR_REG(0xEA);  
	LCD_WR_DATA(0x00); 
	LCD_WR_DATA(0x00); 
	LCD_WR_REG(0xC0);    //Power control 
	LCD_WR_DATA(0x1B);   //VRH[5:0] 
	LCD_WR_REG(0xC1);    //Power control 
	LCD_WR_DATA(0x01);   //SAP[2:0];BT[3:0] 
	LCD_WR_REG(0xC5);    //VCM control 
	LCD_WR_DATA(0x30); 	 //3F
	LCD_WR_DATA(0x30); 	 //3C
	LCD_WR_REG(0xC7);    //VCM control2 
	LCD_WR_DATA(0XB7); 
	LCD_WR_REG(0x36);    // Memory Access Control 
	LCD_WR_DATA(0x48); 
	LCD_WR_REG(0x3A);   
	LCD_WR_DATA(0x55); 
	LCD_WR_REG(0xB1);   
	LCD_WR_DATA(0x00);   
	LCD_WR_DATA(0x1A); 
	LCD_WR_REG(0xB6);    // Display Function Control 
	LCD_WR_DATA(0x0A); 
	LCD_WR_DATA(0xA2); 
	LCD_WR_REG(0xF2);    // 3Gamma Function Disable 
	LCD_WR_DATA(0x00); 
	LCD_WR_REG(0x26);    //Gamma curve selected 
	LCD_WR_DATA(0x01); 
	LCD_WR_REG(0xE0);    //Set Gamma 
	LCD_WR_DATA(0x0F); 
	LCD_WR_DATA(0x2A); 
	LCD_WR_DATA(0x28); 
	LCD_WR_DATA(0x08); 
	LCD_WR_DATA(0x0E); 
	LCD_WR_DATA(0x08); 
	LCD_WR_DATA(0x54); 
	LCD_WR_DATA(0XA9); 
	LCD_WR_DATA(0x43); 
	LCD_WR_DATA(0x0A); 
	LCD_WR_DATA(0x0F); 
	LCD_WR_DATA(0x00); 
	LCD_WR_DATA(0x00); 
	LCD_WR_DATA(0x00); 
	LCD_WR_DATA(0x00); 		 
	LCD_WR_REG(0XE1);    //Set Gamma 
	LCD_WR_DATA(0x00); 
	LCD_WR_DATA(0x15); 
	LCD_WR_DATA(0x17); 
	LCD_WR_DATA(0x07); 
	LCD_WR_DATA(0x11); 
	LCD_WR_DATA(0x06); 
	LCD_WR_DATA(0x2B); 
	LCD_WR_DATA(0x56); 
	LCD_WR_DATA(0x3C); 
	LCD_WR_DATA(0x05); 
	LCD_WR_DATA(0x10); 
	LCD_WR_DATA(0x0F); 
	LCD_WR_DATA(0x3F); 
	LCD_WR_DATA(0x3F); 
	LCD_WR_DATA(0x0F); 
	LCD_WR_REG(0x2B); 
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x01);
	LCD_WR_DATA(0x3f);
	LCD_WR_REG(0x2A); 
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0xef);	 
	LCD_WR_REG(0x11); //Exit Sleep
	delay_ms(120);
	LCD_WR_REG(0x29); //display on	

  LCD_direction(USE_HORIZONTAL);//…Ë÷√LCDœ‘ æ∑ΩœÚ
	LCD_LED=1;//µ„¡¡±≥π‚	 
	LCD_Clear(WHITE);//«Â»´∆¡∞◊…´
}
 
/*****************************************************************************
 * @name       :void LCD_SetWindows(u16 xStar, u16 yStar,u16 xEnd,u16 yEnd)
 * @date       :2018-08-09 
 * @function   :Setting LCD display window
 * @parameters :xStar:the bebinning x coordinate of the LCD display window
								yStar:the bebinning y coordinate of the LCD display window
								xEnd:the endning x coordinate of the LCD display window
								yEnd:the endning y coordinate of the LCD display window
 * @retvalue   :None
******************************************************************************/ 
void LCD_SetWindows(u16 xStar, u16 yStar,u16 xEnd,u16 yEnd)
{	
	LCD_WR_REG(lcddev.setxcmd);	
	LCD_WR_DATA(xStar>>8);
	LCD_WR_DATA(0x00FF&xStar);		
	LCD_WR_DATA(xEnd>>8);
	LCD_WR_DATA(0x00FF&xEnd);

	LCD_WR_REG(lcddev.setycmd);	
	LCD_WR_DATA(yStar>>8);
	LCD_WR_DATA(0x00FF&yStar);		
	LCD_WR_DATA(yEnd>>8);
	LCD_WR_DATA(0x00FF&yEnd);

	LCD_WriteRAM_Prepare();	//ø™ º–¥»ÎGRAM			
}   

/*****************************************************************************
 * @name       :void LCD_SetCursor(u16 Xpos, u16 Ypos)
 * @date       :2018-08-09 
 * @function   :Set coordinate value
 * @parameters :Xpos:the  x coordinate of the pixel
								Ypos:the  y coordinate of the pixel
 * @retvalue   :None
******************************************************************************/ 
void LCD_SetCursor(u16 Xpos, u16 Ypos)
{	  	    			
	LCD_SetWindows(Xpos,Ypos,Xpos,Ypos);	
} 

/*****************************************************************************
 * @name       :void LCD_direction(u8 direction)
 * @date       :2018-08-09 
 * @function   :Setting the display direction of LCD screen
 * @parameters :direction:0-0 degree
                          1-90 degree
													2-180 degree
													3-270 degree
 * @retvalue   :None
******************************************************************************/ 
void LCD_direction(u8 direction)
{ 
			lcddev.setxcmd=0x2A;
			lcddev.setycmd=0x2B;
			lcddev.wramcmd=0x2C;
	switch(direction){		  
		case 0:						 	 		
			lcddev.width=LCD_W;
			lcddev.height=LCD_H;		
			LCD_WriteReg(0x36,(1<<3)|(0<<6)|(0<<7));//BGR==1,MY==0,MX==0,MV==0
		break;
		case 1:
			lcddev.width=LCD_H;
			lcddev.height=LCD_W;
			LCD_WriteReg(0x36,(1<<3)|(0<<7)|(1<<6)|(1<<5));//BGR==1,MY==1,MX==0,MV==1
		break;
		case 2:						 	 		
			lcddev.width=LCD_W;
			lcddev.height=LCD_H;	
			LCD_WriteReg(0x36,(1<<3)|(1<<6)|(1<<7));//BGR==1,MY==0,MX==0,MV==0
		break;
		case 3:
			lcddev.width=LCD_H;
			lcddev.height=LCD_W;
			LCD_WriteReg(0x36,(1<<3)|(1<<7)|(1<<5));//BGR==1,MY==1,MX==0,MV==1
		break;	
		default:break;
	}		
}	 
