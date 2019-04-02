#include "dac.h"

//////////////////////////////////////////////////////////////////////////////////	 
//±¾³ÌĞòÖ»¹©Ñ§Ï°Ê¹ÓÃ£¬Î´¾­×÷ÕßĞí¿É£¬²»µÃÓÃÓÚÆäËüÈÎºÎÓÃÍ¾
//ALIENTEKÕ½½¢STM32¿ª·¢°å
//DAC ´úÂë	   
//ÕıµãÔ­×Ó@ALIENTEK
//¼¼ÊõÂÛÌ³:www.openedv.com
//ĞŞ¸ÄÈÕÆÚ:2012/9/8
//°æ±¾£ºV1.0
//°æÈ¨ËùÓĞ£¬µÁ°æ±Ø¾¿¡£
//Copyright(C) ¹ãÖİÊĞĞÇÒíµç×Ó¿Æ¼¼ÓĞÏŞ¹«Ë¾ 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////
//DACÍ¨µÀ1Êä³ö³õÊ¼»¯
void Dac1_Init(void)
{
  
	GPIO_InitTypeDef GPIO_InitStructure;
	DAC_InitTypeDef DAC_InitType;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE );	  //Ê¹ÄÜPORTAÍ¨µÀÊ±ÖÓ
   	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE );	  //Ê¹ÄÜDACÍ¨µÀÊ±ÖÓ 

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;				 // ¶Ë¿ÚÅäÖÃ
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; 		 //Ä£ÄâÊäÈë
 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_SetBits(GPIOA,GPIO_Pin_4)	;//PA.4 Êä³ö¸ß
					
	DAC_InitType.DAC_Trigger=DAC_Trigger_None;	//²»Ê¹ÓÃ´¥·¢¹¦ÄÜ TEN1=0
	DAC_InitType.DAC_WaveGeneration=DAC_WaveGeneration_None;//²»Ê¹ÓÃ²¨ĞÎ·¢Éú
	DAC_InitType.DAC_LFSRUnmask_TriangleAmplitude=DAC_LFSRUnmask_Bit0;//ÆÁ±Î¡¢·ùÖµÉèÖÃ
	DAC_InitType.DAC_OutputBuffer=DAC_OutputBuffer_Disable ;	//DAC1Êä³ö»º´æ¹Ø±Õ BOFF1=1
    DAC_Init(DAC_Channel_1,&DAC_InitType);	 //³õÊ¼»¯DACÍ¨µÀ1

	DAC_Cmd(DAC_Channel_1, ENABLE);  //Ê¹ÄÜDAC1
  
    DAC_SetChannel1Data(DAC_Align_12b_R, 0);  //12Î»ÓÒ¶ÔÆëÊı¾İ¸ñÊ½ÉèÖÃDACÖµ

}

//è®¾ç½®é€šé“1è¾“å‡ºç”µå‹
//vol:0~3300,presentï¼š0~3.3V
//we can set a ceiling to limit the value
void Dac1_Set_Vol(u16 vol)
{
	float temp=vol;
	temp/=1000;
	temp=temp*4096/3.3;
	DAC_SetChannel1Data(DAC_Align_12b_R,temp);//12Î»ÓÒ¶ÔÆëÊı¾İ¸ñÊ½ÉèÖÃDACÖµ
}








