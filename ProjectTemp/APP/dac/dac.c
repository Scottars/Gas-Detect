#include "dac.h"


/*******************************************************************************
* Function name  : Dac1_Init
* Description  : We initialize the dac pin
* Input : None
* Output  :  None
* Return Value :  None
* Attention: this function is to initiallize the Dac1  

*******************************************************************************/

//DACÕ®µ¿1 ‰≥ˆ≥ı ºªØ
void Dac1_Init(void)
{
  
	GPIO_InitTypeDef GPIO_InitStructure;
	DAC_InitTypeDef DAC_InitType;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE );	  // πƒ‹PORTAÕ®µ¿ ±÷”
   	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE );	  // πƒ‹DACÕ®µ¿ ±÷” 

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;				 // ∂Àø⁄≈‰÷√
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; 		 //ƒ£ƒ‚ ‰»Î
 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_SetBits(GPIOA,GPIO_Pin_4)	;//PA.4  ‰≥ˆ∏ﬂ
					
	DAC_InitType.DAC_Trigger=DAC_Trigger_None;	//≤ª π”√¥•∑¢π¶ƒ‹ TEN1=0
	DAC_InitType.DAC_WaveGeneration=DAC_WaveGeneration_None;//≤ª π”√≤®–Œ∑¢…˙
	DAC_InitType.DAC_LFSRUnmask_TriangleAmplitude=DAC_LFSRUnmask_Bit0;//∆¡±Œ°¢∑˘÷µ…Ë÷√
	DAC_InitType.DAC_OutputBuffer=DAC_OutputBuffer_Disable ;	//DAC1 ‰≥ˆª∫¥Êπÿ±’ BOFF1=1
    DAC_Init(DAC_Channel_1,&DAC_InitType);	 //≥ı ºªØDACÕ®µ¿1

	DAC_Cmd(DAC_Channel_1, ENABLE);  // πƒ‹DAC1
  
    DAC_SetChannel1Data(DAC_Align_12b_R, 0);  //12Œª”“∂‘∆Î ˝æ›∏Ò Ω…Ë÷√DAC÷µ

}

/*******************************************************************************
* Function name  : Dac1_Set_Vol
* Description  : we use this function to set Dac pin output 
* Input : the volatage you want to set(actually it is the voltage * 1000)
* Output  :  None
* Return Value :  None
* Attention: we actually  set the dac channel 
				vol:0~3300,presentÔºö0~3.3V
				we can set a ceiling to limit the value

*******************************************************************************/


void Dac1_Set_Vol(u16 vol)
{
	float temp=vol;
	temp/=1000;
	temp=temp*4096/3.3;
	DAC_SetChannel1Data(DAC_Align_12b_R,temp);//12Œª”“∂‘∆Î ˝æ›∏Ò Ω…Ë÷√DAC÷µ
}








