

#include "adc.h"
#include "systick.h"


/*******************************************************************************
* º¯ Êı Ãû         : adc_init
* º¯Êı¹¦ÄÜ         : IO¶Ë¿ÚÊ±ÖÓ³õÊ¼»¯º¯Êı
* Êä    Èë         : ÎŞ
* Êä    ³ö         : ÎŞ
*******************************************************************************/
volatile uint16_t ADC_ConvertedValue[10][3];

void dma_test()
{

	GPIO_InitTypeDef GPIO_InitStructure;
    ADC_InitTypeDef ADC_InitStructure;
		DMA_InitTypeDef DMA_InitStructure;
	
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);
	
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC|RCC_APB2Periph_AFIO|RCC_APB2Periph_ADC1,ENABLE);

    RCC_ADCCLKConfig(RCC_PCLK2_Div6);//12M  ???14M ????ADC????ADCCLK??

  
		DMA_DeInit(DMA1_Channel1);    //å°†é€šé“ä¸€å¯„å­˜å™¨è®¾ä¸ºé»˜è®¤å€¼
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(ADC1->DR);//è¯¥å‚æ•°ç”¨ä»¥å®šä¹‰DMAå¤–è®¾åŸºåœ°å€
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&ADC_ConvertedValue;//è¯¥å‚æ•°ç”¨ä»¥å®šä¹‰DMAå†…å­˜åŸºåœ°å€(è½¬æ¢ç»“æœä¿å­˜çš„åœ°å€)
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;//è¯¥å‚æ•°è§„å®šäº†å¤–è®¾æ˜¯ä½œä¸ºæ•°æ®ä¼ è¾“çš„ç›®çš„åœ°è¿˜æ˜¯æ¥æºï¼Œæ­¤å¤„æ˜¯ä½œä¸ºæ¥æº
    DMA_InitStructure.DMA_BufferSize = 3*10;//å®šä¹‰æŒ‡å®šDMAé€šé“çš„DMAç¼“å­˜çš„å¤§å°,å•ä½ä¸ºæ•°æ®å•ä½ã€‚è¿™é‡Œä¹Ÿå°±æ˜¯ADCConvertedValueçš„å¤§å°
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//è®¾å®šå¤–è®¾åœ°å€å¯„å­˜å™¨é€’å¢ä¸å¦,æ­¤å¤„è®¾ä¸ºä¸å˜ Disable
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//ç”¨æ¥è®¾å®šå†…å­˜åœ°å€å¯„å­˜å™¨é€’å¢ä¸å¦,æ­¤å¤„è®¾ä¸ºé€’å¢ï¼ŒEnable
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;//æ•°æ®å®½åº¦ä¸º16ä½
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;//æ•°æ®å®½åº¦ä¸º16ä½
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular; //å·¥ä½œåœ¨å¾ªç¯ç¼“å­˜æ¨¡å¼
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;//DMAé€šé“æ‹¥æœ‰é«˜ä¼˜å…ˆçº§ åˆ†åˆ«4ä¸ªç­‰çº§ ä½ã€ä¸­ã€é«˜ã€éå¸¸é«˜
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;//ä½¿èƒ½DMAé€šé“çš„å†…å­˜åˆ°å†…å­˜ä¼ è¾“
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);//æ ¹æ®DMA_InitStructä¸­æŒ‡å®šçš„å‚æ•°åˆå§‹åŒ–DMAçš„é€šé“

    DMA_Cmd(DMA1_Channel1, ENABLE);//å¯åŠ¨DMAé€šé“ä¸€

	
	
	  GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0 |GPIO_Pin_1 | GPIO_Pin_2;//ADC
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AIN; //???????
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_Init(GPIOC,&GPIO_InitStructure);

		
	
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 3;
    ADC_Init(ADC1, &ADC_InitStructure);

    //???????ADC??????????????????????????????????
   
		//RCC_ADCCLKConfig(RCC_PCLK2_Div8);//é…ç½®ADCæ—¶é’Ÿï¼Œä¸ºPCLK2çš„8åˆ†é¢‘ï¼Œå³9Hz  è¿™ä¸ªä¸Šé¢å°±å·²ç»è¿›è¡Œå®šä¹‰äº†
		//ADC_RegularChannelConfig(ADC1,ADC_Channel_10,1,ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1,ADC_Channel_10,1,ADC_SampleTime_239Cycles5);//é€šé“ä¸€è½¬æ¢ç»“æœä¿å­˜åˆ°ADCConvertedValue[0~10][0]
	ADC_RegularChannelConfig(ADC1,ADC_Channel_11,2,ADC_SampleTime_239Cycles5);//é€šé“äºŒè½¬æ¢ç»“æœä¿å­˜åˆ°ADCConvertedValue[0~10][1]
	ADC_RegularChannelConfig(ADC1,ADC_Channel_12,3,ADC_SampleTime_239Cycles5);//é€šé“ä¸‰è½¬æ¢ç»“æœä¿å­˜åˆ°ADCConvertedValue[0~10][2]



		ADC_DMACmd(ADC1,ENABLE);
    ADC_Cmd(ADC1,ENABLE);

    ADC_ResetCalibration(ADC1);//?????????ADC??Ğ£??????
    while(ADC_GetResetCalibrationStatus(ADC1));//???ADC????Ğ£??????????

    ADC_StartCalibration(ADC1);//??????ADC??Ğ£???
    while(ADC_GetCalibrationStatus(ADC1));//??????ADC??Ğ£?????

	ADC_SoftwareStartConvCmd(ADC1, ENABLE);//??????????????ADC???????????????


}


float AD_Conversion_1479A_DMA()
{
	
	
	int sum;
	u8 i,j;
	float ADValue_1479A;//ç”¨æ¥ä¿å­˜ç»è¿‡è½¬æ¢å¾—åˆ°çš„ç”µå‹å€¼
	i=0;
	sum=0;
	
	for(j=0;j<10;j++)
	{
		sum +=ADC_ConvertedValue[j][i];
	//	printf("Insideï¼šã€‚%f\r\n",ADC_ConvertedValue[j][i]);
	}
	ADValue_1479A=(float)sum/(10*4096)*3.3;//æ±‚å¹³å‡å€¼å¹¶è½¬æ¢æˆç”µå‹å€¼
	printf("The current ADValue_1479A value =%f\r\n",ADValue_1479A);
	return ADValue_1479A;
}


float AD_Conversion_627D_DMA()
{
	
	int sum;
	u8 i,j;
	float ADValue_627D;//ç”¨æ¥ä¿å­˜ç»è¿‡è½¬æ¢å¾—åˆ°çš„ç”µå‹å€¼
	i=0;
	sum=0;
	
	for(j=0;j<10;j++)
	{
		sum +=ADC_ConvertedValue[j][i];
	//	printf("Insideï¼šã€‚%f\r\n",ADC_ConvertedValue[j][i]);
	}
	ADValue_627D=(float)sum/(10*4096)*3.3;//æ±‚å¹³å‡å€¼å¹¶è½¬æ¢æˆç”µå‹å€¼
	printf("The currentADValue_627D value =%f\r\n",ADValue_627D);
	return ADValue_627D;
		
}


float AD_Conversion_025D_DMA()
{
	
	int sum;
	u8 i,j;
	float ADValue_025D;//ç”¨æ¥ä¿å­˜ç»è¿‡è½¬æ¢å¾—åˆ°çš„ç”µå‹å€¼
	i=0;
	sum=0;
	
	for(j=0;j<10;j++)
	{
		sum +=ADC_ConvertedValue[j][i];
	//	printf("Insideï¼šã€‚%f\r\n",ADC_ConvertedValue[j][i]);
	}
	ADValue_025D=(float)sum/(10*4096)*3.3;//æ±‚å¹³å‡å€¼å¹¶è½¬æ¢æˆç”µå‹å€¼
	printf("The current ADValue_025D value =%f\r\n",ADValue_025D);
	return ADValue_025D;
}







/*******************************************************************************
* º¯ Êı Ãû         : adc_init
* º¯Êı¹¦ÄÜ         : IO¶Ë¿ÚÊ±ÖÓ³õÊ¼»¯º¯Êı
* Êä    Èë         : ÎŞ
* Êä    ³ö         : ÎŞ
*******************************************************************************/
void adc10_init()
{
    GPIO_InitTypeDef GPIO_InitStructure;
    ADC_InitTypeDef ADC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC|RCC_APB2Periph_AFIO|RCC_APB2Periph_ADC1,ENABLE);

    RCC_ADCCLKConfig(RCC_PCLK2_Div6);//12M  ×î´ó14M ÉèÖÃADCÊ±ÖÓ£¨ADCCLK£©

    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0;//ADC
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AIN; //Ä£ÄâÊäÈë
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_Init(GPIOC,&GPIO_InitStructure);


    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &ADC_InitStructure);

    //ÉèÖÃÖ¸¶¨ADCµÄ¹æÔò×éÍ¨µÀ£¬ÉèÖÃËüÃÇµÄ×ª»¯Ë³ĞòºÍ²ÉÑùÊ±¼ä
    ADC_RegularChannelConfig(ADC1,ADC_Channel_10,1,ADC_SampleTime_239Cycles5);

    ADC_Cmd(ADC1,ENABLE);

    ADC_ResetCalibration(ADC1);//ÖØÖÃÖ¸¶¨µÄADCµÄĞ£×¼¼Ä´æÆ÷
    while(ADC_GetResetCalibrationStatus(ADC1));//»ñÈ¡ADCÖØÖÃĞ£×¼¼Ä´æÆ÷µÄ×´Ì¬

    ADC_StartCalibration(ADC1);//¿ªÊ¼Ö¸¶¨ADCµÄĞ£×¼×´Ì¬
    while(ADC_GetCalibrationStatus(ADC1));//»ñÈ¡Ö¸¶¨ADCµÄĞ£×¼³ÌĞò

    ADC_SoftwareStartConvCmd(ADC1, ENABLE);//Ê¹ÄÜ»òÕßÊ§ÄÜÖ¸¶¨µÄADCµÄÈí¼ş×ª»»Æô¶¯¹¦ÄÜ


}


void adc11_init()
{
    GPIO_InitTypeDef GPIO_InitStructure;
    ADC_InitTypeDef ADC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC|RCC_APB2Periph_AFIO|RCC_APB2Periph_ADC1,ENABLE);

    RCC_ADCCLKConfig(RCC_PCLK2_Div6);//12M  ×î´ó14M ÉèÖÃADCÊ±ÖÓ£¨ADCCLK£©

    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_1;//ADC
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AIN; //Ä£ÄâÊäÈë
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_Init(GPIOC,&GPIO_InitStructure);


    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &ADC_InitStructure);

    //ÉèÖÃÖ¸¶¨ADCµÄ¹æÔò×éÍ¨µÀ£¬ÉèÖÃËüÃÇµÄ×ª»¯Ë³ĞòºÍ²ÉÑùÊ±¼ä
    ADC_RegularChannelConfig(ADC1,ADC_Channel_11,1,ADC_SampleTime_239Cycles5);

    ADC_Cmd(ADC1,ENABLE);

    ADC_ResetCalibration(ADC1);//ÖØÖÃÖ¸¶¨µÄADCµÄĞ£×¼¼Ä´æÆ÷
    while(ADC_GetResetCalibrationStatus(ADC1));//»ñÈ¡ADCÖØÖÃĞ£×¼¼Ä´æÆ÷µÄ×´Ì¬

    ADC_StartCalibration(ADC1);//¿ªÊ¼Ö¸¶¨ADCµÄĞ£×¼×´Ì¬
    while(ADC_GetCalibrationStatus(ADC1));//»ñÈ¡Ö¸¶¨ADCµÄĞ£×¼³ÌĞò

    ADC_SoftwareStartConvCmd(ADC1, ENABLE);//Ê¹ÄÜ»òÕßÊ§ÄÜÖ¸¶¨µÄADCµÄÈí¼ş×ª»»Æô¶¯¹¦ÄÜ


}
void adc12_init()
{
    GPIO_InitTypeDef GPIO_InitStructure;
    ADC_InitTypeDef ADC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC|RCC_APB2Periph_AFIO|RCC_APB2Periph_ADC1,ENABLE);

    RCC_ADCCLKConfig(RCC_PCLK2_Div6);//12M  ×î´ó14M ÉèÖÃADCÊ±ÖÓ£¨ADCCLK£©

    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_2;//ADC
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AIN; //Ä£ÄâÊäÈë
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_Init(GPIOC,&GPIO_InitStructure);


    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &ADC_InitStructure);

    //ÉèÖÃÖ¸¶¨ADCµÄ¹æÔò×éÍ¨µÀ£¬ÉèÖÃËüÃÇµÄ×ª»¯Ë³ĞòºÍ²ÉÑùÊ±¼ä
    ADC_RegularChannelConfig(ADC1,ADC_Channel_12,1,ADC_SampleTime_239Cycles5);

    ADC_Cmd(ADC1,ENABLE);

    ADC_ResetCalibration(ADC1);//ÖØÖÃÖ¸¶¨µÄADCµÄĞ£×¼¼Ä´æÆ÷
    while(ADC_GetResetCalibrationStatus(ADC1));//»ñÈ¡ADCÖØÖÃĞ£×¼¼Ä´æÆ÷µÄ×´Ì¬

    ADC_StartCalibration(ADC1);//¿ªÊ¼Ö¸¶¨ADCµÄĞ£×¼×´Ì¬
    while(ADC_GetCalibrationStatus(ADC1));//»ñÈ¡Ö¸¶¨ADCµÄĞ£×¼³ÌĞò

    ADC_SoftwareStartConvCmd(ADC1, ENABLE);//Ê¹ÄÜ»òÕßÊ§ÄÜÖ¸¶¨µÄADCµÄÈí¼ş×ª»»Æô¶¯¹¦ÄÜ


}





/*******************************************************************************
* Function name  : Get_ADC_Value
* Description  : Get the result of ad conversion which concludes three channels to be choosed
* Input : the channel you want get, and the sample times you want  ,get the mean of these times
* Output  :  the ad result 
* Return Value :  ad result(which is related to the reference)
* Attention: we can only choose channel10 11 12 which mean pc0 pc1 pc2

*******************************************************************************/
u16 Get_ADC_Value(u8 channel,u8 times)
{
    u8 t;  //times ÓÃÀ´ÊµÏÖ¶à´Î²ÉÑùµÄÆ½¾ùÖµ~

    u32 temp=0;

    ADC_RegularChannelConfig(ADC1,channel,1,ADC_SampleTime_239Cycles5);
    for (t=0; t<times; t++)
    {
        ADC_SoftwareStartConvCmd(ADC1, ENABLE);// Èí¼ş´¥·¢
        while (!ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC));  //??????   ±¾ÉíÒ²ÓĞÑÓ³Ù£¬Êµ¼ÊÉÏÈç¹û
        temp += ADC_GetConversionValue(ADC1);        //ÓÃÀ´¶ÁÈ¡ÎÒÃÇµÄ×ª»»Öµ¡£
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
    u16 AD_Channel_Select[3]= {ADC_Channel_10,ADC_Channel_11,ADC_Channel_12}; //¶¨ÒåÍ¨µÀÑ¡ÔñÊı×é
    u32 AD_Channel_10_Value,AD_Channel_11_Value,AD_Channel_12_Value;


    AD_Channel_10_Value = Get_ADC_Value(ADC_Channel_10,1);
    AD_Channel_11_Value = Get_ADC_Value(ADC_Channel_11,1);
    AD_Channel_12_Value = Get_ADC_Value(ADC_Channel_12,1);
    voltage[0] = (float)AD_Channel_10_Value*(3.3/4096);
    voltage[1] = (float)AD_Channel_11_Value*(3.3/4096);
    voltage[2] = (float)AD_Channel_12_Value*(3.3/4096);
    //  printf("Voltage0 is %f\r\n",voltage[0]);
//printf("Voltage1 is %f\r\n",voltage[1]);

    //  printf("Voltage2 is %f\r\n",voltage[2]);
    //·µ»Øvoltage Êı×é

    return voltage ;
}


/*******************************************************************************
* Function name  : AD_Conversion_1479A
* Description  : Get the result of AD_Conversion_1479A
* Input : none
* Output  : the float value  of 1479A channel--channel 10--pc0
* Return Value :  ad voltage (which is related to the reference)
* Attention: the reference voltage value is 3.3V

*******************************************************************************/


float AD_Conversion_1479A(times)

{

    float voltage;
    ////////////////////AD part/////////////////////////

    u32 AD_Channel_10_Value,AD_Channel_11_Value,AD_Channel_12_Value;
    ADC_DeInit(ADC1);

    adc10_init();
    AD_Channel_12_Value = Get_ADC_Value(ADC_Channel_12,times);
    voltage = (float)AD_Channel_12_Value*(0.0008056640625);

  //  printf("AD conversion mid  1479A is %f\r\n",voltage);

    //·µ»Øvoltage Êı×é

    return voltage ;
}
/*******************************************************************************
* Function name  : AD_Conversion_627D
* Description  : Get the result of AD_Conversion_627D
* Input : none
* Output  : the float value  of 627D channel--channel 11--pc1
* Return Value :  ad voltage (which is related to the reference)
* Attention: the reference voltage value is 3.3V

*******************************************************************************/


float AD_Conversion_627D(times)
{
    float voltage;
    ////////////////////AD part/////////////////////////

    u32 AD_Channel_10_Value,AD_Channel_11_Value,AD_Channel_12_Value;
    ADC_DeInit(ADC1);
    adc11_init();
    AD_Channel_11_Value = Get_ADC_Value(ADC_Channel_11,times);
    voltage = (float)AD_Channel_11_Value*(0.0008056640625);

    //printf("AD conversion mid  627Dis %f\r\n",voltage);

    //·µ»Øvoltage Êı×é
    return voltage ;
}

/*******************************************************************************
* Function name  : AD_Conversion_025D
* Description  : Get the result of AD_Conversion_025D
* Input : none
* Output  : the float value  of 025D channel--channel 12--pc2
* Return Value :  ad voltage (which is related to the reference)
* Attention: the reference voltage value is 3.3V

*******************************************************************************/

float AD_Conversion_025D(times)
{
    float voltage;
    ////////////////////AD part/////////////////////////

    u32 AD_Channel_10_Value,AD_Channel_11_Value,AD_Channel_12_Value;

    ADC_DeInit(ADC1);
    adc12_init();

    AD_Channel_10_Value = Get_ADC_Value(ADC_Channel_10,times);

    voltage = (float)AD_Channel_10_Value*(0.0008056640625);
    
    //printf("AD conversion mid  025D is %f\r\n",voltage);

    //·µ»Øvoltage Êı×é

    return voltage ;
}

