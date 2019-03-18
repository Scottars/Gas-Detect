/**********************************************************************************
 * ÎÄ¼şÃû  £ºW5500.c
 * ÃèÊö    £ºW5500 Çı¶¯º¯Êı¿â         
 * ¿â°æ±¾  £ºST_v3.5
 * ÌÔ±¦    £ºhttp://yixindianzikeji.taobao.com/
**********************************************************************************/

#include "stm32f10x.h"
#include "stm32f10x_spi.h"				
#include "W5500.h"	
#include <string.h>
#include "led.h"
#include "printf.h" //µ÷ÊÔÓÃ
#include "IOState.h"
#include "adc.h"
#include "DataProcess.h"
#include "stdint.h"





/***************----- ÍøÂç²ÎÊı±äÁ¿¶¨Òå -----***************/
unsigned char Gateway_IP[4];//Íø¹ØIPµØÖ· 
unsigned char Sub_Mask[4];	//×ÓÍøÑÚÂë 
unsigned char Phy_Addr[6];	//ÎïÀíµØÖ·(MAC) 
unsigned char IP_Addr[4];	//±¾»úIPµØÖ· 

unsigned char S0_Port[2];	//¶Ë¿Ú0µÄ¶Ë¿ÚºÅ(5000) 
unsigned char S0_DIP[4];	//¶Ë¿Ú0Ä¿µÄIPµØÖ· 
unsigned char S0_DPort[2];	//¶Ë¿Ú0Ä¿µÄ¶Ë¿ÚºÅ(6000) 

unsigned char UDP_DIPR[4];	//UDP(¹ã²¥)Ä£Ê½,Ä¿µÄÖ÷»úIPµØÖ·
unsigned char UDP_DPORT[2];	//UDP(¹ã²¥)Ä£Ê½,Ä¿µÄÖ÷»ú¶Ë¿ÚºÅ

/***************----- ¶Ë¿ÚµÄÔËĞĞÄ£Ê½ -----***************/
unsigned char S0_Mode =3;	//¶Ë¿Ú0µÄÔËĞĞÄ£Ê½,0:TCP·şÎñÆ÷Ä£Ê½,1:TCP¿Í»§¶ËÄ£Ê½,2:UDP(¹ã²¥)Ä£Ê½
#define TCP_SERVER	0x00	//TCP·şÎñÆ÷Ä£Ê½
#define TCP_CLIENT	0x01	//TCP¿Í»§¶ËÄ£Ê½ 
#define UDP_MODE	0x02	//UDP(¹ã²¥)Ä£Ê½ 

/***************----- ¶Ë¿ÚµÄÔËĞĞ×´Ì¬ -----***************/
unsigned char S0_State =0;	//¶Ë¿Ú0×´Ì¬¼ÇÂ¼,1:¶Ë¿ÚÍê³É³õÊ¼»¯,2¶Ë¿ÚÍê³ÉÁ¬½Ó(¿ÉÒÔÕı³£´«ÊäÊı¾İ) 
#define S_INIT		0x01	//¶Ë¿ÚÍê³É³õÊ¼»¯ 
#define S_CONN		0x02	//¶Ë¿ÚÍê³ÉÁ¬½Ó,¿ÉÒÔÕı³£´«ÊäÊı¾İ 

/***************----- ¶Ë¿ÚÊÕ·¢Êı¾İµÄ×´Ì¬ -----***************/
unsigned char S0_Data;		//¶Ë¿Ú0½ÓÊÕºÍ·¢ËÍÊı¾İµÄ×´Ì¬,1:¶Ë¿Ú½ÓÊÕµ½Êı¾İ,2:¶Ë¿Ú·¢ËÍÊı¾İÍê³É 
#define S_RECEIVE	 0x01	//¶Ë¿Ú½ÓÊÕµ½Ò»¸öÊı¾İ°ü 
#define S_TRANSMITOK 0x02	//¶Ë¿Ú·¢ËÍÒ»¸öÊı¾İ°üÍê³É 

/***************----- ¶Ë¿ÚÊı¾İ»º³åÇø -----***************/
unsigned char Rx_Buffer[2048];	//¶Ë¿Ú½ÓÊÕÊı¾İ»º³åÇø 
unsigned char Tx_Buffer[2048];	//¶Ë¿Ú·¢ËÍÊı¾İ»º³åÇø 

unsigned char W5500_Interrupt;	//W5500ÖĞ¶Ï±êÖ¾(0:ÎŞÖĞ¶Ï,1:ÓĞÖĞ¶Ï)


void RCC_Configuration(void);		//ÉèÖÃÏµÍ³Ê±ÖÓÎª72MHZ(Õâ¸ö¿ÉÒÔ¸ù¾İĞèÒª¸Ä)
void NVIC_Configuration(void);		//STM32ÖĞ¶ÏÏòÁ¿±íÅäÅäÖÃ
void Timer2_Init_Config(void);		//Timer2³õÊ¼»¯ÅäÖÃ
void System_Initialization(void);	//STM32ÏµÍ³³õÊ¼»¯º¯Êı(³õÊ¼»¯STM32Ê±ÖÓ¼°ÍâÉè)
void Delay(unsigned int d);			//ÑÓÊ±º¯Êı(ms)

unsigned int Timer2_Counter=0; //Timer2¶¨Ê±Æ÷¼ÆÊı±äÁ¿(ms)
unsigned int W5500_Send_Delay_Counter=0; //W5500·¢ËÍÑÓÊ±¼ÆÊı±äÁ¿(ms)




//////////////////////////////ÏÂÃæÊÇÊ¾Àı³ÌĞòÀ´×ÔÖ÷³ÌĞòµÄ²¿·Ö///////////////////////


/*******************************************************************************
* º¯ÊıÃû  : RCC_Configuration
* ÃèÊö    : ÉèÖÃÏµÍ³Ê±ÖÓÎª72MHZ(Õâ¸ö¿ÉÒÔ¸ù¾İĞèÒª¸Ä)
* ÊäÈë    : ÎŞ
* Êä³ö    : ÎŞ
* ·µ»ØÖµ  : ÎŞ
* ËµÃ÷    : STM32F107xºÍSTM32F105xÏµÁĞMCUÓëSTM32F103xÏµÁĞMCUÊ±ÖÓÅäÖÃÓĞËù²»Í¬
*******************************************************************************/
void RCC_Configuration(void)
{
  ErrorStatus HSEStartUpStatus;               //Íâ²¿¸ßËÙÊ±ÖÓ(HSE)µÄ¹¤×÷×´Ì¬±äÁ¿
  
  RCC_DeInit();                               //½«ËùÓĞÓëÊ±ÖÓÏà¹ØµÄ¼Ä´æÆ÷ÉèÖÃÎªÄ¬ÈÏÖµ
  RCC_HSEConfig(RCC_HSE_ON);                  //Æô¶¯Íâ²¿¸ßËÙÊ±ÖÓHSE 
  HSEStartUpStatus = RCC_WaitForHSEStartUp(); //µÈ´ıÍâ²¿¸ßËÙÊ±ÖÓ(HSE)ÎÈ¶¨

  if(SUCCESS == HSEStartUpStatus)             //Èç¹ûÍâ²¿¸ßËÙÊ±ÖÓÒÑ¾­ÎÈ¶¨
  {
    /* Enable Prefetch Buffer */
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable); //FlashÉèÖÃ
    /* Flash 2 wait state */
    FLASH_SetLatency(FLASH_Latency_2);
    
  
    RCC_HCLKConfig(RCC_SYSCLK_Div1); //ÉèÖÃAHBÊ±ÖÓµÈÓÚÏµÍ³Ê±ÖÓ(1·ÖÆµ)/72MHZ
    RCC_PCLK2Config(RCC_HCLK_Div1);  //ÉèÖÃAPB2Ê±ÖÓºÍHCLKÊ±ÖÓÏàµÈ/72MHz(×î´óÎª72MHz)
    RCC_PCLK1Config(RCC_HCLK_Div2);  //ÉèÖÃAPB1Ê±ÖÓÊÇHCLKÊ±ÖÓµÄ2·ÖÆµ/36MHz(×î´óÎª36MHz)
  
#ifndef STM32F10X_CL                 //Èç¹ûÊ¹ÓÃµÄ²»ÊÇSTM32F107x»òSTM32F105xÏµÁĞMCU,PLLÒÔÏÂÅäÖÃ  
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9); //PLLCLK = 8MHz * 9 = 72 MHz 
#else                                //Èç¹ûÊ¹ÓÃµÄÊÇSTM32F107x»òSTM32F105xÏµÁĞMCU,PLLÒÔÏÂÅäÖÃ
    /***** ÅäÖÃPLLx *****/
    /* PLL2 configuration: PLL2CLK = (HSE / 5) * 8 = 40 MHz */
    RCC_PREDIV2Config(RCC_PREDIV2_Div5);
    RCC_PLL2Config(RCC_PLL2Mul_8);

    RCC_PLL2Cmd(ENABLE); //Ê¹ÄÜPLL2 
    while (RCC_GetFlagStatus(RCC_FLAG_PLL2RDY) == RESET);//µÈ´ıPLL2ÎÈ¶¨

    /* PLL configuration: PLLCLK = (PLL2 / 5) * 9 = 72 MHz */ 
    RCC_PREDIV1Config(RCC_PREDIV1_Source_PLL2, RCC_PREDIV1_Div5);
    RCC_PLLConfig(RCC_PLLSource_PREDIV1, RCC_PLLMul_9);
#endif

    RCC_PLLCmd(ENABLE); //Ê¹ÄÜPLL
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET); //µÈ´ıPLLÎÈ¶¨

    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);          //ÉèÖÃÏµÍ³Ê±ÖÓµÄÊ±ÖÓÔ´ÎªPLL

    while(RCC_GetSYSCLKSource() != 0x08);               //¼ì²éÏµÍ³µÄÊ±ÖÓÔ´ÊÇ·ñÊÇPLL
    RCC_ClockSecuritySystemCmd(ENABLE);                 //Ê¹ÄÜÏµÍ³°²È«Ê±ÖÓ 

	/* Enable peripheral clocks --------------------------------------------------*/
  	/* Enable I2C1 and I2C1 clock */
 	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

  	/* Enable GPIOA GPIOB SPI1 and USART1 clocks */
  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB
					| RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD
					| RCC_APB2Periph_AFIO, ENABLE);    
  }
}

/*******************************************************************************
* º¯ÊıÃû  : NVIC_Configuration
* ÃèÊö    : STM32ÖĞ¶ÏÏòÁ¿±íÅäÅäÖÃ
* ÊäÈë    : ÎŞ
* Êä³ö    : ÎŞ
* ·µ»ØÖµ  : ÎŞ
* ËµÃ÷    : ÉèÖÃKEY1(PC11)µÄÖĞ¶ÏÓÅÏÈ×é
*******************************************************************************/
void NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;						//¶¨ÒåNVIC³õÊ¼»¯½á¹¹Ìå

  	/* Set the Vector Table base location at 0x08000000 */
  	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);				//ÉèÖÃÖĞ¶ÏÓÅÏÈ¼¶×éÎª1£¬ÓÅÏÈ×é(¿ÉÉè0¡«4Î»)
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;				//ÉèÖÃÖĞ¶ÏÏòÁ¿ºÅ
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	//ÉèÖÃÇÀÏÈÓÅÏÈ¼¶
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;			//ÉèÖÃÏìÓ¦ÓÅÏÈ¼¶
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				//Ê¹ÄÜNVIC
	NVIC_Init(&NVIC_InitStructure);

	W5500_NVIC_Configuration();	//W5500 ½ÓÊÕÒı½ÅÖĞ¶ÏÓÅÏÈ¼¶ÉèÖÃ
}

/*******************************************************************************
* º¯ÊıÃû  : Timer2_Init_Config
* ÃèÊö    : Timer2³õÊ¼»¯ÅäÖÃ
* ÊäÈë    : ÎŞ
* Êä³ö    : ÎŞ
* ·µ»Ø    : ÎŞ 
* ËµÃ÷    : ÎŞ
*******************************************************************************/
void Timer2_Init_Config(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);		//Ê¹ÄÜTimer2Ê±ÖÓ
	
	TIM_TimeBaseStructure.TIM_Period = 9;						//ÉèÖÃÔÚÏÂÒ»¸ö¸üĞÂÊÂ¼ş×°Èë»î¶¯µÄ×Ô¶¯ÖØ×°ÔØ¼Ä´æÆ÷ÖÜÆÚµÄÖµ(¼ÆÊıµ½10Îª1ms)
	TIM_TimeBaseStructure.TIM_Prescaler = 7199;					//ÉèÖÃÓÃÀ´×÷ÎªTIMxÊ±ÖÓÆµÂÊ³ıÊıµÄÔ¤·ÖÆµÖµ(10KHzµÄ¼ÆÊıÆµÂÊ)
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;		//ÉèÖÃÊ±ÖÓ·Ö¸î:TDTS = TIM_CKD_DIV1
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;	//TIMÏòÉÏ¼ÆÊıÄ£Ê½
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);				//¸ù¾İTIM_TimeBaseInitStructÖĞÖ¸¶¨µÄ²ÎÊı³õÊ¼»¯TIMxµÄÊ±¼ä»ùÊıµ¥Î»
	 
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE ); 				//Ê¹ÄÜTIM2Ö¸¶¨µÄÖĞ¶Ï
	
	TIM_Cmd(TIM2, ENABLE);  									//Ê¹ÄÜTIMxÍâÉè
}

/*******************************************************************************
* º¯ÊıÃû  : TIM2_IRQHandler
* ÃèÊö    : ¶¨Ê±Æ÷2ÖĞ¶Ï¶Ï·şÎñº¯Êı
* ÊäÈë    : ÎŞ
* Êä³ö    : ÎŞ
* ·µ»Ø    : ÎŞ 
* ËµÃ÷    : ÎŞ
*******************************************************************************/
void TIM2_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
		Timer2_Counter++;
		W5500_Send_Delay_Counter++;
	}
}

/*******************************************************************************
* º¯ÊıÃû  : System_Initialization
* ÃèÊö    : STM32ÏµÍ³³õÊ¼»¯º¯Êı(³õÊ¼»¯STM32Ê±ÖÓ¼°ÍâÉè)
* ÊäÈë    : ÎŞ
* Êä³ö    : ÎŞ
* ·µ»Ø    : ÎŞ 
* ËµÃ÷    : ÎŞ
*******************************************************************************/
void System_Initialization(void)
{
	RCC_Configuration();		//ÉèÖÃÏµÍ³Ê±ÖÓÎª72MHZ(Õâ¸ö¿ÉÒÔ¸ù¾İĞèÒª¸Ä)
  	NVIC_Configuration();		//STM32ÖĞ¶ÏÏòÁ¿±íÅäÅäÖÃ
	SPI_Configuration();		//W5500 SPI³õÊ¼»¯ÅäÖÃ(STM32 SPI1)
	Timer2_Init_Config();		//Timer2³õÊ¼»¯ÅäÖÃ
	W5500_GPIO_Configuration();	//W5500 GPIO³õÊ¼»¯ÅäÖÃ	
}

/*******************************************************************************
* º¯ÊıÃû  : Delay
* ÃèÊö    : ÑÓÊ±º¯Êı(ms)
* ÊäÈë    : d:ÑÓÊ±ÏµÊı£¬µ¥Î»ÎªºÁÃë
* Êä³ö    : ÎŞ
* ·µ»Ø    : ÎŞ 
* ËµÃ÷    : ÑÓÊ±ÊÇÀûÓÃTimer2¶¨Ê±Æ÷²úÉúµÄ1ºÁÃëµÄ¼ÆÊıÀ´ÊµÏÖµÄ
*******************************************************************************/

void Delay(unsigned int d)
{
	Timer2_Counter=0; 
	while(Timer2_Counter < d);
}

		




/*******************************************************************************
* º¯ÊıÃû  : W5500_Initialization
* ÃèÊö    : W5500³õÊ¼»õÅäÖÃ
* ÊäÈë    : ÎŞ
* Êä³ö    : ÎŞ
* ·µ»ØÖµ  : ÎŞ
* ËµÃ÷    : ÎŞ
*******************************************************************************/
void W5500_Initialization(void)
{
	W5500_Init();		//³õÊ¼»¯W5500¼Ä´æÆ÷º¯Êı
	Detect_Gateway();	//¼ì²éÍø¹Ø·şÎñÆ÷ 
	Socket_Init(0);		//Ö¸¶¨Socket(0~7)³õÊ¼»¯,³õÊ¼»¯¶Ë¿Ú0
	//	LED0=1;/////////////////////¸øÁËÒ»¸öĞÅºÅÓÃÀ´ÊµÏÖÈ·¶¨ÊÇ·ñÒÑ¾­Íê³ÉÁË³õÊ¼»¯///////////////////////////////////////////////////////////

}

/*******************************************************************************
* º¯ÊıÃû  : Load_Net_Parameters
* ÃèÊö    : ×°ÔØÍøÂç²ÎÊı
* ÊäÈë    : ÎŞ
* Êä³ö    : ÎŞ
* ·µ»ØÖµ  : ÎŞ
* ËµÃ÷    : Íø¹Ø¡¢ÑÚÂë¡¢ÎïÀíµØÖ·¡¢±¾»úIPµØÖ·¡¢¶Ë¿ÚºÅ¡¢Ä¿µÄIPµØÖ·¡¢Ä¿µÄ¶Ë¿ÚºÅ¡¢¶Ë¿Ú¹¤×÷Ä£Ê½
*******************************************************************************/
void Load_Net_Parameters(void)
{
	Gateway_IP[0] = 192;//¼ÓÔØÍø¹Ø²ÎÊı
	Gateway_IP[1] = 168;
	Gateway_IP[2] = 1;
	Gateway_IP[3] = 1;

	Sub_Mask[0]=255;//¼ÓÔØ×ÓÍøÑÚÂë
	Sub_Mask[1]=255;
	Sub_Mask[2]=255;
	Sub_Mask[3]=0;

	Phy_Addr[0]=0x0c;//¼ÓÔØÎïÀíµØÖ·
	Phy_Addr[1]=0x29;
	Phy_Addr[2]=0xab;
	Phy_Addr[3]=0x7c;
	Phy_Addr[4]=0x00;
	Phy_Addr[5]=0x01;

	IP_Addr[0]=192;//¼ÓÔØ±¾»úIPµØÖ·
	IP_Addr[1]=168;
	IP_Addr[2]=1;
	IP_Addr[3]=198;

	S0_Port[0] = 0x13;//¼ÓÔØ¶Ë¿Ú0µÄ¶Ë¿ÚºÅ5000 
	S0_Port[1] = 0x88;

//	S0_DIP[0]=192;//¼ÓÔØ¶Ë¿Ú0µÄÄ¿µÄIPµØÖ·
//	S0_DIP[1]=168;
//	S0_DIP[2]=1;
//	S0_DIP[3]=190;
//	
//	S0_DPort[0] = 0x17;//¼ÓÔØ¶Ë¿Ú0µÄÄ¿µÄ¶Ë¿ÚºÅ6000
//	S0_DPort[1] = 0x70;

	S0_Mode=TCP_SERVER;//¼ÓÔØ¶Ë¿Ú0µÄ¹¤×÷Ä£Ê½,TCP·şÎñ¶ËÄ£Ê½
}

/*******************************************************************************
* º¯ÊıÃû  : W5500_Socket_Set
* ÃèÊö    : W5500¶Ë¿Ú³õÊ¼»¯ÅäÖÃ
* ÊäÈë    : ÎŞ
* Êä³ö    : ÎŞ
* ·µ»ØÖµ  : ÎŞ
* ËµÃ÷    : ·Ö±ğÉèÖÃ4¸ö¶Ë¿Ú,¸ù¾İ¶Ë¿Ú¹¤×÷Ä£Ê½,½«¶Ë¿ÚÖÃÓÚTCP·şÎñÆ÷¡¢TCP¿Í»§¶Ë»òUDPÄ£Ê½.
*			´Ó¶Ë¿Ú×´Ì¬×Ö½ÚSocket_State¿ÉÒÔÅĞ¶Ï¶Ë¿ÚµÄ¹¤×÷Çé¿ö
*******************************************************************************/
void W5500_Socket_Set(void)
{
	if(S0_State==0)//¶Ë¿Ú0³õÊ¼»¯ÅäÖÃ
	{
		if(S0_Mode==TCP_SERVER)//TCP·şÎñÆ÷Ä£Ê½ 
		{
			if(Socket_Listen(0)==TRUE)
				S0_State=S_INIT;
			else
				S0_State=0;
		}
		else if(S0_Mode==TCP_CLIENT)//TCP¿Í»§¶ËÄ£Ê½ 
		{
			if(Socket_Connect(0)==TRUE)
				S0_State=S_INIT;
			else
				S0_State=0;
		}
		else//UDPÄ£Ê½ 
		{
			if(Socket_UDP(0)==TRUE)
				S0_State=S_INIT|S_CONN;
			else
				S0_State=0;
		}
	}
}

/*******************************************************************************
* º¯ÊıÃû  : Process_Socket_Data
* ÃèÊö    : W5500½ÓÊÕ²¢·¢ËÍ½ÓÊÕµ½µÄÊı¾İ
* ÊäÈë    : s:¶Ë¿ÚºÅ
* Êä³ö    : ÎŞ
* ·µ»ØÖµ  : ÎŞ
* ËµÃ÷    : ±¾¹ı³ÌÏÈµ÷ÓÃS_rx_process()´ÓW5500µÄ¶Ë¿Ú½ÓÊÕÊı¾İ»º³åÇø¶ÁÈ¡Êı¾İ,
*			È»ºó½«¶ÁÈ¡µÄÊı¾İ´ÓRx_Buffer¿½±´µ½Temp_Buffer»º³åÇø½øĞĞ´¦Àí¡£
*			´¦ÀíÍê±Ï£¬½«Êı¾İ´ÓTemp_Buffer¿½±´µ½Tx_Buffer»º³åÇø¡£µ÷ÓÃS_tx_process()
*			·¢ËÍÊı¾İ¡£
*******************************************************************************/
typedef union
{
float floatData;
unsigned char byteData[4];
//	uint32_t byteData;
}FLOAT_BYTE;
void Process_Socket_Data(SOCKET s)
{
	
	FLOAT_BYTE testdata;
	char floattest[10];
	 char test1[4];
	unsigned short size;//½ÓÊÕµ½µÄbuffer µÄ´óĞ¡
		///////////////////////Valve Gas Part///////	

	u8 ValveValue_Set[12];  //À´×ÔÍøÂç´«Èë¹ıÀ´µÄĞÅºÅ
	char *ValveValue_Status;
	u8 i;
	////////////////////////////////////////////////////////
	////////////////Õæ¿ÕÇ»²¿·Ö/////////////////////////////
	float VacuumValue_Set;
	
	
	//////////////////Á÷Á¿¼ÆµÄÁ÷Á¿ÖµÉè¶¨Éè¶¨//////////////////////////////////
	float Flow1479AValue_Set;
	
	
	
	////////////////ÆøÌåÅç³ö·åÖµÕæ¿Õ¶ÈÉè¶¨//////////////////////////////////
	float GasPuffValue_Set;
	
	
	////////////////PID²ÎÊıÉè¶¨/////////////////////////
	float Kp,Ki,Kd;
	
	
	
	
	
	
	
	///////////////////AD×ª»»²¿·Ö////////////////
	float *AD_Voltage_Status; 
	float temp=0; // ÓÃÀ´½øĞĞAD×ª»»Ê¹ÓÃµÄ±äÁ¿
	uint8_t temp1[5];
	char AD_Value[50];
	
	
	
	
	
	
	size=Read_SOCK_Data_Buffer(s, Rx_Buffer);
	//printf("\r\nSIZE:%d\r\n",size);
	memcpy(Tx_Buffer, Rx_Buffer, size);			
//	printf("\r\nRX_BUFFER\r\n");
	printf(Rx_Buffer);

	//¹ØÓÚRX_Buffer  ÉÏÎ»»úÀíÂÛÉÏ Ó¦¸Ã´«ÈëµÄÊÇÒ»´®µÄÊı¾İ£¬ ¶ø²»ÊÇÒ»¸öµ¥´¿µÄÎ»
	//¿ÉÒÔ¿´µ½Rx_Buffer Ïàµ±ÓÚÒ»¸öÊı×é£¬16Î»°É
	//¶ÔÓÚModbusĞ­Òé ÏÈ¾Ù¸öÀı×Ó
	/* 
	¶ÁÊı¾İ£º    04               03                   02 																							ÏÂÎ»»ú£º»Ø¸´ÆäĞèÒª¶ÁµÄĞÅÏ¢ 04 03 02 length data1 data2 *** crc
					´Ó»úµØÖ·          ¹¦ÄÜÂë¶Á						¼Ä´æÆ÷µØÖ·  ---------------------------------------¡·                
	Ğ´Êı¾İ£º    04               06                   02             data1  data2 ***  crc16     			ÏÂÎ»»ú£º»Ø¸´ÆäĞèÒªĞ´µÄĞÅÏ¢ 04 06 02  ****     £¨PS£¬Õâ¸öµØ·½»Ø¸´µÄÊÇÓëÉÏÎ»»úÏàÍ¬µÄ¶«Î÷£©     
					´Ó»úµØÖ·          ¹¦ÄÜÂëĞ´						¼Ä´æÆ÷µØÖ·
ÖÃÎ»On/off£º  04               05                   02              data1 data2 ***  crc16					ÏÂÎ»»ú£º	Õâ¸öÊÇ·ñĞèÒª·µ»ØÊı¾İ£¿
					´Ó»úµØÖ·          ¹¦ÄÜÂëÖÃÎ»					¼Ä´æÆ÷µØÖ·      
	±¨´íĞÅÏ¢£º  04               04                  02              data1 data2 ***  crc16            Õâ¸öÊÇÏÂÎ»»ú¶ÔÉÏÎ»»úµÄ·¢ËÍµÄÊı¾İÓĞ²»Í¬Òâ¼û²úÉúµÄ´íÎó
					´Ó»úµØÖ·          ¹¦ÄÜÂëÖÃÎ»					¼Ä´æÆ÷µØÖ· 			
	
					
	*/
	
	
	if (Rx_Buffer[0]==0x04) //±¾»úµÄÉè±¸µØÖ·Îª0x04 
	{
	//	printf("\r\nSLocal Address ok!\r\n");
			
		switch (Rx_Buffer[1])
		{
			case 0x03:    //¶ÁÈ¡¹¦ÄÜÂë¼Ä´æÆ÷µÄ×´Ì¬ --ÊÇ·ñÎª¶ÁÊı¾İ¹¦ÄÜÂë
				 switch  (Rx_Buffer[2])
				 {
					 
				 		 
					case 0x01: //627DÕæ¿Õ¶È¼ì²â£¬Õâ¸öÈç¹ûÉÏÎ»»ú·¢ÁËÃüÁî£¬ÎÒ»¹ÊÇÒª´«Ò»·İÊı¾İ»ØÈ¥Âğ£¿
						Tx_Buffer[0]=0x04; // ±¾»úµØÖ·
						Tx_Buffer[1]=0x03;//¹¦ÄÜÃüÁîÂë
						Tx_Buffer[2]=0x01;//¼Ä´æÆ÷µØÖ	
					
				   	Write_SOCK_Data_Buffer(s, Tx_Buffer, 7);
					case 0x02: //025DÕæ¿Õ¶È¼ì²â£¬Õâ¸öÈç¹ûÉÏÎ»»ú·¢ÁËÃüÁî£¬ÎÒ»¹ÊÇÒª´«Ò»·İÊı¾İ»ØÈ¥Âğ£¿
						Tx_Buffer[0]=0x04; // ±¾»úµØÖ·
						Tx_Buffer[1]=0x03;//¹¦ÄÜÃüÁîÂë
						Tx_Buffer[2]=0x02;//¼Ä´æÆ÷µØÖ
					
					
					
					
						Write_SOCK_Data_Buffer(s, Tx_Buffer, 7);
					
					case 0x03://ÅäÆø¹ñ·§ÃÅ×´Ì¬¶ÁÈ¡
						Tx_Buffer[0]=0x04; // ±¾»úµØÖ·
						Tx_Buffer[1]=0x03;//¹¦ÄÜÃüÁîÂë
						Tx_Buffer[2]=0x03;//¼Ä´æÆ÷µØÖ·	
						//¶ÁÈ¡ÅäÆø¹ñ·§ÃÅ×´Ì¬º¯Êı
						ValveValue_Status=Gas_State_Read();
						Tx_Buffer[3]=ValveValue_Status[0];
						Tx_Buffer[4]=ValveValue_Status[1];
						//size=3;
						Write_SOCK_Data_Buffer(s, Tx_Buffer, 5);
						break;
					case 0x04:
						Tx_Buffer[0]=0x04; // ±¾»úµØÖ·
						Tx_Buffer[1]=0x03;//¹¦ÄÜÃüÁîÂë
						Tx_Buffer[2]=0x04;//¼Ä´æÆ÷µØÖ
						//¶ÁÈ¡ÆøÌåÁ÷Á¿µÄµ±Ç°Öµ£¬´ÓĞÂ¼ÆËãÒ»´Î´«ÈëÉÏÈ¥£¬»¹ÊÇÖ±½ÓÉÏ´«µ±Ç°Öµ
						AD_Voltage_Status = AD_Conversion();
						//AD_Voltage_Status[0 1 2]   ·Ö±ğ±íÊ¾1479A 627D 025d
						//translation
					//	temp	= (int)(AD_Voltage_Status[0] * 100);
						//¶ÔµÃµ½µÄµçÑ¹½øĞĞÁ÷Á¿Öµ½øĞĞ×ª»»,²ÉÓÃIEEE 754 ±ê×¼½øĞĞ×ª»»
						AD_Voltage_Status[0]=2.5;
						temp = GasFloatValue_Calc(AD_Voltage_Status[0]);
						/*printf("%x", temp1[0]);
						printf("%x",temp1[1]);
						printf("%x",temp1[2]);
						printf("%x",temp1[3]);*/
						testdata.floatData=-12.5;
					
						float2Hex(temp1,temp);	
						//sprintf(AD_Value,"%x",*temp1);
					 // sprintf(floattest,"%x",testdata.byteData);
			
					
						Tx_Buffer[3]=testdata.byteData[3];
						
						
						
						
						Tx_Buffer[4]=testdata.byteData[2];
						Tx_Buffer[5]=testdata.byteData[1];
				  	Tx_Buffer[6]=testdata.byteData[0];
					
					
			
					
					
					
						//size=3;
						Write_SOCK_Data_Buffer(s, Tx_Buffer, 7);
						break;
					
				
					
					//Write_SOCK_Data_Buffer(s, Tx_Buffer, 4);
					break;
				 
				 
				 
				 }
					
			
			case 0x05:    //Ğ´Èë¼Ä´æÆ÷µÄ×´Ì¬
				//Éè¶¨12¹âÂ·¡¢Éè¶¨Õæ¿Õ¶È¡¢Éè¶¨Á÷Á¿Öµ
				//Éè¶¨·½°¸£ºÍ¨¹ıÈ«¾Ö±äÁ¿´«ËÍ³öÈ¥£¬»¹ÊÇ×îºóÖ±½Óµ÷ÓÃÏà¹ØµÄº¯ÊıÄØ£¿
				//Ôİ¶¨·½°¸£º Ö±½Óµ÷ÓÃÏà¹ØµÄº¯Êı	
				//Õâ¸öÊÇRx_BufferµÄ¶¨Òå£ºunsigned char Rx_Buffer[2048]

	
			////////////////////¹©ÆøÄ£Ê½////////////////////
			/*********************************************
			*¹©ÆøÄ£Ê½µÃÑ¡ÔñÓĞ2ÖÖ
			* Ä£Ê½1£ºÉÏÎ»»úÖ±½Ó¿ØÖÆÁ÷Á¿¼Æ £¬ Ã¿´Î¸øÁ÷Á¿¼ÆÉè¶¨µ±Ç°µÃÊıÖµ
			* Ä£Ê½2£ºÉÏÎ»»ú²»¿ØÖÆÁ÷Á¿£¬ÈÃÆäÍ¨¹ı×Ô¶¯µÃ±Õ»·½øĞĞ¿ØÖÆ
			*ÊµÏÖµÃ·½°¸
			*
			*
			*********************************************/
				 switch  (Rx_Buffer[2])
				 {
					 
				 		 
					case 0x01: 
		         ////////////////////ÅäÆø¹ñ·§ÃÅ////////////////////
						printf("\r\SetValue ok\r\n");
						for(i=0;i<8;i++)
						{
							if(((Rx_Buffer[4]>>i)& 0x01)==0x01)   //Éè¶¨ÏòÓÒÒÆÎ»£¬´ÓÓÒµ½×ó£¬·Ö±ğ±íÊ¾12Â·¹âÏËµÄ×´Ì¬   ÒÀ´ÎÈ¡Éè¶¨µÄÊıÖµ
								ValveValue_Set[i]=1;
							else
								ValveValue_Set[i]=0;
						}
						for(i=0;i<4;i++)
						{
							if(((Rx_Buffer[3]>>i)& 0x01)==0x01)
								ValveValue_Set[i+8]=1;
							else
								ValveValue_Set[i+8]=0;
						}
					
						//²âÊÔÊÇ·ñ½ÓÊÕ³É¹¦
						//²âÊÔ·½·¨£ºÍ¨¹ı½ÓÊÕÍøÂç´«ËÍ¹ıÀ´µÄÊı¾İ£¬È»ºóÍ¨¹ı´®¿Ú·¢³öÉè¶¨Öµ£¬¿´ÊÇ·ñÕıÈ·
						for(i=0;i<12;i++)
						{
							printf("Setvalue:%d\r\n",ValveValue_Set[i]);
						}
				
				
					
					ValveStateChange(ValveValue_Set);//ÍøÂçĞÅºÅ´«Èë¹ıÀ´µÄ¿ª¶ÏĞÅÏ¢£¬È»ºó³ÌĞòÊµÏÖ×Ô¶¯ÉèÖÃ¶ÔÓ¦µÄIOµÄ¸ßµÍµçÆ½£¬Êµ¼Ê¸Ä±äIO¿ÚÉè¶¨Öµ
					break;
						
					case 0x02:  //ÆøÌåÅç³öÄ£Ê½µÄÆôÍ£  µ±ÆøÌåÂö³å·åÖµÉè¶¨ºÃÁËÖ®ºó£¬ÒÔ¼°´¥·¢·½Ê½Éè¶¨ºÃÁËÖ®ºó£¬Í¨¹ıÕâ¸ö°´¼üÀ´½øĞĞÆô¶¯Í£Ö¹
						if(Rx_Buffer[3]==0x00) //±íÊ¾¹Ø±Õµ±Ç°Åç³öÄ£Ê½ x
						{
							
						}
						if(Rx_Buffer[3]==0xff) //±íÊ¾¿ªÆôµ±Ç°µÄÅç³öÄ£Ê½---Í¬ÑùÊÇ¸ù¾İÉè¶¨µÄÄ¿±êµÄÕæ¿Õ¶È½øĞĞ±Õ»·µÄµ÷½Ú£¬Ö»²»¹ıµ÷½ÚµÄÊÇÓĞÒ»¸ö·åÖµµÄ
						{
							
						}
		

							break;
				 
				 
				 
				 }
			
			case 0x06:
					 switch(Rx_Buffer[2])
					 {
						 	case 0x01: //Õæ¿ÕÇ»Õæ¿Õ¶ÈÉèÖÃ
								
									//Õâ¸öÊ±ºò£¬¶ÔÉÏÎ»»ú´«¹ıÀ´µÄbuffer 3 4 5 6 ÖĞµÄÊı¾İ½øĞĞ´¦Àí£¬´¦ÀíµÄÇ°ÌáÊÇ½øĞĞcrc Ğ£Ñé 
							
							
								
							
							//VacuumValue_Set=
						 
							break;
						 case 0x02:  //¹©ÆøÄ£Ê½Ñ¡Ôñ
								if(Rx_Buffer[3]=0x01) //Ñ¡Ôñ²ÉÓÃÁ÷Á¿¿ØÖÆÄ£Ê½  ÔÚÕâ¸öÄ£Ê½ÏÂ£¬ÎÒÃÇĞèÒª½«ÎÒÃÇÉè¶¨µÄÑ¹µç·§µÄ¿ª¶ÈÉè¶¨µ½×î´óÈ¥
								{
									
								}
								if(Rx_Buffer[3]=0x02) //Ñ¡Ôñ²ÉÓÃ×Ô¶¯±Õ»·¿ØÖÆÄ£Ê½
								{
									
									
								}
						 
						 
						 
								break;
						 case 0x03://1479AÁ÷Á¿ÆäÁ÷Á¿Ä¿±êÉè¶¨
							 	//Õâ¸öÊ±ºò£¬¶ÔÉÏÎ»»ú´«¹ıÀ´µÄbuffer 3 4 5 6 ÖĞµÄÊı¾İ½øĞĞ´¦Àí£¬´¦ÀíµÄÇ°ÌáÊÇ½øĞĞcrc Ğ£Ñé Í¨¹ı
							
							
								
							
							//1479AFloatValue_Set=
						 
						 
						 
							 break;
						 case 0x04: //ÆøÌåÂö³å·åÖµÉè¶¨
							 
						 	 	//Õâ¸öÊ±ºò£¬¶ÔÉÏÎ»»ú´«¹ıÀ´µÄbuffer 3 4 5 6 ÖĞµÄÊı¾İ½øĞĞ´¦Àí£¬´¦ÀíµÄÇ°ÌáÊÇ½øĞĞcrc Ğ£Ñé Í¨¹ı
							
							
								
							
							//GasPuffValue_Set=
						 
						 
						 
							 break;
						 case 0x05://PID²ÎÊıµÄÉè¶¨
								//Õâ¸öÊ±ºò£¬¶ÔÉÏÎ»»ú´«¹ıÀ´µÄbuffer 3 4 5 6    7 8 9 10  11 12 13 14 ÖĞµÄÊı¾İ½øĞĞ´¦Àí£¬´¦ÀíµÄÇ°ÌáÊÇ½øĞĞcrc Ğ£Ñé Í¨¹ı
							
							
								
						
							//=
						 
							 break;
						 case 0x06: //Âö³åÄ£Ê½µÄ´¥·¢µÄ·½Ê½
							if(Rx_Buffer[3]=0x01) //Ñ¡Ôñ²ÉÓÃÉÏÎ»»ú´¥·¢·½Ê½  ÔÚÕâ¸öÄ£Ê½ÏÂ£¬Éè¶¨ĞÂµÄÊıÖµ¸øÎÒÃÇµÄÉè¶¨µÄÕæ¿Õ¶ÈµÄÖµ
								{
									
								}
								if(Rx_Buffer[3]=0x02) //Ñ¡ÔñĞÅºÅ´¥·¢·½Ê½£¬ÔÚÕâ¸öÄ£Ê½ÏÂ£¬Éè¶¨µÄĞÂµÄÊıÖµÔÚÊ±ÖÓĞÅºÅÀ´ÁËµÄÊ±ºò£¬ÎÒÃÇ½«Õâ¸öĞÂµÄÕæ¿Õ¶ÈµÄÊıÖµĞ´ÏÂÈ¥¡£
								{
									
									
								}
								break;
						 
						 
						 
						 
					 }
			
				
			
			
		
					 break;
			
			
			default:
				break;
		}
			
		/*if (Rx_Buffer[0]==0x01)   //ÅĞ¶Ï¼Ä´æÆ÷ÖĞµÄÄÚÈİ¾ÍÍ¨¹ıÕâ¸öÀ´½øĞĞ¡£
			{
				Write_SOCK_Data_Buffer(s, Tx_Buffer, size);
				Write_SOCK_Data_Buffer(0, "\r\n THis is 1\r\n", 23);
			
			}*/
		
	}
	else
	{
		//Èç¹û²»ÊÇ±¾»úµÄµØÖ·µÄÇé¿ö ÈçºÎ½øĞĞ´¦Àí
		
		
		
	}

	
	/*if (Rx_Buffer[0]==0x11)   //ÅĞ¶Ï¼Ä´æÆ÷ÖĞµÄÄÚÈİ¾ÍÍ¨¹ıÕâ¸öÀ´½øĞĞ¡£
	{
		Write_SOCK_Data_Buffer(s, Tx_Buffer, size);
		Write_SOCK_Data_Buffer(0, "\r\n THis is 1\r\n", 23);
		
	}*/
	
		//

//ÏÂÃæĞ´Èë¹ØÓÚ¿ò¼ÜµÄÄÚÈİ£¬½«ÎÒÃÇËùµÃµ½µÄÕû¸öÊı¾İ½øĞĞ·Ö°ü´¦Àí£¬
		
		
		
}




/*******************************************************************************
* º¯ÊıÃû  : W5500_GPIO_Configuration
* ÃèÊö    : W5500 GPIO³õÊ¼»¯ÅäÖÃ
* ÊäÈë    : ÎŞ
* Êä³ö    : ÎŞ
* ·µ»ØÖµ  : ÎŞ
* ËµÃ÷    : ÎŞ
*******************************************************************************/
void W5500_GPIO_Configuration(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	EXTI_InitTypeDef  EXTI_InitStructure;	

	/* W5500_RSTÒı½Å³õÊ¼»¯ÅäÖÃ(PC5) */
	GPIO_InitStructure.GPIO_Pin  = W5500_RST;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(W5500_RST_PORT, &GPIO_InitStructure);
	GPIO_ResetBits(W5500_RST_PORT, W5500_RST);
	
	/* W5500_INTÒı½Å³õÊ¼»¯ÅäÖÃ(PC4) */	
	GPIO_InitStructure.GPIO_Pin = W5500_INT;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(W5500_INT_PORT, &GPIO_InitStructure);
		
	/* Connect EXTI Line4 to PC4 */
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource9);

	/* PC4 as W5500 interrupt input */
	EXTI_InitStructure.EXTI_Line = EXTI_Line9;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
}

/*******************************************************************************
* º¯ÊıÃû  : W5500_NVIC_Configuration
* ÃèÊö    : W5500 ½ÓÊÕÒı½ÅÖĞ¶ÏÓÅÏÈ¼¶ÉèÖÃ
* ÊäÈë    : ÎŞ
* Êä³ö    : ÎŞ
* ·µ»ØÖµ  : ÎŞ
* ËµÃ÷    : ÎŞ
*******************************************************************************/
void W5500_NVIC_Configuration(void)
{
  	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable the EXTI4 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/*******************************************************************************
* º¯ÊıÃû  : EXTI4_IRQHandler
* ÃèÊö    : ÖĞ¶ÏÏß4ÖĞ¶Ï·şÎñº¯Êı(W5500 INTÒı½ÅÖĞ¶Ï·şÎñº¯Êı)
* ÊäÈë    : ÎŞ
* Êä³ö    : ÎŞ
* ·µ»ØÖµ  : ÎŞ
* ËµÃ÷    : ÎŞ
*******************************************************************************/
void EXTI9_5_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line9) != RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line9);
		W5500_Interrupt=1;
	}
}

/*******************************************************************************
* º¯ÊıÃû  : SPI_Configuration
* ÃèÊö    : W5500 SPI³õÊ¼»¯ÅäÖÃ(STM32 SPI3)
* ÊäÈë    : ÎŞ
* Êä³ö    : ÎŞ
* ·µ»ØÖµ  : ÎŞ
* ËµÃ÷    : ÎŞ
*******************************************************************************/
void SPI_Configuration(void)
{
	GPIO_InitTypeDef 	GPIO_InitStructure;
	SPI_InitTypeDef   	SPI_InitStructure;	   
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);
  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB |  RCC_APB2Periph_AFIO, ENABLE);	
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);//´ËÒı½ÅÖ÷ÒªÊÇJTAGµÄ×÷ÓÃ£¬ÕâÀïremap³É¸´ÓÃÒı½Å
	/* ³õÊ¼»¯SCK¡¢MISO¡¢MOSIÒı½Å */
//////	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
//////	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//////	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
//////	GPIO_Init(GPIOB, &GPIO_InitStructure);
//////	GPIO_SetBits(GPIOB,GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5);
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
 
  //SPI_FLASH_SPI pins: MISO--PB4
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
 
  //SPI_FLASH_SPI pins: MOSI--PB5 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	/* ³õÊ¼»¯CSÒı½Å */
	GPIO_InitStructure.GPIO_Pin = W5500_SCS;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_Init(W5500_SCS_PORT, &GPIO_InitStructure);
	GPIO_SetBits(W5500_SCS_PORT, W5500_SCS);

	/* ³õÊ¼»¯ÅäÖÃSTM32 SPI3 */
	SPI_InitStructure.SPI_Direction=SPI_Direction_2Lines_FullDuplex;	//SPIÉèÖÃÎªË«ÏßË«ÏòÈ«Ë«¹¤
	SPI_InitStructure.SPI_Mode=SPI_Mode_Master;							//ÉèÖÃÎªÖ÷SPI
	SPI_InitStructure.SPI_DataSize=SPI_DataSize_8b;						//SPI·¢ËÍ½ÓÊÕ8Î»Ö¡½á¹¹
	SPI_InitStructure.SPI_CPOL=SPI_CPOL_Low;							//Ê±ÖÓĞü¿ÕµÍ
	SPI_InitStructure.SPI_CPHA=SPI_CPHA_1Edge;							//Êı¾İ²¶»ñÓÚµÚ1¸öÊ±ÖÓÑØ
	SPI_InitStructure.SPI_NSS=SPI_NSS_Soft;								//NSSÓÉÍâ²¿¹Ü½Å¹ÜÀí
	SPI_InitStructure.SPI_BaudRatePrescaler=SPI_BaudRatePrescaler_4;	//²¨ÌØÂÊÔ¤·ÖÆµÖµÎª2
	SPI_InitStructure.SPI_FirstBit=SPI_FirstBit_MSB;					//Êı¾İ´«Êä´ÓMSBÎ»¿ªÊ¼
	SPI_InitStructure.SPI_CRCPolynomial=7;								//CRC¶àÏîÊ½Îª7
	SPI_Init(SPI3,&SPI_InitStructure);									//¸ù¾İSPI_InitStructÖĞÖ¸¶¨µÄ²ÎÊı³õÊ¼»¯ÍâÉèSPI3¼Ä´æÆ÷

	SPI_Cmd(SPI3,ENABLE);	//STM32Ê¹ÄÜSPI3
}

/*******************************************************************************
* º¯ÊıÃû  : SPI3_Send_Byte
* ÃèÊö    : SPI3·¢ËÍ1¸ö×Ö½ÚÊı¾İ
* ÊäÈë    : dat:´ı·¢ËÍµÄÊı¾İ
* Êä³ö    : ÎŞ
* ·µ»ØÖµ  : ÎŞ
* ËµÃ÷    : ÎŞ
*******************************************************************************/
void SPI3_Send_Byte(unsigned char dat)
{
	SPI_I2S_SendData(SPI3,dat);//Ğ´1¸ö×Ö½ÚÊı¾İ
	while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_BSY) == SET);//µÈ´ıÊı¾İ¼Ä´æÆ÷¿Õ
}

/*******************************************************************************
* º¯ÊıÃû  : SPI3_Send_Short
* ÃèÊö    : SPI3·¢ËÍ2¸ö×Ö½ÚÊı¾İ(16Î»)
* ÊäÈë    : dat:´ı·¢ËÍµÄ16Î»Êı¾İ
* Êä³ö    : ÎŞ
* ·µ»ØÖµ  : ÎŞ
* ËµÃ÷    : ÎŞ
*******************************************************************************/
void SPI3_Send_Short(unsigned short dat)
{
	SPI3_Send_Byte(dat/256);//Ğ´Êı¾İ¸ßÎ»
	SPI3_Send_Byte(dat);	//Ğ´Êı¾İµÍÎ»
}

/*******************************************************************************
* º¯ÊıÃû  : Write_W5500_1Byte
* ÃèÊö    : Í¨¹ıSPI3ÏòÖ¸¶¨µØÖ·¼Ä´æÆ÷Ğ´1¸ö×Ö½ÚÊı¾İ
* ÊäÈë    : reg:16Î»¼Ä´æÆ÷µØÖ·,dat:´ıĞ´ÈëµÄÊı¾İ
* Êä³ö    : ÎŞ
* ·µ»ØÖµ  : ÎŞ
* ËµÃ÷    : ÎŞ
*******************************************************************************/
void Write_W5500_1Byte(unsigned short reg, unsigned char dat)
{
	GPIO_ResetBits(W5500_SCS_PORT, W5500_SCS);//ÖÃW5500µÄSCSÎªµÍµçÆ½

	SPI3_Send_Short(reg);//Í¨¹ıSPI3Ğ´16Î»¼Ä´æÆ÷µØÖ·
	SPI3_Send_Byte(FDM1|RWB_WRITE|COMMON_R);//Í¨¹ıSPI3Ğ´¿ØÖÆ×Ö½Ú,1¸ö×Ö½ÚÊı¾İ³¤¶È,Ğ´Êı¾İ,Ñ¡ÔñÍ¨ÓÃ¼Ä´æÆ÷
	SPI3_Send_Byte(dat);//Ğ´1¸ö×Ö½ÚÊı¾İ

	GPIO_SetBits(W5500_SCS_PORT, W5500_SCS); //ÖÃW5500µÄSCSÎª¸ßµçÆ½
}

/*******************************************************************************
* º¯ÊıÃû  : Write_W5500_2Byte
* ÃèÊö    : Í¨¹ıSPI3ÏòÖ¸¶¨µØÖ·¼Ä´æÆ÷Ğ´2¸ö×Ö½ÚÊı¾İ
* ÊäÈë    : reg:16Î»¼Ä´æÆ÷µØÖ·,dat:16Î»´ıĞ´ÈëµÄÊı¾İ(2¸ö×Ö½Ú)
* Êä³ö    : ÎŞ
* ·µ»ØÖµ  : ÎŞ
* ËµÃ÷    : ÎŞ
*******************************************************************************/
void Write_W5500_2Byte(unsigned short reg, unsigned short dat)
{
	GPIO_ResetBits(W5500_SCS_PORT, W5500_SCS);//ÖÃW5500µÄSCSÎªµÍµçÆ½
		
	SPI3_Send_Short(reg);//Í¨¹ıSPI3Ğ´16Î»¼Ä´æÆ÷µØÖ·
	SPI3_Send_Byte(FDM2|RWB_WRITE|COMMON_R);//Í¨¹ıSPI3Ğ´¿ØÖÆ×Ö½Ú,2¸ö×Ö½ÚÊı¾İ³¤¶È,Ğ´Êı¾İ,Ñ¡ÔñÍ¨ÓÃ¼Ä´æÆ÷
	SPI3_Send_Short(dat);//Ğ´16Î»Êı¾İ

	GPIO_SetBits(W5500_SCS_PORT, W5500_SCS); //ÖÃW5500µÄSCSÎª¸ßµçÆ½
}

/*******************************************************************************
* º¯ÊıÃû  : Write_W5500_nByte
* ÃèÊö    : Í¨¹ıSPI3ÏòÖ¸¶¨µØÖ·¼Ä´æÆ÷Ğ´n¸ö×Ö½ÚÊı¾İ
* ÊäÈë    : reg:16Î»¼Ä´æÆ÷µØÖ·,*dat_ptr:´ıĞ´ÈëÊı¾İ»º³åÇøÖ¸Õë,size:´ıĞ´ÈëµÄÊı¾İ³¤¶È
* Êä³ö    : ÎŞ
* ·µ»ØÖµ  : ÎŞ
* ËµÃ÷    : ÎŞ
*******************************************************************************/
void Write_W5500_nByte(unsigned short reg, unsigned char *dat_ptr, unsigned short size)
{
	unsigned short i;

	GPIO_ResetBits(W5500_SCS_PORT, W5500_SCS);//ÖÃW5500µÄSCSÎªµÍµçÆ½	
		
	SPI3_Send_Short(reg);//Í¨¹ıSPI3Ğ´16Î»¼Ä´æÆ÷µØÖ·
	SPI3_Send_Byte(VDM|RWB_WRITE|COMMON_R);//Í¨¹ıSPI3Ğ´¿ØÖÆ×Ö½Ú,N¸ö×Ö½ÚÊı¾İ³¤¶È,Ğ´Êı¾İ,Ñ¡ÔñÍ¨ÓÃ¼Ä´æÆ÷

	for(i=0;i<size;i++)//Ñ­»·½«»º³åÇøµÄsize¸ö×Ö½ÚÊı¾İĞ´ÈëW5500
	{
		SPI3_Send_Byte(*dat_ptr++);//Ğ´Ò»¸ö×Ö½ÚÊı¾İ
	}

	GPIO_SetBits(W5500_SCS_PORT, W5500_SCS); //ÖÃW5500µÄSCSÎª¸ßµçÆ½
}

/*******************************************************************************
* º¯ÊıÃû  : Write_W5500_SOCK_1Byte
* ÃèÊö    : Í¨¹ıSPI3ÏòÖ¸¶¨¶Ë¿Ú¼Ä´æÆ÷Ğ´1¸ö×Ö½ÚÊı¾İ
* ÊäÈë    : s:¶Ë¿ÚºÅ,reg:16Î»¼Ä´æÆ÷µØÖ·,dat:´ıĞ´ÈëµÄÊı¾İ
* Êä³ö    : ÎŞ
* ·µ»ØÖµ  : ÎŞ
* ËµÃ÷    : ÎŞ
*******************************************************************************/
void Write_W5500_SOCK_1Byte(SOCKET s, unsigned short reg, unsigned char dat)
{
	GPIO_ResetBits(W5500_SCS_PORT, W5500_SCS);//ÖÃW5500µÄSCSÎªµÍµçÆ½	
		
	SPI3_Send_Short(reg);//Í¨¹ıSPI3Ğ´16Î»¼Ä´æÆ÷µØÖ·
	SPI3_Send_Byte(FDM1|RWB_WRITE|(s*0x20+0x08));//Í¨¹ıSPI3Ğ´¿ØÖÆ×Ö½Ú,1¸ö×Ö½ÚÊı¾İ³¤¶È,Ğ´Êı¾İ,Ñ¡Ôñ¶Ë¿ÚsµÄ¼Ä´æÆ÷
	SPI3_Send_Byte(dat);//Ğ´1¸ö×Ö½ÚÊı¾İ

	GPIO_SetBits(W5500_SCS_PORT, W5500_SCS); //ÖÃW5500µÄSCSÎª¸ßµçÆ½
}

/*******************************************************************************
* º¯ÊıÃû  : Write_W5500_SOCK_2Byte
* ÃèÊö    : Í¨¹ıSPI3ÏòÖ¸¶¨¶Ë¿Ú¼Ä´æÆ÷Ğ´2¸ö×Ö½ÚÊı¾İ
* ÊäÈë    : s:¶Ë¿ÚºÅ,reg:16Î»¼Ä´æÆ÷µØÖ·,dat:16Î»´ıĞ´ÈëµÄÊı¾İ(2¸ö×Ö½Ú)
* Êä³ö    : ÎŞ
* ·µ»ØÖµ  : ÎŞ
* ËµÃ÷    : ÎŞ
*******************************************************************************/
void Write_W5500_SOCK_2Byte(SOCKET s, unsigned short reg, unsigned short dat)
{
	GPIO_ResetBits(W5500_SCS_PORT, W5500_SCS);//ÖÃW5500µÄSCSÎªµÍµçÆ½
			
	SPI3_Send_Short(reg);//Í¨¹ıSPI3Ğ´16Î»¼Ä´æÆ÷µØÖ·
	SPI3_Send_Byte(FDM2|RWB_WRITE|(s*0x20+0x08));//Í¨¹ıSPI3Ğ´¿ØÖÆ×Ö½Ú,2¸ö×Ö½ÚÊı¾İ³¤¶È,Ğ´Êı¾İ,Ñ¡Ôñ¶Ë¿ÚsµÄ¼Ä´æÆ÷
	SPI3_Send_Short(dat);//Ğ´16Î»Êı¾İ

	GPIO_SetBits(W5500_SCS_PORT, W5500_SCS); //ÖÃW5500µÄSCSÎª¸ßµçÆ½
}

/*******************************************************************************
* º¯ÊıÃû  : Write_W5500_SOCK_4Byte
* ÃèÊö    : Í¨¹ıSPI3ÏòÖ¸¶¨¶Ë¿Ú¼Ä´æÆ÷Ğ´4¸ö×Ö½ÚÊı¾İ
* ÊäÈë    : s:¶Ë¿ÚºÅ,reg:16Î»¼Ä´æÆ÷µØÖ·,*dat_ptr:´ıĞ´ÈëµÄ4¸ö×Ö½Ú»º³åÇøÖ¸Õë
* Êä³ö    : ÎŞ
* ·µ»ØÖµ  : ÎŞ
* ËµÃ÷    : ÎŞ
*******************************************************************************/
void Write_W5500_SOCK_4Byte(SOCKET s, unsigned short reg, unsigned char *dat_ptr)
{
	GPIO_ResetBits(W5500_SCS_PORT, W5500_SCS);//ÖÃW5500µÄSCSÎªµÍµçÆ½
			
	SPI3_Send_Short(reg);//Í¨¹ıSPI3Ğ´16Î»¼Ä´æÆ÷µØÖ·
	SPI3_Send_Byte(FDM4|RWB_WRITE|(s*0x20+0x08));//Í¨¹ıSPI3Ğ´¿ØÖÆ×Ö½Ú,4¸ö×Ö½ÚÊı¾İ³¤¶È,Ğ´Êı¾İ,Ñ¡Ôñ¶Ë¿ÚsµÄ¼Ä´æÆ÷

	SPI3_Send_Byte(*dat_ptr++);//Ğ´µÚ1¸ö×Ö½ÚÊı¾İ
	SPI3_Send_Byte(*dat_ptr++);//Ğ´µÚ2¸ö×Ö½ÚÊı¾İ
	SPI3_Send_Byte(*dat_ptr++);//Ğ´µÚ3¸ö×Ö½ÚÊı¾İ
	SPI3_Send_Byte(*dat_ptr++);//Ğ´µÚ4¸ö×Ö½ÚÊı¾İ

	GPIO_SetBits(W5500_SCS_PORT, W5500_SCS); //ÖÃW5500µÄSCSÎª¸ßµçÆ½
}

/*******************************************************************************
* º¯ÊıÃû  : Read_W5500_1Byte
* ÃèÊö    : ¶ÁW5500Ö¸¶¨µØÖ·¼Ä´æÆ÷µÄ1¸ö×Ö½ÚÊı¾İ
* ÊäÈë    : reg:16Î»¼Ä´æÆ÷µØÖ·
* Êä³ö    : ÎŞ
* ·µ»ØÖµ  : ¶ÁÈ¡µ½¼Ä´æÆ÷µÄ1¸ö×Ö½ÚÊı¾İ
* ËµÃ÷    : ÎŞ
*******************************************************************************/
unsigned char Read_W5500_1Byte(unsigned short reg)
{
	unsigned char i;

	GPIO_ResetBits(W5500_SCS_PORT, W5500_SCS);//ÖÃW5500µÄSCSÎªµÍµçÆ½
			
	SPI3_Send_Short(reg);//Í¨¹ıSPI3Ğ´16Î»¼Ä´æÆ÷µØÖ·
	SPI3_Send_Byte(FDM1|RWB_READ|COMMON_R);//Í¨¹ıSPI3Ğ´¿ØÖÆ×Ö½Ú,1¸ö×Ö½ÚÊı¾İ³¤¶È,¶ÁÊı¾İ,Ñ¡ÔñÍ¨ÓÃ¼Ä´æÆ÷

	i=SPI_I2S_ReceiveData(SPI3);
	SPI3_Send_Byte(0x00);//·¢ËÍÒ»¸öÑÆÊı¾İ
	i=SPI_I2S_ReceiveData(SPI3);//¶ÁÈ¡1¸ö×Ö½ÚÊı¾İ

	GPIO_SetBits(W5500_SCS_PORT, W5500_SCS);//ÖÃW5500µÄSCSÎª¸ßµçÆ½
	return i;//·µ»Ø¶ÁÈ¡µ½µÄ¼Ä´æÆ÷Êı¾İ
}

/*******************************************************************************
* º¯ÊıÃû  : Read_W5500_SOCK_1Byte
* ÃèÊö    : ¶ÁW5500Ö¸¶¨¶Ë¿Ú¼Ä´æÆ÷µÄ1¸ö×Ö½ÚÊı¾İ
* ÊäÈë    : s:¶Ë¿ÚºÅ,reg:16Î»¼Ä´æÆ÷µØÖ·
* Êä³ö    : ÎŞ
* ·µ»ØÖµ  : ¶ÁÈ¡µ½¼Ä´æÆ÷µÄ1¸ö×Ö½ÚÊı¾İ
* ËµÃ÷    : ÎŞ
*******************************************************************************/
unsigned char Read_W5500_SOCK_1Byte(SOCKET s, unsigned short reg)
{
	unsigned char i;

	GPIO_ResetBits(W5500_SCS_PORT, W5500_SCS);//ÖÃW5500µÄSCSÎªµÍµçÆ½
			
	SPI3_Send_Short(reg);//Í¨¹ıSPI3Ğ´16Î»¼Ä´æÆ÷µØÖ·
	SPI3_Send_Byte(FDM1|RWB_READ|(s*0x20+0x08));//Í¨¹ıSPI3Ğ´¿ØÖÆ×Ö½Ú,1¸ö×Ö½ÚÊı¾İ³¤¶È,¶ÁÊı¾İ,Ñ¡Ôñ¶Ë¿ÚsµÄ¼Ä´æÆ÷

	i=SPI_I2S_ReceiveData(SPI3);//Çå¿Õ»º´æÊı¾İ£¿£¿£¿£¿£¿
	SPI3_Send_Byte(0x00);//·¢ËÍÒ»¸öÑÆÊı¾İ
	i=SPI_I2S_ReceiveData(SPI3);//¶ÁÈ¡1¸ö×Ö½ÚÊı¾İ

	GPIO_SetBits(W5500_SCS_PORT, W5500_SCS);//ÖÃW5500µÄSCSÎª¸ßµçÆ½
	return i;//·µ»Ø¶ÁÈ¡µ½µÄ¼Ä´æÆ÷Êı¾İ
}

/*******************************************************************************
* º¯ÊıÃû  : Read_W5500_SOCK_2Byte
* ÃèÊö    : ¶ÁW5500Ö¸¶¨¶Ë¿Ú¼Ä´æÆ÷µÄ2¸ö×Ö½ÚÊı¾İ
* ÊäÈë    : s:¶Ë¿ÚºÅ,reg:16Î»¼Ä´æÆ÷µØÖ·
* Êä³ö    : ÎŞ
* ·µ»ØÖµ  : ¶ÁÈ¡µ½¼Ä´æÆ÷µÄ2¸ö×Ö½ÚÊı¾İ(16Î»)
* ËµÃ÷    : ÎŞ
*******************************************************************************/
unsigned short Read_W5500_SOCK_2Byte(SOCKET s, unsigned short reg)
{
	unsigned short i;

	GPIO_ResetBits(W5500_SCS_PORT, W5500_SCS);//ÖÃW5500µÄSCSÎªµÍµçÆ½
			
	SPI3_Send_Short(reg);//Í¨¹ıSPI3Ğ´16Î»¼Ä´æÆ÷µØÖ·
	SPI3_Send_Byte(FDM2|RWB_READ|(s*0x20+0x08));//Í¨¹ıSPI3Ğ´¿ØÖÆ×Ö½Ú,2¸ö×Ö½ÚÊı¾İ³¤¶È,¶ÁÊı¾İ,Ñ¡Ôñ¶Ë¿ÚsµÄ¼Ä´æÆ÷

	i=SPI_I2S_ReceiveData(SPI3);
	SPI3_Send_Byte(0x00);//·¢ËÍÒ»¸öÑÆÊı¾İ
	i=SPI_I2S_ReceiveData(SPI3);//¶ÁÈ¡¸ßÎ»Êı¾İ
	SPI3_Send_Byte(0x00);//·¢ËÍÒ»¸öÑÆÊı¾İ
	i*=256;
	i+=SPI_I2S_ReceiveData(SPI3);//¶ÁÈ¡µÍÎ»Êı¾İ

	GPIO_SetBits(W5500_SCS_PORT, W5500_SCS);//ÖÃW5500µÄSCSÎª¸ßµçÆ½
	return i;//·µ»Ø¶ÁÈ¡µ½µÄ¼Ä´æÆ÷Êı¾İ
}

/*******************************************************************************
* º¯ÊıÃû  : Read_SOCK_Data_Buffer
* ÃèÊö    : ´ÓW5500½ÓÊÕÊı¾İ»º³åÇøÖĞ¶ÁÈ¡Êı¾İ
* ÊäÈë    : s:¶Ë¿ÚºÅ,*dat_ptr:Êı¾İ±£´æ»º³åÇøÖ¸Õë
* Êä³ö    : ÎŞ
* ·µ»ØÖµ  : ¶ÁÈ¡µ½µÄÊı¾İ³¤¶È,rx_size¸ö×Ö½Ú
* ËµÃ÷    : ÎŞ
*******************************************************************************/
unsigned short Read_SOCK_Data_Buffer(SOCKET s, unsigned char *dat_ptr)
{
	unsigned short rx_size;
	unsigned short offset, offset1;
	unsigned short i;
	unsigned char j;

	rx_size=Read_W5500_SOCK_2Byte(s,Sn_RX_RSR);
	if(rx_size==0) return 0;//Ã»½ÓÊÕµ½Êı¾İÔò·µ»Ø
	if(rx_size>1460) rx_size=1460;

	offset=Read_W5500_SOCK_2Byte(s,Sn_RX_RD);
	offset1=offset;
	offset&=(S_RX_SIZE-1);//¼ÆËãÊµ¼ÊµÄÎïÀíµØÖ·

	GPIO_ResetBits(W5500_SCS_PORT, W5500_SCS);//ÖÃW5500µÄSCSÎªµÍµçÆ½

	SPI3_Send_Short(offset);//Ğ´16Î»µØÖ·
	SPI3_Send_Byte(VDM|RWB_READ|(s*0x20+0x18));//Ğ´¿ØÖÆ×Ö½Ú,N¸ö×Ö½ÚÊı¾İ³¤¶È,¶ÁÊı¾İ,Ñ¡Ôñ¶Ë¿ÚsµÄ¼Ä´æÆ÷
	j=SPI_I2S_ReceiveData(SPI3);
	
	if((offset+rx_size)<S_RX_SIZE)//Èç¹û×î´óµØÖ·Î´³¬¹ıW5500½ÓÊÕ»º³åÇø¼Ä´æÆ÷µÄ×î´óµØÖ·
	{
		for(i=0;i<rx_size;i++)//Ñ­»·¶ÁÈ¡rx_size¸ö×Ö½ÚÊı¾İ
		{
			SPI3_Send_Byte(0x00);//·¢ËÍÒ»¸öÑÆÊı¾İ
			j=SPI_I2S_ReceiveData(SPI3);//¶ÁÈ¡1¸ö×Ö½ÚÊı¾İ
			*dat_ptr=j;//½«¶ÁÈ¡µ½µÄÊı¾İ±£´æµ½Êı¾İ±£´æ»º³åÇø
			dat_ptr++;//Êı¾İ±£´æ»º³åÇøÖ¸ÕëµØÖ·×ÔÔö1
		}
	}
	else//Èç¹û×î´óµØÖ·³¬¹ıW5500½ÓÊÕ»º³åÇø¼Ä´æÆ÷µÄ×î´óµØÖ·
	{
		offset=S_RX_SIZE-offset;
		for(i=0;i<offset;i++)//Ñ­»·¶ÁÈ¡³öÇ°offset¸ö×Ö½ÚÊı¾İ
		{
			SPI3_Send_Byte(0x00);//·¢ËÍÒ»¸öÑÆÊı¾İ
			j=SPI_I2S_ReceiveData(SPI3);//¶ÁÈ¡1¸ö×Ö½ÚÊı¾İ
			*dat_ptr=j;//½«¶ÁÈ¡µ½µÄÊı¾İ±£´æµ½Êı¾İ±£´æ»º³åÇø
			dat_ptr++;//Êı¾İ±£´æ»º³åÇøÖ¸ÕëµØÖ·×ÔÔö1
		}
		GPIO_SetBits(W5500_SCS_PORT, W5500_SCS); //ÖÃW5500µÄSCSÎª¸ßµçÆ½

		GPIO_ResetBits(W5500_SCS_PORT, W5500_SCS);//ÖÃW5500µÄSCSÎªµÍµçÆ½

		SPI3_Send_Short(0x00);//Ğ´16Î»µØÖ·
		SPI3_Send_Byte(VDM|RWB_READ|(s*0x20+0x18));//Ğ´¿ØÖÆ×Ö½Ú,N¸ö×Ö½ÚÊı¾İ³¤¶È,¶ÁÊı¾İ,Ñ¡Ôñ¶Ë¿ÚsµÄ¼Ä´æÆ÷
		j=SPI_I2S_ReceiveData(SPI3);

		for(;i<rx_size;i++)//Ñ­»·¶ÁÈ¡ºórx_size-offset¸ö×Ö½ÚÊı¾İ
		{
			SPI3_Send_Byte(0x00);//·¢ËÍÒ»¸öÑÆÊı¾İ
			j=SPI_I2S_ReceiveData(SPI3);//¶ÁÈ¡1¸ö×Ö½ÚÊı¾İ
			*dat_ptr=j;//½«¶ÁÈ¡µ½µÄÊı¾İ±£´æµ½Êı¾İ±£´æ»º³åÇø
			dat_ptr++;//Êı¾İ±£´æ»º³åÇøÖ¸ÕëµØÖ·×ÔÔö1
		}
	}
	GPIO_SetBits(W5500_SCS_PORT, W5500_SCS); //ÖÃW5500µÄSCSÎª¸ßµçÆ½

	offset1+=rx_size;//¸üĞÂÊµ¼ÊÎïÀíµØÖ·,¼´ÏÂ´Î¶ÁÈ¡½ÓÊÕµ½µÄÊı¾İµÄÆğÊ¼µØÖ·
	Write_W5500_SOCK_2Byte(s, Sn_RX_RD, offset1);
	Write_W5500_SOCK_1Byte(s, Sn_CR, RECV);//·¢ËÍÆô¶¯½ÓÊÕÃüÁî
	return rx_size;//·µ»Ø½ÓÊÕµ½Êı¾İµÄ³¤¶È
}

/*******************************************************************************
* º¯ÊıÃû  : Write_SOCK_Data_Buffer
* ÃèÊö    : ½«Êı¾İĞ´ÈëW5500µÄÊı¾İ·¢ËÍ»º³åÇø
* ÊäÈë    : s:¶Ë¿ÚºÅ,*dat_ptr:Êı¾İ±£´æ»º³åÇøÖ¸Õë,size:´ıĞ´ÈëÊı¾İµÄ³¤¶È
* Êä³ö    : ÎŞ
* ·µ»ØÖµ  : ÎŞ
* ËµÃ÷    : ÎŞ
*******************************************************************************/
void Write_SOCK_Data_Buffer(SOCKET s, unsigned char *dat_ptr, unsigned short size)
{
	unsigned short offset,offset1;
	unsigned short i;

	//Èç¹ûÊÇUDPÄ£Ê½,¿ÉÒÔÔÚ´ËÉèÖÃÄ¿µÄÖ÷»úµÄIPºÍ¶Ë¿ÚºÅ
	if((Read_W5500_SOCK_1Byte(s,Sn_MR)&0x0f) != SOCK_UDP)//Èç¹ûSocket´ò¿ªÊ§°Ü
	{		
		Write_W5500_SOCK_4Byte(s, Sn_DIPR, UDP_DIPR);//ÉèÖÃÄ¿µÄÖ÷»úIP  		
		Write_W5500_SOCK_2Byte(s, Sn_DPORTR, UDP_DPORT[0]*256+UDP_DPORT[1]);//ÉèÖÃÄ¿µÄÖ÷»ú¶Ë¿ÚºÅ				
	}

	offset=Read_W5500_SOCK_2Byte(s,Sn_TX_WR);
	offset1=offset;
	offset&=(S_TX_SIZE-1);//¼ÆËãÊµ¼ÊµÄÎïÀíµØÖ·

	GPIO_ResetBits(W5500_SCS_PORT, W5500_SCS);//ÖÃW5500µÄSCSÎªµÍµçÆ½

	SPI3_Send_Short(offset);//Ğ´16Î»µØÖ·
	SPI3_Send_Byte(VDM|RWB_WRITE|(s*0x20+0x10));//Ğ´¿ØÖÆ×Ö½Ú,N¸ö×Ö½ÚÊı¾İ³¤¶È,Ğ´Êı¾İ,Ñ¡Ôñ¶Ë¿ÚsµÄ¼Ä´æÆ÷

	if((offset+size)<S_TX_SIZE)//Èç¹û×î´óµØÖ·Î´³¬¹ıW5500·¢ËÍ»º³åÇø¼Ä´æÆ÷µÄ×î´óµØÖ·
	{
		for(i=0;i<size;i++)//Ñ­»·Ğ´Èësize¸ö×Ö½ÚÊı¾İ
		{
			SPI3_Send_Byte(*dat_ptr++);//Ğ´ÈëÒ»¸ö×Ö½ÚµÄÊı¾İ		
		}
	}
	else//Èç¹û×î´óµØÖ·³¬¹ıW5500·¢ËÍ»º³åÇø¼Ä´æÆ÷µÄ×î´óµØÖ·
	{
		offset=S_TX_SIZE-offset;
		for(i=0;i<offset;i++)//Ñ­»·Ğ´ÈëÇ°offset¸ö×Ö½ÚÊı¾İ
		{
			SPI3_Send_Byte(*dat_ptr++);//Ğ´ÈëÒ»¸ö×Ö½ÚµÄÊı¾İ
		}
		GPIO_SetBits(W5500_SCS_PORT, W5500_SCS); //ÖÃW5500µÄSCSÎª¸ßµçÆ½

		GPIO_ResetBits(W5500_SCS_PORT, W5500_SCS);//ÖÃW5500µÄSCSÎªµÍµçÆ½

		SPI3_Send_Short(0x00);//Ğ´16Î»µØÖ·
		SPI3_Send_Byte(VDM|RWB_WRITE|(s*0x20+0x10));//Ğ´¿ØÖÆ×Ö½Ú,N¸ö×Ö½ÚÊı¾İ³¤¶È,Ğ´Êı¾İ,Ñ¡Ôñ¶Ë¿ÚsµÄ¼Ä´æÆ÷

		for(;i<size;i++)//Ñ­»·Ğ´Èësize-offset¸ö×Ö½ÚÊı¾İ
		{
			SPI3_Send_Byte(*dat_ptr++);//Ğ´ÈëÒ»¸ö×Ö½ÚµÄÊı¾İ
		}
	}
	GPIO_SetBits(W5500_SCS_PORT, W5500_SCS); //ÖÃW5500µÄSCSÎª¸ßµçÆ½

	offset1+=size;//¸üĞÂÊµ¼ÊÎïÀíµØÖ·,¼´ÏÂ´ÎĞ´´ı·¢ËÍÊı¾İµ½·¢ËÍÊı¾İ»º³åÇøµÄÆğÊ¼µØÖ·
	Write_W5500_SOCK_2Byte(s, Sn_TX_WR, offset1);
	Write_W5500_SOCK_1Byte(s, Sn_CR, SEND);//·¢ËÍÆô¶¯·¢ËÍÃüÁî				
}

/*******************************************************************************
* º¯ÊıÃû  : W5500_Hardware_Reset
* ÃèÊö    : Ó²¼ş¸´Î»W5500
* ÊäÈë    : ÎŞ
* Êä³ö    : ÎŞ
* ·µ»ØÖµ  : ÎŞ
* ËµÃ÷    : W5500µÄ¸´Î»Òı½Å±£³ÖµÍµçÆ½ÖÁÉÙ500usÒÔÉÏ,²ÅÄÜÖØÎ§W5500
*******************************************************************************/
void W5500_Hardware_Reset(void)
{
	GPIO_ResetBits(W5500_RST_PORT, W5500_RST);//¸´Î»Òı½ÅÀ­µÍ
	Delay(50);
	GPIO_SetBits(W5500_RST_PORT, W5500_RST);//¸´Î»Òı½ÅÀ­¸ß
	Delay(200);
	while((Read_W5500_1Byte(PHYCFGR)&LINK)==0);//µÈ´ıÒÔÌ«ÍøÁ¬½ÓÍê³É
}

/*******************************************************************************
* º¯ÊıÃû  : W5500_Init
* ÃèÊö    : ³õÊ¼»¯W5500¼Ä´æÆ÷º¯Êı
* ÊäÈë    : ÎŞ
* Êä³ö    : ÎŞ
* ·µ»ØÖµ  : ÎŞ
* ËµÃ÷    : ÔÚÊ¹ÓÃW5500Ö®Ç°£¬ÏÈ¶ÔW5500³õÊ¼»¯
*******************************************************************************/
void W5500_Init(void)
{
	u8 i=0;

	Write_W5500_1Byte(MR, RST);//Èí¼ş¸´Î»W5500,ÖÃ1ÓĞĞ§,¸´Î»ºó×Ô¶¯Çå0
	Delay(10);//ÑÓÊ±10ms,×Ô¼º¶¨Òå¸Ãº¯Êı

	//ÉèÖÃÍø¹Ø(Gateway)µÄIPµØÖ·,Gateway_IPÎª4×Ö½Úunsigned charÊı×é,×Ô¼º¶¨Òå 
	//Ê¹ÓÃÍø¹Ø¿ÉÒÔÊ¹Í¨ĞÅÍ»ÆÆ×ÓÍøµÄ¾ÖÏŞ£¬Í¨¹ıÍø¹Ø¿ÉÒÔ·ÃÎÊµ½ÆäËü×ÓÍø»ò½øÈëInternet
	Write_W5500_nByte(GAR, Gateway_IP, 4);
			
	//ÉèÖÃ×ÓÍøÑÚÂë(MASK)Öµ,SUB_MASKÎª4×Ö½Úunsigned charÊı×é,×Ô¼º¶¨Òå
	//×ÓÍøÑÚÂëÓÃÓÚ×ÓÍøÔËËã
	Write_W5500_nByte(SUBR,Sub_Mask,4);		
	
	//ÉèÖÃÎïÀíµØÖ·,PHY_ADDRÎª6×Ö½Úunsigned charÊı×é,×Ô¼º¶¨Òå,ÓÃÓÚÎ¨Ò»±êÊ¶ÍøÂçÉè±¸µÄÎïÀíµØÖ·Öµ
	//¸ÃµØÖ·ÖµĞèÒªµ½IEEEÉêÇë£¬°´ÕÕOUIµÄ¹æ¶¨£¬Ç°3¸ö×Ö½ÚÎª³§ÉÌ´úÂë£¬ºóÈı¸ö×Ö½ÚÎª²úÆ·ĞòºÅ
	//Èç¹û×Ô¼º¶¨ÒåÎïÀíµØÖ·£¬×¢ÒâµÚÒ»¸ö×Ö½Ú±ØĞëÎªÅ¼Êı
	Write_W5500_nByte(SHAR,Phy_Addr,6);		

	//ÉèÖÃ±¾»úµÄIPµØÖ·,IP_ADDRÎª4×Ö½Úunsigned charÊı×é,×Ô¼º¶¨Òå
	//×¢Òâ£¬Íø¹ØIP±ØĞëÓë±¾»úIPÊôÓÚÍ¬Ò»¸ö×ÓÍø£¬·ñÔò±¾»ú½«ÎŞ·¨ÕÒµ½Íø¹Ø
	Write_W5500_nByte(SIPR,IP_Addr,4);		
	
	//ÉèÖÃ·¢ËÍ»º³åÇøºÍ½ÓÊÕ»º³åÇøµÄ´óĞ¡£¬²Î¿¼W5500Êı¾İÊÖ²á
	for(i=0;i<8;i++)
	{
		Write_W5500_SOCK_1Byte(i,Sn_RXBUF_SIZE, 0x02);//Socket Rx memory size=2k
		Write_W5500_SOCK_1Byte(i,Sn_TXBUF_SIZE, 0x02);//Socket Tx mempry size=2k
	}

	//ÉèÖÃÖØÊÔÊ±¼ä£¬Ä¬ÈÏÎª2000(200ms) 
	//Ã¿Ò»µ¥Î»ÊıÖµÎª100Î¢Ãë,³õÊ¼»¯Ê±ÖµÉèÎª2000(0x07D0),µÈÓÚ200ºÁÃë
	Write_W5500_2Byte(RTR, 0x07d0);

	//ÉèÖÃÖØÊÔ´ÎÊı£¬Ä¬ÈÏÎª8´Î 
	//Èç¹ûÖØ·¢µÄ´ÎÊı³¬¹ıÉè¶¨Öµ,Ôò²úÉú³¬Ê±ÖĞ¶Ï(Ïà¹ØµÄ¶Ë¿ÚÖĞ¶Ï¼Ä´æÆ÷ÖĞµÄSn_IR ³¬Ê±Î»(TIMEOUT)ÖÃ¡°1¡±)
	Write_W5500_1Byte(RCR,8);

	//Æô¶¯ÖĞ¶Ï£¬²Î¿¼W5500Êı¾İÊÖ²áÈ·¶¨×Ô¼ºĞèÒªµÄÖĞ¶ÏÀàĞÍ
	//IMR_CONFLICTÊÇIPµØÖ·³åÍ»Òì³£ÖĞ¶Ï,IMR_UNREACHÊÇUDPÍ¨ĞÅÊ±£¬µØÖ·ÎŞ·¨µ½´ïµÄÒì³£ÖĞ¶Ï
	//ÆäËüÊÇSocketÊÂ¼şÖĞ¶Ï£¬¸ù¾İĞèÒªÌí¼Ó
	Write_W5500_1Byte(IMR,IM_IR7 | IM_IR6);
	Write_W5500_1Byte(SIMR,S0_IMR);
	Write_W5500_SOCK_1Byte(0, Sn_IMR, IMR_SENDOK | IMR_TIMEOUT | IMR_RECV | IMR_DISCON | IMR_CON);
}

/*******************************************************************************
* º¯ÊıÃû  : Detect_Gateway
* ÃèÊö    : ¼ì²éÍø¹Ø·şÎñÆ÷
* ÊäÈë    : ÎŞ
* Êä³ö    : ÎŞ
* ·µ»ØÖµ  : ³É¹¦·µ»ØTRUE(0xFF),Ê§°Ü·µ»ØFALSE(0x00)
* ËµÃ÷    : ÎŞ
*******************************************************************************/
unsigned char Detect_Gateway(void)
{
	unsigned char ip_adde[4];
	ip_adde[0]=IP_Addr[0]+1;
	ip_adde[1]=IP_Addr[1]+1;
	ip_adde[2]=IP_Addr[2]+1;
	ip_adde[3]=IP_Addr[3]+1;

	//¼ì²éÍø¹Ø¼°»ñÈ¡Íø¹ØµÄÎïÀíµØÖ·
	Write_W5500_SOCK_4Byte(0,Sn_DIPR,ip_adde);//ÏòÄ¿µÄµØÖ·¼Ä´æÆ÷Ğ´ÈëÓë±¾»úIP²»Í¬µÄIPÖµ
	Write_W5500_SOCK_1Byte(0,Sn_MR,MR_TCP);//ÉèÖÃsocketÎªTCPÄ£Ê½
	Write_W5500_SOCK_1Byte(0,Sn_CR,OPEN);//´ò¿ªSocket	
	Delay(5);//ÑÓÊ±5ms 	
	
	if(Read_W5500_SOCK_1Byte(0,Sn_SR) != SOCK_INIT)//Èç¹ûsocket´ò¿ªÊ§°Ü
	{
		Write_W5500_SOCK_1Byte(0,Sn_CR,CLOSE);//´ò¿ª²»³É¹¦,¹Ø±ÕSocket
		LED2=0;
		LED1=0;
		return FALSE;//·µ»ØFALSE(0x00)
	}

	Write_W5500_SOCK_1Byte(0,Sn_CR,CONNECT);//ÉèÖÃSocketÎªConnectÄ£Ê½						

	do
	{
		u8 j=0;
		j=Read_W5500_SOCK_1Byte(0,Sn_IR);//¶ÁÈ¡Socket0ÖĞ¶Ï±êÖ¾¼Ä´æÆ÷¡¢¡¢¡¢¡¢¡¢¡¢¡¢¡¢¡¢
		if(j!=0)
		Write_W5500_SOCK_1Byte(0,Sn_IR,j);///¡¢¡¢¡¢¡¢¡¢¡¢¡¢¡¢¡¢¡¢¡¢¡¢¡¢
		Delay(5);//ÑÓÊ±5ms 
		
		if((j&IR_TIMEOUT) == IR_TIMEOUT)
		{
			return FALSE;	
		}
		else if(Read_W5500_SOCK_1Byte(0,Sn_DHAR) != 0xff)/////////////////////////
		{
			Write_W5500_SOCK_1Byte(0,Sn_CR,CLOSE);//¹Ø±ÕSocket//////////////////////////
			return TRUE;							
		}
	}while(1);
}

/*******************************************************************************
* º¯ÊıÃû  : Socket_Init
* ÃèÊö    : Ö¸¶¨Socket(0~7)³õÊ¼»¯
* ÊäÈë    : s:´ı³õÊ¼»¯µÄ¶Ë¿Ú
* Êä³ö    : ÎŞ
* ·µ»ØÖµ  : ÎŞ
* ËµÃ÷    : ÎŞ
*******************************************************************************/
void Socket_Init(SOCKET s)
{
	//ÉèÖÃ·ÖÆ¬³¤¶È£¬²Î¿¼W5500Êı¾İÊÖ²á£¬¸ÃÖµ¿ÉÒÔ²»ĞŞ¸Ä	
	Write_W5500_SOCK_2Byte(0, Sn_MSSR, 1460);//×î´ó·ÖÆ¬×Ö½ÚÊı=1460(0x5b4)
	//ÉèÖÃÖ¸¶¨¶Ë¿Ú
	switch(s)
	{
		case 0:
			//ÉèÖÃ¶Ë¿Ú0µÄ¶Ë¿ÚºÅ
			Write_W5500_SOCK_2Byte(0, Sn_PORT, S0_Port[0]*256+S0_Port[1]);		
			
			break;

		case 1:
			break;

		case 2:
			break;

		case 3:
			break;

		case 4:
			break;

		case 5:
			break;

		case 6:
			break;

		case 7:
			break;

		default:
			break;
	}
}

/*******************************************************************************
* º¯ÊıÃû  : Socket_Connect
* ÃèÊö    : ÉèÖÃÖ¸¶¨Socket(0~7)Îª¿Í»§¶ËÓëÔ¶³Ì·şÎñÆ÷Á¬½Ó
* ÊäÈë    : s:´ıÉè¶¨µÄ¶Ë¿Ú
* Êä³ö    : ÎŞ
* ·µ»ØÖµ  : ³É¹¦·µ»ØTRUE(0xFF),Ê§°Ü·µ»ØFALSE(0x00)
* ËµÃ÷    : µ±±¾»úSocket¹¤×÷ÔÚ¿Í»§¶ËÄ£Ê½Ê±,ÒıÓÃ¸Ã³ÌĞò,ÓëÔ¶³Ì·şÎñÆ÷½¨Á¢Á¬½Ó
*			Èç¹ûÆô¶¯Á¬½Óºó³öÏÖ³¬Ê±ÖĞ¶Ï£¬ÔòÓë·şÎñÆ÷Á¬½ÓÊ§°Ü,ĞèÒªÖØĞÂµ÷ÓÃ¸Ã³ÌĞòÁ¬½Ó
*			¸Ã³ÌĞòÃ¿µ÷ÓÃÒ»´Î,¾ÍÓë·şÎñÆ÷²úÉúÒ»´ÎÁ¬½Ó
*******************************************************************************/
unsigned char Socket_Connect(SOCKET s)
{
	Write_W5500_SOCK_1Byte(s,Sn_MR,MR_TCP);//ÉèÖÃsocketÎªTCPÄ£Ê½
	Write_W5500_SOCK_1Byte(s,Sn_CR,OPEN);//´ò¿ªSocket
	Delay(5);//ÑÓÊ±5ms
	if(Read_W5500_SOCK_1Byte(s,Sn_SR)!=SOCK_INIT)//Èç¹ûsocket´ò¿ªÊ§°Ü
	{
		Write_W5500_SOCK_1Byte(s,Sn_CR,CLOSE);//´ò¿ª²»³É¹¦,¹Ø±ÕSocket
		return FALSE;//·µ»ØFALSE(0x00)
	}
	Write_W5500_SOCK_1Byte(s,Sn_CR,CONNECT);//ÉèÖÃSocketÎªConnectÄ£Ê½
	return TRUE;//·µ»ØTRUE,ÉèÖÃ³É¹¦
}

/*******************************************************************************
* º¯ÊıÃû  : Socket_Listen
* ÃèÊö    : ÉèÖÃÖ¸¶¨Socket(0~7)×÷Îª·şÎñÆ÷µÈ´ıÔ¶³ÌÖ÷»úµÄÁ¬½Ó
* ÊäÈë    : s:´ıÉè¶¨µÄ¶Ë¿Ú
* Êä³ö    : ÎŞ
* ·µ»ØÖµ  : ³É¹¦·µ»ØTRUE(0xFF),Ê§°Ü·µ»ØFALSE(0x00)
* ËµÃ÷    : µ±±¾»úSocket¹¤×÷ÔÚ·şÎñÆ÷Ä£Ê½Ê±,ÒıÓÃ¸Ã³ÌĞò,µÈµÈÔ¶³ÌÖ÷»úµÄÁ¬½Ó
*			¸Ã³ÌĞòÖ»µ÷ÓÃÒ»´Î,¾ÍÊ¹W5500ÉèÖÃÎª·şÎñÆ÷Ä£Ê½
*******************************************************************************/
unsigned char Socket_Listen(SOCKET s)
{
	Write_W5500_SOCK_1Byte(s,Sn_MR,MR_TCP);//ÉèÖÃsocketÎªTCPÄ£Ê½ 
	Write_W5500_SOCK_1Byte(s,Sn_CR,OPEN);//´ò¿ªSocket	
	Delay(5);//ÑÓÊ±5ms////////////////////////////¿¨ÔÚÕâÀï*******************************************************************************************
	if(Read_W5500_SOCK_1Byte(s,Sn_SR)!=SOCK_INIT)//Èç¹ûsocket´ò¿ªÊ§°Ü
	{
		Write_W5500_SOCK_1Byte(s,Sn_CR,CLOSE);//´ò¿ª²»³É¹¦,¹Ø±ÕSocket
		return FALSE;//·µ»ØFALSE(0x00)
	}	
	Write_W5500_SOCK_1Byte(s,Sn_CR,LISTEN);//ÉèÖÃSocketÎªÕìÌıÄ£Ê½	
	Delay(5);//ÑÓÊ±5ms
	
	if(Read_W5500_SOCK_1Byte(s,Sn_SR)!=SOCK_LISTEN)//Èç¹ûsocketÉèÖÃÊ§°Ü
	{
		Write_W5500_SOCK_1Byte(s,Sn_CR,CLOSE);//ÉèÖÃ²»³É¹¦,¹Ø±ÕSocket
		return FALSE;//·µ»ØFALSE(0x00)
	}

	return TRUE;

	//ÖÁ´ËÍê³ÉÁËSocketµÄ´ò¿ªºÍÉèÖÃÕìÌı¹¤×÷,ÖÁÓÚÔ¶³Ì¿Í»§¶ËÊÇ·ñÓëËü½¨Á¢Á¬½Ó,ÔòĞèÒªµÈ´ıSocketÖĞ¶Ï£¬
	//ÒÔÅĞ¶ÏSocketµÄÁ¬½ÓÊÇ·ñ³É¹¦¡£²Î¿¼W5500Êı¾İÊÖ²áµÄSocketÖĞ¶Ï×´Ì¬
	//ÔÚ·şÎñÆ÷ÕìÌıÄ£Ê½²»ĞèÒªÉèÖÃÄ¿µÄIPºÍÄ¿µÄ¶Ë¿ÚºÅ
}

/*******************************************************************************
* º¯ÊıÃû  : Socket_UDP
* ÃèÊö    : ÉèÖÃÖ¸¶¨Socket(0~7)ÎªUDPÄ£Ê½
* ÊäÈë    : s:´ıÉè¶¨µÄ¶Ë¿Ú
* Êä³ö    : ÎŞ
* ·µ»ØÖµ  : ³É¹¦·µ»ØTRUE(0xFF),Ê§°Ü·µ»ØFALSE(0x00)
* ËµÃ÷    : Èç¹ûSocket¹¤×÷ÔÚUDPÄ£Ê½,ÒıÓÃ¸Ã³ÌĞò,ÔÚUDPÄ£Ê½ÏÂ,SocketÍ¨ĞÅ²»ĞèÒª½¨Á¢Á¬½Ó
*			¸Ã³ÌĞòÖ»µ÷ÓÃÒ»´Î£¬¾ÍÊ¹W5500ÉèÖÃÎªUDPÄ£Ê½
*******************************************************************************/
unsigned char Socket_UDP(SOCKET s)
{
	Write_W5500_SOCK_1Byte(s,Sn_MR,MR_UDP);//ÉèÖÃSocketÎªUDPÄ£Ê½*/
	Write_W5500_SOCK_1Byte(s,Sn_CR,OPEN);//´ò¿ªSocket*/
	Delay(5);//ÑÓÊ±5ms
	if(Read_W5500_SOCK_1Byte(s,Sn_SR)!=SOCK_UDP)//Èç¹ûSocket´ò¿ªÊ§°Ü
	{
		Write_W5500_SOCK_1Byte(s,Sn_CR,CLOSE);//´ò¿ª²»³É¹¦,¹Ø±ÕSocket
		return FALSE;//·µ»ØFALSE(0x00)
	}
	else
		return TRUE;

	//ÖÁ´ËÍê³ÉÁËSocketµÄ´ò¿ªºÍUDPÄ£Ê½ÉèÖÃ,ÔÚÕâÖÖÄ£Ê½ÏÂËü²»ĞèÒªÓëÔ¶³ÌÖ÷»ú½¨Á¢Á¬½Ó
	//ÒòÎªSocket²»ĞèÒª½¨Á¢Á¬½Ó,ËùÒÔÔÚ·¢ËÍÊı¾İÇ°¶¼¿ÉÒÔÉèÖÃÄ¿µÄÖ÷»úIPºÍÄ¿µÄSocketµÄ¶Ë¿ÚºÅ
	//Èç¹ûÄ¿µÄÖ÷»úIPºÍÄ¿µÄSocketµÄ¶Ë¿ÚºÅÊÇ¹Ì¶¨µÄ,ÔÚÔËĞĞ¹ı³ÌÖĞÃ»ÓĞ¸Ä±ä,ÄÇÃ´Ò²¿ÉÒÔÔÚÕâÀïÉèÖÃ
}

/*******************************************************************************
* º¯ÊıÃû  : W5500_Interrupt_Process
* ÃèÊö    : W5500ÖĞ¶Ï´¦Àí³ÌĞò¿ò¼Ü
* ÊäÈë    : ÎŞ
* Êä³ö    : ÎŞ
* ·µ»ØÖµ  : ÎŞ
* ËµÃ÷    : ÎŞ
*******************************************************************************/
void W5500_Interrupt_Process(void)
{
	unsigned char i,j;

IntDispose:
	W5500_Interrupt=0;//ÇåÁãÖĞ¶Ï±êÖ¾
	i = Read_W5500_1Byte(IR);//¶ÁÈ¡ÖĞ¶Ï±êÖ¾¼Ä´æÆ÷
	Write_W5500_1Byte(IR, (i&0xf0));//»ØĞ´Çå³ıÖĞ¶Ï±êÖ¾

	if((i & CONFLICT) == CONFLICT)//IPµØÖ·³åÍ»Òì³£´¦Àí
	{
		 //×Ô¼ºÌí¼Ó´úÂë
	}

	if((i & UNREACH) == UNREACH)//UDPÄ£Ê½ÏÂµØÖ·ÎŞ·¨µ½´ïÒì³£´¦Àí
	{
		//×Ô¼ºÌí¼Ó´úÂë
	}

	i=Read_W5500_1Byte(SIR);//¶ÁÈ¡¶Ë¿ÚÖĞ¶Ï±êÖ¾¼Ä´æÆ÷	
	if((i & S0_INT) == S0_INT)//Socket0ÊÂ¼ş´¦Àí 
	{
		j=Read_W5500_SOCK_1Byte(0,Sn_IR);//¶ÁÈ¡Socket0ÖĞ¶Ï±êÖ¾¼Ä´æÆ÷
		Write_W5500_SOCK_1Byte(0,Sn_IR,j);
		if(j&IR_CON)//ÔÚTCPÄ£Ê½ÏÂ,Socket0³É¹¦Á¬½Ó 
		{
			S0_State|=S_CONN;//ÍøÂçÁ¬½Ó×´Ì¬0x02,¶Ë¿ÚÍê³ÉÁ¬½Ó£¬¿ÉÒÔÕı³£´«ÊäÊı¾İ
		}
		if(j&IR_DISCON)//ÔÚTCPÄ£Ê½ÏÂSocket¶Ï¿ªÁ¬½Ó´¦Àí
		{
			Write_W5500_SOCK_1Byte(0,Sn_CR,CLOSE);//¹Ø±Õ¶Ë¿Ú,µÈ´ıÖØĞÂ´ò¿ªÁ¬½Ó 
			Socket_Init(0);		//Ö¸¶¨Socket(0~7)³õÊ¼»¯,³õÊ¼»¯¶Ë¿Ú0
			S0_State=0;//ÍøÂçÁ¬½Ó×´Ì¬0x00,¶Ë¿ÚÁ¬½ÓÊ§°Ü
		}
		if(j&IR_SEND_OK)//Socket0Êı¾İ·¢ËÍÍê³É,¿ÉÒÔÔÙ´ÎÆô¶¯S_tx_process()º¯Êı·¢ËÍÊı¾İ 
		{
			S0_Data|=S_TRANSMITOK;//¶Ë¿Ú·¢ËÍÒ»¸öÊı¾İ°üÍê³É 
		}
		if(j&IR_RECV)//Socket½ÓÊÕµ½Êı¾İ,¿ÉÒÔÆô¶¯S_rx_process()º¯Êı 
		{
			S0_Data|=S_RECEIVE;//¶Ë¿Ú½ÓÊÕµ½Ò»¸öÊı¾İ°ü
		}
		if(j&IR_TIMEOUT)//SocketÁ¬½Ó»òÊı¾İ´«Êä³¬Ê±´¦Àí 
		{
			Write_W5500_SOCK_1Byte(0,Sn_CR,CLOSE);// ¹Ø±Õ¶Ë¿Ú,µÈ´ıÖØĞÂ´ò¿ªÁ¬½Ó 
			S0_State=0;//ÍøÂçÁ¬½Ó×´Ì¬0x00,¶Ë¿ÚÁ¬½ÓÊ§°Ü
		}
	}

	if(Read_W5500_1Byte(SIR) != 0) 
		goto IntDispose;
}



