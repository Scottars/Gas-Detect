#include  "IOState.h"
#include "sys.h"
#include "printf.h"



void ValveState_Init()    //¶Ë¿Ú³õÊ¼»¯
{
    GPIO_InitTypeDef GPIO_InitStructure;    //ÉùÃ÷Ò»¸ö½á¹¹Ìå±äÁ¿£¬ÓÃÀ´³õÊ¼»¯GPIO

    SystemInit();   //ÏµÍ³Ê±ÖÓ³õÊ¼»¯

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |RCC_APB2Periph_GPIOC,ENABLE); /* ¿ªÆôGPIOÊ±ÖÓ */

    /*  ÅäÖÃGPIOµÄÄ£Ê½ºÍIO¿Ú */
    GPIO_InitStructure.GPIO_Pin=Valve_GPIOA;     //Ñ¡ÔñÄãÒªÉèÖÃµÄIO¿Ú
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;    //ÉèÖÃÍÆÍìÊä³öÄ£Ê½
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;   //ÉèÖÃ´«ÊäËÙÂÊ
    GPIO_Init(GPIOA,&GPIO_InitStructure); /* ³õÊ¼»¯GPIO */
    /*  ÅäÖÃGPIOµÄÄ£Ê½ºÍIO¿Ú */
    GPIO_InitStructure.GPIO_Pin=Valve_GPIOB;     //Ñ¡ÔñÄãÒªÉèÖÃµÄIO¿Ú
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;    //ÉèÖÃÍÆÍìÊä³öÄ£Ê½
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;   //ÉèÖÃ´«ÊäËÙÂÊ
    GPIO_Init(GPIOB,&GPIO_InitStructure); /* ³õÊ¼»¯GPIO */

    /*  ÅäÖÃGPIOµÄÄ£Ê½ºÍIO¿Ú */
    GPIO_InitStructure.GPIO_Pin=Valve_GPIOC;     //Ñ¡ÔñÄãÒªÉèÖÃµÄIO¿Ú
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;    //ÉèÖÃÍÆÍìÊä³öÄ£Ê½
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;   //ÉèÖÃ´«ÊäËÙÂÊ
    GPIO_Init(GPIOC,&GPIO_InitStructure); /* ³õÊ¼»¯GPIO */
}


void ValveStateChange(u8 Valve_Status_Set[2] )
{
    //À´×ÔÍøÂçµÄĞÅÏ¢´«ÈëÁË½øÀ´£¬È»ºóÍ¨¹ıÍøÂçËùÉè¶¨µÄ×´Ì¬£¬ÎÒÃÇ½øĞĞÏÂÃæµÄ¸üĞÂ
    //ÎªÁËµ÷ÊÔÊ¹ÓÃ
    //1 2 6



    /*
        Valve_6=0;
        printf("Wirking on GPIO state test!");
        delay_ms(1000);


        Valve_6=1;
        delay_ms(1000);*/
    ////ÏÂÃæÕıÊ½Ê¹ÓÃ£¬Î»´ø²Ù×÷¶ÔÓ¦µÄ°´Å¥///////
    /*
	//state signal order:1		12	  2    3   11	4	10	 5	 9	  8 	16	15	 14    13	 7	  6
	//port signal orderï¼šPC13	PC14  PC15	PC3  PA0  PA1 PA6 PA7 PC4  PB0	PC8 PC9  PC10  PC11  PB6  PB7
	//Gas Valve State¶Ë¿Ú¶¨Òå  ´Ó×óµ½ÓÒ ÒÀ´Î
	
#define Valve_1 PCout(13)
#define Valve_2 PCout(15)
#define Valve_3 PCout(3)
							
#define Valve_4 PAout(1)
#define Valve_5 PAout(7)
							
#define Valve_6 PBout(7)
#define Valve_7 PBout(6)
#define Valve_8 PBout(0)
							
#define Valve_9 PCout(4)
							
#define Valve_10 PAout(6)
#define Valve_11 PAout(0)
							
#define Valve_12 PCout(14)
#define Valve_13 PCout(11)
#define Valve_14 PCout(10)
#define Valve_15 PCout(9)
#define Valve_16 PCout(8)

    */
    u8 ValveSet_Value[16];
    u8 i;
    u8 s=0;

    for(i=0; i<8; i++)
    {
        if(((Valve_Status_Set[1]>>i)& 0x01)==0x01)   //è®¾å®šå‘å³ç§»ä½ï¼Œä»å³åˆ°å·¦ï¼Œåˆ†åˆ«è¡¨ç¤º12è·¯å…‰çº¤çš„çŠ¶æ€   ä¾æ¬¡å–è®¾å®šçš„æ•°å€¼
            ValveSet_Value[i]=1;
        else
            ValveSet_Value[i]=0;
    }
    for(i=0; i<8; i++)
    {
        if(((Valve_Status_Set[0]>>i)& 0x01)==0x01)
            ValveSet_Value[i+8]=1;
        else
            ValveSet_Value[i+8]=0;
    }

    if (ValveSet_Value[0] == 1)
    {
        Valve_1=1; //ÉèÖÃ¸ßµçÆ½ÎªÁÁ
    }
    else
    {
        Valve_1=0;

    }
    if (ValveSet_Value[1] == 1)
    {
        Valve_2=1; //ÉèÖÃ¸ßµçÆ½ÎªÁÁ
    }
    else
    {
        Valve_2=0;

    }
    if (ValveSet_Value[2] == 1)
    {
        Valve_3=1; //ÉèÖÃ¸ßµçÆ½ÎªÁÁ
    }
    else
    {
        Valve_3=0;

    }
    if (ValveSet_Value[3] == 1)
    {
        Valve_4=1; //ÉèÖÃ¸ßµçÆ½ÎªÁÁ
    }
    else
    {
        Valve_4=0;

    }
    if (ValveSet_Value[4] == 1)
    {
        Valve_5=1; //ÉèÖÃ¸ßµçÆ½ÎªÁÁ
    }
    else
    {
        Valve_5=0;

    }
    if (ValveSet_Value[5] == 1)
    {
        Valve_6=1; //ÉèÖÃ¸ßµçÆ½ÎªÁÁ
    }
    else
    {
        Valve_6=0;

    }
    if (ValveSet_Value[6] == 1)
    {
        Valve_7=1; //ÉèÖÃ¸ßµçÆ½ÎªÁÁ
    }
    else
    {
        Valve_7=0;

    }
    if (ValveSet_Value[7] == 1)
    {
        Valve_8=1; //ÉèÖÃ¸ßµçÆ½ÎªÁÁ
    }
    else
    {
        Valve_8=0;

    }
    if (ValveSet_Value[8] == 1)
    {
        Valve_9=1; //ÉèÖÃ¸ßµçÆ½ÎªÁÁ
    }
    else
    {
        Valve_9=0;

    }
    if (ValveSet_Value[9] == 1)
    {
        Valve_10=1; //ÉèÖÃ¸ßµçÆ½ÎªÁÁ
    }
    else
    {
        Valve_10=0;

    }
    if (ValveSet_Value[10] == 1)
    {
        Valve_11=1; //ÉèÖÃ¸ßµçÆ½ÎªÁÁ
    }
    else
    {
        Valve_11=0;

    }
    if (ValveSet_Value[11] == 1)
    {
        Valve_12=1; //ÉèÖÃ¸ßµçÆ½ÎªÁÁ
    }
    else
    {
        Valve_12=0;

    }
	if (ValveSet_Value[12] == 1)
    {
        Valve_13=1; //ÉèÖÃ¸ßµçÆ½ÎªÁÁ
    }
    else
    {
        Valve_13=0;

    }
    if (ValveSet_Value[13] == 1)
    {
        Valve_14=1; //ÉèÖÃ¸ßµçÆ½ÎªÁÁ
    }
    else
    {
        Valve_14=0;

    }
	if (ValveSet_Value[14] == 1)
    {
        Valve_14=1; //ÉèÖÃ¸ßµçÆ½ÎªÁÁ
    }
    else
    {
        Valve_14=0;

    }
    if (ValveSet_Value[15] == 1)
    {
        Valve_16=1; //ÉèÖÃ¸ßµçÆ½ÎªÁÁ
    }
    else
    {
        Valve_16=0;

    }




}
char *Gas_State_Read()
{
    /*
	
	
	//order number ï¼š	  1 	2	  3    	4     5	    6	7	 8	  9   10    11	 12   13    14	  15	 16
	//state signal order:1		12	  2   	 3   11		4	10	 5	  9	  8 	16	 15	  14    13	   7	  6
	//port signal orderï¼šPC13	PC14  PC15	PC3  PA0  PA1  PA6  PA7  PC4  PB0	PC8  PC9  PC10  PC11  PB6    PB7
	//Gas Valve State¶Ë¿Ú¶¨Òå  ´Ó×óµ½ÓÒ ÒÀ´Î

    */

    unsigned char static GPIO_State_return[2]= {0x00,0x00};
//printf("GPIO test in Internet");

//·½Ê½¶ş£º
    if(GPIO_ReadOutputDataBit(GPIOC,GPIO_Pin_13)==1)
    {
        //printf("GPIO test in pin0");
        GPIO_State_return[1] = (GPIO_State_return[1]) | 0x01;
    }
    if(GPIO_ReadOutputDataBit(GPIOC,GPIO_Pin_14)==1)
    {
        //  printf("GPIO test in pin1");
        GPIO_State_return[1] = (GPIO_State_return[1])| 0x02;
    }
    if(GPIO_ReadOutputDataBit(GPIOC,GPIO_Pin_15)==1)
    {
        GPIO_State_return[1] = (GPIO_State_return[1])| 0x04;
    }
    if(GPIO_ReadOutputDataBit(GPIOC,GPIO_Pin_3)==1)
    {
        GPIO_State_return[1] = (GPIO_State_return[1])| 0x08;
    }
    if(GPIO_ReadOutputDataBit(GPIOA,GPIO_Pin_0)==1)
    {
        GPIO_State_return[1] = (GPIO_State_return[1])| 0x10;
    }
    if(GPIO_ReadOutputDataBit(GPIOA,GPIO_Pin_1)==1)
    {
        GPIO_State_return[1] = (GPIO_State_return[1])| 0x20;
    }
    if(GPIO_ReadOutputDataBit(GPIOA,GPIO_Pin_6)==1)
    {
        GPIO_State_return[1] = (GPIO_State_return[1]) | 0x40;
    }
    if(GPIO_ReadOutputDataBit(GPIOA,GPIO_Pin_7)==1)
    {
        GPIO_State_return[1] = (GPIO_State_return[1])| 0x80;
    }
    if(GPIO_ReadOutputDataBit(GPIOC,GPIO_Pin_4)==1)
    {
        GPIO_State_return[0] = (GPIO_State_return[0])| 0x01;
    }
    if(GPIO_ReadOutputDataBit(GPIOB,GPIO_Pin_0)==1)
    {
        GPIO_State_return[0] = (GPIO_State_return[0])| 0x02;
    }
    if(GPIO_ReadOutputDataBit(GPIOC,GPIO_Pin_8)==1)
    {
        GPIO_State_return[0] = (GPIO_State_return[0])| 0x04;
    }
    if(GPIO_ReadOutputDataBit(GPIOC,GPIO_Pin_9)==1)
    {
        GPIO_State_return[0] = (GPIO_State_return[0])| 0x08;
    }
    if(GPIO_ReadOutputDataBit(GPIOC,GPIO_Pin_10)==1)
    {
        GPIO_State_return[0] = (GPIO_State_return[1])| 0x10;
    }
    if(GPIO_ReadOutputDataBit(GPIOC,GPIO_Pin_11)==1)
    {
        GPIO_State_return[0] = (GPIO_State_return[1])| 0x20;
    }
    if(GPIO_ReadOutputDataBit(GPIOB,GPIO_Pin_6)==1)
    {
        GPIO_State_return[0] = (GPIO_State_return[1]) | 0x40;
    }
    if(GPIO_ReadOutputDataBit(GPIOB,GPIO_Pin_7)==1)
    {
        GPIO_State_return[0] = (GPIO_State_return[1])| 0x80;
    }


    return GPIO_State_return ;

}
char *Gas_State_Read_LCD()
{
    /*#define Valve_1 PCout(13)     // Valve1  singal 1   PC13    ----------P7ÅÅÏß
	
	
	//order number ï¼š	  1 	2	  3    	4     5	    6	7	 8	  9   10    11	 12   13    14	  15	 16
	//state signal order:1		12	  2   	 3   11		4	10	 5	  9	  8 	16	 15	  14    13	   7	  6
	//port signal orderï¼šPC13	PC14  PC15	PC3  PA0  PA1  PA6  PA7  PC4  PB0	PC8  PC9  PC10  PC11  PB6    PB7
	//Gas Valve State¶Ë¿Ú¶¨Òå  ´Ó×óµ½ÓÒ ÒÀ´Î

    */

    char static GPIO_State[12];

    GPIO_State[0]=GPIO_ReadOutputDataBit(GPIOC,GPIO_Pin_13);
    GPIO_State[1]=GPIO_ReadOutputDataBit(GPIOC,GPIO_Pin_14);

    GPIO_State[2]=GPIO_ReadOutputDataBit(GPIOC,GPIO_Pin_15);
    GPIO_State[3]=GPIO_ReadOutputDataBit(GPIOC,GPIO_Pin_3);

    GPIO_State[4]=GPIO_ReadOutputDataBit(GPIOA,GPIO_Pin_0);
    GPIO_State[5]=GPIO_ReadOutputDataBit(GPIOA,GPIO_Pin_1);

    GPIO_State[6]=GPIO_ReadOutputDataBit(GPIOA,GPIO_Pin_6);
    GPIO_State[7]=GPIO_ReadOutputDataBit(GPIOA,GPIO_Pin_7);

    GPIO_State[8]=GPIO_ReadOutputDataBit(GPIOC,GPIO_Pin_4);
    GPIO_State[9]=GPIO_ReadOutputDataBit(GPIOB,GPIO_Pin_0);

    GPIO_State[10]=GPIO_ReadOutputDataBit(GPIOC,GPIO_Pin_8);
    GPIO_State[11]=GPIO_ReadOutputDataBit(GPIOC,GPIO_Pin_9);
	
	GPIO_State[8]=GPIO_ReadOutputDataBit(GPIOC,GPIO_Pin_10);
	GPIO_State[9]=GPIO_ReadOutputDataBit(GPIOC,GPIO_Pin_11);
	
	GPIO_State[10]=GPIO_ReadOutputDataBit(GPIOB,GPIO_Pin_6);
	GPIO_State[11]=GPIO_ReadOutputDataBit(GPIOB,GPIO_Pin_7);


    return GPIO_State ;

}
