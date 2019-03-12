#include  "IOState.h"
#include "sys.h"
#include "printf.h"



void ValveState_Init()	  //�˿ڳ�ʼ��
{
	GPIO_InitTypeDef GPIO_InitStructure;	//����һ���ṹ�������������ʼ��GPIO

	SystemInit();	//ϵͳʱ�ӳ�ʼ��

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |RCC_APB2Periph_GPIOC,ENABLE); /* ����GPIOʱ�� */

	/*  ����GPIO��ģʽ��IO�� */
	GPIO_InitStructure.GPIO_Pin=Valve_GPIOA;	 //ѡ����Ҫ���õ�IO��
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;	  //�����������ģʽ
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;	  //���ô�������
	GPIO_Init(GPIOA,&GPIO_InitStructure); /* ��ʼ��GPIO */	
	/*  ����GPIO��ģʽ��IO�� */
	GPIO_InitStructure.GPIO_Pin=Valve_GPIOB;	 //ѡ����Ҫ���õ�IO��
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;	  //�����������ģʽ
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;	  //���ô�������
	GPIO_Init(GPIOB,&GPIO_InitStructure); /* ��ʼ��GPIO */	
	
	/*  ����GPIO��ģʽ��IO�� */
	GPIO_InitStructure.GPIO_Pin=Valve_GPIOC;	 //ѡ����Ҫ���õ�IO��
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;	  //�����������ģʽ
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;	  //���ô�������
	GPIO_Init(GPIOC,&GPIO_InitStructure); /* ��ʼ��GPIO */	
}


void ValveStateChange(u8 ValveSet_Value[12] )
{
	//�����������Ϣ�����˽�����Ȼ��ͨ���������趨��״̬�����ǽ�������ĸ���
	//Ϊ�˵���ʹ��
	//1 2 6
	
/*
	Valve_6=0;
	printf("Wirking on GPIO state test!");
	delay_ms(1000);
	

	Valve_6=1;
	delay_ms(1000);*/
	////������ʽʹ�ã�λ��������Ӧ�İ�ť///////
	/*
#define Valve_1 PCout(13)		// Valve1  singal 1   PC13    ----------P7����   ��ӦLED0
#define Valve_2 PCout(15)//Valve2  singal 2   PC15    ----------P7����      ��Ӧ LED2
#define Valve_3 PCout(3)// Valve3  singal 3   PC3    ----------P7����
#define Valve_4 PAout(1)//Valve4  singal 4   PA1    ----------P7����
#define Valve_5 PAout(0)// Valve5  singal 11   PA0    ----------P7����
#define Valve_6 PCout(14)//Valve6  singal 12   PC14    ----------P7����    ��ӦLED1


#define Valve_7 PAout(7)// Valve7  singal 5   PA7   ----------P8����
#define Valve_8 PBout(0)//Valve8  singal 8   PB0   ----------P8����
#define Valve_9 PCout(4)// Valve9  singal 9   PC4   ----------P8����
#define Valve_10 PAout(6)//Valve10  singal 10   PA6   ----------P8����



#define Valve_11 PBout(7)// Valve11  singal 6   PB7   ----------P11����
#define Valve_12 PBout(6)//Valve12  singal 7   PB6   ----------P11����
	*/
	
	
	if (ValveSet_Value[0] == 1)
	{
		Valve_1=1; //���øߵ�ƽΪ��
	}
	else
	{
		Valve_1=0;
		
	}
		if (ValveSet_Value[1] == 1)
	{
		Valve_2=1; //���øߵ�ƽΪ��
	}
	else
	{
		Valve_2=0;
		
	}
		if (ValveSet_Value[2] == 1)
	{
		Valve_3=1; //���øߵ�ƽΪ��
	}
	else
	{
		Valve_3=0;
		
	}
		if (ValveSet_Value[3] == 1)
	{
		Valve_4=1; //���øߵ�ƽΪ��
	}
	else
	{
		Valve_4=0;
		
	}
		if (ValveSet_Value[4] == 1)
	{
		Valve_5=1; //���øߵ�ƽΪ��
	}
	else
	{
		Valve_5=0;
		
	}
		if (ValveSet_Value[5] == 1)
	{
		Valve_6=1; //���øߵ�ƽΪ��
	}
	else
	{
		Valve_6=0;
		
	}
		if (ValveSet_Value[6] == 1)
	{
		Valve_7=1; //���øߵ�ƽΪ��
	}
	else
	{
		Valve_7=0;
		
	}
		if (ValveSet_Value[7] == 1)
	{
		Valve_8=1; //���øߵ�ƽΪ��
	}
	else
	{
		Valve_8=0;
		
	}
		if (ValveSet_Value[8] == 1)
	{
		Valve_9=1; //���øߵ�ƽΪ��
	}
	else
	{
		Valve_9=0;
		
	}
		if (ValveSet_Value[9] == 1)
	{
		Valve_10=1; //���øߵ�ƽΪ��
	}
	else
	{
		Valve_10=0;
		
	}
			if (ValveSet_Value[10] == 1)
	{
		Valve_11=1; //���øߵ�ƽΪ��
	}
	else
	{
		Valve_11=0;
		
	}
		if (ValveSet_Value[11] == 1)
	{
		Valve_12=1; //���øߵ�ƽΪ��
	}
	else
	{
		Valve_12=0;
		
	}

		
	
	
}
u8 *Gas_State_Read()
{
	/*
	#define Valve_1 PCout(13)		// Valve1  singal 1   PC13    ----------P7����
#define Valve_2 PCout(15)//Valve2  singal 2   PC15    ----------P7����
#define Valve_3 PCout(3)// Valve3  singal 3   PC3    ----------P7����
#define Valve_4 PAout(1)//Valve4  singal 4   PA1    ----------P7����
#define Valve_5 PAout(0)// Valve5  singal 11   PA0    ----------P7����
#define Valve_6 PCout(14)//Valve6  singal 12   PC14    ----------P7����


#define Valve_7 PAout(7)// Valve7  singal 5   PA7   ----------P8����
#define Valve_8 PBout(0)//Valve8  singal 8   PB0   ----------P8����
#define Valve_9 PCout(4)// Valve9  singal 9   PC4   ----------P8����
#define Valve_10 PAout(6)//Valve10  singal 10   PA6   ----------P8����



#define Valve_11 PBout(7)// Valve11  singal 6   PB7   ----------P11����
#define Valve_12 PBout(6)//Valve12  singal 7   PB6   ----------P11����
*/
	
	u8 static GPIO_State[12];
	
	
	GPIO_State[0]=GPIO_ReadOutputDataBit(GPIOC,GPIO_Pin_13);
	GPIO_State[1]=GPIO_ReadOutputDataBit(GPIOC,GPIO_Pin_15);
		
	GPIO_State[2]=GPIO_ReadOutputDataBit(GPIOC,GPIO_Pin_3);
	GPIO_State[3]=GPIO_ReadOutputDataBit(GPIOA,GPIO_Pin_1);
		
	GPIO_State[4]=GPIO_ReadOutputDataBit(GPIOA,GPIO_Pin_0);
	GPIO_State[5]=GPIO_ReadOutputDataBit(GPIOC,GPIO_Pin_14);
	
	
	
	GPIO_State[6]=GPIO_ReadOutputDataBit(GPIOA,GPIO_Pin_7);
	GPIO_State[7]=GPIO_ReadOutputDataBit(GPIOB,GPIO_Pin_0);
	
	GPIO_State[8]=GPIO_ReadOutputDataBit(GPIOC,GPIO_Pin_4);
	GPIO_State[9]=GPIO_ReadOutputDataBit(GPIOA,GPIO_Pin_6);
		
	GPIO_State[10]=GPIO_ReadOutputDataBit(GPIOB,GPIO_Pin_7);
	GPIO_State[11]=GPIO_ReadOutputDataBit(GPIOB,GPIO_Pin_6);
	
	
	return GPIO_State ;
	
}
