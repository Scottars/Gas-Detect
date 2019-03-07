#include "ds18b20.h"
u8 a,c,temp;

/*******************************************************************************
* �� �� ��         : ds18b20_init
* ��������		   : IO�˿�ʱ�ӳ�ʼ������	   
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
void ds18b20_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG,ENABLE);

	GPIO_InitStructure.GPIO_Pin=dq;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOG,&GPIO_InitStructure);
}

/*******************************************************************************
* �� �� ��         : DQININT
* ��������		   : ��������	   
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
void DQININT()	 //��������
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin=dq;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOG,&GPIO_InitStructure);	
}

/*******************************************************************************
* �� �� ��         : DQOUTINT
* ��������		   : �������	   
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
void DQOUTINT()	 //�������
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin=dq;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOG,&GPIO_InitStructure);	
}

/*******************************************************************************
* �� �� ��         : ds18b20init
* ��������		   : DS18B20��ʼ��ʱ��	   
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
void ds18b20init()
{
	DQOUTINT();
	GPIO_SetBits(GPIOG,dq);
	delay_us(10);	
	GPIO_ResetBits(GPIOG,dq);
	delay_us(600);
	GPIO_SetBits(GPIOG,dq);
	delay_us(50);
	GPIO_ResetBits(GPIOG,dq);
	delay_us(200);
	GPIO_SetBits(GPIOG,dq);
	delay_us(400);
}

/*******************************************************************************
* �� �� ��         : ds18b20wr
* ��������		   : DS18B20д����ʱ��	   
* ��    ��         : dat
* ��    ��         : ��
*******************************************************************************/
void ds18b20wr(u8 dat)
{
	u8 i;
	DQOUTINT();
	GPIO_SetBits(GPIOG,dq);
	delay_us(10);
	for(i=0;i<8;i++)		
	{
		GPIO_ResetBits(GPIOG,dq);
		if(dat&0x01==1)
		{
			GPIO_SetBits(GPIOG,dq);	
		}
		else
			GPIO_ResetBits(GPIOG,dq);
		dat>>=1;
		delay_us(60);
		GPIO_SetBits(GPIOG,dq);
		delay_us(10);	
	}
	GPIO_SetBits(GPIOG,dq);
	delay_us(10);
}

/*******************************************************************************
* �� �� ��         : DS18b20rd
* ��������		   : DS18B20������ʱ��	   
* ��    ��         : ��
* ��    ��         : value
*******************************************************************************/
u8 DS18b20rd()
{
	u8 i,value=0;//һ��Ҫ��value����ֵ������ʾ�����	
	DQOUTINT();
	GPIO_SetBits(GPIOG,dq);
	for(i=0;i<8;i++)		
	{	
		GPIO_ResetBits(GPIOG,dq);
		value>>=1;
		GPIO_SetBits(GPIOG,dq);
		DQININT();
		if(GPIO_ReadInputDataBit(GPIOG,dq)==1)
		{
			value|=0x80;
		}
		delay_us(50);
		DQOUTINT();	
		GPIO_SetBits(GPIOG,dq);
		delay_us(1);	
	}
	GPIO_SetBits(GPIOG,dq);
	delay_us(10);
	return value;	
}

/*******************************************************************************
* �� �� ��         : readtemp
* ��������		   : DS18B2�¶ȼĴ������ã��¶ȶ�ȡ	   
* ��    ��         : ��
* ��    ��         : b
*******************************************************************************/
u8 readtemp()			  //��ȡ�¶�����Ҫ��λ��
{
	u8 b;
	ds18b20init();		//��ʼ��
	ds18b20wr(0xcc);   //���ͺ���ROMָ��
	ds18b20wr(0x44);   //�����¶�ת��ָ��
	delay_us(1000);
	ds18b20init();	   //��ʼ��
	ds18b20wr(0xcc);   //���ͺ���ROMָ��
	ds18b20wr(0xbe);   //�����ݴ���ָ��
	a=DS18b20rd();	 //�¶ȵĵͰ�λ
	b=DS18b20rd();	 //�¶ȵĸ߰�λ
	b<<=4;			 //ssss s***��sΪ��־λs=0��ʾ�¶�ֵΪ������s=1�¶�ֵΪ����
	c=b&0x80;		//�¶�������־λȷ��
	b+=(a&0xf0)>>4;
	a=a&0x0f;	  //�¶ȵ�С������
	return b;
}

