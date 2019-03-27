#include "ds18b20.h"
u8 a,c,temp;

/*******************************************************************************
* 函 数 名         : ds18b20_init
* 函数功能		   : IO端口时钟初始化函数	   
* 输    入         : 无
* 输    出         : 无
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
* 函 数 名         : DQININT
* 函数功能		   : 输入配置	   
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void DQININT()	 //输入配置
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin=dq;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOG,&GPIO_InitStructure);	
}

/*******************************************************************************
* 函 数 名         : DQOUTINT
* 函数功能		   : 输出配置	   
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void DQOUTINT()	 //输出配置
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin=dq;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOG,&GPIO_InitStructure);	
}

/*******************************************************************************
* 函 数 名         : ds18b20init
* 函数功能		   : DS18B20初始化时序	   
* 输    入         : 无
* 输    出         : 无
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
* 函 数 名         : ds18b20wr
* 函数功能		   : DS18B20写数据时序	   
* 输    入         : dat
* 输    出         : 无
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
* 函 数 名         : DS18b20rd
* 函数功能		   : DS18B20读数据时序	   
* 输    入         : 无
* 输    出         : value
*******************************************************************************/
u8 DS18b20rd()
{
	u8 i,value=0;//一定要给value附初值否则显示会错误	
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
* 函 数 名         : readtemp
* 函数功能		   : DS18B2温度寄存器配置，温度读取	   
* 输    入         : 无
* 输    出         : b
*******************************************************************************/
u8 readtemp()			  //读取温度内需要复位的
{
	u8 b;
	ds18b20init();		//初始化
	ds18b20wr(0xcc);   //发送忽略ROM指令
	ds18b20wr(0x44);   //发送温度转换指令
	delay_us(1000);
	ds18b20init();	   //初始化
	ds18b20wr(0xcc);   //发送忽略ROM指令
	ds18b20wr(0xbe);   //发读暂存器指令
	a=DS18b20rd();	 //温度的低八位
	b=DS18b20rd();	 //温度的高八位
	b<<=4;			 //ssss s***；s为标志位s=0表示温度值为正数，s=1温度值为负数
	c=b&0x80;		//温度正负标志位确认
	b+=(a&0xf0)>>4;
	a=a&0x0f;	  //温度的小数部分
	return b;
}

