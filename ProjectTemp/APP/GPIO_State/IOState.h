#ifndef _IOState_H_
#define _IOState_H_


#include "stm32f10x.h"
//#define LED GPIO_Pin_14	 	//�ܽź궨��


void ValveState_Init(void);  //�˿ڳ�ʼ��
void ValveStateChange(u8 ValveSet_Value[12]);
char *Gas_State_Read(void);
char *Gas_State_Read_LCD(void);

	

#define Valve_GPIOA  GPIO_Pin_0|GPIO_Pin_1 |GPIO_Pin_6 |GPIO_Pin_7

#define Valve_GPIOB  GPIO_Pin_0|  GPIO_Pin_6 |GPIO_Pin_7


#define Valve_GPIOC  GPIO_Pin_13|  GPIO_Pin_14 |GPIO_Pin_15 |GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_11|GPIO_Pin_10|GPIO_Pin_9|GPIO_Pin_8


//order number ：    1		2 	  3    4   5    6   7   8     9   10   11    12   13   14   15   16
//state signal order:1  	12    2    3   11 	4 	10 	 5 	 9    8 	16  15   14    13    7    6
//port signal order：PC13  PC14  PC15  PC3  PA0  PA1 PA6 PA7 PC4  PB0	PC8 PC9  PC10  PC11  PB6  PB7
//Gas Valve State�˿ڶ���  ������ ����

#define Valve_1 PCout(13)
#define Valve_2 PCout(14)
#define Valve_3 PCout(15)
#define Valve_4 PCout(3)

#define Valve_5 PAout(0)
#define Valve_6 PAout(1)
#define Valve_7 PAout(6)
#define Valve_8 PAout(7)

#define Valve_9 PCout(4)

#define Valve_10 PBout(0)

#define Valve_11 PCout(8)
#define Valve_12 PCout(9)
#define Valve_13 PCout(10)
#define Valve_14 PCout(11)

#define Valve_15 PBout(6)
#define Valve_16 PBout(7)


	 
//λ������,ʵ��51���Ƶ�GPIO���ƹ���
//����ʵ��˼��,�ο�<<CM3Ȩ��ָ��>>������(87ҳ~92ҳ).
//IO�ڲ����궨��
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 
//IO�ڵ�ַӳ��
#define GPIOA_ODR_Addr    (GPIOA_BASE+12) //0x4001080C 
#define GPIOB_ODR_Addr    (GPIOB_BASE+12) //0x40010C0C 
#define GPIOC_ODR_Addr    (GPIOC_BASE+12) //0x4001100C 
#define GPIOD_ODR_Addr    (GPIOD_BASE+12) //0x4001140C 
#define GPIOE_ODR_Addr    (GPIOE_BASE+12) //0x4001180C 
#define GPIOF_ODR_Addr    (GPIOF_BASE+12) //0x40011A0C    
#define GPIOG_ODR_Addr    (GPIOG_BASE+12) //0x40011E0C    

#define GPIOA_IDR_Addr    (GPIOA_BASE+8) //0x40010808 
#define GPIOB_IDR_Addr    (GPIOB_BASE+8) //0x40010C08 
#define GPIOC_IDR_Addr    (GPIOC_BASE+8) //0x40011008 
#define GPIOD_IDR_Addr    (GPIOD_BASE+8) //0x40011408 
#define GPIOE_IDR_Addr    (GPIOE_BASE+8) //0x40011808 
#define GPIOF_IDR_Addr    (GPIOF_BASE+8) //0x40011A08 
#define GPIOG_IDR_Addr    (GPIOG_BASE+8) //0x40011E08 
 
//IO�ڲ���,ֻ�Ե�һ��IO��!
//ȷ��n��ֵС��16!
#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //��� 
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  //���� 

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //��� 
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //���� 

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  //��� 
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  //���� 

#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  //��� 
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  //���� 

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  //��� 
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  //����

#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  //��� 
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  //����

#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  //��� 
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  //����



#endif 


