#ifndef _SPI_H_
#define _SPI_H_


#include "stm32f10x.h"
//#define LED GPIO_Pin_14	 	//管脚宏定义


//#define Value_GPIOC  GPIO_Pin_3|  GPIO_Pin_4 |

//Gas Valve State端口定义  从左到右 依次 
#define Valve_1 PCout(13)		// Valve1  singal 1   PC13    ----------P7排线
#define Valve_2 PCout(14)//Valve2  singal 2   PC14    ----------P7排线
#define Valve_3 PCout(3)// Valve3  singal 3   PC3    ----------P7排线
#define Valve_4 PAout(1)//Valve4  singal 4   PA1    ----------P7排线
#define Valve_5 PAout(0)// Valve5  singal 11   PA0    ----------P7排线
#define Valve_6 PCout(14)//Valve6  singal 12   PC14    ----------P7排线


#define Valve_7 PAout(7)// Valve7  singal 5   PA7   ----------P8排线
#define Valve_8 PBout(0)//Valve8  singal 8   PB0   ----------P8排线
#define Valve_9 PCout(4)// Valve9  singal 9   PC4   ----------P8排线
#define Valve_10 PAout(6)//Valve10  singal 10   PA6   ----------P8排线



#define Valve_11 PBout(7)// Valve11  singal 6   PB7   ----------P11排线
#define Valve_12 PBout(6)//Valve12  singal 7   PB6   ----------P11排线


	 
//位带操作,实现51类似的GPIO控制功能
//具体实现思想,参考<<CM3权威指南>>第五章(87页~92页).
//IO口操作宏定义
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 
//IO口地址映射
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
 
//IO口操作,只对单一的IO口!
//确保n的值小于16!
#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //输出 
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  //输入 

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //输出 
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //输入 

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  //输出 
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  //输入 

#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  //输出 
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  //输入 

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  //输出 
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  //输入

#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  //输出 
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  //输入

#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  //输出 
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  //输入



#endif 

#endif
