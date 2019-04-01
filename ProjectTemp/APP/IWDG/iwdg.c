#include "iwdg.h"
/*
 * 设置 IWDG 的超时时间
 * Tout = prv/40 * rlv (s)
 *      prv可以是[4,8,16,32,64,128,256]
 * prv:预分频器值，取值如下：
 *     @arg IWDG_Prescaler_4: IWDG prescaler set to 4
 *     @arg IWDG_Prescaler_8: IWDG prescaler set to 8
 *     @arg IWDG_Prescaler_16: IWDG prescaler set to 16
 *     @arg IWDG_Prescaler_32: IWDG prescaler set to 32
 *     @arg IWDG_Prescaler_64: IWDG prescaler set to 64
 *     @arg IWDG_Prescaler_128: IWDG prescaler set to 128
 *     @arg IWDG_Prescaler_256: IWDG prescaler set to 256
 *
 *        独立看门狗使用LSI作为时钟。
 *        LSI 的频率一般在 30~60KHZ 之间，根据温度和工作场合会有一定的漂移，我
 *        们一般取 40KHZ，所以独立看门狗的定时时间并一定非常精确，只适用于对时间精度
 *        要求比较低的场合。
 *
 * rlv:重装载寄存器的值，取值范围为：0-0XFFF
 * 函数调用举例：
 * IWDG_Config(IWDG_Prescaler_64 ,625);  // IWDG 1s 超时溢出 
 *                        （64/40）*625 = 1s
 */


void iwdg_init()	//独立看门狗初始化配置
{
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);//使能寄存器，写功能
	IWDG_SetPrescaler(IWDG_Prescaler_64);//设置IWDG预分频值//40K/64=1.6ms
	IWDG_SetReload(4000);//设置IWDG重装载值  12位的（0~4095）//800*1.6ms=1.28s set it to 4000 8.19s
	IWDG_ReloadCounter();//按照IWDG重装载寄存器的值重装载IWDG计数器
	IWDG_Enable();//使能IWDG
}
