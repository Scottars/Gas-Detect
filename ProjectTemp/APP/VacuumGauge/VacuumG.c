
#include "VacuumG.h"
#include "dac.h"
#include "printf.h"

/*************************************************
*这个文件的主要的目的就是希望能够实现闭环的控制，相关的函数等等
*
*
*
*
*
*************************************************/





/*******************************************************************************
* 函数名  : VacuumValue_Adapt
* 描述    : 对给定的真空度，对PV罚的电压进行设定，通过调用目标的电压值
* 输入    : 真空度设定值VacuumValue_Set VacuumValue_Measure  采用哪个值进�
* 输出    : 无
* 返回值  : 无
* 说明    : 这个希望能够调用PWM波的产生程序，通过PID控制对占空比进行调节，来控制来自
*						boost电路的输出电压
						如何对两个闭环进行区分，直接以真空度为标准 或者 以boost 的调节的电路为标准
						
*******************************************************************************/
float VacuumValue_PID(float VacuumValue_Set, float VacuumValue_Status,float Kp,float Ki, float Kd)
{	
	float error;
	float time_now=0;
	float static time_last=0;
	float time_change;
	float D;
	//Kp=1;
	//Ki=0;
	//Kd=0;
	printf("Set P parameter:%f\r\n",Kp);
	printf("Set I parameter:%f\r\n",Ki);
	printf("Set D parameter:%f\r\n",Kd);
	
	
	//VacuumValue_Set=0.3;
	//VacuumValue_Status=0.5;
	
	time_change=0.1;
	error=VacuumValue_Set-VacuumValue_Status;
	
	
	D = Kp * error + Ki * error*time_change + Kd * error/time_change;  
	printf("Set Duty parameter:%f\r\n",D);
	//actually we need to call pwm duty change functiobn
	TIM_SetCompare2(TIM1,0);

	
	return D;
		
	
}






