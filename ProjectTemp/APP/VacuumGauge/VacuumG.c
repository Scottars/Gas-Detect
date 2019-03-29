
#include "VacuumG.h"

#include "printf.h"

/*************************************************
*����ļ�����Ҫ��Ŀ�ľ���ϣ���ܹ�ʵ�ֱջ��Ŀ��ƣ���صĺ����ȵ�
*
*
*
*
*
*************************************************/





/*******************************************************************************
* ������  : VacuumValue_Adapt
* ����    : �Ը�������նȣ���PV���ĵ�ѹ�����趨��ͨ������Ŀ��ĵ�ѹֵ
* ����    : ��ն��趨ֵVacuumValue_Set VacuumValue_Measure  �����ĸ�ֵ���
* ���    : ��
* ����ֵ  : ��
* ˵��    : ���ϣ���ܹ�����PWM���Ĳ�������ͨ��PID���ƶ�ռ�ձȽ��е��ڣ�����������
*						boost��·�������ѹ
						��ζ������ջ��������֣�ֱ������ն�Ϊ��׼ ���� ��boost �ĵ��ڵĵ�·Ϊ��׼
						
*******************************************************************************/
float VacuumValue_PID(float VacuumValue_Set, float VacuumValue_Status,float Kp,float Ki, float Kd)
{	
	float error;
	float time_now=0;
	float static time_last=0;
	float time_change;
	float D;
	float Duty;
	Kp=1;
	Ki=0;
	Kd=0;
	printf("Set P parameter:%f\r\n",Kp);
	printf("Set I parameter:%f\r\n",Ki);
	printf("Set D parameter:%f\r\n",Kd);
	printf("Set Pressure value:%f\r\n",VacuumValue_Set);
	
	
	
	//VacuumValue_Set=0.3;
	//VacuumValue_Status=0.5;
	
	time_change=0.1;
	error=VacuumValue_Set-VacuumValue_Status;
	
	
	D = Kp * error + Ki * error*time_change + Kd * error/time_change; 
	//we think VacuumValueSet is 0.3 to 13.3  
	// vacuumvalue_Setatus   0.3 to 13.3
	//so error=-10 to 10
	//we want duty in 0-3600, if it is linear, we can use the formular below
	Duty=D*180 +1800;

	
	printf("Set Duty parameter:%f\r\n",D);
	//actually we need to call pwm duty change functiobn
	//Duty's range is from 0 to 3600,So we can set our duty to 
	
	TIM_SetCompare2(TIM3,Duty);

	
	return D;
		
	
}






