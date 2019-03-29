
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






