
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
	float error_P;
	float static error_I=0;
	float error_D=0;
	
	float static error_last=0;
	
	float static Input_last=0;
	
	float time_change;
	float D;
	float Duty;
	//Kp=1;
	//Ki=0.0001;
	//Kd=0;
		//VacuumValue_Set=0.3;
	//VacuumValue_Status=0;

	//In this part, we need to set our VacuumValue set to AD value, and we compare them.
	//VacuumValue_Set=Pressure627D_2_ADVoltage(VacuumValue_Set); //This is ad value
	//VacuumValue_Status=Pressure627D_2_ADVoltage(VacuumValue_Status);//this is ad  value
	
	
	printf("Set P parameter:%f\r\n",Kp);
	printf("Set I parameter:%f\r\n",Ki);
	printf("Set D parameter:%f\r\n",Kd);
	printf("Actual Pressure value:%f\r\n",VacuumValue_Status);
	printf("Set Pressure value:%f\r\n",VacuumValue_Set);
	
	

	
	time_change=0.1;
	error_P=(VacuumValue_Set-VacuumValue_Status)*Kp;
	
	error_I +=(VacuumValue_Set-VacuumValue_Status)*Ki;  //error sum

	error_D=(VacuumValue_Status-Input_last)*Kd;

	printf("error_P:%f\r\n",error_P);
	printf("error_I:%f\r\n",error_I);
	printf("error_D:%f\r\n",error_D);
	


	Input_last = VacuumValue_Status;
	
	printf("Inputlast:%f\r\n",Input_last);


		
	D = Kp * error_P + Ki * error_I + Kd * error_D; 
	
	//we think VacuumValueSet is 0.3 to 13.3  
	// vacuumvalue_Setatus   0.3 to 13.3
	//so error=-10 to 10
	//we want duty in 0-3600, if it is linear, we can use the formular below 

	//
	
	Duty=D*180 +1800;

	
	printf("Set Duty parameter:%f\r\n",Duty);
	//actually we need to call pwm duty change functiobn
	//Duty's range is from 0 to 3600,So we can set our duty to 0-3600 present 0% to 100% or
	//100% to 0%
		//we set the limit window so that the pwm adjustment will not be so dangerous
	if(Duty>2880)
		{
		Duty=2880;
		}
	if(Duty<720)
		{
		Duty=720;
		}

	
	TIM_SetCompare2(TIM3,Duty);

	
	return D;
		
	
}






