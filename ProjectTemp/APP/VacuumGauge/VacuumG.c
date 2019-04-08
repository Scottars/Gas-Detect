
#include "VacuumG.h"

#include "printf.h"

/*************************************************
*Õâ¸öÎÄ¼şµÄÖ÷ÒªµÄÄ¿µÄ¾ÍÊÇÏ£ÍûÄÜ¹»ÊµÏÖ±Õ»·µÄ¿ØÖÆ£¬Ïà¹ØµÄº¯ÊıµÈµÈ
*
*
*
*
*
*************************************************/





/*******************************************************************************
* º¯ÊıÃû  : VacuumValue_Adapt
* ÃèÊö    : ¶Ô¸ø¶¨µÄÕæ¿Õ¶È£¬¶ÔPV·£µÄµçÑ¹½øĞĞÉè¶¨£¬Í¨¹ıµ÷ÓÃÄ¿±êµÄµçÑ¹Öµ
* ÊäÈë    : Õæ¿Õ¶ÈÉè¶¨ÖµVacuumValue_Set VacuumValue_Measure  ²ÉÓÃÄÄ¸öÖµ½øĞ
* Êä³ö    : ÎŞ
* ·µ»ØÖµ  : ÎŞ
* ËµÃ÷    : Õâ¸öÏ£ÍûÄÜ¹»µ÷ÓÃPWM²¨µÄ²úÉú³ÌĞò£¬Í¨¹ıPID¿ØÖÆ¶ÔÕ¼¿Õ±È½øĞĞµ÷½Ú£¬À´¿ØÖÆÀ´×Ô
*						boostµçÂ·µÄÊä³öµçÑ¹
						ÈçºÎ¶ÔÁ½¸ö±Õ»·½øĞĞÇø·Ö£¬Ö±½ÓÒÔÕæ¿Õ¶ÈÎª±ê×¼ »òÕß ÒÔboost µÄµ÷½ÚµÄµçÂ·Îª±ê×¼
						
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
	Kp=1;
	//Ki=0;
	//Kd=0;
	printf("Set P parameter:%f\r\n",Kp);
	printf("Set I parameter:%f\r\n",Ki);
	printf("Set D parameter:%f\r\n",Kd);
	printf("Set Pressure value:%f\r\n",VacuumValue_Set);
	
	
	//VacuumValue_Set=0.3;
	VacuumValue_Status=0.3;
	
	
	time_change=0.1;
	error_P=VacuumValue_Set-VacuumValue_Status;
	
	error_I +=(VacuumValue_Set-VacuumValue_Status);  //error sum

	error_D=VacuumValue_Status-Input_last;

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
	//Duty's range is from 0 to 3600,So we can set our duty to 
	
	TIM_SetCompare2(TIM3,Duty);

	
	return D;
		
	
}






