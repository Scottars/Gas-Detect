
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






