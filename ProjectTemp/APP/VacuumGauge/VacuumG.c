
#include "VacuumG.h"

#include "printf.h"
#include "pwm.h"

/*************************************************
*Õâ¸öÎÄ¼þµÄÖ÷ÒªµÄÄ¿µÄ¾ÍÊÇÏ£ÍûÄÜ¹»ÊµÏÖ±Õ»·µÄ¿ØÖÆ£¬Ïà¹ØµÄº¯ÊýµÈµÈ
*
*
*
*
*
*************************************************/


/*******************************************************************************
* º¯ÊýÃû  : VacuumValue_Adapt
* ÃèÊö    : ¶Ô¸ø¶¨µÄÕæ¿Õ¶È£¬¶ÔPV·£µÄµçÑ¹½øÐÐÉè¶¨£¬Í¨¹ýµ÷ÓÃÄ¿±êµÄµçÑ¹Öµ
* ÊäÈë    : Õæ¿Õ¶ÈÉè¶¨ÖµVacuumValue_Set VacuumValue_Measure  ²ÉÓÃÄÄ¸öÖµ½øÐ
* Êä³ö    : ÎÞ
* ·µ»ØÖµ  : ÎÞ
* ËµÃ÷    : Õâ¸öÏ£ÍûÄÜ¹»µ÷ÓÃPWM²¨µÄ²úÉú³ÌÐò£¬Í¨¹ýPID¿ØÖÆ¶ÔÕ¼¿Õ±È½øÐÐµ÷½Ú£¬À´¿ØÖÆÀ´×Ô
*                       boostµçÂ·µÄÊä³öµçÑ¹
                        ÈçºÎ¶ÔÁ½¸ö±Õ»·½øÐÐÇø·Ö£¬Ö±½ÓÒÔÕæ¿Õ¶ÈÎª±ê×¼ »òÕß ÒÔboost µÄµ÷½ÚµÄµçÂ·Îª±ê×¼

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
	pwm_enable();


  //  printf("Set P parameter:%f\r\n",Kp);
  //  printf("Set I parameter:%f\r\n",Ki);
  //  printf("Set D parameter:%f\r\n",Kd);
   // printf("Actual Pressure value:%f\r\n",VacuumValue_Status);
 //   printf("Set Pressure value:%f\r\n",VacuumValue_Set);


    time_change=0.1;
    error_P=(VacuumValue_Set-VacuumValue_Status)*Kp;

    error_I +=(VacuumValue_Set-VacuumValue_Status)*Ki;  //error,we should multiplly the ki otherwise it is wrong

    error_D=(VacuumValue_Status-Input_last)*Kd;

    //printf("error_P:%f\r\n",error_P);
   // printf("error_I:%f\r\n",error_I);
   // printf("error_D:%f\r\n",error_D);

    Input_last = VacuumValue_Status;

  //  printf("Inputlast:%f\r\n",Input_last);

    D = error_P + error_I + error_D;

    //we think VacuumValueSet is 0.3 to 13.3
    // vacuumvalue_Setatus   0.3 to 13.3
    //so error=-10 to 10
    //we want duty in 0-3600, if it is linear, we can use the formular below

    //

    Duty=D*180 +1800;
   // printf("Set Duty parameter:%f\r\n",Duty/3600.0);
    //actually we need to call pwm duty change functiobn
    //Duty's range is from 0 to 3600,So we can set our duty to 0-3600 present 0% to 100% or
    //100% to 0%
    //we set the limit window so that the pwm adjustment will not be so dangerous
    //10% to 80%
    if(Duty>2880)
    {
        Duty=2880;
    }
    if(Duty<360)
    {
        Duty=360;
    }

    TIM_SetCompare2(TIM3,Duty);


    return D;


}






