

#include "VacuumG.h"

#include "printf.h"
#include "pwm.h"
#include "DataProcess.h"


/*************************************************
*这个文件的主要的目的就是希望能够实现闭环的控制，相关的函数等等
*
*
*
*
*
*************************************************/





/*******************************************************************************
* Function name  :  VacuumValue_PID
* Description  : adjust the voltage of the cavity 
* Input : VacuumValue_Set    VacuumValue_Status     Kp   Ki   Kd
* Output  :  None
* Return Value :  None
* Attention: we use the VacuumValue_Set VacuumValue_Statusto to process pid algorithm
			in order that we can make we can get the cavity to the proper pressure
*******************************************************************************/

	void VacuumValue_PID(float VacuumValue_Set, float VacuumValue_Status,float Kp,float Ki, float Kd)
	{
		float error_P;
		float static error_I=0;
		float error_D=0;
	
		float static error_last=0;
	
		float static Input_last=0;
	
		float error_AD;
		float D;
		float Duty;
		//Kp=1;
		//Ki=0.0001;
		//Kd=0;
		//VacuumValue_Set=0.3;
		//VacuumValue_Status=0;
	
		//In this part, we need to set our VacuumValue set to AD value, and we compare them.
		//VacuumValue_Set=Pressure627D_2_ADVoltage(VacuumValue_Set); //This is ad value
		//VacuumValue_Status=Pressure627D_2_ADVoltage(VacuumValue_Status);//this is ad	value
		pwm_enable();
	
		error_AD=Pressure627D_2_ADVoltage(VacuumValue_Set-VacuumValue_Status);
	  /*  printf("Set P parameter:%f\r\n",Kp);
	    printf("Set I parameter:%f\r\n",Ki);
	    printf("Set D parameter:%f\r\n",Kd);
	    printf("Actual Pressure value:%f\r\n",VacuumValue_Status);
	    printf("Set Pressure value:%f\r\n",VacuumValue_Set);
	
	*/
		//time_change=0.1;
		error_P=(error_AD)*Kp;
	
		error_I +=(error_AD)*Ki;	//error,we should multiplly the ki otherwise it is wrong
	
		error_D=(-error_AD)*Kd;
	
		//printf("error_P:%f\r\n",error_P);
	   // printf("error_I:%f\r\n",error_I);
	   // printf("error_D:%f\r\n",error_D);
	
		Input_last = VacuumValue_Status;
	
	  //  printf("Inputlast:%f\r\n",Input_last);
	
		D = error_P + error_I + error_D;
	
		//we think VacuumValueSet is 0.3 to 13.3
		// vacuumvalue_Setatus	 0.3 to 13.3
		//so error=-10 to 10
		//we want duty in 0-3600, if it is linear, we can use the formular below
	
		//
	
		Duty=D*180 +1800;
	  //  printf("Set Duty parameter:%f\r\n",Duty/3600.0);
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
	
	
	
	
	
	}
	
	
	
	
	
	





