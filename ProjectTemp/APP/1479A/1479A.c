
#include "1479A.h"
#include "DataProcess.h"
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
* Function name  : Flow_1479A_Adjustment
* Description  : Adjust the 1479A flow meter's flow
* Input :  the flow value we want to set
* Output  :  None
* Return Value :  None
* Attention: Call DAC_Set function 
						
*******************************************************************************/
void Flow_1479A_Adjustment(float Flow_Value)
{	

	float Flow_ADVoltage;
	//printf("Set Flow_Value parameter:%f\r\n",Flow_Value);
	
	
	//call 1479A_Flow_Value to voltage
	Flow_ADVoltage=Flow1479A_2_ADVoltage(Flow_Value);
	//	the output voltage of flow is 0~5V 


	//Call Dac conversion function 
	//Flow_ADVolatage should be at 0-3300  represent 0-3.3 v
	
	Flow_ADVoltage=(int)(Flow_Value*100);

//	printf("Set Actuall value parameter:%f\r\n",Flow_ADVoltage);
	
	Dac1_Set_Vol(Flow_ADVoltage);
//adtually we donnot need to use pid, beacause we have set the flow value 

	



	//actually we need to call pwm duty change functiobn


	

		
	
}






