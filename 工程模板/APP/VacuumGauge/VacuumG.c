
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
*						boostµçÂ·µÄÊä³öµçÑ¹
						ÈçºÎ¶ÔÁ½¸ö±Õ»·½øÐÐÇø·Ö£¬Ö±½ÓÒÔÕæ¿Õ¶ÈÎª±ê×¼ »òÕß ÒÔboost µÄµ÷½ÚµÄµçÂ·Îª±ê×¼
						
*******************************************************************************/
void VacuumValue_PID(float VacuumValue_Set, float VacuumValue_Measure)
{	
	float error;
	error=VacuumValue_Set-VacuumValue_Measure;
	
	
	
	
	
	
	
		
	
}




/*






/*==================================================================================================== 
PID????
=====================================================================================================*/ 
double PIDCalc( PID *pp, double NextPoint ) 
{ 
    double dError, Error; 
    Error = pp->SetPoint - NextPoint; // ?? 
    pp->SumError += Error; // ?? 
    dError = pp->LastError - pp->PrevError; // ???? 
    pp->PrevError = pp->LastError; 
    pp->LastError = Error; 
    return (pp->Proportion * Error // ??? 
    + pp->Integral * pp->SumError // ??? 
    + pp->Derivative * dError );         // ??? 
} 

/*==================================================================================================== 
PID??????????
=====================================================================================================*/ 
void PIDInit (PID *pp) 
{ 
memset ( pp,0,sizeof(PID)); 
} 

/*==================================================================================================== 
????????(????????100)
======================================================================================================*/ 
double sensor (void)  
{ 
return 100.0; 
} 

/*====================================================================================================
???????? 
======================================================================================================*/ 
void actuator(double rDelta)  
{
} 

//??? 
void main(void) 
{ 
    PID sPID; // PID Control Structure 
    double rOut; // PID Response (Output) 
    double rIn; // PID Feedback (Input) 
    PIDInit ( &sPID ); // Initialize Structure 
    sPID.Proportion = 0.5; // Set PID Coefficients 
    sPID.Integral = 0.5; 
    sPID.Derivative = 0.0; 
    sPID.SetPoint = 100.0; // Set PID Setpoint 
    for (;;) 
    {                                   // Mock Up of PID Processing 
        rIn = sensor ();                // ????????(Read Input )
        rOut = PIDCalc ( &sPID,rIn );   // PID????(Perform PID Interation) 
        actuator ( rOut );              // ????????(Effect Needed Changes) 
    } 
}






**/


/*

int incPIDcalc(PIDtypedef *PIDx,u16 nextpoint)
{
 int iError,iincpid;
 iError=PIDx->setpoint-nextpoint;  //????
 /*iincpid=                                               //????
 PIDx->proportion*iError                //e[k]?
 -PIDx->integral*PIDx->last_error          //e[k-1]
 +PIDx->derivative*PIDx->prev_error;//e[k-2]
*/
iincpid=                                                          //????
PIDx->proportion*(iError-PIDx->last_error)
+PIDx->integral*iError
+PIDx->derivative*(iError-2*PIDx->last_error+PIDx->prev_error);

 PIDx->prev_error=PIDx->last_error; //????,??????
 PIDx->last_error=iError;
 return(iincpid) ;
}









*/