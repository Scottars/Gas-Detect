
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