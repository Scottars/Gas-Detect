
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