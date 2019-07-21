 #include  "DataProcess.h"
#include "printf.h"
/*******************************************************************************
* Function name  : ADVoltage_2_Flow1479A
* Description  : Transfer the ad value to the flow value
* Input :  actual ad value(we can get from the ad conversion )
* Output  :  calculate -----actual pressure(to display 
			on the screen or to the serial port)
* Return Value :  Pressure
* Attention: we can only use this function when we need 
			 to display our value or tell the pc what flow it is now)
						
*******************************************************************************/

float ADVoltage_2_Flow1479A(float ADValue_GasFlow)

{




    return  ADValue_GasFlow;

}

/*******************************************************************************
* Function name  : ADVoltage_2_Pressure627D
* Description  : Transfer the ad value to the pressure value 
* Input :  actual ad value(we can get from the ad conversion )
* Output  :  calculate -----actual pressure(to display 
			on the screen or to the serial port)
* Return Value :  Pressure
* Attention: we can only use this function when we need 
			 to display our value or tell the pc what pressure it is now)
						
*******************************************************************************/



float ADVoltage_2_Pressure627D(float ADValue_Vacuum627D)
{
	
	float Pressure627D;
	Pressure627D=ADValue_Vacuum627D*13.3/2;

//	Pressure627D=6.820512821*ADValue_Vacuum627D;
	//Pressure627D=ADValue_Vacuum627D;	

    return  Pressure627D;

}
/*******************************************************************************
* Function name  : ADVoltage_2_Pressure025D
* Description  : Transfer the ad value to the pressure value 
* Input :  actual ad value(we can get from the ad conversion )
* Output  :  calculate -----actual pressure(to display 
			on the screen or to the serial port)
* Return Value :  Pressure
* Attention: we can only use this function when we need 
			 to display our value or tell the pc what pressure it is now)
						
*******************************************************************************/


float ADVoltage_2_Pressure025D(float ADValue_Vacuum025D)

{
	float Pressure025D;
	//Calculate has some errors
		//Pressure=20*13.3*ADValue_Vacuum025D/(3.9*10);
	Pressure025D=ADValue_Vacuum025D*13.3/2;
	
	
	
		return	Pressure025D;

}


/*******************************************************************************
* Function name  : Flow1479A_2_ADVoltage
* Description  : transfer the flow value to the ad value
* Input : actual flow value
* Output  :  AD voltage
* Return Value :  AD_Voltage
* Attention: we can only use this function when we get our flow
			or pressure from the pc 		
*******************************************************************************/


float Flow1479A_2_ADVoltage(float Flow_Value)
{
	
    return Flow_Value;

}

/*******************************************************************************
* Function name  : Pressure627D_2_ADVoltage
* Description  : transfer the flow value to the ad value
* Input : actual flow value
* Output  :  AD voltage
* Return Value :  AD_Voltage
* Attention: we can only use this function when we get our flow
			or pressure from the pc 		
*******************************************************************************/


float Pressure627D_2_ADVoltage(float Pressure_Value)
{
	float AD_Volatge;
	
		AD_Volatge=Pressure_Value*2/13.3;
		
    return AD_Volatge;

}

/*******************************************************************************
* Function name  : Pressure025D_2_ADVoltage
* Description  : transfer the flow value to the ad value
* Input : actual flow value
* Output  :  AD voltage
* Return Value :  AD_Voltage
* Attention: we can only use this function when we get our flow
			or pressure from the pc 		
*******************************************************************************/

float Pressure025D_2_ADVoltage(float Pressure_Value)
{
	float AD_Volatge;	
		AD_Volatge=Pressure_Value*2/13.3;
    return AD_Volatge;
}




void    float2hex(uint8_t *char_array,float data)
{    uint8_t i;
    for (i = 0; i<4; i++)
    {
        char_array[i] = ((uint8_t*)(&data))[i];
    }
    printf("%x",char_array[0]);
    printf("%x",char_array[1]);
    printf("%x",char_array[2]);
    printf("%x",char_array[3]);
}
/*******************************************************************************
* Function name  : CheckCRC16
* Description  : Determine whether the CRC is correct from the pc 
* Input : The package we received, and the actual data length we 
* Output  :  Crc check okay or not true=1 false = 0 
* Return Value :  1 or 0
* Attention: we use this function to judge the package we get from the pc is correct or not	
*******************************************************************************/

int CheckCRC16(unsigned char *pdat, unsigned char len)
{
    unsigned int CRCi,sum;
    unsigned char CRCj;
    unsigned int crc ;  //unsigned int type which means �޷����ַ�����
    crc = 0xFFFF;
    for (CRCi=0; CRCi<len; CRCi++)
    {
        crc ^= ((*pdat++) & 0x00FF);
        for (CRCj=0; CRCj<8; CRCj++)
        {
            if (crc & 0x0001)
            {
                crc >>= 1;
                crc ^= 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    sum = *pdat++;
    sum += (*pdat) << 8;
    if (sum == crc) return 1;
    return 0;
}
/*******************************************************************************
* Function name  : GetCRC16
* Description  : to get the crc we have put the data we need to send to the internet 
* Input :  The data we need to send to the internet and the length of the data(actually it is a package)
* Output  :  the crc code 
* Return Value :  crc code 
* Attention: we use this function to get the crc code in order to put it at the end of the package

*******************************************************************************/

unsigned int  GetCRC16(unsigned char *pdat,int len)
{
    unsigned int CRCi,sum;
    unsigned char CRCj;
    unsigned int crc ;  //unsigned int type which means �޷����ַ�����
    unsigned char *crc_return;
    crc = 0xFFFF;
    for (CRCi=0; CRCi<len; CRCi++)
    {
        crc ^= ((*pdat++) & 0x00FF);
        for (CRCj=0; CRCj<8; CRCj++)
        {
            if (crc & 0x0001)
            {
                crc >>= 1;
                crc ^= 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }

    return crc;
}

