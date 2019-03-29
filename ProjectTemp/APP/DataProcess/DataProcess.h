#ifndef _DataProcess_H_
#define _DataProcess_H_
#include "stdint.h"

float ADVoltage_2_Flow1479A(float ADValue_GasFlow);
float ADVoltage_2_Pressure627D(float ADValue_Vacuum627D);
float ADVoltage_2_Pressure025D(float ADValue_Vacuum025D);

float Flow1479A_2_ADVoltage(float Flow_Value);
float Pressure627D_2_ADVoltage(float Pressure_Value);
float Pressure025D_2_ADVoltage(float Pressure_Value);






void	float2hex(uint8_t *char_array,float data);

int CheckCRC16(unsigned char *pdat, unsigned char len);
unsigned  int GetCRC16(unsigned char *pdat,int len);





#endif 


