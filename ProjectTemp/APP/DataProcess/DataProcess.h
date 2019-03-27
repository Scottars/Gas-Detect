#ifndef _DataProcess_H_
#define _DataProcess_H_
#include "stdint.h"


float VacuumFloatValue_627DCalc(float ADValue_Vacuum627D);
float VacuumFloatValue_025DCalc(float ADValue_Vacuum025D);


float GasFloatValue_1479ACalc(float ADValue_GasFlow);
void	float2hex(uint8_t *char_array,float data);

int CheckCRC16(unsigned char *pdat, unsigned char len);
unsigned  int GetCRC16(unsigned char *pdat,int len);






#endif 


