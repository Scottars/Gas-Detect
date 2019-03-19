#include  "DataProcess.h"
#include "printf.h"



float GasFloatValue_Calc(float ADValue_GasFlow)
{
	
	
	return	ADValue_GasFlow;

}
void	float2hex(uint8_t *char_array,float data)
{

	
	uint8_t i;
	for (i = 0; i<4; i++)
	{
		char_array[i] = ((uint8_t*)(&data))[i];
	}
	printf("%x",char_array[0]);
	printf("%x",char_array[1]);
	printf("%x",char_array[2]);
	printf("%x",char_array[3]); 
}


