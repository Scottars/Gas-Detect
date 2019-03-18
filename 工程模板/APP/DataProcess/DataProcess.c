#include  "DataProcess.h"
#include "printf.h"



float GasFloatValue_Calc(float ADValue_GasFlow)
{
	
	
	return	ADValue_GasFlow;

}
void	float2Hex(uint8_t *char_array,float data)
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

/*
int *float2Hex_1000(float data)
{
	int HighData=(int) data;
	if data
	
	
	
	char hex_return[4];
	
  data=(int)(data*1000)	;
	
	if (data>)
	{
		
		
	}
	else
	
	
	hex_return[0];
	
	
}
*/
