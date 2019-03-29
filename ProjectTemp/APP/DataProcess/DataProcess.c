#include  "DataProcess.h"
#include "printf.h"



float ADVoltage_2_Flow1479A(float ADValue_GasFlow)

{


    return  ADValue_GasFlow;

}


float ADVoltage_2_Pressure627D(float ADValue_Vacuum627D)

{


    return  ADValue_Vacuum627D;

}

float ADVoltage_2_Pressure025D(float ADValue_Vacuum025D)

{


    return  ADValue_Vacuum025D;

}





float Flow1479A_2_ADVoltage(float Flow_Value)
{

    return Flow_Value;

}



float Pressure627D_2_ADVoltage(float Pressure_Value)
{
    return Pressure_Value;

}


float Pressure025D_2_ADVoltage(float Pressure_Value)
{

    return Pressure_Value;


}









void    float2hex(uint8_t *char_array,float data)
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



int CheckCRC16(unsigned char *pdat, unsigned char len)
{
    unsigned int CRCi,sum;
    unsigned char CRCj;
    unsigned int crc ;  //unsigned int type which means 无符号字符类型
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

unsigned int  GetCRC16(unsigned char *pdat,int len)
{
    unsigned int CRCi,sum;
    unsigned char CRCj;
    unsigned int crc ;  //unsigned int type which means 无符号字符类型
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

