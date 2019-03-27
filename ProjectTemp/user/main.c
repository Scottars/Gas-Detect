/*******************************************************************************
*
*                              ���пƼ�
--------------------------------------------------------------------------------
* ʵ �� ��       : ADCת��ʵ��
* ʵ��˵��       : ͨ��printf��ӡAD������� ������ADģ���Աߵĵ�λ�� �ڴ��������ϼ��������ѹ��
                    ���ļ��ڽ�ͼ
* ���ӷ�ʽ       :
* ע    ��       :  ���ú�����ͷ�ļ�.c�ļ���
*******************************************************************************/


#include "public.h"
#include "printf.h"
#include "adc.h"
#include "systick.h"
#include "dac.h"    //ʵ��ad ת��
#include "led.h"

#include "gui.h"
#include "test.h"
//#include "delay.h"
#include "lcd.h"
#include "IOState.h"

#include "DataProcess.h"
#include "w5500.h"

/****************************************************************************
* Function Name  : main
* Description    : Main program.
* Input          : None
* Output         : None
* Return         : None
****************************************************************************/
unsigned char ADdataA[7];
extern unsigned int W5500_Send_Delay_Counter;
//unsigned char ADdataB[7];
//unsigned char A1;
//unsigned char A2;
typedef union
{
    float floatData;
    unsigned char byteData[4];
//  uint32_t byteData;
} FLOAT_BYTE;
/*
Device_name_verbe


*/



//several package value to set it in the analysis package function
//we can set these package value to be default if we get the package to set this value it will be update to the actual
//value we need to set
float Package_Cavity_627D_Pressure_Set=0.3;

float Package_Flow_1479A_Set=0.3;

float Package_Flow_1479A_Puff_Set=3.0;

float Package_Cavity_627D_Puff_Set=6.0;

char  Package_Valve_Puff_Status_Set[2]= {0x0f,0xff};

char Package_Valve_Status_Set[2]= {0x00,0xff};

////////////////////Singal to Open Part///////////////////
u8 Valve_Signal_Open=0x00;
u8 Timing_Signal_Open=0x00;
u8 Command_Signal_Open=0x00;


////////////////////Mode Switch/////////////////// this should be able to change though the w5500
u8 Normal_Debug_RunningMode=0x00;

u8 Command_Timing_TriggerMode=0x00;

u8 Normal_Puff_RunningMode=0x00;

u8 PEV_1479A_ControlMode=0x00;


int main()
{
    u8 i,j,k;
    float temptofun;
    /**************variable define part************************/





    ////////////////////Value to control///////////////////
    float Cavity_627D_Pressure_Status;
    float Cavity_627D_Pressure_Set;
    float Cavity_627D_Pressure_Default;

    float Flow_1479A_Status;
    float Flow_1479A_Set;
    float Flow_1479A_Default;


    float Flow_1479A_Puff_Set;

    float Cavity_627D_Puff_Set;



    ////////////////////Valve IO part////////////////
    char Valve_Default_Status_Set[2]= {0x00,0x00};
    char Valve_Operation_Status_Set[2];
    char Valve_Puff_Status_Set[2];


    ///////////////////PID Duty Adjustment part//////
    float Duty_P,Duty_I,Duty_D;

    //////////////////


    ////////////////////1479A part/////////////////////////
    /*we need to set target value*/








    float *AD_Voltage_Status;
    char *ValveValue_Status;
    char *ValveValue_Status_LCD;

    ///////////////////we initial our own default data///////////////////

    //Valve_Operation_Status_Set=Valve_Default_Status_Set;






    /****************Initial system part*****************/

    ///////////////////////Һ������ʼ������///////////////////////

    SystemInit();//��ʼ��RCC ����ϵͳ��ƵΪ72MHZ
    delay_init(72);      //��ʱ��ʼ��
    LCD_Init();    //Һ������ʼ��


    /////////////////////LED light initial////////////////////////////

    LED_Init(); //Initial

    ///////////////ADC Initial///////////////////ADCģ��ʹ��������Ƭ��AD PC0 PC1 PC2  �ֱ��� AD10 11 12 ͨ��
    adc_init();  //ADC��ʼ��

    /////////////////Serial port initial//////////////�������д���ͨѶ�ģ�printf ʵ�����ض���
    printf_init(); //printf��ʼ��

    ///////////////DAC Initial /////////////////DAC ���ֵĳ�ʼ�������ǲ��õ���DAC1  Ҳ����PA4 �ڵ����
    Dac1_Init();                //DAC��ʼ��
    DAC_SetChannel1Data(DAC_Align_12b_R, 0);//��ʼֵΪ0


    ///////////////Valve Status Initial/////////////

    ValveState_Init(); //��������GPIO�ڵĲ��ֳ�ʼ��



    ////////////////LCD Intial ///////////////////
    GridLayer();  //��ʾ���������ܲ�
    Gas_StateLayer();//��ʾ�� ������״̬��ʼ������

    //////////////////�����ʼ��//////////////////////



    /*********************************************
    ˵����  PC
                            IP��192.168.1.199
                            �˿ںţ�4001 δ����ͨ������ѡ��ʲô�˿ںŶ��ܹ�ʵ��

                            ���Ե�ʱ���Ŀ���IP��Ŀ��Ķ˿ں�ʹ�õľ���w5500��IP��˿ں�

                    w5500��
                            IP��192.168.1.198
                            �˿ںţ�5000


    **********************************************/

    System_Initialization();    //STM32ϵͳ��ʼ������(��ʼ��STM32ʱ�Ӽ�����)
    Load_Net_Parameters();      //װ���������
    W5500_Hardware_Reset();     //Ӳ����λW5500
    W5500_Initialization();     //W5500��ʼ������
    W5500_Socket_Set();//W5500�˿ڳ�ʼ������





    while(1)
    {




        /****************************************���紦��ͨѶ����********************************************/
        if(W5500_Interrupt)//����W5500�ж�
        {
            //LED0=0;                                                   //LED0      ָʾ�������粿�ֵ��źŵĴ���
            W5500_Interrupt_Process();//W5500�жϴ���������
        }
        if((S0_Data & S_RECEIVE) == S_RECEIVE)//���Socket0���յ�����
        {
            S0_Data&=~S_RECEIVE;
            Process_Socket_Data(0);//W5500���ղ����ͽ��յ�������
            //���ڴ������������Ľ��յ����ݰ������������ַ������п��ǣ�����1�����Ĵ����е����ݷֳ�����Ȼ�����������н��и��ֵĵ������
            //���Ƕ���ֱ�������ݴ������־�ֱ�ӵ������ǵĽ���ʵ�ʵĲ������֡�
        }

        if(Normal_Debug_RunningMode==0x00)
        {
            printf("Normal Running Mode\n");


            /*********Initial Normal Running Mode Circumastance************/


            ////////Set default value to target value////////



            ////////Set Valves that should be set////////


            if(Valve_Signal_Open==0x00) //open valve or the system
            {
                //Wait for opening

                printf("Valve close\n");






            }
            else //Open command
            {
                printf("Valve Open\n");

                //Set actual valve should be opened
                Valve_Operation_Status_Set[0]=Package_Valve_Status_Set[0];//Get the valves to open from the package

                Valve_Operation_Status_Set[1]=Package_Valve_Status_Set[1];

                ValveStateChange(Valve_Operation_Status_Set);

                //


                if (PEV_1479A_ControlMode==0x00) //PEV control Mode 1
                {
                    //PEV,Default to PEV control
                    printf("PEV_Control Mode 1 \n");

					// Set 1479A to fully open , we can use it fully open command   or use the DAC control to make it the biggest

					

					

					


                    //Target pressure set , if there isn't value we use default value
                    Cavity_627D_Pressure_Set= Package_Cavity_627D_Pressure_Set; //Setting by the package we received from the internet



                    //set pid parameter to the function


                    //execute the PID function to set the new pwm duty ratio


                    if (Command_Timing_TriggerMode==0x00) //Command trigger
                    {
                        printf("Command  Trigger Mode\n");
						//all the mode needs to be default 
						//Only set to unpuff mode 
						Normal_Puff_RunningMode=0x00;

                        if(Normal_Puff_RunningMode==0x00) //unpuff
                        {
                            // Close all the puff valve
                            Cavity_627D_Pressure_Set=Cavity_627D_Pressure_Default;

                            Flow_1479A_Set=Flow_1479A_Default;
                            printf("Unpuff Mode\n");

                            /*Switch those value to unpuff value, so that we can make it happen, during next pid adjustment*/
                            // valve to normal
                            Valve_Operation_Status_Set[0]=Package_Valve_Status_Set[0];// we can get it from the Internet

                            Valve_Operation_Status_Set[1]=Package_Valve_Status_Set[1];

                            //to close Puff mode status directly
                            ValveStateChange(Valve_Operation_Status_Set);

                            //1479A flow to normal
                            Flow_1479A_Set=Flow_1479A_Default;
                            //627D Vacuum Pressire to normal
                            Cavity_627D_Pressure_Set=Cavity_627D_Pressure_Default;


                            //this mode we jump out to exexute the pid adjustment again


                        }
                        else //In  the PUff Mode
                        {
                            printf("puff Mode\n");

                            //to open Puff mode status
                            Valve_Operation_Status_Set[0]=Package_Valve_Status_Set[0] | Package_Valve_Puff_Status_Set[0];// we can get it from the Internet

                            Valve_Operation_Status_Set[1]=Package_Valve_Status_Set[1] | Package_Valve_Puff_Status_Set[1];

                            ValveStateChange(Valve_Operation_Status_Set);


                            /*which pressure control mode  we are right now   second layer */

                            if (PEV_1479A_ControlMode==0x00) // Second layer to set puff value
                            {
                                //In PEV control Mode
                                printf("Pev control mode 2\n");

                                Cavity_627D_Puff_Set=Package_Cavity_627D_Puff_Set;

                                //update the target presste value of PID adjustment

                                Cavity_627D_Pressure_Set=Cavity_627D_Puff_Set;





                            }
                            else
                            {
                                //In 1479A control Mode

                                printf("1479A control mode 2\n");

                                Flow_1479A_Puff_Set=Package_Flow_1479A_Puff_Set;   // we can get it from the package we receive



                                //update the target presste value of PID adjustment

                                Flow_1479A_Set=Flow_1479A_Puff_Set;
                            }






                        }




                    }
                    else // timing trigger mode
                    {
                    	printf("Timing Trigger Mode \n");
	                    
					

                        if(Normal_Puff_RunningMode==0x00) //unpuff  mode off
                        {
                            printf("unpuff mode");
                            // Close all the puff valve
                            Cavity_627D_Pressure_Set=Cavity_627D_Pressure_Default;

                            Flow_1479A_Set=Flow_1479A_Default;
                            printf("Unpuff Mode\n");

                            /*Switch those value to unpuff value, so that we can make it happen, during next pid adjustment*/
                            // valve to normal
                            Valve_Operation_Status_Set[0]=Package_Valve_Status_Set[0];// we can get it from the Internet

                            Valve_Operation_Status_Set[1]=Package_Valve_Status_Set[1];


                            //to open Puff mode status directly to open
                            ValveStateChange(Valve_Operation_Status_Set);

                            //1479A flow to normal
                            Flow_1479A_Set=Flow_1479A_Default;
                            //627D Vacuum Pressire to normal
                            Cavity_627D_Pressure_Set=Cavity_627D_Pressure_Default;


                            //this mode we jump out to exexute the pid adjustment again

                        }


                        else // PUff Mode on
                        {

                            //open
                            //Open the puff valve   //about this we should know that  it's the best to set the whole puff valve instead of set an extra valve
                            printf("puff Mode\n");



                            //to open Puff mode status
                            Valve_Operation_Status_Set[0]=Package_Valve_Status_Set[0] | Package_Valve_Puff_Status_Set[0];// we can get it from the Internet

                            Valve_Operation_Status_Set[1]=Package_Valve_Status_Set[1] | Package_Valve_Puff_Status_Set[1];

                            //to open Puff mode status directly to open
                            ValveStateChange(Valve_Operation_Status_Set);



                            /*which pressure control mode  we are right now  */

                            if (PEV_1479A_ControlMode==0x00) //// Second layer to set puff value
                            {
                                //In PEV control Mode
                                printf("Pev control mode 2 \n");

                                Cavity_627D_Puff_Set=Package_Cavity_627D_Puff_Set;


                                //update the target presste value of PID adjustment

                                Cavity_627D_Pressure_Set=Cavity_627D_Puff_Set;





                            }
                            else
                            {
                                //In 1479A control Mode

                                printf("1479A control mode 2 \n");

                                Flow_1479A_Puff_Set=Package_Flow_1479A_Puff_Set;   // we can get it from the package we receive



                                //update the target presste value of PID adjustment

                                Flow_1479A_Set=Flow_1479A_Puff_Set;

                            }





                        }

                    }
                }
                else //1479A control mode 1
                {

				
				
					
					
				
                    printf("1479A_Control Mode 1\n");
					// Set the PEV open in order to using 1479A control mode 
					// we also use pev control, but only to make it open about 100v
					//set puff - 1479A mode voltage
					//PID adjustment
					
					//
						

					//
                    Flow_1479A_Set=Package_Flow_1479A_Set;  //Setting by the package we received from the Internet

					//1479A adjustment DAC



                    //Send back the pressure of the cavity



                    if (Command_Timing_TriggerMode==0x00) //Command trigger
                    {
                        printf("Command Trigger Mode\n");
					

                        if(Normal_Puff_RunningMode==0x00) //unpuff
                        {
                            // Close all the puff valve
                            Cavity_627D_Pressure_Set=Cavity_627D_Pressure_Default;

                            Flow_1479A_Set=Flow_1479A_Default;
                            printf("Unpuff Mode\n");

                            /*Switch those value to unpuff value, so that we can make it happen, during next pid adjustment*/
                            // valve to normal
                            Valve_Operation_Status_Set[0]=Package_Valve_Status_Set[0];// we can get it from the Internet

                            Valve_Operation_Status_Set[1]=Package_Valve_Status_Set[1];

                            //to close Puff mode status directly
                            ValveStateChange(Valve_Operation_Status_Set);

                            //1479A flow to normal
                            Flow_1479A_Set=Flow_1479A_Default;
                            //627D Vacuum Pressire to normal
                            Cavity_627D_Pressure_Set=Cavity_627D_Pressure_Default;


                            //this mode we jump out to exexute the pid adjustment again


                        }
                        else //In  the PUff Mode
                        {
                            printf("puff Mode\n");

                            //to open Puff mode status
                            Valve_Operation_Status_Set[0]=Package_Valve_Status_Set[0] | Package_Valve_Puff_Status_Set[0];// we can get it from the Internet

                            Valve_Operation_Status_Set[1]=Package_Valve_Status_Set[1] | Package_Valve_Puff_Status_Set[1];

                            ValveStateChange(Valve_Operation_Status_Set);


                            /*which pressure control mode  we are right now   second layer */

                            if (PEV_1479A_ControlMode==0x00) // Second layer to set puff value
                            {
                                //In PEV control Mode
                                printf("Pev control mode 2\n");

                                Cavity_627D_Puff_Set=Package_Cavity_627D_Puff_Set;

                                //update the target presste value of PID adjustment

                                Cavity_627D_Pressure_Set=Cavity_627D_Puff_Set;



                            }
                            else
                            {
                                //In 1479A control Mode

                                printf("1479A control mode 2\n");

                                Flow_1479A_Puff_Set=Package_Flow_1479A_Puff_Set;   // we can get it from the package we receive



                                //update the target presste value of PID adjustment

                                Flow_1479A_Set=Flow_1479A_Puff_Set;
                            }






                        }




                    }
                    else // timing trigger mode
                    {


                        if(Normal_Puff_RunningMode==0x00) //unpuff  mode off
                        {
                            printf("unpuff mode");
                            // Close all the puff valve
                            Cavity_627D_Pressure_Set=Cavity_627D_Pressure_Default;

                            Flow_1479A_Set=Flow_1479A_Default;
                            printf("Unpuff Mode\n");

                            /*Switch those value to unpuff value, so that we can make it happen, during next pid adjustment*/
                            // valve to normal
                            Valve_Operation_Status_Set[0]=Package_Valve_Status_Set[0];// we can get it from the Internet

                            Valve_Operation_Status_Set[1]=Package_Valve_Status_Set[1];


                            //to open Puff mode status directly to open
                            ValveStateChange(Valve_Operation_Status_Set);

                            //1479A flow to normal
                            Flow_1479A_Set=Flow_1479A_Default;
                            //627D Vacuum Pressire to normal
                            Cavity_627D_Pressure_Set=Cavity_627D_Pressure_Default;


                            //this mode we jump out to exexute the pid adjustment again

                        }


                        else // PUff Mode on
                        {

                            //open
                            //Open the puff valve   //about this we should know that  it's the best to set the whole puff valve instead of set an extra valve
                            printf("puff Mode\n");



                            //to open Puff mode status
                            Valve_Operation_Status_Set[0]=Package_Valve_Status_Set[0] | Package_Valve_Puff_Status_Set[0];// we can get it from the Internet

                            Valve_Operation_Status_Set[1]=Package_Valve_Status_Set[1] | Package_Valve_Puff_Status_Set[1];

                            //to open Puff mode status directly to open
                            ValveStateChange(Valve_Operation_Status_Set);



                            /*which pressure control mode  we are right now  */

                            if (PEV_1479A_ControlMode==0x00) //// Second layer to set puff value
                            {
                                //In PEV control Mode
                                printf("Pev control mode 2 \n");

                                Cavity_627D_Puff_Set=Package_Cavity_627D_Puff_Set;


                                //update the target presste value of PID adjustment

                                Cavity_627D_Pressure_Set=Cavity_627D_Puff_Set;





                            }
                            else
                            {
                                //In 1479A control Mode

                                printf("1479A control mode 2 \n");

                                Flow_1479A_Puff_Set=Package_Flow_1479A_Puff_Set;   // we can get it from the package we receive



                                //update the target presste value of PID adjustment

                                Flow_1479A_Set=Flow_1479A_Puff_Set;

                            }





                        }

                    }





                }


                //send back the Pressure value
                //PEV control mode or 1479A control mode both ok
                Cavity_Pressure_SendBack();



            }







        }
        else
        {


            printf("we are in dubug mode");

            //this mode is dubug running



        }


        /* In this part we need to update the devices to LCD or the internet  */  //use timer to send my status to the pc
        //LCD to update
        ValveValue_Status=Gas_State_Read(); //����ʵ�ֶ�ȡIO�ڵĸߵ͵�ƽֵ
        ValveValue_Status_LCD = Gas_State_Read_LCD();
        Gas_StateLayerUpdate(ValveValue_Status_LCD);
        AD_Voltage_Status=AD_Conversion();
        ADC_LCD_Out(AD_Voltage_Status);


    }
}

/*******************************************************************************
* ������  : Process_Socket_Data
* ����    : W5500���ղ����ͽ��յ�������
* ����    : s:�˿ں�
* ���    : ��
* ����ֵ  : ��
* ˵��    : �������ȵ���S_rx_process()��W5500�Ķ˿ڽ������ݻ�������ȡ����,
*           Ȼ�󽫶�ȡ�����ݴ�Rx_Buffer������Temp_Buffer���������д�����
*           ������ϣ������ݴ�Temp_Buffer������Tx_Buffer������������S_tx_process()
*           �������ݡ�
*******************************************************************************/

void Process_Socket_Data(SOCKET s)
{

    FLOAT_BYTE testdata;
    char floattest[10];
    char test1[4];
    unsigned short size;//���յ���buffer �Ĵ�С
    ///////////////////////Valve Gas Part///////

    u8 ValveValue_Set[12];  //�������紫��������ź�
    char *ValveValue_Status;
    u8 i;
    ////////////////////////////////////////////////////////
    ////////////////���ǻ����/////////////////////////////
    float VacuumValue_Set;


    //////////////////�����Ƶ�����ֵ�趨�趨//////////////////////////////////
    float Flow1479AValue_Set;



    ////////////////���������ֵ��ն��趨//////////////////////////////////
    float GasPuffValue_Set;


    ////////////////PID�����趨/////////////////////////
    float Kp,Ki,Kd;


    ////////////////CRC_Mid/////////////////////////
    unsigned int CRC_Mid;







    ///////////////////ADת������////////////////
    float *AD_Voltage_Status;
    float temp=0; // ��������ADת��ʹ�õı���
    uint8_t temp1[5];
    char AD_Value[50];






    size=Read_SOCK_Data_Buffer(s, Rx_Buffer);
    //printf("\r\nSIZE:%d\r\n",size);
    memcpy(Tx_Buffer, Rx_Buffer, size);
//  printf("\r\nRX_BUFFER\r\n");
    printf(Rx_Buffer);

    //����RX_Buffer  ��λ�������� Ӧ�ô������һ�������ݣ� ������һ��������λ
    //���Կ���Rx_Buffer �൱��һ�����飬16λ��
    //����ModbusЭ�� �Ⱦٸ�����
    /*
    �����ݣ�    04               03                   02                                                                                            ��λ�����ظ�����Ҫ������Ϣ 04 03 02 length data1 data2 *** crc
                    �ӻ���ַ          �������                      �Ĵ�����ַ  ---------------------------------------��
    д���ݣ�    04               06                   02             data1  data2 ***  crc16                ��λ�����ظ�����Ҫд����Ϣ 04 06 02  ****     ��PS������ط��ظ���������λ����ͬ�Ķ�����
                    �ӻ���ַ          ������д                      �Ĵ�����ַ
    ��λOn/off��  04               05                   02              data1 data2 ***  crc16                   ��λ����    ����Ƿ���Ҫ�������ݣ�
                    �ӻ���ַ          ��������λ                    �Ĵ�����ַ
    ������Ϣ��  04               04                  02              data1 data2 ***  crc16            �������λ������λ���ķ��͵������в�ͬ��������Ĵ���
                    �ӻ���ַ          ��������λ                    �Ĵ�����ַ


    */

    // �ڽ�����֮ǰ�ͽ��У�crc check





    if (Rx_Buffer[0]==0x05) //�������豸��ַΪ0x05
    {
        //  printf("\r\nSLocal Address ok!\r\n");

        switch (Rx_Buffer[1])
        {
            case 0x03:    //��ȡ������Ĵ�����״̬ --�Ƿ�Ϊ�����ݹ�����
                switch  (Rx_Buffer[2])
                {

                    case 0x01:
                        Tx_Buffer[0]=0x05; // ������ַ
                        Tx_Buffer[1]=0x03;//����������
                        Tx_Buffer[2]=0x01;//�Ĵ�����?
                        //��ȡ���������ĵ�ǰֵ�����¼���һ�δ�����ȥ������ֱ���ϴ���ǰֵ
                        AD_Voltage_Status = AD_Conversion();
                        //AD_Voltage_Status[0 1 2]   �ֱ��ʾ1479A 627D 025d

                        //�Եõ��ĵ�ѹ��������ֵ����ת��,
                        temp = GasFloatValue_1479ACalc(AD_Voltage_Status[0]);
                        temp=3;
                        //ת���õ�IEEE 754 ��׼
                        testdata.floatData=temp;

                        Tx_Buffer[3]=testdata.byteData[3];
                        Tx_Buffer[4]=testdata.byteData[2];
                        Tx_Buffer[5]=testdata.byteData[1];
                        Tx_Buffer[6]=testdata.byteData[0];

                        //GetCRC16 �õ���������ݵ�CRCУ����
                        CRC_Mid=GetCRC16(Tx_Buffer,7);

                        Tx_Buffer[7]=(u8)CRC_Mid;
                        Tx_Buffer[8]=(u8)(CRC_Mid>>8);

                        Write_SOCK_Data_Buffer(s, Tx_Buffer, 9);
                        break;


                    case 0x02: //627D��նȼ�⣬��������λ����������һ���Ҫ��һ�����ݻ�ȥ��
                        Tx_Buffer[0]=0x05; // ������ַ
                        Tx_Buffer[1]=0x03;//����������
                        Tx_Buffer[2]=0x02;//�Ĵ�����
                        //������ն�ADת��
                        AD_Voltage_Status = AD_Conversion();

                        //����AD�ɼ�������ֵ���д����õ���նȵı�׼��ֵ
                        //AD_Voltage_Status[0 1 2]   �ֱ��ʾ1479A 627D 025d
                        temp = VacuumFloatValue_627DCalc(AD_Voltage_Status[1]);



                        //��ֵת���ɸ�����
                        testdata.floatData=temp;

                        Tx_Buffer[3]=testdata.byteData[3];
                        Tx_Buffer[4]=testdata.byteData[2];
                        Tx_Buffer[5]=testdata.byteData[1];
                        Tx_Buffer[6]=testdata.byteData[0];



                        //GetCRC16 �õ���������ݵ�CRCУ����
                        CRC_Mid=GetCRC16(Tx_Buffer,7);

                        Tx_Buffer[7]=(u8)CRC_Mid;
                        Tx_Buffer[8]=(u8)(CRC_Mid>>8);

                        Write_SOCK_Data_Buffer(s, Tx_Buffer, 9);


                        break;

                    case 0x03: //025D��նȼ�⣬��������λ����������һ���Ҫ��һ�����ݻ�ȥ��
                        Tx_Buffer[0]=0x05; // ������ַ
                        Tx_Buffer[1]=0x03;//����������
                        Tx_Buffer[2]=0x03;//�Ĵ�����?
                        //������ն�ADת��
                        AD_Voltage_Status = AD_Conversion();

                        //����AD�ɼ�������ֵ���д����õ���նȵı�׼��ֵ
                        //AD_Voltage_Status[0 1 2]   �ֱ��ʾ1479A 627D 025d
                        temp = VacuumFloatValue_025DCalc(AD_Voltage_Status[2]);

                        temp=3;
                        //��ֵת���ɸ�����
                        testdata.floatData=temp;

                        Tx_Buffer[3]=testdata.byteData[3];
                        Tx_Buffer[4]=testdata.byteData[2];
                        Tx_Buffer[5]=testdata.byteData[1];
                        Tx_Buffer[6]=testdata.byteData[0];



                        //GetCRC16 �õ���������ݵ�CRCУ����
                        CRC_Mid=GetCRC16(Tx_Buffer,7);

                        Tx_Buffer[7]=(u8)CRC_Mid;
                        Tx_Buffer[8]=(u8)(CRC_Mid>>8);

                        Write_SOCK_Data_Buffer(s, Tx_Buffer, 9);


                        break;

                    case 0x04://��������״̬��ȡ
                        Tx_Buffer[0]=0x05; // ������ַ
                        Tx_Buffer[1]=0x03;//����������
                        Tx_Buffer[2]=0x04;//�Ĵ�����ַ
                        //��ȡ��������״̬����
                        printf("before the gas state read");
                        ValveValue_Status=Gas_State_Read();
                        Tx_Buffer[3]=ValveValue_Status[0];
                        Tx_Buffer[4]=ValveValue_Status[1];
                        printf("after the gas state read");

                        CRC_Mid=GetCRC16(Tx_Buffer,5);
                        printf("after crc ");
                        Tx_Buffer[5]=(u8)CRC_Mid;
                        Tx_Buffer[6]=(u8)(CRC_Mid>>8);
                        //size=3;
                        Write_SOCK_Data_Buffer(s, Tx_Buffer, 7);
                        break;



                }


            case 0x05:    //д��Ĵ�����״̬
                //�趨12��·���趨��նȡ��趨����ֵ
                //�趨������ͨ��ȫ�ֱ������ͳ�ȥ���������ֱ�ӵ�����صĺ����أ�
                //�ݶ������� ֱ�ӵ�����صĺ���
                //�����Rx_Buffer�Ķ��壺unsigned char Rx_Buffer[2048]


                ////////////////////����ģʽ////////////////////
                /*********************************************
                *����ģʽ��ѡ����2��
                * ģʽ1����λ��ֱ�ӿ��������� �� ÿ�θ��������趨��ǰ����ֵ
                * ģʽ2����λ������������������ͨ���Զ��ñջ����п���
                *ʵ�ֵ÷���
                *
                *
                *********************************************/
                switch  (Rx_Buffer[2])
                {


                    case 0x05:
                        ////////////////////valve to open command///////////////////
                        if(Rx_Buffer[3]==0xff)

                        {
                            Valve_Signal_Open=Rx_Buffer[3];

                            printf("we are in the case 05 in order to open the valve\n");
                        }
                        else
                        {
                            Valve_Signal_Open=0x00;
                            printf("we are in the case 05 in order to close the valve\n");


                        }


                        break;










                    case 0x06:  //�������ģʽ����ͣ  �����������ֵ�趨����֮���Լ�������ʽ�趨����֮��ͨ�������������������ֹͣ
                        //use the puff mode or not
                        //we can use a new parameter in the pid function to enable puff or unpuff
                        if(Rx_Buffer[3]==0xff) //��ʾ�رյ�ǰ���ģʽ x
                        {
                            Normal_Puff_RunningMode=0xff;

                        }
                        else
                        {
                            Normal_Puff_RunningMode=0x00;

                        }



                        break;



                }

            case 0x06:
                switch(Rx_Buffer[2])
                {
                    case 0x07: //���ǻ��ն�����




                        //���ʱ�򣬶���λ����������buffer 3 4 5 6 �е����ݽ��д�����������ǰ���ǽ���crc У��
                        //convert to float type
                        testdata.byteData[3]=Rx_Buffer[3];
                        testdata.byteData[2]=Rx_Buffer[4];
                        testdata.byteData[1]=Rx_Buffer[5];
                        testdata.byteData[0]=Rx_Buffer[6];

                        //Vacuum Value should be changed into Voltage value accoroding 627D manul


                        printf("Setvalue:%f\r\n",testdata.floatData);
                        if (testdata.floatData==-12.5)
                        {
                            printf("OK,In the if");
                            Tx_Buffer[0]=0xff;
                            Write_SOCK_Data_Buffer(s, Tx_Buffer,1);
                        }

                        //if the value is received, we feedback the same package that we get
                        Write_SOCK_Data_Buffer(s, Rx_Buffer,1);


                        //VacuumValue_Set=

                        break;
                    case 0x08:  //feed mode 1479A or pev control mode

                        if (Rx_Buffer[3]==0xff)
                        {
                            PEV_1479A_ControlMode=0xff;
                            printf("Switch to 1479A control Mode");
                        }
                        else
                        {
                            PEV_1479A_ControlMode=0x00;
                            printf("Switch to PEV  control Mode");
                        }




                        break;
                    case 0x09://1479A����������Ŀ���趨
                        //���ʱ�򣬶���λ����������buffer 3 4 5 6 �е����ݽ��д�����������ǰ���ǽ���crc У�� ͨ��
                        //ps:˵������λ���ĸ������Ĵ��͵�˳���Ǹ�λ��ǰ��λ�ں󣬶���bytes�У�Ҳ�ǵ�λ������ĵ�λ
                        testdata.byteData[3]=Rx_Buffer[3];
                        testdata.byteData[2]=Rx_Buffer[4];
                        testdata.byteData[1]=Rx_Buffer[5];
                        testdata.byteData[0]=Rx_Buffer[6];

                        printf("Setvalue:%f\r\n",testdata.floatData);
                        if (testdata.floatData==-12.5)
                        {
                            printf("OK,In the if");
                            Tx_Buffer[0]=0xff;
                            Write_SOCK_Data_Buffer(s, Tx_Buffer,1);
                        }




                        //1479AFloatValue_Set=



                        break;
                    case 0xA: //���������ֵ�趨

                        //���ʱ�򣬶���λ����������buffer 3 4 5 6 �е����ݽ��д�����������ǰ���ǽ���crc У�� ͨ��




                        //GasPuffValue_Set=



                        break;
                    case 0xB://PID�������趨
                        //���ʱ�򣬶���λ����������buffer 3 4 5 6    7 8 9 10  11 12 13 14 �е����ݽ��д�����������ǰ���ǽ���crc У�� ͨ��




                        //=

                        break;
                    case 0x0C: //puff trigger mode --0xff timing mode   0x00 command mode
                        if(Rx_Buffer[3]=0xff) //ѡ�������λ��������ʽ  �����ģʽ�£��趨�µ���ֵ�����ǵ��趨����նȵ�ֵ
                        {

                        }
                        if(Rx_Buffer[3]=0x02) //ѡ���źŴ�����ʽ�������ģʽ�£��趨���µ���ֵ��ʱ���ź����˵�ʱ�����ǽ�����µ���նȵ���ֵд��ȥ��
                        {


                        }
                        break;

                    case 0x0D: //
                        Package_Valve_Status_Set[0]=Rx_Buffer[3];
                        Package_Valve_Status_Set[1]=Rx_Buffer[4];
                        break;

                    case 0x0E: //puff mode auxiliary valve

                        Package_Valve_Puff_Status_Set[0]=Rx_Buffer[3];
                        Package_Valve_Puff_Status_Set[1]=Rx_Buffer[4];


                        break;


                }





                break;


            default:
                break;
        }

        /*if (Rx_Buffer[0]==0x01)   //�жϼĴ����е����ݾ�ͨ����������С�
            {
                Write_SOCK_Data_Buffer(s, Tx_Buffer, size);
                Write_SOCK_Data_Buffer(0, "\r\n THis is 1\r\n", 23);

            }*/

    }
    else
    {
        //������Ǳ����ĵ�ַ����� ��ν��д���



    }


    /*if (Rx_Buffer[0]==0x11)   //�жϼĴ����е����ݾ�ͨ����������С�
    {
        Write_SOCK_Data_Buffer(s, Tx_Buffer, size);
        Write_SOCK_Data_Buffer(0, "\r\n THis is 1\r\n", 23);

    }*/

    //

//����д����ڿ�ܵ����ݣ����������õ����������ݽ��зְ�������



}









void Cavity_Pressure_SendBack()
{
    ////////////////CRC_Mid/////////////////////////
    unsigned int CRC_Mid;
    FLOAT_BYTE testdata;
    float temptofun;
    float *AD_Voltage_Status;




    u8 s=0;
    /*Read the actual flow value , and send it back to the pc*/
    Tx_Buffer[0]=0x05; // ������ַ
    Tx_Buffer[1]=0x03;//����������
    Tx_Buffer[2]=0x01;//�Ĵ�����

    //AD translation
    AD_Voltage_Status=AD_Conversion();

    //value translation
    temptofun=GasFloatValue_1479ACalc(AD_Voltage_Status[0]);
    temptofun=-12.5;
    //value to IEEE 754 standard

    testdata.floatData=temptofun;

    //Set it to Tx_Buffer
    Tx_Buffer[3]=testdata.byteData[3];
    Tx_Buffer[4]=testdata.byteData[2];
    Tx_Buffer[5]=testdata.byteData[1];
    Tx_Buffer[6]=testdata.byteData[0];



    //Get Crc check code
    CRC_Mid=GetCRC16(Tx_Buffer,6);

    //Set it to Tx_Buffer
    Tx_Buffer[7]=(u8)CRC_Mid;
    Tx_Buffer[8]=(u8)(CRC_Mid>>8);

    Write_SOCK_Data_Buffer(s, Tx_Buffer, 8);




}


