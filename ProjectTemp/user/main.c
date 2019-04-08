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
#include "VacuumG.h"
#include "1479A.h"
#include "pwm.h"

#include "iwdg.h"
#include "exti.h"

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
typedef union
{
    unsigned int  CrcData;
    unsigned char byteData[2];
//  uint32_t byteData;
} CRC_BYTE;

/*
Device_name_verbe
*/


//several package value to set it in the analysis package function
//we can set these package value to be default if we get the package to set this value it will be update to the actual
//value we need to set
float Package_Cavity_627D_Pressure_Set=0.3;

float Package_Flow_1479A_Set=0.3;

float Package_Flow_1479A_Puff_Set=6.0;

float Package_Cavity_627D_Puff_Set=6.0;

char  Package_Valve_Puff_Status_Set[2]= {0x0f,0xff};

char Package_Valve_Status_Set[2]= {0x00,0xff};
float Package_Duty_P,Package_Duty_I,Package_Duty_D;



//////////////////flag for mode switch/////////////////

int flag_Debug=0;
int flag_Normal=0;



////////////////////Singal to Open Part///////////////////
u8 Valve_Signal_Open=0x00;
u8 Timing_Signal_Open=0x00;
u8 Command_Signal_Open=0x00;


////////////////////Mode Switch/////////////////// this should be able to change though the w5500
u8 Normal_Debug_RunningMode=0x00;

u8 Command_Timing_TriggerMode=0x00;

u8 Normal_Puff_RunningMode=0x00;

u8 PEV_1479A_ControlMode=0x00;

///////////////////Value to PID/////////////////
float Cavity_627D_Pressure_Status;
float Flow_1479A_Status;


//////////////////The limit value set///////////
/*
*we need to use this parameters to check the number which get from the pc
*
*
*/
float Cavity_627D_Pressure_SetMax=20;
float Cavity_627D_Pressure_SetMin=0;
float Flow_1479A_SetMax=20;
float Flow_1479A_SetMin=0;


float Cavity_627D_Puff_Pressure_SetMax=20;
float Cavity_627D_Puff_Pressure_SetMin=0;
float Flow_1479A_Puff_SetMax=20;
float Flow_1479A_Puff_SetMin=0;



float PID_P_SetMax=10;
float PID_I_SetMax=10;
float PID_D_SetMax=10;


float PID_P_SetMin=0;
float PID_I_SetMin=0;
float PID_D_SetMin=0;






int main()
{
    u8 i,j,k;
    u16 LCD_Display_Flag=1000;
    float temptofun;
    /**************variable define part************************/

    ////////////////////Value to control///////////////////

    float Cavity_627D_Pressure_Set;
    float Cavity_627D_Pressure_Default=0.5;


    //////////////////use to 1479A mode or pev mode working alone///////
    float PEV_FullyOpen_1479AMode=13.3;
    float _1479A_FullyOpen_PEVMode=13.3;
    //for debug mode
    float PEV_FullyClose_1479AMode=13.3;
    float _1479A_FullyClose_PEVMode=13.3;


    float Flow_1479A_Set;
    float Flow_1479A_Default=0.5;
    ////////////////////Valve IO part////////////////
    char Valve_Default_Status_Set[2]= {0x00,0x00};
    char Valve_Operation_Status_Set[2];
    char Valve_Puff_Status_Set[2];



    ///////////////////PID Duty Adjustment part//////
    float Duty_P=1,Duty_I=0,Duty_D=0; // we can set it later


    ////////////////////1479A part/////////////////////////
    /*we need to set target value*/



    ///////////////////we initial our own default data///////////////////

    //Valve_Operation_Status_Set=Valve_Default_Status_Set;
    Package_Cavity_627D_Pressure_Set=Cavity_627D_Pressure_Default;
    Package_Flow_1479A_Set=Flow_1479A_Default;
    Package_Duty_P=Duty_P;
    Package_Duty_I=Duty_I;
    Package_Duty_D=Duty_D;


    //Target pressure set , if there isn't value we use default value actually we need set this in the intial part
    Cavity_627D_Pressure_Set= Package_Cavity_627D_Pressure_Set; //Setting by the package we received from the internet

    //we should also set this function in the intial part
    Flow_1479A_Set=Package_Flow_1479A_Set;  //Setting by the package we received from the Internet


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


    /////////////PWM Initial//////////////////////
    pwm_init(3599,0);



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




    /*****************IWDG--working begin************************/

    iwdg_init();

    printf("watch dog working ");
    ////////////////////////Exti interrupt /////////////////////////////
    exti_init();
		

    while(1)
    {
        //printf("while mid");
        // delay_ms(100);





        /////////////Update the watch dog  register//////////////////////
        IWDG_ReloadCounter();
        printf("Watch dog in while\r\n");
//



        /****************************************���紦��ͨѶ����********************************************/
        if(W5500_Interrupt)//����W5500�ж�
        {
            //LED0=0;                                                   //LED0      ָʾ�������粿�ֵ��źŵĴ���
            W5500_Interrupt_Process();//W5500�жϴ��������
        }
        if((S0_Data & S_RECEIVE) == S_RECEIVE)//���Socket0���յ�����
        {
            S0_Data&=~S_RECEIVE;
            Process_Socket_Data(0);//W5500���ղ����ͽ��յ�������
            //���ڴ������������Ľ��յ����ݰ������������ַ������п��ǣ�����1�����Ĵ����е����ݷֳ�����Ȼ�����������н��и��ֵĵ������
            //���Ƕ���ֱ�������ݴ����־�ֱ�ӵ������ǵĽ���ʵ�ʵĲ������֡�
        }

        if(Normal_Debug_RunningMode==0x00)
        {

            printf("Normal Running Mode\n");
            if(flag_Debug==1)
            {
                //gonna swtich to debugrunning mode

                printf("In normal running mode,from debug mode\r\n");

                //In this situation, you need to close the valve
                //In this state, you need to Set the closed-loop to closed
                //In this state, you need to set the dac conversion to the minimal
                Valve_Operation_Status_Set[0]=0x00;
                Valve_Operation_Status_Set[1]=0x00;
                ValveStateChange(Valve_Operation_Status_Set);

                //to make sure the value is closed after changed the mode
                Valve_Signal_Open=0x00;
                //this sentence is simmilar to the last one, we can abandon the last sentence


                //Flow_1479A_Adjustment(_1479A_FullyOpen_PEVMode);//to make it fully open


                //VacuumValue_PID(PEV_FullyOpen_1479AMode, Cavity_627D_Pressure_Status, Package_Duty_P,Package_Duty_I,Package_Duty_D);

                flag_Debug=0;


            }
            flag_Normal=1;
            if(Valve_Signal_Open==0x00) //open valve or the system
            {
                //Wait for opening
                //In this state, you need to close the valve
                //In this state, you need to Set the closed-loop to closed
                //In this state, you need to set the dac conversion to the minimal
                Valve_Operation_Status_Set[0]=0x00;
                Valve_Operation_Status_Set[1]=0x00;
                ValveStateChange(Valve_Operation_Status_Set);

                //Flow_1479A_Adjustment(_1479A_FullyOpen_PEVMode);//to make it fully open


                //VacuumValue_PID(PEV_FullyOpen_1479AMode, Cavity_627D_Pressure_Status, Package_Duty_P,Package_Duty_I,Package_Duty_D);


                printf("Valve close\n");
            }
            else //Open command
            {
                printf("Valve Open\n");
                //Set actual valve should be opened
                Valve_Operation_Status_Set[0]=Package_Valve_Status_Set[0];//Get the valves to open from the package
                Valve_Operation_Status_Set[1]=Package_Valve_Status_Set[1];
                ValveStateChange(Valve_Operation_Status_Set);


                //Acutally the need both to adjust, so we can just need to write that code in the public area



                //
                if (PEV_1479A_ControlMode==0x00) //PEV control Mode 1
                {
                    //PEV,Default to PEV control
                    printf("PEV_Control Mode 1 \n");

                    //Default Set
                    // Set 1479A to fully open , we can use it fully open command   or use the DAC control to make it the biggest

                    Flow_1479A_Adjustment(_1479A_FullyOpen_PEVMode);//to make it fully ioen
                    //Directly use the value we set


                    printf("Call PID funtion\n\n\n");

                    //set pid parameter to the function
                    //we have set our default value to  Package_P  I D
                    //we should also check the number's reasonable value
                    //execute the PID function to set the new pwm duty ratio
                    VacuumValue_PID(Cavity_627D_Pressure_Set, Cavity_627D_Pressure_Status, Package_Duty_P,Package_Duty_I,Package_Duty_D);



                    if (Command_Timing_TriggerMode==0x00) //Command trigger
                    {
                        printf("Command  Trigger Mode\n");
                        //all the mode needs to be default
                        //Only set to unpuff mode
                        


                        if(Normal_Puff_RunningMode==0x00) //unpuff
                        {

							//this just like that, we open the puff mode 
                            printf("Unpuff Mode\n");

                            /*Switch those value to unpuff value, so that we can make it happen, during next pid adjustment*/
                            // valve to normal
                            Valve_Operation_Status_Set[0]=Package_Valve_Status_Set[0];// we can get it from the Internet

                            Valve_Operation_Status_Set[1]=Package_Valve_Status_Set[1];

                            //to close Puff mode status directly
                            ValveStateChange(Valve_Operation_Status_Set);

                            //1479A flow to normal
                            Flow_1479A_Set=Package_Flow_1479A_Set;
                            //627D Vacuum Pressire to normal
                            Cavity_627D_Pressure_Set=Package_Cavity_627D_Pressure_Set;


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

                                Cavity_627D_Pressure_Set=Package_Cavity_627D_Puff_Set;

                                //update the target presste value of PID adjustment

                            }
                            else
                            {
                                //In 1479A control Mode

                                printf("1479A control mode 2\n");

                                Flow_1479A_Set=Package_Flow_1479A_Puff_Set;   // we can get it from the package we receive




                            }

                        }




                    }
                    else // timing trigger mode
                    {
                        printf("Timing Trigger Mode \n");



                        if(Normal_Puff_RunningMode==0x00) //unpuff  mode off
                        {
                            printf("Unpuff mode");
                            // Close all the puff valve

                            /*Switch those value to unpuff value, so that we can make it happen, during next pid adjustment*/
                            // valve to normal
                            Valve_Operation_Status_Set[0]=Package_Valve_Status_Set[0];// we can get it from the Internet

                            Valve_Operation_Status_Set[1]=Package_Valve_Status_Set[1];

                            //to close Puff mode status directly
                            ValveStateChange(Valve_Operation_Status_Set);

                            //1479A flow to normal
                            Flow_1479A_Set=Package_Flow_1479A_Set;
                            //627D Vacuum Pressire to normal
                            Cavity_627D_Pressure_Set=Package_Cavity_627D_Pressure_Set;
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

                                Cavity_627D_Pressure_Set=Package_Cavity_627D_Puff_Set;


                                //update the target presste value of PID adjustment

                            }
                            else
                            {
                                //In 1479A control Mode

                                printf("1479A control mode 2 \n");

                                Flow_1479A_Set=Package_Flow_1479A_Puff_Set;   // we can get it from the package we receive

                            }

                        }

                    }
                }
                else //1479A control mode 1
                {

                    printf("1479A_Control Mode 1\n\n");
                    // Set the PEV open in order to using 1479A control mode
                    // we also use pev control, but only to make it open about 100v
                    //set puff - PEV's Open voltage do we need a value to set?

                    VacuumValue_PID(PEV_FullyOpen_1479AMode, Cavity_627D_Pressure_Status, Package_Duty_P,Package_Duty_I,Package_Duty_D);




                    printf("Flow Status:%f\n\n",Flow_1479A_Status);
                    printf("Target Flow Value:%f\n\n",Flow_1479A_Set);

                    //1479A adjustment DAC

                    Flow_1479A_Adjustment(Flow_1479A_Set);
                    //pay attention to this, if we directall



                    //Send back the pressure of the cavity
                    if (Command_Timing_TriggerMode==0x00) //Command trigger
                    {
                        printf("Command	Trigger Mode\n");
                        //all the mode needs to be default
                        //Only set to unpuff mode


                        if(Normal_Puff_RunningMode==0x00) //unpuff
                        {

                            printf("Unpuff Mode\n");

                            /*Switch those value to unpuff value, so that we can make it happen, during next pid adjustment*/
                            // valve to normal
                            Valve_Operation_Status_Set[0]=Package_Valve_Status_Set[0];// we can get it from the Internet

                            Valve_Operation_Status_Set[1]=Package_Valve_Status_Set[1];

                            //to close Puff mode status directly
                            ValveStateChange(Valve_Operation_Status_Set);

                            //1479A flow to normal
                            Flow_1479A_Set=Package_Flow_1479A_Set;

                            //627D Vacuum Pressire to normal
                            Cavity_627D_Pressure_Set=Package_Cavity_627D_Pressure_Set;


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

                                Cavity_627D_Pressure_Set=Package_Cavity_627D_Puff_Set;

                                //update the target presste value of PID adjustment
                                //we have already do this in the adjust part

                            }
                            else
                            {
                                //In 1479A control Mode

                                printf("1479A control mode 2\n");

                                Flow_1479A_Set=Package_Flow_1479A_Puff_Set;   // we can get it from the package we receive


                            }


                        }




                    }
                    else // timing trigger mode
                    {

                        printf("Timing Trigger Mode \n");



                        if(Normal_Puff_RunningMode==0x00) //unpuff  mode off
                        {
                            printf("Unpuff mode");
                            // Close all the puff valve

                            /*Switch those value to unpuff value, so that we can make it happen, during next pid adjustment*/
                            // valve to normal
                            Valve_Operation_Status_Set[0]=Package_Valve_Status_Set[0];// we can get it from the Internet

                            Valve_Operation_Status_Set[1]=Package_Valve_Status_Set[1];

                            //to close Puff mode status directly
                            ValveStateChange(Valve_Operation_Status_Set);

                            //1479A flow to normal
                            Flow_1479A_Set=Package_Flow_1479A_Set;
                            //627D Vacuum Pressire to normal
                            Cavity_627D_Pressure_Set=Package_Cavity_627D_Pressure_Set;
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

                                Cavity_627D_Pressure_Set=Package_Cavity_627D_Puff_Set;


                                //update the target presste value of PID adjustment







                            }
                            else
                            {
                                //In 1479A control Mode

                                printf("1479A control mode 2 \n");

                                Flow_1479A_Set=Package_Flow_1479A_Puff_Set;   // we can get it from the package we receive

                            }

                        }

                    }

                }

            }

        }
        else
        {
            //Initial ALl the valve or mode we are using
            if(flag_Normal==1)
            {
                //gonna swtich to debugrunning mode

                printf("In the debug mode,from normal running mode\r\n");

                //In this situation, you need to close the valve
                //In this state, you need to Set the closed-loop to closed
                //In this state, you need to set the dac conversion to the minimal
                Valve_Operation_Status_Set[0]=0x00;
                Valve_Operation_Status_Set[1]=0x00;
                ValveStateChange(Valve_Operation_Status_Set);

                Valve_Signal_Open=0x00;

                //   Flow_1479A_Adjustment(_1479A_FullyClose_PEVMode);//to make it fully open


                //  VacuumValue_PID(PEV_FullyClose_1479AMode, Cavity_627D_Pressure_Status, Package_Duty_P,Package_Duty_I,Package_Duty_D);

                flag_Normal=0;
            }
            flag_Debug=1;

            if(Valve_Signal_Open==0xff)
            {
                //Test whether the valves can work noramlly
                Valve_Operation_Status_Set[0]=Package_Valve_Status_Set[0];
                Valve_Operation_Status_Set[1]=Package_Valve_Status_Set[1];
                //Now, we just directly open the valve
                ValveStateChange(Valve_Operation_Status_Set);

                //Test whether the 1479A Flow Meter work normally
                Flow_1479A_Set=Package_Flow_1479A_Set;
                //Call DAC Translation funtion
                /*      */

                Flow_1479A_Adjustment(Flow_1479A_Set);

                //Test whether the PEV's closed-loop control can work normally
                Cavity_627D_Pressure_Set=Package_Cavity_627D_Pressure_Set;

                //call pid adjustment function
                VacuumValue_PID(Cavity_627D_Pressure_Set, Cavity_627D_Pressure_Status, Package_Duty_P,Package_Duty_I,Package_Duty_D);
                //test pid parameter's function
                //cannot be done with my code
            }
            else
            {

                printf("debug mode, valve closed");
            }


            printf("we are in dubug mode");

            //this mode is dubug running

        }

        //we should also set set currret status to the status register
        //we can use flag to get if we need to update the lcd
        if(LCD_Display_Flag==0)
        {
            printf("after 1000 times");
            LCD_Display_Flag=1000;
            Status_Register_Update();
        }
        LCD_Display_Flag--;
		 Status_Register_Update();

    }
}

/*******************************************************************************
* ������  : Process_Socket_Data
* ����    : W5500���ղ����ͽ��յ�������
* ����    : s:�˿ں�
* ���    : ��
* ����ֵ  : ��
* ˵��    : �������ȵ���S_rx_process()��W5500�Ķ˿ڽ������ݻ�������ȡ����,
*           Ȼ�󽫶�ȡ�����ݴ�Rx_Buffer������Temp_Buffer���������д���
*           ������ϣ������ݴ�Temp_Buffer������Tx_Buffer������������S_tx_process()
*           �������ݡ�
*******************************************************************************/

void Process_Socket_Data(SOCKET s)
{

    FLOAT_BYTE testdata;
    CRC_BYTE crctestdata;
    char floattest[10];
    char test1[4];
    unsigned short size;//���յ���buffer �Ĵ�С
    ///////////////////////Valve Gas Part///////

    u8 ValveValue_Set[12];  //�������紫��������ź�
    char *ValveValue_Status;
    u8 i;

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
    // memcpy(Tx_Buffer, Rx_Buffer, size);
//  printf("\r\nRX_BUFFER\r\n");
//   printf(Rx_Buffer);

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

    //run crc check


    if (Rx_Buffer[0]==0x05) //Slave address 0x05
    {
        //  printf("\r\nSLocal Address ok!\r\n");
        //after the slave address, we use the length to make sure the package is complete

        switch (Rx_Buffer[1])
        {
            case 0x03:    //��ȡ������Ĵ�����״̬ --�Ƿ�Ϊ�����ݹ�����
                switch  (Rx_Buffer[2])
                {

                    case 0x01:   //Read Gas 1479A flow meter value
                        Tx_Buffer[0]=0x05; // Slave address
                        Tx_Buffer[1]=0x03;// function  code
                        Tx_Buffer[2]=0x01;// register address
                        //Ad conversion
                        AD_Voltage_Status = AD_Conversion();
                        //AD_Voltage_Status[0 1 2]   �ֱ��ʾ1479A 627D 025d
                        // Flow_1479A_Status=1.5;

                        //
                        temp = ADVoltage_2_Flow1479A(AD_Voltage_Status[0]);
                        //temp=3;
                        //Transfer the float data to hex data
                        testdata.floatData=temp;

                        Tx_Buffer[3]=testdata.byteData[3];
                        Tx_Buffer[4]=testdata.byteData[2];
                        Tx_Buffer[5]=testdata.byteData[1];
                        Tx_Buffer[6]=testdata.byteData[0];

                        //GetCRC16
                        crctestdata.CrcData=GetCRC16(Tx_Buffer,7);


                        Tx_Buffer[7]=crctestdata.byteData[1];
                        Tx_Buffer[8]=crctestdata.byteData[0];


                        Write_SOCK_Data_Buffer(s, Tx_Buffer, 9);

                        break;


                    case 0x02: //Read 627D pressure value
                        Tx_Buffer[0]=0x05; // Slave address
                        Tx_Buffer[1]=0x03;// function  code
                        Tx_Buffer[2]=0x02;// register address
                        //Ad conversion
                        AD_Voltage_Status = AD_Conversion();

                        // Cavity_627D_Pressure_Status=AD_Voltage_Status[1];

                        //AD_Voltage_Status[0 1 2]   �ֱ��ʾ1479A 627D 025d
                        temp = ADVoltage_2_Pressure627D(AD_Voltage_Status[1]);

                        //float value to hex
                        testdata.floatData=temp;

                        Tx_Buffer[3]=testdata.byteData[3];
                        Tx_Buffer[4]=testdata.byteData[2];
                        Tx_Buffer[5]=testdata.byteData[1];
                        Tx_Buffer[6]=testdata.byteData[0];

                        //GetCRC16
                        crctestdata.CrcData=GetCRC16(Tx_Buffer,7);

                        Tx_Buffer[7]=crctestdata.byteData[1];
                        Tx_Buffer[8]=crctestdata.byteData[0];

                        Write_SOCK_Data_Buffer(s, Tx_Buffer, 9);

                        break;

                    case 0x03: //Read CDG025D  vacuum value
                        Tx_Buffer[0]=0x05; // Slave address
                        Tx_Buffer[1]=0x03;// function  code
                        Tx_Buffer[2]=0x03;// register address
                        //ad conbersion
                        AD_Voltage_Status = AD_Conversion();


                        //AD_Voltage_Status[0 1 2]   �ֱ��ʾ1479A 627D 025d
                        temp = ADVoltage_2_Pressure025D(AD_Voltage_Status[2]);


                        //float to hex
                        testdata.floatData=temp;

                        Tx_Buffer[3]=testdata.byteData[3];
                        Tx_Buffer[4]=testdata.byteData[2];
                        Tx_Buffer[5]=testdata.byteData[1];
                        Tx_Buffer[6]=testdata.byteData[0];

                        //GetCRC16
                        crctestdata.CrcData=GetCRC16(Tx_Buffer,7);


                        Tx_Buffer[7]=crctestdata.byteData[1];
                        Tx_Buffer[8]=crctestdata.byteData[0];

                        Write_SOCK_Data_Buffer(s, Tx_Buffer, 9);


                        break;

                    case 0x04://Read Gas Feed Status
                        Tx_Buffer[0]=0x05; // Slave address
                        Tx_Buffer[1]=0x03;// function  code
                        Tx_Buffer[2]=0x04;// register address
                        //read gas valve status
                        printf("before the gas state read");
                        ValveValue_Status=Gas_State_Read();
                        Tx_Buffer[3]=ValveValue_Status[0];
                        Tx_Buffer[4]=ValveValue_Status[1];
                        printf("after the gas state read");
                        //GetCRC16
                        crctestdata.CrcData=GetCRC16(Tx_Buffer,5);

                        Tx_Buffer[5]=crctestdata.byteData[1];
                        Tx_Buffer[6]=crctestdata.byteData[0];

                        Write_SOCK_Data_Buffer(s, Tx_Buffer, 7);
                        break;

                }

                break;
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

                        if(Rx_Buffer[3]==0x31)

                        {
                            Normal_Debug_RunningMode=0xff;

                            printf("Case 05 to open Debug Mode\n");
                        }
                        else
                        {
                            Normal_Debug_RunningMode=0x00;
                            printf("ase 05 to open Normal Running Mode\n");


                        }


                        break;
                    case 0x06: //Valve To Open

                        if(Rx_Buffer[3]==0x31)

                        {
                            Valve_Signal_Open=0xff;

                            printf("Case 06 to open valves\r\n");
                        }
                        else
                        {
                            Valve_Signal_Open=0x00;
                            printf("case 06 to close vavles\r\n");


                        }


                        break;

                    case 0x07: //PEV control  mode or 1479A control mode


                        if(Rx_Buffer[3]==0x31)

                        {
                            PEV_1479A_ControlMode=0xff;

                            printf("Case 07: to 1479A control Mode\r\n");
                        }
                        else
                        {
                            PEV_1479A_ControlMode=0x00;
                            printf("ase 06 to PEV control mode\r\n");


                        }


                        break;

                    case 0x08:  //Command Mode or timing trigger mode
                        //use the puff mode or not
                        //we can use a new parameter in the pid function to enable puff or unpuff
                        if(Rx_Buffer[3]==0x31) //��ʾ�رյ�ǰ���ģʽ x
                        {
                            Command_Timing_TriggerMode=0xff;
                            printf("Case 08 Timing Trigger Mode\r\n");

                        }
                        else
                        {
                            Command_Timing_TriggerMode=0x00;
                            printf("Case 08 Command Trigger Mode\r\n");

                        }



                        break;


                    case 0x09:  //Puff on or Puff off
                        //use the puff mode or not
                        //we can use a new parameter in the pid function to enable puff or unpuff
                        if(Rx_Buffer[3]==0x31) //��ʾ�رյ�ǰ���ģʽ x
                        {
                            Normal_Puff_RunningMode=0xff;
                            printf("Case 07 puff mode on\r\n");

                        }
                        else
                        {
                            Normal_Puff_RunningMode=0x00;
                            printf("Case 07 puff mode off\r\n");

                        }

                        break;

                    default:
                        break;

                }
                break;
            case 0x06:
                switch(Rx_Buffer[2])
                {
                    case 0x0A: //Vacuum value: Pressure value set 627D pressure value

                        //if the value is received, we feedback the same package that we get

                        size=Read_SOCK_Data_Buffer(s, Rx_Buffer);

                        //convert to float type
                        testdata.byteData[3]=Rx_Buffer[3];
                        testdata.byteData[2]=Rx_Buffer[4];
                        testdata.byteData[1]=Rx_Buffer[5];
                        testdata.byteData[0]=Rx_Buffer[6];

                        //Vacuum Value should be changed into Voltage value accoroding 627D manully

                        //we should set a critical value to limit the data
                        //if it doesn't meet our requirements,we need to send the data error to the pc



                        if((testdata.floatData>Flow_1479A_SetMax)|(testdata.floatData<Flow_1479A_SetMin))
                        {
                            //return the data over information


                        }
                        else //the set is okay
                        {

                            Package_Cavity_627D_Pressure_Set=testdata.floatData;
                            printf("Case 0x0A: set 627D pressure\r\n");

                            memcpy(Tx_Buffer, Rx_Buffer, size);
                            Write_SOCK_Data_Buffer(s, Tx_Buffer,size);
                        }



                        break;

                    case 0x0B:  //1479A Gas  Flow value set



                        //if the value is received, we feedback the same package that we get

                        size=Read_SOCK_Data_Buffer(s, Rx_Buffer);


                        //we should set a critical value to limit the data，
                        //if it doesn't meet our requirements,we need to send the data error to the pc
                        if((testdata.floatData>Cavity_627D_Pressure_SetMax)|(testdata.floatData<Cavity_627D_Pressure_SetMin))
                        {
                            //return the data over information


                        }
                        else //the set is okay
                        {


                            //convert to float type
                            testdata.byteData[3]=Rx_Buffer[3];
                            testdata.byteData[2]=Rx_Buffer[4];
                            testdata.byteData[1]=Rx_Buffer[5];
                            testdata.byteData[0]=Rx_Buffer[6];

                            //1479A Flow Set value

                            Package_Flow_1479A_Set=testdata.floatData;
                            printf("Set 1479A Flow Data\r\n");
                            memcpy(Tx_Buffer, Rx_Buffer, size);
                            Write_SOCK_Data_Buffer(s, Tx_Buffer,size);
                        }


                        break;
                    case 0x0C://Gas Puff mode: Puff Pressure value set

                        //if the value is received, we feedback the same package that we get

                        size=Read_SOCK_Data_Buffer(s, Rx_Buffer);

                        //we should set a critical value to limit the data
                        //if it doesn't meet our requirements,we need to send the data error to the pc

                        
/*                           if(testdata.floatData<)
                           {

						   
                           }*/
                        


                        //convert to float type
                        testdata.byteData[3]=Rx_Buffer[3];
                        testdata.byteData[2]=Rx_Buffer[4];
                        testdata.byteData[1]=Rx_Buffer[5];
                        testdata.byteData[0]=Rx_Buffer[6];

                        //Vacuum Value should be changed into Voltage value accoroding 627D manully

                        Package_Cavity_627D_Puff_Set=testdata.floatData;

                        printf("Case 0x0c: set puff pressure set\r\n");
                        memcpy(Tx_Buffer, Rx_Buffer, size);
                        Write_SOCK_Data_Buffer(s, Tx_Buffer,size);


                        break;
                    case 0x0d://pid code setting , set pid together, you cannot set it alone

                        testdata.byteData[3]=Rx_Buffer[3];
                        testdata.byteData[2]=Rx_Buffer[4];
                        testdata.byteData[1]=Rx_Buffer[5];
                        testdata.byteData[0]=Rx_Buffer[6];
                        //we should set a critical value to limit the data
                        //if it doesn't meet our requirements,we need to send the data error to the pc

                        if((testdata.floatData>PID_P_SetMax)|(testdata.floatData<PID_P_SetMin))
                        {
                            //return the data over information


                        }
                        else //the set is okay
                        {

                            Package_Duty_P=testdata.floatData;


                            testdata.byteData[3]=Rx_Buffer[7];
                            testdata.byteData[2]=Rx_Buffer[8];
                            testdata.byteData[1]=Rx_Buffer[9];
                            testdata.byteData[0]=Rx_Buffer[10];
                            //we should set a critical value to limit the data
                            if((testdata.floatData>PID_I_SetMax)|(testdata.floatData<PID_I_SetMin))
                            {
                                //return the data over information


                            }
                            else //the set is okay
                            {
                                Package_Duty_I=testdata.floatData;

                                testdata.byteData[3]=Rx_Buffer[11];
                                testdata.byteData[2]=Rx_Buffer[12];
                                testdata.byteData[1]=Rx_Buffer[13];
                                testdata.byteData[0]=Rx_Buffer[14];
                                //we should set a critical value to limit the data
                                if((testdata.floatData>PID_D_SetMax)|(testdata.floatData<PID_D_SetMin))
                                {
                                    //return the data over information


                                }
                                else //the set is okay
                                {
                                    Package_Duty_D=testdata.floatData;
                                    printf("Case 0x0d Set PID parameter ok\r\n");

                                    memcpy(Tx_Buffer, Rx_Buffer, size);
                                    Write_SOCK_Data_Buffer(s, Tx_Buffer,size);



                                }
                            }




                        }

                        break;

                    case 0x0E: //
                        Package_Valve_Status_Set[0]=Rx_Buffer[3];
                        Package_Valve_Status_Set[1]=Rx_Buffer[4];
                        printf("Case 0x0E: set 627D pressure\r\n");
                        break;

                    case 0x0F: //puff mode auxiliary valve

                        Package_Valve_Puff_Status_Set[0]=Rx_Buffer[3];
                        Package_Valve_Puff_Status_Set[1]=Rx_Buffer[4];

                        printf("Case 0x0f:Set Puff Mode auxiliary Valve\r\n");

                        break;


                }





                break;


            default:
                break;
        }



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
    temptofun=ADVoltage_2_Flow1479A(AD_Voltage_Status[0]);
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

void Initial_DebugMode()
{
    ////////////////////Singal to Open Part///////////////////
    Valve_Signal_Open=0x00;
    Timing_Signal_Open=0x00;
    Command_Signal_Open=0x00;


    ////////////////////Mode Switch/////////////////// this should be able to change though the w5500
    Normal_Debug_RunningMode=0x00;

    Command_Timing_TriggerMode=0x00;

    Normal_Puff_RunningMode=0x00;

    PEV_1479A_ControlMode=0x00;


}

void Status_Register_Update()
{


    float *AD_Voltage_Status;
    char *ValveValue_Status;
    char *ValveValue_Status_LCD;

    /* In this part we need to update the devices to LCD or the internet  */  //use timer to send my status to the pc
    //LCD to update
    ValveValue_Status=Gas_State_Read(); //����ʵ�ֶ�ȡIO�ڵĸߵ͵�ƽֵ
    ValveValue_Status_LCD = Gas_State_Read_LCD();
    Gas_StateLayerUpdate(ValveValue_Status_LCD);
    AD_Voltage_Status=AD_Conversion();
    ADC_LCD_Out(AD_Voltage_Status);


    //Alter current status ,



    Flow_1479A_Status=ADVoltage_2_Flow1479A(AD_Voltage_Status[0]);
    Cavity_627D_Pressure_Status=ADVoltage_2_Pressure627D(AD_Voltage_Status[1]);




}

