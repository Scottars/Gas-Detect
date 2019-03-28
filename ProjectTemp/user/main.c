/*******************************************************************************
*
*                              ÆÕÖÐ¿Æ¼¼
--------------------------------------------------------------------------------
* Êµ Ñé Ãû       : ADC×ª»»ÊµÑé
* ÊµÑéËµÃ÷       : Í¨¹ýprintf´òÓ¡AD¼ì²âÊý¾Ý £¬µ÷½ÚADÄ£¿éÅÔ±ßµÄµçÎ»Æ÷ ÔÚ´®¿ÚÖúÊÖÉÏ¼´¿ÉÊä³öµçÑ¹£¬
                    ¼ûÎÄ¼þÄÚ½ØÍ¼
* Á¬½Ó·½Ê½       :
* ×¢    Òâ       :  ËùÓÃº¯ÊýÔÚÍ·ÎÄ¼þ.cÎÄ¼þÄÚ
*******************************************************************************/


#include "public.h"
#include "printf.h"
#include "adc.h"
#include "systick.h"
#include "dac.h"    //ÊµÏÖad ×ª»»
#include "led.h"

#include "gui.h"
#include "test.h"
//#include "delay.h"
#include "lcd.h"
#include "IOState.h"

#include "DataProcess.h"
#include "w5500.h"
#include "VacuumG.h"

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

float Package_Flow_1479A_Puff_Set=3.0;

float Package_Cavity_627D_Puff_Set=6.0;

char  Package_Valve_Puff_Status_Set[2]= {0x0f,0xff};

char Package_Valve_Status_Set[2]= {0x00,0xff};

///////////////////PID Duty Adjustment part//////
float Duty_P,Duty_I,Duty_D;

//////////////////


////////////////////Singal to Open Part///////////////////
u8 Valve_Signal_Open=0x00;
u8 Timing_Signal_Open=0x00;
u8 Command_Signal_Open=0x00;


////////////////////Mode Switch/////////////////// this should be able to change though the w5500
u8 Normal_Debug_RunningMode=0x00;

u8 Command_Timing_TriggerMode=0x00;

u8 Normal_Puff_RunningMode=0x00;

u8 PEV_1479A_ControlMode=0x00;

///////////////////Value to PID ///////
float Cavity_627D_Pressure_Status;
float Flow_1479A_Status;





int main()
{
    u8 i,j,k;
    float temptofun;
    /**************variable define part************************/





    ////////////////////Value to control///////////////////

    float Cavity_627D_Pressure_Set;
    float Cavity_627D_Pressure_Default=0.5;


    float Flow_1479A_Set;
    float Flow_1479A_Default=0.5;


    float Flow_1479A_Puff_Set;

    float Cavity_627D_Puff_Set;



    ////////////////////Valve IO part////////////////
    char Valve_Default_Status_Set[2]= {0x00,0x00};
    char Valve_Operation_Status_Set[2];
    char Valve_Puff_Status_Set[2];





    ////////////////////1479A part/////////////////////////
    /*we need to set target value*/








    float *AD_Voltage_Status;
    char *ValveValue_Status;
    char *ValveValue_Status_LCD;

    ///////////////////we initial our own default data///////////////////

    //Valve_Operation_Status_Set=Valve_Default_Status_Set;
    Package_Cavity_627D_Pressure_Set=Cavity_627D_Pressure_Default;
	Package_Flow_1479A_Set=Flow_1479A_Default;
	


    /****************Initial system part*****************/

    ///////////////////////Òº¾§ÆÁ³õÊ¼»¯¹ý³Ì///////////////////////

    SystemInit();//³õÊ¼»¯RCC ÉèÖÃÏµÍ³Ö÷ÆµÎª72MHZ
    delay_init(72);      //ÑÓÊ±³õÊ¼»¯
    LCD_Init();    //Òº¾§ÆÁ³õÊ¼»¯


    /////////////////////LED light initial////////////////////////////

    LED_Init(); //Initial

    ///////////////ADC Initial///////////////////ADCÄ£¿éÊ¹ÓÃÁËÈý¸öÆ¬ÉÏAD PC0 PC1 PC2  ·Ö±ðÊÇ AD10 11 12 Í¨µÀ
    adc_init();  //ADC³õÊ¼»¯

    /////////////////Serial port initial//////////////ÓÃÀ´½øÐÐ´®¿ÚÍ¨Ñ¶µÄ£¬printf ÊµÏÖÁËÖØ¶¨Ïò
    printf_init(); //printf³õÊ¼»¯

    ///////////////DAC Initial /////////////////DAC ²¿·ÖµÄ³õÊ¼»¯£¬ÎÒÃÇ²ÉÓÃµÄÊÇDAC1  Ò²¾ÍÊÇPA4 ¿ÚµÄÊä³ö
    Dac1_Init();                //DAC³õÊ¼»¯
    DAC_SetChannel1Data(DAC_Align_12b_R, 0);//³õÊ¼ÖµÎª0


    ///////////////Valve Status Initial/////////////

    ValveState_Init(); //¹©Æø·§ÃÅGPIO¿ÚµÄ²¿·Ö³õÊ¼»¯



    ////////////////LCD Intial ///////////////////
    GridLayer();  //ÏÔÊ¾ÆÁµÄÍøÂç¿ò¼Ü²ã
    Gas_StateLayer();//ÏÔÊ¾ÆÁ ¹©Æø·§×´Ì¬³õÊ¼»¯²¿·Ö

    //////////////////ÍøÂç³õÊ¼»¯//////////////////////



    /*********************************************
    ËµÃ÷£º  PC
                            IP£º192.168.1.199
                            ¶Ë¿ÚºÅ£º4001 Î´¶¨£¬Í¨¹ý²âÊÔÑ¡ÓÃÊ²Ã´¶Ë¿ÚºÅ¶¼ÄÜ¹»ÊµÏÖ

                            ²âÊÔµÄÊ±ºòµÄÄ¿±êµÄIPºÍÄ¿±êµÄ¶Ë¿ÚºÅÊ¹ÓÃµÄ¾ÍÊÇw5500µÄIPÓë¶Ë¿ÚºÅ

                    w5500£º
                            IP£º192.168.1.198
                            ¶Ë¿ÚºÅ£º5000


    **********************************************/

    System_Initialization();    //STM32ÏµÍ³³õÊ¼»¯º¯Êý(³õÊ¼»¯STM32Ê±ÖÓ¼°ÍâÉè)
    Load_Net_Parameters();      //×°ÔØÍøÂç²ÎÊý
    W5500_Hardware_Reset();     //Ó²¼þ¸´Î»W5500
    W5500_Initialization();     //W5500³õÊ¼»õÅäÖÃ
    W5500_Socket_Set();//W5500¶Ë¿Ú³õÊ¼»¯ÅäÖÃ





    while(1)
    {




        /****************************************ÍøÂç´¦ÀíÍ¨Ñ¶´¦Àí********************************************/
        if(W5500_Interrupt)//´¦ÀíW5500ÖÐ¶Ï
        {
            //LED0=0;                                                   //LED0      Ö¸Ê¾µÄÊÇÍøÂç²¿·ÖµÄÐÅºÅµÄ´«Êä
            W5500_Interrupt_Process();//W5500ÖÐ¶Ï´¦Àí³ÌÐò¿ò¼Ü
        }
        if((S0_Data & S_RECEIVE) == S_RECEIVE)//Èç¹ûSocket0½ÓÊÕµ½Êý¾Ý
        {
            S0_Data&=~S_RECEIVE;
            Process_Socket_Data(0);//W5500½ÓÊÕ²¢·¢ËÍ½ÓÊÕµ½µÄÊý¾Ý
            //¶ÔÓÚ´«ÏÂÀ´µÄÕû¸öµÄ½ÓÊÕµÄÊý¾Ý°ü£¬ÎÒÃÇÓÐÁ½ÖÖ·½·¨½øÐÐ¿¼ÂÇ£¬¿¼ÂÇ1£º½«¼Ä´æÆ÷ÖÐµÄÊý¾Ý·Ö³öÀ´£¬È»ºóÔÚÖ÷º¯ÊýÖÐ½øÐÐ¸÷ÖÖµÄµ÷ÓÃÇé¿ö
            //¿¼ÂÇ¶þ£ºÖ±½ÓÔÚÊý¾Ý´¦Àí²¿·Ö¾ÍÖ±½Óµ÷ÓÃÎÒÃÇµÄ½øÐÐÊµ¼ÊµÄ²Ù×÷²¿·Ö¡£
        }

        if(Normal_Debug_RunningMode==0x00)
        {
            // printf("Normal Running Mode\n");
            if(Valve_Signal_Open==0x00) //open valve or the system
            {
                //Wait for opening
                //      printf("Valve close\n");
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

                    //Default Set
                    // Set 1479A to fully open , we can use it fully open command   or use the DAC control to make it the biggest









                    //Target pressure set , if there isn't value we use default value
                    Cavity_627D_Pressure_Set= Package_Cavity_627D_Pressure_Set; //Setting by the package we received from the internet

                    printf("Target Pressure Valueï¼š%f\r\n",Cavity_627D_Pressure_Set);
                    printf("Status Pressure:%f\r\n",Cavity_627D_Pressure_Status);

                    //set pid parameter to the function
                    VacuumValue_PID(Cavity_627D_Pressure_Set, Cavity_627D_Pressure_Status, Duty_P,Duty_I,Duty_D);


                    //execute the PID function to set the new pwm duty ratio


                    if (Command_Timing_TriggerMode==0x00) //Command trigger
                    {
                        printf("Command  Trigger Mode\n");
                        //all the mode needs to be default
                        //Only set to unpuff mode
                        Normal_Puff_RunningMode=0x00;

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
                    //set puff - PEV's Open voltage do we need a value to set?


                    //PID adjustment

                    //


                    //
                    Flow_1479A_Set=Package_Flow_1479A_Set;  //Setting by the package we received from the Internet
                    printf("Target Flow Valueï¼š%f\n",Flow_1479A_Set);

                    //1479A adjustment DAC



                    //Send back the pressure of the cavity



                    if (Command_Timing_TriggerMode==0x00) //Command trigger
                    {
                        printf("Command Trigger Mode\n");


                        if(Normal_Puff_RunningMode==0x00) //unpuff
                        {
                            // Close all the puff valve
                   
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

                        printf("Timing Trigger Mode\n");


                        if(Normal_Puff_RunningMode==0x00) //unpuff  mode off
                        {
                            printf("unpuff mode");
                            // Close all the puff valve
              
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
                //Cavity_Pressure_SendBack();



            }







        }
        else
        {
            //Initial ALl the valve or mode we are using



            //Test whether the valves can work noramlly
            Valve_Operation_Status_Set[0]=Package_Valve_Status_Set[0];
            Valve_Operation_Status_Set[1]=Package_Valve_Status_Set[1];
            //Now, we just directly open the valve
            ValveStateChange(Valve_Operation_Status_Set);

            //Test whether the 1479A Flow Meter work normally
            Flow_1479A_Set=Package_Flow_1479A_Set;
            //Call DAC Translation funtion
            /*      */


            //Test whether the PEV's closed-loop control can work normally
            Cavity_627D_Pressure_Set=Package_Cavity_627D_Pressure_Set;

            //call pid adjustment function



            //test pid parameter's function
            //cannot be done with my code


            printf("we are in dubug mode");









            //this mode is dubug running



        }


        /* In this part we need to update the devices to LCD or the internet  */  //use timer to send my status to the pc
        //LCD to update
        // ValveValue_Status=Gas_State_Read(); //º¯ÊýÊµÏÖ¶ÁÈ¡IO¿ÚµÄ¸ßµÍµçÆ½Öµ
        //   ValveValue_Status_LCD = Gas_State_Read_LCD();
        //   Gas_StateLayerUpdate(ValveValue_Status_LCD);
        //  AD_Voltage_Status=AD_Conversion();
        //   ADC_LCD_Out(AD_Voltage_Status);


    }
}

/*******************************************************************************
* º¯ÊýÃû  : Process_Socket_Data
* ÃèÊö    : W5500½ÓÊÕ²¢·¢ËÍ½ÓÊÕµ½µÄÊý¾Ý
* ÊäÈë    : s:¶Ë¿ÚºÅ
* Êä³ö    : ÎÞ
* ·µ»ØÖµ  : ÎÞ
* ËµÃ÷    : ±¾¹ý³ÌÏÈµ÷ÓÃS_rx_process()´ÓW5500µÄ¶Ë¿Ú½ÓÊÕÊý¾Ý»º³åÇø¶ÁÈ¡Êý¾Ý,
*           È»ºó½«¶ÁÈ¡µÄÊý¾Ý´ÓRx_Buffer¿½±´µ½Temp_Buffer»º³åÇø½øÐÐ´¦Àí¡£
*           ´¦ÀíÍê±Ï£¬½«Êý¾Ý´ÓTemp_Buffer¿½±´µ½Tx_Buffer»º³åÇø¡£µ÷ÓÃS_tx_process()
*           ·¢ËÍÊý¾Ý¡£
*******************************************************************************/

void Process_Socket_Data(SOCKET s)
{

    FLOAT_BYTE testdata;
    CRC_BYTE crctestdata;
    char floattest[10];
    char test1[4];
    unsigned short size;//½ÓÊÕµ½µÄbuffer µÄ´óÐ¡
    ///////////////////////Valve Gas Part///////

    u8 ValveValue_Set[12];  //À´×ÔÍøÂç´«Èë¹ýÀ´µÄÐÅºÅ
    char *ValveValue_Status;
    u8 i;
    ////////////////////////////////////////////////////////
    ////////////////Õæ¿ÕÇ»²¿·Ö/////////////////////////////
    float VacuumValue_Set;


    //////////////////Á÷Á¿¼ÆµÄÁ÷Á¿ÖµÉè¶¨Éè¶¨//////////////////////////////////
    float Flow1479AValue_Set;



    ////////////////ÆøÌåÅç³ö·åÖµÕæ¿Õ¶ÈÉè¶¨//////////////////////////////////
    float GasPuffValue_Set;


    ////////////////PID²ÎÊýÉè¶¨/////////////////////////
    float Kp,Ki,Kd;


    ////////////////CRC_Mid/////////////////////////
    unsigned int CRC_Mid;







    ///////////////////AD×ª»»²¿·Ö////////////////
    float *AD_Voltage_Status;
    float temp=0; // ÓÃÀ´½øÐÐAD×ª»»Ê¹ÓÃµÄ±äÁ¿
    uint8_t temp1[5];
    char AD_Value[50];






    size=Read_SOCK_Data_Buffer(s, Rx_Buffer);
    //printf("\r\nSIZE:%d\r\n",size);
    // memcpy(Tx_Buffer, Rx_Buffer, size);
//  printf("\r\nRX_BUFFER\r\n");
//   printf(Rx_Buffer);

    //¹ØÓÚRX_Buffer  ÉÏÎ»»úÀíÂÛÉÏ Ó¦¸Ã´«ÈëµÄÊÇÒ»´®µÄÊý¾Ý£¬ ¶ø²»ÊÇÒ»¸öµ¥´¿µÄÎ»
    //¿ÉÒÔ¿´µ½Rx_Buffer Ïàµ±ÓÚÒ»¸öÊý×é£¬16Î»°É
    //¶ÔÓÚModbusÐ­Òé ÏÈ¾Ù¸öÀý×Ó
    /*
    ¶ÁÊý¾Ý£º    04               03                   02                                                                                            ÏÂÎ»»ú£º»Ø¸´ÆäÐèÒª¶ÁµÄÐÅÏ¢ 04 03 02 length data1 data2 *** crc
                    ´Ó»úµØÖ·          ¹¦ÄÜÂë¶Á                      ¼Ä´æÆ÷µØÖ·  ---------------------------------------¡·
    Ð´Êý¾Ý£º    04               06                   02             data1  data2 ***  crc16                ÏÂÎ»»ú£º»Ø¸´ÆäÐèÒªÐ´µÄÐÅÏ¢ 04 06 02  ****     £¨PS£¬Õâ¸öµØ·½»Ø¸´µÄÊÇÓëÉÏÎ»»úÏàÍ¬µÄ¶«Î÷£©
                    ´Ó»úµØÖ·          ¹¦ÄÜÂëÐ´                      ¼Ä´æÆ÷µØÖ·
    ÖÃÎ»On/off£º  04               05                   02              data1 data2 ***  crc16                   ÏÂÎ»»ú£º    Õâ¸öÊÇ·ñÐèÒª·µ»ØÊý¾Ý£¿
                    ´Ó»úµØÖ·          ¹¦ÄÜÂëÖÃÎ»                    ¼Ä´æÆ÷µØÖ·
    ±¨´íÐÅÏ¢£º  04               04                  02              data1 data2 ***  crc16            Õâ¸öÊÇÏÂÎ»»ú¶ÔÉÏÎ»»úµÄ·¢ËÍµÄÊý¾ÝÓÐ²»Í¬Òâ¼û²úÉúµÄ´íÎó
                    ´Ó»úµØÖ·          ¹¦ÄÜÂëÖÃÎ»                    ¼Ä´æÆ÷µØÖ·


    */

    //run crc check





    if (Rx_Buffer[0]==0x05) //Slave address 0x05
    {
        //  printf("\r\nSLocal Address ok!\r\n");

        switch (Rx_Buffer[1])
        {
            case 0x03:    //¶ÁÈ¡¹¦ÄÜÂë¼Ä´æÆ÷µÄ×´Ì¬ --ÊÇ·ñÎª¶ÁÊý¾Ý¹¦ÄÜÂë
                switch  (Rx_Buffer[2])
                {

                    case 0x01:   //Read Gas 1479A flow meter value
                        Tx_Buffer[0]=0x05; // Slave address
                        Tx_Buffer[1]=0x03;// function  code
                        Tx_Buffer[2]=0x01;// register address
                        //Ad conversion
                        AD_Voltage_Status = AD_Conversion();
                        //AD_Voltage_Status[0 1 2]   ·Ö±ð±íÊ¾1479A 627D 025d

                        //
                        temp = GasFloatValue_1479ACalc(AD_Voltage_Status[0]);
                        temp=3;
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
                        AD_Voltage_Status[1]=1.5;
                        Cavity_627D_Pressure_Status=AD_Voltage_Status[1];


                        //AD_Voltage_Status[0 1 2]   ·Ö±ð±íÊ¾1479A 627D 025d
                        temp = VacuumFloatValue_627DCalc(AD_Voltage_Status[1]);



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


                        //AD_Voltage_Status[0 1 2]   ·Ö±ð±íÊ¾1479A 627D 025d
                        temp = VacuumFloatValue_025DCalc(AD_Voltage_Status[2]);

                        temp=3;
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


            case 0x05:    //Ð´Èë¼Ä´æÆ÷µÄ×´Ì¬
                //Éè¶¨12¹âÂ·¡¢Éè¶¨Õæ¿Õ¶È¡¢Éè¶¨Á÷Á¿Öµ
                //Éè¶¨·½°¸£ºÍ¨¹ýÈ«¾Ö±äÁ¿´«ËÍ³öÈ¥£¬»¹ÊÇ×îºóÖ±½Óµ÷ÓÃÏà¹ØµÄº¯ÊýÄØ£¿
                //ÔÝ¶¨·½°¸£º Ö±½Óµ÷ÓÃÏà¹ØµÄº¯Êý
                //Õâ¸öÊÇRx_BufferµÄ¶¨Òå£ºunsigned char Rx_Buffer[2048]


                ////////////////////¹©ÆøÄ£Ê½////////////////////
                /*********************************************
                *¹©ÆøÄ£Ê½µÃÑ¡ÔñÓÐ2ÖÖ
                * Ä£Ê½1£ºÉÏÎ»»úÖ±½Ó¿ØÖÆÁ÷Á¿¼Æ £¬ Ã¿´Î¸øÁ÷Á¿¼ÆÉè¶¨µ±Ç°µÃÊýÖµ
                * Ä£Ê½2£ºÉÏÎ»»ú²»¿ØÖÆÁ÷Á¿£¬ÈÃÆäÍ¨¹ý×Ô¶¯µÃ±Õ»·½øÐÐ¿ØÖÆ
                *ÊµÏÖµÃ·½°¸
                *
                *
                *********************************************/
                switch  (Rx_Buffer[2])
                {


                    case 0x05:

                        if(Rx_Buffer[3]==0xff)

                        {
                            Normal_Debug_RunningMode=Rx_Buffer[3];

                            printf("Case 05 to open Debug Mode\n");
                        }
                        else
                        {
                            Normal_Debug_RunningMode=0x00;
                            printf("ase 05 to open Normal Running Mode\n");


                        }


                        break;
                    case 0x06: //Valve To Open

                        if(Rx_Buffer[3]==0xff)

                        {
                            Valve_Signal_Open=Rx_Buffer[3];

                            printf("Case 06 to open valves\r\n");
                        }
                        else
                        {
                            Valve_Signal_Open=0x00;
                            printf("ase 06 to close vavles\r\n");


                        }


                        break;

                    case 0x07: //PEV control  mode or 1479A control mode


                        if(Rx_Buffer[3]==0xff)

                        {
                            PEV_1479A_ControlMode=Rx_Buffer[3];

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
                        if(Rx_Buffer[3]==0xff) //±íÊ¾¹Ø±Õµ±Ç°Åç³öÄ£Ê½ x
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
                        if(Rx_Buffer[3]==0xff) //±íÊ¾¹Ø±Õµ±Ç°Åç³öÄ£Ê½ x
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



                }

            case 0x06:
                switch(Rx_Buffer[2])
                {
                    case 0x0A: //Vacuum value: Pressure value set 627D pressure value

                        //if the value is received, we feedback the same package that we get

                        size=Read_SOCK_Data_Buffer(s, Rx_Buffer);

                        memcpy(Tx_Buffer, Rx_Buffer, size);
                        Write_SOCK_Data_Buffer(s, Tx_Buffer,size);




                        //convert to float type
                        testdata.byteData[3]=Rx_Buffer[3];
                        testdata.byteData[2]=Rx_Buffer[4];
                        testdata.byteData[1]=Rx_Buffer[5];
                        testdata.byteData[0]=Rx_Buffer[6];

                        //Vacuum Value should be changed into Voltage value accoroding 627D manully

                        Package_Cavity_627D_Pressure_Set=testdata.floatData;



                        break;

                    case 0x09:  //1479A Gas  Flow value set



                        //if the value is received, we feedback the same package that we get

                        size=Read_SOCK_Data_Buffer(s, Rx_Buffer);

                        memcpy(Tx_Buffer, Rx_Buffer, size);
                        Write_SOCK_Data_Buffer(s, Tx_Buffer,size);




                        //convert to float type
                        testdata.byteData[3]=Rx_Buffer[3];
                        testdata.byteData[2]=Rx_Buffer[4];
                        testdata.byteData[1]=Rx_Buffer[5];
                        testdata.byteData[0]=Rx_Buffer[6];

                        //1479A Flow Set value

                        Package_Flow_1479A_Set=testdata.floatData;


                        break;
                    case 0x0C://Gas Puff mode: Puff Pressure value set

                        //if the value is received, we feedback the same package that we get

                        size=Read_SOCK_Data_Buffer(s, Rx_Buffer);

                        memcpy(Tx_Buffer, Rx_Buffer, size);
                        Write_SOCK_Data_Buffer(s, Tx_Buffer,size);




                        //convert to float type
                        testdata.byteData[3]=Rx_Buffer[3];
                        testdata.byteData[2]=Rx_Buffer[4];
                        testdata.byteData[1]=Rx_Buffer[5];
                        testdata.byteData[0]=Rx_Buffer[6];

                        //Vacuum Value should be changed into Voltage value accoroding 627D manully

                        Package_Cavity_627D_Puff_Set=testdata.floatData;


                        break;
                    case 0x0d://pid code setting

                        testdata.byteData[3]=Rx_Buffer[3];
                        testdata.byteData[2]=Rx_Buffer[4];
                        testdata.byteData[1]=Rx_Buffer[5];
                        testdata.byteData[0]=Rx_Buffer[6];
                        Duty_P=testdata.floatData;
                        testdata.byteData[3]=Rx_Buffer[7];
                        testdata.byteData[2]=Rx_Buffer[8];
                        testdata.byteData[1]=Rx_Buffer[9];
                        testdata.byteData[0]=Rx_Buffer[10];
                        Duty_I=testdata.floatData;
                        testdata.byteData[3]=Rx_Buffer[1];
                        testdata.byteData[2]=Rx_Buffer[12];
                        testdata.byteData[1]=Rx_Buffer[13];
                        testdata.byteData[0]=Rx_Buffer[14];
                        Duty_D=testdata.floatData;






                        //1479AFloatValue_Set=



                        break;

                    case 0x0E: //
                        Package_Valve_Status_Set[0]=Rx_Buffer[3];
                        Package_Valve_Status_Set[1]=Rx_Buffer[4];
                        break;

                    case 0x0F: //puff mode auxiliary valve

                        Package_Valve_Puff_Status_Set[0]=Rx_Buffer[3];
                        Package_Valve_Puff_Status_Set[1]=Rx_Buffer[4];


                        break;


                }





                break;


            default:
                break;
        }



    }
    else
    {
        //Èç¹û²»ÊÇ±¾»úµÄµØÖ·µÄÇé¿ö ÈçºÎ½øÐÐ´¦Àí



    }


    /*if (Rx_Buffer[0]==0x11)   //ÅÐ¶Ï¼Ä´æÆ÷ÖÐµÄÄÚÈÝ¾ÍÍ¨¹ýÕâ¸öÀ´½øÐÐ¡£
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
    Tx_Buffer[0]=0x05; // ±¾»úµØÖ·
    Tx_Buffer[1]=0x03;//¹¦ÄÜÃüÁîÂë
    Tx_Buffer[2]=0x01;//¼Ä´æÆ÷µØ

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

