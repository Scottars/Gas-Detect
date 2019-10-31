/*******************************************************************************
*
*                              普中科技
--------------------------------------------------------------------------------
* 实 验 名       : ADC转换实验
* 实验说明       : 通过printf打印AD检测数据 ，调节AD模块旁边的电位器 在串口助手上即可输出电压，
                    见文件内截图
* 连接方式       :
* 注    意       :  所用函数在头文件.c文件内
*******************************************************************************/


#include "public.h"
#include "printf.h"
#include "adc.h"
#include "systick.h"
#include "dac.h"    //实现ad 转换
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
extern uint16_t ADC_ConvertedValue[10][3];//adc 涓夎矾閲囬泦

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
float Package_AD_SamplingTimes=5;



//////////////////flag for mode switch/////////////////

int flag_Debug=0;
int flag_Normal=0;

int flag_Command=0;
int flag_Timing=0;

int flag_627D=0;
int flag_1479A=0;

////////////////////Singal to Open Part///////////////////
u8 Valve_Signal_Open=0x00;

//these two signal are not used
u8 Timing_Signal_Open=0x00;
u8 Command_Signal_Open=0x00;


////////////////////Mode Switch/////////////////// this should be able to change though the w5500
u8 Normal_Debug_RunningMode=0x00;

u8 Command_Timing_TriggerMode=0x00;

u8 Normal_Puff_RunningMode=0x00;

u8 PEV_1479A_ControlMode=0x00;

u8 LCD_Display_offon=0x00;
//This is to decide whether it is okay to send data to hsdd system
u8 hsdd_data_send_offon=0x00;



///////////////////Value to PID/////////////////
float Cavity_627D_Pressure_Status;
float Flow_1479A_Status;
float Cavity_025D_Pressure_Status;


//////////////////The limit value set///////////
/*
*we need to use this parameters to check the number which get from the pc
*
*
*/
float Cavity_627D_Pressure_SetMax=80;
float Cavity_627D_Pressure_SetMin=-20;
float Flow_1479A_SetMax=20;
float Flow_1479A_SetMin=-20;


float Cavity_627D_Puff_Pressure_SetMax=20;
float Cavity_627D_Puff_Pressure_SetMin=-20;

float Flow_1479A_Puff_SetMax=20;
float Flow_1479A_Puff_SetMin=-20;



float PID_P_SetMax=10;
float PID_I_SetMax=10;
float PID_D_SetMax=10;


float PID_P_SetMin=-10;
float PID_I_SetMin=-10;
float PID_D_SetMin=-10;
////////////////////error feedback/////////////////////
//30 none error
//31 an error
u8 Error_Communicate=0x00;
u8 Error_OverSet=0x00;

//puff ready siganl
u8 Pressure_Okay=0x00;
float Pressure_error;




/*
  // this part is the  DATA from the Internet
  //This is the target value
 //Normal Mode
float Package_Cavity_627D_Pressure_Set=0.3;

float Package_Flow_1479A_Set=0.3;


//Puff Mode
float Package_Flow_1479A_Puff_Set=6.0;

float Package_Cavity_627D_Puff_Set=6.0;

*/


////////////////////Value to control///////////////////
float Cavity_627D_Pressure_Set;
float Cavity_627D_Pressure_Default=1;


//////////////////use to 1479A mode or pev mode working alone///////
float PEV_FullyOpen_1479AMode=66;// ---we should translation this to the actual ad voltage
float _1479A_FullyOpen_PEVMode=5;
//for debug mode
float PEV_FullyClose=55;
float _1479A_FullyClose=0;


float Flow_1479A_Set;
float Flow_1479A_Default=0.5;
////////////////////Valve IO part////////////////
char Valve_Default_Status_Set[2]= {0x00,0x00};
char Valve_Operation_Status_Set[2];
char Valve_Puff_Status_Set[2];



///////////////////PID Duty Adjustment part//////
float Duty_P=5,Duty_I=0,Duty_D=0; // we can set it later


int main()
{
    u8 i,j,k;

    int size;
	int sendcount=0;
    FLOAT_BYTE testdata;
    CRC_BYTE crctestdata;

    float temptofun;
    /**************variable define part************************/


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



    //////////////***********TESTING***************////////////
    //In this part we want to test our program so we directly set some value
    //Cavity_627D_Pressure_Status=3.6    This is the status of cavity's pressure




    /////////////PWM Initial//////////////////////
    pwm_init(3599,0);  //we should just initialize it but not enable it




    /****************Initial system part*****************/

    ///////////////////////液晶屏初始化过程///////////////////////

    SystemInit();//初始化RCC 设置系统主频为72MHZ
    delay_init(72);      //延时初始化




    /////////////////////LED light initial////////////////////////////

    LED_Init(); //Initial
    LED10=1;
    LED12=1;
    LED11=1;

    ///////////////ADC Initial///////////////////ADC模块使用了三个片上AD PC0 PC1 PC2  分别是 AD10 11 12 通道
    //  adc1_init(); we did not use the dma mode , so we cannot make it correctly





    /////////////////Serial port initial//////////////用来进行串口通讯的，printf 实现了重定向
    printf_init(); //printf初始化

    ///////////////DAC Initial /////////////////DAC 部分的初始化，我们采用的是DAC1  也就是PA4 口的输出
    dma_test();                //DAC初始化
    // DAC_SetChannel1Data(DAC_Align_12b_R, 0);//初始值为0


    ///////////////Valve Status Initial/////////////

    ValveState_Init(); //供气阀门GPIO口的部分初始化



    ////////////////LCD Intial ///////////////////
    LCD_Init();    //液晶屏初始化
    GridLayer();  //显示屏的网络框架层
    Gas_StateLayer();//显示屏 供气阀状态初始化部分

    //////////////////网络初始化//////////////////////


    /*********************************************
    说明：  PC
                            IP：192.168.1.199
                            端口号：4001 未定，通过测试选用什么端口号都能够实现

                            测试的时候的目标的IP和目标的端口号使用的就是w5500的IP与端口号

                    w5500：
                            IP：192.168.1.198
                            端口号：5000


    **********************************************/
    printf("before lcd display");

    System_Initialization();    //STM32系统初始化函数(初始化STM32时钟及外设)
    Load_Net_Parameters();      //装载网络参数
    W5500_Hardware_Reset();     //硬件复位W5500
    W5500_Initialization();     //W5500 intial set




    /*****************IWDG--working begin************************/

    iwdg_init();

    // printf("watch dog working ");
//   ////////////////////////Exti interrupt /////////////////////////////
    Timing_Signal_init();
    //EXTI_DeInit();
    //we can not mask all the exti ,beacause we use this for internet
    // CLEAR_BIT(EXTI->IMR,EXTI_Line1);
    //      EXTI->IMR = EXTI_IMR_MR1;
    //EXTI->RTSR = EXTI_RTSR_TR0;




    while(1)
    {
        //printf("while mid");
        //      LED2=1;
        //  GPIO_SetBits(GPIOC,GPIO_Pin_14);     //IO口输出高电平
        //  delay_ms(100);
        //      GPIO_ResetBits(GPIOC,GPIO_Pin_14); //IO口输出低电平
        /*
        //LED12=0; //program running
        GPIO_SetBits(GPIOA,GPIO_Pin_11);     //IO口输出高电平
        //LED11=0; //program running
        GPIO_SetBits(GPIOC,GPIO_Pin_6);  //IO口输出高电平
        LED10=0; //program running
        GPIO_SetBits(GPIOC,GPIO_Pin_12);     //IO口输出高电平


        delay_ms(100);
        GPIO_ResetBits(GPIOA,GPIO_Pin_11);   //IO口输出高电平
        //LED11=0; //program running
        GPIO_ResetBits(GPIOC,GPIO_Pin_6);  //IO口输出高电平
        //LED10=0; //program running
        GPIO_ResetBits(GPIOC,GPIO_Pin_12);   //IO口输出高电平

            */
        // GPIO_SetBits(GPIOA,GPIO_Pin_11);     //IO口输出高电平

        LED12=1;
        //delay_ms(100);
        LED12=0;

        //GPIO_ResetBits(GPIOA,GPIO_Pin_11);   //IO口输出高电平
        //delay_ms(100);





        /////////////Update the watch dog  register//////////////////////
        IWDG_ReloadCounter();
       //rintf("Watch dog in while\r\n");
//




        W5500_Socket_Set();//W5500 port initial , we set the w5500 to tcp server mode
        /****************************************网络处理通讯处理********************************************/
        if(W5500_Interrupt)//处理W5500中断
        {
            //LED0      指示的是网络部分的信号的传输
            W5500_Interrupt_Process();//W5500中断处理程序框架
        }
        if((S0_Data & S_RECEIVE) == S_RECEIVE)//  this is flag to tell if we receive the data
        {
            S0_Data&=~S_RECEIVE; //we set that we didnot receive a package data
            Process_Package_Receive();//W5500接收并发送接收到的数据
            // size=Read_SOCK_Data_Buffer(0, Rx_Buffer);
            // Process_Socket_Data(0, 0,size);


        }


        if(Normal_Debug_RunningMode==0x00)
        {

            //printf("Normal Running Mode\r\n");
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


                Flow_1479A_Adjustment(_1479A_FullyClose);//to make it fully open


                VacuumValue_PID(PEV_FullyClose, Cavity_627D_Pressure_Status, Package_Duty_P,Package_Duty_I,Package_Duty_D);

                flag_Debug=0;


            }
            flag_Normal=1;

            if(Valve_Signal_Open==0x00) //close  valve or the system
            {
                //Wait for opening
                //In this state, you need to close the valve
                //In this state, you need to Set the closed-loop to closed ---?
                //In this state, you need to set the dac conversion to the minimal---?
                Valve_Operation_Status_Set[0]=0x00;
                Valve_Operation_Status_Set[1]=0x00;
                ValveStateChange(Valve_Operation_Status_Set);

                Flow_1479A_Adjustment(_1479A_FullyClose);//to make it fully close


                //   printf("Pev_fully close:%f\r\n",PEV_FullyClose);
                VacuumValue_PID(PEV_FullyClose, Cavity_627D_Pressure_Status, Package_Duty_P,Package_Duty_I,Package_Duty_D);


             // printf("Valve close\r\n");
            }
            else //Open command
            {
                printf("Valve Open\r\n");
                //Set actual valve should be opened
                Valve_Operation_Status_Set[0]=Package_Valve_Status_Set[0];//Get the valves to open from the package
                Valve_Operation_Status_Set[1]=Package_Valve_Status_Set[1];
                ValveStateChange(Valve_Operation_Status_Set);


                //Acutally the need both to adjust, so we can just need to write that code in the public area


                //
                if (PEV_1479A_ControlMode==0x00) //PEV control Mode 1
                {
                    //PEV,Default to PEV control
                    printf("PEV_Control Mode 1 \r\n");

                    //Default Set
                    // Set 1479A to fully open , we can use it fully open command   or use the DAC control to make it the biggest

                    Flow_1479A_Adjustment(_1479A_FullyOpen_PEVMode);//to make it fully open
                    //Directly use the value we set


                    //  printf("Call PID funtion\r\n");

                    //set pid parameter to the function
                    //we have set our default value to  Package_P  I D
                    //we should also check the number's reasonable value
                    //execute the PID function to set the new pwm duty ratio
                    printf("Package_Cavity_627D_Pressure_Set%f\r\n",Package_Cavity_627D_Pressure_Set);
                    printf("Cavity_627D_Pressure_Set%f\r\n",Cavity_627D_Pressure_Set);
                    VacuumValue_PID(Cavity_627D_Pressure_Set, Cavity_627D_Pressure_Status, Package_Duty_P,Package_Duty_I,Package_Duty_D);
                    //Cavity_627D_Pressure_Set
                    ///////////////if we can switch to the puff mode /////////////////

                    //we set the signal to enable puff mode
                    //different mdoe , what paramters we should
                    //we should test the set value and actual value's relation
                    //PS锛欰ctually we should not set the pressure to zero,
                    Pressure_error=(Cavity_627D_Pressure_Status-Package_Cavity_627D_Pressure_Set)/(Package_Cavity_627D_Pressure_Set);


                    if ((Pressure_error<=0.5) & (Pressure_error>=-0.5))
                    {
                        //we can send back the ready signal, there are several different comparison for 627d and 1479A
                        Pressure_Okay=0x01;
                    }
                    else
                    {
                        Pressure_Okay=0x00;
                    }
                    if (Command_Timing_TriggerMode==0x00) //Command trigger
                    {
                        printf("Command  Trigger Mode\r\n");
                        //all the mode needs to be default
                        //Only set to unpuff mode


                        //we should set the timing mode's interrupt signal to die
                        //disable exti1
                        //exti_init();
                        //CLEAR_BIT(EXTI_IMR,EXTI_IMR_IM1);
                        //exti_disable();
                        // CLEAR_BIT(EXTI->IMR,EXTI_Line1);


                        if(Normal_Puff_RunningMode==0x00) //unpuff
                        {

                            //this just like that, we open the puff mode
                            printf("Unpuff Mode\r\n");

                            /*Switch those value to unpuff value, so that we can make it happen, during next pid adjustment*/
                            // valve to normal
                            /* Valve_Operation_Status_Set[0]=Package_Valve_Status_Set[0];// we can get it from the Internet
                            Actually we won't execite this sentence
                             Valve_Operation_Status_Set[1]=Package_Valve_Status_Set[1];*/

                            //to close Puff mode status directly
                            //  ValveStateChange(Valve_Operation_Status_Set);

                            //1479A flow to fully open
                            Flow_1479A_Set=_1479A_FullyOpen_PEVMode; //make the 1479a fully open

                            //627D Vacuum Pressire to normal
                            Cavity_627D_Pressure_Set=Package_Cavity_627D_Pressure_Set;


                            //this mode we jump out to exexute the pid adjustment again


                        }
                        else //In  the PUff Mode
                        {
                            printf("puff Mode\r\n");

                            //to open Puff mode status
                            Valve_Operation_Status_Set[0]=Package_Valve_Status_Set[0] | Package_Valve_Puff_Status_Set[0];// we can get it from the Internet

                            Valve_Operation_Status_Set[1]=Package_Valve_Status_Set[1] | Package_Valve_Puff_Status_Set[1];

                            ValveStateChange(Valve_Operation_Status_Set);


                            /*which pressure control mode  we are right now   second layer */

                            if (PEV_1479A_ControlMode==0x00) // Second layer to set puff value
                            {
                                //In PEV control Mode
                                printf("Pev control mode 2\r\n");

                                Cavity_627D_Pressure_Set=Package_Cavity_627D_Puff_Set;

                                //update the target presste value of PID adjustment

                            }
                            else
                            {
                                /**************Actually we cannot enter this mode *********************/
                                //In 1479A control Mode锛?

                                printf("1479A control mode 2\r\n");

                                Flow_1479A_Set=Package_Flow_1479A_Puff_Set;   // we can get it from the package we receive

                            }

                        }




                    }
                    else // timing trigger mode
                    {
                        printf("Timing Trigger Mode\r\n");
                        //open timing interuupt singal
                        //enable exti1
                        //exti_init();
                        //  SET_BIT(EXTI->IMR,EXTI_Line1);
                        Timing_Signal_Check();


                        //if we swtich from command trigger to timing trigger, we need to close puff mode
                        //we can set a flag to judge




                        //we change the normal_puff_Running Mode throught the Mid
                        if(Normal_Puff_RunningMode==0x00) //unpuff  mode off
                        {
                            printf("Unpuff mode\r\n");
                            // Close all the puff valve

                            /*Switch those value to unpuff value, so that we can make it happen, during next pid adjustment*/
                            // valve to normal
                            /* Valve_Operation_Status_Set[0]=Package_Valve_Status_Set[0];// we can get it from the Internet

                             Valve_Operation_Status_Set[1]=Package_Valve_Status_Set[1];
                            */
                            //to close Puff mode status directly
                            ValveStateChange(Valve_Operation_Status_Set);

                            //1479A flow to fully open
                            Flow_1479A_Set=_1479A_FullyOpen_PEVMode;
                            //627D Vacuum Pressire to normal
                            Cavity_627D_Pressure_Set=Package_Cavity_627D_Pressure_Set;
                            //this mode we jump out to exexute the pid adjustment again

                        }


                        else // PUff Mode on
                        {

                            //open
                            //Open the puff valve   //about this we should know that  it's the best to set the whole puff valve instead of set an extra valve
                            printf("puff Mode\r\n");



                            //to open Puff mode status
                            Valve_Operation_Status_Set[0]=Package_Valve_Status_Set[0] | Package_Valve_Puff_Status_Set[0];// we can get it from the Internet

                            Valve_Operation_Status_Set[1]=Package_Valve_Status_Set[1] | Package_Valve_Puff_Status_Set[1];

                            //to open Puff mode status directly to open
                            ValveStateChange(Valve_Operation_Status_Set);



                            /*which pressure control mode  we are right now  */

                            if (PEV_1479A_ControlMode==0x00) //// Second layer to set puff value
                            {
                                //In PEV control Mode
                                printf("Pev control mode 2 \r\n");

                                Cavity_627D_Pressure_Set=Package_Cavity_627D_Puff_Set;


                                //update the target presste value of PID adjustment

                            }
                            else
                            {
                                //In 1479A control Mode

                                printf("1479A control mode 2\r\n");

                                Flow_1479A_Set=Package_Flow_1479A_Puff_Set;   // we can get it from the package we receive

                            }

                        }

                    }
                }
                else //1479A control mode 1
                {

                    printf("1479A_Control Mode 1\r\n");
                    // Set the PEV open in order to using 1479A control mode
                    // we also use pev control, but only to make it open about 100v
                    //set puff - PEV's Open voltage do we need a value to set?

                    VacuumValue_PID(PEV_FullyOpen_1479AMode, Cavity_627D_Pressure_Status, Package_Duty_P,Package_Duty_I,Package_Duty_D);




                    //printf("Flow Status:%f\n\n",Flow_1479A_Status);
                    // printf("Target Flow Value:%f\n\n",Flow_1479A_Set);

                    //1479A adjustment DAC

                    Flow_1479A_Adjustment(Flow_1479A_Set);
                    //pay attention to this, if we directall



                    ///////////////if we can switch to the puff mode /////////////////

                    //we set the signal to enable puff mode
                    //different mdoe , what paramters we should
                    //we should test the set value and actual value's relation
                    Pressure_error=(Cavity_627D_Pressure_Status-Package_Cavity_627D_Pressure_Set)/(Package_Cavity_627D_Pressure_Set);

                    if ((Pressure_error<=0.5) & (Pressure_error<=-0.5))
                    {
                        //we can send back the ready signal, there are several different comparison for 627d and 1479A
                        Pressure_Okay=0x01;
                    }
                    else
                    {
                        Pressure_Okay=0x00;
                    }



                    //Send back the pressure of the cavity
                    if (Command_Timing_TriggerMode==0x00) //Command trigger
                    {
                        printf("Command	Trigger Mode\r\n");
                        //all the mode needs to be default
                        //Only set to unpuff mode

                        //exti_init();
                        //CLEAR_BIT(EXTI_IMR,EXTI_IMR_IM1);
                        //exti_disable();
                        //CLEAR_BIT(EXTI->IMR,EXTI_Line1);
                        if(Normal_Puff_RunningMode==0x00) //unpuff
                        {

                            printf("Unpuff Mode\r\n");

                            /*Switch those value to unpuff value, so that we can make it happen, during next pid adjustment*/
                            // valve to normal
                            /*  Valve_Operation_Status_Set[0]=Package_Valve_Status_Set[0];// we can get it from the Internet

                              Valve_Operation_Status_Set[1]=Package_Valve_Status_Set[1];
                            */
                            //to close Puff mode status directly
                            ValveStateChange(Valve_Operation_Status_Set);

                            //1479A flow to normal
                            Flow_1479A_Set=Package_Flow_1479A_Set;

                            //627D Vacuum Pressire to fully open
                            Cavity_627D_Pressure_Set=PEV_FullyOpen_1479AMode;


                            //this mode we jump out to exexute the pid adjustment again


                        }
                        else //In  the PUff Mode
                        {
                            printf("puff Mode\r\n");

                            //to open Puff mode status
                            Valve_Operation_Status_Set[0]=Package_Valve_Status_Set[0] | Package_Valve_Puff_Status_Set[0];// we can get it from the Internet

                            Valve_Operation_Status_Set[1]=Package_Valve_Status_Set[1] | Package_Valve_Puff_Status_Set[1];

                            ValveStateChange(Valve_Operation_Status_Set);


                            /*which pressure control mode  we are right now   second layer */

                            if (PEV_1479A_ControlMode==0x00) // Second layer to set puff value
                                /*Actually we cannot be in this area*/
                            {
                                //In PEV control Mode
                                printf("Pev control mode 2\r\n");

                                Cavity_627D_Pressure_Set=Package_Cavity_627D_Puff_Set;

                                //update the target presste value of PID adjustment
                                //we have already do this in the adjust part

                            }
                            else
                            {
                                //In 1479A control Mode

                                printf("1479A control mode 2\r\n");

                                Flow_1479A_Set=Package_Flow_1479A_Puff_Set;   // we can get it from the package we receive


                            }


                        }




                    }
                    else // timing trigger mode
                    {


                        printf("Timing Trigger Mode \r\n");

                        //CLEAR_BIT(EXTI_IMR,EXTI_IMR_IM1);
                        //EXTI_DeInit();
                        //exti_init();
                        //SET_BIT(EXTI->IMR,EXTI_Line1);
                        Timing_Signal_Check();


                        if(Normal_Puff_RunningMode==0x00) //unpuff  mode off
                        {
                            printf("Unpuff mode \r\n");
                            // Close all the puff valve

                            /*Switch those value to unpuff value, so that we can make it happen, during next pid adjustment*/
                            // valve to normal
                            /* Valve_Operation_Status_Set[0]=Package_Valve_Status_Set[0];// we can get it from the Internet

                             Valve_Operation_Status_Set[1]=Package_Valve_Status_Set[1];*/

                            //to close Puff mode status directly
                            // ValveStateChange(Valve_Operation_Status_Set);

                            //1479A flow to normal
                            Flow_1479A_Set=Package_Flow_1479A_Set;
                            //627D Vacuum Pressire to normal
                            Cavity_627D_Pressure_Set=PEV_FullyOpen_1479AMode;
                            //this mode we jump out to exexute the pid adjustment again


                        }
                        else // PUff Mode on
                        {

                            //open
                            //Open the puff valve   //about this we should know that  it's the best to set the whole puff valve instead of set an extra valve
                            printf("puff Mode \r\n");



                            //to open Puff mode status
                            Valve_Operation_Status_Set[0]=Package_Valve_Status_Set[0] | Package_Valve_Puff_Status_Set[0];// we can get it from the Internet

                            Valve_Operation_Status_Set[1]=Package_Valve_Status_Set[1] | Package_Valve_Puff_Status_Set[1];

                            //to open Puff mode status directly to open
                            ValveStateChange(Valve_Operation_Status_Set);



                            /*which pressure control mode  we are right now  */

                            if (PEV_1479A_ControlMode==0x00) //// Second layer to set puff value
                            {
                                //In PEV control Mode
                                printf("Pev control mode 2 \r\n");

                                Cavity_627D_Pressure_Set=Package_Cavity_627D_Puff_Set;



                                //update the target presste value of PID adjustment




                            }
                            else
                            {
                                //In 1479A control Mode

                                printf("1479A control mode 2 \r\n");

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

                Flow_1479A_Adjustment(_1479A_FullyClose);//to make it fully open


                VacuumValue_PID(PEV_FullyClose, Cavity_627D_Pressure_Status, Package_Duty_P,Package_Duty_I,Package_Duty_D);

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
                // we still need to make this better
                Valve_Operation_Status_Set[0]=0x00;
                Valve_Operation_Status_Set[1]=0x00;
                ValveStateChange(Valve_Operation_Status_Set);

                printf("debug mode, valve closed");
            }


            printf("we are in dubug mode");

            //this mode is dubug running

        }

        //we should also set set currret status to the status register
        //we can use flag to get if we need to update the lcd
        if(LCD_Display_offon==0xff)
        {
            printf("LCD On Mode");
            //LCD_Display_Flag=1000;

            Status_LCD_Update();
        }
        //  LCD_Display_Flag--;
        Status_Register_Update_DMA();//we use this sentence to pid or dac adjust
        //Status_LCD_Update();

		// we set a flag to control whether it is okay to send out the data 
		hsdd_data_send_offon=0xff;
		delay_us(1000);

		if (hsdd_data_send_offon==0xff)
		{
			//printf("we are sending date to socket 1\r\n");

			//Hsdd_Data_Send();
	
			
			
			if (sendcount<100)
			{
				sendcount = sendcount + 1;
//				printf("i的大小%d",sendcount);
				
				
				//sprintf(str,'\r\nWelcome To YiXinElec!%d\r\n',i);
				//sprintf(str,'%d',i);
				//a=strlen(str);
			
				 Tx_Buffer[0]=0x05; // Slave address
				Tx_Buffer[1]=0x03;// function  code
				Tx_Buffer[2]=0x01;// register address
				Tx_Buffer[3]=0x04;//length of data
				testdata.floatData=sendcount+0.5;
				Tx_Buffer[4]=testdata.byteData[3];
				Tx_Buffer[5]=testdata.byteData[2];
				Tx_Buffer[6]=testdata.byteData[1];
				Tx_Buffer[7]=testdata.byteData[0];
				crctestdata.CrcData=GetCRC16(Tx_Buffer,8);
				   Tx_Buffer[8]=crctestdata.byteData[0];
					Tx_Buffer[9]=crctestdata.byteData[1];
				
				Write_SOCK_Data_Buffer(1, Tx_Buffer,10);//指定Socket(0~7)发送数据处理,端口0发送23字节数据

//				delay_us(1);

				
				Tx_Buffer[0]=0x05; // Slave address
				   Tx_Buffer[1]=0x03;// function  code
				   Tx_Buffer[2]=0x02;// register address
				   Tx_Buffer[3]=0x04;//length of data
				   testdata.floatData=sendcount+1.5;
				   Tx_Buffer[4]=testdata.byteData[3];
				   Tx_Buffer[5]=testdata.byteData[2];
				   Tx_Buffer[6]=testdata.byteData[1];
				   Tx_Buffer[7]=testdata.byteData[0];
				   crctestdata.CrcData=GetCRC16(Tx_Buffer,8);
					  Tx_Buffer[8]=crctestdata.byteData[0];
					   Tx_Buffer[9]=crctestdata.byteData[1];
				   
				   Write_SOCK_Data_Buffer(1, Tx_Buffer,10);//指定Socket(0~7)发送数据处理,端口0发送23字节数据
				
				

				//Write_SOCK_Data_Buffer(0, Tx_Buffer,10);//指定Socket(0~7)发送数据处理,端口0发送23字节数据
				//printf("发送socket0 之后");
			
				
				//memcpy(Tx_Buffer, "", 23);	
				//Write_SOCK_Data_Buffer(0, Tx_Buffer, 23);//指定Socket(0~7)发送数据处理,端口0发送23字节数据
				
				//Write_SOCK_Data_Buffer(0, Tx_Buffer,a);//指定Socket(0~7)发送数据处理,端口0发送23字节数据
			}
			else
			{
				sendcount=0;
			}
			
			
		}


        


        LED12=0; //program running
   
//        delay_us(2000000); 



    }
}



void Process_Package_Receive()
{
    unsigned char RX_Buffer_Receive[2048];
    int size;
    int i=5;
    int j=0;
    int Package_Size;
    int start_point=0;
    size=Read_SOCK_Data_Buffer(0, Rx_Buffer);


    printf("Process_Package_Receive:%d\r\n",size);

    while ((i>0)&(size>0))
    {
        printf("still in the whiler\r\n");
        printf("Start Point %d\r\n",start_point);
        printf("Size  %d\r\n",size);


        switch (Rx_Buffer[start_point+1])
        {



            case 0x03:
                //printf("In the Rx_bufffer part\r\n");
                Package_Size=6;

                for(j=0; j<Package_Size; j++)
                {
                    RX_Buffer_Receive[j]=Rx_Buffer[start_point+j];
                    printf("RX_Buffer_Receive:%x\r\n",RX_Buffer_Receive[j]);



                }
                //if(CheckCRC16(RX_Buffer_Receive,4+(int)Rx_Buffer[3+start_point]))//CRC check, correct crc the
                //the second data is the actual length we get
                if(CheckCRC16(RX_Buffer_Receive,Package_Size-2))//CRC check, correct crc the
                {
                    printf("case 03 CRC check okay\r\n");

                    printf("actual data length we reveive:%d\r\n",4+(int)Rx_Buffer[3+start_point]);
                    printf("Package_Size%d\r\n",Package_Size);

                    Process_Socket_Data(0,start_point,Package_Size);



                }
                else //CRC check, wrong crc, set communication error
                {
                    printf("case 03 CRC check wrong\r\n");
                    printf("actual data length we reveive:%d\r\n",4+(int)Rx_Buffer[3+start_point]);
                    printf("Package_Size%d\r\n",Package_Size);
                    Error_Communicate=0x01;
                    LED11=0; //crc check running  signal

                }
                start_point += Package_Size;
                size -= 6;
                i--;


                break;
            case 0x05:
                //printf("In the Rx_bufffer part\r\n");
                Package_Size=7;
                for(j=0; j<Package_Size; j++)
                {
                    RX_Buffer_Receive[j]=Rx_Buffer[start_point+j];
                    printf("RX_Buffer_Receive:%x\r\n",RX_Buffer_Receive[j]);




                }
                // if(CheckCRC16(RX_Buffer_Receive,4+(int)Rx_Buffer[3+start_point])) //CRC check, correct crc
                if(CheckCRC16(RX_Buffer_Receive,Package_Size-2))//CRC check, correct crc the
                {
                    printf("case05 CRC check okay\r\n");
                    printf("actual data length we reveive:%d\r\n",4+(int)Rx_Buffer[3+start_point]);
                    printf("Package_Size%d\r\n",Package_Size);


                    Process_Socket_Data(0,start_point,Package_Size);



                }
                else //CRC check, wrong crc, set communication error
                {
                    printf("case 05 CRC check wrong \r\n");
                    Error_Communicate=0x01;
                    LED11=0; //crc check running  signal
                }


                start_point += Package_Size;
                size -= 7;
                i--;


                break;


            case 0x06:

                Package_Size=4+Rx_Buffer[start_point+3]+2;
                for(j=0; j<Package_Size; j++)
                {
                    RX_Buffer_Receive[j]=Rx_Buffer[start_point+j];



                }

                if(CheckCRC16(RX_Buffer_Receive,4+(int)Rx_Buffer[3+start_point])) //CRC check, correct crc
                {
                    printf("case 06 CRC check okay\r\n");
                    printf("actual data length we reveive:%d\r\n",4+(int)Rx_Buffer[3+start_point]);


                    Process_Socket_Data(0,start_point,Package_Size);
                    printf("Package_Size%d\r\n",Package_Size);

                }
                else //CRC check, wrong crc, set communication error
                {
                    printf("case 06 CRC check wrong\r\n");
                    printf("actual data length we reveive:%d\r\n",4+(int)Rx_Buffer[3+start_point]);
                    Error_Communicate=0x01;
                    LED11=0; //crc check running  signal
                }


                start_point += Package_Size;
                size -= Package_Size;
                i--;

                break;
            case 0x08:
                Package_Size=6;
                for(j=0; j<Package_Size; j++)
                {
                    RX_Buffer_Receive[j]=Rx_Buffer[start_point+j];



                }
                //printf("In the Rx_bufffer part\r\n");
                // if(CheckCRC16(RX_Buffer_Receive,4+start_point)) //CRC check, correct crc
                if(CheckCRC16(RX_Buffer_Receive,Package_Size-2)) //CRC check, correct crc
                {
                    printf("case 08 CRC check okay\r\n");


                    Process_Socket_Data(0,start_point,Package_Size);



                }
                else //CRC check, wrong crc, set communication error
                {
                    printf("case 08 CRC check wrong\r\n");
                    Error_Communicate=0x01;
                    LED11=0; //crc check running  signal
                }
                start_point += Package_Size;
                size -= 6;
                i--;


                break;

            default:
                i--;
                break;


        }



    }




    printf("we have out of the while loop\r\n");


}



/*******************************************************************************
* Function name  :  Process_Socket_Data
* Description  : Analysis the package we receive from the pc
* Input : Socket 0-7
* Output  :  None
* Return Value :  None
* Attention: we process the data we get from the pc
*******************************************************************************/
void Process_Socket_Data(SOCKET s,int Package_Start,int Package_Size)
{

    FLOAT_BYTE testdata;
    CRC_BYTE crctestdata;
    char floattest[10];
    char test1[4];
    unsigned short size;//接收到的buffer 的大小
    ////////
    ///////////////Valve Gas Part///////

    u8 ValveValue_Set[12];  //来自网络传入过来的信号
    char *ValveValue_Status;
    u8 i;

    ////////////////PID参数设定/////////////////////////
    float Kp,Ki,Kd;


    ////////////////CRC_Mid/////////////////////////
    unsigned int CRC_Mid;

    ///////////////////AD转换部分////////////////
    float *AD_Voltage_Status;
    float AD_temp;

    float temp=0; // 用来进行AD转换使用的变量


    unsigned short Actual_size;


    //  size=Read_SOCK_Data_Buffer(s, Rx_Buffer);
    //we should judge the size so we can process more data

    /*
        switch (Rx_Buffer[1])
        {

            case 0x03:


                if(CheckCRC16(Rx_Buffer,4)) //CRC check, correct crc
                {
                    printf("CRC check okay\r\n");
                }

                else //CRC check, wrong crc, set communication error
                {
                    printf("CRC check wrong\r\n");
                    Error_Communicate=0x31;
                }
                Actual_size=6;

                break;
            case 0x05:
                if(CheckCRC16(Rx_Buffer,4)) //CRC check, correct crc
                {
                    printf("CRC check okay\r\n");
                }
                else //CRC check, wrong crc, set communication error
                {
                    printf("CRC check wrong\r\n");
                    Error_Communicate=0x31;
                }

                Actual_size=6;
                break;
            case 0x06:

                if(CheckCRC16(Rx_Buffer,3+1+(int)Rx_Buffer[3])) //CRC check, correct crc
                {
                    printf("CRC check okay\r\n");
                }
                else //CRC check, wrong crc, set communication error
                {
                    printf("CRC check wrong\r\n");
                    Error_Communicate=0x31;
                }
                Actual_size=6+Rx_Buffer[3];

            case 0x08:
                if(CheckCRC16(Rx_Buffer,4)) //CRC check, correct crc
                {
                    printf("CRC check okay\r\n");
                }

                else //CRC check, wrong crc, set communication error
                {
                    printf("CRC check wrong\r\n");
                    Error_Communicate=0x31;
                }


                Actual_size=6;
                break;

        }*/

    //when we transfer the data very fast, it is not okay to recive the enough package
    /* if(size==Actual_size)
     {
         printf("Length check okay\r\n");
     }
     else
     {

         printf("Length check wrong: %d\r\n",size);
     }*/





    //printf("\r\nSIZE:%d\r\n",size);
    // memcpy(Tx_Buffer, Rx_Buffer, size);
//  printf("\r\nRX_BUFFER\r\n");
//   printf(Rx_Buffer);

    //关于RX_Buffer  上位机理论上 应该传入的是一串的数据， 而不是一个单纯的位
    //可以看到Rx_Buffer 相当于一个数组，16位吧
    //对于Modbus协议 先举个例子
    /*
    读数据：    04               03                   02                                                                                            下位机：回复其需要读的信息 04 03 02 length data1 data2 *** crc
                    从机地址          功能码读                      寄存器地址  ---------------------------------------》
    写数据：    04               06                   02             data1  data2 ***  crc16                下位机：回复其需要写的信息 04 06 02  ****     （PS，这个地方回复的是与上位机相同的东西）
                    从机地址          功能码写                      寄存器地址
    置位On/off：  04               05                   02              data1 data2 ***  crc16                   下位机：    这个是否需要返回数据？
                    从机地址          功能码置位                    寄存器地址
    报错信息：  04               04                  02              data1 data2 ***  crc16            这个是下位机对上位机的发送的数据有不同意见产生的错误
                    从机地址          功能码置位                    寄存器地址


    */
    //

    /* printf("Length Package size: %d\r\n",size);
     for(i=0;i<6;i++)
         {
          printf("Rx_buffer:%x\r\n",Rx_Buffer[i]);
         }
    */






    if (Rx_Buffer[0+Package_Start]==0x05) //Slave address 0x05
    {

        //  printf("\r\nSLocal Address ok!\r\n");
        //after the slave address, we use the length to make sure the package is complete

        switch (Rx_Buffer[1+Package_Start])
        {
            case 0x03:    //读取功能码寄存器的状态 --是否为读数据功能码


                switch  (Rx_Buffer[2+Package_Start])
                {

                    case 0x01:   //Read Gas 1479A flow meter value

                        Tx_Buffer[0]=0x05; // Slave address
                        Tx_Buffer[1]=0x03;// function  code
                        Tx_Buffer[2]=0x01;// register address
                        Tx_Buffer[3]=0x04;//length of data
                        //Ad conversion
                        //  AD_Voltage_Status = AD_Conversion();
                        //AD_Voltage_Status[0 1 2]   分别表示1479A 627D 025d
                        // Flow_1479A_Status=1.5;
                        // AD_temp=AD_Conversion_1479A(Package_AD_SamplingTimes);
                        AD_temp=AD_Conversion_1479A_DMA();

                        //
                        temp = ADVoltage_2_Flow1479A(AD_temp);
                        //  printf("INternet  1479A Actual value:%f\r\n",temp);
                        //Transfer the float data to hex data
                        testdata.floatData=temp;


                        Tx_Buffer[4]=testdata.byteData[3];
                        Tx_Buffer[5]=testdata.byteData[2];
                        Tx_Buffer[6]=testdata.byteData[1];
                        Tx_Buffer[7]=testdata.byteData[0];

                        //GetCRC16
                        crctestdata.CrcData=GetCRC16(Tx_Buffer,8);


                        Tx_Buffer[8]=crctestdata.byteData[0];
                        Tx_Buffer[9]=crctestdata.byteData[1];

                        //  Tx_Buffer[0]=0xff;

                        Write_SOCK_Data_Buffer(s, Tx_Buffer, 10);

                        break;


                    case 0x02: //Read 627D pressure value
                        Tx_Buffer[0]=0x05; // Slave address
                        Tx_Buffer[1]=0x03;// function  code
                        Tx_Buffer[2]=0x02;// register address
                        Tx_Buffer[3]=0x04;//length of data

                        //Ad conversion
                        //AD_Voltage_Status = AD_Conversion();

                        // Cavity_627D_Pressure_Status=AD_Voltage_Status[1];

                        //AD_Voltage_Status[0 1 2]   分别表示1479A 627D 025d
                        AD_temp=AD_Conversion_627D_DMA();

                        //
                        temp = ADVoltage_2_Pressure627D(AD_temp);
                        printf("INternet 627D Actual value:%f\r\n",temp);

                        //float value to hex
                        testdata.floatData=temp;

                        Tx_Buffer[4]=testdata.byteData[3];
                        Tx_Buffer[5]=testdata.byteData[2];
                        Tx_Buffer[6]=testdata.byteData[1];
                        Tx_Buffer[7]=testdata.byteData[0];

                        //GetCRC16
                        crctestdata.CrcData=GetCRC16(Tx_Buffer,8);


                        Tx_Buffer[8]=crctestdata.byteData[0];
                        Tx_Buffer[9]=crctestdata.byteData[1];

                        //  Tx_Buffer[0]=0xff;

                        Write_SOCK_Data_Buffer(s, Tx_Buffer, 10);

                        break;

                    case 0x03: //Read CDG025D  vacuum value
                        Tx_Buffer[0]=0x05; // Slave address
                        Tx_Buffer[1]=0x03;// function  code
                        Tx_Buffer[2]=0x03;// register address
                        Tx_Buffer[3]=0x04;//length of data
                        //ad conbersion
                        //  AD_Voltage_Status = AD_Conversion();


                        //AD_Voltage_Status[0 1 2]   分别表示1479A 627D 025d
                        AD_temp=AD_Conversion_025D_DMA();

                        //
                        temp = ADVoltage_2_Pressure025D(AD_temp);
                        printf("INternet	025D Actual value:%f\r\n",temp);

                        //float to hex
                        testdata.floatData=temp;


                        Tx_Buffer[4]=testdata.byteData[3];
                        Tx_Buffer[5]=testdata.byteData[2];
                        Tx_Buffer[6]=testdata.byteData[1];
                        Tx_Buffer[7]=testdata.byteData[0];

                        //GetCRC16
                        crctestdata.CrcData=GetCRC16(Tx_Buffer,8);


                        Tx_Buffer[8]=crctestdata.byteData[0];
                        Tx_Buffer[9]=crctestdata.byteData[1];

                        //  Tx_Buffer[0]=0xff;

                        Write_SOCK_Data_Buffer(s, Tx_Buffer, 10);

                        break;

                    case 0x04://Read Gas Feed value Status
                        Tx_Buffer[0]=0x05; // Slave address
                        Tx_Buffer[1]=0x03;// function  code
                        Tx_Buffer[2]=0x04;// register address
                        Tx_Buffer[3]=0x02;//length of data
                        //read gas valve status

                        ValveValue_Status=Gas_State_Read();
                        Tx_Buffer[4]=ValveValue_Status[0];
                        Tx_Buffer[5]=ValveValue_Status[1];

                        //GetCRC16
                        crctestdata.CrcData=GetCRC16(Tx_Buffer,6);

                        Tx_Buffer[6]=crctestdata.byteData[0];
                        Tx_Buffer[7]=crctestdata.byteData[1];

                        Write_SOCK_Data_Buffer(s, Tx_Buffer, 8);
                        break;
                    case 0x14: //read whether pressure is okay  for puff

                        Tx_Buffer[0]=0x05; // Slave address
                        Tx_Buffer[1]=0x03;// function  code
                        Tx_Buffer[2]=0x14;// register address
                        Tx_Buffer[3]=0x01;
                        //read gas valve status
                        printf("Read if we are ready to inspire \r\n");
                        ValveValue_Status=Gas_State_Read();
                        Tx_Buffer[4]=Pressure_Okay;

                        //GetCRC16
                        crctestdata.CrcData=GetCRC16(Tx_Buffer,5);

                        Tx_Buffer[5]=crctestdata.byteData[0];
                        Tx_Buffer[6]=crctestdata.byteData[1];

                        Write_SOCK_Data_Buffer(s, Tx_Buffer, 7);
                        break;

                    case 0x15: //Read VacuumValue set

                        Tx_Buffer[0]=0x05; // Slave address
                        Tx_Buffer[1]=0x03;// function  code
                        Tx_Buffer[2]=0x15;// register address
                        Tx_Buffer[3]=0x04;//length of data
                        //read gas valve status
                        printf("Read Vacuum value set\r\n");

                        testdata.floatData=Package_Cavity_627D_Pressure_Set;


                        Tx_Buffer[4]=testdata.byteData[3];
                        Tx_Buffer[5]=testdata.byteData[2];
                        Tx_Buffer[6]=testdata.byteData[1];
                        Tx_Buffer[7]=testdata.byteData[0];

                        //GetCRC16
                        crctestdata.CrcData=GetCRC16(Tx_Buffer,8);


                        Tx_Buffer[8]=crctestdata.byteData[0];
                        Tx_Buffer[9]=crctestdata.byteData[1];

                        //  Tx_Buffer[0]=0xff;

                        Write_SOCK_Data_Buffer(s, Tx_Buffer, 10);

                        break;
                        break;
                    case 0x16://read 1479A flow value (it is stable part set )


                        Tx_Buffer[0]=0x05; // Slave address
                        Tx_Buffer[1]=0x03;// function  code
                        Tx_Buffer[2]=0x16;// register address
                        Tx_Buffer[3]=0x04;//length of data
                        //read gas valve status
                        printf("Read 1479A flow  value set\r\n");

                        testdata.floatData=Package_Flow_1479A_Set;


                        Tx_Buffer[4]=testdata.byteData[3];
                        Tx_Buffer[5]=testdata.byteData[2];
                        Tx_Buffer[6]=testdata.byteData[1];
                        Tx_Buffer[7]=testdata.byteData[0];

                        //GetCRC16
                        crctestdata.CrcData=GetCRC16(Tx_Buffer,8);


                        Tx_Buffer[8]=crctestdata.byteData[0];
                        Tx_Buffer[9]=crctestdata.byteData[1];

                        //  Tx_Buffer[0]=0xff;

                        Write_SOCK_Data_Buffer(s, Tx_Buffer, 10);

                        break;
                    case 0x17: //read puff value set .we actually have two puff set


                        Tx_Buffer[0]=0x05; // Slave address
                        Tx_Buffer[1]=0x03;// function  code
                        Tx_Buffer[2]=0x17;// register address
                        Tx_Buffer[3]=0x04;//length of data

                        printf("Read puff  value set\r\n");

                        testdata.floatData=Package_Cavity_627D_Puff_Set;


                        Tx_Buffer[4]=testdata.byteData[3];
                        Tx_Buffer[5]=testdata.byteData[2];
                        Tx_Buffer[6]=testdata.byteData[1];
                        Tx_Buffer[7]=testdata.byteData[0];

                        //GetC RC16
                        crctestdata.CrcData=GetCRC16(Tx_Buffer,8);


                        Tx_Buffer[8]=crctestdata.byteData[0];
                        Tx_Buffer[9]=crctestdata.byteData[1];

                        //  Tx_Buffer[0]=0xff;

                        Write_SOCK_Data_Buffer(s, Tx_Buffer, 10);

                        break;
                    case 0x18://read pid parameters set

                        Tx_Buffer[0]=0x05; // Slave address
                        Tx_Buffer[1]=0x03;// function  code
                        Tx_Buffer[2]=0x18;// register address
                        Tx_Buffer[3]=0x0c;//length of data
                        //read gas valve status
                        printf("Read pid parameters \r\n");



                        testdata.floatData=Package_Duty_P;

                        Tx_Buffer[4]=testdata.byteData[3];
                        Tx_Buffer[5]=testdata.byteData[2];
                        Tx_Buffer[6]=testdata.byteData[1];
                        Tx_Buffer[7]=testdata.byteData[0];

                        testdata.floatData=Package_Duty_I;

                        Tx_Buffer[8]=testdata.byteData[3];
                        Tx_Buffer[9]=testdata.byteData[2];
                        Tx_Buffer[10]=testdata.byteData[1];
                        Tx_Buffer[11]=testdata.byteData[0];

                        testdata.floatData=Package_Duty_D;

                        Tx_Buffer[12]=testdata.byteData[3];
                        Tx_Buffer[13]=testdata.byteData[2];
                        Tx_Buffer[14]=testdata.byteData[1];
                        Tx_Buffer[15]=testdata.byteData[0];

                        //GetCRC16
                        crctestdata.CrcData=GetCRC16(Tx_Buffer,16);


                        Tx_Buffer[16]=crctestdata.byteData[0];
                        Tx_Buffer[17]=crctestdata.byteData[1];

                        Write_SOCK_Data_Buffer(s, Tx_Buffer, 18);

                        break;
                }
                break;




            case 0x05:    //写入寄存器的状态
                //设定12光路、设定真空度、设定流量值
                //设定方案：通过全局变量传送出去，还是最后直接调用相关的函数呢？
                //暂定方案： 直接调用相关的函数
                //这个是Rx_Buffer的定义：unsigned char Rx_Buffer[2048]


                ////////////////////供气模式////////////////////
                /*********************************************
                *供气模式得选择有2种
                * 模式1：上位机直接控制流量计 ， 每次给流量计设定当前得数值
                * 模式2：上位机不控制流量，让其通过自动得闭环进行控制
                *实现得方案
                *
                *
                *********************************************/

                switch  (Rx_Buffer[2+Package_Start])
                {


                    case 0x05:

                        if(Rx_Buffer[4+Package_Start]==0x01)

                        {
                            Normal_Debug_RunningMode=0xff;

                            printf("Case 05 to open Debug Mode\n");
                        }
                        else
                        {
                            Normal_Debug_RunningMode=0x00;
                            printf("Case 05 to open Normal Running Mode\n");


                        }
                        for(i=0; i<Package_Size; i++)
                        {
                            Tx_Buffer[i]=Rx_Buffer[Package_Start+i];
                            printf("Txbuffer content:%x\r\n",Tx_Buffer[i]);
                            printf("I number:%d\r\n",i);
                        }
                        printf("Set Rxbuffer okay\r\n");
                        //memcpy(Tx_Buffer, Rx_Buffer, size);
                        Write_SOCK_Data_Buffer(s, Tx_Buffer,Package_Size);

                        break;
                    case 0x06: //Valve To Open

                        if(Rx_Buffer[4+Package_Start]==0x01)

                        {
                            Valve_Signal_Open=0xff;

                            printf("Case 06 to open valves\r\n");
                        }
                        else
                        {
                            Valve_Signal_Open=0x00;
                            printf("case 06 to close vavles\r\n");


                        }
                        for(i=0; i<Package_Size; i++)
                        {
                            Tx_Buffer[i]=Rx_Buffer[Package_Start+i];
                        }
                        printf("Set Rxbuffer okay\r\n");
                        //memcpy(Tx_Buffer, Rx_Buffer, size);
                        Write_SOCK_Data_Buffer(s, Tx_Buffer,Package_Size);


                        break;

                    case 0x07: //PEV control  mode or 1479A control mode
                        if(Rx_Buffer[4+Package_Start]==0x01)

                        {
                            PEV_1479A_ControlMode=0xff;

                            printf("Case 07: to 1479A control Mode\r\n");
                        }
                        else
                        {
                            PEV_1479A_ControlMode=0x00;
                            printf("Case 07 to PEV control mode\r\n");


                        }

                        for(i=0; i<Package_Size; i++)
                        {
                            Tx_Buffer[i]=Rx_Buffer[Package_Start+i];
                        }
                        printf("Set Rxbuffer okay\r\n");
                        //memcpy(Tx_Buffer, Rx_Buffer, size);
                        Write_SOCK_Data_Buffer(s, Tx_Buffer,Package_Size);

                        break;

                    case 0x08:  //Command Mode or timing trigger mode
                        //use the puff mode or not
                        //we can use a new parameter in the pid function to enable puff or unpuff
                        if(Rx_Buffer[4+Package_Start]==0x01) //表示关闭当前喷出模式 x
                        {
                            Command_Timing_TriggerMode=0xff;
                            printf("Case 08 Timing Trigger Mode\r\n");

                        }
                        else
                        {
                            Command_Timing_TriggerMode=0x00;
                            printf("Case 08 Command Trigger Mode\r\n");

                        }

                        for(i=0; i<Package_Size; i++)
                        {
                            Tx_Buffer[i]=Rx_Buffer[Package_Start+i];
                        }
                        printf("Set Rxbuffer okay\r\n");
                        //memcpy(Tx_Buffer, Rx_Buffer, size);
                        Write_SOCK_Data_Buffer(s, Tx_Buffer,Package_Size);


                        break;


                    case 0x09:  //Puff on or Puff off
                        //use the puff mode or not
                        //we can use a new parameter in the pid function to enable puff or unpuff
                        if(Rx_Buffer[4+Package_Start]==0x01) //表示关闭当前喷出模式 x
                        {
                            Normal_Puff_RunningMode=0xff;
                            printf("Case 09 puff mode on\r\n");

                        }
                        else
                        {
                            Normal_Puff_RunningMode=0x00;
                            printf("Case 09 puff mode off\r\n");

                        }

                        for(i=0; i<Package_Size; i++)
                        {
                            Tx_Buffer[i]=Rx_Buffer[Package_Start+i];
                        }
                        printf("Set Rxbuffer okay\r\n");
                        //memcpy(Tx_Buffer, Rx_Buffer, size);
                        Write_SOCK_Data_Buffer(s, Tx_Buffer,Package_Size);

                        break;



                    case 0x19:  //use screen or not use screen


                        if(Rx_Buffer[4+Package_Start]==0x01)

                        {
                            LCD_Display_offon=0xff;

                            printf("Case 19 to use screen\n");
                        }
                        else
                        {
                            LCD_Display_offon=0x00;
                            printf("Case 19 to not use sreen\n");


                        }
                        for(i=0; i<Package_Size; i++)
                        {
                            Tx_Buffer[i]=Rx_Buffer[Package_Start+i];
                            printf("Txbuffer content:%x\r\n",Tx_Buffer[i]);
                            printf("I number:%d\r\n",i);
                        }
                        printf("Set Rxbuffer okay\r\n");
                        //memcpy(Tx_Buffer, Rx_Buffer, size);
                        Write_SOCK_Data_Buffer(s, Tx_Buffer,Package_Size);

                        break;


                    default:
                        break;

                }
                break;





            case 0x06:

                switch(Rx_Buffer[2+Package_Start])
                {
                    case 0x0A: //Vacuum value: Pressure value set 627D pressure value

                        //if the value is received, we feedback the same package that we get




                        //convert to float type
                        testdata.byteData[3]=Rx_Buffer[3+1+Package_Start];
                        testdata.byteData[2]=Rx_Buffer[4+1+Package_Start];
                        testdata.byteData[1]=Rx_Buffer[5+1+Package_Start];
                        testdata.byteData[0]=Rx_Buffer[6+1+Package_Start];

                        //Vacuum Value should be changed into Voltage value accoroding 627D manully

                        //we should set a critical value to limit the data
                        //if it doesn't meet our requirements,we need to send the data error to the pc
                        if((testdata.floatData>Cavity_627D_Pressure_SetMax)|(testdata.floatData<Cavity_627D_Pressure_SetMin))
                        {
                            //return the data over information
                            Error_OverSet=0x01;
                            LED10=0; //overset float value error signal


                        }
                        else //the set is okay
                        {
                            Error_OverSet=0x00;

                            Package_Cavity_627D_Pressure_Set=testdata.floatData;
                            printf("Case 0x0A: set 627D pressure\r\n");




                        }
                        for(i=0; i<Package_Size; i++)
                        {
                            Tx_Buffer[i]=Rx_Buffer[Package_Start+i];
                        }
                        printf("Set Rxbuffer okay\r\n");
                        //memcpy(Tx_Buffer, Rx_Buffer, size);
                        Write_SOCK_Data_Buffer(s, Tx_Buffer,Package_Size);
                        break;
                    case 0x0B:  //1479A Gas  Flow value set



                        //if the value is received, we feedback the same package that we get

                        //convert to float type
                        testdata.byteData[3]=Rx_Buffer[3+1+Package_Start];
                        testdata.byteData[2]=Rx_Buffer[4+1+Package_Start];
                        testdata.byteData[1]=Rx_Buffer[5+1+Package_Start];
                        testdata.byteData[0]=Rx_Buffer[6+1+Package_Start];


                        //we should set a critical value to limit the data锛?
                        //if it doesn't meet our requirements,we need to send the data error to the pc
                        if((testdata.floatData>Flow_1479A_SetMax)|(testdata.floatData<Flow_1479A_SetMin))
                        {
                            //return the data over information
                            Error_OverSet=0x01;
                            LED10=0; //overset float value error signal


                        }
                        else //the set is okay
                        {

                            Error_OverSet=0x00;



                            //1479A Flow Set value

                            Package_Flow_1479A_Set=testdata.floatData;
                            printf("Case 0x0B Set 1479A Flow Data\r\n");


                        }
                        for(i=0; i<Package_Size; i++)
                        {
                            Tx_Buffer[i]=Rx_Buffer[Package_Start+i];
                        }
                        printf("Set Rxbuffer okay\r\n");
                        //memcpy(Tx_Buffer, Rx_Buffer, size);
                        Write_SOCK_Data_Buffer(s, Tx_Buffer,Package_Size);


                        break;
                    case 0x0C://Gas Puff mode: Puff Pressure value set

                        //if the value is received, we feedback the same package that we get

                        //convert to float type
                        testdata.byteData[3]=Rx_Buffer[3+1+Package_Start];
                        testdata.byteData[2]=Rx_Buffer[4+1+Package_Start];
                        testdata.byteData[1]=Rx_Buffer[5+1+Package_Start];
                        testdata.byteData[0]=Rx_Buffer[6+1+Package_Start];

                        //we should set a critical value to limit the data
                        //if it doesn't meet our requirements,we need to send the data error to the pc

                        if((testdata.floatData>Flow_1479A_Puff_SetMax)|(testdata.floatData<Flow_1479A_Puff_SetMin))
                        {
                            //return the data over information
                            Error_OverSet=0x01;
                            LED10=0; //overset float value error signal


                        }
                        else //the set is okay
                        {
                            Error_OverSet=0x00;

                            //Vacuum Value should be changed into Voltage value accoroding 627D manully

                            Package_Cavity_627D_Puff_Set=testdata.floatData;

                            printf("Case 0x0c: set puff pressure set\r\n");


                        }
                        for(i=0; i<Package_Size; i++)
                        {
                            Tx_Buffer[i]=Rx_Buffer[Package_Start+i];
                        }
                        printf("Set Rxbuffer okay\r\n");
                        //memcpy(Tx_Buffer, Rx_Buffer, size);
                        Write_SOCK_Data_Buffer(s, Tx_Buffer,Package_Size);
                        break;
                    case 0x0d://pid code setting , set pid together, you cannot set it alone

                        testdata.byteData[3]=Rx_Buffer[3+1+Package_Start];
                        testdata.byteData[2]=Rx_Buffer[4+1+Package_Start];
                        testdata.byteData[1]=Rx_Buffer[5+1+Package_Start];
                        testdata.byteData[0]=Rx_Buffer[6+1+Package_Start];
                        //we should set a critical value to limit the data
                        //if it doesn't meet our requirements,we need to send the data error to the pc

                        if((testdata.floatData>PID_P_SetMax)|(testdata.floatData<PID_P_SetMin))
                        {
                            //return the data over information

                            // LED11=1; //clear crc check running  signal
                            LED10=0; //overset float value error signal


                        }
                        else //the set is okay
                        {

                            Package_Duty_P=testdata.floatData;

                            printf("Case 0x0d Set P parameter ok\r\n");

                        }
                        for(i=0; i<Package_Size; i++)
                        {
                            Tx_Buffer[i]=Rx_Buffer[Package_Start+i];
                        }
                        printf("Set Rxbuffer okay\r\n");
                        //memcpy(Tx_Buffer, Rx_Buffer, size);
                        Write_SOCK_Data_Buffer(s, Tx_Buffer,Package_Size);

                        break;
                    case 0x14://pid code setting , set pid together, you cannot set it alone

                        testdata.byteData[3]=Rx_Buffer[3+1+Package_Start];
                        testdata.byteData[2]=Rx_Buffer[4+1+Package_Start];
                        testdata.byteData[1]=Rx_Buffer[5+1+Package_Start];
                        testdata.byteData[0]=Rx_Buffer[6+1+Package_Start];
                        //we should set a critical value to limit the data
                        //if it doesn't meet our requirements,we need to send the data error to the pc

                        if((testdata.floatData>PID_I_SetMax)|(testdata.floatData<PID_I_SetMin))
                        {
                            //return the data over information
                            LED10=0; //overset float value error signal


                        }
                        else //the set is okay
                        {

                            Package_Duty_I=testdata.floatData;

                            printf("Case 0x14 Set I parameter ok\r\n");

                        }
                        for(i=0; i<Package_Size; i++)
                        {
                            Tx_Buffer[i]=Rx_Buffer[Package_Start+i];
                        }
                        printf("Set Rxbuffer okay\r\n");
                        //memcpy(Tx_Buffer, Rx_Buffer, size);
                        Write_SOCK_Data_Buffer(s, Tx_Buffer,Package_Size);

                        break;
                    case 0x15://pid code setting , set pid together, you cannot set it alone

                        testdata.byteData[3]=Rx_Buffer[3+1+Package_Start];
                        testdata.byteData[2]=Rx_Buffer[4+1+Package_Start];
                        testdata.byteData[1]=Rx_Buffer[5+1+Package_Start];
                        testdata.byteData[0]=Rx_Buffer[6+1+Package_Start];
                        //we should set a critical value to limit the data
                        //if it doesn't meet our requirements,we need to send the data error to the pc

                        if((testdata.floatData>PID_D_SetMax)|(testdata.floatData<PID_D_SetMin))
                        {
                            //return the data over information
                            LED10=0; //overset float value error signal


                        }
                        else //the set is okay
                        {

                            Package_Duty_D=testdata.floatData;

                            printf("Case 0x15 Set D parameter ok\r\n");

                        }
                        for(i=0; i<Package_Size; i++)
                        {
                            Tx_Buffer[i]=Rx_Buffer[Package_Start+i];
                        }
                        printf("Set Rxbuffer okay\r\n");
                        //memcpy(Tx_Buffer, Rx_Buffer, size);
                        Write_SOCK_Data_Buffer(s, Tx_Buffer,Package_Size);

                        break;

                    case 0x0E: // Set the value status
                        Package_Valve_Status_Set[0]=Rx_Buffer[3+1+Package_Start];
                        Package_Valve_Status_Set[1]=Rx_Buffer[4+1+Package_Start];
                        printf("Received Value high: %x\r\n",Package_Valve_Status_Set[0]);
                        printf("Received Value: low : %x\r\n",Package_Valve_Status_Set[1]);

                        printf("Case 0x0E: Stable running valves\r\n");
                        for(i=0; i<Package_Size; i++)
                        {
                            Tx_Buffer[i]=Rx_Buffer[Package_Start+i];
                        }
                        printf("Set Rxbuffer okay\r\n");
                        //memcpy(Tx_Buffer, Rx_Buffer, size);
                        Write_SOCK_Data_Buffer(s, Tx_Buffer,Package_Size);
                        break;

                    case 0x0F: //puff mode auxiliary valve

                        Package_Valve_Puff_Status_Set[0]=Rx_Buffer[3+1+Package_Start];
                        Package_Valve_Puff_Status_Set[1]=Rx_Buffer[4+1+Package_Start];

                        printf("Case 0x0f:Set Puff Mode auxiliary Valve\r\n");
                        for(i=0; i<Package_Size; i++)
                        {
                            Tx_Buffer[i]=Rx_Buffer[Package_Start+i];
                        }
                        printf("Set Rxbuffer okay\r\n");
                        //memcpy(Tx_Buffer, Rx_Buffer, size);
                        Write_SOCK_Data_Buffer(s, Tx_Buffer,Package_Size);

                        break;
                    case 0x13:  //clear error register

                        Error_Communicate=0x00;
                        Error_OverSet=0x00;

                        LED11=1; //clear crc check running  signal
                        LED10=1; //overset float value error signal

                        for(i=0; i<Package_Size; i++)
                        {
                            Tx_Buffer[i]=Rx_Buffer[Package_Start+i];
                        }
                        printf("Set Rxbuffer okay\r\n");
                        //memcpy(Tx_Buffer, Rx_Buffer, size);
                        Write_SOCK_Data_Buffer(s, Tx_Buffer,Package_Size);


                        break;
                    case 0x19: //AD sampling times setup

                        //if the value is received, we feedback the same package that we get

                        //convert to float type
                        testdata.byteData[3]=Rx_Buffer[3+1+Package_Start];
                        testdata.byteData[2]=Rx_Buffer[4+1+Package_Start];
                        testdata.byteData[1]=Rx_Buffer[5+1+Package_Start];
                        testdata.byteData[0]=Rx_Buffer[6+1+Package_Start];


                        //we should set a critical value to limit the data锛?
                        //if it doesn't meet our requirements,we need to send the data error to the pc
                        if((testdata.floatData>100)|(testdata.floatData<1))
                        {
                            //return the data over information
                            Error_OverSet=0x01;
                            LED10=0; //overset float value error signal


                        }
                        else //the set is okay
                        {

                            Error_OverSet=0x00;



                            //1479A Flow Set value

                            Package_AD_SamplingTimes=testdata.floatData;
                            printf("Case 0x19 Set AD sampling times\r\n");


                        }
                        for(i=0; i<Package_Size; i++)
                        {
                            Tx_Buffer[i]=Rx_Buffer[Package_Start+i];
                        }
                        printf("Set Rxbuffer okay\r\n");
                        //memcpy(Tx_Buffer, Rx_Buffer, size);
                        Write_SOCK_Data_Buffer(s, Tx_Buffer,Package_Size);


                        break;

                }

                break;




            case 0x08:

                switch(Rx_Buffer[2+Package_Start])
                {
                    case 0x11://Read communicate error
                        //ps: if we have this problem, how can we solve this problem

                        printf("Case 0x11: read communicate error\r\n");

                        Tx_Buffer[0]=0x05; // Slave address
                        Tx_Buffer[1]=0x08;// function  code
                        Tx_Buffer[2]=0x11;// register address
                        Tx_Buffer[3]=0x01;//length of data

                        if(Error_Communicate==0x01)
                        {
                            Tx_Buffer[4]=0x01;//there is an error
                        }
                        else
                        {
                            Tx_Buffer[4]=0x00;//none error
                        }


                        crctestdata.CrcData=GetCRC16(Tx_Buffer,5);


                        Tx_Buffer[5]=crctestdata.byteData[0];
                        Tx_Buffer[6]=crctestdata.byteData[1];

                        //  Tx_Buffer[0]=0xff;

                        Write_SOCK_Data_Buffer(s, Tx_Buffer, 7);

                        break;


                    case 0x12://set value error over


                        printf("Case 0x12: set value over error\r\n");

                        Tx_Buffer[0]=0x05; // Slave address
                        Tx_Buffer[1]=0x08;// function  code
                        Tx_Buffer[2]=0x12;// register address
                        Tx_Buffer[3]=0x01;//length of data

                        if(Error_OverSet==1)
                        {
                            Tx_Buffer[4]=0x01;//there is an error
                        }
                        else
                        {
                            Tx_Buffer[4]=0x00;//none error
                        }


                        crctestdata.CrcData=GetCRC16(Tx_Buffer,5);


                        Tx_Buffer[5]=crctestdata.byteData[0];
                        Tx_Buffer[6]=crctestdata.byteData[1];

                        //  Tx_Buffer[0]=0xff;

                        Write_SOCK_Data_Buffer(s, Tx_Buffer, 7);

                        break;
                    default:
                        break;

                }
                break;




            default:
                break;
        }



    }
    else
    {
        //receive the wrong package, we set the crcheck wrong message





    }




    /*if (Rx_Buffer[0]==0x11)   //判断寄存器中的内容就通过这个来进行。
    {
        Write_SOCK_Data_Buffer(s, Tx_Buffer, size);
        Write_SOCK_Data_Buffer(0, "\r\n THis is 1\r\n", 23);

    }*/

//




}

/*******************************************************************************
* Function name  :  Cavity_Pressure_SendBack
* Description  : this function  is to mainly send the package to the pc
* Input : None
* Output  :  None
* Return Value :  None
* Attention: send the pack to the pc
*******************************************************************************/

void Cavity_Pressure_SendBack()
{
    ////////////////CRC_Mid/////////////////////////
    unsigned int CRC_Mid;
    FLOAT_BYTE testdata;
    float temptofun;
    float *AD_Voltage_Status;




    u8 s=0;
    /*Read the actual flow value , and send it back to the pc*/
    Tx_Buffer[0]=0x05; // 本机地址
    Tx_Buffer[1]=0x03;//功能命令码
    Tx_Buffer[2]=0x01;//寄存器地

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


/*******************************************************************************
* Function name  :  Status_Register_Update_DMA
* Description  : Status_Register_Update_DMA,in order to do pwm adjustment and the 1479a flow adjustment
* Input : None
* Output  :  None
* Return Value :  None
* Attention: we process the data we get from the pc
*******************************************************************************/

void Status_Register_Update_DMA()
{
    float AD_Voltage_Status[3];
    char *ValveValue_Status;
    float ADC_Value[3];//鐢ㄦ潵淇濆瓨缁忚繃杞崲寰楀埌鐨勭數鍘嬪�?
    int sum;
    u8 i,j;

    /*
        ValveValue_Status=Gas_State_Read(); //函数实现读取IO口的高低电平值

        AD_Voltage_Status[0]=AD_Conversion_1479A(Package_AD_SamplingTimes);

        AD_Voltage_Status[1]=AD_Conversion_627D(Package_AD_SamplingTimes);
        AD_Voltage_Status[2]=AD_Conversion_025D(Package_AD_SamplingTimes);


    Flow_1479A_Status=ADVoltage_2_Flow1479A(AD_Voltage_Status[0]);
    Cavity_627D_Pressure_Status=ADVoltage_2_Pressure627D(AD_Voltage_Status[1]);
    Cavity_025D_Pressure_Status=ADVoltage_2_Pressure025D(AD_Voltage_Status[2]);

    */

    for(i=0; i<3; i++)
    {
        sum=0;

        for(j=0; j<10; j++)
        {
            sum +=ADC_ConvertedValue[j][i];
        }
        AD_Voltage_Status[i]=(float)sum/(10*4096)*3.3;//姹傚钩鍧囧�煎苟杞崲鎴愮數鍘嬪�?


    }


    //intf("The current AD value =%f\r\n",AD_Voltage_Status[0]);
    //intf("The current AD value =%f\r\n",AD_Voltage_Status[1]);
    //intf("The current AD value =%f\r\n",AD_Voltage_Status[2]);

    Flow_1479A_Status=ADVoltage_2_Flow1479A(AD_Voltage_Status[0]);
    Cavity_627D_Pressure_Status=ADVoltage_2_Pressure627D(AD_Voltage_Status[1]);
    Cavity_025D_Pressure_Status=ADVoltage_2_Pressure025D(AD_Voltage_Status[2]);


}

/*******************************************************************************
* Function name  :  Status_Register_Update
* Description  : Status_Register_Update,in order to do pwm adjustment and the 1479a flow adjustment
* Input : None
* Output  :  None
* Return Value :  None
* Attention: we process the data we get from the pc
*******************************************************************************/

void Status_Register_Update()
{
    float AD_Voltage_Status[3];
    char *ValveValue_Status;
    float ADC_Value[3];//鐢ㄦ潵淇濆瓨缁忚繃杞崲寰楀埌鐨勭數鍘嬪�?
    int sum;

    ValveValue_Status=Gas_State_Read(); //函数实现读取IO口的高低电平值

    AD_Voltage_Status[0]=AD_Conversion_1479A(Package_AD_SamplingTimes);

    AD_Voltage_Status[1]=AD_Conversion_627D(Package_AD_SamplingTimes);
    AD_Voltage_Status[2]=AD_Conversion_025D(Package_AD_SamplingTimes);


    Flow_1479A_Status=ADVoltage_2_Flow1479A(AD_Voltage_Status[0]);
    Cavity_627D_Pressure_Status=ADVoltage_2_Pressure627D(AD_Voltage_Status[1]);
    Cavity_025D_Pressure_Status=ADVoltage_2_Pressure025D(AD_Voltage_Status[2]);


}










/*******************************************************************************
* Function name  :  Status_LCD_Update
* Description  : to update the lcd display
* Input : None
* Output  :  None
* Return Value :  None
* Attention: it costs a lot of time, which we cannot use this function every time
*******************************************************************************/

void Status_LCD_Update()
{
    float AD_Voltage_Status[3];

    char *ValveValue_Status;
    char *ValveValue_Status_LCD;
    float Set_Voltage[3];
    /* In this part we need to update the devices to LCD or the internet  */  //use timer to send my status to the pc
    //LCD to update
    ValveValue_Status=Gas_State_Read(); //函数实现读取IO口的高低电平值
    ValveValue_Status_LCD = Gas_State_Read_LCD();
    Gas_StateLayerUpdate(ValveValue_Status_LCD);
    //AD_Voltage_Status=AD_Conversion();

    AD_Voltage_Status[0]=AD_Conversion_1479A(Package_AD_SamplingTimes);

    AD_Voltage_Status[1]=AD_Conversion_627D(Package_AD_SamplingTimes);
    AD_Voltage_Status[2]=AD_Conversion_025D(Package_AD_SamplingTimes);




    //this sentence destroy the value
    /* printf("AD_Voltage_Status[0]:%f\r\n",AD_Voltage_Status[0]);
     printf("AD_Voltage_Status[1]:%f\r\n",AD_Voltage_Status[1]);
     printf("AD_Voltage_Status[2]:%f\r\n",AD_Voltage_Status[2]);
     //
    */
    Set_Voltage[0]=Flow_1479A_Set;
    Set_Voltage[1]=Cavity_627D_Pressure_Set;
    Set_Voltage[2]=12;


    Flow_1479A_Status=ADVoltage_2_Flow1479A(AD_Voltage_Status[0]);
    Cavity_627D_Pressure_Status=ADVoltage_2_Pressure627D(AD_Voltage_Status[1]);
    Cavity_025D_Pressure_Status=ADVoltage_2_Pressure025D(AD_Voltage_Status[2]);

    ADC_LCD_Out(AD_Voltage_Status,Set_Voltage);
    /*  printf("Flow_1479A_Status:%f\r\n",Flow_1479A_Status);
      printf("Cavity_627D_Pressure_Status:%f\r\n",Cavity_627D_Pressure_Status);
      printf("Cavity_025D_Pressure_Status:%f\r\n",Cavity_025D_Pressure_Status);*/
    //Alter current status ,

    //printf("Update the voltage status in the funtion\r\n");

}

