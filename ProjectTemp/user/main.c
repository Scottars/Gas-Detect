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

    ///////////////////////液晶屏初始化过程///////////////////////

    SystemInit();//初始化RCC 设置系统主频为72MHZ
    delay_init(72);      //延时初始化
    LCD_Init();    //液晶屏初始化


    /////////////////////LED light initial////////////////////////////

    LED_Init(); //Initial

    ///////////////ADC Initial///////////////////ADC模块使用了三个片上AD PC0 PC1 PC2  分别是 AD10 11 12 通道
    adc_init();  //ADC初始化

    /////////////////Serial port initial//////////////用来进行串口通讯的，printf 实现了重定向
    printf_init(); //printf初始化

    ///////////////DAC Initial /////////////////DAC 部分的初始化，我们采用的是DAC1  也就是PA4 口的输出
    Dac1_Init();                //DAC初始化
    DAC_SetChannel1Data(DAC_Align_12b_R, 0);//初始值为0


    ///////////////Valve Status Initial/////////////

    ValveState_Init(); //供气阀门GPIO口的部分初始化



    ////////////////LCD Intial ///////////////////
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

    System_Initialization();    //STM32系统初始化函数(初始化STM32时钟及外设)
    Load_Net_Parameters();      //装载网络参数
    W5500_Hardware_Reset();     //硬件复位W5500
    W5500_Initialization();     //W5500初始货配置
    W5500_Socket_Set();//W5500端口初始化配置





    while(1)
    {




        /****************************************网络处理通讯处理********************************************/
        if(W5500_Interrupt)//处理W5500中断
        {
            //LED0=0;                                                   //LED0      指示的是网络部分的信号的传输
            W5500_Interrupt_Process();//W5500中断处理程序框架
        }
        if((S0_Data & S_RECEIVE) == S_RECEIVE)//如果Socket0接收到数据
        {
            S0_Data&=~S_RECEIVE;
            Process_Socket_Data(0);//W5500接收并发送接收到的数据
            //对于传下来的整个的接收的数据包，我们有两种方法进行考虑，考虑1：将寄存器中的数据分出来，然后在主函数中进行各种的调用情况
            //考虑二：直接在数据处理部分就直接调用我们的进行实际的操作部分。
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
        ValveValue_Status=Gas_State_Read(); //函数实现读取IO口的高低电平值
        ValveValue_Status_LCD = Gas_State_Read_LCD();
        Gas_StateLayerUpdate(ValveValue_Status_LCD);
        AD_Voltage_Status=AD_Conversion();
        ADC_LCD_Out(AD_Voltage_Status);


    }
}

/*******************************************************************************
* 函数名  : Process_Socket_Data
* 描述    : W5500接收并发送接收到的数据
* 输入    : s:端口号
* 输出    : 无
* 返回值  : 无
* 说明    : 本过程先调用S_rx_process()从W5500的端口接收数据缓冲区读取数据,
*           然后将读取的数据从Rx_Buffer拷贝到Temp_Buffer缓冲区进行处理。
*           处理完毕，将数据从Temp_Buffer拷贝到Tx_Buffer缓冲区。调用S_tx_process()
*           发送数据。
*******************************************************************************/

void Process_Socket_Data(SOCKET s)
{

    FLOAT_BYTE testdata;
    char floattest[10];
    char test1[4];
    unsigned short size;//接收到的buffer 的大小
    ///////////////////////Valve Gas Part///////

    u8 ValveValue_Set[12];  //来自网络传入过来的信号
    char *ValveValue_Status;
    u8 i;
    ////////////////////////////////////////////////////////
    ////////////////真空腔部分/////////////////////////////
    float VacuumValue_Set;


    //////////////////流量计的流量值设定设定//////////////////////////////////
    float Flow1479AValue_Set;



    ////////////////气体喷出峰值真空度设定//////////////////////////////////
    float GasPuffValue_Set;


    ////////////////PID参数设定/////////////////////////
    float Kp,Ki,Kd;


    ////////////////CRC_Mid/////////////////////////
    unsigned int CRC_Mid;







    ///////////////////AD转换部分////////////////
    float *AD_Voltage_Status;
    float temp=0; // 用来进行AD转换使用的变量
    uint8_t temp1[5];
    char AD_Value[50];






    size=Read_SOCK_Data_Buffer(s, Rx_Buffer);
    //printf("\r\nSIZE:%d\r\n",size);
    memcpy(Tx_Buffer, Rx_Buffer, size);
//  printf("\r\nRX_BUFFER\r\n");
    printf(Rx_Buffer);

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

    // 在解析包之前就进行，crc check





    if (Rx_Buffer[0]==0x05) //本机的设备地址为0x05
    {
        //  printf("\r\nSLocal Address ok!\r\n");

        switch (Rx_Buffer[1])
        {
            case 0x03:    //读取功能码寄存器的状态 --是否为读数据功能码
                switch  (Rx_Buffer[2])
                {

                    case 0x01:
                        Tx_Buffer[0]=0x05; // 本机地址
                        Tx_Buffer[1]=0x03;//功能命令码
                        Tx_Buffer[2]=0x01;//寄存器地?
                        //读取气体流量的当前值，从新计算一次传入上去，还是直接上传当前值
                        AD_Voltage_Status = AD_Conversion();
                        //AD_Voltage_Status[0 1 2]   分别表示1479A 627D 025d

                        //对得到的电压进行流量值进行转换,
                        temp = GasFloatValue_1479ACalc(AD_Voltage_Status[0]);
                        temp=3;
                        //转换得到IEEE 754 标准
                        testdata.floatData=temp;

                        Tx_Buffer[3]=testdata.byteData[3];
                        Tx_Buffer[4]=testdata.byteData[2];
                        Tx_Buffer[5]=testdata.byteData[1];
                        Tx_Buffer[6]=testdata.byteData[0];

                        //GetCRC16 得到上面得数据的CRC校验码
                        CRC_Mid=GetCRC16(Tx_Buffer,7);

                        Tx_Buffer[7]=(u8)CRC_Mid;
                        Tx_Buffer[8]=(u8)(CRC_Mid>>8);

                        Write_SOCK_Data_Buffer(s, Tx_Buffer, 9);
                        break;


                    case 0x02: //627D真空度检测，这个如果上位机发了命令，我还是要传一份数据回去吗？
                        Tx_Buffer[0]=0x05; // 本机地址
                        Tx_Buffer[1]=0x03;//功能命令码
                        Tx_Buffer[2]=0x02;//寄存器地
                        //进行真空度AD转换
                        AD_Voltage_Status = AD_Conversion();

                        //进行AD采集到的数值进行处理得到真空度的标准数值
                        //AD_Voltage_Status[0 1 2]   分别表示1479A 627D 025d
                        temp = VacuumFloatValue_627DCalc(AD_Voltage_Status[1]);



                        //数值转换成浮点数
                        testdata.floatData=temp;

                        Tx_Buffer[3]=testdata.byteData[3];
                        Tx_Buffer[4]=testdata.byteData[2];
                        Tx_Buffer[5]=testdata.byteData[1];
                        Tx_Buffer[6]=testdata.byteData[0];



                        //GetCRC16 得到上面得数据的CRC校验码
                        CRC_Mid=GetCRC16(Tx_Buffer,7);

                        Tx_Buffer[7]=(u8)CRC_Mid;
                        Tx_Buffer[8]=(u8)(CRC_Mid>>8);

                        Write_SOCK_Data_Buffer(s, Tx_Buffer, 9);


                        break;

                    case 0x03: //025D真空度检测，这个如果上位机发了命令，我还是要传一份数据回去吗？
                        Tx_Buffer[0]=0x05; // 本机地址
                        Tx_Buffer[1]=0x03;//功能命令码
                        Tx_Buffer[2]=0x03;//寄存器地?
                        //进行真空度AD转换
                        AD_Voltage_Status = AD_Conversion();

                        //进行AD采集到的数值进行处理得到真空度的标准数值
                        //AD_Voltage_Status[0 1 2]   分别表示1479A 627D 025d
                        temp = VacuumFloatValue_025DCalc(AD_Voltage_Status[2]);

                        temp=3;
                        //数值转换成浮点数
                        testdata.floatData=temp;

                        Tx_Buffer[3]=testdata.byteData[3];
                        Tx_Buffer[4]=testdata.byteData[2];
                        Tx_Buffer[5]=testdata.byteData[1];
                        Tx_Buffer[6]=testdata.byteData[0];



                        //GetCRC16 得到上面得数据的CRC校验码
                        CRC_Mid=GetCRC16(Tx_Buffer,7);

                        Tx_Buffer[7]=(u8)CRC_Mid;
                        Tx_Buffer[8]=(u8)(CRC_Mid>>8);

                        Write_SOCK_Data_Buffer(s, Tx_Buffer, 9);


                        break;

                    case 0x04://配气柜阀门状态读取
                        Tx_Buffer[0]=0x05; // 本机地址
                        Tx_Buffer[1]=0x03;//功能命令码
                        Tx_Buffer[2]=0x04;//寄存器地址
                        //读取配气柜阀门状态函数
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










                    case 0x06:  //气体喷出模式的启停  当气体脉冲峰值设定好了之后，以及触发方式设定好了之后，通过这个按键来进行启动停止
                        //use the puff mode or not
                        //we can use a new parameter in the pid function to enable puff or unpuff
                        if(Rx_Buffer[3]==0xff) //表示关闭当前喷出模式 x
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
                    case 0x07: //真空腔真空度设置




                        //这个时候，对上位机传过来的buffer 3 4 5 6 中的数据进行处理，处理的前提是进行crc 校验
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
                    case 0x09://1479A流量其流量目标设定
                        //这个时候，对上位机传过来的buffer 3 4 5 6 中的数据进行处理，处理的前提是进行crc 校验 通过
                        //ps:说明：上位机的浮点数的传送的顺序，是高位在前低位在后，而在bytes中，也是低位在数组的低位
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
                    case 0xA: //气体脉冲峰值设定

                        //这个时候，对上位机传过来的buffer 3 4 5 6 中的数据进行处理，处理的前提是进行crc 校验 通过




                        //GasPuffValue_Set=



                        break;
                    case 0xB://PID参数的设定
                        //这个时候，对上位机传过来的buffer 3 4 5 6    7 8 9 10  11 12 13 14 中的数据进行处理，处理的前提是进行crc 校验 通过




                        //=

                        break;
                    case 0x0C: //puff trigger mode --0xff timing mode   0x00 command mode
                        if(Rx_Buffer[3]=0xff) //选择采用上位机触发方式  在这个模式下，设定新的数值给我们的设定的真空度的值
                        {

                        }
                        if(Rx_Buffer[3]=0x02) //选择信号触发方式，在这个模式下，设定的新的数值在时钟信号来了的时候，我们将这个新的真空度的数值写下去。
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

        /*if (Rx_Buffer[0]==0x01)   //判断寄存器中的内容就通过这个来进行。
            {
                Write_SOCK_Data_Buffer(s, Tx_Buffer, size);
                Write_SOCK_Data_Buffer(0, "\r\n THis is 1\r\n", 23);

            }*/

    }
    else
    {
        //如果不是本机的地址的情况 如何进行处理



    }


    /*if (Rx_Buffer[0]==0x11)   //判断寄存器中的内容就通过这个来进行。
    {
        Write_SOCK_Data_Buffer(s, Tx_Buffer, size);
        Write_SOCK_Data_Buffer(0, "\r\n THis is 1\r\n", 23);

    }*/

    //

//下面写入关于框架的内容，将我们所得到的整个数据进行分包处理，



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
    Tx_Buffer[0]=0x05; // 本机地址
    Tx_Buffer[1]=0x03;//功能命令码
    Tx_Buffer[2]=0x01;//寄存器地

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



