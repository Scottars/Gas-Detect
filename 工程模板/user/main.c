/*******************************************************************************
*                 
*                 		       ���пƼ�
--------------------------------------------------------------------------------
* ʵ �� ��		 : ADCת��ʵ��
* ʵ��˵��       : ͨ��printf��ӡAD������� ������ADģ���Աߵĵ�λ�� �ڴ��������ϼ��������ѹ��
					���ļ��ڽ�ͼ
* ���ӷ�ʽ       : 
* ע    ��		 : 	���ú�����ͷ�ļ�.c�ļ���
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



int main()
{	
		u8 i,j,k;		
	////////////////////AD part/////////////////////////
	u16 dacval;
	
	float voltage[3];
	u16 AD_Channel_Select[3]={ADC_Channel_10,ADC_Channel_11,ADC_Channel_12};//����ͨ��ѡ������
	u32 AD_Channel_10_Value,AD_Channel_11_Value,AD_Channel_12_Value;
	

	

	///////////////////////Һ������ʼ������///////////////////////
	
	SystemInit();//��ʼ��RCC ����ϵͳ��ƵΪ72MHZ
	delay_init(72);	     //��ʱ��ʼ��
	LCD_Init();	   //Һ������ʼ��
	
	
	//////////////////////////////////////////////////////////////
		
	LED_Init(); //��ʼ����ʾ�ƵĲ���  �����ڵ��Ե�ʱ��ʹ�õ���
	
	
	
	////////////////////////////////////////////
	//ADCģ��ʹ��������Ƭ��AD PC0 PC1 PC2  �ֱ��� AD10 11 12 ͨ��
	adc_init();	 //ADC��ʼ��
	
	/////////////////////////////
	//�������д���ͨѶ�ģ�printf ʵ�����ض���
	printf_init(); //printf��ʼ��
	
	/////////////////////////////////////////
	//DAC ���ֵĳ�ʼ�������ǲ��õ���DAC1  Ҳ����PA4 �ڵ����
	Dac1_Init();				//DAC��ʼ��
	DAC_SetChannel1Data(DAC_Align_12b_R, 0);//��ʼֵΪ0	 


////////////////////
//���ת�����ֵ����ŵĵĳ�ʼ��
	ValveState_Init(); //��������GPIO�ڵĲ��ֳ�ʼ��
	
	
	
//�����ʼ��//

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

		System_Initialization();	//STM32ϵͳ��ʼ������(��ʼ��STM32ʱ�Ӽ�����)
		Load_Net_Parameters();		//װ���������	
		W5500_Hardware_Reset();		//Ӳ����λW5500
		W5500_Initialization();		//W5500��ʼ������
	
	
	
	

		
	while(1)
	{
		
	
	//	Gas_StateLayerUpdate(ValveStateValue);
		/*
		//////////////////////��Ļtests ����/////////////////////////////
		main_test(); 		//����������
		Test_Color();  		//��ˢ��������
		Test_FillRec();		//GUI���λ�ͼ����
		Test_Circle(); 		//GUI��Բ����
		Test_Triangle();    //GUI�����λ�ͼ����
		English_Font_test();//Ӣ������ʾ������
		Chinese_Font_test();//��������ʾ������
		Pic_test();			//ͼƬ��ʾʾ������
		Rotate_Test();   //��ת��ʾ����
		//����������������߲���Ҫ�������ܣ���ע�͵����津����������
		//Touch_Test();		//��������д����  
			
		//////////////////////////////////////////////////////
			
		*/
		
	
				
		//���紫��
		
		/****************************************���紦��ͨѶ����********************************************/
		
		W5500_Socket_Set();//W5500�˿ڳ�ʼ������

		if(W5500_Interrupt)//����W5500�ж�		
		{
			//LED0=0;   												//LED0      ָʾ�������粿�ֵ��źŵĴ���
			W5500_Interrupt_Process();//W5500�жϴ��������
		}
		if((S0_Data & S_RECEIVE) == S_RECEIVE)//���Socket0���յ�����
		{
			S0_Data&=~S_RECEIVE;
			Process_Socket_Data(0);//W5500���ղ����ͽ��յ�������
			//���ڴ������������Ľ��յ����ݰ������������ַ������п��ǣ�����1�����Ĵ����е����ݷֳ�����Ȼ�����������н��и��ֵĵ������
			//���Ƕ���ֱ�������ݴ����־�ֱ�ӵ������ǵĽ���ʵ�ʵĲ������֡�
		}
		
		
		
		

		
		/*******************************************************************************************************/
	
			
		
		//��ȡIO�ڵ����ݣ������������ǵ�״̬�ѱ䣬���߿��Բ�ȡ�жϵķ�ʽ
		//�򵥵ط�ʽ����ÿ���߹�����ط������е�IO������ȫ��ʹ��һ�£�����һ�ַ�������ͨ���жϣ�   �жϻ���Ӧ����ȡ�ⲿ�ж���  ��Ȼ����
		//
		
		

		
		
		
		
		
		
		//	j++;
		/*	if(j%20==0)
		{		led_display(); //LED2 
			
			
		}*/


	/*
		for (i=0;i<10;i=i+1)
		{
			led_display();
	
			//i=1500;
			dacval=i*200;
			Dac1_Set_Vol(dacval);//����DACֵ	
		
			printf("The target dac boltage is : %f\n",dacval*3.3/3300);	
	
			
			
			AD_Channel_10_Value = Get_ADC_Value(ADC_Channel_10,20);
			AD_Channel_11_Value = Get_ADC_Value(ADC_Channel_11,20);
			AD_Channel_12_Value = Get_ADC_Value(ADC_Channel_12,20);
			voltage[0] = (float)AD_Channel_10_Value*(3.3/4096);
			voltage[1] = (float)AD_Channel_11_Value*(3.3/4096);
			voltage[2] = (float)AD_Channel_12_Value*(3.3/4096);
			printf("Voltage0 is %f\n",voltage[0]);
			printf("Voltage1 is %f\n",voltage[1]);
			printf("Voltage2 is %f\n",voltage[2]);
			ADC_LCD_Out(voltage);
		}
	
		
		
		*/
		
		
		
					
	}			
}


