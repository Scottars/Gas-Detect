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

/****************************************************************************
* Function Name  : main
* Description    : Main program.
* Input          : None
* Output         : None
* Return         : None
****************************************************************************/
unsigned char ADdataA[7];
//unsigned char ADdataB[7];
//unsigned char A1;
//unsigned char A2;


int main()
{	
	
	
	u16 dacval;
	u16 value;
	float voltage1;
	u8 i,j;		
	///////////////////////Һ������ʼ������///////////////////////
	
	SystemInit();//��ʼ��RCC ����ϵͳ��ƵΪ72MHZ
	delay_init(72);	     //��ʱ��ʼ��
	LCD_Init();	   //Һ������ʼ��
	
	
	//////////////////////////////////////////////////////////////
		
	LED_Init();
	adc_init();	 //ADC��ʼ��
	printf_init(); //printf��ʼ��
	
	Dac1_Init();				//DAC��ʼ��
	
	DAC_SetChannel1Data(DAC_Align_12b_R, 0);//��ʼֵΪ0	    
	
	
	while(1)
	{
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
		GridLayer();
		Gas_State();
		
			j++;
			if(j%20==0)
		{		led_display();
			
			
		}


		for (i=0;i<10;i=i+1)
		{
			led_display();
	
			//i=1500;
			dacval=i*200;
			Dac1_Set_Vol(dacval);//����DACֵ	
		
			delay_ms(50);
			delay_ms(500);
			printf("The target dac boltage is : %f\n",dacval*3.3/3300);	
			value = Get_ADC_Value(ADC_Channel_10,20);
			printf("The target adc VALUE is : %d \n",value);
			voltage1 = (float)value*(3.3/4096);
			printf("Voltage is %f \n",voltage1);
			ADC_Out(voltage1);
		}
	
		
		
		
		
		
		
					
	}			
}


