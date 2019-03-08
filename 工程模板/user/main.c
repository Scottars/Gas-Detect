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
	float voltage[3];
	u16 AD_Channel_Select[3]={ADC_Channel_10,ADC_Channel_11,ADC_Channel_12};//����ͨ��ѡ������
	u32 AD_Channel_10_Value,AD_Channel_11_Value,AD_Channel_12_Value;
	
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
	
	
	GridLayer();

	Gas_StateLayer();
		
	while(1)
	{
		
		//Gas_StateLayerUpdate();
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
		
		//�����ʼ��//

		
		//��ȡIO�ڵ����ݣ������������ǵ�״̬�ѱ䣬���߿��Բ�ȡ�жϵķ�ʽ
		//�򵥵ط�ʽ����ÿ���߹�����ط������е�IO������ȫ��ʹ��һ�£�����һ�ַ�������ͨ���жϣ�   �жϻ���Ӧ����ȡ�ⲿ�ж���  ��Ȼ����
		//
		//Gas_State_Read();
			
		
		
		//���紫��
		
		
		
		
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
			ADC_Out(voltage);
		}
	
		
		
		
		
		
		
					
	}			
}


