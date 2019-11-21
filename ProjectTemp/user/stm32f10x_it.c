/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "systick.h"
#include "printf.h"
#include "smg.h"
#include "w5500.h"





extern u8 Normal_Puff_RunningMode;
/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

void EXTI2_IRQHandler()	   //�ⲿ�ж�2�жϺ���
{
	if(EXTI_GetITStatus(EXTI_Line2)==SET)
	{
   		EXTI_ClearITPendingBit(EXTI_Line0);//���EXTI��·����λ
		delay_ms(10);//��������
		if(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_2)==Bit_RESET)	   //k_left��������
		{
			if(GPIO_ReadOutputDataBit(GPIOC,GPIO_Pin_0)==Bit_RESET)
			{
				//LED Ϩ��
			   GPIO_SetBits(GPIOC,GPIO_Pin_0);	
			}
			else
			{
			   //LED ����
				GPIO_ResetBits(GPIOC,GPIO_Pin_0);
			}
		} 
		while(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_2)==0);
	}		
}


void EXTI1_IRQHandler()   //外部中断2中断函数
	
{
	printf("In the mid exti1"); //
		
			if(EXTI_GetITStatus(EXTI_Line1)==SET)
			{
				EXTI_ClearITPendingBit(EXTI_Line1);//清除EXTI线路挂起�?
				delay_ms(10);//消抖处理
				if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1)==Bit_RESET)	   
					{
					delay_ms(10);//消抖处理
					Normal_Puff_RunningMode=0xff; //to open the puff mode in the interru7pt mode 
					
					printf("we are in the middle of exti interrupt\r\n");
					
					
				} 

			}		
		

	}

void USART1_IRQHandler(void)	//����1�жϺ���
{
	static u8 k;
	USART_ClearFlag(USART1,USART_FLAG_TC);
	if(USART_GetITStatus(USART1,USART_IT_RXNE)!=Bit_RESET)//���ָ����USART�жϷ������
	{
		k=USART_ReceiveData(USART1);
	//	k++;	
		USART_SendData(USART1,k);//ͨ������USARTx���͵�������
		while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==Bit_RESET);	
	}
}
void USART2_IRQHandler(void)	//485ͨ���жϺ���
{
	static u8 k;
	USART_ClearFlag(USART2,USART_FLAG_TC);
	if(USART_GetITStatus(USART2,USART_IT_RXNE)!=RESET)//���ָ����USART�жϷ������	
	{
		k=USART_ReceiveData(USART2);
		GPIO_SetBits(GPIOG,GPIO_Pin_3);
		delay_ms(1);
		USART_SendData(USART2,k);
		while(USART_GetFlagStatus(USART2,USART_FLAG_TXE)==RESET);
		delay_ms(2);		
		GPIO_ResetBits(GPIOG,GPIO_Pin_3);
	}
}
/****************************************************************************
* Function Name  : TIM3_IRQHandler
* Description    : PWM timer ʹ��
* Input          : None
* Output         : None
* Return         : None
****************************************************************************/

void TIM3_IRQHandler()	  //��ʱ��3�жϺ���
{
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);
	
}

/****************************************************************************
* Function Name  : TIM4_IRQHandler
* Description    : PWM timer ʹ��
* Input          : None
* Output         : None
* Return         : None
****************************************************************************/
	
void TIM4_IRQHandler()	  //��ʱ��3�жϺ���
{
	if(TIM_GetITStatus(TIM4, TIM_IT_Update)!= RESET)
	{
		TIM_ClearITPendingBit(TIM4,TIM_IT_Update);
//		printf("IN\r\n");
		Hsdd_Data_Send(1);

		
		
	
	}
	
}


/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
