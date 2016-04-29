/**
  ******************************************************************************
  * @file    IO_Toggle/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    19-September-2011
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
#include "stm32f4xx_it.h"

/** @addtogroup STM32F4_Discovery_Peripheral_Examples
  * @{
  */

/** @addtogroup IO_Toggle
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

///**
//  * @brief  This function handles Hard Fault exception.
//  * @param  None
//  * @retval None
//  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
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
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/
void EXTI0_IRQHandler(void)	//- -必须加上0
{
//  if(EXTI_GetITStatus(EXTI_Line0) != RESET)
//    {	
//    /* Toggle LED4 */
//    GPIO_ToggleBits(GPIOD,GPIO_Pin_13); 

//    /* Clear the EXTI line 5 pending bit */
//    EXTI_ClearITPendingBit(EXTI_Line0);
//	}
}
void EXTI9_5_IRQHandler(void)
{}
void TIM3_IRQHandler(void)//定时器3中断处理函数
{
//	u16 capture=0;
//  	if (TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET)//比较通道1中断
//  	{
//    	TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
//		GPIO_ToggleBits(GPIOD,GPIO_Pin_12);;//LED2闪烁
//		capture = TIM_GetCapture1(TIM3);//得到比较寄存器1的值
//    	TIM_SetCompare1(TIM3, capture + 10000);//重新设置比较寄存器1的值
//  	}
//  	else if (TIM_GetITStatus(TIM3, TIM_IT_CC2) != RESET)//比较通道2中断
//  	{
//    	TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);
//		GPIO_ToggleBits(GPIOD,GPIO_Pin_14);;//LED3闪烁
//    	capture = TIM_GetCapture2(TIM3);//得到比较寄存器2的值
//    	TIM_SetCompare2(TIM3, capture + 5000);//重新设置比较寄存器2的值
//  	}
}

//void SDIO_IRQHandler(void)
//{
//  	SD_ProcessIRQSrc();
//}

//void SD_SDIO_DMA_IRQHANDLER(void)
//{
//  	SD_ProcessDMAIRQ();
//}

/**
  * @}
  */ 

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
