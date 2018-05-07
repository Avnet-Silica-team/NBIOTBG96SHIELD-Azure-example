/**
  ******************************************************************************
  * @file    stm32f4xx_it.c 
  * @author  Central LAB
  * @version V3.2.2
  * @date    22-Jan-2018
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
#ifndef STM32F429xx
  //#include "wifi_globals.h"
#else /* STM32F429xx */
  #include "ethernetif.h"
  #include "Typedef_LwIP_IKS01AX.h"
#endif /* STM32F429xx */
#include "TargetFeatures.h"

#include "BG96_interface.h"
#include "stm32_BG96_iot.h"

/* Imported variables ---------------------------------------------------------*/
extern TIM_HandleTypeDef    TimCCHandle;

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

#ifdef STM32F401xE
/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}
#endif /* STM32F401xE */

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}
#ifdef STM32F401xE
/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}
#endif /* STM32F401xE */

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
  HAL_IncTick();
#ifdef STM32F401xE
  //Wifi_SysTick_Isr();
#else /* STM32F401xE */
  osSystickHandler();
#endif /* STM32F401xE */
}

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles TIM1 Interrupt request
  * @param  None
  * @retval None
  */
void TIM1_CC_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&TimCCHandle);
}

#ifdef STM32F401xE

/**
  * @brief  This function handles TIM interrupt request.
  * @param  None
  * @retval None
  */
#if 0
void TIM2_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&PushTimHandle);
}
#endif

#endif /* STM32F401xE */
/**
* @brief  This function handles External line 10-15 interrupt request.
* @param  None
* @retval None
*/

void EXTI15_10_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(KEY_BUTTON_PIN);
}
#ifndef IKS01A2
/**
  * @brief  This function handles External line 4 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI4_IRQHandler( void )
{
  HAL_GPIO_EXTI_IRQHandler(M_INT1_PIN);
}
#endif /* IKS01A2 */
/**
  * @brief  This function handles External line 5-9 interrupt request
  * @param  None
  * @retval None
  */
void EXTI9_5_IRQHandler( void )
{
  HAL_GPIO_EXTI_IRQHandler(LSM6DSL_INT1_O_PIN);
}

/*******************************************************************************************************
//Usart2 GSM UG96 module IRQ service functions
*******************************************************************************************************/
#if BG96_ENABLE

uint32_t bg96_uart_err_counter = 0;

#include "stm32f4xx_hal.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_usart.h"

/**
  * @brief  UART error callbacks
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
static void UART_Error_Callback(void)
{
  __IO uint32_t isr_reg;

  /* Disable USARTx_IRQn */
  //NVIC_DisableIRQ(USARTx_IRQn);
  
  /* Error handling example :
    - Read USART ISR register to identify flag that leads to IT raising
    - Perform corresponding error handling treatment according to flag
  */

	bg96_uart_err_counter++;
	//LL_USART_ClearFlag_FE(BG96_UART);
	//LL_USART_ClearFlag_NE(BG96_UART);
	//LL_USART_ClearFlag_ORE(BG96_UART);

}

	uint32_t 	isr_reg_uart1;

/**
  * @brief  This function handles UART interrupt request.
  * @param  None
  * @retval None
  */
void USARTbg_IRQHandler(void)
{
  /* Customize process using LL interface to improve the performance (exhaustive feature management not handled) */
	uint8_t 	BG96_rx_char;
	
	isr_reg_uart1 = LL_USART_ReadReg(BG96_UART, SR);
	
	if ((isr_reg_uart1 & USART_SR_ORE) == USART_SR_ORE)
				UART_Error_Callback();
	
  /* Check RXNE flag value in ISR register */
	if ((isr_reg_uart1 & USART_SR_RXNE) == USART_SR_RXNE)
  //if(LL_USART_IsActiveFlag_RXNE(BG96_UART) && LL_USART_IsEnabledIT_RXNE(BG96_UART))
  {
    /* RXNE flag will be cleared by reading of RDR register (done in call) */
    /* Call function in charge of handling Character reception */
		BG96_rx_char = LL_USART_ReceiveData8(BG96_UART);
    BG96_rx_callback(BG96_rx_char);
		//_parser_char_recv(BG96_rx_char);
  }

}


void TIMbg_IRQHandler(void)
{
	__HAL_TIM_CLEAR_IT(&BG96_TimHandle, TIM_FLAG_UPDATE);
	BG96_timers_callback();
}
#endif


#if 0
void ETH_IRQHandler(void)
{
  ETHERNET_IRQHandler();
}
#endif /* STM32F401xE */
/************************ (C) COPYRIGHT 2017 STMicroelectronics *****END OF FILE****/
