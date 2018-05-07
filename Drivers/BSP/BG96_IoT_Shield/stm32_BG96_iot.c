/**
 ******************************************************************************
 * @file    stm32_BG96_iot.c
 * @author  abe
 * @version V1.1.0
 * @date    24-04-2108
 * @brief   HAL related functionality for BG96 Quectel IoT module sample demo
 ******************************************************************************

 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/

#if defined (USE_STM32F4XX_NUCLEO)
#include "stm32f4xx_hal.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_usart.h"
#endif

#if defined (USE_STM32L4XX_NUCLEO)
#include "stm32l4xx_hal.h"
#include "stm32l4xx_ll_system.h"
#include "stm32l4xx_ll_usart.h"
#endif

#include "stm32_BG96_iot.h"

/* TIM handle declaration */
TIM_HandleTypeDef   	BG96_TimHandle;

/* UART handle declaration */
UART_HandleTypeDef 		UartBG96Handle;

# if 1

/** @addtogroup BSP
* @{
*/ 


/** @defgroup  NUCLEO_BG96_DRIVER
  * @brief Wi-Fi_driver modules
  * @{
  */


/** @defgroup NUCLEO_BG96_DRIVER_Private_Defines
  * @{
  */
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Prescaler declaration */
uint32_t uwPrescalerValue = 0;  

/* TIM handle declaration */

/**
  * @}
  */

/** @addtogroup NUCLEO_BG96_DRIVER_Private_Variables
  * @{
  */
/* Private variables ---------------------------------------------------------*/

/**
  * @}
  */

  
/** @defgroup NUCLEO_BG96_DRIVER_Private_Functions
  * @{
  */

  /*##-1- Configure the TIM peripheral #######################################*/
  /* -----------------------------------------------------------------------
    In this example TIM3 input clock (TIM3CLK)  is set to APB1 clock (PCLK1) x2,
    since APB1 prescaler is set to 4 (0x100).
       TIM3CLK = PCLK1*2
       PCLK1   = HCLK/2
    => TIM3CLK = PCLK1*2 = (HCLK/2)*2 = HCLK = SystemCoreClock
    To get TIM3 counter clock at 10 KHz, the Prescaler is computed as following:
    Prescaler = (TIM3CLK / TIM3 counter clock) - 1
    Prescaler = (SystemCoreClock /10 KHz) - 1

    Note:
     SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f1xx.c file.
     Each time the core clock (HCLK) changes, user had to update SystemCoreClock
     variable value. Otherwise, any configuration based on this variable will be incorrect.
     This variable is updated in three ways:
      1) by calling CMSIS function SystemCoreClockUpdate()
      2) by calling HAL API function HAL_RCC_GetSysClockFreq()
      3) each time HAL_RCC_ClockConfig() is called to configure the system clock frequency
  ----------------------------------------------------------------------- */
void BG96_Timer_Config(void)
{
  /* Compute the prescaler value to have TIMx counter clock equal to 10000 Hz */
  uwPrescalerValue = (uint32_t)(SystemCoreClock / 10000) - 1;
  
  /* Set TIMbg instance */
  BG96_TimHandle.Instance = TIMbg;

  /* Initialize TIMx peripheral as follows:
       + Period = 10000 - 1
       + Prescaler = (SystemCoreClock/10000) - 1
       + ClockDivision = 0
       + Counter direction = Up
  */
	#if defined (USE_STM32F4XX_NUCLEO) || (USE_STM32L4XX_NUCLEO)
  BG96_TimHandle.Init.Period            = 10 - 1;
  BG96_TimHandle.Init.Prescaler         = uwPrescalerValue;
  BG96_TimHandle.Init.ClockDivision     = 0;
  BG96_TimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
 
  if (HAL_TIM_Base_Init(&BG96_TimHandle) != HAL_OK)
  {
    /* Initialization Error */
    BG96_Error_Handler(); 
  }
	
	HAL_TIM_Base_Start_IT(&BG96_TimHandle);			//Start the TIM timer
	#endif
	
}

/**
* @brief  USART_Configuration
* BG96_UART configured as follow:
*      - BaudRate = 115200 baud  
*      - Word Length = 8 Bits
*      - One Stop Bit
*      - No parity
*      - Hardware flow control enabled (RTS and CTS signals)
*      - Receive and transmit enabled
*
* @param  None
* @retval None
*/
void BG96_Configuration(void)
{
	GPIO_InitTypeDef  GPIO_InitStruct;

  RESET_GPIO_CLK_ENABLE();
	HAL_GPIO_WritePin(BG96_RESET_GPIO_PORT, BG96_RESET_GPIO_PIN, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin       = BG96_RESET_GPIO_PIN;
  GPIO_InitStruct.Mode      = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(BG96_RESET_GPIO_PORT, &GPIO_InitStruct);
	
	PWRKEY_GPIO_CLK_ENABLE();
	HAL_GPIO_WritePin(BG96_PWRKEY_GPIO_PORT, BG96_PWRKEY_GPIO_PIN, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin       = BG96_PWRKEY_GPIO_PIN;
  GPIO_InitStruct.Mode      = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(BG96_PWRKEY_GPIO_PORT, &GPIO_InitStruct);
	
	VBAT_GPIO_CLK_ENABLE();
	HAL_GPIO_WritePin(BG96_VBAT_GPIO_PORT, BG96_VBAT_GPIO_PIN, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin       = BG96_VBAT_GPIO_PIN;
  GPIO_InitStruct.Mode      = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(BG96_VBAT_GPIO_PORT, &GPIO_InitStruct);
		
  UartBG96Handle.Instance             = BG96_UART;
  UartBG96Handle.Init.BaudRate        = 115200;
  UartBG96Handle.Init.WordLength      = UART_WORDLENGTH_8B;
  UartBG96Handle.Init.StopBits        = UART_STOPBITS_1;
  UartBG96Handle.Init.Parity          = UART_PARITY_NONE ;
  UartBG96Handle.Init.HwFlowCtl       = UART_HWCONTROL_RTS;//UART_HWCONTROL_NONE;
  UartBG96Handle.Init.Mode            = UART_MODE_TX_RX;
  UartBG96Handle.Init.OverSampling    = UART_OVERSAMPLING_16;
  //UartWiFiHandle.Init.OneBitSampling  = UART_ONEBIT_SAMPLING_ENABLED;
  
  if(HAL_UART_DeInit(&UartBG96Handle) != HAL_OK)
  {
    BG96_Error_Handler();
  }  
  if(HAL_UART_Init(&UartBG96Handle) != HAL_OK)
  {
    BG96_Error_Handler();
  }  
	
	/* Enable RXNE and Error interrupts */  
  LL_USART_EnableIT_RXNE(BG96_UART);
  LL_USART_EnableIT_ERROR(BG96_UART);
	
}

/**
  * @brief  String send to BG96
  */ 

HAL_StatusTypeDef BG96_uart_send(uint8_t* string, uint16_t size)
{
	return HAL_UART_Transmit(&UartBG96Handle, (uint8_t*)string, size, HAL_MAX_DELAY);

}

/**
  * @brief  Single char sned to BG96
  */ 

int BG96_char_send(uint8_t c)
{
	return HAL_UART_Transmit(&UartBG96Handle, &c, 1, HAL_MAX_DELAY);
}
 
/**
  * @}
  */ 



 
/**
  * @}
  */ 

 /**
* @brief  Error_Handler
*         This function is executed in case of error occurrence.
* @param  None
* @retval None
*/
void BG96_Error_Handler(void)
{
  /* Turn LED2 on */
  BSP_LED_On(LED2);
  //The following while(1) is commented as it prevents standby functionality
  while(1)
  {
    //Error if LED2 is slowly blinking (1 sec. period)
    BSP_LED_Toggle(LED2); 
    HAL_Delay(1000); 
  } 

}

/**
  * @}
  */ 
 
/* Power KEY pin *****************************************************************/
void BG96_pwr_key_on(void)
{
		HAL_GPIO_WritePin(BG96_PWRKEY_GPIO_PORT, BG96_PWRKEY_GPIO_PIN, GPIO_PIN_SET);
}


void BG96_pwr_key_off(void)
{
		HAL_GPIO_WritePin(BG96_PWRKEY_GPIO_PORT, BG96_PWRKEY_GPIO_PIN, GPIO_PIN_RESET);
}
	
/* RESET pin *********************************************************************/
void BG96_reset_on(void)
{
		HAL_GPIO_WritePin(BG96_RESET_GPIO_PORT, BG96_RESET_GPIO_PIN, GPIO_PIN_SET);
}


void BG96_reset_off(void)
{
		HAL_GPIO_WritePin(BG96_RESET_GPIO_PORT, BG96_RESET_GPIO_PIN, GPIO_PIN_RESET);
}

/* VBAT pin **********************************************************************/
void BG96_vbat_on(void)
{
		HAL_GPIO_WritePin(BG96_VBAT_GPIO_PORT, BG96_VBAT_GPIO_PIN, GPIO_PIN_SET);
}


void BG96_vbat_off(void)
{
		HAL_GPIO_WritePin(BG96_VBAT_GPIO_PORT, BG96_VBAT_GPIO_PIN, GPIO_PIN_RESET);
}

#endif

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/

