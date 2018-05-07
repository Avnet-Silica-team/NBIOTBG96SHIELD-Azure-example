/**
 ******************************************************************************
 * @file    stm32_BG96_iot.h
 * @author  abe
 * @version V1.1.0
 * @date    24-04-2108
 * @brief   HAL header for related functionality for BG96 Quectel IoT module sample demo
 ******************************************************************************

 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BG96_IOT_MODULE_CONF_H
#define __BG96_IOT_MODULE_CONF_H

/* Includes ------------------------------------------------------------------*/

#ifdef USE_STM32F4XX_NUCLEO  
#include "stm32f4xx_hal.h"
#include "stm32f4xx_nucleo.h"
#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_hal_rcc_ex.h"
#endif

#ifdef USE_STM32L4XX_NUCLEO
#include "stm32l4xx_hal.h"
#include "stm32l4xx_nucleo.h"
#include "stm32l4xx_hal_rcc.h"
#include "stm32l4xx_hal_rcc_ex.h"
#endif

/* Exported macro ------------------------------------------------------------*/
//#define DEBUG_PRINT 1
#define BG96_ENABLE			1

/********** TimeOUT *******************/
#define BG96_TIME_OUT           				40000		//4 sec
#define BG96_HTTP_REQUEST_INTERVAL      60			//6sec

/* User can use this section to tailor USARTx/UARTx instance used and associated 
resources */
/* Definition for USARTx clock resources */
#ifdef USE_STM32F4XX_NUCLEO

#define BG96_UART                     USART1
#define USARTbg_CLK_ENABLE()              __HAL_RCC_USART1_CLK_ENABLE()
#define USARTbg_RX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()
#define USARTbg_TX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()

#define USARTbg_FORCE_RESET()             __HAL_RCC_USART1_FORCE_RESET()
#define USARTbg_RELEASE_RESET()           __HAL_RCC_USART1_RELEASE_RESET()

#endif //USE_STM32F4XX_NUCLEO  

#ifdef USE_STM32L4XX_NUCLEO
#define BG96_UART                     USART1
#define USARTbg_CLK_ENABLE()              __HAL_RCC_USART1_CLK_ENABLE()
#define USARTbg_RX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()
#define USARTbg_TX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()

#define USARTbg_FORCE_RESET()             __HAL_RCC_USART1_FORCE_RESET()
#define USARTbg_RELEASE_RESET()           __HAL_RCC_USART1_RELEASE_RESET()

#endif //USE_STM32L4XX_NUCLEO 




/* Definition for USARTx Pins */
#define BG96_USART_TX_PIN                    GPIO_PIN_9
#define BG96_USART_TX_GPIO_PORT              GPIOA
#define BG96_USART_RX_PIN                    GPIO_PIN_10
#define BG96_USART_RX_GPIO_PORT              GPIOA

#define BG96_USART_RTS_PIN                   GPIO_PIN_12
#define BG96_USART_RTS_GPIO_PORT             GPIOA
#define BG96_USART_CTS_PIN                   GPIO_PIN_11
#define BG96_USART_CTS_GPIO_PORT             GPIOA

#if defined (USE_STM32L4XX_NUCLEO) || (USE_STM32F4XX_NUCLEO)
#define BG96_USART_TX_AF                    GPIO_AF7_USART1
#define BG96_USART_RX_AF                    GPIO_AF7_USART1
#endif    


#define  BG96_RESET_GPIO_PIN              GPIO_PIN_8
#define  BG96_RESET_GPIO_PORT             GPIOA /*PA_8*/

#define  BG96_PWRKEY_GPIO_PIN             GPIO_PIN_6
#define  BG96_PWRKEY_GPIO_PORT            GPIOB /*PB_6*/

#define  BG96_VBAT_GPIO_PIN             	GPIO_PIN_7
#define  BG96_VBAT_GPIO_PORT            	GPIOA /*PA_7*/


#if defined (USE_STM32L4XX_NUCLEO) || (USE_STM32F4XX_NUCLEO)
#define RESET_GPIO_CLK_ENABLE() 		 __HAL_RCC_GPIOA_CLK_ENABLE()
#define VBAT_GPIO_CLK_ENABLE()    	 __HAL_RCC_GPIOA_CLK_ENABLE()
#define PWRKEY_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOB_CLK_ENABLE()
#endif

/* Definition for USARTx's NVIC */
#if defined (USE_STM32L4XX_NUCLEO) || (USE_STM32F4XX_NUCLEO)
#define USARTbg_IRQn                     USART1_IRQn
#define USARTbg_IRQHandler               USART1_IRQHandler

#endif //USE_STM32F4XX_NUCLEO

#if defined (USE_STM32L4XX_NUCLEO) || (USE_STM32F4XX_NUCLEO)
#define TIMbg                           TIM3
#define TIMbg_CLK_ENABLE()              __HAL_RCC_TIM3_CLK_ENABLE()

/* Definition for TIMbg's NVIC */
#define TIMbg_IRQn                      TIM3_IRQn
#define TIMbg_IRQHandler                TIM3_IRQHandler
#endif


/* Exported functions ------------------------------------------------------- */

/* Size of Trasmission buffer */
#define BG96_TXBUFFERSIZE                   (COUNTOF(aTxBuffer) - 1)
/* Size of Reception buffer */
#define BG96_RXBUFFERSIZE                    BG96_TXBUFFERSIZE

#define BG96_USART_BAUD_RATE                115200

/* Exported variables and structures ------------------------------------------------------- */

extern TIM_HandleTypeDef   	BG96_TimHandle;
extern UART_HandleTypeDef 	UartBG96Handle;

/* Exported functions prototipes ------------------------------------------------------- */

HAL_StatusTypeDef BG96_uart_send(uint8_t* string, uint16_t size);
int BG96_char_send(uint8_t c);

void BG96_Error_Handler(void);

void BG96_Timer_Config(void);
void BG96_Configuration(void);

void BG96_pwr_key_on(void);
void BG96_pwr_key_off(void);
void BG96_reset_on(void);
void BG96_reset_off(void);
void BG96_vbat_on(void);
void BG96_vbat_off(void);

#endif //__BG96_IOT_MODULE_CONF_H
