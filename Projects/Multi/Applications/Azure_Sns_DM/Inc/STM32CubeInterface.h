/**
 ******************************************************************************
 * @file    STM32CubeInterface.h
 * @author  Central LAB
  * @version V3.2.2
  * @date    22-Jan-2018
 * @brief   Header file for STM32 Cube interface file
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32CUBEINTERFACE_H
#define __STM32CUBEINTERFACE_H

/* Includes ------------------------------------------------------------------*/
#include <time.h>
#include "TargetFeatures.h"
#ifndef USE_STM32L475E_IOT01
#include "HWAdvanceFeatures.h"
#ifndef STM32F429xx
//#include "wifi_globals.h"
//#include "wifi_interface.h"
#include "BG96_interface.h"
#endif /* STM32F429xx */
#endif /* USE_STM32L475E_IOT01 */
#include "azure_c_shared_utility/macro_utils.h"

/* Exported Types ----------------------------------------------------------- */
#ifndef STM32F429xx

#if 0 /* NO WIFI */
#ifdef USE_STM32L475E_IOT01
typedef enum
{
  None          = 0, 
  WEP           = 1,
  WPA_Personal  = 2,
} WiFi_Priv_Mode;
#else /* USE_STM32L475E_IOT01 */
typedef enum {
  wifi_state_reset = 0,
  wifi_state_ready,
  wifi_state_idle,
  wifi_state_connected,
  wifi_state_connecting,
  wifi_state_disconnected,
  wifi_state_activity,
  wifi_state_inter,
  wifi_state_print_data,
  wifi_state_error,
  wifi_undefine_state       = 0xFF,
} wifi_state_t;
#endif /* USE_STM32L475E_IOT01 */

typedef struct {
  char ssid[40];
  char seckey[40];
  WiFi_Priv_Mode mode;
} WIFI_CredAcc_t;
#endif	/* NO WIFI */

#endif /* STM32F429xx */
#ifndef STM32F401xE
#define AZURE1_OTA_STATUS \
  OTA_STATUS_NULL, \
  OTA_STATUS_START, \
  OTA_STATUS_RUNNIG, \
  OTA_STATUS_END, \
  OTA_STATUS_ERROR
DEFINE_ENUM(AZURE1_OTA, AZURE1_OTA_STATUS)
#endif /* STM32F401xE */

#define AZURE1_ERROR_VALUES \
  AZURE_ERR_NULL, \
  AZURE_ERR_PLATFORM_INIT, \
  AZURE_ERR_FOTA, \
  AZURE_ERR_BL_COMPLIANCE, \
  AZURE_ERR_IOT_START, \
  AZURE_ERR_NFC, \
  AZURE_ERR_WIFI_CRED, \
  AZURE_ERR_WIFI, \
  AZURE_ERR_WIFI_CON, \
  AZURE_ERR_MALLOC, \
  AZURE_ERR_REGISTRATION, \
  AZURE_ERR_NTP
DEFINE_ENUM(AZURE1_ERROR, AZURE1_ERROR_VALUES)

/* Exported variables --------------------------------------------------------*/
extern volatile uint32_t ButtonPressed;
extern volatile uint32_t SendData;
#if ((defined USE_STM32L475E_IOT01) | (defined STM32F429xx))
  extern uint8_t macAddress[6];
#endif /* ((defined USE_STM32L475E_IOT01) | (defined STM32F429xx)) */

#ifndef USE_STM32L475E_IOT01
  extern volatile uint32_t MEMSInterrupt;
  #ifndef STM32F429xx
    extern uint8_t macaddstart[32];
  #endif /* STM32F429xx */
#endif /* USE_STM32L475E_IOT01 */

/* Exported functions ------------------------------------------------------- */
extern void RTC_CalendarShow(uint8_t *showtime, uint8_t *showdate);
extern time_t TimingSystemGetSystemTime(void);
extern int TimingSystemSetSystemTime(time_t epochTimeNow);
extern time_t SynchronizationAgentConvertNTPTime2EpochTime(uint8_t* pBufferTimingAnswer,size_t sizeBuffer);
#ifndef USE_STM32L475E_IOT01
extern uint32_t MEMSCallback(void);
#endif /* USE_STM32L475E_IOT01 */
extern void StartTimer1(void);
extern void StopTimer1(void);
extern void AzureExit(AZURE1_ERROR Value);
#ifndef STM32F401xE
extern AZURE1_OTA FOTACallback(char * hostname,char type, uint32_t  port_num,char * path);
#endif /* STM32F401xE */

#endif /* __STM32CUBEINTERFACE_H */
/******************* (C) COPYRIGHT 2017 STMicroelectronics *****END OF FILE****/
