/**
  ******************************************************************************
  * @file    azure1_config.h
  * @author  Central LAB
  * @version V3.2.2
  * @date    22-Jan-2018
  * @brief   azure1 configuration
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
#ifndef __azure1_CONFIG_H
#define __azure1_CONFIG_H

/* Exported define ------------------------------------------------------------*/

/*************** Enable Print Informations ******************/
#define AZURE_ENABLE_PRINTF

/*************** Default WIFI Credential ******************/
#ifndef STM32F429xx
#define AZURE_DEFAULT_SSID "STM"
#define AZURE_DEFAULT_SECKEY "STMdemoPWD"
#define AZURE_DEFAULT_PRIV_MODE WPA_Personal

#define AZURE_DEFAULT_SIMPIN "0000"
#define AZURE_DEFAULT_APN "internet"

#endif /* STM32F429xx */

/* Uncomment the following define for enabling the Registration to STM32ODE IoT Dashboard (https://stm32ode.azurewebsites.net/) */
//#define AZURE_ENABLE_REGISTRATION

/* NTP end points */
#define NTP_ENDPOINT_IPADDRESS_DEFAULT         "time.nist.gov"
#define NTP_ENDPOINT_IPADDRESS_BACKUP1         "time-c.nist.gov"
#define NTP_ENDPOINT_IPADDRESS_BACKUP2         "time-b.nist.gov"
#define NTP_ENDPOINT_TCPPORT           37
#define NTP_MAX_RETRIES                 2

#ifdef AZURE_ENABLE_REGISTRATION
  #define WEB_DASHBOARD_URL "stm32ode.azurewebsites.net"
#else /* AZURE_ENABLE_REGISTRATION */
  /* String containing Hostname, Device Id & Device Key in the format:                         */
  //#define AZUREDEVICECONNECTIONSTRING "HostName=<host_name>;DeviceId=<device_id>;SharedAccessKey=<device_key>"
	#define AZUREDEVICECONNECTIONSTRING "HostName=STM32-IoTHub.azure-devices.net;DeviceId=0080E1B9739B;SharedAccessKey=8UTQau5R9DdHlQp1K+yia3+z+q7jvlblLppR9kiHaRk="
#endif /* AZURE_ENABLE_REGISTRATION */

#if ( (defined USE_STM32L4XX_NUCLEO) || (defined USE_STM32L475E_IOT01) )
// #define AZURE_IOT_CENTRAL
#endif /* ((defined USE_STM32L4XX_NUCLEO)  */

/*************** Don't Change the following defines *************/

/* Package Version only numbers 0->9 */
#define AZURE_VERSION_MAJOR '3'
#define AZURE_VERSION_MINOR '2'
#define AZURE_VERSION_PATCH '2'

/* Package Name */
#ifdef STM32F401xE
  #define AZURE_PACKAGENAME "Azure_DM"
#else /* STM32F401xE */
  #define AZURE_PACKAGENAME "Azure_Sns_DM"
#endif /* STM32F401xE */

#ifdef AZURE_ENABLE_PRINTF
  #define AZURE_PRINTF(...) printf(__VA_ARGS__)
#else /* AZURE_ENABLE_PRINTF */
  #define AZURE_PRINTF(...)
#endif /* AZURE_ENABLE_PRINTF */

#ifdef USE_STM32L475E_IOT01
/**
 * UART1
 */
#define CFG_HW_UART1_BAUDRATE             115200
#define CFG_HW_UART1_WORDLENGTH           UART_WORDLENGTH_8B
#define CFG_HW_UART1_STOPBITS             UART_STOPBITS_1
#define CFG_HW_UART1_PARITY               UART_PARITY_NONE
#define CFG_HW_UART1_HWFLOWCTL            UART_HWCONTROL_NONE
#define CFG_HW_UART1_MODE                 UART_MODE_TX_RX
#define CFG_HW_UART1_ADVFEATUREINIT       UART_ADVFEATURE_NO_INIT

/**
 * NFC
 */

/**
  * @brief  M24SR_EEPROM_Size_Version
  */
#define M24SR04_NDEF_MAX_SIZE                0x200
#define M24SR16_NDEF_MAX_SIZE                0x800
#define M24SR64_NDEF_MAX_SIZE                0x2000
/* The maximum size of a NDEF will be 64kBits with M24SR64 */
/* if smaller memory used update this define to save space */
#define NFC_TT4_NDEF_MAX_SIZE          M24SR16_NDEF_MAX_SIZE

/**
  * @brief  M24SR_GPO_Config_Mode
  */
#define M24SR_I2C_GPO_POLLING   0 /* Normal  I²C polling */
#define M24SR_I2C_GPO_SYNCHRO   1 /* allow to use GPO polling as I2C synchronization */ 
#define M24SR_I2C_GPO_INTERRUPT 2 /* allow to use GPO interrupt as I2C synchronization */ 
/* The maximum size of a NDEF will be 64kBits with M24SR64 */
/* if smaller memory used update this define to save space */
#define M24SR_I2C_GPO_MODE            M24SR_I2C_GPO_INTERRUPT
    
/**
 * WIFI
 */
#define  WIFI_CONNECT_MAX_ATTEMPT_COUNT  3
#define  WIFI_GET_IP_MAX_ATTEMPT_COUNT  10
#define  WIFI_WRITE_TIMEOUT   400
#define  WIFI_READ_TIMEOUT    400
#define WIFI_PRODUCT_INFO_SIZE ES_WIFI_MAX_SSID_NAME_SIZE
#define WIFI_PAYLOAD_SIZE      ES_WIFI_PAYLOAD_SIZE
#endif /* USE_STM32L475E_IOT01 */
#endif /* __azure1_CONFIG_H */

/******************* (C) COPYRIGHT 2017 STMicroelectronics *****END OF FILE****/
