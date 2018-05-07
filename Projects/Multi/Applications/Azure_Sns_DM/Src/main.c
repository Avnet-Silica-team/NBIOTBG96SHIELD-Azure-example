/**
  ******************************************************************************
  * @file    main.c
  * @author  Central LAB
  * @version V3.2.2
  * @date    22-Jan-2018
  * @brief   main function
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

/**
 * @mainpage azure Device Management Software
 *
 * @image html st_logo.png
 *
 * <b>SW Diagram</b>
 *
 * @image html FP-CLD-AZURE1_SW_Diagram.png
 *
 * <b>Introduction</b>
 *
 * FP-CLD-AZURE1 provides a software running on STM32 which offers a complete middleware to build applications based on 
 * WiFi/Ethernet connectivity (SPW01SA/Nucleo-F429ZI) and to connect STM32 Nucleo boards (or B-L475E-IOT01A Discovery kit IoT node) with Microsoft Azure IoT services.
 * The software leverages functionalities provided by the following expansion boards (for Nucleo boards)
 *    -X-NUCLEO-IKS01A1 or X-NUCLEO-IKS01A2 featuring temperature, humidity, pressure and motion MEMS sensors
 *    -X-NUCLEO-IDW01M1 a Wi-Fi evaluation board based on the SPWF01SA module
 *    -X-NUCLEO-NFC01A1 a Dynamic NFC tag evaluation board

 * FP-CLD-AZURE1 is an function package software for STM32Cube. The software runs on the STM32 micro-controller and 
 * includes drivers that recognize WiFi/Ethernet module (SPWF01SA/Nucleo-F429ZI), sensor devices and dynamic NFC/RFID tag (M24SR64-Y).
 * It also includes the porting of the Microsoft Azure IoT device SDK for easy connection with 
 * Azure IoT services. The expansion software is built on STM32Cube software technology 
 * to ease portability across different STM32 microcontrollers. The software comes with examples for registering the 
 * device on Microsoft Azure IoT Hub, transmit data and receive commands, and allow Device Management with FOTA capability (Not for Nucleo F401RE)
 *
 * This application allows the user to control the initialization phase via UART.
 * Launch a terminal application and set the UART port to 115200 bps, 8 bit, No Parity, 1 stop bit
 *
 *                     ----------------------------------------
 *                     | VERY IMPORTANT For Nucleo != F401RE: |
 *                     ----------------------------------------
 * 1) This example support the Firmware-Over-The-Air (FOTA) update.
 *
 * 2) This example must run starting at address 0x08004000 in memory and works ONLY if the BootLoader 
 * is saved at the beginning of the FLASH (address 0x08000000)
 *
 *
 * For each IDE (IAR/µVision/System Workbench) there are some scripts CleanAzure1mbedTLS.bat/CleanAzure1mbedTLS.sh that makes the following operations:
 * - Full Flash Erase
 * - Load the BootLoader on the rigth flash region
 * - Load the Program (after the compilation) on the rigth flash region (This could be used for a FOTA)
 * - Dump back one single binary that contain BootLoader+Program that could be
 *  flashed at the flash beginning (address 0x08000000) (This COULD BE NOT used for FOTA)
 * - Reset the board
 */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#ifdef STM32F401xE
  #include "AzureClient_mqtt_TM.h"
#else /* STM32F401xE */
  #include "AzureClient_mqtt_DM_TM.h"
#endif /* STM32F401xE */
#ifdef STM32F429xx
  #include "cmsis_os.h"
#endif /* STM32F429xx */

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
void main(void)
{
  /* Working application */
#ifdef STM32F401xE
  AzureClient_mqtt_TM();
#else /* STM32F401xE */
  #ifdef STM32F429xx
    osThreadDef(Azure, AzureClient_mqtt_DM_TM, osPriorityNormal, 0, 4096 );

    osThreadCreate (osThread(Azure), NULL);

    /* Start scheduler */
    osKernelStart();

    /* We should never get here as control is now taken by the scheduler */
    for( ;; );
  #else /* STM32F429xx */
    AzureClient_mqtt_DM_TM();
  #endif /* STM32F429xx */
#endif /* STM32F401xE */
}

#ifdef configCHECK_FOR_STACK_OVERFLOW
void vApplicationStackOverflowHook( TaskHandle_t xTask, char *pcTaskName )
{
  printf("Stack overflow\r\n");
}
#endif /* configCHECK_FOR_STACK_OVERFLOW */

#ifdef configUSE_MALLOC_FAILED_HOOK
void vApplicationMallocFailedHook( void )
{
  printf("Heap  Alloc failed\r\n");
}
#endif /* configUSE_MALLOC_FAILED_HOOK */

/******************* (C) COPYRIGHT 2017 STMicroelectronics *****END OF FILE****/
