/**
  ******************************************************************************
  * @file    m24sr_wrapper.h
  * @author  MCD Application Team
  * @version V3.2.2
  * @date    22-Jan-2018
  * @brief   This file makes the bridge between middleware generic NCF TagType4 
  *          function calls and the M24SR component. Furthermore it simplifies 
  *          the use of the M24SR driver by sequencing some commands.
   ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V.
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __M24SR_WRAPPER_H
#define __M24SR_WRAPPER_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "string.h"
#include "stm32l475e_iot01.h"
#include "stm32l4xx.h"
#include "azure1_config.h"
//#include "lib_TagType4.h"
#include "m24sr_v1.1.h"

/** @addtogroup M24SR_WRAPPER
  * @{
  */

/** @defgroup M24SR_WRAPPER_Exported_Constants  M24SR_WRAPPER Exported Constants
  * @{
  */

/**
  * @brief  GPO mode structure 
  */
typedef enum{
  M24SR_RF_GPO= 0,
  M24SR_I2C_GPO
}M24SR_GPO_MODE; 

/**
  * @brief  M24SR_WRAPPER_Status_Code
  */
/* ---------------------- status code ----------------------------------------*/
#define NFC_TT4_ACTION_COMPLETED    M24SR_ACTION_COMPLETED

/**
  * @brief  lib_M24SR_Flag_to_select_open_session_command
  */  
#define NFC_TT4_ASK_FOR_SESSION                     0x0000
#define NFC_TT4_TAKE_SESSION                        0xFFFF

#define ReadData              NFC_TT4_ReadData
#define WriteData             NFC_TT4_WriteData
#define NDEF_MAX_SIZE	      M24SR16_NDEF_MAX_SIZE
#define NDEF_ACTION_COMPLETED M24SR_ACTION_COMPLETED
#define OpenNDEFSession       NFC_TT4_OpenNDEFSession
#define CloseNDEFSession      NFC_TT4_CloseNDEFSession
#define TagT4Init             NFC_TT4_Initialization
#define ASK_FOR_SESSION       NFC_TT4_ASK_FOR_SESSION
#define TAKE_SESSION          NFC_TT4_TAKE_SESSION
#define ForceReadData         NFC_TT4_ForceReadData

/**
  * @}
  */


/** @defgroup M24SR_WRAPPER_Exported_FunctionsPrototypes  M24SR_WRAPPER Exported FunctionsPrototypes
  * @{
  */

uint16_t NFC_TT4_Initialization (uint8_t* pCCBuffer, uint8_t size );

uint16_t NFC_TT4_GetNDEFFileId ( uint16_t *NDEF_fileID );
uint16_t NFC_TT4_OpenNDEFSession ( uint16_t NDEF_fileID, uint16_t Priority );
uint16_t NFC_TT4_ReadData ( uint16_t Offset , uint16_t DataSize , uint8_t* pData);
uint16_t NFC_TT4_ForceReadData ( uint16_t Offset , uint16_t DataSize , uint8_t* pData);
uint16_t NFC_TT4_WriteData ( uint16_t Offset , uint16_t DataSize , uint8_t* pData);
uint16_t NFC_TT4_CloseNDEFSession ( uint16_t NDEF_fileID );

uint16_t NFC_TT4_EnableReadPassword( uint8_t* pCurrentWritePassword, uint8_t* pNewPassword);
uint16_t NFC_TT4_DisableReadPassword( uint8_t* pCurrentWritePassword );
uint16_t NFC_TT4_EnableWritePassword( uint8_t* pCurrentWritePassword, uint8_t* pNewPassword);
uint16_t NFC_TT4_DisableWritePassword( uint8_t* pCurrentWritePassword );
uint16_t NFC_TT4_DisableAllPassword( uint8_t* pSuperUserPassword);

uint16_t NFC_TT4_EnableReadOnly( uint8_t* pCurrentWritePassword);
uint16_t NFC_TT4_DisableReadOnly( uint8_t* pCurrentWritePassword);
uint16_t NFC_TT4_EnableWriteOnly( uint8_t* pCurrentWritePassword);
uint16_t NFC_TT4_DisableWriteOnly( uint8_t* pCurrentWritePassword);

uint16_t NFC_TT4_ManageGPO( uint8_t gpo_config, uint8_t mode);

/**
  * @}
  */
  
/**
  * @}
  */  
#ifdef __cplusplus
}
#endif


#endif /* __M24SR_WRAPPER_H */
 
/******************* (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
