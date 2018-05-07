/**
 ******************************************************************************
 * @file    tcpsocketSTM32Cube.c
 * @author  Central LAB
  * @version V3.2.2
  * @date    22-Jan-2018
 * @brief   Adapter to tcp Socket
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

//#define DEBUG_TCP_CLIENT
 
#ifdef DEBUG_TCP_CLIENT
#include "STM32CubeInterface.h"
#endif /* DEBUG_TCP_CLIENT */


#include <stddef.h>
#include <stdlib.h>
#include "TargetFeatures.h"
#include "tcpsocketSTM32Cube.h"
#include "azure_c_shared_utility/xlogging.h"
//#include "wifi_interface.h"
#include "BG96_interface.h"
#include "TLocalBuffer.h"
#include "azure1_config.h"

#ifndef STM32F401xE
extern uint8_t SocketHandleAzure[2];
static uint32_t  connected_sockets = 0;
#endif /* STM32F401xE */

/**
 * @brief Function for allocating a TCP Socket Structure
 * @param None
 * @retval TCPSOCKETCONNECTION_HANDLE Pointer to the TCP Socket structure
 */
TCPSOCKETCONNECTION_HANDLE tcpsocketconnection_create(void)
{
#ifdef DEBUG_TCP_CLIENT
  AZURE_PRINTF(">>tcpsocketconnection_create...TBD\r\n");
#endif /* DEBUG_TCP_CLIENT */
  TCPSOCKETCONNECTION_HANDLE handler;
  handler=(TCPSOCKETCONNECTION_HANDLE )malloc(sizeof(TCPSOCKETCONNECTION));
  
  return handler;
}

/**
 * @brief Function not implemented
 * @param TCPSOCKETCONNECTION_HANDLE tcpSocketConnectionHandle Pointer to the TCP Socket structure
 * @param int blocking
 * @param  unsigned int timeout
 * @retval None
 */
void tcpsocketconnection_set_blocking(TCPSOCKETCONNECTION_HANDLE tcpSocketConnectionHandle, int blocking, unsigned int timeout)
{
#ifdef DEBUG_TCP_CLIENT
  AZURE_PRINTF(">>tcpsocketconnection_set_blocking...TBD\r\n");
#endif /* DEBUG_TCP_CLIENT */
}

/**
 * @brief Function for de-allocating a TCP Socket Structure
 * @param TCPSOCKETCONNECTION_HANDLE tcpSocketConnectionHandle Pointer to the TCP Socket structure
 * @retval None
 */
void tcpsocketconnection_destroy(TCPSOCKETCONNECTION_HANDLE tcpSocketConnectionHandle)
{
  free(tcpSocketConnectionHandle);
}

/**
 * @brief Function for opening a Socket whit one host using a specific port
 * @param TCPSOCKETCONNECTION_HANDLE tcpSocketConnectionHandle Pointer to the TCP Socket structure
 * @param char* host Host address
 * @param int port Port for TCP connection
 * @retval Int OK/Error (0/1)
 */
int tcpsocketconnection_connect(TCPSOCKETCONNECTION_HANDLE tcpSocketConnectionHandle, const char* host, const int port)
{
  BG96_Status_t BG96_status;
#ifdef DEBUG_TCP_CLIENT
  AZURE_PRINTF(">>tcpsocketconnection_connect port=%d [%s]\r\n",port,host);
#endif /* DEBUG_TCP_CLIENT */
#ifndef STM32F401xE
  if (connected_sockets > 1) {
    AZURE_PRINTF("Err: it's possible to have only 2 Socket with mbedTLS\r\n");
    return 1;
  }
#endif /* STM32F401xE */

#ifdef STM32F401xE
  BG96_status = BG96_socket_client_open((uint8_t*)host, (uint32_t)port, (uint8_t*) "t", &tcpSocketConnectionHandle->SocketHandle);
#else /* STM32F401xE */
  BG96_status = BG96_socket_client_open((uint8_t*)host, (uint32_t)port, (uint8_t*) "t", &SocketHandleAzure[connected_sockets]);
  tcpSocketConnectionHandle->SocketHandle = SocketHandleAzure[connected_sockets];
#endif /* STM32F401xE */

  if(BG96_status != BG96_MODULE_SUCCESS) {
    return 1;
  } else {
#ifndef STM32F401xE
    connected_sockets++;
#endif /* STM32F401xE */
#ifdef DEBUG_TCP_CLIENT
  #ifdef STM32F401xE
    AZURE_PRINTF("Socket open\r\n");
  #else /* STM32F401xE */
    AZURE_PRINTF("Socket open -> connected_sockets=%d\r\n",connected_sockets);
  #endif /* STM32F401xE */
#endif /* DEBUG_TCP_CLIENT */
    return 0;
  }
}

/**
 * @brief Function for closing a Socket
 * @param TCPSOCKETCONNECTION_HANDLE tcpSocketConnectionHandle Pointer to the TCP Socket structure
 * @retval None
 */
void tcpsocketconnection_close(TCPSOCKETCONNECTION_HANDLE tcpSocketConnectionHandle)
{
  BG96_Status_t BG96_status;

#ifdef DEBUG_TCP_CLIENT
  AZURE_PRINTF(">>tcpsocketconnection_close\r\n");
#endif /* DEBUG_TCP_CLIENT */

  BG96_status = BG96_socket_client_close(tcpSocketConnectionHandle->SocketHandle);
#ifndef STM32F401xE
  tcpSocketConnectionHandle->SocketHandle = SocketHandleAzure[connected_sockets-1]=0xFF;
#endif /* STM32F401xE */

  if(BG96_status != BG96_MODULE_SUCCESS){
   // print error message
  } else {
#ifndef STM32F401xE
   connected_sockets--;
#endif /* STM32F401xE */
#ifdef DEBUG_TCP_CLIENT
  #ifdef STM32F401xE
   AZURE_PRINTF("Socket close\r\n");
  #else /* STM32F401xE */
   AZURE_PRINTF("Socket close -> connected_sockets=%d\r\n",connected_sockets);
  #endif /* STM32F401xE */
#endif /* DEBUG_TCP_CLIENT */
  }
  return;
}



/**
* @brief Function for sending data to a socket
* @param TCPSOCKETCONNECTION_HANDLE tcpSocketConnectionHandle Pointer to the TCP Socket structure
* @param char* data source buffer
* @param int length data leght
* @retval int Number of Bytes written
*/
int tcpsocketconnection_send(TCPSOCKETCONNECTION_HANDLE tcpSocketConnectionHandle, const char* data, int length)
{
#ifdef DEBUG_TCP_CLIENT
  AZURE_PRINTF("<<tcpsocketconnection_send SocketHandle=%d size=%d\r\n",tcpSocketConnectionHandle->SocketHandle,length);
#endif /* DEBUG_TCP_CLIENT */
  int32_t DataLength=0;
#define TPCSOCKET_MIN(a,b) (((a)>(b))? (b) : (a))
  while(DataLength<length) {
    int32_t ChunkData = TPCSOCKET_MIN((length-DataLength),1020);
    BG96_Status_t BG96Status = BG96_socket_client_write(tcpSocketConnectionHandle->SocketHandle, (uint16_t) ChunkData,(char *) data+DataLength);
    DataLength += ChunkData;
    if ( BG96Status == BG96_MODULE_SUCCESS ) {
#ifdef DEBUG_TCP_CLIENT
      AZURE_PRINTF(">>tcpsocketconnection_send (BG96_MODULE_SUCCESS) size=%d\r\n", ChunkData);
#endif /* DEBUG_TCP_CLIENT */
    } else {
#ifdef DEBUG_TCP_CLIENT
      AZURE_PRINTF(">>tcpsocketconnection_send (NOT BG96_MODULE_SUCCESS but=%d) size=%d\r\n",WifiStatus,ChunkData);
#endif /* DEBUG_TCP_CLIENT */
      return -BG96Status;
    }
  }
#undef TPCSOCKET_MIN  
  return length;
}


/**
 * @brief Function for reading data from a socket
 * @param TCPSOCKETCONNECTION_HANDLE tcpSocketConnectionHandle Pointer to the TCP Socket structure
 * @param char* data destination buffer
 * @param int length data leght to read
 * @retval int Number of Bytes read
 */
int tcpsocketconnection_receive(TCPSOCKETCONNECTION_HANDLE tcpSocketConnectionHandle, char* data, int length)
{
#ifdef DEBUG_TCP_CLIENT
  AZURE_PRINTF(">>tcpsocketconnection_receive size=%d\r\n",length);
#endif /* DEBUG_TCP_CLIENT */
  int sizeReceived;
#ifdef STM32F401xE
  sizeReceived = LocalBufferGetSizeBuffer(&localBufferReading);
#else /* STM32F401xE */
  sizeReceived = LocalBufferGetSizeBuffer(&localBufferReading[tcpSocketConnectionHandle->SocketHandle]);
#endif /* STM32F401xE */

  if(sizeReceived > 0){
    if(sizeReceived >= (int)length) {
#ifdef STM32F401xE
      LocalBufferPopBuffer(&localBufferReading, data, (int)length);
#else /* STM32F401xE */
      LocalBufferPopBuffer(&localBufferReading[tcpSocketConnectionHandle->SocketHandle], data, (int)length);
#endif /* STM32F401xE */
      return length;
    } else {
#ifdef STM32F401xE
      LocalBufferPopBuffer(&localBufferReading, data, sizeReceived);
#else /* STM32F401xE */
      LocalBufferPopBuffer(&localBufferReading[tcpSocketConnectionHandle->SocketHandle], data, sizeReceived);
#endif /* STM32F401xE */
      return sizeReceived;
    }
  }
  return 0;
}

/**
 * @brief Function for reading data from a socket
 * @param TCPSOCKETCONNECTION_HANDLE tcpSocketConnectionHandle Pointer to the TCP Socket structure
 * @param char* data destination buffer
 * @param int length data leght to read
 * @retval int Number of Bytes read
 */
int tcpsocketconnection_receive_all(TCPSOCKETCONNECTION_HANDLE tcpSocketConnectionHandle, char* data, int length)
{
#ifdef DEBUG_TCP_CLIENT
  AZURE_PRINTF(">>tcpsocketconnection_receive_all (don't know the difference with tcpsocketconnection_send) size=%d\r\n",length);
#endif /* DEBUG_TCP_CLIENT */
  int sizeReceived;
#ifdef STM32F401xE
  sizeReceived = LocalBufferGetSizeBuffer(&localBufferReading);
#else /* STM32F401xE */
  sizeReceived = LocalBufferGetSizeBuffer(&localBufferReading[tcpSocketConnectionHandle->SocketHandle]);
#endif /* STM32F401xE */

  if(sizeReceived > 0){
    if(sizeReceived >= (int)length) {
#ifdef STM32F401xE
      LocalBufferPopBuffer(&localBufferReading, data, (int)length);
#else /* STM32F401xE */
      LocalBufferPopBuffer(&localBufferReading[tcpSocketConnectionHandle->SocketHandle], data, (int)length);
#endif /* STM32F401xE */
      return length;
    } else {
#ifdef STM32F401xE
      LocalBufferPopBuffer(&localBufferReading, data, sizeReceived);
#else /* STM32F401xE */
      LocalBufferPopBuffer(&localBufferReading[tcpSocketConnectionHandle->SocketHandle], data, sizeReceived);
#endif /* STM32F401xE */
      return sizeReceived;
    }
  }
  return 0;
}
/******************* (C) COPYRIGHT 2017 STMicroelectronics *****END OF FILE****/
