/**
  ******************************************************************************
  * @file    RegistrationAgentInternal.c
  * @author  Central LAB
  * @version V3.2.2
  * @date    25-January-2018
  * @brief   Registration Agent for usage of FP with https://stm32ode.azurewebsites.net/
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


#include <stdio.h>
#include <stdlib.h>
#include "STM32CubeInterface.h"
#include "RegistrationAgent.h"
#include "azure_c_shared_utility/crt_abstractions.h"
#include "azure_c_shared_utility/platform.h"
#include "azure_c_shared_utility/tls_config.h"
#include "azure_c_shared_utility/tlsio_mbedtls.h"
#include "azure_c_shared_utility/tlsio.h"
#ifdef STM32L475xx
  #include "tlsio_mbedtls_STM32Cube_iot01l475vg.h"
#endif /* STM32L475xx */

#ifdef AZURE_ENABLE_REGISTRATION

//#define DEBUG_REGISTRATION_AGENT

//#define REG_AGENTE_RELEASE

/* Imported variables --------------------------------------------------------*/
#if ((!defined(STM32L475xx)) && (!defined (STM32F429xx)))
extern uint8_t macaddstart[32];
#endif /* ((!defined(STM32L475xx)) && (!defined (STM32F429xx))) */
extern const char certificates[];

/* Local Defines */
#define SIZE_BUFFER_ANSWERSERVICE 1024

#define REGISTRATIONAGENT_DEFAULT_ENDPOINT_TCPPORT      443
#define AZURE_OFFSET_WEBSERVICE_ANSWER_BODY             8  // FOR "Token":"
#define AZURE_CONNSTRING_TOKEN_SIZE                     64  // TO BE CHECK MAX SIZE TOKEN FOR AZURE TOKEN. ALWAYS 44?
#define AZURE_FIXED_SIZE_CONNSTRING                     128 // Included margin
#define POST_CONTENT_SIZE			        128
#define HTTP_SEND_BUFFER_SIZE                           512
/* Default IoT HUB */
#define REGISTRATIONAGENT_DEFAULT_IOT_HUB               "STM32IoTHub.azure-devices.net"



/* STM32 Unique ID */
#ifdef USE_STM32F4XX_NUCLEO
#define STM32_UUID ((uint32_t *)0x1FFF7A10)
#endif /* USE_STM32F4XX_NUCLEO */

#if ((defined USE_STM32L4XX_NUCLEO) | (defined STM32L475xx))
#define STM32_UUID ((uint32_t *)0x1FFF7590)
#endif /* ((defined USE_STM32L4XX_NUCLEO) | (defined STM32L475xx)) */

#define AZURE1_REG_STATUS \
  REG_STATUS_NULL, \
  REG_STATUS_START, \
  REG_STATUS_RUNNIG, \
  REG_STATUS_END, \
  REG_STATUS_ERROR
DEFINE_ENUM(AZURE1_REG, AZURE1_REG_STATUS)

/* Local prototypes */
static void onSendCompleteReg(void* context, IO_SEND_RESULT send_result);
static void onIoErrorReg(void* context);
static void onCloseCompleteReg(void* context);
static void onBytesReceivedReg(void* context, const unsigned char* data_ptr, size_t size);


/* Local Variables */
static AZURE1_REG REGStatus=REG_STATUS_NULL;
static uint8_t *BufferServiceAnswer=NULL;
static uint32_t BufferServiceAnswerPointer=0;

/**
* @brief  function called after the end of GET/HEAD request for Registration
* @param  void* context (Not Used)
* @param  IO_SEND_RESULT send_result Results of the request (IO_SEND_OK/...)
* @retval None
*/
static void onSendCompleteReg(void* context, IO_SEND_RESULT send_result)
{  
  if (send_result != IO_SEND_OK){
    AZURE_PRINTF("onSendCompleteReg Err \r\n");
    REGStatus = REG_STATUS_ERROR;
  }
}

/**
* @brief  function called when there is one error
* @param  void* context (not Used)
* @retval None
*/
static void onIoErrorReg(void* context)
{  
  AZURE_PRINTF("Err: onIoErrorReg callback\r\n");
}

/** @brief  function called after the Socket close request  for Registration
* @param  void* context (not Used)
* @retval None
*/
static void onCloseCompleteReg(void* context)
{
  AZURE_PRINTF("onCloseCompleteReg callback\r\n");
}

/**
* @brief  function called when we are receiving data on the socket open for Registration
* @param  void* context (Not Used)
* @param  unsigned char* data_ptr pointer to the data received
* @param  size_t size size to the data received
* @retval None
*/
static void onBytesReceivedReg(void* context, const unsigned char* data_ptr, size_t size)
{
#ifdef DEBUG_REGISTRATION_AGENT
  AZURE_PRINTF("onBytesReceivedReg [%ld] <<%s>> \r\n",size,data_ptr);
#endif /* DEBUG_REGISTRATION_AGENT */
	memcpy(BufferServiceAnswer+BufferServiceAnswerPointer,data_ptr,size);
  BufferServiceAnswerPointer+=size;
  /* Control section */
  if(BufferServiceAnswerPointer>=SIZE_BUFFER_ANSWERSERVICE) {
    AZURE_PRINTF("\r\nonBytesReceivedReg Error... buffer overflow. RegistrationAgentInternal.c line %d. \r\n", __LINE__);
  }
		
  if( (size<64) || (strstr(data_ptr, "\" not found.")!=NULL) ) {
    REGStatus = REG_STATUS_END;
#ifdef DEBUG_REGISTRATION_AGENT
    AZURE_PRINTF("onBytesReceivedReg END trasmission reached...\r\n");
#endif /* DEBUG_REGISTRATION_AGENT */
  }
}


/**
* @brief  function called After the Socket open request  for Registration
* @param  void* context (not Used)
* @param  IO_OPEN_RESULT open_result Results of the open command (IO_OPEN_OK/...)
* @retval None
*/
static void onOpenCompleteReg(void* context, IO_OPEN_RESULT open_result)
{  
  if (open_result != IO_OPEN_OK){
    AZURE_PRINTF("onOpenCompleteReg Err\r\n"); 
  } else {
    AZURE_PRINTF("onOpenCompleteReg\r\n");
  }
}

/**
  * @brief  Register device and retrieve connection string  
  * @param  char **connectionString pointer to the Connection string
  * @retval int32_t value for success/failure (0/1)
  */ 
int32_t RegistrationAgent(char **connectionString)
{
  int32_t RetValue=0;
  uint8_t RegistrationDefaultEndPointIPAddress[36];

  RegistrationDefaultEndPointIPAddress[0 ] ='\0';
  if(RegistrationDefaultEndPointIPAddress[0]=='\0') {
    AZURE_PRINTF("Err Add a valid RegistrationDefaultEndPointIPAddress on %s\r\n",__FILE__);
    return 1;
  }
  
  /* Memory Allocation */
  BufferServiceAnswer  = (uint8_t *) malloc (SIZE_BUFFER_ANSWERSERVICE);
  if(BufferServiceAnswer==NULL) {
     AZURE_PRINTF("Err Allocating the BufferServiceAnswer\r\n");
     return 1;
  }

  (*connectionString) = malloc (sizeof(REGISTRATIONAGENT_DEFAULT_IOT_HUB)+AZURE_FIXED_SIZE_CONNSTRING);
  /* Control the pointer */
  if((*connectionString)==NULL) {
     AZURE_PRINTF("Err Allocating the Connection String\r\n");
     return 1;
  }

  {
    const IO_INTERFACE_DESCRIPTION* io_interface_description;
    XIO_HANDLE XIO_Instance;
    TLSIO_CONFIG tls_io_config;
    char UUID_Nucleo[32];
    char    buf[HTTP_SEND_BUFFER_SIZE];
    char MAC_RegisterdAddress[13];
    bool     b_do_register_board = false;
    int currentBoardType, registeredBoardType;
		unsigned int ui_timeout_answer = 0; // 10 sec

#ifdef STM32F401xE
    currentBoardType = 1;
#elif STM32L476xx
    currentBoardType = 2;
#elif STM32F429xx
    currentBoardType = 3;
#elif STM32L475xx
    currentBoardType = 4;
#endif /* STM32F401xE */
    AZURE_PRINTF("BoardType: %d\r\n", currentBoardType);

    /* Interface Description */
#ifdef STM32L475xx
    io_interface_description = tlsio_mbedtls_STM32Cube_get_interface_description();
#else /* STM32L475xx */
    io_interface_description = tlsio_mbedtls_get_interface_description();
#endif /* STM32L475xx */
    if (io_interface_description == NULL) {
      AZURE_PRINTF("Err: io_interface_description\r\n");
      return 1;
    } 
    AZURE_PRINTF("Ok io_interface_description\r\n");
    tls_io_config.hostname = (char *) RegistrationDefaultEndPointIPAddress;
    tls_io_config.port = REGISTRATIONAGENT_DEFAULT_ENDPOINT_TCPPORT;
    /* XIO_CREATE */
    XIO_Instance = xio_create(io_interface_description, &tls_io_config);
    if(XIO_Instance==NULL) {
      AZURE_PRINTF("Err: xio_create\r\n");
      return 1;
    }
    AZURE_PRINTF("Ok xio_create\r\n");

    /* XIO_SETOPTION */
    if(xio_setoption(XIO_Instance,  "TrustedCerts", certificates)!=0) {
      AZURE_PRINTF("Err: xio_setoption\r\n");
      return 1;
    }
    AZURE_PRINTF("Ok xio_setoption\r\n");
    
    /* XIO_OPEN */
    if(xio_open(XIO_Instance, onOpenCompleteReg, NULL, onBytesReceivedReg, NULL, onIoErrorReg, NULL)!=0) {
      AZURE_PRINTF("Err: xio_open\r\n");
      return 1;
    }
    AZURE_PRINTF("Ok xio_open\r\n");
    
    /* Board UID */
    sprintf (UUID_Nucleo,"%.10ld%.10ld%.10ld",STM32_UUID[0],STM32_UUID[1],STM32_UUID[2]);

#if defined(STM32L475xx) || defined (STM32F429xx)
    sprintf(MAC_RegisterdAddress,"%02x%02x%02x%02x%02x%02x",
         macAddress[0], macAddress[1], macAddress[2],
         macAddress[3], macAddress[4], macAddress[5]);
#else /* STM32L475xx || STM32F429xx */
    /* WIFI Mac Address removing the ":" from MAC like 00:80:E1:B8:87:1F */
    {
      int32_t Read;
      int32_t Write=0;
      for(Read=0;Read<17;Read++) {
        if(macaddstart[Read]!=':') {
          MAC_RegisterdAddress[Write] = macaddstart[Read];
          Write++;
        }
        MAC_RegisterdAddress[Write]='\0'; // Termination
      }
    }
#endif /* STM32L475xx */
    /* Compose the GET Message */
    sprintf(buf,"GET /api/tokens/%s HTTP/1.1\r\nHost: %s\r\n\r\n",MAC_RegisterdAddress,RegistrationDefaultEndPointIPAddress);
    /* Send the Request */
    REGStatus = REG_STATUS_START;
    BufferServiceAnswerPointer=0;
    if(xio_send(XIO_Instance, buf, strlen(buf),  onSendCompleteReg, NULL)!=0) {
      return 1;
    } else {
      AZURE_PRINTF("Ok xio_send GET Request\r\n");
    }   
    /* Wait the end of GET */
		ui_timeout_answer = 0;
    while((REGStatus != REG_STATUS_END) & (REGStatus != REG_STATUS_ERROR)) {
      xio_dowork(XIO_Instance);
      HAL_Delay(100);
			ui_timeout_answer++;
			// 10 sec
			if (ui_timeout_answer > 50){				
        AZURE_PRINTF("[WARNING Registration]: timeout expired after GET. RegistrationAgentInternal.c Line %d \r\n", __LINE__);
				break;
			}	
    }
		
    b_do_register_board = false;
    /* Retrieve and parse answer for GET-Credentials message */
    if(strstr((char *)(BufferServiceAnswer), "not found") != NULL) {
        // Register again
        b_do_register_board = true;
        AZURE_PRINTF("[Registration]. Device not registered. \r\n");
        memset(BufferServiceAnswer, 0, SIZE_BUFFER_ANSWERSERVICE);
    } else {  
      char * pBoardType;
      pBoardType = strstr((char *)(BufferServiceAnswer), "BoardType");
      /* Check for BoardType */  
      if(pBoardType == NULL) {
        // No board type in the answer : Cancel Registration then register again with boardType  
        // Delete registration 
        AZURE_PRINTF("[Registration]. Device already registered but no boardType in the answer. Request new registration. \r\n");
        memset(BufferServiceAnswer, 0, SIZE_BUFFER_ANSWERSERVICE);
        /* Compose the DELETE Message */
        sprintf(buf,"DELETE /api/tokens/?deviceId=%s HTTP/1.1\r\nHost: %s\r\n\r\n",MAC_RegisterdAddress,RegistrationDefaultEndPointIPAddress);
        /* Send the Request */
        REGStatus = REG_STATUS_START;
        BufferServiceAnswerPointer=0;
        if(xio_send(XIO_Instance, buf, strlen(buf),  onSendCompleteReg, NULL)!=0) {
          return 1;
        } else {
          AZURE_PRINTF("Ok xio_send DELETE Request\r\n");
        }        
        /* Wait the end of DELETE */
				ui_timeout_answer = 0;
        while((REGStatus != REG_STATUS_END) & (REGStatus != REG_STATUS_ERROR)) {
          xio_dowork(XIO_Instance);
          HAL_Delay(100);
  			  ui_timeout_answer++;
			    // 10 sec
			    if (ui_timeout_answer > 50){				
               AZURE_PRINTF("[WARNING Registration]: timeout expired after DELETE. RegistrationAgentInternal.c Line %d \r\n", __LINE__);
				       break;
			    }	
        }
        // Use the same pBoardType to check an error in the DELETE request
        pBoardType = strstr((char *)(BufferServiceAnswer), "204 No Content");
        if (pBoardType == NULL){
            AZURE_PRINTF("[Registration]. Error in delete response. \r\n");
            RetValue = 1;
            return RetValue;
        }
        else 
          AZURE_PRINTF("Delete OK. \r\n");
        
        // Register again
        memset(BufferServiceAnswer, 0, SIZE_BUFFER_ANSWERSERVICE);
        b_do_register_board = true;
      } else {
        // read boardType 
        // "BoardType": 3
        sscanf ( (pBoardType+11), "%d", &registeredBoardType);
        AZURE_PRINTF("The board was registered with boardType : %d \r\n", registeredBoardType);       
        
        // check boardType 
        if (registeredBoardType != currentBoardType){
           AZURE_PRINTF("[Registration]. This device was registered with a different boardType. Request new registration. \r\n");
           memset(BufferServiceAnswer, 0, SIZE_BUFFER_ANSWERSERVICE);
           // Delete registration 
          /* Compose the DELETE Message */
          sprintf(buf,"DELETE /api/tokens/?deviceId=%s HTTP/1.1\r\nHost: %s\r\n\r\n",MAC_RegisterdAddress,RegistrationDefaultEndPointIPAddress);
          /* Send the Request */
          REGStatus = REG_STATUS_START;
          BufferServiceAnswerPointer=0;
          if(xio_send(XIO_Instance, buf, strlen(buf),  onSendCompleteReg, NULL)!=0) {
            return 1;
          } else {
            AZURE_PRINTF("Ok xio_send DELETE Request\r\n");
          }        
          /* Wait the end of DELETE */
					ui_timeout_answer = 0;
          while((REGStatus != REG_STATUS_END) & (REGStatus != REG_STATUS_ERROR)) {
            xio_dowork(XIO_Instance);
            HAL_Delay(100);
						ui_timeout_answer++;
						// 10 sec
						if (ui_timeout_answer > 50){				
                 AZURE_PRINTF("[WARNING Registration]: timeout expired after DELETE. RegistrationAgentInternal.c Line %d \r\n", __LINE__);
								 break;
						}							
          }
          // Use the same pBoardType to check an error in the DELETE request
          pBoardType = strstr((char *)(BufferServiceAnswer), "204 No Content");
          if (pBoardType == NULL){
              AZURE_PRINTF("[Registration]. Error in delete response. \r\n");
              RetValue = 1;
              return RetValue;
          }
          else 
            AZURE_PRINTF("Delete OK. \r\n");  
          
           // Register again           
           memset(BufferServiceAnswer, 0, SIZE_BUFFER_ANSWERSERVICE);
           b_do_register_board = true;  
        }
        else {
          AZURE_PRINTF("[Registration]. This device was registered with the correct board type. \r\n");   
        }
        
      }
    } 
      
    if (b_do_register_board == false ) {
      char * pConnString;
      char connToken[AZURE_CONNSTRING_TOKEN_SIZE];
      int i = 0;

      AZURE_PRINTF("[Registration]. Retrieving connection string... \r\n");

      pConnString = strstr((char *)(BufferServiceAnswer), "Token");

      if ( pConnString != NULL){
        pConnString += AZURE_OFFSET_WEBSERVICE_ANSWER_BODY;
        while ( *pConnString != '"' ){
          connToken[i] = *pConnString;
          pConnString++;
          i++;
          if (i >= AZURE_CONNSTRING_TOKEN_SIZE){
							AZURE_PRINTF("[Registration]. Connection string token not found! \r\n");
							RetValue = 1;
							return RetValue;								 
					 }						 
        }
        connToken[i] = '\0';

  #ifdef DEBUG_REGISTRATION_AGENT
        AZURE_PRINTF("[Registration]. HostName is is [%s] \r\n", REGISTRATIONAGENT_DEFAULT_IOT_HUB);
        AZURE_PRINTF("[Registration]. DeviceId is is [%s] \r\n", MAC_RegisterdAddress);
        AZURE_PRINTF("[Registration]. Token is is [%s] \r\n", connToken);
  #endif /* DEBUG_REGISTRATION_AGENT */

        sprintf(*connectionString,"HostName=%s;DeviceId=%s;SharedAccessKey=%s",REGISTRATIONAGENT_DEFAULT_IOT_HUB, MAC_RegisterdAddress, connToken );
  #ifdef DEBUG_REGISTRATION_AGENT
        AZURE_PRINTF("[Registration]. Connection string is [%s] \r\n", *connectionString);
  #endif /* DEBUG_REGISTRATION_AGENT */
        
        AZURE_PRINTF("[Registration]. Retrieved connection string. \r\n");

      } else {
        AZURE_PRINTF("[Registration]. Err retriving the connection string\r\n");
        RetValue =1;
      }

      if(xio_close(XIO_Instance, onCloseCompleteReg, NULL)!=0) {
        AZURE_PRINTF("Err: xio_close\r\n");
        return 1;
      } else {
        AZURE_PRINTF("Ok xio_close\r\n");
      }

      xio_destroy(XIO_Instance);
      AZURE_PRINTF("Ok xio_destroy\r\n");
      
    } else {
      // Register board
      char * pConnString;
      char connToken[AZURE_CONNSTRING_TOKEN_SIZE];
      int i = 0;
      char postContent[POST_CONTENT_SIZE];
      AZURE_PRINTF("[Registration]. Device not yet registerd. Sending registration request... \r\n");
       /* Compose the POST Message */
       sprintf(postContent,"{\"id\": \"%s+%s\", \"boardType\": %d}", 
               UUID_Nucleo, 
               MAC_RegisterdAddress,   
               currentBoardType
                 );
       
       
#ifdef REG_AGENTE_RELEASE
      //Release
      sprintf(buf,"POST /api/tokens/ HTTP/1.1\r\n"
              "Host: %s\r\n"
              "Content-Length: %d\r\n"
              "Content-Type: application/json\r\n\r\n"
              "%s\r\n",
              RegistrationDefaultEndPointIPAddress,
              strlen(postContent),
              postContent);
#else /* REG_AGENTE_RELEASE */
      //Demo
      sprintf(buf,"POST /api/tokens/?demo=true HTTP/1.1\r\n"
              "Host: %s\r\nContent-Length: %d\r\n"
              "Content-Type: application/json\r\n\r\n"
              "%s\r\n",
              RegistrationDefaultEndPointIPAddress,
              strlen(postContent),
              postContent);
#endif /* REG_AGENTE_RELEASE */
   
      /* Send the Request */
      REGStatus = REG_STATUS_START;
      BufferServiceAnswerPointer=0;
      if(xio_send(XIO_Instance, buf, strlen(buf),  onSendCompleteReg, NULL)!=0) {
        return 1;
      } else {
        AZURE_PRINTF("Ok xio_send POST Request\r\n");
      }

      /* Wait the end of POST */
			ui_timeout_answer = 0;
      while((REGStatus != REG_STATUS_END) & (REGStatus != REG_STATUS_ERROR)) {
        xio_dowork(XIO_Instance);
        HAL_Delay(100);
			  ui_timeout_answer++;
				if (ui_timeout_answer > 50){				
                 AZURE_PRINTF("[WARNING Registration]: timeout expired after POST. RegistrationAgentInternal.c Line %d \r\n", __LINE__);
								 break;
				}							
      }

      pConnString = strstr((char *)(BufferServiceAnswer), "Token");
      
      if ( pConnString != NULL){
        pConnString += AZURE_OFFSET_WEBSERVICE_ANSWER_BODY;
        while ( *pConnString != '"' ){
          connToken[i] = *pConnString;
          pConnString++;
          i++;
          if (i >= AZURE_CONNSTRING_TOKEN_SIZE){
							AZURE_PRINTF("[Registration]. Connection string token not found! \r\n");
							RetValue = 1;
							return RetValue;
						}
        }
        connToken[i] = '\0';
        sprintf(*connectionString,"HostName=%s;DeviceId=%s;SharedAccessKey=%s",REGISTRATIONAGENT_DEFAULT_IOT_HUB, MAC_RegisterdAddress, connToken );
#ifdef DEBUG_REGISTRATION_AGENT
        AZURE_PRINTF("[Registration]. Token is is %s \r\n", connToken);
        AZURE_PRINTF("[Registration]. Connection string is %s \r\n", *connectionString);
#endif /* DEBUG_REGISTRATION_AGENT */
        AZURE_PRINTF("[Registration]. Registered device and retrieved connection string. \r\n");
      } else {
        AZURE_PRINTF("[Registration]. Err retriving the connection string\r\n");
        RetValue = 1;
      }
      
      if(xio_close(XIO_Instance, onCloseCompleteReg, NULL)!=0) {
        AZURE_PRINTF("Err: xio_close\r\n");
        return 1;
      } else {
        AZURE_PRINTF("Ok xio_close\r\n");
      }

      xio_destroy(XIO_Instance);
      AZURE_PRINTF("Ok xio_destroy\r\n");
    }
  }

  free(BufferServiceAnswer);

  /* If Everything ok */
  if(RetValue==0) {
 #ifdef DEBUG_REGISTRATION_AGENT
    AZURE_PRINTF("[Registration]. Device Regsitration to Microsoft Azure Successfully Completed\r\n");
    AZURE_PRINTF("[Registration]. Connection String=\r\n\t%s\r\n",*connectionString);
 #endif   
  }
  return RetValue;
}
#endif /* AZURE_ENABLE_REGISTRATION */
