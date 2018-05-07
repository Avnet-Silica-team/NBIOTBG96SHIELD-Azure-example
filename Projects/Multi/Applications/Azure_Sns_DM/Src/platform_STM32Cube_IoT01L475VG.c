/**
 ******************************************************************************
 * @file    platform_STM32Cube_IoT01L475VG.c
 * @author  Central LAB
  * @version V3.2.2
  * @date    22-Jan-2018
 * @brief   Platform specific adapter
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

#include <stdlib.h>
#ifdef _CRTDBG_MAP_ALLOC
#include <crtdbg.h>
#endif
#include "STM32CubeInterface.h"
#include "MetaDataManager.h"
#include "azure_c_shared_utility/crt_abstractions.h"
#include "azure_c_shared_utility/platform.h"
#include "OTA.h"
#include "AzureClient_mqtt_DM_TM.h"

#include "azure_c_shared_utility/tls_config.h"
#include "azure_c_shared_utility/tlsio_mbedtls.h"
#include "azure_c_shared_utility/tlsio.h"

#include "tlsio_mbedtls_STM32Cube_iot01l475vg.h"
#include "net.h"

/* Exported variables ---------------------------------------------------------*/
TargetFeatures_t TargetBoardFeatures;
UART_HandleTypeDef UartHandle;

/* Socket with mbedTLS ... first for Telemetry, Second one for FOTA */
uint8_t SocketHandleAzure[2]={0xFF,0xFF};

TIM_HandleTypeDef    TimCCHandle;

volatile uint32_t ButtonPressed =0;
volatile uint32_t SendData =0;

uint8_t macAddress[6];

#if defined (__CC_ARM)
/* For avoiding Keil error */
MDM_knownOsxLicense_t known_OsxLic[]={
  {OSX_END,"LAST",""}/* THIS MUST BE THE LAST ONE */
};
#endif /* (__CC_ARM) */

net_hnd_t hnet;

/* Imported variables --------------------------------------------------------*/
extern const char certificates[];
#ifndef AZURE_ENABLE_REGISTRATION
extern char *connectionString;
#endif /* AZURE_ENABLE_REGISTRATION */

/* Local variables ---------------------------------------------------------*/

static uint32_t FullOTASize=0;

static AZURE1_OTA OTAStatus=OTA_STATUS_NULL;

DEFINE_ENUM_STRINGS(AZURE1_ERROR, AZURE1_ERROR_VALUES)

#define NTP_ENDPOINT_IPADDRESS \
        NTP_DEFAULT, \
        NTP_BACKUP1, \
        NTP_BACKUP2
DEFINE_ENUM(NTP_ENDPOINT, NTP_ENDPOINT_IPADDRESS)

#ifndef AZURE_ENABLE_REGISTRATION
  /**
   * @brief  Azure's Connection String Structure
   */
  #define AZURE_MAX_CONN_STRING 256
  typedef struct
  {
    char ConnectionString[AZURE_MAX_CONN_STRING];
  } AzureConnection_t;

AzureConnection_t AzureConnection;
#endif /* AZURE_ENABLE_REGISTRATION */

static WIFI_CredAcc_t WIFI_CredAcc;

/* Local defines -------------------------------------------------------------*/
//#define DEBUG_OTA_RECEIVE

//2kHz/0.5 For Sensors Data data@0.5Hz
#define DEFAULT_TIM_CC1_PULSE  4000

/* Defines related to Clock configuration */    
#define RTC_ASYNCH_PREDIV  0x7F   /* LSE as RTC clock */
#define RTC_SYNCH_PREDIV   0x00FF /* LSE as RTC clock */

#define HTTP_GET_REQUEST 0
#define HTTP_HEAD_REQUEST 1

#define FOTA_CHUNK_SIZE 1024

/* Local function prototypes --------------------------------------------------*/
static void STM32_Error_Handler(void);
static void Init_MEM1_Sensors(void);
static void SystemClock_Config(void);
static void InitTimers(void);
static void InitRTC(void);
#ifdef USE_STM32L4XX_NUCLEO
static uint32_t GetPage(uint32_t Address);
static uint32_t GetBank(uint32_t Address);
#endif /* USE_STM32L4XX_NUCLEO */

static void onSendCompleteFOTA(void* context, IO_SEND_RESULT send_result);
static void onOpenCompleteFOTA(void* context, IO_OPEN_RESULT open_result);
static void onBytesReceivedFOTA(void* context, const unsigned char* buffer, size_t size);
static void onIoErrorFOTA(void* context);
static void onCloseCompleteOTA(void* context);

static void UART_Console_Init(void);
static int net_if_init(void * if_ctxt);

/**
  * @brief  Function for Initializing the platform
  * @param  None
  * @retval int Ok/Error (0/1)
  */
int platform_init(void)
{
  int32_t CounterButtonPress=0;

  /* Init Platform */
  HAL_Init();

  /* Configure the System clock */
  SystemClock_Config();

#ifdef AZURE_ENABLE_PRINTF
  UART_Console_Init();
  AZURE_PRINTF("UART Initialized\r\n");
#endif /* AZURE_ENABLE_PRINTF */


  /* Initialize button */
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Initialize LED */
  BSP_LED_Init(LED2);

  AZURE_PRINTF("\r\nSTMicroelectronics %s:\r\n"
         "\tVersion %c.%c.%c\r\n"
        "\tB-L475E-IOT01 IoT node Discovery board"
          "\r\n",
          AZURE_PACKAGENAME,
          AZURE_VERSION_MAJOR,AZURE_VERSION_MINOR,AZURE_VERSION_PATCH);

  AZURE_PRINTF("\tAzure SDK Version %s\r\n",IOTHUB_SDK_VERSION);

  /* Reset all the Target's Features */
  memset(&TargetBoardFeatures, 0, sizeof(TargetFeatures_t));
  /* Discovery and Intialize all the Target's Features */
  Init_MEM1_Sensors();

  AZURE_PRINTF("\t(HAL %ld.%ld.%ld_%ld)\r\n"
        "\tCompiled %s %s"
#if defined (__IAR_SYSTEMS_ICC__)
        " (IAR)\r\n"
#elif defined (__CC_ARM)
        " (KEIL)\r\n"
#elif defined (__GNUC__)
        " (openstm32)\r\n"
#endif
           ,
           HAL_GetHalVersion() >>24,
          (HAL_GetHalVersion() >>16)&0xFF,
          (HAL_GetHalVersion() >> 8)&0xFF,
           HAL_GetHalVersion()      &0xFF,
         __DATE__,__TIME__);

  AZURE_PRINTF("\tOTA with one HTTP HEAD + Multiple HTTP one GET of %dBytes\r\n",FOTA_CHUNK_SIZE);

  /* Check the BootLoader Compliance */
  if(CheckBootLoaderCompliance()) {
    AZURE_PRINTF("BootLoader Compliant with FOTA procedure\r\n");
  } else {
    AZURE_PRINTF("Err: BootLoader NOT Compliant with FOTA procedure\r\n");
    AzureExit(AZURE_ERR_BL_COMPLIANCE);
  }

  /* Check the NFC */
  if(TT4_Init() == SUCCESS) {
    TargetBoardFeatures.NFCInitialized = 1;
  }

  /* initialize timers */
  InitTimers();
  AZURE_PRINTF("Init Application's Timers\r\n");

  /* Initialize random number generartion */
#ifdef USE_MBED_TLS
  TargetBoardFeatures.RngHandle.Instance = RNG;
  if(HAL_RNG_Init(&TargetBoardFeatures.RngHandle) != HAL_OK) {
    AZURE_PRINTF("Err Initializing Random Number Generator\r\n");
  } else {
    AZURE_PRINTF("Init Random Number Generator\r\n");
  }
#endif /* USE_MBED_TLS*/

  /* Set the WIFI Credential and Connection String */
  {
    char console_input[128];
    int32_t NecessityToSaveMMD=0;
    MDM_knownGMD_t known_MetaData[]={
      {GMD_WIFI,sizeof(WIFI_CredAcc_t)},
#ifndef AZURE_ENABLE_REGISTRATION
      {GMD_AZURE,sizeof(AzureConnection_t)},
#endif /* AZURE_ENABLE_REGISTRATION */
      {GMD_END    ,0}/* THIS MUST BE THE LAST ONE */
    };

    AZURE_PRINTF("--------------------------\r\n");
    AZURE_PRINTF("|    WIFI Credential     |\r\n");
    AZURE_PRINTF("--------------------------\r\n");
    
    /* Initialize the MetaDataManager */
    InitMetaDataManager((void *)&known_MetaData,MDM_DATA_TYPE_GMD,NULL);

    /* Recall the WIFI Credential saved */
    memset(&WIFI_CredAcc,0,sizeof(WIFI_CredAcc_t));
    MDM_ReCallGMD(GMD_WIFI,(void *)&WIFI_CredAcc);

    if(WIFI_CredAcc.ssid[0]==0) {
      sprintf(WIFI_CredAcc.ssid,"%s",AZURE_DEFAULT_SSID);
      sprintf(WIFI_CredAcc.seckey,"%s",AZURE_DEFAULT_SECKEY);
      WIFI_CredAcc.mode = AZURE_DEFAULT_PRIV_MODE;
      AZURE_PRINTF("\tDefault SSID   : %s\r\n",WIFI_CredAcc.ssid);
      AZURE_PRINTF("\tDefault PassWd : %s\r\n",WIFI_CredAcc.seckey);
      AZURE_PRINTF("\tDefault EncMode: %s\r\n",(WIFI_CredAcc.mode == None) ? "Open" : ((WIFI_CredAcc.mode == WEP) ? "WEP" : "WPA2/WPA2-Personal"));
    } else {
      AZURE_PRINTF("\tSaved SSID   : %s\r\n",WIFI_CredAcc.ssid);
      AZURE_PRINTF("\tSaved PassWd : %s\r\n",WIFI_CredAcc.seckey);
      AZURE_PRINTF("\tSaved EncMode: %s\r\n",(WIFI_CredAcc.mode == None) ? "Open" : ((WIFI_CredAcc.mode == WEP) ? "WEP" : "WPA2/WPA2-Personal"));
    }

    AZURE_PRINTF("Wait 3 seconds for allowing User Button Control for changing it\r\n");
    {
      int32_t CountOn,CountOff;
      for(CountOn=0;CountOn<3;CountOn++) {
        BSP_LED_On(LED2);
        for(CountOff=0;CountOff<10;CountOff++) {
          if(CountOff==2) {
            BSP_LED_Off(LED2);
          }
          /* Handle user button */
          if(ButtonPressed) {
            CounterButtonPress++;
            ButtonPressed=0;
            goto INSERT_LABEL1;
          }
          HAL_Delay(100);
        }
      }
    }
INSERT_LABEL1:
    if(CounterButtonPress) {
      /* Enabling UART interaction */
      AZURE_PRINTF("\r\nDo you want to change them?(y/n)\r\n");
      scanf("%s",console_input);
      if(console_input[0]=='y') {
        int32_t ReadFromTerminal=1;

        /* If the NFC is present */
        if(TargetBoardFeatures.NFCInitialized) {
          AZURE_PRINTF("\tDo you want to read them from NFC?(y/n)\r\n");
          scanf("%s",console_input);
          if(console_input[0]=='y') {
            sWifiTokenInfo TokenInfo;
            if(TT4_ReadWifiToken(&TokenInfo) == SUCCESS) {
              ReadFromTerminal=0;

              sprintf(WIFI_CredAcc.ssid,"%s",TokenInfo.NetworkSSID);
              sprintf(WIFI_CredAcc.seckey,"%s",TokenInfo.NetworkKey);

              if (strcmp(TokenInfo.AuthenticationType,"NONE")==0) {
                WIFI_CredAcc.mode = None;
              } else if(strcmp(TokenInfo.AuthenticationType,"WEP")==0) {
                WIFI_CredAcc.mode = WEP;
              } else {
                WIFI_CredAcc.mode = WPA_Personal;
              }
            } else {
              AZURE_PRINTF("Err reading the WIFI's credentials from NFC\r\n");
              AZURE_PRINTF("Add them manually\r\n");
            }
          }
        }

        if(ReadFromTerminal==1) {
          AZURE_PRINTF("\r\nEnter the SSID:\r\n");
          scanf("%s",console_input);
          sprintf(WIFI_CredAcc.ssid,"%s",console_input);
          AZURE_PRINTF("\r\nEnter the PassWd:\r\n");
          scanf("%s",console_input);
          sprintf(WIFI_CredAcc.seckey,"%s",console_input);
          AZURE_PRINTF("\r\nEnter the encryption mode(0:Open, 1:WEP, 2:WPA2/WPA2-Personal):\r\n");
          scanf("%s",console_input);
          switch(console_input[0]){
            case '0':
              WIFI_CredAcc.mode = None;
            break;
            case '1':
              WIFI_CredAcc.mode = WEP;
            break;
            case '2':
              WIFI_CredAcc.mode = WPA_Personal;
            break;
            default:
              AZURE_PRINTF("\r\nWrong Entry. Priv Mode is not compatible\r\n");
              AzureExit(AZURE_ERR_WIFI_CRED);
          }
        }
        AZURE_PRINTF("\tNew SSID   : %s\r\n",WIFI_CredAcc.ssid);
        AZURE_PRINTF("\tNew PassWd : %s\r\n",WIFI_CredAcc.seckey);
        AZURE_PRINTF("\tNew EncMode: %s\r\n",(WIFI_CredAcc.mode == None) ? "Open" : ((WIFI_CredAcc.mode == WEP) ? "WEP" : "WPA2/WPA2-Personal"));

        /* Save the WIFI Credential on MetaDataManager */
        MDM_SaveGMD(GMD_WIFI,(void *)&WIFI_CredAcc);

        /* Will be Necessary Save the Meta Data Manager in Flash */
        NecessityToSaveMMD=1;

        AZURE_PRINTF("-----------------------------\r\n");
      }
      AZURE_PRINTF("\r\n");
    }

#ifndef AZURE_ENABLE_REGISTRATION
    AZURE_PRINTF("--------------------------\r\n");
    AZURE_PRINTF("|   Connection String    |\r\n");
    AZURE_PRINTF("--------------------------\r\n");

    connectionString = AzureConnection.ConnectionString;

    /* Recall Connection String from Meta Data Manager */
    memset(&AzureConnection,0,sizeof(AzureConnection_t));
    MDM_ReCallGMD(GMD_AZURE,(void *)&AzureConnection);
    {
      int32_t ReadFromTerminal=1;
      CounterButtonPress=0;

      if(AzureConnection.ConnectionString[0]!=0) {
        AZURE_PRINTF("\tSaved Connection String :\r\n");
        AZURE_PRINTF("\t\t%s\r\n",AzureConnection.ConnectionString);
#ifdef AZUREDEVICECONNECTIONSTRING
      } else {
        sprintf(AzureConnection.ConnectionString,"%s", (char *) AZUREDEVICECONNECTIONSTRING);
        AZURE_PRINTF("\tDefault Connection String  : %s\r\n", AzureConnection.ConnectionString);
#endif /* AZUREDEVICECONNECTIONSTRING */
      }

      if(AzureConnection.ConnectionString[0]!=0) {
        AZURE_PRINTF("Wait 3 seconds for allowing User Button Control for changing it\r\n");
        {
          int32_t CountOn,CountOff;
          for(CountOn=0;CountOn<3;CountOn++) {
            BSP_LED_On(LED2);
            for(CountOff=0;CountOff<10;CountOff++) {
              if(CountOff==2) {
                BSP_LED_Off(LED2);
              }
              /* Handle user button */
              if(ButtonPressed) {
                CounterButtonPress++;
                ButtonPressed=0;
                goto INSERT_LABEL2;
              }
              HAL_Delay(100);
            }
          }
        }
INSERT_LABEL2:
        if(CounterButtonPress) {
          AZURE_PRINTF("\tDo you want to change it?(y/n)\r\n");
          scanf("%s",console_input);
          if(console_input[0]!='y') {
            /* If we don't want to change it */
            ReadFromTerminal=0;
          } else {
            /* If the NFC is present */
            if(TargetBoardFeatures.NFCInitialized) {
              AZURE_PRINTF("\tDo you want to read them from NFC?(y/n)\r\n");
              scanf("%s",console_input);
              if(console_input[0]=='y') {
                NDEF_Text_info_t TextInfo;
                if(TT4_ReadTextToken(&TextInfo) == SUCCESS) {
                  ReadFromTerminal=0;
                  sprintf(AzureConnection.ConnectionString,"%s",TextInfo.text);
                  NecessityToSaveMMD=1;
                  MDM_SaveGMD(GMD_AZURE,(void *)&AzureConnection);
                  AZURE_PRINTF("connectionString from NFC=\r\n\t%s\r\n",connectionString);
                } else {
                  AZURE_PRINTF("Err reading the Connection String from NFC\r\n");
                  AZURE_PRINTF("Add them manually\r\n");
                }
              }
            }
          }
        } else {
          ReadFromTerminal=0;
        }
      } else {
        AZURE_PRINTF("Azure Connection String not present\r\n");
        /* If the NFC is present */
        if(TargetBoardFeatures.NFCInitialized) {
          AZURE_PRINTF("\tDo you want to read them from NFC?(y/n)\r\n");
          scanf("%s",console_input);
          if(console_input[0]=='y') {
            NDEF_Text_info_t TextInfo;
            if(TT4_ReadTextToken(&TextInfo) == SUCCESS) {
              ReadFromTerminal=0;
              sprintf(AzureConnection.ConnectionString,"%s",TextInfo.text);
              NecessityToSaveMMD=1;
              MDM_SaveGMD(GMD_AZURE,(void *)&AzureConnection);
              AZURE_PRINTF("connectionString from NFC=\r\n\t%s\r\n",connectionString);
            } else {
              AZURE_PRINTF("Err reading the Connection String from NFC\r\n");
              AZURE_PRINTF("Add them manually\r\n");
            }
          }
        }
      }

      /* if we need to read the connection String from UART */
      if(ReadFromTerminal) {
        AZURE_PRINTF("Enter the Azure Connection String:\r\n");
        scanf("%s",AzureConnection.ConnectionString);
        NecessityToSaveMMD=1;
        MDM_SaveGMD(GMD_AZURE,(void *)&AzureConnection);
        AZURE_PRINTF("\r\n");
      }
    }

#endif /* AZURE_ENABLE_REGISTRATION */
    
    /* Save the MetaDataManager in Flash if it's necessary */
    if(NecessityToSaveMMD) {
      EraseMetaDataManager();
      SaveMetaDataManager();
    }
  }

  /* Initialize the WIFI module */
  if (net_init(&hnet, NET_IF_WLAN, (net_if_init)) != NET_OK) {
    AzureExit(AZURE_ERR_WIFI);
  }
  
  /* initialize Real Time Clock */
  InitRTC();
  /* Check if Data stored in BackUp register0: No Need to reconfigure RTC */
  while(HAL_RTCEx_BKUPRead(&TargetBoardFeatures.RtcHandle, RTC_BKP_DR0) != 0x32F2){
    /* Configure RTC Calendar */
    char hostname[16]; // "time-d.nist.gov";
    uint8_t tcpPort  =  NTP_ENDPOINT_TCPPORT;
    net_sockhnd_t socket;
    uint8_t NumRentryNTP=0;
    static NTP_ENDPOINT NTP_Server=NTP_DEFAULT;
    switch(NTP_Server) {
      case NTP_DEFAULT:
        strcpy(hostname, NTP_ENDPOINT_IPADDRESS_DEFAULT);
      break;
      case NTP_BACKUP1:
        strcpy(hostname, NTP_ENDPOINT_IPADDRESS_BACKUP1);
      break;
      case NTP_BACKUP2:
        strcpy(hostname, NTP_ENDPOINT_IPADDRESS_BACKUP2);
      break;
      default:
        AzureExit(AZURE_ERR_NTP);
    }

    AZURE_PRINTF("Contact NTP server %s \r\n", hostname);

    if(net_sock_create(hnet, &socket, NET_PROTO_TCP)!=NET_OK) {
      AZURE_PRINTF("Err creating socket for NTP server\r\n");
    } else {
      AZURE_PRINTF("WiFi created socket for NTP server\r\n");
      if(net_sock_open(socket, hostname, tcpPort)!= NET_OK) {
        AZURE_PRINTF("Err opening socket with NTP server\r\n");
      } else {
        uint8_t buf[4];
        AZURE_PRINTF("WiFi opened socket with NTP server\r\n");
        if(net_sock_recv(socket, buf, 4)!=4) {
          AZURE_PRINTF("Err Failed to retrieve data from NTP server \r\n");
        } else {
          time_t epochTimeToSetForSystem = SynchronizationAgentConvertNTPTime2EpochTime(buf,4);
          if (TimingSystemSetSystemTime(epochTimeToSetForSystem)== 0){
            AZURE_PRINTF("Err Failed to set system time. \r\n");
          } else {
            AZURE_PRINTF("Set UTC Time: %s\r\n",(get_ctime(&epochTimeToSetForSystem)));
          }
        }
      }
    }

    if(HAL_RTCEx_BKUPRead(&TargetBoardFeatures.RtcHandle, RTC_BKP_DR0) != 0x32F2) {
      // NTP server cannot be queried more frequently than once every 4 seconds
      AZURE_PRINTF("I'll try again to connect to NTP server in 4 seconds\r\n");
      HAL_Delay(4000);
      NumRentryNTP++;
      if (NumRentryNTP == NTP_MAX_RETRIES){
        NumRentryNTP = 0;

        switch(NTP_Server) {
          case NTP_DEFAULT:
            AZURE_PRINTF("NTP backup 1\r\n");
            NTP_Server = NTP_BACKUP1;
          break;
          case NTP_BACKUP1:
            AZURE_PRINTF("NTP backup 2\r\n");
            NTP_Server = NTP_BACKUP2;
          break;
          case NTP_BACKUP2:
            AZURE_PRINTF("NTP Default\r\n");
            NTP_Server = NTP_DEFAULT;
          break;
          default:
            AzureExit(AZURE_ERR_NTP);
        }
      }
    }
    if(net_sock_close(socket)!=NET_OK) {
      AZURE_PRINTF("Err closing the socket with NTP server\r\n");
    } else {
      AZURE_PRINTF("WIFI closed socket with NTP server\r\n");
    }
    if(net_sock_destroy(socket)!=NET_OK) {
      AZURE_PRINTF("Err destroying the socket for NTP server\r\n");
    } else {
      AZURE_PRINTF("WIFI destroied socket for NTP server\r\n");
    }
  }
  /* RTC Is ok */
  {
    uint8_t aShowTime[50] = {0};
    uint8_t aShowDate[50] = {0};
    RTC_CalendarShow(aShowTime,aShowDate);
    AZURE_PRINTF("Real Time Clock Intialized: %s %s\r\n",aShowDate,aShowTime);
  }
  return 0;
}

/**
  * @brief  Function for initializing the WIFI module
  * @param  void *if_ctxt pointer to the context
  * @retval None
  */
static int net_if_init(void * if_ctxt)
{
  char moduleinfo[ES_WIFI_MAX_SSID_NAME_SIZE];
  WIFI_Ecn_t security_mode = (WIFI_CredAcc.mode == None) ? WIFI_ECN_OPEN : ((WIFI_CredAcc.mode == WEP) ? WIFI_ECN_WEP : WIFI_ECN_WPA2_PSK);
  int32_t wifiConnectCounter=1;

  if(WIFI_Init()!=WIFI_STATUS_OK) {
    AZURE_PRINTF("Failed to initialize WIFI module\r\n");
    AzureExit(AZURE_ERR_WIFI);
  } else {
    AZURE_PRINTF("WIFI module initialized\r\n");
  }

  /* Retrieve the WiFi module mac address to confirm that it is detected and communicating. */
  WIFI_GetModuleName(moduleinfo);
  AZURE_PRINTF("Module initialized successfully: %s\r\n",moduleinfo);

  WIFI_GetModuleID(moduleinfo);
  AZURE_PRINTF("\t%s\r\n",moduleinfo);

  WIFI_GetModuleFwRevision(moduleinfo);
  AZURE_PRINTF("\t%s\r\n",moduleinfo);

  if(WIFI_GetMAC_Address( (uint8_t*)macAddress)!=WIFI_STATUS_OK) {
    AZURE_PRINTF("Failed to get MAC address\r\n");
    AzureExit(AZURE_ERR_WIFI);
  } else {
    AZURE_PRINTF("WiFi MAC Address is: %02x:%02x:%02x:%02x:%02x:%02x\r\n",
         macAddress[0], macAddress[1], macAddress[2],
         macAddress[3], macAddress[4], macAddress[5]);
  }

  /* Connect to the specified SSID. */
  AZURE_PRINTF("Trying to Connect WIFI to AP\r\n");
  while(WIFI_Connect(WIFI_CredAcc.ssid, WIFI_CredAcc.seckey, security_mode) != WIFI_STATUS_OK) {
    AZURE_PRINTF("WiFi not able Connected to AP Attempt = %ld\r\n",wifiConnectCounter++);
    HAL_Delay(1000);
    /* Max number of attempts */
    if(wifiConnectCounter == WIFI_CONNECT_MAX_ATTEMPT_COUNT) {
	  AZURE_PRINTF("WiFi not able to connected to AP %s\r\n",WIFI_CredAcc.ssid);
	  AzureExit(AZURE_ERR_WIFI);
    }
  }
  AZURE_PRINTF("WiFi Connected to AP %s\r\n",WIFI_CredAcc.ssid);

  /* Until is ok */
  return 0;
}

/**
  * @brief  Function for signaling Error
  * @param  AZURE1_ERROR Value Error Type
  * @retval None
  */
void AzureExit(AZURE1_ERROR Value)
{
  AZURE_PRINTF("\r\nErr Type %s\r\n",ENUM_TO_STRING(AZURE1_ERROR,Value));
  AZURE_PRINTF("\tNecessity to restart the Board\r\n");
  /* Infinite led blinking */
  BSP_LED_Off(LED2);
  while(1) {
    int32_t Counter;
    /* Blinking Error Code... */
    for(Counter=0;Counter<Value;Counter++) {
      BSP_LED_On(LED2);
      HAL_Delay(200);
      BSP_LED_Off(LED2);
      HAL_Delay(400);
    }
    HAL_Delay(4000);
  }
}

/**
  * @brief  Function for starting the timer for sending the Telemetry data to IoT
  * @param  None
  * @retval None
  */
void StartTimer1(void) {
  /* Starting timer */
  if(HAL_TIM_OC_Start_IT(&TimCCHandle, TIM_CHANNEL_1) != HAL_OK){
    /* Starting Error */
    STM32_Error_Handler();
  }

  /* Set the new Capture compare value */
  {
    uint32_t uhCapture = __HAL_TIM_GET_COUNTER(&TimCCHandle);
    /* Set the Capture Compare Register value */
    __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_1, (uhCapture + TargetBoardFeatures.TIM_CC1_Pulse));
  }

  AZURE_PRINTF("Channel 1 for Timer 1 started\r\n");
}

/**
  * @brief  Function for pausing the timer for sending the Telemetry data to IoT
  * @param  None
  * @retval None
  */
void StopTimer1(void) {
  /* Stop timer */
  if(HAL_TIM_OC_Stop_IT(&TimCCHandle, TIM_CHANNEL_1) != HAL_OK){
    /* Starting Error */
    STM32_Error_Handler();
  }
  AZURE_PRINTF("Channel 1 for Timer 1 stopped\r\n");
}


/** @brief function for providing the default TLS I/O interface
 * @param None
 * @retval IO_INTERFACE_DESCRIPTION* Pointer to the default TLS I/O interface
 */
const IO_INTERFACE_DESCRIPTION* platform_get_default_tlsio(void)
{
  return tlsio_mbedtls_STM32Cube_get_interface_description();
}

/** @brief Function for de-initializing the platform
 * @param None
 * @retval None
 */
void platform_deinit(void)
{
  /* TODO: Insert here any platform specific one time de-initialization code like.
  This has to undo what has been done in platform_init. */
}


/** @brief Initialize all the MEMS1 sensors
 * @param None
 * @retval None
 */
static void Init_MEM1_Sensors(void)
{
  /* Accelero */
  if (ACCELERO_OK != BSP_ACCELERO_Init()) {
   AZURE_PRINTF("Err Accelero Sensor\n\r");
  } else {
    AZURE_PRINTF("Ok Accelero Sensor\n\r");
  }

  /* Gyro */
  if (GYRO_OK != BSP_GYRO_Init()) {
   AZURE_PRINTF("Err Gyroscope Sensor\n\r");
  } else {
    AZURE_PRINTF("Ok Gyroscope Sensor\n\r");
  }

  /* Mag */
  if (MAGNETO_OK != BSP_MAGNETO_Init()) {
   AZURE_PRINTF("Err Magneto Sensor\n\r");
  } else {
    AZURE_PRINTF("Ok Magneto Sensor\n\r");
  } 

  /* Humidity */
  if (HSENSOR_OK != BSP_HSENSOR_Init()) {
   AZURE_PRINTF("Err Humidity Sensor\n\r");
  } else {
    AZURE_PRINTF("Ok Humidity Sensor\n\r");
  }

  /* Temperature */
  if (TSENSOR_OK != BSP_TSENSOR_Init()) {
   AZURE_PRINTF("Err Temperature Sensor\n\r");
  } else {
    AZURE_PRINTF("Ok Temperature Sensor\n\r");
  }

  /* Pressure */  
  if (PSENSOR_OK != BSP_PSENSOR_Init()) {
   AZURE_PRINTF("Err Pressure Sensor\n\r");
  } else {
    AZURE_PRINTF("Ok Pressure Sensor\n\r");
  }
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (MSI)
  *            SYSCLK(Hz)                     = 80000000
  *            HCLK(Hz)                       = 80000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            MSI Frequency(Hz)              = 48000000
  *            PLL_M                          = 6
  *            PLL_N                          = 20
  *            PLL_R                          = 2
  *            PLL_P                          = 7
  *            PLL_Q                          = 2
  *            Flash Latency(WS)              = 4
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;
  
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    /* Initialization Error */
    while(1);
  }
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
  clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    /* Initialization Error */
    STM32_Error_Handler();
  }
  
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USART1
                                       |RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_I2C2
                                       |RCC_PERIPHCLK_RNG;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.RngClockSelection = RCC_RNGCLKSOURCE_MSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    /* Initialization Error */
   STM32_Error_Handler();
  }
  
  __HAL_RCC_PWR_CLK_ENABLE();
  
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    /* Initialization Error */
    STM32_Error_Handler();
  }
  
  /* Enable MSI PLL mode */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief  Output Compare callback in non blocking mode
  * @param  htim : TIM OC handle
  * @retval None
  */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
  uint32_t uhCapture=0;

  /* TIM1_CH1 toggling with frequency = 2Hz */
  if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
    uhCapture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    /* Set the Capture Compare Register value */
    __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_1, (uhCapture + TargetBoardFeatures.TIM_CC1_Pulse));
    SendData=1;
  }
}


/**
* @brief  Function for initializing timers for sending the Telemetry data to IoT hub
 * @param  None
 * @retval None
 */
static void InitTimers(void)
{
  uint32_t uwPrescalerValue;

  /* Timer Output Compare Configuration Structure declaration */
  TIM_OC_InitTypeDef sConfig;

  /* Compute the prescaler value to have TIM3 counter clock equal to 2 KHz */
  uwPrescalerValue = (uint32_t) ((SystemCoreClock / 2000) - 1);

  /* Set TIM1 instance (Motion)*/
  TimCCHandle.Instance = TIM1;
  TimCCHandle.Init.Period        = 65535;
  TimCCHandle.Init.Prescaler     = uwPrescalerValue;
  TimCCHandle.Init.ClockDivision = 0;
  TimCCHandle.Init.CounterMode   = TIM_COUNTERMODE_UP;
  if(HAL_TIM_OC_Init(&TimCCHandle) != HAL_OK) {
    /* Initialization Error */
    STM32_Error_Handler();
  }

 /* Configure the Output Compare channels */
 /* Common configuration for all channels */
  sConfig.OCMode     = TIM_OCMODE_TOGGLE;
  sConfig.OCPolarity = TIM_OCPOLARITY_LOW;

  /* Output Compare Toggle Mode configuration: Channel1 */
  TargetBoardFeatures.TIM_CC1_Pulse = sConfig.Pulse = DEFAULT_TIM_CC1_PULSE;
  if(HAL_TIM_OC_ConfigChannel(&TimCCHandle, &sConfig, TIM_CHANNEL_1) != HAL_OK) {
    /* Configuration Error */
    STM32_Error_Handler();
  }
}


/**
* @brief  Function for initializing the Real Time Clock
 * @param  None
 * @retval None
 */
static void InitRTC(void)
{
  RTC_HandleTypeDef *RtcHandle = &TargetBoardFeatures.RtcHandle;

  /* Configure RTC prescaler and RTC data registers */
  /* RTC configured as follow:
      - Hour Format    = Format 24
      - Asynch Prediv  = Value according to source clock
      - Synch Prediv   = Value according to source clock
      - OutPut         = Output Disable
      - OutPutPolarity = High Polarity
      - OutPutType     = Open Drain
  */

  RtcHandle->Instance = RTC;
  RtcHandle->Init.HourFormat = RTC_HOURFORMAT_24;
  RtcHandle->Init.AsynchPrediv = RTC_ASYNCH_PREDIV;
  RtcHandle->Init.SynchPrediv = RTC_SYNCH_PREDIV;
  RtcHandle->Init.OutPut = RTC_OUTPUT_DISABLE;
  RtcHandle->Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  RtcHandle->Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;

  if(HAL_RTC_Init(RtcHandle) != HAL_OK) {
    STM32_Error_Handler();
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void STM32_Error_Handler(void)
{
  /* User may add here some code to deal with this error */
  while(1){
  }
}

/**
 * @brief  EXTI line detection callback.
 * @param  uint16_t GPIO_Pin Specifies the pins connected EXTI line
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch(GPIO_Pin){
  case USER_BUTTON_PIN:
    ButtonPressed = 1;
    break;
  case NFC_GPIO_GPO_PIN:
      M24SR_GPO_Callback();
    break;
  }
}


/**
 * @brief  Configures UART interface
 * @param  None
 * @retval None
 */
static void UART_Console_Init(void)
{ 
  UartHandle.Instance = USART1;
  UartHandle.Init.BaudRate = CFG_HW_UART1_BAUDRATE;
  UartHandle.Init.WordLength = CFG_HW_UART1_WORDLENGTH;
  UartHandle.Init.StopBits = CFG_HW_UART1_STOPBITS;
  UartHandle.Init.Parity = CFG_HW_UART1_PARITY;
  UartHandle.Init.Mode = CFG_HW_UART1_MODE;
  UartHandle.Init.HwFlowCtl = CFG_HW_UART1_HWFLOWCTL;  
  UartHandle.AdvancedInit.AdvFeatureInit = CFG_HW_UART1_ADVFEATUREINIT;
  BSP_COM_Init(COM1,&UartHandle);
}


/**
* @brief  function called after the end of GET/HEAD request for FOTA
* @param  void* context type of request (GET/HEAD)
* @param  IO_SEND_RESULT send_result Results of the request (IO_SEND_OK/...)
* @retval None
*/
static void onSendCompleteFOTA(void* context, IO_SEND_RESULT send_result)
{
  AZURE_PRINTF("onSendCompleteFOTA\r\n");
}


/**
* @brief  function called After the Socket open request  for FOTA
* @param  void* context (not Used)
* @param  IO_OPEN_RESULT open_result Results of the open command (IO_OPEN_OK/...)
* @retval None
*/
static void onOpenCompleteFOTA(void* context, IO_OPEN_RESULT open_result)
{  
  AZURE_PRINTF("onOpenCompleteFOTA\r\n");
}

/**
* @brief  function called when we are receiving data on the socket open for FOTA
* @param  void* context type of request (GET/HEAD)
* @param  unsigned char* data_ptr pointer to the data received
* @param  size_t size size to the data received
* @retval None
*/
static void onBytesReceivedFOTA(void* context, const unsigned char* data_ptr, size_t size)
{
   int32_t *TypeOfRequest = (int32_t *)context; 
#if ((defined USE_STM32L4XX_NUCLEO) | (defined USE_STM32L475E_IOT01))
  static int32_t PaddingBytes=0;
#endif /* ((defined USE_STM32L4XX_NUCLEO) | (defined USE_STM32L475E_IOT01)) */

  if((*TypeOfRequest) == HTTP_HEAD_REQUEST) {
    /* for the HEAD Request */
    uint8_t *StartAddress=NULL;
    OTAStatus = OTA_STATUS_RUNNIG;
#ifdef DEBUG_OTA_RECEIVE
    AZURE_PRINTF("message from HEAD Request Size=%d\r\n",size);
#endif /* DEBUG_OTA_RECEIVE */

    /* Make the parsing of the header */
    if(FullOTASize==0) {
      /* Search the String "Content-Range: bytes" */
      StartAddress = (uint8_t *) strstr( (char *) data_ptr, "Content-Range: bytes");
      if(StartAddress!=NULL) {
          StartAddress = (uint8_t *) strstr((char *) StartAddress, "/");
          if(StartAddress!=NULL) {
            StartAddress++;
            FullOTASize = atoi((char *) StartAddress);
            AZURE_PRINTF("(Content-Range:) Full OTA size=%ld\r\n",FullOTASize);
          } else {
            AZURE_PRINTF("Err... retriving the Full OTA Size...(Content-Range: bytes)\r\n");
            OTAStatus = OTA_STATUS_ERROR;
            return;
          }
      } else {
        /* Search the String "Content-Lenght:" */
        StartAddress = (uint8_t *) strstr( (char *) data_ptr, "Content-Length:");
        if(StartAddress!=NULL) {
          StartAddress+=16; // for moving to the end of "Content-Length: "
          FullOTASize = atoi((char *) StartAddress);
          AZURE_PRINTF("(Content-Length:) Full OTA size=%ld\r\n",FullOTASize);
        } else {
          AZURE_PRINTF("Err... retriving the Full OTA Size...(Content-Length:)\r\n");
          OTAStatus = OTA_STATUS_ERROR;
          return;
        }
      }
      /* Padding the OTA size for L476 Board */
#if ((defined USE_STM32L4XX_NUCLEO) | (defined USE_STM32L475E_IOT01))
      if(FullOTASize&0x7) {
        PaddingBytes = 8 - (FullOTASize&0x7);
        AZURE_PRINTF("\tPaddingBytes=%ld\r\n",PaddingBytes);
        FullOTASize += PaddingBytes;
        AZURE_PRINTF("\tOTA Round=%ld\r\n",FullOTASize);
      }
#endif /* ((defined USE_STM32L4XX_NUCLEO) | (defined USE_STM32L475E_IOT01)) */
    } else {
#ifdef DEBUG_OTA_RECEIVE
      AZURE_PRINTF("Consuming remaining bytes from HEAD request\r\n");
#endif /* DEBUG_OTA_RECEIVE */
      /* Search the end of HEAD request... */
      StartAddress = (uint8_t *) strstr( (char *) data_ptr, "\r\n\r\n");      
      if(StartAddress!=NULL) {
        OTAStatus = OTA_STATUS_END;
      }
    }
  } else {
    /* for the GET Request */
    uint8_t *StartAddress=NULL;
    static uint32_t SizeOfUpdate=0;
    uint32_t RealMessageSize=0;
#if ((defined USE_STM32L4XX_NUCLEO) | (defined USE_STM32L475E_IOT01))
    static uint64_t ValueToWrite=0;
    static uint8_t *Pointer= (uint8_t *) &ValueToWrite;
    static int32_t PointerCounter= 0;
    int32_t counter;
#endif /* ((defined USE_STM32L4XX_NUCLEO) | (defined USE_STM32L475E_IOT01)) */

#define MCR_OTA_DOUBLE_WORLD(size)                  \
{                                                   \
  for(counter=0;counter<(size);counter++) {         \
    Pointer[PointerCounter] = StartAddress[counter];\
    PointerCounter++;                               \
    /* Write the Double Word */                     \
    if(PointerCounter==8) {                         \
      UpdateFWBlueMS(&SizeOfUpdate,Pointer, 8,1);   \
      PointerCounter=0;                             \
      ValueToWrite=0;                               \
    }                                               \
  }                                                 \
}
    
    if(SizeOfUpdate==0) {
      /* Search the Content-Length: */
     StartAddress = (uint8_t *) strstr( (char *) data_ptr, "Content-Length:");
      if(StartAddress!=NULL) {
        StartAddress+=16; // for moving to the end of "Content-Length: "
        SizeOfUpdate = atoi((char *) StartAddress);
#ifdef DEBUG_OTA_RECEIVE
        AZURE_PRINTF("Chunck OTA size=%ld\r\n",SizeOfUpdate);
#endif /* DEBUG_OTA_RECEIVE */
      } else {
#ifdef DEBUG_OTA_RECEIVE
        AZURE_PRINTF("Starting OTA not found.. move to next chunk\r\n");
#endif /* DEBUG_OTA_RECEIVE */
      }
    }

    /* If we had found the Chunk OTA Size */
    if(SizeOfUpdate!=0) {
      if(OTAStatus==OTA_STATUS_START) {
        /* Search the real Start Postion skipping the Banner */
        StartAddress = (uint8_t *) strstr( (char *) data_ptr, "\r\n\r\n");
        if(StartAddress!=NULL) {
          StartAddress += 4;
          RealMessageSize= size - (StartAddress - data_ptr);
#ifdef DEBUG_OTA_RECEIVE
          AZURE_PRINTF("RealMessageSize=%d\r\n",RealMessageSize);
#endif /* DEBUG_OTA_RECEIVE */

          OTAStatus=OTA_STATUS_RUNNIG;

          MCR_OTA_DOUBLE_WORLD(RealMessageSize);
#if ((defined USE_STM32L4XX_NUCLEO) | (defined USE_STM32L475E_IOT01))
          if(PaddingBytes) {
            if(ReadRemSizeOfUpdate()==8) {
              /* Flash the Padding bytes */
              AZURE_PRINTF("Flashing the last chunk of 8 bytes\r\n");
              SizeOfUpdate+=PaddingBytes; /* This for the next if(SizeOfUpdate==0) condition */
              UpdateFWBlueMS(&SizeOfUpdate,Pointer, 8,1);
              PointerCounter=0; /* This could be avoided */
              ValueToWrite=0;   /* This could be avoided */
            }
          }
#endif /* ((defined USE_STM32L4XX_NUCLEO) | (defined USE_STM32L475E_IOT01)) */

          /* Control if we are at the end... */
          if(SizeOfUpdate==0) {
            OTAStatus = OTA_STATUS_END;
#ifdef DEBUG_OTA_RECEIVE
            AZURE_PRINTF("A OTA_STATUS_END\r\n");
          } else {
            AZURE_PRINTF("A Remaining Size of Update=%d\r\n",SizeOfUpdate);
#endif /* DEBUG_OTA_RECEIVE */
          }
        } else {
#ifdef DEBUG_OTA_RECEIVE
          AZURE_PRINTF("Starting OTA not found.. move to next chunk\r\n");
#endif /* DEBUG_OTA_RECEIVE */
        }
      } else {
        StartAddress = (unsigned char *)data_ptr;
#ifdef DEBUG_OTA_RECEIVE
        AZURE_PRINTF("New Message=%d\r\n",size);
#endif /* DEBUG_OTA_RECEIVE */

        MCR_OTA_DOUBLE_WORLD(size);

#if ((defined USE_STM32L4XX_NUCLEO) | (defined USE_STM32L475E_IOT01))
        if(PaddingBytes) {
          if(ReadRemSizeOfUpdate()==8) {
            /* Flash the Padding bytes */
            AZURE_PRINTF("Flashing the last chunk of 8 bytes\r\n");
            SizeOfUpdate+=PaddingBytes; /* This for the next if(SizeOfUpdate==0) condition */
            UpdateFWBlueMS(&SizeOfUpdate,Pointer, 8,1);
            PointerCounter=0; /* This could be avoided */
            ValueToWrite=0;   /* This could be avoided */
          }
        }
#endif /* ((defined USE_STM32L4XX_NUCLEO) | (defined USE_STM32L475E_IOT01)) */

        /* Control if we are at the end... */
        if(SizeOfUpdate==0) {
          OTAStatus = OTA_STATUS_END;
#ifdef DEBUG_OTA_RECEIVE
          AZURE_PRINTF("B OTA_STATUS_END\r\n");
        } else {
          AZURE_PRINTF("B Remaining Size of Update=%d\r\n",SizeOfUpdate);
#endif /* DEBUG_OTA_RECEIVE */
        }
      }
    }
  }
}

/**
* @brief  function called when there is one error
* @param  void* context (not Used)
* @retval None
*/
static void onIoErrorFOTA(void* context)
{  
  AZURE_PRINTF("Err: onIoErrorFOTA callback\r\n");
}

/** @brief  function called after the Socket close request  for FOTA
* @param  void* context (not Used)
* @retval None
*/
static void onCloseCompleteOTA(void* context)
{  
  AZURE_PRINTF("onCloseCompleteOTA callback\r\n");
}

/**
  * @brief  Execute the Firmware update after the Command FOTA
  * @param  char *hostname Server address
  * @param  char type 's'/'t'->Secure/NotSecure socket
  * @param  uint32_t port_num Server port number
  * @param  char *path FOTA path
  * @retval AZURE1_OTA FOTA Status Error/Success -> OTA_STATUS_ERROR/OTA_STATUS_NULL
  */
AZURE1_OTA FOTACallback(char * hostname,char type, uint32_t  port_num,char * path)
{
  char    buf[256];
  const IO_INTERFACE_DESCRIPTION* io_interface_description;
  XIO_HANDLE XIO_Instance;
  TLSIO_CONFIG tls_io_config;
  int32_t TypeOfRequest;
  int32_t ChunkNum;

  AZURE_PRINTF("Download FOTA from: HostName=[%s] Type=[%s] port=[%ld] File=[%s]\r\n",hostname,(type=='s') ? "Secure": "NotSecure", port_num,path);

  ReportState(Downloading);
  /* This because we want to be sure that this message reach the IOT hub */
  WaitAllTheMessages();

  /* Interface Description */
  io_interface_description = tlsio_mbedtls_STM32Cube_get_interface_description();
  if (io_interface_description == NULL) {
    AZURE_PRINTF("Err: io_interface_description\r\n");
    return OTA_STATUS_ERROR;
  }    
  AZURE_PRINTF("Ok io_interface_description\r\n");

  tls_io_config.hostname = hostname;
  tls_io_config.port = port_num;
  /* XIO_CREATE */
  XIO_Instance = xio_create(io_interface_description, &tls_io_config);
  if(XIO_Instance==NULL) {
    AZURE_PRINTF("Err: xio_create\r\n");
    OTAStatus = OTA_STATUS_ERROR;
    goto FOTA_exit;
  }
  AZURE_PRINTF("Ok xio_create\r\n");

  /* XIO_SETOPTION */
  if(xio_setoption(XIO_Instance,  "TrustedCerts", certificates)!=0) {
    AZURE_PRINTF("Err: xio_setoption\r\n");
    OTAStatus = OTA_STATUS_ERROR;
    goto FOTA_exit;
  }
  AZURE_PRINTF("Ok xio_setoption\r\n");

  /* XIO_OPEN */
  OTAStatus=OTA_STATUS_NULL;
  if(xio_open(XIO_Instance, onOpenCompleteFOTA, NULL, onBytesReceivedFOTA, &TypeOfRequest, onIoErrorFOTA, NULL)!=0) {
    AZURE_PRINTF("Err: xio_open\r\n");
    OTAStatus = OTA_STATUS_ERROR;
    goto FOTA_exit;
  }
  AZURE_PRINTF("Ok xio_open\r\n");
  
  /* XIO_SEND HEAD Request  */
  OTAStatus=OTA_STATUS_START;
  TypeOfRequest= HTTP_HEAD_REQUEST;
  sprintf(buf,"HEAD %s HTTP/1.1\r\nHost: %s\r\nRange: bytes=0-1\r\n\r\n",path,hostname);
  if(xio_send(XIO_Instance, buf, strlen(buf),  onSendCompleteFOTA, &TypeOfRequest)!=0) {
    AZURE_PRINTF("Err: xio_send\r\n");
    OTAStatus = OTA_STATUS_ERROR;
    goto FOTA_exit;
  } else {
    AZURE_PRINTF("Ok xio_send HEAD Request\r\n");
  }

  /* Wait the end of HEAD */
  while((OTAStatus != OTA_STATUS_END) & (OTAStatus != OTA_STATUS_ERROR)) {
    xio_dowork(XIO_Instance);
    HAL_Delay(100);
  }

  if(OTAStatus == OTA_STATUS_ERROR) {
    goto FOTA_exit;
  }

  /* Clean the Flash before the Real HTTP GET */
  StartUpdateFWBlueMS(FullOTASize, 0);
  for(ChunkNum=0;(ChunkNum*FOTA_CHUNK_SIZE)<FullOTASize;ChunkNum++) {

    if((ChunkNum%0x3F)==0) {
      /* For avoiding that the wifi module closes the Socket with the IoT hub */
      ReportState(Downloading);
      /* This because we want to be sure that this message reach the IOT hub */
      WaitAllTheMessages();
    }

    /* XIO_SEND GET Request  */
    OTAStatus = OTA_STATUS_START;
    TypeOfRequest= HTTP_GET_REQUEST;


    /* Prepare Request */
    if(((ChunkNum+1)*FOTA_CHUNK_SIZE-1)<FullOTASize) {
      sprintf(buf,"GET %s HTTP/1.1\r\nHost: %s\r\nRange: bytes=%ld-%ld\r\n\r\n",path,hostname,ChunkNum*FOTA_CHUNK_SIZE,(ChunkNum+1)*FOTA_CHUNK_SIZE-1);
    } else {
      sprintf(buf,"GET %s HTTP/1.1\r\nHost: %s\r\nRange: bytes=%ld-%ld\r\n\r\n",path,hostname,ChunkNum*FOTA_CHUNK_SIZE,FullOTASize);
    }

    if(xio_send(XIO_Instance, buf, strlen(buf),  onSendCompleteFOTA, &TypeOfRequest)!=0) {
      AZURE_PRINTF("Err: xio_send\r\n");
      OTAStatus = OTA_STATUS_ERROR;
      goto FOTA_exit;
    } else {
      AZURE_PRINTF("Ok xio_send GET (%03ld/%03ld) Request\r\n",ChunkNum,(FullOTASize+FOTA_CHUNK_SIZE-1)/FOTA_CHUNK_SIZE-1);
    }

    /* Wait the end of Chuck or Error */
    while((OTAStatus != OTA_STATUS_END) & (OTAStatus != OTA_STATUS_ERROR)) {
      xio_dowork(XIO_Instance);
      HAL_Delay(100);
    }

    if(OTAStatus == OTA_STATUS_ERROR) {
      goto FOTA_exit;
    } else {
      OTAStatus = OTA_STATUS_NULL;
    }
  }

  if(xio_close(XIO_Instance, onCloseCompleteOTA, NULL)!=0) {
    AZURE_PRINTF("Err: xio_close\r\n");
    OTAStatus = OTA_STATUS_ERROR;
    goto FOTA_exit;
  } else {
    AZURE_PRINTF("Ok xio_close\r\n");
  }

  xio_destroy(XIO_Instance);
  AZURE_PRINTF("Ok xio_destroy\r\n");

FOTA_exit:
  if(OTAStatus == OTA_STATUS_NULL) {
    /* Everything was ok */
    AZURE_PRINTF("OTA Downloaded\r\n");
    ReportState(DownloadComplete);
    WaitAllTheMessages();
    return OTAStatus;
  } else {
    /* The socket was closed before the end of FOTA trasmission... */
    AZURE_PRINTF("\r\nErr During FOTA... Sorry retry...\r\n");
    return OTA_STATUS_ERROR;
  }
}


/**
  * @brief This function provides accurate delay (in milliseconds) based
  *        on variable incremented.
  * @note This is a user implementation using WFI state
  * @param Delay specifies the delay time length, in milliseconds.
  * @retval None
  */
void HAL_Delay(__IO uint32_t Delay)
{
  uint32_t tickstart = 0;
  tickstart = HAL_GetTick();
  while((HAL_GetTick() - tickstart) < Delay){
    __WFI();
  }
}

/**
  * @brief  Definition of ThreadAPI_Sleep
  * @param milliseconds: delay in millesconds
  */
void ThreadAPI_Sleep(unsigned int milliseconds)
{
  HAL_Delay(milliseconds);
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: AZURE_PRINTF("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1){
  }
}
#endif

#if ((defined USE_STM32L4XX_NUCLEO ) || (defined STM32L475xx))
/**
  * @brief  Gets the page of a given address
  * @param  Addr: Address of the FLASH Memory
  * @retval The page of a given address
  */
static uint32_t GetPage(uint32_t Addr)
{
  uint32_t page = 0;

  if (Addr < (FLASH_BASE + FLASH_BANK_SIZE)) {
    /* Bank 1 */
    page = (Addr - FLASH_BASE) / FLASH_PAGE_SIZE;
  } else {
    /* Bank 2 */
    page = (Addr - (FLASH_BASE + FLASH_BANK_SIZE)) / FLASH_PAGE_SIZE;
  }
  return page;
}

/**
  * @brief  Gets the bank of a given address
  * @param  Addr: Address of the FLASH Memory
  * @retval The bank of a given address
  */
static uint32_t GetBank(uint32_t Addr)
{
  uint32_t bank = 0;

  if (READ_BIT(SYSCFG->MEMRMP, SYSCFG_MEMRMP_FB_MODE) == 0){
    /* No Bank swap */
    if (Addr < (FLASH_BASE + FLASH_BANK_SIZE)) {
      bank = FLASH_BANK_1;
    } else {
      bank = FLASH_BANK_2;
    }
  } else {
    /* Bank swap */
    if (Addr < (FLASH_BASE + FLASH_BANK_SIZE)) {
      bank = FLASH_BANK_2;
    } else {
      bank = FLASH_BANK_1;
    }
  }
  return bank;
}

/**
 * @brief User function for Erasing the MDM on Flash
 * @param None
 * @retval uint32_t Success/NotSuccess [1/0]
 */
uint32_t UserFunctionForErasingFlash(void) {
  FLASH_EraseInitTypeDef EraseInitStruct;
  uint32_t SectorError = 0;
  uint32_t Success=1;

  EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.Banks       = GetBank(MDM_FLASH_ADD);
  EraseInitStruct.Page        = GetPage(MDM_FLASH_ADD);
  EraseInitStruct.NbPages     = 2;

  /* Unlock the Flash to enable the flash control register access *************/
  HAL_FLASH_Unlock();

  if(HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK){
    /* Error occurred while sector erase. 
      User can add here some code to deal with this error. 
      SectorError will contain the faulty sector and then to know the code error on this sector,
      user can call function 'HAL_FLASH_GetError()'
      FLASH_ErrorTypeDef errorcode = HAL_FLASH_GetError(); */
    Success=0;
    STM32_Error_Handler();
  }

  /* Lock the Flash to disable the flash control register access (recommended
  to protect the FLASH memory against possible unwanted operation) *********/
  HAL_FLASH_Lock();

  return Success;
}

/**
 * @brief User function for Saving the MDM  on the Flash
 * @param void *InitMetaDataVector Pointer to the MDM beginning
 * @param void *EndMetaDataVector Pointer to the MDM end
 * @retval uint32_t Success/NotSuccess [1/0]
 */
uint32_t UserFunctionForSavingFlash(void *InitMetaDataVector,void *EndMetaDataVector)
{
  uint32_t Success=1;

  /* Store in Flash Memory */
  uint32_t Address = MDM_FLASH_ADD;
  uint64_t *WriteIndex;

  /* Unlock the Flash to enable the flash control register access *************/
  HAL_FLASH_Unlock();
  for(WriteIndex =((uint64_t *) InitMetaDataVector); WriteIndex<((uint64_t *) EndMetaDataVector); WriteIndex++) {
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, Address,*WriteIndex) == HAL_OK){
      Address = Address + 8;
    } else {
      /* Error occurred while writing data in Flash memory.
         User can add here some code to deal with this error
         FLASH_ErrorTypeDef errorcode = HAL_FLASH_GetError(); */
      STM32_Error_Handler();
      Success =0;
    }
  }

  /* Lock the Flash to disable the flash control register access (recommended
   to protect the FLASH memory against possible unwanted operation) *********/
  HAL_FLASH_Lock();
 
  return Success;
}
#endif /* ((defined USE_STM32L4XX_NUCLEO ) || (defined STM32L475xx)) */

STRING_HANDLE platform_get_platform_info(void)
{
  return STRING_construct("(STM32Cube)");
}
/******************* (C) COPYRIGHT 2017 STMicroelectronics *****END OF FILE****/
