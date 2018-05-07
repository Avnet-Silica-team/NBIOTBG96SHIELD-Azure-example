/**
 ******************************************************************************
 * @file    platform_STM32Cube_NucleoF401RE.c
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
#include "TLocalBuffer.h"
#include "AzureClient_mqtt_TM.h"

#include "BG96_interface.h"
#include "stm32_BG96_iot.h"

#include "azure_c_shared_utility/tls_config.h"
#include "azure_c_shared_utility/tlsio_mbedtls.h"
#include "azure_c_shared_utility/tlsio.h"

/* M24SR GPIO mapping -------------------------------------------------------------------------*/
#define M24SR_GPO_PIN        GPIO_PIN_6
#define M24SR_GPO_PIN_PORT   GPIOA
#define M24SR_RFDIS_PIN      GPIO_PIN_7
#define M24SR_RFDIS_PIN_PORT GPIOA

/* Exported variables ---------------------------------------------------------*/
TargetFeatures_t TargetBoardFeatures;

TIM_HandleTypeDef    TimCCHandle;

volatile uint32_t ButtonPressed =0;
volatile uint32_t MEMSInterrupt =0;
volatile uint32_t SendData =0;

uint8_t macaddstart[32];

#if defined (__CC_ARM)
/* For avoiding Keil error */
MDM_knownOsxLicense_t known_OsxLic[]={
  {OSX_END,"LAST",""}/* THIS MUST BE THE LAST ONE */
};
#endif /* (__CC_ARM) */

/* Imported variables --------------------------------------------------------*/
extern const char certificates[];
#ifndef AZURE_ENABLE_REGISTRATION
extern char *connectionString;
#endif /* AZURE_ENABLE_REGISTRATION */

/* Local variables ---------------------------------------------------------*/

static __IO BG96_state_t GSM_state;

#define AZURE1_NTP_STATUS \
  NTP_STATUS_NULL, \
  NTP_STATUS_CONNECTED, \
  NTP_STATUS_CLOSED
DEFINE_ENUM(AZURE1_NTP, AZURE1_NTP_STATUS)
static AZURE1_NTP NTP_Status=NTP_STATUS_NULL;

#define NTP_ENDPOINT_IPADDRESS \
        NTP_DEFAULT, \
        NTP_BACKUP1, \
        NTP_BACKUP2
DEFINE_ENUM(NTP_ENDPOINT, NTP_ENDPOINT_IPADDRESS)

DEFINE_ENUM_STRINGS(AZURE1_ERROR, AZURE1_ERROR_VALUES)
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
 
/* Local defines -------------------------------------------------------------*/

/* Defines related to Clock configuration */    
#define RTC_ASYNCH_PREDIV  0x7F   /* LSE as RTC clock */
#define RTC_SYNCH_PREDIV   0x00FF /* LSE as RTC clock */

//2kHz/0.5 For Sensors Data data@0.5Hz
#define DEFAULT_TIM_CC1_PULSE  10000					///means 5 seconds

/* Local function prototypes --------------------------------------------------*/
static void STM32_Error_Handler(void);
static void Init_MEM1_Sensors(void);
static void SystemClock_Config(void);
static void InitTimers(void);
static void InitRTC(void);

/**
  * @brief  Function for Initializing the platform
  * @param  None
  * @retval int Ok/Error (0/1)
  */
int platform_init(void)
{
  //WIFI_CredAcc_t WIFI_CredAcc;
	BG96_CredAcc_t BG96_CredAcc;
  //wifi_config config;
  int32_t CounterButtonPress=0;
  BG96_Status_t status = BG96_MODULE_SUCCESS;

  /* Init Platform */
  HAL_Init();

  /* Configure the System clock */
  SystemClock_Config();

#ifdef AZURE_ENABLE_PRINTF
  /* UART Initialization */
  if(UART_Global_Init()!=HAL_OK) {
    STM32_Error_Handler();
  } else {
    AZURE_PRINTF("UART Initialized\r\n");
  }
#endif /* AZURE_ENABLE_PRINTF */
  /* I2C Initialization */
  if(I2C_Global_Init()!=HAL_OK) {
    STM32_Error_Handler();
  } else {
    AZURE_PRINTF("I2C  Initialized\r\n");
  }

  /* Initialize button */
  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);

  /* Initialize LED */
  BSP_LED_Init(LED2);

  AZURE_PRINTF("\r\nSTMicroelectronics %s:\r\n"
         "\tVersion %c.%c.%c\r\n"
        "\tSTM32F401RE-Nucleo board"
          "\r\n",
          AZURE_PACKAGENAME,
          AZURE_VERSION_MAJOR,AZURE_VERSION_MINOR,AZURE_VERSION_PATCH);

  AZURE_PRINTF("\tAzure SDK Version %s\r\n",IOTHUB_SDK_VERSION);
	
	AZURE_PRINTF("\tBG96 IoT Shield demo version\r\n");

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

  if(TargetBoardFeatures.HWAdvanceFeatures) {
    InitHWFeatures();
  }
  
  /* Set Full Scale to +/-2g */
  (TargetBoardFeatures.SnsAltFunc ? BSP_ACCELERO_Set_FS_Value_IKS01A2 : BSP_ACCELERO_Set_FS_Value)(TargetBoardFeatures.HandleAccSensor,2.0f);

  /* initialize timers */
  InitTimers();
  AZURE_PRINTF("Init Application's Timers\r\n");

  /* Initialize random number generartion */
#ifdef USE_MBED_TLS
  srand(HAL_GetTick());
  AZURE_PRINTF("Init Random Number Generator\r\n");
#endif /* USE_MBED_TLS*/
    
  /* Enabling HW Features... FreeFall */
  EnableHWFreeFall();

  /* Check if NFC is present */
  if(TT4_Init() == SUCCESS) {
    AZURE_PRINTF("X-NUCLEO-NFC01A1 is present\r\n");
    TargetBoardFeatures.NFCInitialized = 1;
  }

  /* Initializes Timers for GSM */
  BG96_Timer_Config();
  AZURE_PRINTF("Init BG96's Timers\r\n");

  /* Initializes UART1 and GPIO pins for GSM module BG96 */
  BG96_Configuration();
  AZURE_PRINTF("Init BG96's UART & GPIO\r\n");

  /* Set the WIFI Credential and Connection String */
  {
    char console_input[128];
    int32_t NecessityToSaveMMD=0;
    MDM_knownGMD_t known_MetaData[]={
      {GMD_BG96,sizeof(BG96_CredAcc_t)},
#ifndef AZURE_ENABLE_REGISTRATION
      {GMD_AZURE,sizeof(AzureConnection_t)},
#endif /* AZURE_ENABLE_REGISTRATION */
      {GMD_END    ,0}/* THIS MUST BE THE LAST ONE */
    };

    AZURE_PRINTF("--------------------------\r\n");
    AZURE_PRINTF("|    META Data info       |\r\n");
    AZURE_PRINTF("--------------------------\r\n");		
		
		    /* Initialize the MetaDataManager */
    InitMetaDataManager((void *)&known_MetaData,MDM_DATA_TYPE_GMD,NULL);

    AZURE_PRINTF("--------------------------\r\n");
    AZURE_PRINTF("|    GSM Credential       |\r\n");
    AZURE_PRINTF("--------------------------\r\n");
    
    /* Recall the WIFI Credential saved */
    memset(&BG96_CredAcc,0,sizeof(BG96_CredAcc_t));
    MDM_ReCallGMD(GMD_BG96,(void *)&BG96_CredAcc);

    if(BG96_CredAcc.sim_pin[0]==0) {
      sprintf(BG96_CredAcc.sim_pin,"%s",AZURE_DEFAULT_SIMPIN);
      sprintf(BG96_CredAcc.sim_apn,"%s",AZURE_DEFAULT_APN);
      //WIFI_CredAcc.mode = AZURE_DEFAULT_PRIV_MODE;
      AZURE_PRINTF("\tDefault SIM PIN   : %s\r\n",BG96_CredAcc.sim_pin);
      AZURE_PRINTF("\tDefault APN : %s\r\n",BG96_CredAcc.sim_apn);
      //AZURE_PRINTF("\tDefault EncMode: %s\r\n",(WIFI_CredAcc.mode == None) ? "Open" : ((WIFI_CredAcc.mode == WEP) ? "WEP" : "WPA2/WPA2-Personal"));
    } else {
      AZURE_PRINTF("\tSaved SIM PIN   : %s\r\n",BG96_CredAcc.sim_pin);
      AZURE_PRINTF("\tSaved APN : %s\r\n",BG96_CredAcc.sim_apn);
      //AZURE_PRINTF("\tSaved EncMode: %s\r\n",(WIFI_CredAcc.mode == None) ? "Open" : ((WIFI_CredAcc.mode == WEP) ? "WEP" : "WPA2/WPA2-Personal"));
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
      if((console_input[0]=='y') || (console_input[0]=='Y'))
				{
        int32_t ReadFromTerminal=1;

				#if 0
				/* If the NFC is present */
        if(TargetBoardFeatures.NFCInitialized) {
          AZURE_PRINTF("\tDo you want to read them from NFC?(y/n)\r\n");
          scanf("%s",console_input);
          if(console_input[0]=='y') {
            sWifiTokenInfo TokenInfo;
            if(TT4_ReadWifiToken(&TokenInfo) == SUCCESS) {
              ReadFromTerminal=0;

              sprintf(BG96_CredAcc.sim_pin,"%s",TokenInfo.NetworkSSID);
              sprintf(BG96_CredAcc.sim_apn,"%s",TokenInfo.NetworkKey);

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
				#endif
				
        if(ReadFromTerminal==1) {
          AZURE_PRINTF("\r\nEnter the SIM PIN:\r\n");
          scanf("%s",console_input);
          sprintf(BG96_CredAcc.sim_pin,"%s",console_input);
          AZURE_PRINTF("\r\nEnter the APN:\r\n");
          scanf("%s",console_input);
          sprintf(BG96_CredAcc.sim_apn,"%s",console_input);
					/*
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
					*/
        }
        AZURE_PRINTF("\tNew SIM PIN   : %s\r\n",BG96_CredAcc.sim_pin);
        AZURE_PRINTF("\tNew APN : %s\r\n",BG96_CredAcc.sim_apn);
        //AZURE_PRINTF("\tNew EncMode: %s\r\n",(WIFI_CredAcc.mode == None) ? "Open" : ((WIFI_CredAcc.mode == WEP) ? "WEP" : "WPA2/WPA2-Personal"));

        /* Save the WIFI Credential on MetaDataManager */
        MDM_SaveGMD(GMD_BG96,(void *)&BG96_CredAcc);

        /* Will be Necessary Save the Meta Data Manager in Flash */
        NecessityToSaveMMD=1;

        AZURE_PRINTF("-----------------------------\r\n");
      }
      AZURE_PRINTF("\r\n");
    }

#ifndef AZURE_ENABLE_REGISTRATION
    AZURE_PRINTF("--------------------------\r\n");
    AZURE_PRINTF("|   Connection String     |\r\n");
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

/***************************************************************************************/
//application startup section	
		
  /* Initialize the GSM module */
  GSM_state = BG96_state_idle;
	
  //memset(&config,0,sizeof(wifi_config));
  //config.power=wifi_active;
  //config.power_level=high;
  //config.dhcp=on;//use DHCP IP address
  //status = wifi_init(&config);
	/*
  if(gsm_init(BG96_CredAcc.sim_pin, BG96_CredAcc.sim_apn) == false) {
    AZURE_PRINTF("Err in Config\r\n");
    AzureExit(AZURE_ERR_WIFI);
  } else {
    AZURE_PRINTF("\r\nGSM module configured and connected \r\n");
  }

	GET_BG96_mac(macaddstart);
	AZURE_PRINTF("WiFi MAC Address is: %s\r\n",macaddstart);
	*/
	/*
  GET_Configuration_Value("nv_wifi_macaddr",(uint32_t *)macaddstart);
  macaddstart[17]='\0';
  AZURE_PRINTF("WiFi MAC Address is: %s\r\n",macaddstart);
	*/
	
	/*
  status = wifi_connect(WIFI_CredAcc.ssid, WIFI_CredAcc.seckey, WIFI_CredAcc.mode);
  if(status!=WiFi_MODULE_SUCCESS) {
    AZURE_PRINTF("Err Connecting to WIFI\r\n");
    AzureExit(AZURE_ERR_WIFI_CON);
  }
	*/

  /* Wait WIFI Connection */
	AZURE_PRINTF("\r\nGSM init... wait\r\n");
  while(GSM_state != BG96_state_connected) 
	{
		if(gsm_init(BG96_CredAcc.sim_pin, BG96_CredAcc.sim_apn) == false) 
			HAL_Delay(5);
	}  
	AZURE_PRINTF("\r\nGSM module configured and connected \r\n");
  GET_BG96_mac(macaddstart);
	AZURE_PRINTF("-> MAC Address is: %s\r\n",macaddstart);
	
  /* initialize Real Time Clock */
  InitRTC();
	#if 1
  /* Check if Data stored in BackUp register0: No Need to reconfigure RTC */
  while(HAL_RTCEx_BKUPRead(&TargetBoardFeatures.RtcHandle, RTC_BKP_DR0) != 0x32F2){
    /* Configure RTC Calendar */
    char ipAddress[16]; // "time-d.nist.gov";
    uint8_t tcpPort  =  NTP_ENDPOINT_TCPPORT;
    uint8_t socketHandle;
    uint8_t NumRentryNTP=0;
    static NTP_ENDPOINT NTP_Server=NTP_DEFAULT;
    switch(NTP_Server) {
      case NTP_DEFAULT:
        strcpy(ipAddress, NTP_ENDPOINT_IPADDRESS_DEFAULT);
      break;
      case NTP_BACKUP1:
        strcpy(ipAddress, NTP_ENDPOINT_IPADDRESS_BACKUP1);
      break;
      case NTP_BACKUP2:
        strcpy(ipAddress, NTP_ENDPOINT_IPADDRESS_BACKUP2);
      break;
      default:
        AzureExit(AZURE_ERR_NTP);
    }

    AZURE_PRINTF("Contact NTP server %s \r\n", ipAddress);

    status = BG96_socket_client_open((uint8_t*)ipAddress, tcpPort, (uint8_t*) "t", &socketHandle);
		
    if(status!=BG96_MODULE_SUCCESS) {
      AZURE_PRINTF("Err opening socket with NTP server\r\n");
    } else {
      AZURE_PRINTF("WiFi opened socket with NTP server [%s]\r\n",ipAddress);
      NTP_Status = NTP_STATUS_CONNECTED;

      // Wait to receive data and that NTP server closes the Socket
      while(NTP_Status!=NTP_STATUS_CLOSED) {
        HAL_Delay(100);
      }
      AZURE_PRINTF("Socket closed with NTP server\r\n");
      NTP_Status=NTP_STATUS_NULL;
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
  }
	#endif
  /* RTC Is ok */
  {
    uint8_t aShowTime[50] = {0};
    uint8_t aShowDate[50] = {0};
    RTC_CalendarShow(aShowTime,aShowDate);
    AZURE_PRINTF("Init Real Time Clock %s %s\r\n",aShowDate,aShowTime);
  }

  /* Init the Local Buffer for mbedTLS for Telemetry */
  LocalBufferInit(&localBufferReading);
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

/**
  * @brief  Function called when the Socket is closed
  * @param  uint8_t* socketID pointer to the socket handle
  * @retval None
  */
void ind_bg96_socket_client_remote_server_closed(uint8_t * socketID)
{
  AZURE_PRINTF("GSM socket closed =%x\r\n",* socketID);
  if (NTP_Status==NTP_STATUS_CONNECTED) {
    NTP_Status = NTP_STATUS_CLOSED;
  }
}

/**
 * @brief  GSM callback for data received
 * @param  uint8_t pSocketHandle socket handle
 * @param  uint8_t * data_ptr pointer to buffer containing data received 
 * @param  uint32_t message_size number of bytes in message received
 * @param  uint32_t chunk_size numeber of bytes in chunk
 * @retval None
 */
void ind_bg96_socket_data_received(uint8_t pSocketHandle,uint8_t * data_ptr, uint32_t message_size, uint32_t chunk_size)
{
#if 0
  /* Just for Debug */
  AZURE_PRINTF("****** Received buffer for a socket...***** \r\n");
  AZURE_PRINTF("socket received =\r\n");
  AZURE_PRINTF("\tSock=%d\r\n",pSocketHandle);
  //AZURE_PRINTF("\tdata=%s\r\n",data_ptr);
  AZURE_PRINTF("\tmessage_size=%ld\r\n",message_size);
  AZURE_PRINTF("\tchunk_size=%ld\r\n",chunk_size);
#endif

  if(NTP_Status==NTP_STATUS_CONNECTED) {
    if(message_size==4){
      time_t epochTimeToSetForSystem = SynchronizationAgentConvertNTPTime2EpochTime(data_ptr,message_size);
      if (TimingSystemSetSystemTime(epochTimeToSetForSystem)== 0){
        AZURE_PRINTF("Err Failed to set system time. \r\n");
      } else {
        AZURE_PRINTF("Set UTC Time: %s\r\n",(get_ctime(&epochTimeToSetForSystem)));
      }
    }
  } else {
    /* Secure mbedTLS socket for Telemetry */
    if(LocalBufferPushBuffer(&localBufferReading, (void*) data_ptr,chunk_size)==0) {
      AZURE_PRINTF("Err: for Pushing Buffer for Telemetry\r\n");
    }
  }
}

/**
 * @brief  WiFi callback when it is connected to Access Point
 * @param  None
 * @retval None
 */
void ind_bg96_connected()
{
  AZURE_PRINTF("GSM connected to network\r\n");
  GSM_state = BG96_state_connected;
}

/**
 * @brief  WiFi callback when it is initilized
 * @param  None
 * @retval None
 */
void ind_wifi_on()
{
  AZURE_PRINTF("\r\n\nwifi started and ready...\r\n");
  GSM_state = BG96_state_ready;
}

/** @brief function for providing the default TLS I/O interface
 * @param None
 * @retval IO_INTERFACE_DESCRIPTION* Pointer to the default TLS I/O interface
 */
const IO_INTERFACE_DESCRIPTION* platform_get_default_tlsio(void)
{
#ifdef USE_MBED_TLS
  return tlsio_mbedtls_get_interface_description();
#else /* USE_MBED_TLS */
  return tlsio_STM32Cube_get_interface_description();
#endif /* USE_MBED_TLS */
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
#ifndef IKS01A2
  /* Test if the board is IK01A1 */
  if (BSP_ACCELERO_Init_IKS01A1( ACCELERO_SENSORS_AUTO, &TargetBoardFeatures.HandleAccSensor ) == COMPONENT_OK) {
    AZURE_PRINTF("IKS01A1 board\n\r");
    AZURE_PRINTF("Ok Accelero Sensor\n\r");
    TargetBoardFeatures.SnsAltFunc = 0;
  } else {
#else /* IKS01A2 */
  {
#endif /* IKS01A2 */
    TargetBoardFeatures.SnsAltFunc = 1;
    if (BSP_ACCELERO_Init_IKS01A2( ACCELERO_SENSORS_AUTO, &TargetBoardFeatures.HandleAccSensor ) == COMPONENT_OK){
      AZURE_PRINTF("IKS01A2 board\n\r");
      AZURE_PRINTF("Ok Accelero Sensor\n\r");
    } else {
      AZURE_PRINTF("IKS01A2 or IKS01A1 board not present, Emulation enabled\n\r");
      TargetBoardFeatures.EmulateSns=1;
    }
  }

  if(!TargetBoardFeatures.EmulateSns) {
    /* DS3/DSM or DS0 */
    /* This section works with IKS01A1 or with IKS01A1/A2 Autodiscovery */
#ifndef IKS01A2
    if(!TargetBoardFeatures.SnsAltFunc){
      uint8_t WhoAmI;
      BSP_ACCELERO_Get_WhoAmI(TargetBoardFeatures.HandleAccSensor,&WhoAmI);
      if(LSM6DS3_ACC_GYRO_WHO_AM_I==WhoAmI) {
        AZURE_PRINTF("\tDS3 DIL24 Present\n\r");
        TargetBoardFeatures.HWAdvanceFeatures = 1;
      } else {
        TargetBoardFeatures.HWAdvanceFeatures = 0;
      }
    } else {
#else /* IKS01A2 */
   {
#endif /* IKS01A2 */
      TargetBoardFeatures.HWAdvanceFeatures = 1;
    }

    /* Gyro */
    if((TargetBoardFeatures.SnsAltFunc ? BSP_GYRO_Init_IKS01A2 : BSP_GYRO_Init)( GYRO_SENSORS_AUTO, &TargetBoardFeatures.HandleGyroSensor )==COMPONENT_OK){
      AZURE_PRINTF("Ok Gyroscope Sensor\n\r");
    } else {
      AZURE_PRINTF("Err Gyroscope Sensor\n\r");
      while(1);
    }

    if((TargetBoardFeatures.SnsAltFunc ? BSP_MAGNETO_Init_IKS01A2 : BSP_MAGNETO_Init)( MAGNETO_SENSORS_AUTO, &TargetBoardFeatures.HandleMagSensor )==COMPONENT_OK){
      AZURE_PRINTF("Ok Magneto Sensor\n\r");
    } else {
      AZURE_PRINTF("Err Magneto Sensor\n\r");
      while(1);
    }

    /* Humidity */
    if((TargetBoardFeatures.SnsAltFunc ? BSP_HUMIDITY_Init_IKS01A2 : BSP_HUMIDITY_Init)( HUMIDITY_SENSORS_AUTO, &TargetBoardFeatures.HandleHumSensor )==COMPONENT_OK){
      AZURE_PRINTF("Ok Humidity Sensor\n\r");
    } else {
      AZURE_PRINTF("Err Humidity Sensor\n\r");
    }

    /* Temperature1 */
    if((TargetBoardFeatures.SnsAltFunc ? BSP_TEMPERATURE_Init_IKS01A2 : BSP_TEMPERATURE_Init)( TEMPERATURE_SENSORS_AUTO, &TargetBoardFeatures.HandleTempSensors[TargetBoardFeatures.NumTempSensors] )==COMPONENT_OK){
       AZURE_PRINTF("Ok Temperature Sensor1\n\r");
       TargetBoardFeatures.NumTempSensors++;
    } else {
      AZURE_PRINTF("Err Temperature Sensor1\n\r");
    }

    /* Temperature2 */
#ifndef IKS01A2
    if((TargetBoardFeatures.SnsAltFunc ? BSP_TEMPERATURE_Init_IKS01A2 : BSP_TEMPERATURE_Init) (TargetBoardFeatures.SnsAltFunc ? LPS22HB_T_0: LPS25HB_T_0,&TargetBoardFeatures.HandleTempSensors[TargetBoardFeatures.NumTempSensors] )==COMPONENT_OK){
#else /* IKS01A2 */
    if(BSP_TEMPERATURE_Init_IKS01A2(LPS22HB_T_0,&TargetBoardFeatures.HandleTempSensors[TargetBoardFeatures.NumTempSensors] )==COMPONENT_OK){      
#endif /* IKS01A2 */
      AZURE_PRINTF("Ok Temperature Sensor2\n\r");
      TargetBoardFeatures.NumTempSensors++;
    } else {
      AZURE_PRINTF("Err Temperature Sensor2\n\r");
    }

    /* Pressure */
    if((TargetBoardFeatures.SnsAltFunc ? BSP_PRESSURE_Init_IKS01A2 : BSP_PRESSURE_Init)( PRESSURE_SENSORS_AUTO, &TargetBoardFeatures.HandlePressSensor )==COMPONENT_OK){
      AZURE_PRINTF("Ok Pressure Sensor\n\r");
    } else {
      AZURE_PRINTF("Err Pressure Sensor\n\r");
    }

    /*  Enable all the sensors */
    if(TargetBoardFeatures.HandleAccSensor) {
      if((TargetBoardFeatures.SnsAltFunc ? BSP_ACCELERO_Sensor_Enable_IKS01A2 : BSP_ACCELERO_Sensor_Enable)( TargetBoardFeatures.HandleAccSensor )==COMPONENT_OK) {
        AZURE_PRINTF("Enabled Accelero Sensor\n\r");
      }
    }

    if(TargetBoardFeatures.HandleGyroSensor) {
      if((TargetBoardFeatures.SnsAltFunc ? BSP_GYRO_Sensor_Enable_IKS01A2 : BSP_GYRO_Sensor_Enable)( TargetBoardFeatures.HandleGyroSensor )==COMPONENT_OK) {
        AZURE_PRINTF("Enabled Gyroscope Sensor\n\r");
      }
    }

    if(TargetBoardFeatures.HandleMagSensor) {
      if((TargetBoardFeatures.SnsAltFunc ? BSP_MAGNETO_Sensor_Enable_IKS01A2 : BSP_MAGNETO_Sensor_Enable)( TargetBoardFeatures.HandleMagSensor )==COMPONENT_OK) {
        AZURE_PRINTF("Enabled Magneto Sensor\n\r");
      }
    }

    if(TargetBoardFeatures.HandleHumSensor) {
      if((TargetBoardFeatures.SnsAltFunc ? BSP_HUMIDITY_Sensor_Enable_IKS01A2 : BSP_HUMIDITY_Sensor_Enable) ( TargetBoardFeatures.HandleHumSensor)==COMPONENT_OK) {
        AZURE_PRINTF("Enabled Humidity Sensor\n\r");
      }
    }

    if(TargetBoardFeatures.HandleTempSensors[0]){
      if((TargetBoardFeatures.SnsAltFunc ? BSP_TEMPERATURE_Sensor_Enable_IKS01A2 : BSP_TEMPERATURE_Sensor_Enable)( TargetBoardFeatures.HandleTempSensors[0])==COMPONENT_OK) {
        AZURE_PRINTF("Enabled Temperature Sensor1\n\r");
      }
    }

    if(TargetBoardFeatures.HandleTempSensors[1]){
      if((TargetBoardFeatures.SnsAltFunc ? BSP_TEMPERATURE_Sensor_Enable_IKS01A2 : BSP_TEMPERATURE_Sensor_Enable)( TargetBoardFeatures.HandleTempSensors[1])==COMPONENT_OK) {
        AZURE_PRINTF("Enabled Temperature Sensor2\n\r");
      }
    }

    if(TargetBoardFeatures.HandlePressSensor) {
      if((TargetBoardFeatures.SnsAltFunc ? BSP_PRESSURE_Sensor_Enable_IKS01A2 : BSP_PRESSURE_Sensor_Enable)( TargetBoardFeatures.HandlePressSensor)==COMPONENT_OK) {
        AZURE_PRINTF("Enabled Pressure Sensor\n\r");
      }
    }
  }
}

/**
  * @brief  Send Notification where there is a interrupt from MEMS
  * @param  None
  * @retval uint32_t type of Hardware Feature Answer
  */
uint32_t MEMSCallback(void)
{
  ACCELERO_Event_Status_t status;

  if(W2ST_CHECK_HW_FEATURE(W2ST_HWF_FREE_FALL)) {
    /* Check if the interrupt is due to Free Fall */
    (TargetBoardFeatures.SnsAltFunc ? BSP_ACCELERO_Get_Event_Status_Ext_IKS01A2 : BSP_ACCELERO_Get_Event_Status_Ext)(TargetBoardFeatures.HandleAccSensor,&status);
    if(status.FreeFallStatus!=0) {
      /***** Only for testing the RTC. From here */
      uint8_t aShowTime[50] = {0};
      uint8_t aShowDate[50] = {0};
      RTC_CalendarShow(aShowTime,aShowDate);
      AZURE_PRINTF("%s %s ->",aShowDate,aShowTime);
      /* Until here *****/
      AZURE_PRINTF("Free Fall\r\n");
      return ACC_FREE_FALL;
    }
  }

  if(W2ST_CHECK_HW_FEATURE(W2ST_HWF_DOUBLE_TAP)) {
    /* Check if the interrupt is due to Double Tap */
    (TargetBoardFeatures.SnsAltFunc ? BSP_ACCELERO_Get_Event_Status_Ext_IKS01A2 : BSP_ACCELERO_Get_Event_Status_Ext)(TargetBoardFeatures.HandleAccSensor,&status);
    if(status.DoubleTapStatus!=0) {
      AZURE_PRINTF("Double Tap\r\n");
      return ACC_DOUBLE_TAP;
    }
  }

  if(W2ST_CHECK_HW_FEATURE(W2ST_HWF_SINGLE_TAP)) {
    /* Check if the interrupt is due to Single Tap */
    (TargetBoardFeatures.SnsAltFunc ? BSP_ACCELERO_Get_Event_Status_Ext_IKS01A2 : BSP_ACCELERO_Get_Event_Status_Ext)(TargetBoardFeatures.HandleAccSensor,&status);
    if(status.TapStatus!=0) {
      AZURE_PRINTF("Single Tap\r\n");
      return ACC_SINGLE_TAP;
    }
  }

  if(W2ST_CHECK_HW_FEATURE(W2ST_HWF_WAKE_UP)) {
    /* Check if the interrupt is due to Wake Up */
    (TargetBoardFeatures.SnsAltFunc ? BSP_ACCELERO_Get_Event_Status_Ext_IKS01A2 : BSP_ACCELERO_Get_Event_Status_Ext)(TargetBoardFeatures.HandleAccSensor,&status);
    if(status.WakeUpStatus!=0) {
      AZURE_PRINTF("Wake Up\r\n");
      return ACC_WAKE_UP;
    }
  }

  if(W2ST_CHECK_HW_FEATURE(W2ST_HWF_TILT)) {
    /* Check if the interrupt is due to Tilt */
    (TargetBoardFeatures.SnsAltFunc ? BSP_ACCELERO_Get_Event_Status_Ext_IKS01A2 : BSP_ACCELERO_Get_Event_Status_Ext)(TargetBoardFeatures.HandleAccSensor,&status);
    if(status.TiltStatus!=0) {
      AZURE_PRINTF("Tilt\r\n");
      return ACC_TILT;
    }
  }

  if(W2ST_CHECK_HW_FEATURE(W2ST_HWF_6DORIENTATION)) {
    /* Check if the interrupt is due to 6D Orientation */
    (TargetBoardFeatures.SnsAltFunc ? BSP_ACCELERO_Get_Event_Status_Ext_IKS01A2 : BSP_ACCELERO_Get_Event_Status_Ext)(TargetBoardFeatures.HandleAccSensor,&status);
    if(status.D6DOrientationStatus!=0) {
      AccEventType Orientation = GetHWOrientation6D();
      AZURE_PRINTF("6DOrientation=%d\r\n",Orientation);
      return Orientation;
    }
  }

  if(W2ST_CHECK_HW_FEATURE(W2ST_HWF_PEDOMETER)) {
    /* Check if the interrupt is due to Pedometer */
    (TargetBoardFeatures.SnsAltFunc ? BSP_ACCELERO_Get_Event_Status_Ext_IKS01A2 : BSP_ACCELERO_Get_Event_Status_Ext)(TargetBoardFeatures.HandleAccSensor,&status);
    if(status.StepStatus!=0) {
      uint16_t StepCount = GetStepHWPedometer();
      AZURE_PRINTF("StepCount=%d\r\n",StepCount);
      return StepCount;
    }
  }
  /* If nothing... */
  return ACC_NOT_USED;
}

#ifdef USE_STM32F4XX_NUCLEO
/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow:
  *            System Clock source            = PLL (HSI)
  *            SYSCLK(Hz)                     = 84000000
  *            HCLK(Hz)                       = 84000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 2
  *            APB2 Prescaler                 = 1
  *            HSI Frequency(Hz)              = 16000000
  *            PLL_M                          = 16
  *            PLL_N                          = 336
  *            PLL_P                          = 4
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale2 mode
  *            Flash Latency(WS)              = 2
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable Power Control clock */
  __PWR_CLK_ENABLE();

  /* The voltage scaling allows optimizing the power consumption when the device is
     clocked below the maximum system frequency, to update the voltage scaling value
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /* Enable HSI Oscillator and activate PLL with HSI as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 0x10;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK){
    STM32_Error_Handler();
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK){
    STM32_Error_Handler();
  }
}
#endif /* USE_STM32F4XX_NUCLEO */

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
  case KEY_BUTTON_PIN:
    ButtonPressed = 1;
    break;
#ifndef IKS01A2
   case M_INT1_PIN:
#endif /* IKS01A2 */
   case LSM6DSL_INT1_O_PIN:
    MEMSInterrupt=1;
    break;
  }
}

#if 0
/**
* @brief  Period elapsed callback in non blocking mode
*         This timer is used for calling back User registered functions with information
* @param  htim TIM handle
* @retval None
*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  Wifi_TIM_Handler(htim);
}
#endif

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
  * @brief This function initializes the GPIO for NFC expansion board
  * @param None
  * @retval None
  */

void M24SR_GPOInit(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOA_CLK_ENABLE();

  /* Configure GPIO pins for GPO (PA6)*/
#ifndef I2C_GPO_INTERRUPT_ALLOWED
  GPIO_InitStruct.Pin = M24SR_GPO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(M24SR_GPO_PIN_PORT, &GPIO_InitStruct);
#else
  GPIO_InitStruct.Pin = M24SR_GPO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(M24SR_GPO_PIN_PORT, &GPIO_InitStruct);
  /* Enable and set EXTI9_5 Interrupt to the lowest priority */
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
#endif

  /* Configure GPIO pins for DISABLE (PA7)*/
  GPIO_InitStruct.Pin = M24SR_RFDIS_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(M24SR_RFDIS_PIN_PORT, &GPIO_InitStruct);
}

/**
  * @brief  This function wait the time given in param (in milisecond)
	* @param	time_ms: time value in milisecond
  */
void M24SR_WaitMs(uint32_t time_ms)
{
  HAL_Delay(time_ms);
}

/**
  * @brief  This function retrieve current tick
  * @param	ptickstart: pointer on a variable to store current tick value
  */
void M24SR_GetTick( uint32_t *ptickstart )
{
  *ptickstart = HAL_GetTick();
}
/**
  * @brief  This function read the state of the M24SR GPO
	* @param	none
  * @retval GPIO_PinState : state of the M24SR GPO
  */
void M24SR_GPO_ReadPin( GPIO_PinState * pPinState)
{
  *pPinState = HAL_GPIO_ReadPin(M24SR_GPO_PIN_PORT,M24SR_GPO_PIN);
}

/**
  * @brief  This function set the state of the M24SR RF disable pin
	* @param	PinState: put RF disable pin of M24SR in PinState (1 or 0)
  */
void M24SR_RFDIS_WritePin( GPIO_PinState PinState)
{
  HAL_GPIO_WritePin(M24SR_RFDIS_PIN_PORT,M24SR_RFDIS_PIN,PinState);
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

#ifdef USE_STM32F4XX_NUCLEO
/**
 * @brief User function for Erasing the MDM on Flash
 * @param None
 * @retval uint32_t Success/NotSuccess [1/0]
 */
uint32_t UserFunctionForErasingFlash(void) {
  FLASH_EraseInitTypeDef EraseInitStruct;
  uint32_t SectorError = 0;
  uint32_t Success=1;

  EraseInitStruct.TypeErase = TYPEERASE_SECTORS;
  EraseInitStruct.VoltageRange = VOLTAGE_RANGE_3;
  EraseInitStruct.Sector = FLASH_SECTOR_7;
  EraseInitStruct.NbSectors = 1;

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
  uint32_t *WriteIndex;

  /* Unlock the Flash to enable the flash control register access *************/
  HAL_FLASH_Unlock();

  for(WriteIndex =((uint32_t *) InitMetaDataVector); WriteIndex<((uint32_t *) EndMetaDataVector); WriteIndex++) {
    if (HAL_FLASH_Program(TYPEPROGRAM_WORD, Address,*WriteIndex) == HAL_OK){
      Address = Address + 4;
    } else {
      /* Error occurred while writing data in Flash memory.
         User can add here some code to deal with this error
         FLASH_ErrorTypeDef errorcode = HAL_FLASH_GetError(); */
      STM32_Error_Handler();
      Success=0;
    }
  }

  /* Lock the Flash to disable the flash control register access (recommended
   to protect the FLASH memory against possible unwanted operation) *********/
  HAL_FLASH_Lock();
 
  return Success;
}
#endif /* USE_STM32F4XX_NUCLEO */

STRING_HANDLE platform_get_platform_info(void)
{
  return STRING_construct("(STM32Cube)");
}
/******************* (C) COPYRIGHT 2017 STMicroelectronics *****END OF FILE****/
