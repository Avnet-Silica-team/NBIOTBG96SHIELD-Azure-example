/**
 ******************************************************************************
 * @file    platform_STM32Cube_NucleoF429ZI.c
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
#include "main.h"
#include "lwip/netif.h"
#include "lwip/tcpip.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "cmsis_os.h"
#include "ethernetif.h"

#include "Typedef_LwIP_IKS01AX.h"
#include "STM32CubeInterface.h"
#include "MetaDataManager.h"
#include "azure_c_shared_utility/crt_abstractions.h"
#include "azure_c_shared_utility/platform.h"
#include "OTA.h"
#include "AzureClient_mqtt_DM_TM.h"

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

volatile uint32_t ButtonPressed = 0;
volatile uint32_t MEMSInterrupt = 0;
volatile uint32_t SendData = 0;

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

static volatile int32_t ConnectedToNTP=0;

static uint32_t FullOTASize=0;


//#define DEBUG_OTA_RECEIVE

static AZURE1_OTA OTAStatus=OTA_STATUS_NULL;

struct netif gnetif; /* network interface structure */

uint8_t macAddress[6];

/**
 * @brief  Eth Configuration Structure
 */
typedef struct
{
  int use_dhcp;
  uint8_t macAddress[6];
  int ipaddress[4];
  int gwaddress[4];
} AzureEthConfiguration_t;

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
DEFINE_ENUM_STRINGS(AZURE1_ERROR, AZURE1_ERROR_VALUES)

//2kHz/0.5 For Sensors Data data @ 0.5Hz
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
static void SetRTC(void);
static void ReadNFC(AzureEthConfiguration_t *EthConfiguration);
static void ReadUART(AzureEthConfiguration_t *EthConfiguration);

/* DHCP process states */
#define DHCP_OFF                   (uint8_t) 0
#define DHCP_START                 (uint8_t) 1
#define DHCP_WAIT_ADDRESS          (uint8_t) 2
#define DHCP_ADDRESS_ASSIGNED      (uint8_t) 3
#define DHCP_TIMEOUT               (uint8_t) 4
#define DHCP_LINK_DOWN             (uint8_t) 5

void User_notification(struct netif *netif,AzureEthConfiguration_t EthConfiguration);

#include "lwip/dhcp.h"
void DHCP_init(void const * argument);
#define MAX_DHCP_TRIES  4
__IO uint8_t DHCP_state;

static void Netif_Config(AzureEthConfiguration_t EthConfiguration);

/**
  * @brief  Function for Initializing the platform
  * @param  None
  * @retval int Ok/Error (0/1)
  */
int platform_init(void)
{
  AzureEthConfiguration_t EthConfiguration;

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
         "\tSTM32F429ZI-Nucleo board"
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

  if(TargetBoardFeatures.HWAdvanceFeatures) {
    InitHWFeatures();
  }

  /* Check the BootLoader Compliance */
  if(CheckBootLoaderCompliance()) {
    AZURE_PRINTF("BootLoader Compliant with FOTA procedure\r\n");
  } else {
    AZURE_PRINTF("Err: BootLoader NOT Compliant with FOTA procedure\r\n");
    AzureExit(AZURE_ERR_BL_COMPLIANCE);
  }

  /* Set Full Scale to +/-2g */
  (TargetBoardFeatures.SnsAltFunc ? BSP_ACCELERO_Set_FS_Value_IKS01A2 : BSP_ACCELERO_Set_FS_Value)(TargetBoardFeatures.HandleAccSensor,2.0f);

  /* initialize timers */
  InitTimers();
  AZURE_PRINTF("Init Application's Timers\r\n");

  /* Initialize random number generartion */
  TargetBoardFeatures.RngHandle.Instance = RNG;
  HAL_RNG_Init(&TargetBoardFeatures.RngHandle);
  AZURE_PRINTF("Init Random Number Generator\r\n");
    
  /* Enabling HW Features... FreeFall */
  EnableHWFreeFall();
  
  /* Check if NFC is present */
  if(TT4_Init() == SUCCESS) {
    AZURE_PRINTF("X-NUCLEO-NFC01A1 is present\r\n");
    TargetBoardFeatures.NFCInitialized = 1;
  }

  {
    int32_t CounterButtonPress=0;
    char console_input[128];
    int32_t NecessityToSaveMMD=0;

    MDM_knownGMD_t known_MetaData[]={
      {GMD_ETH,sizeof(AzureEthConfiguration_t)},
#ifndef AZURE_ENABLE_REGISTRATION
      {GMD_AZURE,sizeof(AzureConnection_t)},
#endif /* AZURE_ENABLE_REGISTRATION */
      {GMD_END    ,0}/* THIS MUST BE THE LAST ONE */
    };
    
    /* Initialize the MetaDataManager */
    InitMetaDataManager((void *)&known_MetaData,MDM_DATA_TYPE_GMD,NULL);

    /* Recall the Eth Configuration saved */
    memset(&EthConfiguration,0,sizeof(AzureEthConfiguration_t));
    MDM_ReCallGMD(GMD_ETH,(void *)&EthConfiguration);
    
    if((EthConfiguration.macAddress[0]==0) &
       (EthConfiguration.macAddress[1]==0) &
       (EthConfiguration.macAddress[2]==0) &
       (EthConfiguration.macAddress[3]==0) &
       (EthConfiguration.macAddress[4]==0) &
       (EthConfiguration.macAddress[5]==0)) {

      AZURE_PRINTF("Eth Configuration not initialized->Assign default values\r\n");
      /* Assign default values */
      EthConfiguration.ipaddress[0] = IP_ADDR0;
      EthConfiguration.ipaddress[1] = IP_ADDR1;
      EthConfiguration.ipaddress[2] = IP_ADDR2;
      EthConfiguration.ipaddress[3] = IP_ADDR3;

      EthConfiguration.gwaddress[0] = GW_ADDR0;
      EthConfiguration.gwaddress[1] = GW_ADDR1;
      EthConfiguration.gwaddress[2] = GW_ADDR2;
      EthConfiguration.gwaddress[3] = GW_ADDR3;

      EthConfiguration.use_dhcp = true;

      EthConfiguration.macAddress[0] = macAddress[0] = MAC_ADDR0;
      EthConfiguration.macAddress[1] = macAddress[1] = MAC_ADDR1;
      EthConfiguration.macAddress[2] = macAddress[2] = MAC_ADDR2;
      EthConfiguration.macAddress[3] = macAddress[3] = MAC_ADDR3;
      EthConfiguration.macAddress[4] = macAddress[4] = MAC_ADDR4;
      EthConfiguration.macAddress[5] = macAddress[5] = MAC_ADDR5;
    } else {
      macAddress[0] = EthConfiguration.macAddress[0];
      macAddress[1] = EthConfiguration.macAddress[1];
      macAddress[2] = EthConfiguration.macAddress[2];
      macAddress[3] = EthConfiguration.macAddress[3];
      macAddress[4] = EthConfiguration.macAddress[4];
      macAddress[5] = EthConfiguration.macAddress[5];
    }

    /* Print out the ETH configuration */
    AZURE_PRINTF("    MAC addr saved is %02x:%02x:%02x:%02x:%02x:%02x\r\n",
               macAddress[0],macAddress[1],macAddress[2],
               macAddress[3],macAddress[4],macAddress[5]);

    if(EthConfiguration.use_dhcp == false) {
      AZURE_PRINTF("    IP addr saved is %d.%d.%d.%d\r\n",
                   EthConfiguration.ipaddress[0],EthConfiguration.ipaddress[1],EthConfiguration.ipaddress[2],EthConfiguration.ipaddress[3]);
      AZURE_PRINTF("    GW addr saved is %d.%d.%d.%d\r\n",
                   EthConfiguration.gwaddress[0],EthConfiguration.gwaddress[1],EthConfiguration.gwaddress[2],EthConfiguration.gwaddress[3]);
    } else {
      AZURE_PRINTF("    IP addr with DHCP\r\n");
    }

    {
      int32_t CountOn,CountOff;
      AZURE_PRINTF("\r\nWait 3 seconds for allowing User Button Control for changing it\r\n");
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
            ReadFromTerminal=0;
            ReadNFC(&EthConfiguration);
          }
        }

        if(ReadFromTerminal) {
          ReadUART(&EthConfiguration);
        }

        /* Save the ETH Configuration on MetaDataManager */
        MDM_SaveGMD(GMD_ETH,(void *)&EthConfiguration);

        /* We need to save the MetaDataManager */
        NecessityToSaveMMD=1;
      }
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

  /* Create tcp_ip stack thread */
  tcpip_init(NULL, NULL);
  
  /* Initialize the LwIP stack */
  Netif_Config(EthConfiguration);

  /* Notify user about the network interface config */
  User_notification(&gnetif,EthConfiguration);
  
  if ( EthConfiguration.use_dhcp )
    DHCP_init(&gnetif);
  
  /* initialize Real Time Clock */
  InitRTC();
  
  SetRTC();
  
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
    __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_1, (uhCapture + 
                                                        TargetBoardFeatures.TIM_CC1_Pulse));
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
  * @brief  Function for getting default tls io
  * @param  None
  * @retval IO_INTERFACE_DESCRIPTION*
  */
const IO_INTERFACE_DESCRIPTION* platform_get_default_tlsio(void)
{
  return tlsio_mbedtls_get_interface_description();
}

/** @brief Function for de-initializing the platform
 *  @param None
 *  @retval None
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
  /* Test if the board is IK01A1 */
  if (BSP_ACCELERO_Init_IKS01A1( ACCELERO_SENSORS_AUTO, &TargetBoardFeatures.HandleAccSensor ) == COMPONENT_OK) {
    AZURE_PRINTF("IKS01A1 board\n\r");
    AZURE_PRINTF("OK Accelero Sensor\n\r");
    TargetBoardFeatures.SnsAltFunc = 0;
  } else {
    TargetBoardFeatures.SnsAltFunc = 1;
    if (BSP_ACCELERO_Init_IKS01A2( ACCELERO_SENSORS_AUTO, &TargetBoardFeatures.HandleAccSensor ) == COMPONENT_OK){
      AZURE_PRINTF("IKS01A2 board\n\r");
      AZURE_PRINTF("OK Accelero Sensor\n\r");
    } else {
      AZURE_PRINTF("IKS01A2 or IKS01A1 board not present, Emulation enabled\n\r");
      TargetBoardFeatures.EmulateSns=1;
    }
  }

  if(!TargetBoardFeatures.EmulateSns) {
    /* DS3/DSM or DS0 */
    /* This section works with IKS01A1 or with IKS01A1/A2 Autodiscovery */
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
      TargetBoardFeatures.HWAdvanceFeatures = 1;
    }

    /* Gyro */
    if((TargetBoardFeatures.SnsAltFunc ? BSP_GYRO_Init_IKS01A2 : BSP_GYRO_Init)( GYRO_SENSORS_AUTO, &TargetBoardFeatures.HandleGyroSensor )==COMPONENT_OK){
      AZURE_PRINTF("OK Gyroscope Sensor\n\r");
    } else {
      AZURE_PRINTF("Error Gyroscope Sensor\n\r");
      while(1);
    }

    if((TargetBoardFeatures.SnsAltFunc ? BSP_MAGNETO_Init_IKS01A2 : BSP_MAGNETO_Init)( MAGNETO_SENSORS_AUTO, &TargetBoardFeatures.HandleMagSensor )==COMPONENT_OK){
      AZURE_PRINTF("OK Magneto Sensor\n\r");
    } else {
      AZURE_PRINTF("Error Magneto Sensor\n\r");
      while(1);
    }

    /* Humidity */
    if((TargetBoardFeatures.SnsAltFunc ? BSP_HUMIDITY_Init_IKS01A2 : BSP_HUMIDITY_Init)( HUMIDITY_SENSORS_AUTO, &TargetBoardFeatures.HandleHumSensor )==COMPONENT_OK){
      AZURE_PRINTF("OK Humidity Sensor\n\r");
    } else {
      AZURE_PRINTF("Error Humidity Sensor\n\r");
    }

    /* Temperature1 */
    if((TargetBoardFeatures.SnsAltFunc ? BSP_TEMPERATURE_Init_IKS01A2 : BSP_TEMPERATURE_Init)( TEMPERATURE_SENSORS_AUTO, &TargetBoardFeatures.HandleTempSensors[TargetBoardFeatures.NumTempSensors] )==COMPONENT_OK){
       AZURE_PRINTF("OK Temperature Sensor1\n\r");
       TargetBoardFeatures.NumTempSensors++;
    } else {
      AZURE_PRINTF("Error Temperature Sensor1\n\r");
    }

    /* Temperature2 */
    if((TargetBoardFeatures.SnsAltFunc ? BSP_TEMPERATURE_Init_IKS01A2 : BSP_TEMPERATURE_Init) (TargetBoardFeatures.SnsAltFunc ? LPS22HB_T_0: LPS25HB_T_0,&TargetBoardFeatures.HandleTempSensors[TargetBoardFeatures.NumTempSensors] )==COMPONENT_OK){
      AZURE_PRINTF("OK Temperature Sensor2\n\r");
      TargetBoardFeatures.NumTempSensors++;
    } else {
      AZURE_PRINTF("Error Temperature Sensor2\n\r");
    }

    /* Pressure */
    if((TargetBoardFeatures.SnsAltFunc ? BSP_PRESSURE_Init_IKS01A2 : BSP_PRESSURE_Init)( PRESSURE_SENSORS_AUTO, &TargetBoardFeatures.HandlePressSensor )==COMPONENT_OK){
      AZURE_PRINTF("OK Pressure Sensor\n\r");
    } else {
      AZURE_PRINTF("Error Pressure Sensor\n\r");
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

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 180000000
  *            HCLK(Hz)                       = 180000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 8
  *            PLL_N                          = 360
  *            PLL_P                          = 2
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  
  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  
  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
   while(1) {};
  }
  
  if(HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
   while(1) {};
  }
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
   while(1) {};
  }
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
    __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_1, (uhCapture + 
                                                        TargetBoardFeatures.TIM_CC1_Pulse));
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
  /* Set TIM1 instance */
#if ((defined (USE_STM32F4XX_NUCLEO)) || (defined (USE_STM32L4XX_NUCLEO)))
  TimCCHandle.Instance = TIM1;
#endif
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
 * @brief  Function for setting the Real Time Clock
 * @param  None
 * @retval None
 */
void SetRTC(void)
{
  const char msg[] = "\n";
  char buf[8] = "";
  struct sockaddr_in address;
  struct hostent *server;
  
  char ipAddress[16]; // "time-d.nist.gov";
  uint8_t tcpPort  =  NTP_ENDPOINT_TCPPORT;
  int8_t socketHandle;
  uint8_t NumRentryNTP=0;
  
  time_t epochTimeToSetForSystem;
  
#define NTP_ENDPOINT_IPADDRESS \
        NTP_DEFAULT, \
        NTP_BACKUP1, \
        NTP_BACKUP2
DEFINE_ENUM(NTP_ENDPOINT, NTP_ENDPOINT_IPADDRESS)

  static NTP_ENDPOINT NTP_Server=NTP_DEFAULT;

  /* Check if Data stored in BackUp register0: No Need to reconfigure RTC */
  while(HAL_RTCEx_BKUPRead(&TargetBoardFeatures.RtcHandle, RTC_BKP_DR0) != 0x32F2){
    /* Configure RTC Calendar */
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

    if ((socketHandle = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
      AZURE_PRINTF("Can not create socket\r\n");
      return;
    }
    
    /* gethostbyname: get the server's DNS entry */
    server = gethostbyname(ipAddress);
    if (server == NULL) {
      AZURE_PRINTF("ERROR, no such host as %s\r\n", ipAddress);
      goto RETRY;
    }
    
    /* build the server's Internet address */
    memset((char *) &address, 0, sizeof(address));
    memcpy((char *)&address.sin_addr.s_addr,
           (char *)server->h_addr, server->h_length);
    address.sin_family = AF_INET;
    address.sin_port = htons(tcpPort);
    
    /* connect: create a connection with the server */
    if (connect(socketHandle, (struct sockaddr*)&address, sizeof(address)) < 0) {
      AZURE_PRINTF("ERROR connecting\r\n");
      goto RETRY;
    }

    /* write message to socket */
    if (write(socketHandle, msg, strlen(msg)) < 0) {
      AZURE_PRINTF("ERROR writing to socket");
      goto RETRY;
    }
    
    /* read reply from server */
    if ((read(socketHandle, buf, 8))< 0) {
      AZURE_PRINTF("ERROR reading from socket");
      goto RETRY;
    }
    
    buf[4]='\0';
    
    epochTimeToSetForSystem = SynchronizationAgentConvertNTPTime2EpochTime((uint8_t*)buf,4);
    if (TimingSystemSetSystemTime(epochTimeToSetForSystem)== 0){
      AZURE_PRINTF("Error Failed to set system time. \r\n");
      goto RETRY;
    } else {
      AZURE_PRINTF("Set UTC Time: %s\r\n",(get_ctime(&epochTimeToSetForSystem)));
    }
    
  RETRY:
    /* close socket */
    close(socketHandle);

    if(HAL_RTCEx_BKUPRead(&TargetBoardFeatures.RtcHandle, RTC_BKP_DR0) != 0x32F2) {

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
  /* RTC is ok */
  {
    uint8_t aShowTime[50] = {0};
    uint8_t aShowDate[50] = {0};
    RTC_CalendarShow(aShowTime,aShowDate);
    AZURE_PRINTF("Init Real Time Clock %s %s\r\n",aShowDate,aShowTime);
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
   case M_INT1_PIN:
   case LSM6DSL_INT1_O_PIN:
    MEMSInterrupt=1;
    break;
  }
}

/**
  * @brief  function called after the end of GET/HEAD request for FOTA
  * @param  void* context type of request (GET/HEAD)
  * @param  IO_SEND_RESULT send_result Results of the request (IO_SEND_OK/...)
  * @retval None
  */
static void onSendCompleteFOTA(void* context, IO_SEND_RESULT send_result)
{
  int32_t *TypeOfRequest = (int32_t *)context;
  if (send_result != IO_SEND_OK){
    AZURE_PRINTF("onSendCompleteFOTA Err for HTTP %s\r\n",((*TypeOfRequest) == HTTP_HEAD_REQUEST) ? "HEAD" : "GET"); 
  }
}

/**
  * @brief  function called After the Socket open request  for FOTA
  * @param  void* context (not Used)
  * @param  IO_OPEN_RESULT open_result Results of the open command (IO_OPEN_OK/...)
  * @retval None
  */
static void onOpenCompleteFOTA(void* context, IO_OPEN_RESULT open_result)
{  
  if (open_result != IO_OPEN_OK){
    AZURE_PRINTF("onOpenCompleteFOTA Err\r\n"); 
  }
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

  static uint32_t SizeOfUpdate=0;

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
    uint32_t RealMessageSize=0;

#define MCR_OTA_DOUBLE_WORLD(size) UpdateFWBlueMS(&SizeOfUpdate,StartAddress, size,1)
    
    if(SizeOfUpdate==0) {
      /* Search the Content-Length: */
     StartAddress = (uint8_t *) strstr( (char *) data_ptr, "Content-Length:");
      if(StartAddress!=NULL) {
        StartAddress+=16; // for moving to the end of "Content-Length: "
        SizeOfUpdate = atoi((char *) StartAddress);
#ifdef DEBUG_OTA_RECEIVE
        AZURE_PRINTF("Chunk OTA size=%ld\r\n",SizeOfUpdate);
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
  * example: FOTA,192.168.43.1,12345,/HelloL4_WithBL.bin
  * @param  char *hostname Server address
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
  io_interface_description = tlsio_mbedtls_get_interface_description();
  if (io_interface_description == NULL) {
    AZURE_PRINTF("Err: io_interface_description\r\n");
    return OTA_STATUS_ERROR;
  }    
  AZURE_PRINTF("OK io_interface_description\r\n");

  tls_io_config.hostname = hostname;
  tls_io_config.port = port_num;
  /* XIO_CREATE */
  XIO_Instance = xio_create(io_interface_description, &tls_io_config);
  if(XIO_Instance==NULL) {
    AZURE_PRINTF("Err: xio_create\r\n");
    OTAStatus = OTA_STATUS_ERROR;
    goto FOTA_exit;
  }
  AZURE_PRINTF("OK xio_create\r\n");

  /* XIO_SETOPTION */
  if(xio_setoption(XIO_Instance,  "TrustedCerts", certificates)!=0) {
    AZURE_PRINTF("Err: xio_setoption\r\n");
    OTAStatus = OTA_STATUS_ERROR;
    goto FOTA_exit;
  }
  AZURE_PRINTF("OK xio_setoption\r\n");

  /* XIO_OPEN */
  OTAStatus=OTA_STATUS_NULL;
  if(xio_open(XIO_Instance, onOpenCompleteFOTA, NULL, onBytesReceivedFOTA, &TypeOfRequest, onIoErrorFOTA, NULL)!=0) {
    AZURE_PRINTF("Err: xio_open\r\n");
    OTAStatus = OTA_STATUS_ERROR;
    goto FOTA_exit;
  }
  AZURE_PRINTF("OK xio_open\r\n");
  
  /* XIO_SEND HEAD Request  */
  OTAStatus=OTA_STATUS_START;
  TypeOfRequest= HTTP_HEAD_REQUEST;
  sprintf(buf,"HEAD %s HTTP/1.1\r\nHost: %s\r\nRange: bytes=0-1\r\n\r\n",path,hostname);
  if(xio_send(XIO_Instance, buf, strlen(buf),  onSendCompleteFOTA, &TypeOfRequest)!=0) {
    AZURE_PRINTF("Err: xio_send\r\n");
    OTAStatus = OTA_STATUS_ERROR;
    goto FOTA_exit;
  } else {
    AZURE_PRINTF("OK xio_send HEAD Request\r\n");
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
      AZURE_PRINTF("OK xio_send GET (%ld/%ld) Request\r",ChunkNum,(FullOTASize+FOTA_CHUNK_SIZE-1)/FOTA_CHUNK_SIZE-1);
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
    AZURE_PRINTF("OK xio_close\r\n");
  }

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
  * @param Delay: specifies the delay time length, in milliseconds.
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
#if (defined USE_STM32F4XX_NUCLEO) || (defined USE_STM32L4XX_NUCLEO) || (defined USE_STM32F3XX_NUCLEO) || \
     (defined USE_STM32L1XX_NUCLEO) || (defined USE_STM32F1XX_NUCLEO)
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
#elif (defined USE_STM32L0XX_NUCLEO) || (defined USE_STM32F0XX_NUCLEO)
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
#endif

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

/*-------------------------------------------------------------------*/


/**
  * @brief  Initializes the lwIP stack
  * @param  AzureEthConfiguration_t EthConfiguration Ethernet configuration
  * @retval None
  */
static void Netif_Config(AzureEthConfiguration_t EthConfiguration)
{
  ip_addr_t ipaddr;
  ip_addr_t netmask;
  ip_addr_t gw;	
  
  /* IP address setting */
  IP4_ADDR(&ipaddr, EthConfiguration.ipaddress[0], EthConfiguration.ipaddress[1], EthConfiguration.ipaddress[2], EthConfiguration.ipaddress[3]);
  IP4_ADDR(&netmask, NETMASK_ADDR0, NETMASK_ADDR1 , NETMASK_ADDR2, NETMASK_ADDR3);
  IP4_ADDR(&gw, EthConfiguration.gwaddress[0], EthConfiguration.gwaddress[1], EthConfiguration.gwaddress[2], EthConfiguration.gwaddress[3]);
  
  /* - netif_add(struct netif *netif, struct ip_addr *ipaddr,
  struct ip_addr *netmask, struct ip_addr *gw,
  void *state, err_t (* init)(struct netif *netif),
  err_t (* input)(struct pbuf *p, struct netif *netif))
  
  Adds your network interface to the netif_list. Allocate a struct
  netif and pass a pointer to this structure as the first argument.
  Give pointers to cleared ip_addr structures when using DHCP,
  or fill them with sane numbers otherwise. The state pointer may be NULL.
  
  The init function pointer must point to a initialization function for
  your ethernet netif interface. The following code illustrates it's use.*/
  
  netif_add(&gnetif, &ipaddr, &netmask, &gw, NULL, &ethernetif_init, &tcpip_input);
  
  /*  Registers the default network interface. */
  netif_set_default(&gnetif);
  
  if (netif_is_link_up(&gnetif))
  {
    /* When the netif is fully configured this function must be called.*/
    netif_set_up(&gnetif);
  }
  else
  {
    /* When the netif link is down this function must be called */
    netif_set_down(&gnetif);
  }
}

/**
  * @brief  Notify the User about the network interface config status
  * @param  netif: the network interface
  * @param  EthConfiguration: the Ethernet configuration
  * @retval None
  */
void User_notification(struct netif *netif,AzureEthConfiguration_t EthConfiguration) 
{
  if (netif_is_up(netif))
  {
    /* Update DHCP state machine */
    if ( EthConfiguration.use_dhcp )
      DHCP_state = DHCP_START;
  }
  else
  {  
    /* Update DHCP state machine */
    if ( EthConfiguration.use_dhcp )
      DHCP_state = DHCP_LINK_DOWN;
  } 
}

/**
* @brief  DHCP Process
* @param  argument: network interface
* @retval None
*/
void DHCP_init(void const * argument)
{
  struct netif *netif = (struct netif *) argument;
  ip_addr_t ipaddr;
  ip_addr_t netmask;
  ip_addr_t gw;
  struct dhcp *dhcp;
  uint32_t IPaddress;
  
  for (;;)
  {
    switch (DHCP_state)
    {
    case DHCP_START:
      {
        ip_addr_set_zero_ip4(&netif->ip_addr);
        ip_addr_set_zero_ip4(&netif->netmask);
        ip_addr_set_zero_ip4(&netif->gw);       
        dhcp_start(netif);
        DHCP_state = DHCP_WAIT_ADDRESS;
      }
      break;
      
    case DHCP_WAIT_ADDRESS:
      {                
        if (dhcp_supplied_address(netif)) 
        {
          DHCP_state = DHCP_ADDRESS_ASSIGNED;	
          /* Read the new IP address */
          IPaddress = netif->ip_addr.addr;
          AZURE_PRINTF("IPaddress = %ld.%ld.%ld.%ld\r\n",
                 IPaddress&0xFF,
                 (IPaddress>> 8)&0xFF,
                 (IPaddress>>16)&0xFF,
                 (IPaddress>>24)&0xFF                 
                   );
          
          return;
        }
        else
        {
          dhcp = (struct dhcp *)netif_get_client_data(netif, LWIP_NETIF_CLIENT_DATA_INDEX_DHCP);

          /* DHCP timeout */
          if (dhcp->tries > MAX_DHCP_TRIES)
          {
            DHCP_state = DHCP_TIMEOUT;
            
            /* Stop DHCP */
            dhcp_stop(netif);
            
            /* Static address used */
            IP_ADDR4(&ipaddr, IP_ADDR0 ,IP_ADDR1 , IP_ADDR2 , IP_ADDR3 );
            IP_ADDR4(&netmask, NETMASK_ADDR0, NETMASK_ADDR1, NETMASK_ADDR2, NETMASK_ADDR3);
            IP_ADDR4(&gw, GW_ADDR0, GW_ADDR1, GW_ADDR2, GW_ADDR3);
            netif_set_addr(netif, ip_2_ip4(&ipaddr), ip_2_ip4(&netmask), ip_2_ip4(&gw));
            
          }
        }
      }
      break;
  case DHCP_LINK_DOWN:
    {
      /* Stop DHCP */
      dhcp_stop(netif);
      DHCP_state = DHCP_OFF; 
    }
    break;
    default: break;
    }
    
    /* wait 250 ms */
    osDelay(250);
  }
}
                                     
/**
* @brief  If NFC is present, read MAC and IP/GW addresses
* @param  EthConfiguration: Pointer to the Ethernet configuration
* @retval None
*/
static void ReadNFC(AzureEthConfiguration_t *EthConfiguration)
{
  NDEF_Text_info_t TextInfo;
  char *StartAddress;
  char InputString[18];

  if(TT4_ReadTextToken(&TextInfo) == SUCCESS) {
    //AZURE_PRINTF("Text content: '%s'\r\n",TextInfo.text);

    if ( (StartAddress=strstr(TextInfo.text,"MAC")) != NULL ) {
      StartAddress+=4;
      strncpy(InputString,StartAddress,17);
      InputString[17]='\0';
      sscanf(InputString,"%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
             macAddress  ,macAddress+1,macAddress+2,
             macAddress+3,macAddress+4,macAddress+5);

      memcpy(EthConfiguration->macAddress,macAddress,6);
    } else {
      AZURE_PRINTF("MAC address not found\r\n");
    }

    if ( (StartAddress=strstr(TextInfo.text,"IP")) != NULL ){
      StartAddress+=3;
      strncpy(InputString,StartAddress,15);
      InputString[15]='\0';
      if ( !strcmp(InputString,"dynamic") || !strcmp(InputString,"dhcp") ){
          EthConfiguration->use_dhcp = true;
          memset(EthConfiguration->ipaddress,0,4*4);
      } else {
        EthConfiguration->use_dhcp = false;
        sscanf(InputString,"%d.%d.%d.%d",&EthConfiguration->ipaddress[0],&EthConfiguration->ipaddress[1],
               &EthConfiguration->ipaddress[2],&EthConfiguration->ipaddress[3]);
      }
    } else {
      AZURE_PRINTF("IP address not found\r\n");
    }
    
    if (EthConfiguration->use_dhcp == false) 
    {
      if ( (StartAddress=strstr(TextInfo.text,"GW")) != NULL ){
        StartAddress+=3;
        strncpy(InputString,StartAddress,15);
        InputString[15]='\0';
        sscanf(InputString,"%d.%d.%d.%d",&EthConfiguration->gwaddress[0],&EthConfiguration->gwaddress[1],
               &EthConfiguration->gwaddress[2],&EthConfiguration->gwaddress[3]);
      } else {
        AZURE_PRINTF("GW address not found\r\n");
      }
    }
    
  } else {
    AZURE_PRINTF("Read nothing\r\n");
  }

  AZURE_PRINTF("    MAC addr is %02x:%02x:%02x:%02x:%02x:%02x\r\n",
               macAddress[0],macAddress[1],macAddress[2],
               macAddress[3],macAddress[4],macAddress[5]);

  if(EthConfiguration->use_dhcp == false) {
    AZURE_PRINTF("    IP addr is %d.%d.%d.%d\r\n",
                 EthConfiguration->ipaddress[0],EthConfiguration->ipaddress[1],EthConfiguration->ipaddress[2],EthConfiguration->ipaddress[3]);
    AZURE_PRINTF("    GW addr is %d.%d.%d.%d\r\n",
                 EthConfiguration->gwaddress[0],EthConfiguration->gwaddress[1],EthConfiguration->gwaddress[2],EthConfiguration->gwaddress[3]);
  } else {
    AZURE_PRINTF("    IP addr with DHCP\r\n");
  }
}

/**
* @brief  Read MAC and IP/GW addresses from serial
* @param  EthConfiguration: Pointer to the Ethernet configuration
* @retval None
*/
static void ReadUART(AzureEthConfiguration_t *EthConfiguration)
{
  char InputString[18];

  AZURE_PRINTF("Enter MAC address (: separated)\r\n");
  scanf("%s",InputString);
  sscanf(InputString,"%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
         macAddress  ,macAddress+1,macAddress+2,
         macAddress+3,macAddress+4,macAddress+5);
  memcpy(EthConfiguration->macAddress,macAddress,6);

  AZURE_PRINTF("Enter IP address (. separated or 'dhcp')\r\n");
  scanf("%s",InputString);
  
  if ( strstr(InputString,"dhcp") || strstr(InputString,"DHCP") )
  {
    EthConfiguration->use_dhcp = true;
    memset(EthConfiguration->ipaddress,0,4*4);
    memset(EthConfiguration->gwaddress,0,4*4);
  }
  else
  {
    EthConfiguration->use_dhcp = false;

    sscanf(InputString,"%d.%d.%d.%d",
           &EthConfiguration->ipaddress[0],&EthConfiguration->ipaddress[1],&EthConfiguration->ipaddress[2],&EthConfiguration->ipaddress[3]);

    AZURE_PRINTF("Enter GW address (. separated)\r\n");
    scanf("%s",InputString);
    sscanf(InputString,"%d.%d.%d.%d",
           &EthConfiguration->gwaddress[0],&EthConfiguration->gwaddress[1],&EthConfiguration->gwaddress[2],&EthConfiguration->gwaddress[3]);
  }
}

STRING_HANDLE platform_get_platform_info(void)
{
  return STRING_construct("(STM32Cube)");
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

  EraseInitStruct.TypeErase = TYPEERASE_SECTORS;
  EraseInitStruct.VoltageRange = VOLTAGE_RANGE_3;
  EraseInitStruct.Sector = FLASH_SECTOR_23;
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

/******************* (C) COPYRIGHT 2017 STMicroelectronics *****END OF FILE****/
