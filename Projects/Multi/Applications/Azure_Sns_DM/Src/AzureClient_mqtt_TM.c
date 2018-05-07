/**
  ******************************************************************************
  * @file    AzureClient_mqtt_TM.c
  * @author  Central LAB
  * @version V3.2.2
  * @date    22-Jan-2018
  * @brief   Main MQTT application for Telemetry
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
#include <stdio.h>
#include "STM32CubeInterface.h"
#include "iothub_client_ll.h"
#include "iothub_message.h"
#include "azure_c_shared_utility/crt_abstractions.h"
#include "azure_c_shared_utility/platform.h"
#include "iothubtransportmqtt.h"
#include "AzureClient_mqtt_TM.h"
#ifdef USE_MBED_TLS
#include "certificates.h"
#endif /* USE_MBED_TLS */
#ifdef AZURE_ENABLE_REGISTRATION
  #include "RegistrationAgent.h"
#endif /* AZURE_ENABLE_REGISTRATION */

#include "serializer.h"
#include "serializer_devicetwin.h"
#include "jsondecoder.h"

#ifdef USE_MBED_TLS
  #include "azure_c_shared_utility/tls_config.h"
  #if !defined(MBEDTLS_CONFIG_FILE)
    #include "mbedtls/config.h"
  #else
    #include MBEDTLS_CONFIG_FILE
  #endif
#endif

/* Connection string containing Hostname, Device Id & Device Key */
char *connectionString = NULL;

DEFINE_ENUM_STRINGS(FIRMWARE_UPDATE_STATUS, FIRMWARE_UPDATE_STATUS_VALUES)


BEGIN_NAMESPACE(AzureModel);

DECLARE_MODEL(Azure1_t,
/* Event data: Temperature, Humidity... */
WITH_DATA(ascii_char_ptr, deviceId),
WITH_DATA(int, messageId),
WITH_DATA(float, Temperature),
WITH_DATA(float, Humidity),
WITH_DATA(float, Pressure),
WITH_DATA(int, accX),
WITH_DATA(int, accY),
WITH_DATA(int, accZ),
WITH_DATA(int, gyrX),
WITH_DATA(int, gyrY),
WITH_DATA(int, gyrZ),
WITH_DATA(int, magX),
WITH_DATA(int, magY),
WITH_DATA(int, magZ),
WITH_DATA(int, HWCounter),
WITH_DATA(ascii_char_ptr, HWOrientation),
WITH_DATA(EDM_DATE_TIME_OFFSET, ts),
/* Methods */
WITH_METHOD(Reboot),
WITH_METHOD(Quit),
/* Commands */
WITH_ACTION(Pause),
WITH_ACTION(Play),
WITH_ACTION(LedOn),
WITH_ACTION(LedOff),
WITH_ACTION(LedBlink),
/* Desired Properties */
WITH_DESIRED_PROPERTY(int, DesiredTelemetryInterval, ChangeTelemetryInterval),
WITH_DESIRED_PROPERTY(int, DesiredHWMode, ChangeHWMode),
/* Reported Properties */
/* Register Commands with IoT Hub */
WITH_REPORTED_PROPERTY(ascii_char_ptr_no_quotes, SupportedCommands),
/* Register Direct Methods with IoT Hub */
WITH_REPORTED_PROPERTY(ascii_char_ptr_no_quotes, SupportedMethods),
/* Telemetry Interval in Seconds... value from 1 to: 0xFFFF/2000 = About 30 Seconds */
WITH_REPORTED_PROPERTY(int,TelemetryInterval),
WITH_REPORTED_PROPERTY(ascii_char_ptr,AzureStatus),
WITH_REPORTED_PROPERTY(int, ActiveHWMode),
WITH_REPORTED_PROPERTY(ascii_char_ptr, AzureFwVersion)
);

END_NAMESPACE(AzureModel);

Azure1_t *Azure1;

typedef struct EVENT_INSTANCE_TAG
{
  IOTHUB_MESSAGE_HANDLE messageHandle;
  size_t messageTrackingId;  // For tracking the messages within the user callback.
  void *this;  // For Memory management (free)
} EVENT_INSTANCE;

IOTHUB_CLIENT_LL_HANDLE iotHubClientHandle;

#define AZURE_LED_MODE \
  AZURE_LED_ON, \
  AZURE_LED_OFF, \
  AZURE_LED_BLINK
DEFINE_ENUM(AZURE_LED, AZURE_LED_MODE)


static bool g_continueRunning;
static bool g_reboot     = false;
static AZURE_LED LedMode = AZURE_LED_BLINK;
/* Counters for Sent/Received Status */
static size_t StatusBackCount = 0;
static size_t SentStatusCount = 0;
/* Counters for Sent/Received Messages */
static size_t MessagesBackCount = 0;
static size_t SentMessagesCount = 0;

static volatile int32_t ReceivedDeviceTwinProperty=0;

uint32_t HWEventCounter[5] = {0 /* FreeFall */,
                              0 /* Single Tap */,
                              0 /* Double Tap */,
                              0 /* Wake Up */,
                              0 /* Tilt */};

static void SendSNSData(void);

static void Send6DOrientationData(void);

static EDM_DATE_TIME_OFFSET GetDateTimeOffset(time_t time)
{
  struct tm *newTime;
  EDM_DATE_TIME_OFFSET dateTimeOffset;
  newTime = gmtime(&time);
  dateTimeOffset.dateTime = *newTime;
  dateTimeOffset.fractionalSecond = 0;
  dateTimeOffset.hasFractionalSecond = 0;
  dateTimeOffset.hasTimeZone = 0;
  dateTimeOffset.timeZoneHour = 0;
  dateTimeOffset.timeZoneMinute = 0;
  return dateTimeOffset;
}

/* Handles for Commands */

/**
 * @brief Implementation of Pause Command
 * @param Azure1_t *Azure1 Pointer to the Model instance
 * @retval EXECUTE_COMMAND_RESULT Command Result
 */
EXECUTE_COMMAND_RESULT Pause(Azure1_t *Azure1)
{
  (void)(Azure1);
  AZURE_PRINTF("Received Pause command\r\n");
  StopTimer1();
  return EXECUTE_COMMAND_SUCCESS;
}

/**
 * @brief Implementation of Play Command
 * @param Azure1_t *Azure1 Pointer to the Model instance
 * @retval EXECUTE_COMMAND_RESULT Command Result
 */
EXECUTE_COMMAND_RESULT Play(Azure1_t *Azure1)
{
  (void)(Azure1);
  AZURE_PRINTF("Received Play command\r\n");
  StartTimer1();
  return EXECUTE_COMMAND_SUCCESS;
}

/**
 * @brief Implementation of LedOn Command
 * @param Azure1_t *Azure1 Pointer to the Model instance
 * @retval EXECUTE_COMMAND_RESULT Command Result
 */
EXECUTE_COMMAND_RESULT LedOn(Azure1_t *Azure1)
{
  (void)(Azure1);
  AZURE_PRINTF("Received LedOn command\r\n");
  LedMode = AZURE_LED_ON;
  BSP_LED_On(LED2);
  return EXECUTE_COMMAND_SUCCESS;
}

/**
 * @brief Implementation of LedOff Command
 * @param Azure1_t *Azure1 Pointer to the Model instance
 * @retval EXECUTE_COMMAND_RESULT Command Result
 */
EXECUTE_COMMAND_RESULT LedOff(Azure1_t *Azure1)
{
  (void)(Azure1);
  AZURE_PRINTF("Received LedOff command\r\n");
  LedMode = AZURE_LED_OFF;
  BSP_LED_Off(LED2);
  return EXECUTE_COMMAND_SUCCESS;
}

/**
 * @brief Implementation of LedBlink Command
 * @param Azure1_t *Azure1 Pointer to the Model instance
 * @retval EXECUTE_COMMAND_RESULT Command Result
 */
EXECUTE_COMMAND_RESULT LedBlink(Azure1_t *Azure1)
{
  (void)(Azure1);
  AZURE_PRINTF("Received LedBlink command\r\n");
  LedMode = AZURE_LED_BLINK;
  return EXECUTE_COMMAND_SUCCESS;
}

/* Handles for direct methods */

/**
 * @brief Implementation of Reboot Direct Method
 * @param Azure1_t *Azure1 Pointer to the Model instance
 * @retval METHODRETURN_HANDLE Direct Method Result
 */
METHODRETURN_HANDLE Reboot(Azure1_t *Azure1)
{
  (void)(Azure1);
  METHODRETURN_HANDLE result = MethodReturn_Create(201, "\"Rebooting\"");
  AZURE_PRINTF("Received Reboot request\r\n");
  g_reboot = true;
  return result;
}

/**
 * @brief Implementation of Quit Direct Method
 * @param Azure1_t *Azure1 Pointer to the Model instance
 * @retval METHODRETURN_HANDLE Direct Method Result
 */
METHODRETURN_HANDLE Quit(Azure1_t *Azure1)
{
  (void)(Azure1);
  METHODRETURN_HANDLE result = MethodReturn_Create(201,  "\"I will stop\"");
  g_continueRunning = false;
  AZURE_PRINTF("Received Quit request\r\n");
  return result;
}

/* Handle for Desired Properties */

/**
 * @brief Function for changing the Telemetry Interval
 * @param void *argument Pointer to the Model instance
 * @retval None
 */
void ChangeTelemetryInterval(void* argument)
{
  Azure1_t *Azure = argument;
  if((Azure1->DesiredTelemetryInterval>0) & (Azure1->DesiredTelemetryInterval<=30)){
    Azure1->TelemetryInterval = Azure1->DesiredTelemetryInterval;
    TargetBoardFeatures.TIM_CC1_Pulse = Azure1->DesiredTelemetryInterval*2000 /* 2Hz Timer Frequency */;
    ReportState(Running);
  } else {
    AZURE_PRINTF("Err: Received a Not Allowed desired Telemetry Interval=%d (1<=Allowed<=30)\r\n",Azure->DesiredTelemetryInterval);
  }
}

/**
 * @brief Function for changing the HW Mode
 * @param void *argument Pointer to the Model instance
 * @retval None
 */
void ChangeHWMode(void* argument)
{
  Azure1_t *Azure = argument;
  if((Azure1->DesiredHWMode>0) & (Azure1->DesiredHWMode<8)){
    switch(Azure1->DesiredHWMode) {
      case 1:
        EnableHWPedometer();
        Azure1->HWCounter = GetStepHWPedometer();
      break;
      case 2:
        EnableHWFreeFall();
        Azure1->HWCounter = HWEventCounter[0];
      break;
      case 3:
        EnableHWSingleTap();
        Azure1->HWCounter = HWEventCounter[1];
      break;
      case 4:
        EnableHWDoubleTap();
        Azure1->HWCounter = HWEventCounter[2];
      break;
      case 5:
        EnableHWWakeUp();
        Azure1->HWCounter = HWEventCounter[3];
      break;
      case 6:
        EnableHWTilt();
        Azure1->HWCounter = HWEventCounter[4];
      break;
      case 7:
        EnableHWOrientation6D();
        Azure1->HWCounter =0; /* Not Used for 6D Orientation */
      break;
    }
    /* Change the Active HW Mode */
    Azure->ActiveHWMode = Azure->DesiredHWMode;
    ReportState(Running);
  } else {
    AZURE_PRINTF("Err: Received a Not Allowed desired HW mode=%d (1<=Allowed<=7)\r\n",Azure->DesiredHWMode);
  }
}

/**
  * @brief  Fill the Model Instance(date, sensors data)
  * @param  None
  * @retval None
  */
static void FillTheModelInstance(void)
{
  time_t now;
  float SensorValueT;
  float SensorValueH;
  float SensorValueP;
  uint8_t Status;
  SensorAxes_t ACC_Value;
  SensorAxes_t GYR_Value;
  SensorAxes_t MAG_Value;

  /* Read or Emulate the Sensors */
  if(!TargetBoardFeatures.EmulateSns) {
    if((TargetBoardFeatures.SnsAltFunc ? BSP_TEMPERATURE_IsInitialized_IKS01A2 : BSP_TEMPERATURE_IsInitialized)(TargetBoardFeatures.HandleTempSensors[0],&Status)==COMPONENT_OK){
      (TargetBoardFeatures.SnsAltFunc ? BSP_TEMPERATURE_Get_Temp_IKS01A2 : BSP_TEMPERATURE_Get_Temp)(TargetBoardFeatures.HandleTempSensors[0],(float *)&SensorValueT);
    }

    if(TargetBoardFeatures.HandleHumSensor) {
      if((TargetBoardFeatures.SnsAltFunc ? BSP_HUMIDITY_IsInitialized_IKS01A2 : BSP_HUMIDITY_IsInitialized)(TargetBoardFeatures.HandleHumSensor,&Status)==COMPONENT_OK){
        (TargetBoardFeatures.SnsAltFunc ? BSP_HUMIDITY_Get_Hum_IKS01A2 : BSP_HUMIDITY_Get_Hum)(TargetBoardFeatures.HandleHumSensor,(float *)&SensorValueH);
      }
    }

    if(TargetBoardFeatures.HandlePressSensor) {
      if((TargetBoardFeatures.SnsAltFunc ? BSP_PRESSURE_IsInitialized_IKS01A2 : BSP_PRESSURE_IsInitialized)(TargetBoardFeatures.HandlePressSensor,&Status)==COMPONENT_OK){
        (TargetBoardFeatures.SnsAltFunc ? BSP_PRESSURE_Get_Press_IKS01A2 :BSP_PRESSURE_Get_Press)(TargetBoardFeatures.HandlePressSensor,(float *)&SensorValueP);
      }
    }

    /* Read the Acc values */
    (TargetBoardFeatures.SnsAltFunc ? BSP_ACCELERO_Get_Axes_IKS01A2 : BSP_ACCELERO_Get_Axes)(TargetBoardFeatures.HandleAccSensor,&ACC_Value);

    /* Read the Gyro values */
    (TargetBoardFeatures.SnsAltFunc ? BSP_GYRO_Get_Axes_IKS01A2 : BSP_GYRO_Get_Axes)(TargetBoardFeatures.HandleGyroSensor,&GYR_Value);

    /* Read the Mag values */
    (TargetBoardFeatures.SnsAltFunc ? BSP_MAGNETO_Get_Axes_IKS01A2 : BSP_MAGNETO_Get_Axes)(TargetBoardFeatures.HandleMagSensor,&MAG_Value);    
  } else {
    /* Emulatation IkS01A1/IKS01A2 */
    ACC_Value.AXIS_X =  100 + (((int32_t)rand())&0x5FF);
    ACC_Value.AXIS_Y =  300 - (((int32_t)rand())&0x5FF);
    ACC_Value.AXIS_Z = -200 + (((int32_t)rand())&0x5FF);

    GYR_Value.AXIS_X =  30000 + (((int32_t)rand())&0x4FFFF);
    GYR_Value.AXIS_Y =  30000 - (((int32_t)rand())&0x4FFFF);
    GYR_Value.AXIS_Z = -30000 + (((int32_t)rand())&0x4FFFF);

    MAG_Value.AXIS_X =    (((int32_t)rand())&0x5FF);
    MAG_Value.AXIS_Y =  - (((int32_t)rand())&0x5FF);
    MAG_Value.AXIS_Z =    (((int32_t)rand())&0x5FF);

    SensorValueT  = 26.0 + (((int16_t)rand())&0x14);
    SensorValueH  = 50.0 + (((int16_t)rand())&0x1F);
    SensorValueP  = 1000.0 + (((int16_t)rand())&0xF);
  }

  /* Read the Time  from RTC */
  now = TimingSystemGetSystemTime();
  /* Time Stamp */
  Azure1->ts = GetDateTimeOffset(now);

  /* Update the Azure1 Model */
  Azure1->Temperature = SensorValueT;
  Azure1->Humidity  = SensorValueH;
  Azure1->Pressure  = SensorValueP;

  Azure1->accX = ACC_Value.AXIS_X;
  Azure1->accY = ACC_Value.AXIS_Y;
  Azure1->accZ = ACC_Value.AXIS_Z;

  Azure1->gyrX = GYR_Value.AXIS_X;
  Azure1->gyrY = GYR_Value.AXIS_Y;
  Azure1->gyrZ = GYR_Value.AXIS_Z;

  Azure1->magX = MAG_Value.AXIS_X;
  Azure1->magY = MAG_Value.AXIS_Y;
  Azure1->magZ = MAG_Value.AXIS_Z;
}

/**
 * @brief CallBack received for Device Twin status update
 * @param int status_code Status received
 * @param void* userContextCallback unused
 * @retval None
 */
static void DeviceTwinCallback(int status_code, __attribute__((unused)) void* userContextCallback)
{
  StatusBackCount++;
  AZURE_PRINTF("-->DeviceTwin CallBack [%d]: Status_code = %d\r\n", StatusBackCount, status_code);
}

/**
 * @brief CallBack received at the beginning on when there is a change on desired properties
 * @param DEVICE_TWIN_UPDATE_STATE status_code Status received
 * @param unsigned char* payload Payload received (json)
 * @param size_t size size of the Payload received
 * @param void* userContextCallback pointer to the model instance
 * @retval None
 */
static void DeviceTwinCallbackStatus(DEVICE_TWIN_UPDATE_STATE status_code,  const unsigned char* payload, size_t size, void* userContextCallback)
{
  /* Only for Debug */
  //AZURE_PRINTF("DeviceTwinCallbackStatus Method payload: %.*s\r\nStatus_code = %d\r\n", size,(const char*)payload,status_code);

  /* query the DeviceTwin Status */
  {
    JSON_DECODER_RESULT JSONDec;
    MULTITREE_HANDLE multiTreeHandle;
    char* temp = malloc(size + 1);
    if (temp == NULL) {
      AZURE_PRINTF("Err: failed to malloc\r\n");
      return;
    }
    /* We need to add the missing termination char */
    (void)memcpy(temp, payload, size);
    temp[size] = '\0';
    if((JSONDec = JSONDecoder_JSON_To_MultiTree(temp,&multiTreeHandle))!=JSON_DECODER_OK) {
      AZURE_PRINTF("Err: Decoding JSON Code=%d\r\n",JSONDec);
      free(temp);
      return;
    } else {
      AZURE_PRINTF("JSON Decoded\r\n");
    }

    /* Search the Reported Properties Only at the beginning */
    if(ReceivedDeviceTwinProperty==0){
      MULTITREE_HANDLE childHandle;
      ReceivedDeviceTwinProperty=1;
      /* Check also the Desired Properties */
      if(MultiTree_GetChildByName(multiTreeHandle, "desired", &childHandle)== MULTITREE_OK) {
        MULTITREE_HANDLE childHandle2;
        AZURE_PRINTF("Desired Property Found\r\n");
        if(MultiTree_GetChildByName(childHandle, "DesiredTelemetryInterval", &childHandle2) != MULTITREE_OK) {
          AZURE_PRINTF("DesiredTelemetryInterval Desired Property not Found\r\n");
        } else {
          void const *ResultInterval;
          //AZURE_PRINTF("AzureStatus reported Property Found\r\n");
          if(MultiTree_GetValue(childHandle2, &ResultInterval)!= MULTITREE_OK) {
            AZURE_PRINTF("Err: Reading the DesiredTelemetryInterval Desired Property value\r\n");
          } else {
            Azure1_t *Azure = userContextCallback;
            Azure->DesiredTelemetryInterval= atoi(((char *) ResultInterval));
            AZURE_PRINTF("Desired Telemetry Interval= %d\r\n", Azure->DesiredTelemetryInterval);
            ChangeTelemetryInterval(userContextCallback);
          }
        }
        if(MultiTree_GetChildByName(childHandle, "DesiredHWMode", &childHandle2) != MULTITREE_OK) {
          AZURE_PRINTF("DesiredHWMode Desired Property not Found\r\n");
        } else {
          void const *ResultHWMode;
          //AZURE_PRINTF("AzureStatus reported Property Found\r\n");
          if(MultiTree_GetValue(childHandle2, &ResultHWMode)!= MULTITREE_OK) {
            AZURE_PRINTF("Err: Reading the DesiredHWMode Desired Property value\r\n");
          } else {
            Azure1_t *Azure = userContextCallback;
            Azure->DesiredHWMode= atoi(((char *) ResultHWMode));
            AZURE_PRINTF("Desired HW Mode= %d\r\n", Azure->DesiredHWMode);
            ChangeHWMode(userContextCallback);
          }
        }
      }
    } else {
      /* if we are not at the beginning... there is a change on the Desided properties */
      MULTITREE_HANDLE childHandle;
      if(MultiTree_GetChildByName(multiTreeHandle, "DesiredTelemetryInterval", &childHandle) != MULTITREE_OK) {
        AZURE_PRINTF("DesiredTelemetryInterval Desired Property not Found\r\n");
      } else {
        void const *ResultInterval;
        if(MultiTree_GetValue(childHandle, &ResultInterval)!= MULTITREE_OK) {
          AZURE_PRINTF("Err: Reading the DesiredTelemetryInterval Desired Property value\r\n");
        } else {
          Azure1_t *Azure = userContextCallback;
          Azure->DesiredTelemetryInterval= atoi(((char *) ResultInterval));
          AZURE_PRINTF("Received a new Desired Telemetry Interval= %d\r\n", Azure->DesiredTelemetryInterval);
          ChangeTelemetryInterval(userContextCallback);
        }
      }
      if(MultiTree_GetChildByName(multiTreeHandle, "DesiredHWMode", &childHandle) != MULTITREE_OK) {
        AZURE_PRINTF("DesiredHWMode Desired Property not Found\r\n");
      } else {
        void const *ResultHWMode;
        if(MultiTree_GetValue(childHandle, &ResultHWMode)!= MULTITREE_OK) {
          AZURE_PRINTF("Err: Reading the DesiredHWMode Desired Property value\r\n");
        } else {
          Azure1_t *Azure = userContextCallback;
          Azure->DesiredHWMode= atoi(((char *) ResultHWMode));
          AZURE_PRINTF("Desired HW Mode= %d\r\n", Azure->DesiredHWMode);
          ChangeHWMode(userContextCallback);
        }
      }
    }
    MultiTree_Destroy(multiTreeHandle);
    free(temp);
  }
}

/**
 * @brief Function for reporting the Model Status
 * @param FIRMWARE_UPDATE_STATUS status that we want send (Running/Downloading...)
 * @retval None
 */
void ReportState(FIRMWARE_UPDATE_STATUS status)
{
  unsigned char *buffer;
  size_t         bufferSize;

  /* Fill the report State */
  sprintf(Azure1->AzureStatus,"%s",ENUM_TO_STRING(FIRMWARE_UPDATE_STATUS,status));

  /*serialize the model using SERIALIZE_REPORTED_PROPERTIES */
  if (SERIALIZE_REPORTED_PROPERTIES(&buffer, &bufferSize, *Azure1) != CODEFIRST_OK) {
    AZURE_PRINTF("Err: Serializing Reported State\r\n");
  } else {
    /* send the data up stream*/
    if(IoTHubClient_LL_SendReportedState(iotHubClientHandle, buffer, bufferSize, DeviceTwinCallback, NULL)!= IOTHUB_CLIENT_OK) {
      AZURE_PRINTF("Err: Failure Sending Reported State\r\n");
    } else {
      SentStatusCount++;
      AZURE_PRINTF("Ok reported State [%d]: %s\r\n", SentStatusCount,Azure1->AzureStatus);
    }
    free(buffer);
  }
}

/* @brief Callback called when one C2D message is received
 * @param IOTHUB_MESSAGE_HANDLE message Pointer to the received message
 * @param void* userContextCallback pointer to the model instance
 * @retval IOTHUBMESSAGE_DISPOSITION_RESULT Results of the messagge (Abandoned/Accepted/...)
 */
static IOTHUBMESSAGE_DISPOSITION_RESULT ReceiveMessageCallback(IOTHUB_MESSAGE_HANDLE message, void* userContextCallback)
{
  IOTHUBMESSAGE_DISPOSITION_RESULT result;
  const unsigned char* buffer;
  size_t size;
  if (IoTHubMessage_GetByteArray(message, &buffer, &size) != IOTHUB_MESSAGE_OK){
    AZURE_PRINTF("unable to retrieve the message data\r\n");
    result = IOTHUBMESSAGE_ABANDONED;
  } else {
    /*buffer is not zero terminated*/
    char* temp = malloc(size + 1);
    if (temp == NULL) {
      AZURE_PRINTF("Err: failed to malloc\r\n");
      result = IOTHUBMESSAGE_ABANDONED;
    } else {
      EXECUTE_COMMAND_RESULT executeCommandResult;

      AZURE_PRINTF("Received Message with Data: <<<%.*s>>> & Size=%d\r\n", (int)size, buffer, (int)size);

      (void)memcpy(temp, buffer, size);
      temp[size] = '\0';
      executeCommandResult = EXECUTE_COMMAND(userContextCallback, temp);
      result =
          (executeCommandResult == EXECUTE_COMMAND_ERROR) ? IOTHUBMESSAGE_ABANDONED :
          (executeCommandResult == EXECUTE_COMMAND_SUCCESS) ? IOTHUBMESSAGE_ACCEPTED :
          IOTHUBMESSAGE_REJECTED;
      free(temp);
    }
  }
  return result;
}

/* @brief Callback received when a message is received from IoT
 * @param IOTHUB_CLIENT_CONFIRMATION_RESULT result confirmation result from IoTs
 * @param void* userContextCallback pointer to event instance
 * @retval None
 */
static void SendConfirmationCallback(IOTHUB_CLIENT_CONFIRMATION_RESULT result, void* userContextCallback)
{
  EVENT_INSTANCE* eventInstance = (EVENT_INSTANCE*)userContextCallback;

  AZURE_PRINTF("Confirmation received for message [%d] with result = %s\r\n", eventInstance->messageTrackingId, ENUM_TO_STRING(IOTHUB_CLIENT_CONFIRMATION_RESULT, result));
  MessagesBackCount++;
  free(eventInstance->this);
}


extern bool size_err_mesg_flag;
/**
  * @brief  Send Sensors' Data to IoT Hub
  * @param  None
  * @retval None
  */
static void SendSNSData(void)
{
  EVENT_INSTANCE *messages;
  unsigned char* destination;
  size_t destinationSize;

	if (size_err_mesg_flag == true)
	{
		size_err_mesg_flag = false;
		AZURE_PRINTF("--->>>> ERROR DATA PENDING EXCEED BUFFER SIZE, DISCHARGED!!\r\n");
	}
		
  /* Read the Data from the Sensors */
  FillTheModelInstance();

  messages = (EVENT_INSTANCE *) calloc(1,sizeof(EVENT_INSTANCE));
  if(messages==NULL) {
    AZURE_PRINTF("Err: Allocating Memory for messages to IoT Hub\r\n");
    while(1) {
      ;
    }
  } else {
    messages->this = (void *)messages;
  }

  SentMessagesCount++;
  Azure1->messageId = messages->messageTrackingId = SentMessagesCount;

  if (SERIALIZE(&destination, &destinationSize,
                Azure1->deviceId, Azure1->messageId,
                Azure1->Temperature, Azure1->Humidity, Azure1->Pressure,
                Azure1->accX , Azure1->accY, Azure1->accZ,
                Azure1->gyrX , Azure1->gyrY, Azure1->gyrZ,
                Azure1->magX , Azure1->magY, Azure1->magZ,
                Azure1->HWCounter,
                Azure1->ts) != CODEFIRST_OK){
    AZURE_PRINTF("Err: Failed to serialize\r\n");
    while(1) {
      ;
    }
  }

  /* Only for Debug */
  //AZURE_PRINTF("MessageToSend=%.*s\r\n",destinationSize,destination);

  if ((messages->messageHandle = IoTHubMessage_CreateFromByteArray(destination, destinationSize)) == NULL) {
    AZURE_PRINTF("Err: iotHubMessageHandle is NULL!\r\n");
  } else {
    char msgText[32];
    MAP_HANDLE propMap = IoTHubMessage_Properties(messages->messageHandle);
    sprintf_s(msgText, sizeof(msgText), "PropMsg_%zu", SentMessagesCount);
    if (Map_AddOrUpdate(propMap, "PropName", msgText) != MAP_OK){
      AZURE_PRINTF("Err: Map_AddOrUpdate Failed!\r\n");
    }
    if (IoTHubClient_LL_SendEventAsync(iotHubClientHandle, messages->messageHandle, SendConfirmationCallback, messages) != IOTHUB_CLIENT_OK) {
      AZURE_PRINTF("Err: IoTHubClient_LL_SendEventAsync..........FAILED!\r\n");
    } else {
      AZURE_PRINTF("IoTHubClient_LL_SendEventAsync accepted message [%d] for transmission to IoT Hub.\r\n", SentMessagesCount);
    }
    free(destination);
  }
  IoTHubMessage_Destroy(messages->messageHandle);
}

/**
  * @brief  Send 6D Orientation to IoT Hub
  * @param  None
  * @retval None
  */
static void Send6DOrientationData(void)
{
  EVENT_INSTANCE *messages;
  unsigned char* destination;
  size_t destinationSize;
  time_t now;

  messages = (EVENT_INSTANCE *) calloc(1,sizeof(EVENT_INSTANCE));
  if(messages==NULL) {
    AZURE_PRINTF("Err: Allocating Memory for messages to IoT Hub\r\n");
    while(1) {
      ;
    }
  } else {
    messages->this = (void *)messages;
  }

  SentMessagesCount++;
  Azure1->messageId = messages->messageTrackingId = SentMessagesCount;

  /* Read the Time  from RTC */
  now = TimingSystemGetSystemTime();
  /* Time Stamp */
  Azure1->ts = GetDateTimeOffset(now);

  if (SERIALIZE(&destination, &destinationSize,
                Azure1->deviceId, Azure1->messageId,
                Azure1->HWOrientation,
                Azure1->ts) != CODEFIRST_OK){
    AZURE_PRINTF("Err: Failed to serialize\r\n");
    while(1) {
      ;
    }
  }

  /* Only for Debug */
  //AZURE_PRINTF("MessageToSend=%.*s\r\n",destinationSize,destination);

  if ((messages->messageHandle = IoTHubMessage_CreateFromByteArray(destination, destinationSize)) == NULL) {
    AZURE_PRINTF("Err: iotHubMessageHandle is NULL!\r\n");
  } else {
    char msgText[32];
    MAP_HANDLE propMap = IoTHubMessage_Properties(messages->messageHandle);
    sprintf_s(msgText, sizeof(msgText), "PropMsg_%zu", SentMessagesCount);
    if (Map_AddOrUpdate(propMap, "PropName", msgText) != MAP_OK){
      AZURE_PRINTF("Err: Map_AddOrUpdate Failed!\r\n");
    }
    if (IoTHubClient_LL_SendEventAsync(iotHubClientHandle, messages->messageHandle, SendConfirmationCallback, messages) != IOTHUB_CLIENT_OK) {
      AZURE_PRINTF("Err: IoTHubClient_LL_SendEventAsync..........FAILED!\r\n");
    } else {
      AZURE_PRINTF("IoTHubClient_LL_SendEventAsync accepted message [%d] for transmission to IoT Hub.\r\n", SentMessagesCount);
    }
    free(destination);
  }
  IoTHubMessage_Destroy(messages->messageHandle);
}

/**
  * @brief  Callback for user button
  * @param  None
  * @retval None
  */
static void ButtonCallback(void)
{
  AZURE_PRINTF("Button Pressed\r\n");
  g_continueRunning = false;
}

/**
  * @brief  Function for waiting that the IoT has received all the messages/status sent
  * @param  None
  * @retval None
  */
void WaitAllTheMessages(void)
{
  /* Wait All the Reported State/Message confirmation */
  while((StatusBackCount<SentStatusCount) | (MessagesBackCount<SentMessagesCount)) {
    IoTHubClient_LL_DoWork(iotHubClientHandle);
    HAL_Delay(10);
  }
}

/**
  * @brief  Main MQTT application for Telemetry
  * @param  None
  * @retval None
  */
void AzureClient_mqtt_TM(void)
{
  if (platform_init() != 0){
    AZURE_PRINTF("Failed to initialize the platform.\r\n");
    AzureExit(AZURE_ERR_PLATFORM_INIT);
  } else {
    char MAC_RegisterdAddress[13];
    g_continueRunning = true;
    AZURE_PRINTF("Platform Init Done\r\n");
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

    /* Led on for signaling the end of Platform init phase */
    BSP_LED_On(LED2);

    if (serializer_init(NULL) != SERIALIZER_OK) {
      AZURE_PRINTF("Err: Unable to Initialize Serializer\r\n");
      AzureExit(AZURE_ERR_IOT_START);
    }
    AZURE_PRINTF("Serializer Initialized\r\n");

#ifdef AZURE_ENABLE_REGISTRATION
    /* Board Registration Procedure */
    if(RegistrationAgent(&connectionString)!=0) {
      AZURE_PRINTF("Err: During Board Registration\r\n");
      AzureExit(AZURE_ERR_IOT_START);
    } else {
      sURI_Info URI = { URI_ID_0x04_STRING, "\0", "\0"};
      AZURE_PRINTF("Board Registration Done\r\n\r\n");
      /* if NFC is present */
      if(TargetBoardFeatures.NFCInitialized) {
        sprintf(URI.URI_Message,"%s/Home/Index/%s\r\n\r\n",WEB_DASHBOARD_URL, MAC_RegisterdAddress);
        while (TT4_WriteURI(&URI) != SUCCESS) {
          ;
        }
        AZURE_PRINTF("Read the NFC or Connect to:\r\n");
      } else {
        AZURE_PRINTF("Connect to:\r\n");
      }
      AZURE_PRINTF("https://%s/Home/Index/%s\r\n\r\n",WEB_DASHBOARD_URL, MAC_RegisterdAddress);
    }
#else /* AZURE_ENABLE_REGISTRATION */
    if(connectionString==NULL) {
      AZURE_PRINTF("Err: connectionString is NULL!\r\n");
      AZURE_PRINTF("\tSet AZUREDEVICECONNECTIONSTRING in azure1_config.h file or via serial terminal.\r\n");
      AzureExit(AZURE_ERR_IOT_START);
    }
#endif /* AZURE_ENABLE_REGISTRATION */

    if((iotHubClientHandle = IoTHubClient_LL_CreateFromConnectionString(connectionString, MQTT_Protocol)) == NULL){
      AZURE_PRINTF("Err: iotHubClientHandle is NULL!\r\n");
			AZURE_PRINTF("\tRestart the board and set connections string!!\r\n\t** OR **\n\r");
      AZURE_PRINTF("\tchange AZUREIOTHUBCONNECTIONSTRING on Inc/azure1_config.h file\r\n");
      AzureExit(AZURE_ERR_IOT_START);
    } else {
      AZURE_PRINTF("iotHubClientHandle Created\r\n");
    }

    {
      bool traceOn = true;
      if(IoTHubClient_LL_SetOption(iotHubClientHandle, "logtrace", &traceOn)==IOTHUB_CLIENT_OK) {
        AZURE_PRINTF("iotHubClientSetOption logtrace Ok\r\n");
      }
    }

    Azure1 = CREATE_MODEL_INSTANCE(AzureModel, Azure1_t);
    if (Azure1 == NULL){
      AZURE_PRINTF("Err: Creating the Model Instance\r\n");
      AzureExit(AZURE_ERR_IOT_START);
    } else {
      AZURE_PRINTF("Model Instance Created\r\n");
    }

    /* Allocate Space for storing Board ID */
    Azure1->deviceId = (char *) calloc(16,1);
    if(Azure1->deviceId==NULL) {
      AZURE_PRINTF("Err Allocating memory\r\n\tFile %s Line %d\r\n",__FILE__, __LINE__);
      AzureExit(AZURE_ERR_MALLOC);
    }

    /* Allocate Space for storing Device Status */
    Azure1->AzureStatus = (char *) calloc(16,1);
    if(Azure1->AzureStatus==NULL) {
      AZURE_PRINTF("Err Allocating memory\r\n\tFile %s Line %d\r\n",__FILE__, __LINE__);
      AzureExit(AZURE_ERR_MALLOC);
    }

    /* Allocate Space for storing Device Firmware Version */
    Azure1->AzureFwVersion = (char *) calloc(64,1);
    if(Azure1->AzureFwVersion==NULL) {
      AZURE_PRINTF("Err Allocating memory\r\n\tFile %s Line %d\r\n",__FILE__, __LINE__);
      AzureExit(AZURE_ERR_MALLOC);
    }

    /* Allocate Space for storing the HW orientation */
    Azure1->HWOrientation = (char *) calloc(16,1);
    if(Azure1->HWOrientation==NULL) {
      AZURE_PRINTF("Err Allocating memory\r\n\tFile %s Line %d\r\n",__FILE__, __LINE__);
      AzureExit(AZURE_ERR_MALLOC);
    }
    sprintf(Azure1->HWOrientation,"%s","Unknowed");

    /* Supported Methods */
    Azure1->SupportedMethods = "{\"Reboot\": \"Reboot the device\",\
                                 \"Quit\": \"Stop the trasmission\"}";
    /* Supported Commands */
    Azure1->SupportedCommands = "{\"Pause\": \"Pause Sensors' Data trasmission\",\
                                 \"Play\": \"Resume Sensors' Data trasmission\",\
                                 \"LedOn\": \"Switch On the User Led\",\
                                 \"LedOff\": \"Switch Off the User Led\",\
                                 \"LedBlink\": \"Blinking User Led for every Sensor's Data trasmission\"}";

    /* Fixed Data Fields an reported Properties */
    sprintf(Azure1->AzureFwVersion,"%s V%c.%c.%c SDK=%s", AZURE_PACKAGENAME,
            AZURE_VERSION_MAJOR,AZURE_VERSION_MINOR,AZURE_VERSION_PATCH,IOTHUB_SDK_VERSION);
    /* Board Identification */
    sprintf(Azure1->deviceId,"%s",MAC_RegisterdAddress);

    /* Telemetry Intervall */
    Azure1->TelemetryInterval = TargetBoardFeatures.TIM_CC1_Pulse/2000 /* 2Hz Timer Frequency */;


    /* Active HW Mode == FreeFall */
    Azure1->ActiveHWMode = 2;

    /* Setting Message call back, so we can receive Commands. */
    if (IoTHubClient_LL_SetMessageCallback(iotHubClientHandle, ReceiveMessageCallback, Azure1) != IOTHUB_CLIENT_OK){
      AZURE_PRINTF("Err: IoTHubClient_LL_SetMessageCallback..........FAILED!\r\n");
      AzureExit(AZURE_ERR_IOT_START);
    } else {
      AZURE_PRINTF("IoTHubClient_LL_SetMessageCallback...successful.\r\n");
    }

    if(IoTHubClient_LL_SetDeviceMethodCallback(iotHubClientHandle, deviceMethodCallback, Azure1) != IOTHUB_CLIENT_OK){
      AZURE_PRINTF("Err: IoTHubClient_LL_SetDeviceMethodCallback..........FAILED!\r\n");
      AzureExit(AZURE_ERR_IOT_START);
    } else {
      AZURE_PRINTF("IoTHubClient_LL_SetDeviceMethodCallback...successful.\r\n");
    }

#ifdef USE_MBED_TLS
    /* Load certificates */
    if (IoTHubClient_LL_SetOption(iotHubClientHandle, "TrustedCerts", certificates) != IOTHUB_CLIENT_OK ) {
      AZURE_PRINTF("Err: IoTHubClient_LL_SetOption(certificates)..........FAILED!\r\n");
      AzureExit(AZURE_ERR_IOT_START);
    } else {
      AZURE_PRINTF("IoTHubClient_LL_SetOption(certificates)...successful.\r\n");
    }
#endif  /* USE_MBED_TLS */

    if(IoTHubClient_LL_SetDeviceTwinCallback(iotHubClientHandle, DeviceTwinCallbackStatus, Azure1) != IOTHUB_CLIENT_OK){
      AZURE_PRINTF("Err: IoTHubClient_LL_SetDeviceTwinCallback..........FAILED!\r\n");
      AzureExit(AZURE_ERR_IOT_START);
    } else {
      AZURE_PRINTF("IoTHubClient_LL_SetDeviceTwinCallback...successful.\r\n");
    }

    /* Wait the Answer from IOT */
    while(ReceivedDeviceTwinProperty==0){
      IoTHubClient_LL_DoWork(iotHubClientHandle);
      HAL_Delay(100);
    }

    /* Led off for signaling the end IOT connection procedure */
    BSP_LED_Off(LED2);

    Azure1->HWCounter =0;

    StartTimer1();

    ReportState(Running);

    /* Now that we are ready to receive commands and send messages */
    while(g_continueRunning) {
      /* Handle Interrupt from MEMS */
      if(MEMSInterrupt) {
        uint32_t HWFeaturesAnswer;
        HWFeaturesAnswer = MEMSCallback();
        MEMSInterrupt=0;

        if(HWFeaturesAnswer!=ACC_NOT_USED) {
          switch(Azure1->ActiveHWMode) {
            case 1:
              /* Pedometer */
              Azure1->HWCounter = HWFeaturesAnswer;
            break;
            case 2:
              /* Free Fall */
            case 3:
              /* * Single Tap */
            case 4:
              /* * Double Tap */
            case 5:
              /* Wake Up */
            case 6:
              /* Tilt */
              HWEventCounter[Azure1->ActiveHWMode-2]++;
              Azure1->HWCounter = HWEventCounter[Azure1->ActiveHWMode-2];
            break;
            case 7:
              /* If we had enabled the 6D orientation HW feature */
              switch(HWFeaturesAnswer) {
                case ACC_6D_OR_TOP:
                  sprintf(Azure1->HWOrientation,"%s","Top");
                break;
                case ACC_6D_OR_LEFT:
                  sprintf(Azure1->HWOrientation,"%s","Left");
                break;
                case ACC_6D_OR_BOTTOM:
                  sprintf(Azure1->HWOrientation,"%s","Bottom");
                break;
                case ACC_6D_OR_RIGTH:
                  sprintf(Azure1->HWOrientation,"%s","Right");
                break;
                case ACC_6D_OR_UP:
                  sprintf(Azure1->HWOrientation,"%s","Up");
                break;
                case ACC_6D_OR_DOWN:
                  sprintf(Azure1->HWOrientation,"%s","Down");
                break;
              }
              Send6DOrientationData();
            break;
          }
        }
      }

      /* Handle user button */
      if(ButtonPressed) {
        ButtonCallback();
        ButtonPressed=0;
      }

      /* Environmental Data */
      if(SendData) {
        SendData=0;
        /* Led Toggle */
        if(LedMode == AZURE_LED_BLINK) {
          BSP_LED_Toggle(LED2);
        }
        SendSNSData();
      }

      /* Reboot System */
      if(g_reboot) {
        g_reboot = false;
        ReportState(Rebootting);
        WaitAllTheMessages();
        AZURE_PRINTF("Call to HAL_NVIC_SystemReset\r\n");
        HAL_NVIC_SystemReset();
      }
      IoTHubClient_LL_DoWork(iotHubClientHandle);
    }

    StopTimer1();

    AZURE_PRINTF("iothub_client_sample_mqtt has gotten quit message\r\n");

    ReportState(Ended);

    WaitAllTheMessages();

    free(Azure1->deviceId);
    free(Azure1->AzureStatus);
    free(Azure1->AzureFwVersion);
    free(Azure1->HWOrientation);

    DESTROY_MODEL_INSTANCE(Azure1);

    IoTHubClient_LL_Destroy(iotHubClientHandle);
    AZURE_PRINTF("iotHubClientHandle Destroied\r\n");

    serializer_deinit();
    platform_deinit();
    AZURE_PRINTF("Platform DeInit\r\n");
  }
}


#ifdef MBEDTLS_ENTROPY_HARDWARE_ALT

#include "mbedtls/entropy_poll.h"
/**
  * @brief Function for initializing the Random Generator
  * @param void* Data
  * @param unsigned char* Output
  * @param size_t Len Lenght of the Input
  * @param size_t* oLen Lenght of the Output
  * @retval int ok/error (0/-1)
  */
int mbedtls_hardware_poll( void *Data, unsigned char *Output, size_t Len, size_t *oLen )
{
  uint32_t index;
  uint32_t randomValue;
		
  for (index = 0; index < Len/4; index++) {
    if (randomValue=rand()) {
      *oLen += 4;
      memset(&(Output[index * 4]), (int)randomValue, 4);
    } else {
      AZURE_PRINTF("Err random number\r\n");
      return -1;
    }
  }
  return 0;
}

#endif /* MBEDTLS_ENTROPY_HARDWARE_ALT */

/******************* (C) COPYRIGHT 2017 STMicroelectronics *****END OF FILE****/
