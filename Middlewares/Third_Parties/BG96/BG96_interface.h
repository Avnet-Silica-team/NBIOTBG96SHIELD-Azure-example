/**
  ******************************************************************************
  * @file           : MQTT_BG96_interface.h
  * @brief          : Header for MQTT_BG96_interface.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2018 RSR
  *
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MQTTBG96_H__
#define __MQTTBG96_H__

#include <stdint.h>
#include <stdbool.h>

typedef struct {
  char sim_pin[12];
  char sim_apn[64];
}BG96_CredAcc_t;


/********** Wi-Fi Error *************/
typedef enum
{ 
  BG96_MODULE_SUCCESS           = 0,
  BG96_TIME_OUT_ERROR           = 1,  
  BG96_MODULE_ERROR,
  BG96_HAL_OK,
  BG96_NOT_SUPPORTED,
  BG96_NOT_READY,
  BG96_SCAN_FAILED,
  BG96_AT_CMD_BUSY,
  BG96_SSID_ERROR,
  BG96_SecKey_ERROR,
  BG96_CONFIG_ERROR,
  BG96_STA_MODE_ERROR,
  BG96_AP_MODE_ERROR,
  BG96_AT_CMD_RESP_ERROR,
  BG96_AT_FILE_LENGTH_ERROR,
  BG96_HAL_UART_ERROR,
  BG96_IN_LOW_POWER_ERROR,
  BG96_HW_FAILURE_ERROR,
  BG96_HEAP_TOO_SMALL_WARNING,
  BG96_STACK_OVERFLOW_ERROR,
  BG96_HARD_FAULT_ERROR,
  BG96_MALLOC_FAILED_ERROR,
  BG96_INIT_ERROR,
  BG96_POWER_SAVE_WARNING,
  BG96_SIGNAL_LOW_WARNING,
  BG96_JOIN_FAILED,
  BG96_SCAN_BLEWUP,
  BG96_START_FAILED_ERROR,
  BG96_EXCEPTION_ERROR,
  BG96_DE_AUTH,
  BG96_DISASSOCIATION,
  BG96_UNHANDLED_IND_ERROR,
  BG96_RX_MGMT,
  BG96_RX_DATA,
  BG96_RX_UNK  
} BG96_Status_t;

typedef enum {
  BG96_state_reset = 0,
  BG96_state_ready,
  BG96_state_idle,
  BG96_state_connected,
  BG96_state_connecting,
  BG96_state_disconnected,
  BG96_state_activity,
  BG96_state_inter,
  BG96_state_print_data,
  BG96_state_error,
  BG96_undefine_state       = 0xFF,
} BG96_state_t;

/**
  * @brief network structure, contain socket id, and pointer to MQTT read,write and disconnect functions 
  */	
typedef struct network
{
	int my_socket;
	int my_net;
}Network;


typedef enum{
	BG96_PWS_NONE,
	BG96_PWS_ON,
	BG96_PWS_OFF
}BG96_PWS_STATE;

/***********************************************/
// general BG96 state enums

typedef enum{
	BG96_IDLE,
	BG96_BUSY,
	BG96_RESET,
	BG96_W_OK,
	BG96_INIT,
	BG96_DEINIT,
	BG96_ECHO,
	BG96_CONNECT,
	BG96_CFUN_DELAY,
	BG96_PIN_REQUEST,
	BG96_PIN_SET,
	BG96_PIN_SEND,
	BG96_PIN_READY,
	BG96_BUSY_ERROR,
	BG96_PB_READY,
	BG96_COPS_RQST,
	BG96_APN_CFG,
	BG96_APN_READ,
	BG96_ACT_ACTIVATE,
	BG96_NTP_REQUEST,
	BG96_NTP_END_OK,
	B96_MAC_RQST,
	B96_MAC_RQST_OK,
	
	
	BG96_SSL_CONFIGURE,
	BG96_SSL_CONFIGURE_CHIP,
	BG96_SSL_CONFIGURE_SEC,
	BG96_SSL_CONFIGURE_CERT,
	BG96_SOCKET_OPEN,
	BG96_SOCKET_OPEN_WANSW,
	BG96_SOCKET_STATUS,
	BG96_INIT_DONE,
	BG96_SEND,
	BG96_READY,
	BG96_W_ANSW,
	BG96_WAIT_OK,
}BG96_STATE;

/***********************************************/
// send state machine enums
typedef enum{
	BG96_SEND_NONE,
	BG96_SEND_REQUEST,
	BG96_SEND_SEND,
	BG96_SEND_ANSW,
	BG96_SEND_DONE,
	BG96_SEND_ERROR,
}BG96_SEND_STATE;

typedef enum{
	SEND_NONE,
	SEND_IN_PROGRESS,
	SEND_DONE,
	SEND_ERROR,
	SEND_RETRY
}BG96_SEND_RESULT;

/***********************************************/
// read state machine enums
typedef enum{
	BG96_READ_NONE,
	BG96_READ_REQUEST,
	//BG96_READ_TOTAL_SIZE,
	//BG96_STORE_TOTAL_SIZE,
	BG96_READ_PROCESS,
	BG96_READ_SEND,
	BG96_READ_ANSW,
	BG96_READ_DONE,
	BG96_READ_ERROR,
	BG96_READ_ERROR_SOCK,
	BG96_READ_ERROR_SOCK_CLOSED,
	//BG96_FAILURE_ERROR
}BG96_READ_STATE;

typedef enum{
	READ_NONE,
	READ_IN_PROGRESS,
	READ_DONE,
	READ_ERROR,
	READ_RETRY
}BG96_READ_RESULT;


/***********************************************/
// OPEN socket state machine enums
typedef enum{
	BG96_OPEN_NONE,
	BG96_OPEN_REQUEST,
	BG96_OPEN_PROCESS,
	BG96_OPEN_SEND,
	BG96_OPEN_ANSW,
	BG96_OPEN_DONE,
	BG96_OPEN_ERROR,
	BG96_OPEN_ERROR_SOCK,
	BG96_OPEN_ERROR_SOCK_CLOSED,
	//BG96_FAILURE_ERROR
}BG96_OPEN_STATE;

typedef enum{
	OPEN_NONE,
	OPEN_IN_PROGRESS,
	OPEN_DONE,
	OPEN_ERROR,
	OPEN_RETRY
}BG96_OPEN_RESULT;


/***********************************************/
// general type result enums

typedef enum{
	BG96_WAIT,
	BG96_ASNWER_DONE,
	BG96_OK,
	BG96_CME_ERROR,
	BG96_ERROR,
	BG96_GENERAL_FAILURE,
	BG96_RX_TIMEOUT,
	BG96_TX_INVALID,
	BG96_SOCK_CLOSED,
	BG96_SOCK_DATA,
	BG96_NONE
}BG96_result;

typedef enum{
	BG96_PWR_NONE,
	BG96_PWR_W_ON,
	BG96_PWR_ON_WAIT,
	BG96_PWR_W_OFF,
	BG96_PWR_W_OFF_DELAY,
	BG96_PWR_W_ON_DELAY,
	BG96_PWR_WAIT_DELAY,
	BG96_PWR_W_OK,
	BG96_PWR_DONE,
	BG96_PWR_GPS,
	BG96_ANSW_GPS_WAIT,
	BG96_PWR_GPS_WAIT,
	BG96_PWR_HIGH,
	BG96_PWR_LOW
}BG96_PWR_INIT;

typedef enum{
	BG96_POWER_OFF,
	BG96_POWER_ON
}BG96_PDW;

typedef enum{
	BG96_CONNECTION_STATUS_NONE = 0,
	BG96_CONNECTION_STATUS_POWERED,
	BG96_CONNECTION_STATUS_READY,
	BG96_CONNECTION_STATUS_ERROR,
	BG96_CONNECTION_STATUS_SMS_FAIL
}BG96_CONNECTION_TABLE;

//void BG96_power(BG96_PDW pwd_state);
void BG96_rx_callback(uint8_t rxc);
void BG96_timers_callback(void);

uint8_t BG96_check_ready(void);
uint8_t BG96_check_reset(void);
BG96_STATE BG96_check_status(void);

void BG96_deinit(void);
void BG96_init(void);

void BG96_restart(void);
bool check_BG96_restart(void);

bool BG96_power_on(void);
uint8_t BG96_check_power_on(void);

bool BG96_power_off(void);
uint8_t BG96_check_power_off(void);

void BG96_check_power_in_progress(void);

void BG96_RESET_state(void);
//void BG96_rxChar_Enable(void);
uint8_t BG96_read_char(void);

BG96_result BG96_socket_recv (void);
void BG96NewNetwork(Network* n) ;

BG96_Status_t BG96_socket_write (uint8_t sock_id, uint16_t DataLength, char * pData);
int BG96_socket_read (Network* net, unsigned char* i, int ch, int timeout);
void BG96_socket_close (uint8_t sock_id);

void BG96_socket_alloc(uint8_t sock);
void BG96_socket_free(uint8_t sock);

bool gsm_init(char* pin, char* apn);
void GET_BG96_mac(uint8_t* mac);

//int BG96_socket_open(uint8_t * hostname, uint32_t port_number, uint8_t * type, uint8_t sock_id);


extern char SIM_pin[12];
extern char BG96_sn[15];

BG96_Status_t BG96_socket_client_open(uint8_t * hostname, uint32_t port_number, uint8_t * protocol, uint8_t * sock_id); 
BG96_Status_t BG96_socket_client_write(uint8_t sock_id, uint16_t DataLength, char * pData);
BG96_Status_t BG96_socket_client_close(uint8_t sock_close_id);

void ind_bg96_socket_data_received(uint8_t pSocketHandle,uint8_t * data_ptr, uint32_t message_size, uint32_t chunk_size);
void ind_bg96_socket_client_remote_server_closed(uint8_t * socketID);
void ind_bg96_connected(void);
	
#endif
