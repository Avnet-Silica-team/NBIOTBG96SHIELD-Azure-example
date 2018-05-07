/**
 ******************************************************************************
 * @file    BG96_interface.c
 * @author  abe
 * @version V1.1.0
 * @date    24-04-2108
 * @brief   interface functionality for BG96 Quectel IoT module sample demo
 ******************************************************************************

 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/

 
#if defined (USE_STM32F4XX_NUCLEO)
#include "stm32f4xx_hal.h"
#endif

#if defined (USE_STM32L4XX_NUCLEO)
#include "stm32l4xx_hal.h"
#endif

#include "BG96_interface.h"
#include "stm32_BG96_iot.h"

//#include "ATParser.h"

#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"

#define ONE_SEC									1000
#define BG96_BUFF_SIZE 					4096	
#define MAX_ANSWER_NUMBER 			(100)                     //max number of string messages that can be queued before process it
#define BG96_MAX_RESPONSE_TIME 	(uint32_t)(90*ONE_SEC)
#define RX_MAX_SIZE							(4096)										// 

#define SIZE_SOCKET_BUFFER			(4096)


#define	ANSWER_MAX_SIZE					256

char SIM_Pin[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
char BG96_sn[15] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
char SIM_Apn[40] = {0,0,0,0,0,0,0,0,0,0,
										0,0,0,0,0,0,0,0,0,0,
										0,0,0,0,0,0,0,0,0,0,
										0,0,0,0,0,0,0,0,0,0};
		
#if BG96_ENABLE

//BG96 general purpose timeout
volatile uint32_t         BG96_delay = 0;
volatile uint32_t         BG96_timeout = 0;
volatile uint32_t         BG96_timeout_counter = 0;
volatile uint32_t         BG96_time_stamps = 0;

//BG96 reception buffers, pointers and flags, used to store single string and pointers are on rx_start array
uint8_t										BG96_rx_chr[BG96_BUFF_SIZE];
uint32_t            		  rx_start[MAX_ANSWER_NUMBER];
uint32_t            		  rx_scan = 0;
uint8_t										rx_string[RX_MAX_SIZE];
//volatile uint8_t     			mc60_rx_done = false;					//general purpose flags for communication

//temp buffer to store socket data unless any search...
 char		 					 	BG96_rx_data[RX_MAX_SIZE];
 uint32_t					 	BG96_ix = 0;

//temp buffer to store genar answer data ...
static char		 					 	BG96_answer_data[RX_MAX_SIZE];
static uint32_t					 	BG96_answ_ix = 0;

//uint8_t									 answer_copy_buffer[RX_SIZE];
uint16_t 									BG96_rx_ix = 0;
uint8_t 								 	uart_rx_status_flag = 0;		//for \r\n detection
uint32_t          			 	answer_w_point = 0;
uint32_t           			 	answer_r_point = 0;
uint8_t         			   	rx_overwrite = false;
uint8_t          				 	more_answer = true;					//unused here

//BG96 state machine definitions
BG96_STATE BG96_status 		= BG96_RESET;
BG96_STATE BG96_next_status = BG96_RESET;

/* string definitions */
/* BG96 answer */
const uint8_t BG96_W_CPIN[] = 					{"+CPIN: SIM PIN\r\n"};
const uint8_t BG96_NO_COPS[] = 					{"+COPS: 0\r\n"};
const uint8_t BG96_MSG_PIN_READY[] = 		{"+CPIN: READY\r\n"};
const uint8_t BG96_CME_PIN_ERROR[] = 		{"+CME ERROR: 13\r\n"};
const uint8_t BG96_MSG_PB_DONE[] = 			{"+QIND: PB DONE\r\n"};
const uint8_t BG96_MSG_COPS[] =					{"+COPS: 0,0,\"I WIND\",2\r\n"};
const uint8_t BG96_MSG_SOCKET_OK[] =		{"+QSSLOPEN: 1,0\r\n"};
const uint8_t BG96_MSG_SEND_PROMPT[] = 	{"> "};
const uint8_t BG96_MSG_SEND_OK[]	= 		{"SEND OK\r\n"};
const uint8_t BG96_MSG_CLOSED[] = 			{"+QSSLURC: \"closed\",1\r\n"};
const uint8_t BG96_MSG_RECV[] = 				{"+QSSLURC: \"recv\",1\r\n"};
const uint8_t BG96_LTS_ANSW[] = 				{"+QLTS: "};

const uint8_t BG96_MSG_OPEN_OK[] =			{"+QIOPEN: 1,0"};

/*BG96 null answer (use for read BG96 answer string) */
const uint8_t BG96_NULL[] =							{"\0"};

/* BG96 commands */
const uint8_t BG96_SEND_PIN[] = 				{"AT+CPIN=%04d\r"};

/* relayr backend IP address */
const char IP_HOST_ADD[] =							{"\"52.29.237.86\""};
//const char IP_HOST_ADD[] =							{"\"35.156.87.63\""}; //--> fails???

/* APN string */
const char APN_SETTING[] = 							{"\"internet.wind\""};

//for debug
bool prompt_flag = false;
bool BG96_restart_flag = false;

uint8_t debug_ok_count = 0;
uint8_t debug_answ_count = 0;
uint8_t* BG96_answ_point = 0;
static volatile uint32_t BG96_wait_timer = 0;

extern UART_HandleTypeDef	 huart2;
extern UART_HandleTypeDef	 UartMsgHandle;

/*************************************************
 * PRIVATE FUNCTIONS
 *************************************************/
 /**
* @brief  Put out string to BG96 modem
* @param  string pointer, size
* @retval None
*/
static void vPrintf(char* string, uint16_t size)
{
	HAL_StatusTypeDef BG96_uart_result;

	if (size == 0)
			size = strlen(string);
	BG96_uart_result = BG96_uart_send((uint8_t*)string, size);
	
	if (BG96_uart_result != HAL_OK)
		while(1);			//LOOP error

}

 /**
* @brief  load BG96 timeout
* @param  timeout value msec
* @retval None
*/
static void BG96_timeout_load(uint32_t tout)
{
    if (tout == 0)
    	BG96_timeout = BG96_MAX_RESPONSE_TIME;
    else
    	BG96_timeout = tout;
}

uint8_t closing_socket = 0;
uint8_t recv_socket = 0;

 /**
* @brief  BG96 answer scan and store in multiple answer buffer
* @param  timeout
* @retval result
*/
static BG96_result BG96_receive(uint32_t timeout)
{
		BG96_result result = BG96_NONE;
    uint32_t local_answer_w_point;
    uint32_t len_of_string;
		uint32_t ii;
    uint32_t p;
    uint8_t rx_data;
	
    int stcp;
    more_answer =  false;
      //check timeout expiring
       if (BG96_timeout == 0)
       {
    	   BG96_timeout_counter++;
					 BG96_restart_flag = true;
           return BG96_RX_TIMEOUT;
       }

       //read the write pointer to check if a valid message is arrived
       //disable uart IRQ and make a local copy to prevent unwanted value
			 HAL_NVIC_DisableIRQ(USARTbg_IRQn);
       local_answer_w_point = answer_w_point;
			 HAL_NVIC_EnableIRQ(USARTbg_IRQn);
       //check if read pointer differ from write pointer (message to be processed)
       if (answer_r_point != local_answer_w_point)
       {
           //valid message detected
           len_of_string = 0;
					 memset(rx_string, 0, sizeof(rx_string));
           p = rx_start[answer_r_point];
         do
         {
               //copy valid data string, terminator is 0x0...
							 rx_data = (uint8_t)(BG96_rx_chr[p]);
               rx_string[len_of_string] = rx_data;
							 
							 if (len_of_string < RX_MAX_SIZE)
								len_of_string++;
							 else
								 rx_data = 0;				//truncate buffer to prevent crash!!
							 
               p = (p+1)%BG96_BUFF_SIZE;

          }while (rx_data != 0);

     //answer_r_point = (answer_r_point+1)%MAX_ANSWER_NUMBER;
		 stcp = strcmp((char*)rx_string, "\r\n") ; 

       //stcp = strcmp((char*)BG96_rx_buff, "\r\n") ;
      if ((len_of_string>2) && (stcp != 0))
			{

					 //general error answer, restart BG96 modem
				   if (memcmp((char*)rx_string, "ERROR", 5) == 0)
								{
								    result = BG96_ERROR;
										///no action will be taken, application stops!!
										//BG96_restart_flag = true;
								}
								
					  else	
					 //pdp context killed, restart BG96 modem
					 if (memcmp((char*)rx_string, "+QIURC: \"pdpdeact\",1", 20) == 0)
								{
										result = BG96_ERROR;
										///no action will be taken, application stops!!
										//BG96_restart_flag = true;
								}
								
							else 
  					if (BG96_answ_point != 0)
							{
								ii=0;
								while (1)
								 {
									 if (*(BG96_answ_point+ii) == 0)
										 break;
									 ii++;
									}
								 
								if (memcmp((char*)rx_string, (char*)BG96_answ_point, ii) == 0)
									{
           		   	   result = BG96_ASNWER_DONE;
										 BG96_answ_ix = 0;
										 memset(BG96_answer_data, 0, sizeof(BG96_answer_data));
											do
											{
												BG96_answer_data[BG96_answ_ix] = rx_string[ii];
												ii++;
												BG96_answ_ix++;
												//if general data over max buffer len truncate to prevent crash
												if (BG96_answ_ix > ANSWER_MAX_SIZE)
													break;
											}while (rx_string[ii] != 0);
            	    }
							}

								// else //
						 if (strcmp((char*)rx_string, "OK\r\n") == 0)
                {
                    result = BG96_OK;
                }
							 else //
						 if ( (memcmp((char*)rx_string, "+CME ERROR:", 11) == 0) || (memcmp((char*)rx_string, "+CMS ERROR:", 11) == 0) )
                {
										//CME or CMS error, need to be processed ...
                    result = BG96_CME_ERROR;
                }
							 else
						 //if (memcmp((char*)rx_string, "+QIURC: \"closed\",1", 18) == 0)
						 if (memcmp((char*)rx_string, "+QIURC: \"closed\",", 17) == 0)
								{
										//socket externally closed ... 
										result = BG96_SOCK_CLOSED;
										closing_socket = atoi((const char*)&rx_string[17]);
								}
							 else	
						  //if (memcmp((char*)rx_string, "+QIURC: \"recv\",1", 16) == 0)
							if (memcmp((char*)rx_string, "+QIURC: \"recv\",", 15) == 0)
								{
										//data pending from socket...
										//printf("QUIRC..\r\n");
										result = BG96_SOCK_DATA;
										recv_socket = atoi((const char*)&rx_string[15]);
								}
								
                //reload answer_write pointer to check if there are others pending messages
							  HAL_NVIC_DisableIRQ(USARTbg_IRQn);
                local_answer_w_point = answer_w_point;
								HAL_NVIC_EnableIRQ(USARTbg_IRQn);
                if (answer_r_point != local_answer_w_point)
                    more_answer = true;      // almost another answer pending!!

         }
				 HAL_NVIC_DisableIRQ(USARTbg_IRQn);
				 //update answer pointer
				 answer_r_point = (answer_r_point+1)%MAX_ANSWER_NUMBER;
				 HAL_NVIC_EnableIRQ(USARTbg_IRQn);
       }

    return result;

}

/*******************************************************************************
 * BG96 power on or off functios
 ******************************************************************************/

BG96_PWR_INIT BG96_PWR_status = BG96_PWR_NONE;
 /**
* @brief  power on/off modem
* @param  ON or OFF 
* @retval none
*/
static void BG96_power(BG96_PDW pwd_state)
{
	//BG96_result res;
	switch (BG96_PWR_status){
			case BG96_PWR_NONE:
			case BG96_PWR_DONE:
				if (pwd_state == BG96_POWER_OFF)
				{
					BG96_vbat_off();
					BG96_reset_off();
					BG96_pwr_key_off();
					BG96_wait_timer = (100);
					BG96_PWR_status = BG96_PWR_W_OFF;
				}
				else
				{
					BG96_pwr_key_on();
					BG96_reset_on();
					BG96_vbat_on();
					BG96_wait_timer = (300);
					BG96_PWR_status = BG96_PWR_WAIT_DELAY;
				}
				break;

			case BG96_PWR_WAIT_DELAY:
				if (BG96_wait_timer == 0)
				{
					//BG96_pwr_key_on();
					BG96_reset_off();
					//BG96 power on
					BG96_wait_timer = (50);
					BG96_PWR_status = BG96_PWR_W_ON;
				}
				break;
				
			case BG96_PWR_W_ON:
				if (BG96_wait_timer == 0)
					{
						//BG96_pwr_key_off();
						BG96_wait_timer = (ONE_SEC*5);
						BG96_PWR_status = BG96_PWR_ON_WAIT;
					}
				break;
				
			case BG96_PWR_ON_WAIT:
				if (BG96_wait_timer == 0)
					{
						printf("\r\n [BG96] power on!!\r\n");
						BG96_PWR_status = BG96_PWR_DONE;
						
						//while(1);
					}
				break;
			
		  case BG96_PWR_W_OFF:
				if (BG96_wait_timer == 0)
					{
						BG96_PWR_status = BG96_PWR_NONE;
						printf("\r\n [BG96] power off!!\r\n");
					}
				break;

		  default:
			  break;

	}
}

/*******************************************************************************
 * Socket allocation/deallocation
 ******************************************************************************/
 /**
* @brief  BG96 socket number assign 
* @param  none
* @retval socket number
*/

bool open_sockets[8] = {0,0,0,0,0,0,0,0};

static void BG96_socket_alloc(uint8_t sock)
{
	open_sockets[sock] = true;
}
		
static void BG96_socket_free(uint8_t sock)
{
	open_sockets[sock] = false;
}


/*******************************************************************************
 * BG96 INIT & DEINIT set waiting answer pointer an start timeout
 ******************************************************************************/
 /**
* @brief  BG96 set answer for general read functions
* @param  timeout, next state
* @retval result
*/
static void BG96_set_aswer(uint32_t timeout, BG96_STATE next)
{
		BG96_timeout_load(timeout);
    BG96_status = BG96_W_ANSW;
    BG96_next_status = next;
}

/**************************************************************
 * PUBLIC FUNCTIONS
 *************************************************************/
#define		BG96_size		4096
uint8_t 	BG96_buff[BG96_size];
uint16_t 	BG96_buff_ix = 0;

/*************************************************
 * RX IRQ callback
 *************************************************/
 /**
* @brief  IRQ rx from BG96
* @param  rx char
* @retval result
*/
void BG96_rx_callback(uint8_t rxc)
{
	
	BG96_buff[BG96_buff_ix] = rxc;
	BG96_buff_ix = (BG96_buff_ix+1)%BG96_size;
	
  BG96_rx_data[BG96_ix] = rxc;
  BG96_ix = (BG96_ix + 1)%RX_MAX_SIZE;

	BG96_rx_chr[BG96_rx_ix] = rxc;
	BG96_rx_ix = (BG96_rx_ix+1)%BG96_BUFF_SIZE;

    switch(rxc){
        case '\r':
            //if two consecutive \r, previous response echo!!
            if (uart_rx_status_flag == 0x08){
                //overwrite previous \r to fix answer
            	BG96_rx_chr[(BG96_rx_ix-2)%BG96_BUFF_SIZE] = 0;
                rx_start[answer_w_point] = rx_scan;
                answer_w_point = (answer_w_point+1)%MAX_ANSWER_NUMBER;
                //if answer_write pointer roll-over the answer read pointer, overwrite flag set
                //IMPORTANT if overwrite flag setted, some messages will be lost!!
                if (answer_w_point == answer_r_point)
                {
                    rx_overwrite = true;
                }

                rx_scan = (BG96_rx_ix-1)%BG96_BUFF_SIZE;
            }
                uart_rx_status_flag |= 0x8;
            break;

        case '\n':
            //previous char is \r?
            if (uart_rx_status_flag == 0x8)
               {

                   //valid terminator, reception complete!!
                   //string terminator for strcmp functions
            		BG96_rx_chr[BG96_rx_ix] = 0;
                    BG96_rx_ix = (BG96_rx_ix+1)%BG96_BUFF_SIZE;
                    rx_start[answer_w_point] = rx_scan;
                    //increment valid answer_write pointer
                    answer_w_point = (answer_w_point+1)%MAX_ANSWER_NUMBER;
                    //if answer_write pointer roll-over the answer read pointer, overwrite flag set
                    //IMPORTANT if overwrite flag setted, some messages will be lost!!
                    if (answer_w_point == answer_r_point)
                    {
                        rx_overwrite = true;
                    }

                    rx_scan = BG96_rx_ix;
               }

           uart_rx_status_flag = 0;
           //w_end_terminator = false;
           break;

        case '>':
        	prompt_flag = true;
        	break;

        case ' ':
        	if (prompt_flag == true){
        		prompt_flag = false;
                //valid SMS prompt, reception complete!!
                //string terminator for strcmp functions
         		BG96_rx_chr[BG96_rx_ix] = 0;
                BG96_rx_ix = (BG96_rx_ix+1)%BG96_BUFF_SIZE;
                 rx_start[answer_w_point] = rx_scan;
                 //increment valid answer_write pointer
                 answer_w_point = (answer_w_point+1)%MAX_ANSWER_NUMBER;
                 //if answer_write pointer roll-over the answer read pointer, overwrite flag set
                 //IMPORTANT if overwrite flag setted, some messages will be lost!!
                 if (answer_w_point == answer_r_point)
                 {
                     rx_overwrite = true;
                 }

                 rx_scan = BG96_rx_ix;
            }
        	break;

        default:
        	prompt_flag = false;
        	uart_rx_status_flag = 0;
            break;
    }

}

/*******************************************************************************
 * IRQ timer 1msec callback for BG96 general timer application
 ******************************************************************************/
uint8_t disable_recv = 0;

 /**
* @brief  BG96 general function timers
* @param  none
* @retval none
*/
void BG96_timers_callback(void)
{

	if (BG96_wait_timer)
		BG96_wait_timer--;

	if (BG96_timeout)
		BG96_timeout--;
	
	//check if socket data pending..
	BG96_socket_recv();
	
}

/*******************************************************************************
 * BG96 check status and power general utility functions
 ******************************************************************************/

uint8_t BG96_check_ready(void)
{
	if (BG96_status == BG96_READY)
		return true;
	return false;
}

bool BG96_power_on(void)
{
	if (BG96_PWR_status == BG96_PWR_DONE)
		return true;
		
	BG96_power(BG96_POWER_ON);
	return false;
}

bool BG96_power_off(void)
{
	if (BG96_PWR_status == BG96_PWR_NONE)
		return true;
	
	BG96_power(BG96_POWER_OFF);
		return false;
}

void BG96_RESET_state(void)
{
		BG96_status = BG96_RESET;
}

bool check_BG96_restart(void)
{
		return  BG96_restart_flag;
}

void BG96_restart(void)
{
	if (BG96_power_off() == true)
	{
		BG96_RESET_state();
		BG96_restart_flag = false;
	}
		
}

/*******************************************************************************
 * BG96 init and check if GSM connection ready!!
 ******************************************************************************/
//////////////////////////////////////////////////////
//INIT MODEM, GO TO NETWORK
 /**
* @brief  BG96 startup and functional configuration
* @param  none
* @retval none
*/
void BG96_init(void)
{
	BG96_result res;
	//uint8_t apn_size;
	bool pin_valid;
	int i;
	//uint8_t mc60_ev;
	char send_string[80];
	//memset (send_string, 0, sizeof(send_string));
	switch (BG96_status){
		
		case BG96_RESET:
			printf(" [BG96] Reset\r\n");
			BG96_status = BG96_IDLE;
			//check power...
			break;

		case BG96_IDLE:
			BG96_answ_point = 0;	
				//restore factory default
				vPrintf("AT&F\r\n", 0);
		    BG96_set_aswer(0, BG96_ECHO);
		    break;
		
		case BG96_ECHO:
			BG96_answ_point = 0;	
				//disable ECHO
		    vPrintf("ATE0\r\n", 0);
				BG96_set_aswer(0, BG96_CONNECT);
				break;
			
		case BG96_CONNECT:
			BG96_answ_point = 0;
				//set low functionallity to reset internal status
				vPrintf("AT+CFUN=0\r\n", 0);
		    BG96_set_aswer(0, BG96_CFUN_DELAY);
		    break;
		
		case BG96_CFUN_DELAY:
				BG96_wait_timer = (ONE_SEC*5);
				BG96_status = BG96_PIN_REQUEST;
				break;
		
		case BG96_PIN_REQUEST:
				if (BG96_wait_timer == 0)
				{
					//BG96_answ_point = (uint8_t*)BG96_W_CPIN;
					BG96_answ_point = 0;
					//request PIN state to reset memory ...
					vPrintf("AT+CPIN?\r\n", 0);
					BG96_set_aswer(0, BG96_INIT);
				}
				break;

		case BG96_INIT:
			//BG96_answ_point = (uint8_t*)BG96_W_CPIN;
			BG96_answ_point = 0;
				//set full modem functionallity
		    vPrintf("AT+CFUN=1\r\n", 0);
				BG96_wait_timer = (ONE_SEC*2);
		    BG96_set_aswer(0, BG96_PIN_SEND);
				//BG96_set_aswer(0, BG96_PIN_REQUEST);
		    break;

		case BG96_PIN_SEND:
				//send PIN
				if (BG96_wait_timer == 0)
				{
					pin_valid = false;
					do
					{
						if (SIM_Pin[i] != 0x30)
							{
							pin_valid = true;
							}
							i++;
					}while ((SIM_Pin[i] != 0) || (i > sizeof(SIM_Pin)));
						
					if (pin_valid == false)
					{
						printf(" [BG96] SIM PIN = 0, nothing to send\r\n");
						BG96_wait_timer = (ONE_SEC*5);
						BG96_status = BG96_COPS_RQST;
						BG96_next_status = BG96_COPS_RQST;
						break;
					}
						
					BG96_answ_point = 0;
					//BG96_answ_point = (uint8_t*)BG96_MSG_PIN_READY;
					memset (send_string, 0, sizeof(send_string));
					memcpy (send_string, "AT+CPIN=", 10 );
					strcat (send_string, (const char *)SIM_Pin);
					strcat(send_string, "\r");
					printf(" [BG96] send PIN %s\n", send_string);
					vPrintf(send_string, strlen(send_string));
					BG96_set_aswer(0, BG96_PB_READY);
				}
		    break;
		
		case BG96_PB_READY:
			BG96_answ_point = (uint8_t*)BG96_MSG_PIN_READY;
				printf(" [BG96] Wait PIN\r\n");
				vPrintf("AT+CPIN?\r\n", 0);
				//wait pin ready message...
		    BG96_set_aswer(0, BG96_W_OK);
		    break;

		case BG96_W_OK:
			BG96_answ_point = 0;
				//wait for OK after pin ready...
				BG96_wait_timer = (ONE_SEC*5);
		    BG96_set_aswer(0, BG96_COPS_RQST);
		    break;		
		
		case BG96_COPS_RQST:
			if (BG96_wait_timer == 0)
				{
					BG96_answ_point = 0;
					//check cops...only for debug no action taken
					vPrintf("AT+COPS?\r\n", 0);
					BG96_set_aswer(0, BG96_APN_CFG);
				}
		    break;

		case BG96_APN_CFG:
			BG96_answ_point = 0;
				//configure APN 
				printf(" [BG96] set APN & ACT\r\n");
				memset (send_string, 0, sizeof(send_string));
				memcpy (send_string, "AT+QICSGP=1,1,\"", 15 );
				/*
				apn_size = sizeof(APN_SETTING);
				//check len of APN name, if more than 40 chars truncate!!
				if ((apn_size) > 40)
					apn_size = 40;
				*/
				//prepare string to set APN
				strcat (send_string, SIM_Apn);
				//memcpy (&send_string[14], APN_SETTING, apn_size);
				strcat (send_string, "\",\"\",\"\",1\r");
				vPrintf(send_string, strlen(send_string));
		    BG96_set_aswer(0, BG96_ACT_ACTIVATE);
		    break;

		case BG96_ACT_ACTIVATE:
			BG96_answ_point = 0;
			//activate ACT context 1
			vPrintf("AT+QIACT=1\r\n", 0);
		    BG96_set_aswer(0, BG96_APN_READ);
		    break;
		
		case BG96_APN_READ:
			BG96_answ_point = 0;
			//chek ACT ... only for debug no action taken
			vPrintf("AT+QIACT?\r\n", 0);
				BG96_set_aswer(0, B96_MAC_RQST);
		    break;
		
		case B96_MAC_RQST:
			BG96_answ_point = (uint8_t*)BG96_NULL;
			//get BG96 serial number to compute MAC of this device
			vPrintf("AT+CGSN\r", 0);
		    BG96_set_aswer(0, B96_MAC_RQST_OK);
		    break;
		
		case B96_MAC_RQST_OK:
			BG96_answ_point = 0;
			memcpy(BG96_sn, BG96_answer_data, sizeof(BG96_sn));
		    BG96_set_aswer(0, BG96_INIT_DONE);
		    break;

		case BG96_INIT_DONE:
				//BG96 ready!!
		    BG96_status = BG96_READY;
				printf(" [BG96] Module ready!!\r\n");
				ind_bg96_connected();
		    BG96_next_status = BG96_READY;
			break;
		
		case BG96_READY:
			break;

		case BG96_BUSY_ERROR:
			//if error received, wait for next answer or timeout
			BG96_set_aswer(0, BG96_W_ANSW);
			//BG96_status = BG96_W_ANSW;
			break;

		case BG96_W_ANSW:
			res = BG96_receive(0);

			switch (res){

				case BG96_OK:
					if (BG96_answ_point == 0)
						BG96_status = BG96_next_status;
					break;

				case BG96_ASNWER_DONE:
					BG96_status = BG96_next_status;
					break;

				case BG96_ERROR:
					printf(" [BG96] ERROR during config, check settings\r\n");
				case BG96_RX_TIMEOUT:
					BG96_status = BG96_BUSY_ERROR;
					break;
				
				case BG96_CME_ERROR:
					if (BG96_next_status == BG96_INIT)
					{
						BG96_status = BG96_next_status;
					}
					else
					{
						printf(" [BG96] CME or CMS ERROR during config, check settings\r\n");
						BG96_status = BG96_BUSY_ERROR;
					}
					break;

				default:
					break;
			}
			break;

		default:
			break;
	}

}


/*******************************************************************************
 * BG96 mQTT interface functions section
 ******************************************************************************/

/*******************************************************************************
 * SOCKET send function
 ******************************************************************************/

static BG96_READ_STATE BG96_READ_status = BG96_READ_NONE;
static BG96_READ_STATE BG96_READ_next_status = BG96_READ_NONE;

 bool socket_data_to_read =  false;
 bool socket_closed				=  false;

static BG96_SEND_RESULT BG96_socket_send(uint8_t socket, uint32_t size, uint8_t* message)
{
	static BG96_SEND_STATE BG96_SEND_status = BG96_SEND_NONE;
	static BG96_SEND_STATE BG96_SEND_next_status = BG96_SEND_NONE;
	
	BG96_SEND_RESULT send_result = SEND_IN_PROGRESS;
	char send_string[32];
	BG96_result res;

	//ir IRQ read in process, wait...
	if (BG96_READ_status != BG96_READ_NONE)
		return send_result;
	
	switch (BG96_SEND_status){

		case BG96_SEND_NONE:
			BG96_SEND_status = BG96_SEND_REQUEST;
			//enable IRQ socket receive (BG96_receive) 
			disable_recv &= 0xfe;
			break;

		case BG96_SEND_REQUEST:
			//disable IRQ socket receive (BG96_receive) to prevent unwanted reentrant calls
			disable_recv |= 0x01;
			BG96_answ_point = (uint8_t*)BG96_MSG_SEND_PROMPT;
			memset (send_string, 0, sizeof(send_string));
			sprintf(send_string, "AT+QISEND=%d,%d\r\n", socket, size);
			vPrintf(send_string, strlen(send_string));
			//printf("QISEND = %d\r\n", size);
			BG96_timeout_load(10000);
			BG96_SEND_status = BG96_SEND_ANSW;
			BG96_SEND_next_status = BG96_SEND_SEND;
			//while(1);		//ABELE test
			break;

		case BG96_SEND_SEND:
			BG96_answ_point = (uint8_t*)BG96_MSG_SEND_OK;
			//send message string to BG96
			vPrintf ((char *)message, size);
			BG96_timeout_load(0);
			BG96_SEND_status = BG96_SEND_ANSW;
			BG96_SEND_next_status = BG96_SEND_DONE;
			break;

		case BG96_SEND_DONE:
			send_result = SEND_DONE;
			//enable IRQ socket receive (BG96_receive)
			disable_recv &= 0xfe;
			BG96_SEND_status = BG96_SEND_NONE;
			BG96_SEND_next_status = BG96_SEND_DONE;
			break;
		
		case BG96_SEND_ERROR:
			BG96_SEND_status = BG96_SEND_NONE;
			send_result = SEND_ERROR;
			break;

		case BG96_SEND_ANSW:
			res = BG96_receive(0);

			switch (res){
			case BG96_OK:
				if (BG96_answ_point == 0)
					BG96_SEND_status = BG96_SEND_next_status;
				break;

			case BG96_ASNWER_DONE:
				BG96_SEND_status = BG96_SEND_next_status;
				break;

			case BG96_SOCK_CLOSED:
				socket_closed = true;
				//BG96_READ_status = BG96_READ_ERROR_SOCK;
				break;
							
			case BG96_SOCK_DATA:
				socket_data_to_read = true;
				//BG96_READ_status = BG96_READ_PROCESS;
				break;

			case BG96_RX_TIMEOUT:
			case BG96_ERROR:
				printf(" [BG96] ERROR, application stops!!\r\n");
				BG96_SEND_status = BG96_SEND_ERROR;
				BG96_SEND_next_status = BG96_SEND_ERROR;
				//send_result = SEND_ERROR;
				break;

			default:
				break;
			}
			break;

		default:
			break;
	}
	return send_result;
}

 /**
* @brief  BG96 socket write interface to application
* @param  socket id (unused) len of data, pointer to data string
* @retval none
*/
BG96_Status_t BG96_socket_write (uint8_t sock_id, uint16_t DataLength, char * pData)
{
  //Check if sock_id is open
  if(!open_sockets[sock_id])
    return BG96_NOT_READY;

  if(DataLength>=1021 || DataLength<=0)
    return BG96_NOT_SUPPORTED;

  //Queue_Client_Write_Event(sock_id,DataLength,pData);
  //status = USART_Receive_AT_Resp( );
  //return status;
	BG96_SEND_RESULT result;
	//sock_id = 1;

	do
	{
			result = BG96_socket_send(sock_id, DataLength, (uint8_t*)pData);
	}while (result == SEND_IN_PROGRESS);
	
	if (result == SEND_ERROR)
		return BG96_AT_CMD_RESP_ERROR;		//--> -1 = MQTT_ERROR
	return BG96_MODULE_SUCCESS;
	
}



/*******************************************************************************
 * SOCKET read function
 ******************************************************************************/

uint32_t MQTT_read_rq = 0;

uint8_t socket_data_buffer[SIZE_SOCKET_BUFFER];
uint32_t local_amount = 0;
uint32_t local_pt = 0;
uint32_t recv_amount = 0;
uint32_t stored_amount = 0;
const uint8_t socket_ID = 1;

//for debug
bool size_err_mesg_flag = false;
uint32_t tot_am;
uint16_t recv_amount_log[20];
uint8_t ra_ix = 0;
uint16_t total_amount_log[20];
uint8_t ta_ix = 0;

static void BG96_set_READ_aswer(uint32_t timeout, BG96_READ_STATE next)
{
		BG96_timeout_load(timeout);
    BG96_READ_status = BG96_READ_ANSW;
    BG96_READ_next_status = next;
		//if ((BG96_READ_next_status != BG96_READ_NONE) && (BG96_READ_next_status != BG96_READ_PROCESS))
			//printf("next read stat %d\r\n", BG96_READ_next_status);
}

 /**
* @brief  BG96 check if socket data pending ... called on irq
* @param  none
* @retval BG96 result
*/
BG96_result BG96_socket_recv(void)
{
	
		if (BG96_check_ready() == false)
			return BG96_ERROR;
	
		//calls counter, only for debug..
		MQTT_read_rq++;
		BG96_result res = BG96_NONE;
		int stcp = 0;
		//int i;
		
		//unwanted reentrant prewent flag
		if (disable_recv != 0) 
		{
			BG96_READ_status = BG96_READ_NONE;
			return res;	
		}

		switch (BG96_READ_status)
		{
			case BG96_READ_NONE:
				//during SOCKET SEND or SOCKET OPEN, if socket close is received ...
				if (socket_closed == true)
				{
					socket_closed = false;
					//if socket closed, no data can be readed??
					socket_data_to_read = false;
					BG96_READ_status = BG96_READ_ERROR_SOCK;
					break;
				}
				else
				//during SOCKET SEND or SOCKET OPEN, if socket data are pending ...
				if (socket_data_to_read == true)
				{
					socket_data_to_read = false;
					//printf("RXflag!!\r\n");
					stored_amount = 0;
					BG96_READ_status = BG96_READ_PROCESS;	
					break;
				}
				
				BG96_answ_point = 0;
				BG96_set_READ_aswer(5, BG96_READ_PROCESS);
				break;
			
			case BG96_READ_ERROR_SOCK:
				BG96_answ_point = 0;
				vPrintf("AT+QICLOSE=1\r\n", 0);
				BG96_set_READ_aswer(0, BG96_READ_ERROR_SOCK_CLOSED);
				break;
			
			case BG96_READ_ERROR_SOCK_CLOSED:
				//BG96_status = BG96_SOCKET_OPEN;
				BG96_READ_status = BG96_READ_NONE;
				break;
			
			case BG96_READ_PROCESS:
				BG96_answ_point = 0;
				//printf("Read process!!\r\n\n");
				HAL_NVIC_DisableIRQ(USARTbg_IRQn);
				BG96_ix = 0;
				memset(BG96_rx_data, 0, sizeof(BG96_rx_data));
				HAL_NVIC_EnableIRQ(USARTbg_IRQn);
				vPrintf("AT+QIRD=1\r\n", 0);
				BG96_set_READ_aswer(0, BG96_READ_DONE);
				break;
			
			case BG96_READ_DONE:
				//find receiving data amount
				sscanf(&BG96_rx_data[9], "%u", &recv_amount);
				//log for debug...
				recv_amount_log[ra_ix] = recv_amount;
				ra_ix = (ra_ix+1)%20;
				//printf("QUIRD = %d\r\n", recv_amount);

				//store data until recv amount is = 0
				if (recv_amount == 0)
				{
					//printf("END data rx %d\r\n", stored_amount);
					ind_bg96_socket_data_received(socket_ID, socket_data_buffer, stored_amount, stored_amount);
					stored_amount = 0;
					BG96_READ_status = BG96_READ_NONE;
					BG96_READ_next_status = BG96_READ_NONE;
					break;
				}

				//searc 0x0A ... to find starting data area
				while(1)
				{
					if (BG96_rx_data[stcp+9] == 0x0A)
						break;
					stcp++;
				}
				//compute data inital pointer
				stcp = (stcp+1+9);
				
				//to prevent exceed from buffer size...
				tot_am = (stored_amount+recv_amount);
				if (tot_am > sizeof(socket_data_buffer))
				{
					//printf("--->>>> ERROR DATA PENDING EXCEED BUFFER SIZE, DISCHARGED!!\r\n");
					BG96_READ_status = BG96_READ_NONE;
					size_err_mesg_flag = true;
				}
				else
				{
					//copy data into buffer...
					memcpy (&socket_data_buffer[stored_amount], &BG96_rx_data[stcp], recv_amount);
					stored_amount += recv_amount;
					BG96_READ_status = BG96_READ_PROCESS;
					BG96_READ_next_status = BG96_READ_PROCESS;
				}
				break;
			
			case BG96_READ_ERROR:
				BG96_READ_status = BG96_READ_NONE;
				recv_amount = 0;
				break;

			case BG96_READ_ANSW:
				res = BG96_receive(0);
					switch (res)
					{
						case BG96_OK:
							if (BG96_answ_point == 0)
								BG96_READ_status = BG96_READ_next_status;
							break;

						case BG96_ASNWER_DONE:
							BG96_READ_status = BG96_READ_next_status;
							break;

						case BG96_RX_TIMEOUT:
						case BG96_ERROR:
							BG96_READ_status = BG96_READ_ERROR;
							break;
						
						case BG96_SOCK_CLOSED:
							ind_bg96_socket_client_remote_server_closed((uint8_t *)&socket_ID);		//same as wifi..
							BG96_socket_free(socket_ID);
							BG96_READ_status = BG96_READ_ERROR_SOCK;
							break;
							
						case BG96_SOCK_DATA:
							if (BG96_READ_next_status == BG96_READ_PROCESS)	
							{
								BG96_READ_status = BG96_READ_PROCESS;	
								//socket_data_to_read = true;
								stored_amount = 0;
							}
							//else
								//printf("\r\n\n *** SOCK DATA FAILURE %d***\r\n\n", BG96_READ_next_status);
							break;
													
						case BG96_NONE:
							if (BG96_READ_next_status == BG96_READ_PROCESS)	
								BG96_READ_status = BG96_READ_NONE;
							break;

						default:
							break;
					}
				break;
			}

		return res;
	
}

/*******************************************************************************
 * SOCKET close function
 ******************************************************************************/
 
/**
  * @brief  Close socket, unused
  * @param  n : Network structure
  * @retval None
  */
void BG96_socket_close (uint8_t sock_id)
{
	//TODO implement socket close dinamic as needed
	//the demo has only socket 1..
	socket_closed = true;
}


/*******************************************************************************
 * SOCKET open function
 ******************************************************************************/
 /**
* @brief  BG96 open socket
* @param  hostname, port, socket type, socket id (unused here)
* @retval BG96 result
*/
static BG96_OPEN_RESULT BG96_socket(uint8_t * hostname, uint32_t port_number, uint8_t * type, uint8_t sock_id)
{
	
	static BG96_OPEN_STATE BG96_OPEN_status = BG96_OPEN_NONE;
	static BG96_OPEN_STATE BG96_OPEN_next_status = BG96_OPEN_NONE;

	BG96_OPEN_RESULT open_result = OPEN_IN_PROGRESS;
	char send_string[256];
	BG96_result res;

	//ir IRQ read in process, wait...
	if (BG96_READ_status != BG96_READ_NONE)
		return open_result;
	
	switch (BG96_OPEN_status)
	{

		case BG96_OPEN_NONE:
			//enable IRQ socket receive (BG96_receive) 
			disable_recv &= 0xfd;
			BG96_OPEN_status = BG96_OPEN_REQUEST;
			break;

		case BG96_OPEN_REQUEST:
			//disable IRQ socket receive (BG96_receive) to prevent unwanted reentrant calls
			disable_recv |= 0x02;
			BG96_answ_point = 0;
			memset (send_string, 0, sizeof(send_string));
			snprintf(send_string, sizeof(send_string), "AT+QIOPEN=1,%d,\"%s\",\"%s\",%d,0,0\r", (uint8_t)sock_id, type, hostname, port_number);
			vPrintf(send_string, strlen(send_string));
			BG96_timeout_load(10000);
			BG96_OPEN_status = BG96_OPEN_ANSW;
			BG96_OPEN_next_status = BG96_OPEN_PROCESS;
			break;

		case BG96_OPEN_PROCESS:
			//wait for socket open ...
			BG96_answ_point = (uint8_t*)BG96_MSG_OPEN_OK;
			BG96_timeout_load(0);
			BG96_OPEN_status = BG96_OPEN_ANSW;
			BG96_OPEN_next_status = BG96_OPEN_DONE;
			break;

		case BG96_OPEN_DONE:
			open_result = OPEN_DONE;
			//enable IRQ socket receive (BG96_receive) 
			disable_recv &= 0xfd;
			BG96_OPEN_status = BG96_OPEN_NONE;
			BG96_OPEN_next_status = BG96_OPEN_NONE;
			break;
		
		case BG96_OPEN_ERROR:
			BG96_OPEN_status = BG96_OPEN_NONE;
			open_result = OPEN_ERROR;
			break;

		case BG96_OPEN_ANSW:
			res = BG96_receive(0);

			switch (res){
			case BG96_OK:
				if (BG96_answ_point == 0)
					BG96_OPEN_status = BG96_OPEN_next_status;
				break;

			case BG96_ASNWER_DONE:
				BG96_OPEN_status = BG96_OPEN_next_status;
				break;

			case BG96_SOCK_CLOSED:
				socket_closed = true;
				//BG96_READ_status = BG96_READ_ERROR_SOCK;
				break;
							
			case BG96_SOCK_DATA:
				socket_data_to_read = true;
				//BG96_READ_status = BG96_READ_PROCESS;
				break;

			case BG96_RX_TIMEOUT:
			case BG96_ERROR:
				BG96_OPEN_status = BG96_OPEN_ERROR;
				BG96_OPEN_next_status = BG96_OPEN_ERROR;
				break;

			default:
				break;
			}
			break;

		default:
			break;
	}
	return open_result;
}

 /**
* @brief  BG96 open socket interface to application
* @param  hostname, port, socket type, socket id (unused here)
* @retval BG96 result
*/
int BG96_socket_open(uint8_t * hostname, uint32_t port_number, uint8_t * type, uint8_t sock)
{
		char send_type[16];
		const char *send_type_pt = send_type;
		BG96_OPEN_RESULT result;
		uint8_t sock_id = sock;
	
	  switch(*type) 
			{
        case 'u':
            send_type_pt = "UDP";
            //sock_id = 0;
            break;
        case 't':
            send_type_pt = "TCP";
            //sock_id = 1;
            break;
        default:
            send_type_pt = "TCP";
            break;
			};
	
	do
	{
			result = BG96_socket(hostname, port_number, (uint8_t *)send_type_pt, sock_id);
	}while (result == OPEN_IN_PROGRESS);
	
	if (result == OPEN_ERROR)
		return false;	
	BG96_socket_alloc(sock_id);
	return true;
}
	
	


/*******************************************************************************
 * BG96 init interface 
 ******************************************************************************/
 /**
* @brief  BG96 starturp, called from platform_init() main function
* @param  pin, apn
* @retval boolean
*/
bool gsm_init(char* pin, char* apn)
{
	
	memcpy(SIM_Pin, pin, sizeof(SIM_Pin));
	memcpy(SIM_Apn, apn, sizeof(SIM_Apn));
		
	if (BG96_power_on() != true)
	{
		return false;
	}
	
	if (BG96_check_ready() == false)
	{
		BG96_init();
		return false;
	}

	return true;
}


/*******************************************************************************
 * General interface utility functions
 ******************************************************************************/
 /**
* @brief  Compute MAC from BG96 serial number
* @param  pointer to application MAC string
* @retval boolean
*/

#define IMEI_OFFS			0

void GET_BG96_mac(uint8_t* mac)
{
   /* 00 80 e1 b7 d3 37 for Avnet ST*/
	 uint8_t sn_to_mac[6];
	 int i;
	
   sn_to_mac[0]=0x00;
   sn_to_mac[1]=0x80;
   sn_to_mac[2]=(uint8_t)(((BG96_sn[4+IMEI_OFFS]&0x0f)<<4 | (BG96_sn[5+IMEI_OFFS]&0x0f)) + ((BG96_sn[12+IMEI_OFFS]&0x0f)<<4));
   sn_to_mac[3]=(uint8_t)(((BG96_sn[6+IMEI_OFFS]&0x0f)<<4 | (BG96_sn[7+IMEI_OFFS]&0x0f)) + (uint8_t)((BG96_sn[0+IMEI_OFFS]&0x0f)<<4 | (BG96_sn[1+IMEI_OFFS]&0x0f)));
   sn_to_mac[4]=(uint8_t)(((BG96_sn[8+IMEI_OFFS]&0x0f)<<4 | (BG96_sn[9+IMEI_OFFS]&0x0f)) + ((BG96_sn[13+IMEI_OFFS]&0x0f)));
   sn_to_mac[5]=(uint8_t)(((BG96_sn[10+IMEI_OFFS]&0x0f)<<4 | (BG96_sn[11+IMEI_OFFS]&0x0f)) + (uint8_t)((BG96_sn[2+IMEI_OFFS]&0x0f)<<4 | (BG96_sn[3+IMEI_OFFS]&0x0f)));
	
	for (i=0; i<6; i++)
		sprintf ((char*)&mac[i*3], "%02x:", sn_to_mac[i]);
	
	mac[(i*3)-1] = 0;

}

/*******************************************************************************
 * Azure Socket interface utility functions
 ******************************************************************************/

const uint8_t socket_number[4] = {0,1,2,3};
/**
* @brief  BG96_socket_client_open
*         Open a network socket
* @param  Hostname hostname to connect to
*         portnumber portnumber of the Host to connect to
*         protocol tcp or udp protocol
*         sock_id socket id of the opened socket returned to the user
* @retval BG96_Status_t : return status of socket open request
*/
BG96_Status_t BG96_socket_client_open(uint8_t * hostname, uint32_t port_number, uint8_t * protocol, uint8_t * sock_id) 
{
	uint8_t BG96_sock_id;
	//fixed socket number = 1, TODO implement dinamic socket number assign...
	*sock_id = socket_number[1];
	BG96_sock_id = socket_number[1];
	if (BG96_socket_open(hostname, port_number, protocol, BG96_sock_id) == false)
	    return BG96_TIME_OUT_ERROR;		//BG96_MODULE_ERROR;
	return BG96_MODULE_SUCCESS;
}

/**
* @brief  wBG96_socket_client_write
*         Write len bytes of data to socket
* @param  sock_id socket ID of the socket to write to
*         DataLength: data length to send
*         pData : pointer of data buffer to be written
* @retval BG96_Status_t : return status of socket write request
*/
BG96_Status_t BG96_socket_client_write(uint8_t sock_id, uint16_t DataLength, char * pData)
{
	BG96_Status_t status;
	status = BG96_socket_write(sock_id, DataLength, pData);
	    //return BG96_MODULE_ERROR;
		//return BG96_MODULE_SUCCESS;
	return status;
}

/**
* @brief  wifi_socket_client_close
*         The SOCKC command allows to close socket
* @param  sock_close_id the socket ID of the socket which needs to be closed.
* @retval WiFi_Status_t : return status of socket close request
*/
BG96_Status_t BG96_socket_client_close(uint8_t sock_close_id)
{
			//TODO implement close as needed!!!
	    return BG96_MODULE_ERROR;
}

#endif /* BG96_ENABLE */



