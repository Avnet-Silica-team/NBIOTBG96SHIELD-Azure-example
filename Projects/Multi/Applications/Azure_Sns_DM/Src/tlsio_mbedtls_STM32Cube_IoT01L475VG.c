// Copyright (c) Microsoft. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

/* Adaptation to the net.h interface:
 * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V. 
 * All rights reserved.</center></h2>
 */

#ifdef USE_MBED_TLS

/* Includes ------------------------------------------------------------------*/
#include <stdlib.h>

#include "mbedtls/debug.h"
#include "mbedtls/ssl.h"
#include "mbedtls/entropy.h"
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/error.h"
#include "mbedtls/certs.h"
#include "mbedtls/entropy_poll.h"

#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "TargetFeatures.h"
#include "net.h"
#include "mbedtls_net.h"
#include "msg.h"
#include "azure_c_shared_utility/optimize_size.h"
#include "azure_c_shared_utility/tlsio.h"
#include "tlsio_mbedtls_STM32Cube_iot01l475vg.h"

extern int mbedtls_hardware_poll (void *data, unsigned char *output, size_t len, size_t *olen);
extern void *hnet;

/* Private typedef -----------------------------------------------------------*/
typedef enum TLSIO_STATE_ENUM_TAG
{
  TLSIO_STATE_NOT_OPEN,
  TLSIO_STATE_OPENING_UNDERLYING_IO,
  TLSIO_STATE_IN_HANDSHAKE,
  TLSIO_STATE_OPEN,
  TLSIO_STATE_CLOSING,
  TLSIO_STATE_ERROR
} TLSIO_STATE_ENUM;

typedef struct TLS_IO_INSTANCE_TAG
{
  net_sockhnd_t sockhnd;
  ON_BYTES_RECEIVED on_bytes_received;
  ON_IO_OPEN_COMPLETE on_io_open_complete;
  ON_IO_CLOSE_COMPLETE on_io_close_complete;
  ON_IO_ERROR on_io_error;
  void* on_bytes_received_context;
  void* on_io_open_complete_context;
  void* on_io_close_complete_context;
  void* on_io_error_context;
  TLSIO_STATE_ENUM tlsio_state;
  ON_SEND_COMPLETE on_send_complete;
  void* on_send_complete_callback_context;
  mbedtls_entropy_context    entropy;
  mbedtls_ctr_drbg_context   ctr_drbg;
  mbedtls_ssl_context        ssl;
  mbedtls_ssl_config         config;
  mbedtls_x509_crt           cacert;
  mbedtls_ssl_session        ssn;
  const unsigned char * cacert_pem;   /**< Reference to the root CA certificate of the remote host. Backup used at reconnection time. */
  char *  hostname;                   /**< Copy of the hostname of the remote host. Backup used at reconnection time. */
  int port;                           /**< Port number of the remote host. Backup used at reconnection time. */
} TLS_IO_INSTANCE;

static const IO_INTERFACE_DESCRIPTION tlsio_mbedtls_interface_description =
{
  tlsio_mbedtls_STM32Cube_retrieveoptions,
  tlsio_mbedtls_STM32Cube_create,
  tlsio_mbedtls_STM32Cube_destroy,
  tlsio_mbedtls_STM32Cube_open,
  tlsio_mbedtls_STM32Cube_close,
  tlsio_mbedtls_STM32Cube_send,
  tlsio_mbedtls_STM32Cube_dowork,
  tlsio_mbedtls_STM32Cube_setoption
};

/* Private defines -----------------------------------------------------------*/
#define MBED_TLS_DEBUG_ENABLE

/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
#if defined (MBED_TLS_DEBUG_ENABLE)
void mbedtls_debug(void *ctx, int level,const char *file, int line, const char *str );
#endif

/* Functions Definition ------------------------------------------------------*/

#if defined (MBED_TLS_DEBUG_ENABLE)
void mbedtls_debug(void *ctx, int level,const char *file, int line, const char *str )
{
  ((void) level);
  printf("%s (%d): %s\r\n", file,line,str);
}
#endif

static void indicate_error(TLS_IO_INSTANCE* tls_io_instance)
{
  if (tls_io_instance->on_io_error != NULL)
  {
    tls_io_instance->on_io_error(tls_io_instance->on_io_error_context);
  }
}

static void indicate_open_complete(TLS_IO_INSTANCE* tls_io_instance, IO_OPEN_RESULT open_result)
{
  if (tls_io_instance->on_io_open_complete != NULL)
  {
    tls_io_instance->on_io_open_complete(tls_io_instance->on_io_open_complete_context, open_result);
  }
}

static int decode_ssl_received_bytes(TLS_IO_INSTANCE* tls_io_instance)
{
  int result = 0;
  unsigned char buffer[64];
  int rcv_bytes = 1;

  while (rcv_bytes > 0)
  {
    rcv_bytes = mbedtls_ssl_read(&tls_io_instance->ssl, buffer, sizeof(buffer));
    if (rcv_bytes > 0)
    {
      if (tls_io_instance->on_bytes_received != NULL)
      {
        tls_io_instance->on_bytes_received(tls_io_instance->on_bytes_received_context, buffer, rcv_bytes);
      }
    }
    else
    {
      switch(rcv_bytes)
      {
        case MBEDTLS_ERR_SSL_WANT_READ:  
        case MBEDTLS_ERR_SSL_WANT_WRITE:
          break;
        case 0:
          msg_error("mbedtls_ssl_read returned 0 (MBEDTLS_ERR_SSL_CONN_EOF).\n");
          msg_error("The TCP and TLS connections must be reset.\n");
          tls_io_instance->tlsio_state = TLSIO_STATE_ERROR;
          indicate_error(tls_io_instance); // Should rather be called from tlsio_mbedtls_STM32Cube_dowork()
          break;
        case MBEDTLS_ERR_SSL_CLIENT_RECONNECT:
          msg_error("mbedtls_ssl_read returned MBEDTLS_ERR_SSL_CLIENT_RECONNECT.\n");
          msg_error("The TLS connections must be reset via mbedtls_ssl_session_reset() (not necessarily the TCP connection).\n");
          tls_io_instance->tlsio_state = TLSIO_STATE_ERROR;
          indicate_error(tls_io_instance);
          break;
        default:
          msg_error("mbedtls_ssl_read returned %d\n", rcv_bytes);
          tls_io_instance->tlsio_state = TLSIO_STATE_ERROR;
          indicate_error(tls_io_instance);
          break;
      }
    }
  }

  return result;
}


int g_created = 0;

#if 0 // currently unused
static void on_underlying_io_close_complete(void* context)
{
  TLS_IO_INSTANCE* tls_io_instance = (TLS_IO_INSTANCE*)context;

  if (tls_io_instance->tlsio_state == TLSIO_STATE_CLOSING)
  {
    if (tls_io_instance->on_io_close_complete != NULL)
    {
      tls_io_instance->on_io_close_complete(tls_io_instance->on_io_close_complete_context);
    }
//	if (g_created)
//		tls_io_instance->tlsio_state = TLSIO_STATE_NOT_OPEN;
  }
}
#endif // 0

static void mbedtls_init(void *instance,const char *host) {
  TLS_IO_INSTANCE *result = (TLS_IO_INSTANCE *)instance;
  char *pers = "azure_iot_client";

  // mbedTLS initialize...
  mbedtls_entropy_init(&result->entropy);
  mbedtls_ctr_drbg_init(&result->ctr_drbg);
  mbedtls_ssl_init(&result->ssl);
  mbedtls_ssl_session_init(&result->ssn);
  mbedtls_ssl_config_init(&result->config);
  mbedtls_x509_crt_init(&result->cacert);
#if 1
    mbedtls_entropy_add_source(&result->entropy,mbedtls_hardware_poll,(void*)&TargetBoardFeatures.RngHandle.Instance,1,MBEDTLS_ENTROPY_SOURCE_STRONG);
#else
    mbedtls_entropy_add_source(&result->entropy,mbedtls_hardware_poll,(void*)&hrng,1,MBEDTLS_ENTROPY_SOURCE_STRONG);
#endif
  mbedtls_ctr_drbg_seed(&result->ctr_drbg,mbedtls_entropy_func,&result->entropy,(const unsigned char *)pers,strlen(pers));
  mbedtls_ssl_config_defaults(&result->config,MBEDTLS_SSL_IS_CLIENT,MBEDTLS_SSL_TRANSPORT_STREAM,MBEDTLS_SSL_PRESET_DEFAULT);
  mbedtls_ssl_conf_rng(&result->config,mbedtls_ctr_drbg_random,&result->ctr_drbg);
  mbedtls_ssl_conf_authmode(&result->config,MBEDTLS_SSL_VERIFY_REQUIRED);
  mbedtls_ssl_conf_min_version(&result->config,MBEDTLS_SSL_MAJOR_VERSION_3,MBEDTLS_SSL_MINOR_VERSION_3);          // v1.2
  mbedtls_ssl_set_bio(&result->ssl,result->sockhnd,mbedtls_net_send,mbedtls_net_recv,NULL);
  mbedtls_ssl_set_hostname(&result->ssl,host);
  mbedtls_ssl_set_session(&result->ssl,&result->ssn);

#if defined (MBED_TLS_DEBUG_ENABLE)
  mbedtls_ssl_conf_dbg(&result->config, mbedtls_debug,stdout);
  mbedtls_debug_set_threshold(1);
#endif

  if (mbedtls_ssl_setup(&result->ssl,&result->config) != 0)
  {
	msg_error("mbedtls_ssl_setup() allocation failed!\n");
  }
}

OPTIONHANDLER_HANDLE tlsio_mbedtls_STM32Cube_retrieveoptions(CONCRETE_IO_HANDLE handle)
{
  (void)handle;
  return NULL;
}

CONCRETE_IO_HANDLE tlsio_mbedtls_STM32Cube_create(void* io_create_parameters)
{
  TLSIO_CONFIG* tls_io_config = io_create_parameters;
  TLS_IO_INSTANCE* result;
  int ret = 0;
  
  if (tls_io_config == NULL)
  {
    LogError("NULL tls_io_config");
    result = NULL;
  }
  else
  {
    result = malloc(sizeof(TLS_IO_INSTANCE));
    if (result != NULL)
    {
      result->sockhnd = NULL;
      
      result->on_bytes_received = NULL;
      result->on_bytes_received_context = NULL;

      result->on_io_open_complete = NULL;
      result->on_io_open_complete_context = NULL;

      result->on_io_close_complete = NULL;
      result->on_io_close_complete_context = NULL;

      result->on_io_error = NULL;
      result->on_io_error_context = NULL;

      result->cacert_pem = NULL;
      result->hostname = NULL;
      result->port = -1;
      
      if((ret = net_sock_create(hnet, &result->sockhnd, NET_PROTO_TCP)) != NET_OK )
      {
        msg_error(" failed to create a TCP socket  ! net_sock_create %d\n", ret);
      }
      else
      {
       if( (ret = net_sock_setopt(result->sockhnd, "sock_noblocking", NULL, 0)) != NET_OK )
       {
         msg_error(" failed to set the TCP socket noblocking ! net_sock_setopt %d\n", ret);
       }  
      }
      
      if (ret != NET_OK)
      {
        LogError("socket xio create failed");
        free(result);
        result = NULL;
      }
      else
      {    
        result->on_send_complete = NULL;
        result->on_send_complete_callback_context = NULL;
        
        /* Save the connection config */
        result->hostname = malloc(strlen(tls_io_config->hostname) + 1);
      }

      if (result->hostname == NULL)
      {
        free(result);
        result = NULL;
      }
      else
      {
        memcpy(result->hostname, tls_io_config->hostname, strlen(tls_io_config->hostname) + 1);
        result->port = tls_io_config->port;
        result->tlsio_state = TLSIO_STATE_NOT_OPEN;
        g_created = 1;
      }
    }
  }

  return result;
}

void tlsio_mbedtls_STM32Cube_destroy(CONCRETE_IO_HANDLE tls_io)
{
  int ret = 0;
  if (tls_io != NULL)
  {
    TLS_IO_INSTANCE* tls_io_instance = (TLS_IO_INSTANCE*)tls_io;
    net_sockhnd_t socket = tls_io_instance->sockhnd;

    // mbedTLS cleanup...
    mbedtls_ssl_free(&tls_io_instance->ssl);
    mbedtls_ssl_config_free(&tls_io_instance->config);
    mbedtls_x509_crt_free(&tls_io_instance->cacert);
    mbedtls_ctr_drbg_free(&tls_io_instance->ctr_drbg);
    mbedtls_entropy_free(&tls_io_instance->entropy);

    ret = net_sock_close(socket);
    ret |= net_sock_destroy(socket);
    if (ret != NET_OK)
    {
      msg_error("net_sock_close() or net_sock_destroy() failed.\n");
    }
    free(tls_io_instance->hostname);
    free(tls_io);
  }
  g_created = 0;
}

int tlsio_mbedtls_STM32Cube_open(CONCRETE_IO_HANDLE tls_io, ON_IO_OPEN_COMPLETE on_io_open_complete, void* on_io_open_complete_context, ON_BYTES_RECEIVED on_bytes_received, void* on_bytes_received_context, ON_IO_ERROR on_io_error, void* on_io_error_context)
{
  int result = __FAILURE__;

  if (tls_io == NULL)
  {
      LogError("NULL tls_io");
  }
  else
  {
    TLS_IO_INSTANCE* tls_io_instance = (TLS_IO_INSTANCE*)tls_io;

    switch(tls_io_instance->tlsio_state)
    {
      case TLSIO_STATE_ERROR:
        msg_info("Re-opening an IO in error state.\n");
        result = 0;
        break;
      case TLSIO_STATE_NOT_OPEN:
        result = 0;
        break;
      case TLSIO_STATE_OPEN:
      {
        /* Workaround in case the SAS token was about to expire and mqtt_client_disconnect() was called.
         * The driver is not aware that the connection was closed because the netif had not notified it. */
        msg_info("Connection expired. Reconnecting.\n");
        indicate_error(tls_io_instance);
        result = 0;
        break;
      }
      default:
        LogError("IO is not in the expected state: %d\n", tls_io_instance->tlsio_state);
    }
    
    if (result == 0)
    {
      tls_io_instance->tlsio_state = TLSIO_STATE_OPENING_UNDERLYING_IO;
      
      mbedtls_init(tls_io_instance, tls_io_instance->hostname);
      if(0 != mbedtls_x509_crt_parse(&tls_io_instance->cacert, tls_io_instance->cacert_pem, (int)(strlen((char const *)tls_io_instance->cacert_pem) + 1)))
      {
        msg_error("Failed parsing the root CA certificate.\n");
        result = __FAILURE__;
      }
      else
      {
        int ret = NET_OK;
        mbedtls_ssl_conf_ca_chain(&tls_io_instance->config, &tls_io_instance->cacert, NULL);
          
        if ((ret = net_sock_open(tls_io_instance->sockhnd, tls_io_instance->hostname, tls_io_instance->port)) != NET_OK)
        {
          msg_error(" failed TCP connect to %s:%d  ! net_sock_open %d\n", tls_io_instance->hostname, tls_io_instance->port, ret);
          result = __FAILURE__;
        }
        else
        {
          tls_io_instance->on_bytes_received = on_bytes_received;
          tls_io_instance->on_bytes_received_context = on_bytes_received_context;

          tls_io_instance->on_io_open_complete = on_io_open_complete;
          tls_io_instance->on_io_open_complete_context = on_io_open_complete_context;

          tls_io_instance->on_io_error = on_io_error;
          tls_io_instance->on_io_error_context = on_io_error_context;
          
          tls_io_instance->tlsio_state = TLSIO_STATE_IN_HANDSHAKE;

          do {
            ret = mbedtls_ssl_handshake(&tls_io_instance->ssl);
          } while( ret == MBEDTLS_ERR_SSL_WANT_READ || ret == MBEDTLS_ERR_SSL_WANT_WRITE );

          if (ret != 0)
          {
            msg_error(" failed TLS connect to %s:%d  ! mbedtls_connect %d\n", tls_io_instance->hostname, tls_io_instance->port, ret);
            result = __FAILURE__;
          }
          else
          {
            tls_io_instance->tlsio_state = TLSIO_STATE_OPEN;
          }
        }        
      }
    }
    
    if (result == 0)
    {
      indicate_open_complete(tls_io_instance, IO_OPEN_OK);
    }
    else
    {
      tls_io_instance->tlsio_state = TLSIO_STATE_ERROR;
      indicate_open_complete(tls_io_instance, IO_OPEN_ERROR);
    }
  }
  
  return result;
}

int tlsio_mbedtls_STM32Cube_close(CONCRETE_IO_HANDLE tls_io, ON_IO_CLOSE_COMPLETE on_io_close_complete, void* callback_context)
{
  int result = 0;

  if (tls_io == NULL)
  {
    result = __FAILURE__;
  }
  else
  {
    TLS_IO_INSTANCE* tls_io_instance = (TLS_IO_INSTANCE*)tls_io;

    if ((tls_io_instance->tlsio_state == TLSIO_STATE_NOT_OPEN) ||
        (tls_io_instance->tlsio_state == TLSIO_STATE_CLOSING))
    {
      result = __FAILURE__;
    }
    else
    {
      tls_io_instance->tlsio_state = TLSIO_STATE_CLOSING;
      int ret = 0;
      do
      {
        ret = mbedtls_ssl_close_notify(&tls_io_instance->ssl);
      }
      while ( (ret == MBEDTLS_ERR_SSL_WANT_WRITE) || (ret == MBEDTLS_ERR_SSL_WANT_READ) );
      
      if (net_sock_close(tls_io_instance->sockhnd) != NET_OK)
      {
        msg_error("net_sock_close() failed.\n");
        tls_io_instance->tlsio_state = TLSIO_STATE_ERROR;
      }
      else
      {
        mbedtls_x509_crt_free(&tls_io_instance->cacert);
        mbedtls_ssl_free(&tls_io_instance->ssl);
        mbedtls_ssl_config_free(&tls_io_instance->config);
        mbedtls_ctr_drbg_free(&tls_io_instance->ctr_drbg);
        mbedtls_entropy_free(&tls_io_instance->entropy);
        mbedtls_ssl_session_free(&tls_io_instance->ssn);

        tls_io_instance->on_io_close_complete = on_io_close_complete;
        tls_io_instance->on_io_close_complete_context = callback_context;

        if (tls_io_instance->on_io_close_complete != NULL)
        {
          tls_io_instance->on_io_close_complete(tls_io_instance->on_io_close_complete_context);
        }
        if (g_created)
        {
          tls_io_instance->tlsio_state = TLSIO_STATE_NOT_OPEN;
        }
        
        result = 0;
      }
    }
  }

  return result;
}

int tlsio_mbedtls_STM32Cube_send(CONCRETE_IO_HANDLE tls_io, const void* buffer, size_t size, ON_SEND_COMPLETE on_send_complete, void* callback_context)
{
  int result;

  if (tls_io == NULL)
  {
    result = __FAILURE__;
  }
  else
  {
    TLS_IO_INSTANCE* tls_io_instance = (TLS_IO_INSTANCE*)tls_io;

    if (tls_io_instance->tlsio_state != TLSIO_STATE_OPEN)
    {
      result = __FAILURE__;
    }
    else
    {
      tls_io_instance->on_send_complete = on_send_complete;
      tls_io_instance->on_send_complete_callback_context = callback_context;

      int res = mbedtls_ssl_write(&tls_io_instance->ssl, buffer, size);
      if (res != size)
      {
        tls_io_instance->tlsio_state = TLSIO_STATE_ERROR;
        indicate_error(tls_io_instance);
        result = __FAILURE__;
      }
      else
      {
        result = 0;
      }
    }
  }

  return result;
}

void tlsio_mbedtls_STM32Cube_dowork(CONCRETE_IO_HANDLE tls_io)
{
  if (tls_io != NULL)
  {
    TLS_IO_INSTANCE* tls_io_instance = (TLS_IO_INSTANCE*)tls_io;

    if ((tls_io_instance->tlsio_state != TLSIO_STATE_NOT_OPEN) &&
        (tls_io_instance->tlsio_state != TLSIO_STATE_ERROR))
    {
      decode_ssl_received_bytes(tls_io_instance);
      /* Note: The Wifi LL is not threaded, and there is no "posted work".
       * The receive low level processing is done in the IT handler and in
       * the mbedTLS receive wrapper.
       */
      //xio_dowork(tls_io_instance->socket_io);
    }
  }
}

const IO_INTERFACE_DESCRIPTION* tlsio_mbedtls_STM32Cube_get_interface_description(void)
{
  return &tlsio_mbedtls_interface_description;
}

int tlsio_mbedtls_STM32Cube_setoption(CONCRETE_IO_HANDLE tls_io, const char* optionName, const void* value)
{
  int result = 0;
  mbedtls_x509_crt *crt;

  if (tls_io == NULL || optionName == NULL)
  {
    result = __FAILURE__;
  }
  else
  {
    TLS_IO_INSTANCE* tls_io_instance = (TLS_IO_INSTANCE*)tls_io;

    if (strcmp("TrustedCerts", optionName) == 0)
    {

      crt = malloc(sizeof(mbedtls_x509_crt));

      mbedtls_x509_crt_init(crt);

      result = mbedtls_x509_crt_parse(crt,(const unsigned char *)value,(int)(strlen(value)+1));
      if( result != 0 )
      {
        result = __FAILURE__;
      }
      else
      {
        tls_io_instance->cacert_pem = (const unsigned char *)value;
      }

      mbedtls_x509_crt_free(crt);

      free(crt);

    }
    else
    {
      /* Note: Setting the socket timeout could be considered, but its implementation would 
       * conflict with the mbedTLS wrapper read timeout when the blocking interface is use.
       */
        //result = xio_setoption(tls_io_instance->socket_io, optionName, value);
      msg_error("Unsupported option setting.\n");
    }
  }

  return result;
}

#endif // USE_MBED_TLS
