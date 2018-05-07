/* Copyright (c) 2015 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * @section DESCRIPTION
 *
 * Parser for the AT command syntax
 *
 */


#ifndef __AT_PARSER_H__
#define __AT_PARSER_H__

#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>


/********** At PARSER *******************/
#define ONESEC									1000
#define SIZE_PARSER_BUFFER			1024
#define DEFAULT_TIMEOUT					90*ONESEC

extern uint8_t  AT_Parser_Buffer[SIZE_PARSER_BUFFER];
extern volatile uint32_t parser_write_ix;
extern volatile uint32_t parser_read_ix;

typedef struct{
	volatile	uint8_t		index;
	volatile	bool			enable;
	volatile 	uint32_t	time;
	volatile 	uint32_t	_timeout;
}Timer;

typedef struct{
	int						(*send) (uint8_t);
	void			*recv;
	uint32_t			(*timer_read_ms) (void);
	void			*udef;
}AT_parser_call;

#if 0

/**
* Parser for parsing AT commands
*
* Here are some examples:
* @code
* ATParser at = ATParser(serial, "\r\n");
* int value;
* char buffer[100];
*
* at.send("AT") && at.recv("OK");
* at.send("AT+CWMODE=%d", 3) && at.recv("OK");
* at.send("AT+CWMODE?") && at.recv("+CWMODE:%d\r\nOK", &value);
* at.recv("+IPD,%d:", &value);
* at.read(buffer, value);
* at.recv("OK");
* @endcode
*/

    /**
    * Allows timeout to be changed between commands
    *
    * @param timeout timeout of the connection
    */
    void ATParser_setTimeout(int timeout) {
        _timeout = timeout;
    }

    int ATParser_getTimeout(void) {
        return _timeout;
    }

    /**
    * Sets string of characters to use as line delimiters
    *
    * @param delimiter string of characters to use as line delimiters
    */
    void ATParser_setDelimiter(const char *delimiter) {
        _delimiter = delimiter;
        _delim_size = strlen(delimiter);
    }
    
    /**
    * Allows echo to be on or off
    *
    * @param echo 1 for echo and 0 turns it off
    */
    void ATParser_debugOn(uint8_t on) {
        dbg_on = (on) ? 1 : 0;
    }

    /**
    * Sends an AT command
    *
    * Sends a formatted command using printf style formatting
    * @see ::printf
    *
    * @param command printf-like format string of command to send which
    *                is appended with the specified delimiter
    * @param ... all printf-like arguments to insert into command
    * @return true only if command is successfully sent
    */
    bool ATParser_send(const char *command, ...);
    bool ATParser_vsend(const char *command, va_list args);

    /**
    * Recieve an AT response
    *
    * Recieves a formatted response using scanf style formatting
    * @see ::scanf
    *
    * Responses are parsed line at a time using the specified delimiter.
    * Any recieved data that does not match the response is ignored until
    * a timeout occurs.
    *
    * @param response scanf-like format string of response to expect
    * @param ... all scanf-like arguments to extract from response
    * @return true only if response is successfully matched
    */
    bool ATParser_recv(const char *response, ...);
    bool ATParser_vrecv(const char *response, va_list args);

    /**
    * Write a single byte to the underlying stream
    *
    * @param c The byte to write
    * @return The byte that was written or -1 during a timeout
    */
    int ATParser_putc(char c);

    /**
    * Get a single byte from the underlying stream
    *
    * @return The byte that was read or -1 during a timeout
    */
    int ATParser_getc();

    /**
    * Write an array of bytes to the underlying stream
    *
    * @param data the array of bytes to write
    * @param size number of bytes to write
    * @return number of bytes written or -1 on failure
    */
    int ATParser_write(const char *data, int size);

    /**
    * Read an array of bytes from the underlying stream
    *
    * @param data the destination for the read bytes
    * @param size number of bytes to read
    * @return number of bytes read or -1 on failure
    */
    int ATParser_read(char *data, int size);

    /**
    * Direct printf to underlying stream
    * @see ::printf
    *
    * @param format format string to pass to printf
    * @param ... arguments to printf
    * @return number of bytes written or -1 on failure
    */
    int ATParser_printf(const char *format, ...);
    int ATParser_vprintf(const char *format, va_list args);

    /**
    * Direct scanf on underlying stream
    * @see ::scanf
    *
    * @param format format string to pass to scanf
    * @param ... arguments to scanf
    * @return number of bytes read or -1 on failure
    */
    int ATParser_scanf(const char *format, ...);
    int ATParser_vscanf(const char *format, va_list args);

    /**
    * Flushes the underlying stream
    */
    void ATParser_flush();
		
		void AT_parser_init(void);

#endif

#endif
