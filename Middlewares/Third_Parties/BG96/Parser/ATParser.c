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
 
#include "stm32f4xx_hal.h" 

#include "ATParser.h"
#include "stm32_BG96_iot.h"

const char* 			_delimiter = {"\r\n"};
uint8_t					 	_delim_size = 2;


#define _buffer_size		256

AT_parser_call	AT_call;

uint8_t AT_Parser_Buffer[SIZE_PARSER_BUFFER];
uint8_t _buffer[_buffer_size];
volatile uint32_t parser_write_ix = 0;
volatile uint32_t parser_read_ix = 0;

static int timer_init(Timer *timer, uint32_t tout)
{
	timer->index = 0;
	timer->enable = false;
	timer->time = tout;
	timer->_timeout = 0;
	return 1;
}
	
static int timer_start(Timer *timer)
{	
	//assert (AT_call.timer_read_ms);
	uint32_t now_time = AT_call.timer_read_ms();
	if (timer->time == 0)
		timer->_timeout = (now_time + DEFAULT_TIMEOUT);
	else
		timer->_timeout = (now_time + timer->time);
	
	timer->enable = true;
	return 1;
}

static int timer_run(Timer *timer)
{
		//assert (AT_call.timer_read_ms);
		volatile uint32_t now_time = AT_call.timer_read_ms();
		if (timer->enable == false)
			return -1;
		else
		{
			if (timer->_timeout < now_time)
				return -1;
			return 0;
		}
}
		
static int timer_stop(Timer *timer)
{
	timer->enable = false;
	return 1;
}


 /**
* @brief  Receive interface, call from HW reception driver
*         
* @param  uint8 character received from device
* @retval None
*/
void _parser_char_recv(uint8_t ch)
{
	AT_Parser_Buffer[parser_write_ix] = ch;
	parser_write_ix = (parser_write_ix +1)%SIZE_PARSER_BUFFER;
}


void AT_parser_init(void)
{
		AT_call.recv = 0;
		AT_call.send = BG96_char_send;
		AT_call.timer_read_ms = HAL_GetTick;
		AT_call.udef = 0;
												HAL_NVIC_DisableIRQ(USARTbg_IRQn);
		parser_read_ix = parser_write_ix;
											HAL_NVIC_EnableIRQ(USARTbg_IRQn);
}



// getc/putc handling with timeouts
int ATParser_putc(char c)
{
	return AT_call.send(c);
	/*

    Timer timer;
    timer_start(timer);

    while (true) {
        if (_serial->writeable()) {
            return putc(c);
        }
        if (timer_run(timer) < 0) {
            return -1;
        }
    }
	*/
}

int ATParser_getc()
{
    static Timer timer;
		uint8_t	ch;
		//timer_init(&timer, 5);
    //timer_start(&timer);
		uint32_t local_write_ix;
	
		HAL_NVIC_DisableIRQ(USARTbg_IRQn);
		local_write_ix = parser_write_ix;
		HAL_NVIC_EnableIRQ(USARTbg_IRQn);
    while (true) 
		{
        if (parser_read_ix != parser_write_ix) 
					{
            ch = AT_Parser_Buffer[parser_read_ix];
						parser_read_ix = (parser_read_ix + 1)%SIZE_PARSER_BUFFER;
						return ch;
					}
					/*
        if (timer_run(&timer) < 0) 
					{
            return -1;
					}
					*/
    }
		return -1;
}

#if 0
void ATParser_flush()
{
    while (_serial->readable()) {
        _serial->getc();
    }
}
#endif

// read/write handling with timeouts
int ATParser_write(const char *data, int size)
{
    int i = 0;
    for ( ; i < size; i++) {
        if (ATParser_putc(data[i]) < 0) {
            return -1;
        }
    }
    return i;
}

int ATParser_read(char *data, int size)
{
    int i = 0;
    for ( ; i < size; i++) {
        int c = ATParser_getc();
        if (c < 0) {
            return -1;
        }
        data[i] = c;
    }
    return i;
}

#if 0
// printf/scanf handling
int ATParser_vprintf(const char *format, va_list args)
{
    if (vsprintf(_buffer, format, args) < 0) {
        return false;
    }
    int i = 0;
    for ( ; _buffer[i]; i++) {
        if (AT_putc(_buffer[i]) < 0) {
            return -1;
        }
    }
    return i;
}
#endif

int ATParser_vscanf(const char *format, va_list args)
{
    // Since format is const, we need to copy it into our buffer to
    // add the line's null terminator and clobber value-matches with asterisks.
    //
    // We just use the beginning of the buffer to avoid unnecessary allocations.
    int i = 0;
    int offset = 0;

    while (format[i]) {
        if (format[i] == '%' && format[i+1] != '%' && format[i+1] != '*') {
            _buffer[offset++] = '%';
            _buffer[offset++] = '*';
            i++;
        } else {
            _buffer[offset++] = format[i++];
        }
    }

    // Scanf has very poor support for catching errors
    // fortunately, we can abuse the %n specifier to determine
    // if the entire string was matched.
    _buffer[offset++] = '%';
    _buffer[offset++] = 'n';
    _buffer[offset++] = 0;

    // To workaround scanf's lack of error reporting, we actually
    // make two passes. One checks the validity with the modified
    // format string that only stores the matched characters (%n).
    // The other reads in the actual matched values.
    //
    // We keep trying the match until we succeed or some other error
    // derails us.
    int j = 0;

    while (true) {
        // Ran out of space
        if (j+1 >= _buffer_size - offset) {
            return false;
        }
        // Recieve next character
        int c = ATParser_getc();
        if (c < 0) {
            return -1;
        }
        _buffer[offset + j++] = c;
        _buffer[offset + j] = 0;

        // Check for match
        int count = -1;
        sscanf(_buffer+offset, _buffer, &count);

        // We only succeed if all characters in the response are matched
        if (count == j) {
            // Store the found results
            vsscanf(_buffer+offset, format, args);
            return j;
        }
    }
}


// Command parsing with line handling
bool ATParser_vsend(const char *command, va_list args)
{
    // Create and send command
    if (vsprintf(_buffer, command, args) < 0) {
        return false;
    }
    for (int i = 0; _buffer[i]; i++) {
        if (ATParser_putc(_buffer[i]) < 0) {
            return false;
        }
    }

    // Finish with CR "\r"
    //for (int i = 0; _delimiter[i]; i++) {
        if (ATParser_putc(_delimiter[0]) < 0) {
            return false;
        }
    //}

    //debug_if(dbg_on, "AT> %s\r\n", _buffer);
    return true;
}

bool ATParser_vrecv(const char *response, va_list args)
{
    bool delim_recv = false;
    // Iterate through each line in the expected response
    while (response[0]) {
        // Since response is const, we need to copy it into our buffer to
        // add the line's null terminator and clobber value-matches with asterisks.
        //
        // We just use the beginning of the buffer to avoid unnecessary allocations.
        int i = 0;
        int offset = 0;

        while (response[i]) {
            if (memcmp(&response[i+1-_delim_size], _delimiter, _delim_size) == 0) {
                i++;
                break;
            } else if (response[i] == '%' && response[i+1] != '%' && response[i+1] != '*') {
                _buffer[offset++] = '%';
                _buffer[offset++] = '*';
                i++;
            } else {
                _buffer[offset++] = response[i++];
            }
        }

        // Scanf has very poor support for catching errors
        // fortunately, we can abuse the %n specifier to determine
        // if the entire string was matched.
        _buffer[offset++] = '%';
        _buffer[offset++] = 'n';
        _buffer[offset++] = 0;

        // To workaround scanf's lack of error reporting, we actually
        // make two passes. One checks the validity with the modified
        // format string that only stores the matched characters (%n).
        // The other reads in the actual matched values.
        //
        // We keep trying the match until we succeed or some other error
        // derails us.
        int j = 0;
        
        while (true) {
            // Receive next character
            int c = ATParser_getc();
            if (c < 0) {
                return false;
            }
            _buffer[offset + j++] = c;
            _buffer[offset + j] = 0;

            // Check for match
            int count = -1;
            sscanf(_buffer+offset, _buffer, &count);
            
            // We only succeed if all characters in the response are matched
            if(count==j) {
                //check if last input is a number
                if (_buffer[offset-6]=='%' && _buffer[offset-5]=='*' 
                    && (_buffer[offset-4]=='x' || _buffer[offset-4]=='d' || _buffer[offset-4]=='u'))
                {
                    //if the last char is a number, keep getting the next character till CR
                    while(true)
                    {
                        int c = ATParser_getc();
                        if (c < 0) {
                            return false;
                        }
                        if(c==0xD) {//there is no next number so exit from condition   
                            c = ATParser_getc();//get rid of the following '\n' delimiter
                            delim_recv = true;
                            break;
                        }
                        else {
                            _buffer[offset + j++] = c;
                            _buffer[offset + j] = 0;
                        }
                    }
                }
                    
                //debug_if(dbg_on, "AT= %s\r\n", _buffer+offset);
                // Reuse the front end of the buffer
                memcpy(_buffer, response, i);
                _buffer[i] = 0;

                // Store the found results
                vsscanf(_buffer+offset, _buffer, args);

                // Jump to next line and continue parsing
                response += i;
                
                // receive trailing delimiters
                 for(int i=0; !delim_recv && i<_delim_size; i++) {
                    c = ATParser_getc();
                    if(c < 0)
                        break;
                }
                break;
            }

            // Clear the buffer when we hit a newline or ran out of space
            // running out of space usually means we ran into binary data
            if (j+1 >= _buffer_size - offset ||
                strcmp(&_buffer[offset + j-_delim_size], _delimiter) == 0) {

                //debug_if(dbg_on, "AT< %s", _buffer+offset);
                j = 0;
            }
        }
    }

    return true;
}

#if 0
// Mapping to vararg functions
int ATParser_printf(const char *format, ...)
{
    va_list args;
    va_start(args, format);
    int res = ATParser_vprintf(format, args);
    va_end(args);
    return res;
}
#endif

int ATParser_scanf(const char *format, ...)
{
    va_list args;
    va_start(args, format);
    int res = ATParser_vscanf(format, args);
    va_end(args);
    return res;
}

bool ATParser_send(const char *command, ...)
{
    va_list args;
    va_start(args, command);
    bool res = ATParser_vsend(command, args);
    va_end(args);
    return res;
}

bool ATParser_recv(const char *response, ...)
{
    va_list args;
    va_start(args, response);
		memset(_buffer, 0, sizeof(_buffer));
    bool res = ATParser_vrecv(response, args);
    va_end(args);
    return res;
}



