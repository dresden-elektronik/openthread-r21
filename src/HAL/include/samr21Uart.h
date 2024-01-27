/*
 * Copyright (c) 2023 dresden elektronik ingenieurtechnik gmbh.
 * All rights reserved.
 *
 * The software in this package is published under the terms of the BSD
 * style license a copy of which has been included with this distribution in
 * the LICENSE.txt file.
 *
 */


#ifndef _SAMR21_UART_H_
#define _SAMR21_UART_H_

#include "samr21.h"
#include "samr21Uart.h"

#ifndef UART_BUFFER_SIZE
    #define UART_BUFFER_SIZE 128
#endif

#ifndef NUM_UART_BUFFER 
    #define NUM_UART_BUFFER 3
#endif

typedef struct uartBuffer_s
{
    uint8_t data[UART_BUFFER_SIZE];
    int16_t length;
}uartBuffer_t;

/**
 * Inits the SERCOM2 and corrsponding Pins of the SAMR21 as a UART Debug Output
 */
void samr21Uart_init();

/**
 * Deinits the SERCOM2 of the SAMR21
 */
void samr21Uart_deinit();


/**
 * Sends a Single Character of data via the DEBUG-UART (SERCOM2)
 * 
 * @param[in] data Character to be vie Debug UART
 */
void samr21Uart_sendByte(uint8_t data);


uartBuffer_t * samr21Uart_allocTransmitBuffer(void);
uint16_t samr21Uart_write(uint8_t * data, uint16_t length);
void samr21Uart_checkForPendingTransmit(void);

#endif //_SAMR21_UART_H_
