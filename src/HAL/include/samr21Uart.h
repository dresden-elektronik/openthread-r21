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


void samr21UartInit();

void samr21UartDeinit();

void samr21UartSend(uint8_t data);


#endif //_SAMR21_UART_H_
