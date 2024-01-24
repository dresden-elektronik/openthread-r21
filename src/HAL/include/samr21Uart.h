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
void samr21Uart_send(uint8_t data);


#endif //_SAMR21_UART_H_
