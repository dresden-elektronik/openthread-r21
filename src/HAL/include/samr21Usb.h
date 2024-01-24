/*
 * Copyright (c) 2023 dresden elektronik ingenieurtechnik gmbh.
 * All rights reserved.
 *
 * The software in this package is published under the terms of the BSD
 * style license a copy of which has been included with this distribution in
 * the LICENSE.txt file.
 *
 */


#ifndef _SAMR21_USB_H_
#define _SAMR21_USB_H_

#include "samr21.h"



/**
 * Inits the Ports and Clocks for the TinyUSB Driver
*/
void samr21Usb_init();

/**
 * Deinit Clock for USB Module
*/
void samr21Usb_deinit();

#endif //_SAMR21_USB_H_