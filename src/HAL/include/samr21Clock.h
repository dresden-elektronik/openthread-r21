/*
 * Copyright (c) 2023 dresden elektronik ingenieurtechnik gmbh.
 * All rights reserved.
 *
 * The software in this package is published under the terms of the BSD
 * style license a copy of which has been included with this distribution in
 * the LICENSE.txt file.
 *
 */

#ifndef _SAMR21_CLOCK_H_
#define _SAMR21_CLOCK_H_

#include "samr21.h"
#include <stdbool.h>
#include <stdint.h>


/**
 * Inits the Clocking Tree to a minimum state where all peripheral are disabled.
 * Only the SERCOM4 Module gets feed by a internal 1MHz Oscillator Clock.
 * This is done so a basic Communication with the AT86RF233 is possible.
 * 
 * CPU  = 1MHz 
 * USB  = disabled
 * EIC  = disabled
 * Timer and  RTC = disabled
 * UART = disabled
 * TRX-SPI = 1MHz
 */
void samr21Clock_enableFallbackClockTree(void);

/**
 * Inits the DFLL to output 48MHz.
 * Uses the MCLK from the AT86RF233 as a Reference for Accuracy.
 * Before calling this Function the AT86RF233 must be setup to output 1MHz on the MCLK Pin.
 * 
 * Alternatively the USB-BUS SOF Signal can be used as the Reference (#ifdef SAMR21_USE_USB_CLOCK).
 * 
 * 
 * CPU  = 48MHz 
 * USB  = 48MHz 
 * EIC  = 48MHz 
 * Timer and  RTC = 1MHz 
 * UART = 1MHz 
 * TRX-SPI = 12MHz
 */
void samr21Clock_enableOperatingClockTree(void);
#endif //_SAMR21_CLOCK_H_