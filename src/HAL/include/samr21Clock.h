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
#include "samr21NopDelay.h"
#include "samr21Rtc.h"
#include "samr21Timer.h"
#include "samr21Usb.h"
#include "samr21Trx.h"


/**
 * Inits the Clocking System of the SAMR21
 * Activates a Clock Source GCLKIN derived from the Crystal on the AT86rf233 (MCLK 1MHz)
 * 
 * 
 * Sets up GCLK0 (CPU Clock) to use the 48MHz DFLL
 * Sets up GCLK1 to sourced from GCLKIN  (1Mhz MCLK stepped down to 32.250 kHz)
 * Sets up GCLK3 to 1MHz stepped down from the 48MHz DFLL (For Timer and RTC)
 * Sets up GCLK4 to 8MHz stepped down from the 48MHz DFLL (For SPI-Communication dit AT86RF233)
 * 
 */
void samr21Clock_init(void);

#endif //_SAMR21_CLOCK_H_