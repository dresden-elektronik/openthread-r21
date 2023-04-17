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
 * Deinitialize all Clocks Derived from the Crystal on the AT86rf233 (MCLK)
*/
void samr21ClockRemoveExternalSource();

/**
 * Inits the Clocking System of the SAMR21
 * Activates a Clock Source GCLKIN derived from the Crystal on the AT86rf233 (MCLK)
 * 
 * This function should be called BEFORE the mClk on the AT86RF233 ist changed to output 16MHz 
 * 
 * Sets up CLKGEN0 (CPU Clock) to use the internal OSC8M while clocks are being setup
 * 
 * Sets up CLKGEN1 to sourced from GCLKIN  (1Mhz or 16 MHz after Trx Setup)
 * Sets up CLKGEN2 to be mClk devided by 16 (for a stable 1us Clock after the AT86r233 is setup to output 16MHz)
 * 
 * CLKGEN0 (temp. 8Mhz) is temporally used to provide a CPU-Clk
 * CLKGEN1 (1 or 16Mhz) is used by SERCOM4 for Synchronous SPI Communication with the at86rf233
 * CLKGEN2 (1/16 or 1Mhz)is used for all Timers including the RTC
 */
void samr21ClockTrxSrcInit();

/**
 * Inits the Clocks derived from a the 16Mhz mClk of the at86rf233
 * 
 * This function should be called AFTER the mClk on the AT86RF233 was changed to output 16MHz 
 * 
 * Sets up CLKGEN3 to output 31.250kHz derived from the 16MHz MClk
 * Sets up the DFLL to output 48MHz and use CLKGEN3 as a reference  
 * Sets up CLKGEN0 (CPU Clock) to use the DFLL (48MHz)
 * 
 * CLKGEN0 (48MHz) is used for the CPU, USB and EIC-Detect Clock
 */
void samr21ClockInitAfterTrxSetup();

#endif //_SAMR21_CLOCK_H_