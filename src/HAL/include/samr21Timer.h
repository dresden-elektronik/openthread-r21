/*
 * Copyright (c) 2023 dresden elektronik ingenieurtechnik gmbh.
 * All rights reserved.
 *
 * The software in this package is published under the terms of the BSD
 * style license a copy of which has been included with this distribution in
 * the LICENSE.txt file.
 *
 */

#ifndef _SAMR21_TIMER_H_
#define _SAMR21_TIMER_H_

#include <stdint.h>
#include <stdbool.h>

#include "samr21.h"
#include "samr21NopDelay.h"

/**
 * Inits Timer TCC0 as a oneshot Timer with a 1MHz Clk
 * 
 * @param[in] divider divides down the SourceClk by 2^Value (1Mhz / 2^divider)
 */
void samr21Timer0Init(uint8_t divider);
/**
 * Sets a oneshot timer on TCC0 (24 Bit)
 * 
 * @param[in] timerTicks amount of TimerTicks until it the Timer fires
 */
void samr21Timer0Set(uint32_t timerTicks);
/**
 * Aborts the currently running Oneshot Timer on TCC0
 * 
 */
void samr21Timer0Stop();


/**
 * Inits Timer TCC1 as a oneshot Timer with a 1MHz Clk
 * 
 * @param[in] divider divides down the SourceClk by 2^Value (1Mhz / 2^divider)
 */
void samr21Timer1Init(uint8_t divider);
/**
 * Sets a oneshot timer on TCC1 (24 Bit)
 * 
 * @param[in] timerTicks amount of TimerTicks until it the Timer fires
 */
void samr21Timer1Set(uint32_t timerTicks);
/**
 * Aborts the currently running Oneshot Timer on TCC1
 * 
 */
void samr21Timer1Stop();


/**
 * Inits Timer TCC2 as a oneshot Timer with a 1MHz Clk
 * 
 * @param[in] divider divides down the SourceClk by 2^Value (1Mhz / 2^divider)
 */
void samr21Timer2Init(uint8_t divider);
/**
 * Sets a oneshot timer on TCC2 (16 Bit)
 * 
 * @param[in] timerTicks amount of TimerTicks until it the Timer fires
 */
void samr21Timer2Set(uint16_t timerTicks);
/**
 * Aborts the currently running Oneshot Timer on TCC2
 * 
 */
void samr21Timer2Stop();


/**
 * Inits Timer TC3 as a oneshot Timer with a 1MHz Clk
 * 
 * @param[in] divider divides down the SourceClk by 2^Value (1Mhz / 2^divider)
 */
void samr21Timer3Init(uint8_t divider);
/**
 * Sets a oneshot timer on TC3 (16 Bit)
 * 
 * @param[in] timerTicks amount of TimerTicks until it the Timer fires
 */
void samr21Timer3Set(uint16_t timerTicks);
/**
 * Aborts the currently running Oneshot Timer on TC3
 * 
 */
void samr21Timer3Stop();


/**
 * Inits Timer TC4 as a oneshot Timer with a 1MHz Clk
 * 
 * @param[in] divider divides down the SourceClk by 2^Value (1Mhz / 2^divider)
 */
void samr21Timer4Init(uint8_t divider);
/**
 * Sets a oneshot timer on TC4 (16 Bit)
 * 
 * @param[in] timerTicks amount of TimerTicks until it the Timer fires
 */
void samr21Timer4Set(uint16_t timerTicks);
/**
 * Aborts the currently running Oneshot Timer on TC4
 * 
 */
void samr21Timer4Stop();

/**
 * Inits Timer TC5 as a oneshot Timer with a 1MHz Clk
 * 
 * @param[in] divider divides down the SourceClk by 2^Value (1Mhz / 2^divider)
 */
void samr21Timer5Init(uint8_t divider);
/**
 * Sets a oneshot timer on TC5 (16 Bit)
 * 
 * @param[in] timerTicks amount of TimerTicks until it the Timer fires
 */
void samr21Timer5Set(uint16_t timerTicks);
/**
 * Aborts the currently running Oneshot Timer on TC5
 * 
 */
void samr21Timer5Stop();


/**
 * Disables all Timers
 * Used for SoftReset
 * 
 */
void samr21TimerDeinitAll();
#endif //_SAMR21_TIMER_H_