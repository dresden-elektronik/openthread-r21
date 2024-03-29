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
#include "samr21SysTick.h"

/**
 * Inits Timer TCC0 as a oneshot or periodic Timer with a 1MHz Clk
 * 
 * @param[in] divider divides down the SourceClk by 2^Value (1Mhz / 2^divider)
 * @param[in] oneshot if true the timer stops after it triggers once, otherwise it will wrap around
 * @param[in] interrupt if true the TCC0_Handler() gets called once the timer fires
 * 
 */
void samr21Timer0_init(uint8_t divider, bool oneshot, bool interrupt);
/**
 * Sets a oneshot timer on TCC0 (24 Bit)
 * 
 * @param[in] timerTicks amount of TimerTicks until it the Timer fires
 */
void samr21Timer0_set(uint32_t timerTicks);
/**
 * Aborts the currently running Oneshot Timer on TCC0
 * 
 */
void samr21Timer0_stop();


/**
 * Inits Timer TCC1 as a oneshot or periodic Timer with a 1MHz Clk
 * 
 * @param[in] divider divides down the SourceClk by 2^Value (1Mhz / 2^divider)
 * @param[in] oneshot if true the timer stops after it triggers once, otherwise it will wrap around
 * @param[in] interrupt if true the TCC1_Handler() gets called once the timer fires
 */
void samr21Timer1_init(uint8_t divider, bool oneshot, bool interrupt);
/**
 * Sets a oneshot timer on TCC1 (24 Bit)
 * 
 * @param[in] timerTicks amount of TimerTicks until it the Timer fires
 */
void samr21Timer1_startOneshot(uint32_t timerTicks);
/**
 * Aborts the currently running Oneshot Timer on TCC1
 * 
 */
void samr21Timer1_stop();


/**
 * Inits Timer TCC2 as a oneshot or periodic Timer with a 1MHz Clk
 * 
 * @param[in] divider divides down the SourceClk by 2^Value (1Mhz / 2^divider)
 * @param[in] oneshot if true the timer stops after it triggers once, otherwise it will wrap around
 * @param[in] interrupt if true the TCC2_Handler() gets called once the timer fires
 */
void samr21Timer2_init(uint8_t divider, bool oneshot, bool interrupt);
/**
 * Sets a oneshot timer on TCC2 (16 Bit)
 * 
 * @param[in] timerTicks amount of TimerTicks until it the Timer fires
 */
void samr21Timer2_startOneshot(uint16_t timerTicks);
/**
 * Aborts the currently running Oneshot Timer on TCC2
 * 
 */
void samr21Timer2_stop();


/**
 * Inits Timer TC3 as a oneshot or periodic Timer with a 1MHz Clk
 * 
 * @param[in] divider divides down the SourceClk by 2^Value (1Mhz / 2^divider)
 * @param[in] oneshot if true the timer stops after it triggers once, otherwise it will wrap around
 * @param[in] interrupt if true the TC3_Handler() gets called once the timer fires
 */
void samr21Timer3_init(uint8_t divider, bool oneshot, bool interrupt);

/**
 * Sets a periodic timer on TC3 (16 Bit)
 * 
 * @param[in] timerTicks amount of TimerTicks until it the Timer fires
 */
void samr21Timer3_setContinuousPeriod(uint16_t timerTicks);

/**
 * Sets a oneshot timer on TC3 (16 Bit)
 * 
 * @param[in] timerTicks amount of TimerTicks until it the Timer fires
 */
void samr21Timer3_startOneshot(uint16_t timerTicks);

/**
 * Aborts the currently running Oneshot Timer on TC3
 * 
 */
void samr21Timer3_Stop();


/**
 * Inits Timer TC4 as a oneshot or periodic Timer with a 1MHz Clk
 * 
 * @param[in] divider divides down the SourceClk by 2^Value (1Mhz / 2^divider)
 * @param[in] oneshot if true the timer stops after it triggers once, otherwise it will wrap around
 * @param[in] interrupt if true the TC4_Handler() gets called once the timer fires
 */
void samr21Timer4_init(uint8_t divider, bool oneshot, bool interrupt);
/**
 * Sets a oneshot timer on TC4 (16 Bit)
 * 
 * @param[in] timerTicks amount of TimerTicks until it the Timer fires
 */
void samr21Timer4_startOneshot(uint16_t timerTicks);
/**
 * Aborts the currently running Oneshot Timer on TC4
 * 
 */
void samr21Timer4_stop();

/**
 * Inits Timer TC5 as a oneshot or periodic Timer with a 1MHz Clk
 * 
 * @param[in] divider divides down the SourceClk by 2^Value (1Mhz / 2^divider)
 * @param[in] oneshot if true the timer stops after it triggers once, otherwise it will wrap around
 * @param[in] interrupt if true the TC5_Handler() gets called once the timer fires
 */
void samr21Timer5_init(uint8_t divider, bool oneshot, bool interrupt);
/**
 * Sets a oneshot timer on TC5 (16 Bit)
 * 
 * @param[in] timerTicks amount of TimerTicks until it the Timer fires
 */
void samr21Timer5_startOneshot(uint16_t timerTicks);
/**
 * Aborts the currently running Oneshot Timer on TC5
 * 
 */
void samr21Timer5_stop();


/**
 * Disables all Timers
 * Used for SoftReset
 * 
 */
void samr21Timer_deinitAll();
#endif //_SAMR21_TIMER_H_