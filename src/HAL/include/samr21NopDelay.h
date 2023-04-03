/*
 * Copyright (c) 2023 dresden elektronik ingenieurtechnik gmbh.
 * All rights reserved.
 *
 * The software in this package is published under the terms of the BSD
 * style license a copy of which has been included with this distribution in
 * the LICENSE.txt file.
 *
 */

#ifndef _SAMR21_NOP_DELAY_H_
#define _SAMR21_NOP_DELAY_H_

#include "samr21.h"

/**
 * Sets the SysTick Counter to block Operation till the specified amount of CPU Cycles has passed
 * 
 * @param[in] delayCycles    amount of CPU Cycles the function should block operation
 */
void samr21delaySysTick(uint32_t delayCycles);

#endif // _SAMR21_NOP_DELAY_H_