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

void samr21delaySysTick(uint32_t delayCycles);

void samr21delayLoop(uint32_t delayCycles);

#endif // _SAMR21_NOP_DELAY_H_