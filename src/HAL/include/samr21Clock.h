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

void samr21ClockTrxSrcInit();

void samr21ClockInitAfterTrxSetup();

#endif //_SAMR21_CLOCK_H_