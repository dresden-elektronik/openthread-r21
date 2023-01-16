/*
 * Copyright (c) 2023 dresden elektronik ingenieurtechnik gmbh.
 * All rights reserved.
 *
 * The software in this package is published under the terms of the BSD
 * style license a copy of which has been included with this distribution in
 * the LICENSE.txt file.
 *
 */

#ifndef _SAMR21_RTC_H_
#define _SAMR21_RTC_H_

#include "samr21.h"
#include <stdbool.h>

#define SAMR21_RTC_MAX_VALUE 0xFFFFFFFF

void samr21RtcInit();
void samr21RtcDeinit();

uint32_t samr21RtcGetTimestamp();

void samr21RtcSetAlarm(uint32_t alarmTimestamp);
#endif //_SAMR21_RTC_H_          