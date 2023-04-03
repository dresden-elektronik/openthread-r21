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

/**
 * Sets up the RTC as a microsecond timer
 * and forces a synchronous read-access to the rtc Value Register
 */
void samr21RtcInit();

/**
 * Disables the RTC and the synchronous read-access to the rtc Value Register
 */
void samr21RtcDeinit();

/**
 * Gets a microseconds timestamp since the last initiation of the RTC-Module
 *
 * @returns a uint32 microseconds Timestamp since the initiation of the rtc
 */
uint32_t samr21RtcGetTimestamp();

/**
 * Sets a microseconds timestamp for an Alarm
 * RTC_Handler()-ISR is invoked when alarm triggers
 *
 * @param[in] alarmTimestamp    an absolute timestamp for when the RTC Alarm is supposed to be triggered
 */
void samr21RtcSetAbsoluteAlarm(uint32_t alarmTimestamp);

/**
 * Sets a Alarm on the RTC that triggers after the specified Time
 * RTC_Handler()-ISR is invoked when alarm triggers
 *
 * @param[in] duration    the duration after which the Alarm triggers
 */
void samr21RtcSetRelativeAlarm(uint32_t duration);

/**
 * Stops the currently armed RTC Alarm
 */
void samr21RtcStopAlarm();

#endif //_SAMR21_RTC_H_