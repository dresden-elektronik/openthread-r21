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
#include <stddef.h>

#include "samr21.h"

typedef void (*samr21Timer_fired_cb)(void);

typedef union timestamp_u
{
    struct 
    {
        uint32_t lower;
        uint32_t upper;
    }u32Value;

    uint64_t u64Value;
    
} timestamp_t;

typedef struct timerJob_s
{
    timestamp_t triggerTime;
    samr21Timer_fired_cb callback;
}timerJob_t;

void samr21Timer_init(void);
void samr21Timer_tick(void);

uint32_t samr21Timer_getNowU32(void);
uint64_t samr21Timer_getNowU64(void);
timestamp_t samr21Timer_getCurrentTimestamp(void);

bool samr21Timer_addScheduledTask(timestamp_t desiredTriggerTime, samr21Timer_fired_cb triggerCallback);
bool samr21Timer_removeScheduledTask(samr21Timer_fired_cb triggerCallback);

bool samr21Timer_addDelayedAction(uint32_t delay, samr21Timer_fired_cb triggerCallback);
bool samr21Timer_removeDelayedAction(samr21Timer_fired_cb triggerCallback);

void samr21Timer_initDmaPaceMaker(void);
void samr21Timer_startDmaPaceMaker(uint16_t period);
void samr21Timer_stopDmaPaceMaker(void);

#endif //_SAMR21_TIMER_H_