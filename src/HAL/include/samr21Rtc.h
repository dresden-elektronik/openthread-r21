//Author Eric Härtel @ dresden elektronik ingenieurtechnik gmbh © 2022

#ifndef _SAMR21_RTC_H_
#define _SAMR21_RTC_H_

#include "samr21.h"
#include <stdbool.h>

#define SAMR21_RTC_MAX_VALUE 0xFFFFFFFF

void samr21RtcInit();
void samr21RtcDeinit();

uint32_t samr21RtcGetTimestamp();

#endif //_SAMR21_RTC_H_          