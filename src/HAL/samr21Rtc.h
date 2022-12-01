//Author Eric Härtel @ dresden elektronik ingenieurtechnik gmbh © 2022

#ifndef _SAMR21_RTC_H_
#define _SAMR21_RTC_H_

#include "samr21.h"
#include <stdbool.h>

void samr21RtcInit();

uint32_t samr21RtcGetTimestamp();

#endif //_SAMR21_RTC_H_          