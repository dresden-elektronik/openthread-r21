//Author Eric Härtel @ dresden elektronik ingenieurtechnik gmbh © 2022

#ifndef _SAMR21_CLOCK_H_
#define _SAMR21_CLOCK_H_

#include "samr21.h"
#include "samr21NopDelay.h"

void samr21ClockInit();
void samr21ClockInputInit();

void samr21ClockTrxSrcInit();

void samr21ClockInitAfterTrxSetup();

#endif //_SAMR21_CLOCK_H_