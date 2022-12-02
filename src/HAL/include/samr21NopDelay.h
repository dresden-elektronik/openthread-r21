//Author Eric Härtel @ dresden elektronik ingenieurtechnik gmbh © 2022

#ifndef _SAMR21_NOP_DELAY_H_
#define _SAMR21_NOP_DELAY_H_

#include "samr21.h"

void samr21delaySysTick(uint32_t delayCycles);

void samr21delayLoop(uint32_t delayCycles);

#endif // _SAMR21_NOP_DELAY_H_