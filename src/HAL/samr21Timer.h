//Author Eric Härtel @ dresden elektronik ingenieurtechnik gmbh © 2022

#ifndef _SAMR21_TIMER_H_
#define _SAMR21_TIMER_H_


#include "samr21.h"



void samr21TimerInit();
void samr21Timer4Set(uint16_t value_us);
void samr21Timer4Stop();

void samr21Timer5Set(uint16_t value_us);
void samr21Timer5Stop();

#endif //_SAMR21_TIMER_H_