//Author Eric Härtel @ dresden elektronik ingenieurtechnik gmbh © 2022

#ifndef _SAMR21_TIMER_H_
#define _SAMR21_TIMER_H_


#include "samr21.h"



void samr21TimerInit();

//Used For OpenThread - Alarm (otPlatAlarm.c)
void samr21Timer3Set(uint16_t value_us);
void samr21Timer3Stop();

//Used For RadioFSM - Events (samr21RadioFSM.c)
void samr21Timer4Set(uint16_t value_us);
void samr21Timer4Stop();
//Used For RadioFSM - Events (samr21RadioFSM.c)
void samr21Timer5Set(uint16_t value_us);
void samr21Timer5Stop();

#endif //_SAMR21_TIMER_H_