//Author Eric Härtel @ dresden elektronik ingenieurtechnik gmbh © 2022
#include "openthread/platform/alarm-milli.h"
#include "openthread/platform/alarm-micro.h"

#include "otPlatSystemHeader.h"

#include "samr21Timer.h"
#include "samr21Rtc.h"

#define MICRO_SECS_PER_MILLI_SEC 1000

static otInstance *s_instance = NULL;

void otPlatAlarmMilliStartAt(otInstance *a_instance, uint32_t a_t0, uint32_t a_dT)
{
    s_instance = a_instance;
    samr21Timer2Set( a_dT - ( samr21RtcGetTimestamp() - a_t0 ) );
}

void otPlatAlarmMicroStartAt(otInstance *a_Instance, uint32_t a_t0, uint32_t a_dT)
{
    s_instance = a_Instance;
    samr21Timer1Set( a_dT - ( samr21RtcGetTimestamp() - a_t0 ) );
}

void otPlatAlarmMilliStop(otInstance *a_Instance)
{
    s_instance = a_Instance;
    samr21Timer2Stop();
}

void otPlatAlarmMicroStop(otInstance *a_Instance)
{
    s_instance = a_Instance;
    samr21Timer1Stop();
}

uint32_t otPlatAlarmMilliGetNow(void)
{
    return samr21RtcGetTimestamp() / MICRO_SECS_PER_MILLI_SEC;
}

uint32_t otPlatAlarmMicroGetNow(void)
{
    return samr21RtcGetTimestamp();
}

void TCC1_Handler(){
    TCC1->INTFLAG.bit.OVF = 1;

    if(s_instance){
        otPlatAlarmMicroFired(s_instance);
    }
}

void TCC2_Handler(){
    TCC2->INTFLAG.bit.OVF = 1;

    if(s_instance){
        otPlatAlarmMilliFired(s_instance);
    }
}