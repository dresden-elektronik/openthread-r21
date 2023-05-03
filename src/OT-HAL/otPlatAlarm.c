//Author Eric Härtel @ dresden elektronik ingenieurtechnik gmbh © 2022
#include "openthread/platform/alarm-milli.h"
#include "openthread/platform/alarm-micro.h"

#include "otPlatSystemHeader.h"

#include "samr21Timer.h"
#include "samr21Rtc.h"

#define MICRO_SECS_PER_MILLI_SEC 1000

static otInstance *s_instance = NULL;

void otPlatAlarmMilliStartAt(otInstance *a_instance, uint32_t a_t0_ms, uint32_t a_dT_ms)
{
    s_instance = a_instance;

    uint32_t now_ms = samr21RtcGetTimestamp()  /  1000;

    uint32_t offset_ms = now_ms  - a_t0_ms;

    samr21Timer2Oneshot( a_dT_ms  -  offset_ms);
}

void otPlatAlarmMicroStartAt(otInstance *a_Instance, uint32_t a_t0_ms, uint32_t a_dT_ms)
{
    s_instance = a_Instance;


    uint32_t now_ms = samr21RtcGetTimestamp();

    uint32_t offset_ms = now_ms  - a_t0_ms;

    samr21Timer1Oneshot( a_dT_ms - ( samr21RtcGetTimestamp() - a_t0_ms ) );
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