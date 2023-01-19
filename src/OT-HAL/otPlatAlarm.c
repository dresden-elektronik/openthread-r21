//Author Eric Härtel @ dresden elektronik ingenieurtechnik gmbh © 2022
#include "openthread/platform/alarm-milli.h"
#include "openthread/platform/alarm-micro.h"

#include "otPlatSystemHeader.h"

#include "samr21Timer.h"
#include "samr21Rtc.h"

#define MICRO_SECS_PER_MILLI_SEC 1000

static otInstance *s_instance = NULL;

void otPlatAlarmMilliStartAt(otInstance *aInstance, uint32_t aT0, uint32_t aDt)
{
    s_instance = aInstance;
    samr21Timer1Set( ( aDt - ( samr21RtcGetTimestamp() - aT0 ) ) * MICRO_SECS_PER_MILLI_SEC );
}

void otPlatAlarmMicroStartAt(otInstance *aInstance, uint32_t aT0, uint32_t aDt)
{
    s_instance = aInstance;
    samr21Timer0Set( ( aDt - ( samr21RtcGetTimestamp() - aT0 ) ) );
}

void otPlatAlarmMilliStop(otInstance *aInstance)
{
    s_instance = aInstance;
    samr21Timer1Stop();
}

void otPlatAlarmMicroStop(otInstance *aInstance)
{
    s_instance = aInstance;
    samr21Timer0Stop();
}

uint32_t otPlatAlarmMilliGetNow(void)
{
    return samr21RtcGetTimestamp() * MICRO_SECS_PER_MILLI_SEC;
}

uint32_t otPlatAlarmMicroGetNow(void)
{
    return samr21RtcGetTimestamp();
}

void TCC0_Handler(){
    TCC0->INTFLAG.bit.OVF = 1;
    otPlatAlarmMilliFired(s_instance);
}

void TCC1_Handler(){
    TCC1->INTFLAG.bit.OVF = 1;
    otPlatAlarmMilliFired(s_instance);
}