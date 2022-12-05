#include "openthread/platform/alarm-milli.h"
#include "samr21Timer.h"
#include "samr21Rtc.h"

#define MICRO_SECS_PER_MILLI_SEC 1000

void otPlatAlarmMilliStartAt(otInstance *aInstance, uint32_t aT0, uint32_t aDt)
{
    OT_UNUSED_VARIABLE(aInstance);
    samr21Timer3Set( ( aDt - ( samr21RtcGetTimestamp() - aT0 ) ) * MICRO_SECS_PER_MILLI_SEC );
}

void otPlatAlarmMilliStop(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);
    samr21Timer3Stop();
}

uint32_t otPlatAlarmMilliGetNow(void)
{
    return samr21RtcGetTimestamp() * MICRO_SECS_PER_MILLI_SEC;
}

void TC3_Handler(){
    TC3->COUNT16.INTFLAG.bit.OVF = 1;

//Copied from https://github.com/openthread/ot-samr21/blob/main/src/alarm.c
#if OPENTHREAD_CONFIG_DIAG_ENABLE
        if (otPlatDiagModeGet())
        {
            otPlatDiagAlarmFired(aInstance);
        }
        else
#endif
        {
            otPlatAlarmMilliFired(aInstance);
        }
}

