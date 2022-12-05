#include "openthread/platform/alarm-milli.h"
#include "openthread/platform/alarm-micro.h"

#include "samr21Rtc.h"
#include "otPlatSystemHeader.h"

#include "samr21Timer.h"
#include "samr21Rtc.h"

#define MICRO_SECS_PER_MILLI_SEC 1000

volatile static bool s_alarmFired = false;

void otPlatAlarmMilliStartAt(otInstance *aInstance, uint32_t aT0, uint32_t aDt)
{
    OT_UNUSED_VARIABLE(aInstance);
    s_alarmFired = false;
    samr21Timer3Set( ( aDt - ( samr21RtcGetTimestamp() - aT0 ) ) * MICRO_SECS_PER_MILLI_SEC );
}

// void otPlatAlarmMicroStartAt(otInstance *aInstance, uint32_t aT0, uint32_t aDt)
// {
//     OT_UNUSED_VARIABLE(aInstance);
//     s_alarmFired = false;
//     samr21Timer3Set( ( aDt - ( samr21RtcGetTimestamp() - aT0 ) ) );
// }

void otPlatAlarmMilliStop(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);
    samr21Timer3Stop();
}

// void otPlatAlarmMicroStop(otInstance *aInstance)
// {
//     OT_UNUSED_VARIABLE(aInstance);
//     samr21Timer3Stop();
// }

uint32_t otPlatAlarmMilliGetNow(void)
{
    return samr21RtcGetTimestamp() * MICRO_SECS_PER_MILLI_SEC;
}

// uint32_t otPlatAlarmMicroGetNow(void)
// {
//     return samr21RtcGetTimestamp();
// }

void samr21OtPlatAlarmTask(otInstance *aInstance){
    if(!s_alarmFired){
        return;
    }

    //Copied from https://github.com/openthread/ot-samr21/blob/main/src/alarm.c
#if OPENTHREAD_CONFIG_DIAG_ENABLE
        if (otPlatDiagModeGet())
        {
            s_alarmFired = false;
            otPlatDiagAlarmFired(aInstance);
        }
        else
#endif
        {
            s_alarmFired = false;
            otPlatAlarmMilliFired(aInstance);
        }
    //End copied Snippet
}

void TC3_Handler(){
    TC3->COUNT16.INTFLAG.bit.OVF = 1;
    s_alarmFired = true;
}