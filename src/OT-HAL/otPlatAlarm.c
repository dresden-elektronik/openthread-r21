//Author Eric Härtel @ dresden elektronik ingenieurtechnik gmbh © 2022
#include "openthread/platform/alarm-milli.h"
#include "openthread/platform/alarm-micro.h"

#include "otPlatSystemHeader.h"

#include "samr21Timer.h"
#include "samr21Rtc.h"

#define MICRO_SECS_PER_MILLI_SEC 1000

static otInstance *s_instance = NULL;


static void otPlatAlarmMicroFired_cb(void)
{
    otPlatAlarmMicroFired(s_instance);
}
static void otPlatAlarmMilliFired_cb(void)
{
    otPlatAlarmMilliFired(s_instance);
}

void otPlatAlarmMilliStartAt(otInstance *instance, uint32_t t0_ms, uint32_t dT_ms)
{
    s_instance = instance;

    timestamp_t now = samr21Timer_getCurrentTimestamp();

    uint32_t now_ms = (uint32_t) (now.u64Value  /  MICRO_SECS_PER_MILLI_SEC);

    uint32_t offset_ms = now_ms  - t0_ms;

    timestamp_t triggerTime = now;

    triggerTime.u64Value += (dT_ms - offset_ms) * MICRO_SECS_PER_MILLI_SEC;

    samr21Timer_addScheduledTask(triggerTime, otPlatAlarmMilliFired_cb);
}

void otPlatAlarmMicroStartAt(otInstance *instance, uint32_t t0_ms, uint32_t dT_ms)
{
    s_instance = instance;

    timestamp_t now = samr21Timer_getCurrentTimestamp();

    uint32_t offset =  now.u32Value.lower - t0_ms;

    timestamp_t triggerTime = now;

    triggerTime.u64Value += (dT_ms - offset); 

    samr21Timer_addScheduledTask(triggerTime, otPlatAlarmMicroFired_cb);
}

void otPlatAlarmMilliStop(otInstance *a_Instance)
{
    samr21Timer_removeScheduledTask(otPlatAlarmMilliFired_cb);
}

void otPlatAlarmMicroStop(otInstance *a_Instance)
{
    samr21Timer_removeScheduledTask(otPlatAlarmMicroFired_cb);
}

uint32_t otPlatAlarmMilliGetNow(void)
{
    return (uint32_t)(samr21Timer_getNowU64() / MICRO_SECS_PER_MILLI_SEC);
}

uint32_t otPlatAlarmMicroGetNow(void)
{
    return samr21Timer_getNowU32();
}
