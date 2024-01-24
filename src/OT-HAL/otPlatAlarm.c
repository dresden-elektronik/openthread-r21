//Author Eric Härtel @ dresden elektronik ingenieurtechnik gmbh © 2022
#include "openthread/platform/alarm-milli.h"
#include "openthread/platform/alarm-micro.h"

#include "otPlatSystemHeader.h"

#include "samr21Timer.h"
#include "samr21Rtc.h"

#define MICRO_SECS_PER_MILLI_SEC 1000

static otInstance *s_instance = NULL;

static struct otPlatAlarmVars_s
{
    otInstance *instance;

    bool millisAlarmArmed;
    bool microsAlarmArmed;

}s_otPlatAlarmVars;




void otPlatAlarmMilliStartAt(otInstance *a_instance, uint32_t a_t0_ms, uint32_t a_dT_ms)
{
    s_otPlatAlarmVars.instance = a_instance;

    uint32_t now_ms = samr21Rtc_getTimestamp()  /  MICRO_SECS_PER_MILLI_SEC;

    uint32_t offset_ms = now_ms  - a_t0_ms;

    s_otPlatAlarmVars.millisAlarmArmed = true;
    samr21Timer2_startOneshot( a_dT_ms  -  offset_ms);
}

void otPlatAlarmMicroStartAt(otInstance *a_Instance, uint32_t a_t0_ms, uint32_t a_dT_ms)
{
    s_otPlatAlarmVars.instance = a_Instance;

    uint32_t now_ms = samr21Rtc_getTimestamp();

    uint32_t offset_ms = now_ms  - a_t0_ms;

    s_otPlatAlarmVars.microsAlarmArmed = true;
    samr21Timer1_startOneshot( a_dT_ms - ( samr21Rtc_getTimestamp() - a_t0_ms ) );
}

void otPlatAlarmMilliStop(otInstance *a_Instance)
{
    s_otPlatAlarmVars.instance = a_Instance;
    samr21Timer2_stop();

    s_otPlatAlarmVars.millisAlarmArmed = false;
}

void otPlatAlarmMicroStop(otInstance *a_Instance)
{
    s_otPlatAlarmVars.instance = a_Instance;
    samr21Timer1_stop();

    s_otPlatAlarmVars.microsAlarmArmed = false;
}

uint32_t otPlatAlarmMilliGetNow(void)
{
    return samr21Rtc_getTimestamp() / MICRO_SECS_PER_MILLI_SEC;
}

uint32_t otPlatAlarmMicroGetNow(void)
{
    return samr21Rtc_getTimestamp();
}

// void TCC1_Handler(){
//     TCC1->INTFLAG.bit.OVF = 1;

//     if(s_instance){
//         otPlatAlarmMicroFired(s_instance);
//     }
// }

// void TCC2_Handler(){
//     TCC2->INTFLAG.bit.OVF = 1;

//     if(s_instance){
//         otPlatAlarmMilliFired(s_instance);
//     }
// }

void samr21OtPlatAlarmInit(void)
{
    //TCC1 Used by OT Micros Alarm
    samr21Timer1_init(0,true,false); // 1MHz / (2^0) -> 1us resolution
    //TCC2 Used by OT Millis Alarm
    samr21Timer2_init(7, true,false); // 1MHz / (2^7) -> ~1ms resolution
}

void samr21OtPlatAlarmTask(void)
{
    if (TCC1->INTFLAG.bit.OVF)
    {
        //Clear Trigger Flag
        TCC1->INTFLAG.bit.OVF = 1;

        if(s_otPlatAlarmVars.microsAlarmArmed)
        {
            s_otPlatAlarmVars.microsAlarmArmed = false;
            
            //Inform OT
            otPlatAlarmMicroFired(s_otPlatAlarmVars.instance);
        }
    }
    
    if(TCC2->INTFLAG.bit.OVF)
    {
        //Clear Trigger Flag
        TCC2->INTFLAG.bit.OVF = 1;

        if(s_otPlatAlarmVars.millisAlarmArmed)
        {
            s_otPlatAlarmVars.millisAlarmArmed = false;

            //Inform OT
            otPlatAlarmMilliFired(s_otPlatAlarmVars.instance);
        }
    }
}