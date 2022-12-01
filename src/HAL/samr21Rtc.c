//Author Eric Härtel @ dresden elektronik ingenieurtechnik gmbh © 2022
#include "samr21Rtc.h"


extern uint32_t g_currentRtcClkCycle_ns;

void samr21RtcInit(){

    //Reset RTC first
    RTC->MODE0.CTRL.bit.SWRST = 1;

    //Wait for Reset to finish
    while(RTC->MODE0.CTRL.bit.SWRST || RTC->MODE0.STATUS.bit.SYNCBUSY);

    //Setup the RTC
    RTC->MODE0.CTRL.reg=
        RTC_MODE0_CTRL_PRESCALER(0x0) // take __PERIPHERALCLK__ (16Mhz or 1 Mhz depending on TRX settings)
        //|RTC_MODE0_CTRL_MATCHCLR
        |RTC_MODE0_CTRL_MODE(0x0) //COUNT 32Bit mode
        |RTC_MODE0_CTRL_ENABLE
    ;

    //Wait for Setup to finish
    while(RTC->MODE0.STATUS.bit.SYNCBUSY);

    //Force permanent Sync with COUT Register
    RTC->MODE0.READREQ.reg=
        RTC_READREQ_ADDR(RTC_MODE0_COUNT_OFFSET)
        |RTC_READREQ_RCONT
    ;
}

uint32_t samr21RtcGetTimestamp(){
    return RTC->MODE0.COUNT.reg;
}
