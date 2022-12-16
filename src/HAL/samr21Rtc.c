//Author Eric Härtel @ dresden elektronik ingenieurtechnik gmbh © 2022
#include "samr21Rtc.h"


extern uint32_t g_currentRtcClkCycle_ns;

void samr21RtcInit(){

    //Use GCLKGEN 2 (1MHz) for RTC 
    GCLK->CLKCTRL.reg =
        //GCLK_CLKCTRL_WRTLOCK
        GCLK_CLKCTRL_CLKEN
        |GCLK_CLKCTRL_GEN(2) // GCLKGEN2
        |GCLK_CLKCTRL_ID(GCLK_CLKCTRL_ID_RTC_Val)
    ;
    //Wait for synchronization 
    while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY );

    //Enable RTC In Power Manger
    PM->APBAMASK.bit.RTC_ = 1;

    //Reset RTC first
    RTC->MODE0.CTRL.bit.SWRST = 1;

    //Wait for Reset to finish
    while(RTC->MODE0.CTRL.bit.SWRST || RTC->MODE0.STATUS.bit.SYNCBUSY);

    //Setup the RTC
    RTC->MODE0.CTRL.reg=
        RTC_MODE0_CTRL_PRESCALER(0x0) // 1 Mhz 
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

void samr21RtcDeinit(){
    //Disable permanent Sync with COUT Register
    RTC->MODE0.READREQ.reg = 0x00;
    samr21delaySysTick(100);

    //Disable RTC
    RTC->MODE0.CTRL.bit.ENABLE = 0;
    while(RTC->MODE0.STATUS.bit.SYNCBUSY);

    //Disable RTC In Power Manger
    PM->APBAMASK.bit.RTC_ = 0;

    //Disable CLKGEN
    GCLK->CLKCTRL.reg =
        //GCLK_CLKCTRL_WRTLOCK
        //GCLK_CLKCTRL_CLKEN
        GCLK_CLKCTRL_GEN(2) // GCLKGEN2
        |GCLK_CLKCTRL_ID(GCLK_CLKCTRL_ID_RTC_Val)
    ;
}

uint32_t samr21RtcGetTimestamp(){
    return RTC->MODE0.COUNT.reg;
}
