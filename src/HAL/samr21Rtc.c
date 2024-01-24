/*
 * Copyright (c) 2023 dresden elektronik ingenieurtechnik gmbh.
 * All rights reserved.
 *
 * The software in this package is published under the terms of the BSD
 * style license a copy of which has been included with this distribution in
 * the LICENSE.txt file.
 *
 */
#include "samr21Rtc.h"

extern uint32_t g_currentRtcClkCycle_ns;

void samr21Rtc_init(){

    //Use GCLKGEN 3 (1MHz) for RTC 
    GCLK->CLKCTRL.reg =
        //GCLK_CLKCTRL_WRTLOCK
        GCLK_CLKCTRL_CLKEN
        |GCLK_CLKCTRL_GEN(3) 
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

void samr21Rtc_deinit(){
    //Disable IRQ
    NVIC_DisableIRQ(RTC_IRQn);

     if (RTC->MODE0.READREQ.bit.RCONT)
    {
         //Disable permanent Sync with COUT Register
        RTC->MODE0.READREQ.bit.RCONT = 0;
        while (RTC->MODE0.READREQ.bit.RCONT );
    }
   
    if ( RTC->MODE0.CTRL.bit.ENABLE ){
        RTC->MODE0.CTRL.bit.ENABLE = 0;
        while(RTC->MODE0.CTRL.bit.ENABLE || RTC->MODE0.STATUS.bit.SYNCBUSY);
    }
}

uint32_t samr21Rtc_getTimestamp(){
    return RTC->MODE0.COUNT.reg;
}

void samr21Rtc_setAbsoluteAlarm(uint32_t a_alarmTimestamp){
    RTC->MODE0.COMP[0].reg = a_alarmTimestamp;
    RTC->MODE0.INTFLAG.bit.CMP0 = 1;
    RTC->MODE0.INTENSET.bit.CMP0 = 1;
    NVIC_EnableIRQ(RTC_IRQn);
}

void samr21Rtc_setRelativeAlarm(uint32_t a_duration){
    RTC->MODE0.COMP[0].reg = samr21Rtc_getTimestamp() + a_duration;
    RTC->MODE0.INTFLAG.bit.CMP0 = 1;
    RTC->MODE0.INTENSET.bit.CMP0 = 1;
    NVIC_EnableIRQ(RTC_IRQn);
}

void samr21Rtc_disableAlarm(){
    NVIC_DisableIRQ(RTC_IRQn);
    RTC->MODE0.INTFLAG.bit.CMP0 = 1;
    RTC->MODE0.INTENCLR.bit.CMP0 = 1;
}

