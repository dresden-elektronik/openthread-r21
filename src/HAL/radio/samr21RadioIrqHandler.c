/*
 * Copyright (c) 2023 dresden elektronik ingenieurtechnik gmbh.
 * All rights reserved.
 *
 * The software in this package is published under the terms of the BSD
 * style license a copy of which has been included with this distribution in
 * the LICENSE.txt file.
 *
 */
#include "samr21RadioIrqHandler.h"

static EventHandlerFunc s_eventHandlerFunc = NULL;

void samr21RadioSetEventHandler(EventHandlerFunc eventHandlerFunc){
    s_eventHandlerFunc = eventHandlerFunc;
}

void samr21RadioRemoveEventHandler(){
    s_eventHandlerFunc = NULL;
}

void EIC_Handler()
{
    // Clear IRQ
    EIC->INTFLAG.bit.EXTINT0 = 1;

    g_trxLastIrq = (AT86RF233_REG_IRQ_STATUS_t)samr21TrxReadRegister(IRQ_STATUS_REG);

    if (g_trxLastIrq.reg == 0x00 || s_eventHandlerFunc == NULL)
    {
        return;
    }

    if (g_trxLastIrq.bit.rxStart)
    {
        (*s_eventHandlerFunc)(TRX_EVENT_RX_START);
    }

    if (g_trxLastIrq.bit.trxEnd)
    {
        (*s_eventHandlerFunc)(TRX_EVENT_TRX_END);
    }

    if (g_trxLastIrq.bit.pllLock)
    {
        (*s_eventHandlerFunc)(TRX_EVENT_PLL_LOCK);
    }

    if (g_trxLastIrq.bit.pllUnlock)
    {
        (*s_eventHandlerFunc)(TRX_EVENT_PLL_UNLOCK);
    }

    if (g_trxLastIrq.bit.ccaEdDone)
    {
        (*s_eventHandlerFunc)(TRX_EVENT_CCA_ED_DONE);
    }

    if (g_trxLastIrq.bit.addressMatch)
    {
        (*s_eventHandlerFunc)(TRX_EVENT_RX_ADDRESS_MATCH);
    }

    if (g_trxLastIrq.bit.bufferUnderRun)
    {
        (*s_eventHandlerFunc)(TRX_EVENT_BUFFER_READ_UNDERRUN);
    }

    if (g_trxLastIrq.bit.batteryLow)
    {
        (*s_eventHandlerFunc)(TRX_EVENT_BAT_LOW);
    }
}

// timer via TC3
void TC3_Handler()
{
    // Reset IRQ
    TC3->COUNT16.INTFLAG.bit.OVF = 1;

    if(s_eventHandlerFunc){
        (*s_eventHandlerFunc)(TIMER_EVENT_3_TRIGGER);
    } 
}

// timer via TC4
void TC4_Handler()
{
    // Reset IRQ
    TC4->COUNT16.INTFLAG.bit.OVF = 1;

    if(s_eventHandlerFunc){
        (*s_eventHandlerFunc)(TIMER_EVENT_4_TRIGGER);
    } 
}

// timer via TC5
void TC5_Handler()
{
    // Reset IRQ
    TC5->COUNT16.INTFLAG.bit.OVF = 1;

    if(s_eventHandlerFunc){
        (*s_eventHandlerFunc)(TIMER_EVENT_5_TRIGGER);
    } 
}


void RTC_Handler()
{
    RTC->MODE0.INTENFLAG.bit.CMP0 = 1;
    RTC->MODE0.INTENCLR.bit.CMP0 = 1;
    NVIC_DisableIRQ(RTC_IRQn);


    if(s_eventHandlerFunc){
        (*s_eventHandlerFunc)(RTC_EVENT_ALARM_TRIGGER);
    } 
}