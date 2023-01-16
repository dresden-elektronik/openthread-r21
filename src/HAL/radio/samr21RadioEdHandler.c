/*
 * Copyright (c) 2023 dresden elektronik ingenieurtechnik gmbh.
 * All rights reserved.
 *
 * The software in this package is published under the terms of the BSD
 * style license a copy of which has been included with this distribution in
 * the LICENSE.txt file.
 *
 */
#include "samr21RadioEdHandler.h"

static uint32_t s_edScansLeft;
static uint8_t s_maxEdLevel;

static bool s_edDone = false;



//Called at the beginning of each ED
static void samr21RadioEdStartScan()
{   
    if(s_edDone){
        samr21RadioEdCleanup();
        return;
    }
    //Add a Timeout in case something goes wrong
    samr21Timer4Set(0xFF);

    // Prepare CCA Measurment
    g_phyCcCcaReg.bit.ccaRequest = 1;

    // Start CCA Measurment
    samr21TrxWriteRegister(PHY_CC_CCA_REG, g_phyCcCcaReg.reg);

    // Reset local copy of ccaRequest Bit
    g_phyCcCcaReg.bit.ccaRequest = 0;

}

//Called after a RADIO_EVENT_IRQ_CCA_ED_DONE
static void samr21RadioEdEval()
{
    samr21Timer4Stop();

    uint8_t edReading = samr21TrxReadRegister(PHY_ED_LEVEL_REG);

    if(s_maxEdLevel < edReading){
        s_maxEdLevel = edReading;
    }

    samr21RadioEdStartScan();    
}

static void samr21RadioEdCleanup(){
    samr21RadioCtrlReturnToLastHandler();
    cb_samr21RadioEdDone(AT86RF233_RSSI_BASE_VAL + s_maxEdLevel);
}

void samr21RadioEdEventHandler(IrqEvent event)
{
    switch (event)
    {
    case TRX_EVENT_CCA_ED_DONE:
        //Scan Done
        samr21RadioEdEval();
        return;

    case TIMER_EVENT_4_TRIGGER:
        if(s_edDone){
            samr21RadioEdCleanup();
            return;
        }

        samr21RadioEdStartScan();    
        return;

    case RTC_EVENT_ALARM_TRIGGER:
        //Timeout
        s_edDone = true;
        return;

    default:
        return;
    }
}

bool samr21RadioEdStart(uint8_t channel, uint16_t duration_ms)
{
    if(!s_edDone){
        return false;
    }

    samr21RadioRemoveEventHandler();

    //Reset relevant Timer
    samr21Timer4Stop();
    
    s_edDone = false;
    s_maxEdLevel = 0;

    // Set relevant IRQ Mask
    g_irqMask = (AT86RF233_REG_IRQ_MASK_t){
        .bit.pllLock = 0,
        .bit.pllUnlock = 0,
        .bit.rxStart = 0,
        .bit.trxEnd = 0,
        .bit.ccaEdDone = 1,
        .bit.addressMatch = 0,
        .bit.bufferUnderRun = 0,
        .bit.batteryLow = 0
    };
    samr21TrxWriteRegister(IRQ_MASK_REG, g_irqMask.reg);

    // change to desired channel
    g_phyCcCcaReg.bit.channel = channel;
    samr21TrxWriteRegister(PHY_CC_CCA_REG, g_phyCcCcaReg.reg);

    // Put Transciver is in recive state
    if (g_trxStatus.bit.trxStatus != TRX_STATUS_RX_ON)
    {
        samr21TrxWriteRegister(TRX_STATE_REG, TRX_CMD_RX_ON);
        while (g_trxStatus.bit.trxStatus != TRX_STATUS_RX_ON){
            samr21TrxUpdateStatus();
        }
    }

    samr21RtcSetAlarm(samr21RtcGetTimestamp() + ( duration_ms * 1000 ) ); //1000us in 1 ms 

    samr21RadioEdStartScan();

    return true;
}

int8_t samr21RadioEdGetLastResult(){
    return AT86RF233_RSSI_BASE_VAL + s_maxEdLevel;
}