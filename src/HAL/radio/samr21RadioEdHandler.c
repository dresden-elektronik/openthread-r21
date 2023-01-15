//Author Eric Härtel @ dresden elektronik ingenieurtechnik gmbh © 2022
#include "samr21RadioEdHandler.h"

static uint32_t s_edScansLeft;
static uint8_t s_maxEdLevel;

static EdStatus s_edStatus = ED_STATUS_IDLE;

bool samr21RadioEdSetup(uint8_t channel, uint16_t duration_ms)
{
    if(s_edStatus != ED_STATUS_IDLE){
        return false;
    }

    samr21RadioRemoveEventHandler();

    //Reset relevant Timer
    samr21Timer5Stop();
    samr21Timer3Stop();


    s_edStatus = ED_STATUS_SETUP;
    s_edScansLeft = duration_ms * NUM_ENERGY_DETECTION_SCANS_PER_MS;
    s_maxEdLevel = 0;
    samr21RadioCtrlSetState(SAMR21_RADIO_STATE_RECEIVE);

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

    samr21RadioSetEventHandler(&samr21RadioEdEventHandler);

    //Timeout timer
    samr21Timer5Set(0xFFFF);

    samr21RadioStartEd();

    return true;
}

void samr21RadioEdEventHandler(IrqEvent event)
{
    switch (event)
    {

    case TIMER_EVENT_3_TRIGGER:
        //Next Scan
        samr21RadioEdStartScan();
        return;

    case TRX_EVENT_CCA_ED_DONE:
        //Scan Done
        samr21RadioEdEval();
        return;

    case TIMER_EVENT_5_TRIGGER:
        //Timeout
        samr21RadioEdCleanup();
        return;

    default:
        return;
    }
}


//Called at the beginning of each ED
void samr21RadioEdStartScan()
{   
    // Prepare CCA Measurment
    g_phyCcCcaReg.bit.ccaRequest = 1;

    // Start CCA Measurment
    samr21TrxWriteRegister(PHY_CC_CCA_REG, g_phyCcCcaReg.reg);
    s_edStatus = ED_STATUS_WAIT_FOR_RESULT;

    // Reset local copy of ccaRequest Bit
    g_phyCcCcaReg.bit.ccaRequest = 0;

    //Queue the next ED allrdy
    if (--s_edScansLeft){
        samr21Timer3Set(TIME_UNTIL_NEXT_ENERGY_DETECTION_SCAN_us);
    }
}

//Called after a RADIO_EVENT_IRQ_CCA_ED_DONE
void samr21RadioEdEval()
{   
    uint8_t edReading = samr21TrxReadRegister(PHY_ED_LEVEL_REG);

    if(s_maxEdLevel < edReading){
        s_maxEdLevel = edReading;
    }

    if(s_edScansLeft > 0){
        s_edStatus = ED_STATUS_WAIT_FOR_NEXT_SCAN;
        return;
    }
    s_edStatus = ED_STATUS_DONE;
    samr21RadioEdCleanup();
}

void samr21RadioEdCleanup(){
    samr21Timer5Stop();
    samr21Timer3Stop();

    samr21RadioRemoveEventHandler();
    
    cb_samr21RadioEdDone(AT86RF233_RSSI_BASE_VAL + s_maxEdLevel);
    s_edStatus = ED_STATUS_IDLE;

    // Set relevant IRQ Mask to return to Recive State

    samr21RadioCtrlSetIdle();
}