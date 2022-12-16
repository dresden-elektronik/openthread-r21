// Author Eric Härtel @ dresden elektronik ingenieurtechnik gmbh © 2022

#include "samr21Radio.h"

// Ringbuffer for Transmissions
static struct RingTransmissionBuffer
{
    JobBuffer_t bufferData[NUM_RADIO_JOB_BUFFER];
    uint8_t activeBufferIndex;
} s_ringTransmissionBuffer;

static JobBuffer_t *sf_ringBufferGetCurrent()
{
    return &(s_ringTransmissionBuffer.bufferData[s_ringTransmissionBuffer.activeBufferIndex]);
};

static JobBuffer_t *sf_ringBufferGetNext()
{
    return &(s_ringTransmissionBuffer.bufferData[(s_ringTransmissionBuffer.activeBufferIndex + 1) % NUM_RADIO_JOB_BUFFER]);
};

static void sf_ringBufferMoveForward()
{
    s_ringTransmissionBuffer.activeBufferIndex = (s_ringTransmissionBuffer.activeBufferIndex + 1) % NUM_RADIO_JOB_BUFFER;
    samr21RadioFsmChangeJobStatePointer(
        &(s_ringTransmissionBuffer.bufferData[s_ringTransmissionBuffer.activeBufferIndex].jobState)
    );
};

//Frame Pending Table Short Addr
static struct framePendingTableShortAddr
{
    uint16_t addr[SIZE_TABLE_FRAME_PENDING_SHORT_ADDR];
    uint8_t size;
} s_framePendingTableShortAddr;

//Frame Pending Table IEEE Addr
static struct framePendingTableIeeeAddr
{
    uint64_t addr[SIZE_TABLE_FRAME_PENDING_IEEE_ADDR];
    uint8_t size;
} s_framePendingTableIeeeAddr;


static RadioState s_radioState = RADIO_STATE_IDLE;

// Local Register Copys of At86rf233 to save some Read Acceses
extern AT86RF233_REG_TRX_STATUS_t   g_trxStatus;  // from samr21trx.c
extern AT86RF233_REG_IRQ_STATUS_t   g_trxLastIrq; // from samr21trx.c
extern AT86RF233_REG_TRX_CTRL_0_t   g_trxCtrl0;   // from samr21trx.c

static AT86RF233_REG_PHY_CC_CCA_t   s_phyCcCcaReg;
static AT86RF233_REG_PHY_RSSI_t     s_phyRssi;
static AT86RF233_REG_CCA_THRES_t    s_ccaThres;

static AT86RF233_REG_TRX_CTRL_1_t   s_trxCtrl1 =
    {
        .bit.irqPolarity = 0,
        .bit.irqMaskMode = 0,
        .bit.spiCmdMode = 0x1,
        .bit.rxBlCtrl = 1,
        .bit.txAutoCrcOn = 1,
        .bit.irq2ExtEn = 0,
        .bit.paExtnEn = 0
    };

static AT86RF233_REG_PHY_TX_PWR_t   s_phyTxPwr =
    {
        .bit.txPwr = 0x7
    };

static AT86RF233_REG_IRQ_MASK_t     s_irqMask =
    {
        .bit.pllLock = 1,
        .bit.pllUnlock = 0,
        .bit.rxStart = 1,
        .bit.trxEnd = 1,
        .bit.ccaEdDone = 1,
        .bit.addressMatch = 1,
        .bit.bufferUnderRun = 1,
        .bit.batteryLow = 0
    };

// 802_15_4 "Mac-Lite" Vars
static uint8_t s_numMaxTransmissionRetrys = 3;
static uint8_t s_numMaxCsmaBackoffs = 4;

static bool s_promiscuousMode = false;

static uint32_t s_csmaBackoffIntervallMin_us    = 16;   // SYMBOL_DURATION_802_15_4_us*((2^minBE)-1) (default: minBE = 1)
static uint32_t s_csmaBackoffIntervallMax_us    = 240;  // SYMBOL_DURATION_802_15_4_us*((2^maxBE)-1) (default: maxBE = 4)
static uint32_t s_csmaBackoffIntervallDiff_us   = 224; // sCsmaBackoffIntervallMax - sCsmaBackoffIntervallMax
static uint32_t s_csmaBackoffIntervallStep_us   = 14;  // sCsmaBackoffIntervallDiff / 16, need cause at86rf233 can create 2 Random bits per SPI Read 

const static uint16_t s_ackMaxWaitDuration_us   = 250; // 12 * SYMBOL_DURATION_802_15_4_us
static uint16_t s_transmissionTimeout_us        = 0xFFFF;

static uint8_t s_ieeeAddr[8];
static uint8_t s_shortAddr[2];
static uint8_t s_panId[2];

//TX Power Table (Factor x4 for Resolution and to avoid float)(r21 Datasheet Table 38-9)
#define SIZE_AT86RF233_TX_POWER_TABLE 16
static const int8_t s_txPowerTable[SIZE_AT86RF233_TX_POWER_TABLE]={16,15,14,12,10,8,4,0,-4,-8,-12,-16,-24,-32,-48,-68};

void samr21RadioInit()
{
    //Write default Values to Config Registers of AT86rf233 (that are not allrdy writen by samr21TrxInterfaceInit)
    samr21TrxWriteRegister(TRX_CTRL_1_REG, s_trxCtrl1.reg);
    samr21TrxWriteRegister(PHY_TX_PWR_REG, s_phyTxPwr.reg);
    samr21TrxWriteRegister(IRQ_MASK_REG, s_irqMask.reg);


    // Get TRX into RX State
    samr21TrxWriteRegister(TRX_STATE_REG, TRX_CMD_FORCE_TRX_OFF);
    while (g_trxStatus.bit.trxStatus != TRX_STATUS_TRX_OFF){
        samr21TrxUpdateStatus();
    }

    // Setup ringBuffer
    s_ringTransmissionBuffer.activeBufferIndex = 0;
    for(uint8_t i = 0; i < NUM_RADIO_JOB_BUFFER; i++){
        s_ringTransmissionBuffer.bufferData->jobState = RADIO_JOB_STATE_IDLE;
    }

    // Setup inital State for FSM
    samr21RadioFsmChangeJobStatePointer(&(sf_ringBufferGetCurrent()->jobState));

    // Enable IRQ via EIC
    //Use GCLKGEN0 as core Clock for EIC (At86rf233, IRQ_Detect)
    GCLK->CLKCTRL.reg =
        //GCLK_CLKCTRL_WRTLOCK
        GCLK_CLKCTRL_CLKEN
        |GCLK_CLKCTRL_GEN(0) // GCLKGEN1
        |GCLK_CLKCTRL_ID(GCLK_CLKCTRL_ID_EIC_Val)
    ;
    //Wait for synchronization 
    while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY );  
    // Reset first and wait for reset to finish
    EIC->CTRL.bit.SWRST = 1;
    while (EIC->STATUS.bit.SYNCBUSY);
    while (EIC->CTRL.bit.SWRST);

    // Enable EXINT[0]
    EIC->CONFIG[0].bit.FILTEN0 = 0;
    EIC->CONFIG[0].bit.SENSE0 = EIC_CONFIG_SENSE0_RISE_Val;
    EIC->INTENSET.bit.EXTINT0 = 1;

    // Enable Module
    EIC->CTRL.bit.ENABLE = 1;
    while (EIC->STATUS.bit.SYNCBUSY);
    while (!(EIC->CTRL.bit.ENABLE));

    //cLEAR irq BEFORE ENABLING
    EIC->INTFLAG.bit.EXTINT0 = 1;

    //Read Once to clear all pending Interrupts
    samr21TrxReadRegister(IRQ_STATUS_REG);

    // Enable IRQ in NVIC
    __NVIC_EnableIRQ(EIC_IRQn);
}

void samr21RadioChangeState(RadioState newState, uint8_t channel){
    switch (newState)
    {
    case RADIO_STATE_IDLE:
        samr21RadioFsmEnable(false);
        samr21TrxWriteRegister(TRX_STATE_REG, TRX_CMD_FORCE_TRX_OFF);
        s_radioState = RADIO_STATE_IDLE;
        break;

    case RADIO_STATE_SLEEP:
        //Real SleepMode cant be used cause the MCLK form TRX would be disabled
        samr21RadioFsmEnable(false);
        samr21TrxWriteRegister(TRX_STATE_REG, TRX_CMD_FORCE_TRX_OFF);
        s_radioState = RADIO_STATE_SLEEP;
        break;

    case RADIO_STATE_RX:
        //Look if channel needs to be changed
        if( channel && ( channel != s_phyCcCcaReg.bit.channel ) ){
            s_phyCcCcaReg.bit.channel = channel;
            samr21TrxWriteRegister(PHY_CC_CCA_REG, s_phyCcCcaReg.reg);
        }
        samr21RadioFsmEnable(true);
        samr21TrxWriteRegister(TRX_STATE_REG, TRX_CMD_RX_ON);
        s_radioState = RADIO_STATE_RX;
        break;

    case RADIO_STATE_TX:
        //Look if channel needs to be changed
        if( channel && ( channel != s_phyCcCcaReg.bit.channel ) ){
            s_phyCcCcaReg.bit.channel = channel;
            samr21TrxWriteRegister(PHY_CC_CCA_REG, s_phyCcCcaReg.reg);
        }
        samr21RadioFsmEnable(true);

        //RX_ON, cause almost all transmissions start with a CCA
        samr21TrxWriteRegister(TRX_STATE_REG, TRX_CMD_RX_ON);
        s_radioState = RADIO_STATE_TX;
        break;
    
    default:
        break;
    }
}

RadioState samr21RadioGetStatus(){
    return s_radioState;
}

void samr21RadioSetTxPower(int8_t txPower)
{
    txPower *= 4; // multiply by 4 (for resolution in txPowerTable)

    for(uint8_t i = 0; i < SIZE_AT86RF233_TX_POWER_TABLE; i++){
        if(
            (s_txPowerTable[i] >= txPower)
            && (s_txPowerTable[i+1] <= txPower)
        ){
            s_phyTxPwr.bit.txPwr = i;
            goto writeReg;
        }
    }

    //Default
    s_phyTxPwr.bit.txPwr = 0x7;

writeReg:
    samr21TrxWriteRegister(PHY_TX_PWR_REG, s_phyTxPwr.reg);
}

int8_t samr21RadioGetTxPower(){
    return s_txPowerTable[s_phyTxPwr.bit.txPwr] >> 2; // devide by 4
}

uint8_t samr21RadioReadLastEDMeasurment()
{
    return samr21TrxReadRegister(PHY_ED_LEVEL_REG);
}

uint8_t samr21RadioGetRandomCrumb()
{
    return (samr21TrxReadRegister(PHY_RSSI_REG) >> 5) & 0b00000011;
}
uint8_t samr21RadioGetRandomNibble()
{
    return ((samr21TrxReadRegister(PHY_RSSI_REG) >> 5) & 0b00000011) | ((samr21TrxReadRegister(PHY_RSSI_REG) >> 3) & 0b00001100);
}
uint8_t samr21RadioGetRandomByte()
{
    uint8_t rVal = (samr21TrxReadRegister(PHY_RSSI_REG) & 0x60) >> 5;
    rVal |= (samr21TrxReadRegister(PHY_RSSI_REG) & 0x60) >> 3;
    rVal |= (samr21TrxReadRegister(PHY_RSSI_REG) & 0x60) >> 1;
    rVal |= (samr21TrxReadRegister(PHY_RSSI_REG) & 0x60) << 1;
    return rVal;
}

void samr21RadioSetShortAddr(uint8_t* shortAddr)
{
    s_shortAddr[0] = shortAddr[0];
    s_shortAddr[1] = shortAddr[1];

    samr21TrxWriteRegister(SHORT_ADDR_0_REG, shortAddr[0]);
    samr21TrxWriteRegister(SHORT_ADDR_1_REG, shortAddr[1]);
}

void samr21RadioSetPanId(uint8_t* panId)
{
    s_panId[0] = panId[0];
    s_panId[1] = panId[1];

    samr21TrxWriteRegister(PAN_ID_0_REG, panId[0]);
    samr21TrxWriteRegister(PAN_ID_1_REG, panId[1]);
}

void samr21RadioSetIeeeAddr(uint8_t* ieeeAddr)
{
    for (uint8_t i = 0; i < 8; i++) // IEEE address has 8 bytes
    {
        s_ieeeAddr[i] = ieeeAddr[i];
        samr21TrxWriteRegister(IEEE_ADDR_0_REG + i, ieeeAddr[i]);
    }
}

void samr21RadioChangeChannel(uint8_t newChannel)
{
    s_phyCcCcaReg.bit.channel = newChannel;
    samr21TrxWriteRegister(PHY_CC_CCA_REG, s_phyCcCcaReg.reg);
}

void samr21RadioChangeCCAMode(uint8_t newCcaMode)
{
    s_phyCcCcaReg.bit.ccaMode = newCcaMode;
    samr21TrxWriteRegister(PHY_CC_CCA_REG, s_phyCcCcaReg.reg);
}

void samr21RadioSetCcaThreshold(int8_t threshold)
{
    int8_t diff = (AT86RF233_RSSI_BASE_VAL - threshold);
    s_ccaThres.bit.ccaEdThres = abs(diff) >> 1; //Devide by 2

    samr21TrxWriteRegister(CCA_THRES_REG, s_ccaThres.reg);
}

int8_t samr21RadioGetCurrentCcaThreshold()
{
    return ( AT86RF233_RSSI_BASE_VAL + ( s_ccaThres.bit.ccaEdThres << 1 ) ); //Multiply by 2
}

void samr21RadioChangeCsmaBackoffExponent(uint8_t minBE, uint8_t maxBE)
{
    s_csmaBackoffIntervallMin_us = ((1 << minBE) - 1) * SYMBOL_DURATION_802_15_4_us;
    s_csmaBackoffIntervallMax_us = ((1 << maxBE) - 1) * SYMBOL_DURATION_802_15_4_us;
    s_csmaBackoffIntervallDiff_us = s_csmaBackoffIntervallMax_us - s_csmaBackoffIntervallMin_us;
}

void samr21RadioChangeNumBackoffsCsma(uint8_t numBackoffs)
{
    s_numMaxCsmaBackoffs = numBackoffs;
}

void samr21RadioChangeNumTransmitRetrys(uint8_t numRetrys)
{
    s_numMaxTransmissionRetrys = numRetrys;
}

void samr21RadioEnablePromiscuousMode(bool enable){
    s_promiscuousMode = enable;
}

/*------------------*/
// Interface Functions
/*------------------*/
bool samr21RadioSendFrame(FrameBuffer_t *frame, uint8_t channel)
{
    // Get the next avivable Buffer
    __disable_irq();
    JobBuffer_t *buffer = sf_ringBufferGetNext();

    //Check if Last Queued Transmission did even start yet
    if(buffer->jobState == RADIO_JOB_STATE_TX_READY){
        __enable_irq();
        return false;
    }

    // mark buffer as in Setup
    buffer->jobState = RADIO_JOB_STATE_IN_SETUP;
    __enable_irq();

    buffer->channel = channel;
    // Fill buffer (dirty memcpy)
    for (uint8_t i = 0; i <= frame->header.lenght; i++)
    {
        buffer->outboundFrame.raw[i] = frame->raw[i];
    }
    buffer->retrysLeft.csma = s_numMaxCsmaBackoffs;
    buffer->retrysLeft.transmission = s_numMaxTransmissionRetrys;
    

    // If Radio is in an Idle State, send immediately
    __disable_irq();
    buffer->jobState = RADIO_JOB_STATE_TX_READY;
    
    if (
        sf_ringBufferGetCurrent()->jobState == RADIO_JOB_STATE_IDLE
        || sf_ringBufferGetCurrent()->jobState == RADIO_JOB_STATE_RX_IDLE
        || sf_ringBufferGetCurrent()->jobState > MARKER_RADIO_JOB_STATES_BEGINN_OTHER_DONE
    ){
        samr21RadioChangeState(RADIO_STATE_TX, buffer->channel);
        // Make prepared Buffer the active One
        sf_ringBufferMoveForward();
        __enable_irq();

        // Jump Start Statemachnine
        samr21RadioFsmHandleEvent(RADIO_SOFTEVENT_START_TX);
        return true;
    }

    __enable_irq();
    return true;
}

bool samr21RadioReceive(uint8_t channel)
{
    __disable_irq();

    if(
        sf_ringBufferGetCurrent()->channel == channel
        && sf_ringBufferGetCurrent()->jobState > MARKER_RADIO_JOB_STATES_BEGINN_RX
        && sf_ringBufferGetCurrent()->jobState < MARKER_RADIO_JOB_STATES_END_RX
    ){
        __enable_irq();
        return true;
    }

    // Get the next avivable Buffer
    JobBuffer_t *buffer = sf_ringBufferGetNext();

    //Check if Last Queued Transmission did even start yet
    if(buffer->jobState == RADIO_JOB_STATE_TX_READY){
        __enable_irq();
        return false;
    }

    buffer->jobState = RADIO_JOB_STATE_RX_IDLE;
    
    if (
        sf_ringBufferGetCurrent()->jobState == RADIO_JOB_STATE_IDLE
        || sf_ringBufferGetCurrent()->jobState == RADIO_JOB_STATE_RX_IDLE
        || sf_ringBufferGetCurrent()->jobState > MARKER_RADIO_JOB_STATES_BEGINN_OTHER_DONE
    ){
        samr21RadioChangeState(RADIO_STATE_RX, buffer->channel);
        // Make prepared Buffer the active One
        sf_ringBufferMoveForward();

        __enable_irq();
        return true;
    }

    __enable_irq();
    return true;
}

bool samr21RadioStartEnergyDetection(uint8_t channel, uint16_t duration)
{
    // Get the next avivable Buffer
    __disable_irq();
    JobBuffer_t *buffer = sf_ringBufferGetNext();

    //Check if Last Queued Transmission did even start yet
    if(buffer->jobState == RADIO_JOB_STATE_TX_READY){
        __enable_irq();
        return false;
    }

    // mark buffer 
    buffer->jobState = RADIO_JOB_STATE_ED_READY;
    buffer->edTimeleft = duration;
    buffer->measuredEngeryLevel = 0;
    buffer->channel = channel;

    // If Radio is in an Idle State, start immediately
    if (
        sf_ringBufferGetCurrent()->jobState == RADIO_JOB_STATE_IDLE
        || sf_ringBufferGetCurrent()->jobState == RADIO_JOB_STATE_RX_IDLE
        || sf_ringBufferGetCurrent()->jobState > MARKER_RADIO_JOB_STATES_BEGINN_OTHER_DONE
    ){
        samr21RadioChangeState(RADIO_STATE_RX, buffer->channel);
        // Make prepared Buffer the active One
        sf_ringBufferMoveForward();

        __enable_irq();
        // Jump Start Statemachnine
        samr21RadioFsmHandleEvent(RADIO_SOFTEVENT_START_ED);
        return true;
    }

    __enable_irq();
    return true;
}


JobBuffer_t* samr21RadioGetNextFinishedJobBuffer(){
    for (uint8_t i = 0; i < NUM_RADIO_JOB_BUFFER; i++){
        if(s_ringTransmissionBuffer.bufferData[i].jobState > MARKER_RADIO_JOB_STATES_BEGINN_OTHER_DONE){
            return &(s_ringTransmissionBuffer.bufferData[i]);
        }
    }
    return NULL;
}


bool samr21RadioAddShortAddrToPendingFrameTable(uint8_t* shortAddr){
    if(s_framePendingTableShortAddr.size >= SIZE_TABLE_FRAME_PENDING_SHORT_ADDR){
        //No Space left
        return false;
    }

    ((uint8_t*) &(s_framePendingTableShortAddr.addr[s_framePendingTableShortAddr.size]))[0] = shortAddr[0];
    ((uint8_t*) &(s_framePendingTableShortAddr.addr[s_framePendingTableShortAddr.size++]))[1] = shortAddr[1];
    return true;
} 

bool samr21RadioFindShortAddrInPendingFrameTable(uint8_t* shortAddr, bool remove){
    for (uint8_t i = 0; i < s_framePendingTableShortAddr.size; i++)
    {
        if(((uint8_t*) &(s_framePendingTableShortAddr.addr[i]))[0] != shortAddr[0]){
            continue;
        }

        if(((uint8_t*) &(s_framePendingTableShortAddr.addr[i]))[1] != shortAddr[1]){
            if(remove){
                s_framePendingTableShortAddr.addr[i] = s_framePendingTableShortAddr.addr[s_framePendingTableShortAddr.size--];
            }   
            return true;
        }
    }
    //Address not found, can't be removed
    return false;
} 

void samr21RadioClearShortAddrPendingFrameTable(){
    s_framePendingTableShortAddr.size = 0;
} 

bool samr21RadioAddIeeeAddrToPendingFrameTable(uint8_t * ieeeAddr){
    if(s_framePendingTableIeeeAddr.size >= SIZE_TABLE_FRAME_PENDING_IEEE_ADDR){
        //No Space left
        return false;
    }

    for (uint8_t i = 0; i < sizeof(uint64_t); i++)
    {
        ((uint8_t*) &(s_framePendingTableIeeeAddr.addr[s_framePendingTableIeeeAddr.size]))[i] = ieeeAddr[i];
    }
    s_framePendingTableIeeeAddr.size++;
    return true;
} 

bool samr21RadioFindIeeeAddrInPendingFrameTable(uint8_t* ieeeAddr, bool remove){
    uint8_t i = 0;
    while(i < s_framePendingTableIeeeAddr.size)
    {
        for (uint8_t j = 0; j < (sizeof(uint64_t)); j++)
        {
            if(((uint8_t*) &(s_framePendingTableIeeeAddr.addr[i]))[j] != ieeeAddr[j]){
                goto nextEntry;
            }
        }

        if(remove){
            s_framePendingTableIeeeAddr.addr[i] = s_framePendingTableIeeeAddr.addr[s_framePendingTableIeeeAddr.size--];
        }
        return true;

nextEntry:
        i++;
    }

    //Address not found
    return false;
} 

void samr21RadioClearIeeeAddrPendingFrameTable(){
    s_framePendingTableIeeeAddr.size = 0;
} 

/*------------------*/
// State Machine Transition Functions
/*------------------*/
void fsm_func_samr21RadioStartCCA()
{
    // Start TimeoutTimer
    if(sf_ringBufferGetCurrent()->jobState == RADIO_JOB_STATE_TX_READY){
        samr21Timer5Set(s_transmissionTimeout_us);
    }

    // Check if Transciver is in recive state
    if (g_trxStatus.bit.trxStatus != TRX_STATUS_RX_ON)
    {
        samr21TrxWriteRegister(TRX_STATE_REG, TRX_CMD_RX_ON);
        while (g_trxStatus.bit.trxStatus != TRX_STATUS_RX_ON){
            samr21TrxUpdateStatus();
        }
    }

    // Prepare CCA Measurment
    s_phyCcCcaReg.bit.ccaRequest = 1;

    // Start CCA Measurment
    samr21TrxWriteRegister(PHY_CC_CCA_REG, s_phyCcCcaReg.reg);

    // Reset local copy of ccaRequest Bit
    s_phyCcCcaReg.bit.ccaRequest = 0;
}

void fsm_func_samr21StartEd()
{   
    // Check if Transciver is in recive state
    if (g_trxStatus.bit.trxStatus != TRX_STATUS_RX_ON)
    {
        samr21TrxWriteRegister(TRX_STATE_REG, TRX_CMD_RX_ON);
        while (g_trxStatus.bit.trxStatus != TRX_STATUS_RX_ON){
            samr21TrxUpdateStatus();
        }
    }

    // Prepare CCA Measurment
    s_phyCcCcaReg.bit.ccaRequest = 1;

    // Start CCA Measurment
    samr21TrxWriteRegister(PHY_CC_CCA_REG, s_phyCcCcaReg.reg);

    // Reset local copy of ccaRequest Bit
    s_phyCcCcaReg.bit.ccaRequest = 0;

    //Queue the next ED allrdy (once per ms)
    samr21Timer4Set(1000);
}

void fsm_func_samr21RadioEvalCCA()
{
    if (g_trxStatus.bit.ccaStatus)
    {
        samr21RadioFsmQueueSoftEvent(RADIO_SOFTEVENT_CHANNEL_CLEAR);
        return;
    }

    if(sf_ringBufferGetCurrent()->retrysLeft.csma--){
        samr21RadioFsmQueueSoftEvent(RADIO_SOFTEVENT_CHANNEL_BUSY);
        return;
    }

    samr21RadioFsmQueueSoftEvent(RADIO_SOFTEVENT_NO_RETRYS_LEFT);
    return; 
}

void fsm_func_samr21RadioStartBackoffTimer()
{
    samr21Timer4Set((s_csmaBackoffIntervallStep_us * samr21RadioGetRandomNibble()) + s_csmaBackoffIntervallMin_us);
}

void fsm_func_samr21RadioSendTXPayload()
{
    samr21TrxWriteRegister(TRX_STATE_REG, TRX_CMD_FORCE_PLL_ON);
        while (g_trxStatus.bit.trxStatus != TRX_STATUS_PLL_ON){
            samr21TrxUpdateStatus();
        }

    __disable_irq();
    // Start Trasmission
    samr21TrxSetSLP_TR(true);

    // add tx timestamp
    sf_ringBufferGetCurrent()->txTimestamp = samr21RtcGetTimestamp();

    // Enable SPI Slave Select, to start uploading the Frame Payload in parallel while SHR is still being send

    samr21TrxSetSSel(true);
#ifdef __CONSERVATIVE_TRX_SPI_TIMING__
    samr21delaySysTick(CPU_WAIT_CYCLE_AFTER_SSEL_LOW);
#endif

    // Transmission should have started here allrdy so trigger can be disabled
    samr21TrxSetSLP_TR(false);

    // Send Write Frame Buffer Command and get Status Byte (see r2 datasheet 35.4 Radio Transceiver Status Information)
    g_trxStatus.reg = samr21TrxSpiTransceiveByteRaw(AT86RF233_CMD_FRAMEBUFFER_WRITE);

    if (g_trxStatus.bit.trxStatus != TRX_STATUS_BUSY_TX)
    {
        samr21TrxSetSSel(false);
        samr21RadioFsmQueueSoftEvent(RADIO_EVENT_ERROR);
        return;
    }

    // Leave the Last 2 Bytes empty cause CRC is generated by at86rf233
    for (int16_t i = 0; i <= (sf_ringBufferGetCurrent()->outboundFrame.header.lenght - IEEE_802_15_4_CRC_SIZE); i++)
    {
#ifdef __CONSERVATIVE_TRX_SPI_TIMING__
        samr21delaySysTick(CPU_WAIT_CYCLE_BETWEEN_BYTES);
#endif    
        samr21TrxSpiTransceiveByteRaw(sf_ringBufferGetCurrent()->outboundFrame.raw[i]);
    }

    // Disable Slave Select
#ifdef __CONSERVATIVE_TRX_SPI_TIMING__
    samr21delaySysTick(CPU_WAIT_CYCLE_BEFORE_SSEL_HIGH);
#endif
    samr21TrxSetSSel(false);
    __enable_irq();

    if (sf_ringBufferGetCurrent()->outboundFrame.header.frameControlField1.ackRequest)
    {
        samr21RadioFsmQueueSoftEvent(RADIO_SOFTEVENT_ACK_REQUESTED);
    }

    // Queue Move to RX
    samr21TrxWriteRegister(TRX_STATE_REG, TRX_CMD_RX_ON);
}

void fsm_func_samr21RadioJobCleanup()
{
    __disable_irq();
    // Stop Pending Timer
    samr21Timer4Stop();
    samr21Timer5Stop();

    // Look for a Queued Transmission
    if (sf_ringBufferGetNext()->jobState == RADIO_JOB_STATE_TX_READY)
    {
        samr21RadioChangeState(RADIO_STATE_TX, sf_ringBufferGetNext()->channel);
        // Make prepared Buffer the active One
        sf_ringBufferMoveForward();

        // Jump Start Next Statemachnine
        samr21RadioFsmQueueSoftEvent(RADIO_SOFTEVENT_START_TX);
        
        goto exit;
    }

    // Move to next buffer, Unless a TX Transmisssion is in setup
    if (sf_ringBufferGetNext()->jobState == RADIO_JOB_STATE_IN_SETUP){
        //Do nothing the samr21RadioSendFrame-func will jumpstart the statemachine once the setup is completed
        goto exit;
    }

    // Look for a Queued Energy Scans
    if (sf_ringBufferGetNext()->jobState == RADIO_JOB_STATE_ED_READY)
    {
        samr21RadioChangeState(RADIO_STATE_RX, sf_ringBufferGetNext()->channel);
        // Make prepared Buffer the active One
        sf_ringBufferMoveForward();

        // Jump Start Next Statemachnine
        samr21RadioFsmQueueSoftEvent(RADIO_SOFTEVENT_START_ED);
        
        goto exit;
    }

    // Move to next buffer, when in RX Mode so next frame can be recived
    if (sf_ringBufferGetCurrent()->jobState == RADIO_JOB_STATE_RX_DONE)
    {
        // Prepare the next Buffer
        sf_ringBufferGetNext()->jobState = RADIO_JOB_STATE_RX_IDLE;
        sf_ringBufferGetNext()->channel  = sf_ringBufferGetCurrent()->channel;
        // Move to Next Buffer
        sf_ringBufferMoveForward();
        goto exit;
    }

exit:
    __enable_irq();
    return;
}

void fsm_func_samr21RadioWaitForAck()
{
    samr21Timer4Set(s_ackMaxWaitDuration_us);
}

void fsm_func_samr21RadioAckReceptionStarted(){
    samr21Timer4Stop();
}

void fsm_func_samr21RadioEvalRetransmission()
{
    if(sf_ringBufferGetCurrent()->retrysLeft.transmission--){
        sf_ringBufferGetCurrent()->retrysLeft.csma = s_numMaxCsmaBackoffs;
        samr21RadioFsmQueueSoftEvent(RADIO_SOFTEVENT_START_TX);
        return;
    }

    samr21RadioFsmQueueSoftEvent(RADIO_SOFTEVENT_NO_RETRYS_LEFT);
    return;
}

void fsm_func_samr21RadioEvalAck()
{
    samr21Timer4Stop();

    __disable_irq();

    //get current JobBuffer
    JobBuffer_t* buffer = sf_ringBufferGetCurrent();

    // add rx timestamp
    buffer->rxTimestamp = samr21RtcGetTimestamp();

    // Enable SPI Slave Select
    samr21TrxSetSSel(true);
    samr21delaySysTick(CPU_WAIT_CYCLE_AFTER_SSEL_LOW);

    // Send Read Frame Buffer Command and get Status Byte (see r21 datasheet 35.4 Radio Transceiver Status Information)
    g_trxStatus.reg = samr21TrxSpiTransceiveByteRaw(AT86RF233_CMD_FRAMEBUFFER_READ);

    // First Byte is the msg Lenght
#ifdef __CONSERVATIVE_TRX_SPI_TIMING__
    samr21delaySysTick(CPU_WAIT_CYCLE_BETWEEN_BYTES);
#endif
    buffer->inboundFrame.header.lenght = samr21TrxSpiTransceiveByteRaw(SPI_DUMMY_BYTE);

    if(buffer->inboundFrame.header.lenght > IEEE_802_15_4_PDSU_SIZE){
        samr21TrxSetSSel(false);
        __enable_irq();
        goto ackInvalid;
    }

    // Download Recived Frame
    for (uint8_t i = 1; i <= (buffer->inboundFrame.header.lenght); i++)
    {
#ifdef __CONSERVATIVE_TRX_SPI_TIMING__
        samr21delaySysTick(CPU_WAIT_CYCLE_BETWEEN_BYTES);
#endif
        sf_ringBufferGetCurrent()->inboundFrame.raw[i] = samr21TrxSpiTransceiveByteRaw(SPI_DUMMY_BYTE);
    }

    // 3 Byte after the msg Frame are LQI,RSSI and CRC Informations (see r21 datasheet 35.3.2 Frame Buffer Access Mode)
#ifdef __CONSERVATIVE_TRX_SPI_TIMING__
    samr21delaySysTick(CPU_WAIT_CYCLE_BETWEEN_BYTES);
#endif
    buffer->rxLQI = samr21TrxSpiTransceiveByteRaw(SPI_DUMMY_BYTE);

#ifdef __CONSERVATIVE_TRX_SPI_TIMING__
    samr21delaySysTick(CPU_WAIT_CYCLE_BETWEEN_BYTES);
#endif
    buffer->rxRSSI = AT86RF233_RSSI_BASE_VAL + samr21TrxSpiTransceiveByteRaw(SPI_DUMMY_BYTE);

    AT86RF233_REG_RX_STATUS_t rxStatus = (AT86RF233_REG_RX_STATUS_t)samr21TrxSpiTransceiveByteRaw(SPI_DUMMY_BYTE);

    // Disable Slave Select
#ifdef __CONSERVATIVE_TRX_SPI_TIMING__
    samr21delaySysTick(CPU_WAIT_CYCLE_BEFORE_SSEL_HIGH);
#endif

    samr21TrxSetSSel(false);
    __enable_irq();

    if (!rxStatus.bit.crcValid)
    {
        goto ackInvalid;
    }

    if (buffer->inboundFrame.header.sequenceNumber != buffer->outboundFrame.header.sequenceNumber)
    {
        goto ackInvalid;
    }

ackValid:
    samr21RadioFsmQueueSoftEvent(RADIO_SOFTEVENT_ACK_VALID);
    return;

ackInvalid:
    if(sf_ringBufferGetCurrent()->retrysLeft.transmission--){
        sf_ringBufferGetCurrent()->retrysLeft.csma = s_numMaxCsmaBackoffs;
        samr21RadioFsmQueueSoftEvent(RADIO_SOFTEVENT_ACK_INVALID);
        return;
    }

    samr21RadioFsmQueueSoftEvent(RADIO_SOFTEVENT_NO_RETRYS_LEFT);
    return;
}

void fsm_func_samr21RadioTxAbort()
{
    __NOP();
}

void fsm_func_samr21RadioLiveRxParser()
{
    __disable_irq();

    // add rx timestamp
    JobBuffer_t * buffer = sf_ringBufferGetCurrent();
    buffer->rxTimestamp = samr21RtcGetTimestamp();

    // Enable SPI Slave Select
    samr21TrxSetSSel(true);
#ifdef __CONSERVATIVE_TRX_SPI_TIMING__
    samr21delaySysTick(CPU_WAIT_CYCLE_AFTER_SSEL_LOW);
#endif

    // Send Read Frame Buffer Command and get Status Byte (see r21 datasheet 35.4 Radio Transceiver Status Information)
    g_trxStatus.reg = samr21TrxSpiTransceiveByteRaw(AT86RF233_CMD_FRAMEBUFFER_READ);

    // First Byte is the msg Lenght
#ifdef __CONSERVATIVE_TRX_SPI_TIMING__
    samr21delaySysTick(CPU_WAIT_CYCLE_BETWEEN_BYTES);
#endif
    buffer->inboundFrame.header.lenght = samr21TrxSpiTransceiveByteRaw(SPI_DUMMY_BYTE);
    buffer->downloadedSize = 1;

    

    if(buffer->inboundFrame.header.lenght > IEEE_802_15_4_PDSU_SIZE){
        samr21RadioFsmQueueSoftEvent(RADIO_SOFTEVENT_MSG_INVALID);

#ifdef __CONSERVATIVE_TRX_SPI_TIMING__
        samr21delaySysTick(CPU_WAIT_CYCLE_BEFORE_SSEL_HIGH);
#endif
        samr21TrxSetSSel(false);
        __enable_irq();
        return;
    }

    // Download Recived Frame Till FCF
    while (buffer->downloadedSize <  4) //1Byte PhyHeader, 2Byte FCF, 1Byte Sequenz Number
    { 
        
        //(see r21 datasheet, 40.7 Frame Buffer Empty Indicator)
#ifdef __CONSERVATIVE_TRX_SPI_TIMING__
        samr21delaySysTick(CPU_WAIT_CYCLE_FOR_FRAME_BUFFER_EMPTY_FLAG);
#endif
        for(uint32_t timeout = 0; timeout < 0x005FFF; timeout++){
            if(!(PORT->Group[1].IN.reg & PORT_PB00)){
                break;
            }
        }
        //  while (PORT->Group[1].IN.reg & PORT_PB00);

        buffer->inboundFrame.raw[buffer->downloadedSize++] =
            samr21TrxSpiTransceiveByteRaw(SPI_DUMMY_BYTE);
    }

    //Skip Addr Filtering in Promiscuous Mode
    if(s_promiscuousMode){
        goto downloadRemainingData;
    }

    //Extract Address Postion Infromation from recived Frame FCS
    uint8_t 
        posSourcePanId, 
        posDestinationAddr,  
        posDestinationPanId, 
        posSourceAddr,
        posAddrHeaderTrail
    ;

    samr21RadioParserGetAddrPositions( 
        &(buffer->inboundFrame), 
        &posDestinationPanId,
        &posDestinationAddr,
        &posSourcePanId,
        &posSourceAddr,
        &posAddrHeaderTrail
    );

    // Download The Address Information data of the Frame 
    while (buffer->downloadedSize < posAddrHeaderTrail)
    {   
#ifdef __CONSERVATIVE_TRX_SPI_TIMING__
        samr21delaySysTick(CPU_WAIT_CYCLE_FOR_FRAME_BUFFER_EMPTY_FLAG);
#endif
        //(see r21 datasheet, 40.7 Frame Buffer Empty Indicator)
        for(uint32_t timeout = 0; timeout < 0x005FFF; timeout++){
            if(!(PORT->Group[1].IN.reg & PORT_PB00)){
                break;
            }
        }
        //  while (PORT->Group[1].IN.reg & PORT_PB00);

        buffer->inboundFrame.raw[buffer->downloadedSize++] =
            samr21TrxSpiTransceiveByteRaw(SPI_DUMMY_BYTE);
    }



    if ((buffer->inboundFrame.header.frameControlField2.destinationAddressingMode == IEEE_802_15_4_ADDR_SHORT))
    {
        if(!samr21RadioFilterPanId(&(buffer->inboundFrame.raw[posDestinationPanId]))){
            samr21RadioFsmQueueSoftEvent(RADIO_SOFTEVENT_MSG_INVALID);

#ifdef __CONSERVATIVE_TRX_SPI_TIMING__
            samr21delaySysTick(CPU_WAIT_CYCLE_BEFORE_SSEL_HIGH);
#endif
            samr21TrxSetSSel(false);
            __enable_irq();
            return;
        }
checkShortAddr:
        if(!samr21RadioFilterShortAddr(&(buffer->inboundFrame.raw[posDestinationAddr]))){
            samr21RadioFsmQueueSoftEvent(RADIO_SOFTEVENT_MSG_INVALID);

#ifdef __CONSERVATIVE_TRX_SPI_TIMING__
            samr21delaySysTick(CPU_WAIT_CYCLE_BEFORE_SSEL_HIGH);
#endif
            samr21TrxSetSSel(false);
            __enable_irq();
            return;
        }
    }

    if (buffer->inboundFrame.header.frameControlField2.destinationAddressingMode == IEEE_802_15_4_ADDR_IEEE)
    {   
        if(!samr21RadioFilterIeeeAddr(&(buffer->inboundFrame.raw[posDestinationAddr]))){
            samr21RadioFsmQueueSoftEvent(RADIO_SOFTEVENT_MSG_INVALID);

#ifdef __CONSERVATIVE_TRX_SPI_TIMING__
            samr21delaySysTick(CPU_WAIT_CYCLE_BEFORE_SSEL_HIGH);
#endif
            samr21TrxSetSSel(false);
            __enable_irq();
            return;
        }
    }

    if (
        buffer->inboundFrame.header.frameControlField2.destinationAddressingMode == IEEE_802_15_4_ADDR_NONE
        &&buffer->inboundFrame.header.frameControlField2.destinationAddressingMode != IEEE_802_15_4_ADDR_NONE
        &&buffer->inboundFrame.header.frameControlField1.panIdCompression
    ){   
        if(!samr21RadioFilterIeeeAddr(&(buffer->inboundFrame.raw[posDestinationPanId]))){
            samr21RadioFsmQueueSoftEvent(RADIO_SOFTEVENT_MSG_INVALID);

#ifdef __CONSERVATIVE_TRX_SPI_TIMING__
            samr21delaySysTick(CPU_WAIT_CYCLE_BEFORE_SSEL_HIGH);
#endif
            samr21TrxSetSSel(false);
            __enable_irq();
            return;
        }
    }

    if (buffer->inboundFrame.header.frameControlField2.sourceAddressingMode == IEEE_802_15_4_ADDR_SHORT)
    {
        buffer->outboundFrame.header.frameControlField1.framePending = 
            samr21RadioFindShortAddrInPendingFrameTable(&(buffer->inboundFrame.raw[posSourceAddr]), false)
        ;
    }

    if (buffer->inboundFrame.header.frameControlField2.sourceAddressingMode == IEEE_802_15_4_ADDR_IEEE)
    {
        buffer->outboundFrame.header.frameControlField1.framePending = 
            samr21RadioFindIeeeAddrInPendingFrameTable(&(buffer->inboundFrame.raw[posSourceAddr]), false)
        ;
    }


downloadRemainingData:
    // Download The Remaining data of the Frame 
    while (buffer->downloadedSize <= buffer->inboundFrame.header.lenght)
    {
#ifdef __CONSERVATIVE_TRX_SPI_TIMING__
        samr21delaySysTick(CPU_WAIT_CYCLE_FOR_FRAME_BUFFER_EMPTY_FLAG);
#endif
        //(see r21 datasheet, 40.7 Frame Buffer Empty Indicator)
        for(uint32_t timeout = 0; timeout < 0x005FFF; timeout++){
            if(!(PORT->Group[1].IN.reg & PORT_PB00)){
                break;
            }
        }
        //while (PORT->Group[1].IN.reg & PORT_PB00);

        buffer->inboundFrame.raw[buffer->downloadedSize++] =
            samr21TrxSpiTransceiveByteRaw(SPI_DUMMY_BYTE)
        ;
    }

    // 3 Byte after the msg Frame are LQI,RSSI and CRC Informations (see r21 datasheet 35.3.2 Frame Buffer Access Mode)
#ifdef __CONSERVATIVE_TRX_SPI_TIMING__
    samr21delaySysTick(CPU_WAIT_CYCLE_BETWEEN_BYTES);
#endif
    buffer->rxLQI = samr21TrxSpiTransceiveByteRaw(SPI_DUMMY_BYTE);

#ifdef __CONSERVATIVE_TRX_SPI_TIMING__
    samr21delaySysTick(CPU_WAIT_CYCLE_BETWEEN_BYTES);
#endif
    buffer->rxRSSI = AT86RF233_RSSI_BASE_VAL + samr21TrxSpiTransceiveByteRaw(SPI_DUMMY_BYTE);

    AT86RF233_REG_RX_STATUS_t rxStatus = (AT86RF233_REG_RX_STATUS_t)samr21TrxSpiTransceiveByteRaw(SPI_DUMMY_BYTE);

    // Disable Slave Select
#ifdef __CONSERVATIVE_TRX_SPI_TIMING__
    samr21delaySysTick(CPU_WAIT_CYCLE_BEFORE_SSEL_HIGH);
#endif
    samr21TrxSetSSel(false);
    __enable_irq();

    //allrdy prep the trx to send ack (timing is critical)
    samr21TrxWriteRegister(TRX_STATE_REG, TRX_CMD_FORCE_PLL_ON);


    if (!rxStatus.bit.crcValid)
    {
        //Abort sending Ack
        samr21TrxWriteRegister(TRX_STATE_REG, TRX_CMD_RX_ON);
        samr21RadioFsmQueueSoftEvent(RADIO_SOFTEVENT_MSG_INVALID);
        return;
    }

    // Check if Ack is needed
    if (sf_ringBufferGetCurrent()->inboundFrame.header.frameControlField1.ackRequest && !s_promiscuousMode)
    {
        samr21RadioFsmQueueSoftEvent(RADIO_SOFTEVENT_ACK_REQUESTED);
        return;
    }

    // No Ack needed 
    // Abort preped trx, cause no Ack needs to be sended
    samr21TrxWriteRegister(TRX_STATE_REG, TRX_CMD_RX_ON);
    samr21RadioFsmQueueSoftEvent(RADIO_SOFTEVENT_MSG_VALID);
    return;
}

void fsm_func_samr21RadioSendAck()
{
    while (g_trxStatus.bit.trxStatus != TRX_STATUS_PLL_ON)
    {
        samr21TrxUpdateStatus();
    }
    __disable_irq();
    
    // Start Trasmission
    samr21TrxSetSLP_TR(true);
    // add tx timestamp
    JobBuffer_t * buffer = sf_ringBufferGetCurrent();

    buffer->txTimestamp = samr21RtcGetTimestamp();

    // Enable SPI Slave Select, to start uploading the Frame Payload in parallel while SHR is still being send
    samr21TrxSetSSel(true);
#ifdef __CONSERVATIVE_TRX_SPI_TIMING__
    samr21delaySysTick(CPU_WAIT_CYCLE_AFTER_SSEL_LOW);
#endif
    // Transmission should have started here allrdy so trigger can be disabled
    samr21TrxSetSLP_TR(false);

    // Send Write Frame Buffer Command and get Status Byte (see r2 datasheet 35.4 Radio Transceiver Status Information)
    g_trxStatus.reg = samr21TrxSpiTransceiveByteRaw(AT86RF233_CMD_FRAMEBUFFER_WRITE);

    if (g_trxStatus.bit.trxStatus != TRX_STATUS_BUSY_TX)
    {
        
        samr21TrxSetSSel(false);
        samr21delaySysTick(CPU_WAIT_CYCLE_AFTER_SSEL_LOW);

        //go Back to RX
        samr21TrxWriteRegister(TRX_STATE_REG, TRX_CMD_RX_ON);
        samr21RadioFsmQueueSoftEvent(RADIO_EVENT_ERROR);

        __enable_irq();
        
        return;
    }

    //Preapre The Ack Package
    buffer->outboundFrame.header.sequenceNumber = 
        buffer->inboundFrame.header.sequenceNumber;
    buffer->outboundFrame.header.frameControlField1.ackRequest = 0;
    buffer->outboundFrame.header.frameControlField1.securityEnabled = 0;
    buffer->outboundFrame.header.frameControlField1.panIdCompression = 1;
    //buffer->outboundFrame.header.frameControlField1.framePending... is Set in function: samr21RadioLiveRxParser

    buffer->outboundFrame.header.frameControlField2.destinationAddressingMode = IEEE_802_15_4_ADDR_NONE;
    buffer->outboundFrame.header.frameControlField2.sourceAddressingMode = IEEE_802_15_4_ADDR_NONE;
    buffer->outboundFrame.header.frameControlField2.informationElementsPresent = 0;
    buffer->outboundFrame.header.frameControlField2.sequenceNumberSuppression = 0;
    buffer->outboundFrame.header.frameControlField2.frameVersion = IEEE_802_15_4_VERSION_2006;
    buffer->outboundFrame.header.lenght = 5;

    // Leave the Last 2 Bytes empty cause CRC is generated by the at86rf233
    for (int16_t i = 0; i <= (sf_ringBufferGetCurrent()->outboundFrame.header.lenght - IEEE_802_15_4_CRC_SIZE); i++)
    {
        samr21delaySysTick(CPU_WAIT_CYCLE_BETWEEN_BYTES);
        samr21TrxSpiTransceiveByteRaw(sf_ringBufferGetCurrent()->outboundFrame.raw[i]);
    }

    // Disable Slave Select
#ifdef __CONSERVATIVE_TRX_SPI_TIMING__
    samr21delaySysTick(CPU_WAIT_CYCLE_BEFORE_SSEL_HIGH);
#endif
    samr21TrxSetSSel(false);

    //Romve the framePending Entry after Ack transmission
    if(buffer->outboundFrame.header.frameControlField1.framePending){

        uint8_t 
            posSourcePanId, 
            posDestinationAddr,  
            posDestinationPanId, 
            posSourceAddr,
            posAddrHeaderTrail
        ;

        samr21RadioParserGetAddrPositions( 
            &(buffer->inboundFrame), 
            &posDestinationPanId,
            &posDestinationAddr,
            &posSourcePanId,
            &posSourceAddr,
            &posAddrHeaderTrail
        );

        if (buffer->inboundFrame.header.frameControlField2.sourceAddressingMode == IEEE_802_15_4_ADDR_SHORT)
        {
            samr21RadioFindShortAddrInPendingFrameTable(&(buffer->inboundFrame.raw[posSourceAddr]), true);
        }

        if (buffer->inboundFrame.header.frameControlField2.sourceAddressingMode == IEEE_802_15_4_ADDR_IEEE)
        {
            samr21RadioFindIeeeAddrInPendingFrameTable(&(buffer->inboundFrame.raw[posSourceAddr]), true);
        }
    }
    
    __enable_irq();

    // Queue Move to RX
    samr21TrxWriteRegister(TRX_STATE_REG, TRX_CMD_RX_ON);
}

void fsm_func_samr21RadioAbortLiveRxParser()
{
    __NOP();
}

void fsm_func_samr21EvalEd()
{   
    uint8_t edReading = samr21TrxReadRegister(PHY_ED_LEVEL_REG);

    if(sf_ringBufferGetCurrent()->measuredEngeryLevel < edReading){
        sf_ringBufferGetCurrent()->measuredEngeryLevel = edReading;
    }
}

void fsm_func_samr21EvalEdContinuation()
{   
    if (--(sf_ringBufferGetCurrent()->edTimeleft))
    {
        samr21RadioFsmQueueSoftEvent(RADIO_SOFTEVENT_START_ED);
        return;
    }

    samr21RadioFsmQueueSoftEvent(RADIO_SOFTEVENT_STOP_ED);
    return;
}


void samr21RadioParserGetAddrPositions(
    FrameBuffer_t* frame,
    uint8_t* posDestinationPanId,
    uint8_t* posDestinationAddr,
    uint8_t* posSourcePanId,
    uint8_t* posSourceAddr,
    uint8_t* posAddrHeaderTrail
){ 
    *posAddrHeaderTrail = 4; // 1Byte PhyHeader, 2Byte FCF, 1Byte Sequenz Number

    
parseDestinationPanId:
    //(TABLE 7.2 IEEE 802.15.4-2015) 
    if(frame->header.frameControlField2.destinationAddressingMode == IEEE_802_15_4_ADDR_SHORT){
        if(
            frame->header.frameControlField2.sourceAddressingMode == IEEE_802_15_4_ADDR_NONE
            && frame->header.frameControlField1.panIdCompression
        ){
            //IEEE 802.15.4-2015 TABLE 7.2 column4
            *posDestinationPanId = 0;
            goto parseDestinationAddr; 
        }
        *posDestinationPanId = *posAddrHeaderTrail;
        *posAddrHeaderTrail += sizeof(uint16_t);
        goto parseDestinationAddr; 
    }
  
    if(frame->header.frameControlField2.destinationAddressingMode == IEEE_802_15_4_ADDR_IEEE){
        if(
            frame->header.frameControlField2.sourceAddressingMode == IEEE_802_15_4_ADDR_NONE
            && frame->header.frameControlField1.panIdCompression
        ){
            //IEEE 802.15.4-2015 TABLE 7.2 column4
            *posDestinationPanId = 0;
            goto parseDestinationAddr; 
        }
        if(
            frame->header.frameControlField2.sourceAddressingMode == IEEE_802_15_4_ADDR_IEEE
            && frame->header.frameControlField1.panIdCompression
        ){
            //IEEE 802.15.4-2015 TABLE 7.2 column8
            *posDestinationPanId = 0;
            goto parseDestinationAddr; 
        }

        *posDestinationPanId = *posAddrHeaderTrail;
        *posAddrHeaderTrail += sizeof(uint16_t);
        goto parseDestinationAddr;
    }

    //Destination Addr Mode == None    
    if(
        frame->header.frameControlField2.sourceAddressingMode == IEEE_802_15_4_ADDR_NONE
        && frame->header.frameControlField1.panIdCompression
    ){
        //IEEE 802.15.4-2015 TABLE 7.2 column2
        *posDestinationPanId = *posAddrHeaderTrail;
        *posAddrHeaderTrail += sizeof(uint16_t);
        goto parseDestinationAddr; 
    }
    *posDestinationPanId = 0;

parseDestinationAddr:
    if(frame->header.frameControlField2.destinationAddressingMode == IEEE_802_15_4_ADDR_SHORT ){
        *posDestinationAddr = *posAddrHeaderTrail ;
        *posAddrHeaderTrail += sizeof(uint16_t);
        goto parseSourcePanId;
    }
    if(frame->header.frameControlField2.destinationAddressingMode == IEEE_802_15_4_ADDR_IEEE ){
        *posDestinationAddr = *posAddrHeaderTrail ;
        *posAddrHeaderTrail += sizeof(uint64_t);
        goto parseSourcePanId;
    }  
    *posDestinationAddr = 0;

parseSourcePanId:
    if(!(frame->header.frameControlField1.panIdCompression)){
        if(frame->header.frameControlField2.sourceAddressingMode == IEEE_802_15_4_ADDR_SHORT){
            posSourcePanId = *posAddrHeaderTrail;
            *posAddrHeaderTrail += sizeof(uint16_t);
            goto parseSourceAddr;
        }
        if( 
            frame->header.frameControlField2.sourceAddressingMode == IEEE_802_15_4_ADDR_IEEE
            && frame->header.frameControlField2.destinationAddressingMode != IEEE_802_15_4_ADDR_IEEE
        ){
            posSourcePanId = *posAddrHeaderTrail;
            *posAddrHeaderTrail += sizeof(uint16_t);
            goto parseSourceAddr;
        }
        posSourcePanId = 0;
    }


parseSourceAddr:
    if(frame->header.frameControlField2.sourceAddressingMode == IEEE_802_15_4_ADDR_SHORT ){
        *posSourceAddr = *posAddrHeaderTrail ;
        *posAddrHeaderTrail += sizeof(uint16_t);
        return;
    }
    if(frame->header.frameControlField2.sourceAddressingMode == IEEE_802_15_4_ADDR_IEEE ){
        *posSourceAddr = *posAddrHeaderTrail ;
        *posAddrHeaderTrail += sizeof(uint64_t);
        return;
    }  
    *posSourceAddr = 0;
    return;
}



//Filter Functions
bool samr21RadioFilterPanId(uint8_t * panID){
filterBroadcast:

    if(panID[0] < 0xfe){ 
        goto filterPanId;
    }
    if(panID[1] != 0xff){ 
        goto filterPanId;
    }
    return true;

filterPanId:
    if(panID[0] != ((uint8_t*) &s_panId)[0]){ 
        return false;
    }
    if(panID[1] != ((uint8_t*) &s_panId)[1]){ 
        return false;
    }
    return true;
}

bool samr21RadioFilterShortAddr(uint8_t * shortAddr){
filterBroadcast:
    if(shortAddr[0] != 0xff){ 
        goto filterShortAddr;
    }
    if(shortAddr[1] != 0xff){ 
        goto filterShortAddr;
    }
    return true;

filterShortAddr:
    if(shortAddr[0] != ((uint8_t*) &s_shortAddr)[0]){ 
        return false;
    }
    if(shortAddr[1] != ((uint8_t*) &s_shortAddr)[1]){ 
        return false;
    }
    return true;
}


bool samr21RadioFilterIeeeAddr(uint8_t * ieeeAddr){
filterBroadcast:
    for (uint8_t i = 0; i < sizeof(uint64_t); i++)
    {
        if(ieeeAddr[i] != 0xff){ 
            goto filterIeeeId;
        }
    }
    return true;

filterIeeeId:
    for (uint8_t i = 0; i < sizeof(uint16_t); i++)
    {
        if(ieeeAddr[i] != ((uint8_t*) &s_ieeeAddr)[i]){ //Little Endian Order
            return false;
        }
    }
    return true;
}

