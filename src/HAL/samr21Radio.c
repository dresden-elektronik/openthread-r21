// Author Eric Härtel @ dresden elektronik ingenieurtechnik gmbh © 2022

#include "samr21Radio.h"

// Ringbuffer for Transmissions
static struct RingTransmissionBuffer
{
    TransmissionBuffer_t bufferData[NUM_RADIO_BUFFER];
    uint8_t activeBufferIndex;
} s_ringTransmissionBuffer;

static TransmissionBuffer_t *sf_ringBufferGetCurrent()
{
    return &(s_ringTransmissionBuffer.bufferData[s_ringTransmissionBuffer.activeBufferIndex]);
};

static TransmissionBuffer_t *sf_ringBufferGetNext()
{
    return &(s_ringTransmissionBuffer.bufferData[(s_ringTransmissionBuffer.activeBufferIndex + 1) % NUM_RADIO_BUFFER]);
};

static void sf_ringBufferMoveForward()
{
    s_ringTransmissionBuffer.activeBufferIndex = (s_ringTransmissionBuffer.activeBufferIndex + 1) % NUM_RADIO_BUFFER;
};

//Frame Pending Table Short Addr
static struct framePendingTableShortAddr
{
    uint16_t addr[SIZE_TABLE_FRAME_PENDING_SHORT_ADDR];
    uint8_t size;
} s_framePendingTableShortAddr;

static bool sf_addShortAddrToPendingFrameTable(uint16_t shortAddr){
    if(s_framePendingTableShortAddr.size >= SIZE_TABLE_FRAME_PENDING_SHORT_ADDR){
        //No Space left
        return false;
    }

    s_framePendingTableShortAddr.addr[s_framePendingTableShortAddr.size++] = shortAddr;
    return true;
} 

static bool sf_findShortAddrInPendingFrameTable(uint16_t shortAddr, bool remove){
    for (uint8_t i = 0; i < SIZE_TABLE_FRAME_PENDING_SHORT_ADDR; i++)
    {
        if(s_framePendingTableShortAddr.addr[i] == shortAddr){

            if(remove){
                s_framePendingTableShortAddr.addr[i] = s_framePendingTableShortAddr.addr[s_framePendingTableShortAddr.size--];
            }   
            return true;
        }
    }
    //Address not found, can't be removed
    return false;
} 


//Frame Pending Table IEEE Addr
static struct framePendingTableIeeeAddr
{
    uint64_t addr[SIZE_TABLE_FRAME_PENDING_IEEE_ADDR];
    uint8_t size;
} s_framePendingTableIeeeAddr;

static bool sf_addIeeeAddrToPendingFrameTable(uint64_t ieeeAddr){
    if(s_framePendingTableIeeeAddr.size >= SIZE_TABLE_FRAME_PENDING_IEEE_ADDR){
        //No Space left
        return false;
    }
    s_framePendingTableIeeeAddr.addr[s_framePendingTableIeeeAddr.size++] = ieeeAddr;
    return true;
} 

static bool sf_findIeeeAddrInPendingFrameTable(uint64_t ieeeAddr, bool remove){

    //Split the addr into 2 Half Words, cause the M0+ is 32 Bit based, so time can be saved
    uint32_t addrLowerHalfWord = ((uint32_t*)(&ieeeAddr))[0];
    uint32_t addrUpperHalfWord = ((uint32_t*)(&ieeeAddr))[1];

    for (uint8_t i = 0; i < SIZE_TABLE_FRAME_PENDING_SHORT_ADDR; i++)
    {
        //only check lower Halfword first to safe some Time
        if(((uint32_t*)(&s_framePendingTableIeeeAddr.addr[i])[0]) != addrLowerHalfWord){
            continue;     
        }

        //if the lower Halfword matches, the upper Halfword is also checked
        if(((uint32_t*)(&s_framePendingTableIeeeAddr.addr[i])[1]) == addrUpperHalfWord){

            if(remove){
                s_framePendingTableIeeeAddr.addr[i] = s_framePendingTableIeeeAddr.addr[s_framePendingTableIeeeAddr.size--];
            }

            return true;
        }
    }

    //Address not found
    return false;
} 


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
        .bit.txPwr = 0x0
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

static uint32_t s_csmaBackoffIntervallMin_us    = 16;   // SYMBOL_DURATION_802_15_4_us*((2^minBE)-1) (default: minBE = 1)
static uint32_t s_csmaBackoffIntervallMax_us    = 240;  // SYMBOL_DURATION_802_15_4_us*((2^maxBE)-1) (default: maxBE = 4)
static uint32_t s_csmaBackoffIntervallDiff_us   = 224; // sCsmaBackoffIntervallMax - sCsmaBackoffIntervallMax
static uint32_t s_csmaBackoffIntervallStep_us   = 14;  // sCsmaBackoffIntervallDiff / 16, need cause at86rf233 can create 2 Random bits per SPI Read 

const static uint16_t s_ackMaxWaitDuration_us   = 250; // 12 * SYMBOL_DURATION_802_15_4_us
static uint16_t s_transmissionTimeout_us        = 10000;

static uint64_t s_ieeeAddr;
static uint16_t s_shortAddr;
static uint16_t s_panId;

void samr21RadioInit()
{
    // Get TRX into RX State
    samr21RadioChangeState(TRX_CMD_FORCE_TRX_OFF);
    while ((samr21TrxReadRegister(TRX_STATUS_REG) & TRX_STATUS_MASK) != TRX_STATUS_TRX_OFF)
        ;
    samr21RadioChangeState(TRX_CMD_RX_ON);
    while ((samr21TrxReadRegister(TRX_STATUS_REG) & TRX_STATUS_MASK) != TRX_STATUS_RX_ON)
        ;

    // Clear all pending IRQ by reading Register
    samr21TrxReadRegister(IRQ_STATUS_REG);
    // Setup Buffer
    s_ringTransmissionBuffer.activeBufferIndex = 0;

    for(uint8_t i = 0; i < NUM_RADIO_BUFFER; i++){
        s_ringTransmissionBuffer.bufferData->currentJobState = RADIO_STATE_IDLE;
    }
    
    samr21RadioFsmChangeJobStatePointer(&(sf_ringBufferGetCurrent()->currentJobState));

    // Enable IRQ via EIC
    // Reset first and wait for reset to finish
    EIC->CTRL.bit.SWRST = 1;
    while (EIC->STATUS.bit.SYNCBUSY)
        ;
    while (EIC->CTRL.bit.SWRST)
        ;

    // Enable EXINT[0]
    EIC->CONFIG[0].bit.FILTEN0 = 0;
    EIC->CONFIG[0].bit.SENSE0 = EIC_CONFIG_SENSE0_RISE_Val;
    EIC->INTENSET.bit.EXTINT0 = 1;

    // Enable Module
    EIC->CTRL.bit.ENABLE = 1;
    while (EIC->STATUS.bit.SYNCBUSY)
        ;
    while (!(EIC->CTRL.bit.ENABLE))
        ;

    EIC->INTFLAG.bit.EXTINT0 = 1;

    samr21TrxWriteRegister(TRX_CTRL_1_REG, s_trxCtrl1.reg);
    samr21TrxWriteRegister(PHY_TX_PWR_REG, s_phyTxPwr.reg);
    samr21TrxWriteRegister(IRQ_MASK_REG, s_irqMask.reg);

    // Read Once to clear pending Interrupts
    samr21TrxReadRegister(IRQ_STATUS_REG);

    // Enable in NVIC
    __NVIC_EnableIRQ(EIC_IRQn);
}

void samr21RadioChangeState(uint8_t newState)
{
    // Request new State
    samr21TrxWriteRegister(TRX_STATE_REG, newState);
}

void samr21RadioChangeTXPower(uint8_t txPower)
{
    AT86RF233_REG_PHY_TX_PWR_t temp;
    temp.bit.txPwr = txPower;
    samr21TrxWriteRegister(PHY_TX_PWR_REG, temp.reg);
}

uint8_t samr21RadioReadLastEDMeasurment()
{
    return samr21TrxReadRegister(PHY_ED_LEVEL_REG);
}

uint8_t samr21RadioGetRandom2Bit()
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

void samr21RadioSetShortAddr(uint16_t shortAddr)
{
    s_shortAddr = shortAddr;

    samr21TrxWriteRegister(SHORT_ADDR_0_REG, (s_shortAddr & 0x00FF));
    samr21TrxWriteRegister(SHORT_ADDR_1_REG, ((s_shortAddr & 0xFF00) >> 8));
}

void samr21RadioSetPanID(uint16_t panId)
{
    s_panId = panId;

    samr21TrxWriteRegister(PAN_ID_0_REG, (s_panId & 0x00FF));
    samr21TrxWriteRegister(PAN_ID_1_REG, ((s_panId & 0xFF00) >> 8));
}

void samr21RadioSetIEEEAddr(uint64_t ieeeAddr)
{
    s_ieeeAddr = ieeeAddr;

    for (uint8_t i = 0; i < 4; i++) // IEEE address has 8 bytes
    {
        samr21TrxWriteRegister(IEEE_ADDR_0_REG + i, (uint8_t *)(&s_ieeeAddr)[i]);
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

void samr21RadioChangeCCAThreshold(uint8_t threshold)
{
    AT86RF233_REG_CCA_THRES_t temp;
    temp.bit.ccaEdThres = threshold;
    samr21TrxWriteRegister(CCA_THRES_REG, temp.reg);
}

void samr21RadioChangeCSMABackoffExponent(uint8_t minBE, uint8_t maxBE)
{
    s_csmaBackoffIntervallMin_us = ((1 << minBE) - 1) * SYMBOL_DURATION_802_15_4_us;
    s_csmaBackoffIntervallMax_us = ((1 << maxBE) - 1) * SYMBOL_DURATION_802_15_4_us;
    s_csmaBackoffIntervallDiff_us = s_csmaBackoffIntervallMax_us - s_csmaBackoffIntervallMin_us;
}

void samr21RadioChangeNumBackoffsCSMA(uint8_t numBackoffs)
{
    s_numMaxCsmaBackoffs = numBackoffs;
}

void samr21RadioChangeNumTransmitRetrys(uint8_t numRetrys)
{
    s_numMaxTransmissionRetrys = numRetrys;
}

/*------------------*/
// Interface Functions
/*------------------*/
bool samr21RadioSendFrame(FrameBuffer_t *frame)
{
    // Get the next avivable Buffer
    __disable_irq();
    TransmissionBuffer_t *buffer = sf_ringBufferGetNext();

    //Check if Last Queued Transmission started yet
    if(buffer->currentJobState == RADIO_STATE_TX_READY){
        __enable_irq();
        return false;
    }
    PORT->Group[0].OUTSET.reg= PORT_PA09;

    // mark buffer as in Setup
    buffer->currentJobState = RADIO_STATE_IN_SETUP;
    __enable_irq();

    // Fill buffer (dirty memcpy)
    for (uint8_t i = 0; i <= frame->header.lenght; i++)
    {
        buffer->outboundFrame.raw[i] = frame->raw[i];
    }
    buffer->retrysLeft.csma = s_numMaxCsmaBackoffs;
    buffer->retrysLeft.transmission = s_numMaxTransmissionRetrys;
    

    // If Radio is in an Idle State, send immediately
    __disable_irq();
    buffer->currentJobState = RADIO_STATE_TX_READY;
    if (
        sf_ringBufferGetCurrent()->currentJobState == RADIO_STATE_IDLE
        || sf_ringBufferGetCurrent()->currentJobState > MARKER_RADIO_STATES_BEGINN_OTHER_DONE
    ){
        
        // Make prepared Buffer the active One
        sf_ringBufferMoveForward();

        // Change Active State Ptr in FSM
        samr21RadioFsmChangeJobStatePointer(&(buffer->currentJobState));
        __enable_irq();
       

        // Jump Start Statemachnine
        samr21RadioFsmHandleEvent(RADIO_SOFTEVENT_START_TX);
        return true;
    }

    __enable_irq();
    return true;
}

/*------------------*/
// State Machine Transition Functions
/*------------------*/
void fsm_func_samr21RadioStartCCA()
{
    PORT->Group[0].OUTCLR.reg= PORT_PA07;
    
    // Start TimeoutTimer
    if(sf_ringBufferGetCurrent()->currentJobState == RADIO_STATE_TX_READY){
        samr21Timer5Set(s_transmissionTimeout_us);
    }

    // Check if Transciver is in recive state
    if (g_trxStatus.bit.trxStatus != TRX_STATUS_RX_ON)
    {
        samr21RadioChangeState(TRX_CMD_RX_ON);
        while ((samr21TrxReadRegister(TRX_STATUS_REG) & TRX_STATUS_MASK) != TRX_STATUS_RX_ON)
            ;
    }

    // Prepare CCA Measurment
    s_phyCcCcaReg.bit.ccaRequest = 1;

    // Start CCA Measurment
    samr21TrxWriteRegister(PHY_CC_CCA_REG, s_phyCcCcaReg.reg);

    // Reset local copy of ccaRequest Bit
    s_phyCcCcaReg.bit.ccaRequest = 0;
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
    PORT->Group[0].OUTSET.reg= PORT_PA07;
    samr21Timer4Set((s_csmaBackoffIntervallStep_us * samr21RadioGetRandomNibble()) + s_csmaBackoffIntervallMin_us);
}

void fsm_func_samr21RadioSendTXPayload()
{

    samr21RadioChangeState(TRX_CMD_FORCE_PLL_ON);
    while (g_trxStatus.bit.trxStatus != TRX_STATUS_PLL_ON)
    {
        samr21TrxUpdateStatus();
    }

    __disable_irq();
    // Start Trasmission
    samr21TrxSetSLP_TR(true);
    PORT->Group[0].OUTSET.reg= PORT_PA06;

    // add tx timestamp
    sf_ringBufferGetCurrent()->txTimestamp = samr21RtcGetTimestamp();

    // Enable SPI Slave Select, to start uploading the Frame Payload in parallel while SHR is still being send

    samr21TrxSetSSel(true);

    samr21delayLoop(CPU_WAIT_CYCLE_AFTER_SSEL_LOW);

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
        samr21delayLoop(CPU_WAIT_CYCLE_BETWEEN_BYTES);
        samr21TrxSpiTransceiveByteRaw(sf_ringBufferGetCurrent()->outboundFrame.raw[i]);
    }

    // Disable Slave Select
    samr21delaySysTick(CPU_WAIT_CYCLE_BEFORE_SSEL_HIGH);
    samr21TrxSetSSel(false);
    __enable_irq();

    if (sf_ringBufferGetCurrent()->outboundFrame.header.frameControlField1.ackRequest)
    {
        samr21RadioFsmQueueSoftEvent(RADIO_SOFTEVENT_ACK_REQUESTED);
    }

    // Queue Move to RX
    samr21TrxWriteRegister(TRX_STATE_REG, TRX_CMD_RX_ON);
}

void fsm_func_samr21RadioTransmissionCleanup()
{
    __disable_irq();
    // Stop Pending Timer
    samr21Timer4Stop();
    samr21Timer5Stop();
    PORT->Group[0].OUTCLR.reg= PORT_PA09;

    // Look for a Queued Transmission
    if (sf_ringBufferGetNext()->currentJobState == RADIO_STATE_TX_READY)
    {
        PORT->Group[0].OUTSET.reg= PORT_PA09;
        // Make prepared Buffer the active One
        sf_ringBufferMoveForward();

        // Change Active State Ptr in FSM
        samr21RadioFsmChangeJobStatePointer(&(sf_ringBufferGetCurrent()->currentJobState));

        // Jump Start Next Statemachnine
        samr21RadioFsmQueueSoftEvent(RADIO_SOFTEVENT_START_TX);
        
        goto exit;
    }

    // Move to next buffer, Unless a TX Transmisssion is in setup
    if (sf_ringBufferGetNext()->currentJobState != RADIO_STATE_IN_SETUP)
    {
        // Reset next Buffer
        sf_ringBufferGetNext()->currentJobState = RADIO_STATE_IDLE;
        // Move to Next Buffer
        sf_ringBufferMoveForward();
        // Change Active State Ptr in FSM
        samr21RadioFsmChangeJobStatePointer(&(sf_ringBufferGetCurrent()->currentJobState));
        
        goto exit;
    }

exit:
    __enable_irq();
    return;
}

void fsm_func_samr21RadioWaitForAck()
{
    PORT->Group[0].OUTCLR.reg= PORT_PA06;
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

    // add rx timestamp
    sf_ringBufferGetCurrent()->rxTimestamp = samr21RtcGetTimestamp();

    // Enable SPI Slave Select
    samr21TrxSetSSel(true);
    samr21delayLoop(CPU_WAIT_CYCLE_AFTER_SSEL_LOW);

    // Send Read Frame Buffer Command and get Status Byte (see r21 datasheet 35.4 Radio Transceiver Status Information)
    g_trxStatus.reg = samr21TrxSpiTransceiveByteRaw(AT86RF233_CMD_FRAMEBUFFER_READ);

    // First Byte is the msg Lenght
    samr21delayLoop(CPU_WAIT_CYCLE_BETWEEN_BYTES);
    sf_ringBufferGetCurrent()->inboundFrame.header.lenght = samr21TrxSpiTransceiveByteRaw(SPI_DUMMY_BYTE);

    // Download Recived Frame
    for (uint8_t i = 1; i <= (sf_ringBufferGetCurrent()->inboundFrame.header.lenght); i++)
    {
        samr21delayLoop(CPU_WAIT_CYCLE_BETWEEN_BYTES);
        sf_ringBufferGetCurrent()->inboundFrame.raw[i] = samr21TrxSpiTransceiveByteRaw(SPI_DUMMY_BYTE);
    }

    // 3 Byte after the msg Frame are LQI,RSSI and CRC Informations (see r21 datasheet 35.3.2 Frame Buffer Access Mode)
    samr21delayLoop(CPU_WAIT_CYCLE_BETWEEN_BYTES);
    sf_ringBufferGetCurrent()->rxLQI = samr21TrxSpiTransceiveByteRaw(SPI_DUMMY_BYTE);

    samr21delayLoop(CPU_WAIT_CYCLE_BETWEEN_BYTES);
    sf_ringBufferGetCurrent()->rxRSSI = AT86RF233_RSSI_BASE_VAL + samr21TrxSpiTransceiveByteRaw(SPI_DUMMY_BYTE);

    AT86RF233_REG_RX_STATUS_t rxStatus = (AT86RF233_REG_RX_STATUS_t)samr21TrxSpiTransceiveByteRaw(SPI_DUMMY_BYTE);

    // Disable Slave Select
    samr21delaySysTick(CPU_WAIT_CYCLE_BEFORE_SSEL_HIGH);
    samr21TrxSetSSel(false);
    __enable_irq();

    if (!rxStatus.bit.crcValid)
    {
        goto ackInvalid;
    }

    if (sf_ringBufferGetCurrent()->inboundFrame.header.sequenceNumber != sf_ringBufferGetCurrent()->outboundFrame.header.sequenceNumber)
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
    PORT->Group[0].OUTSET.reg = PORT_PA15;

    // add rx timestamp
    TransmissionBuffer_t * buffer = sf_ringBufferGetCurrent();
    buffer->rxTimestamp = samr21RtcGetTimestamp();

    // Enable SPI Slave Select
    samr21TrxSetSSel(true);
    samr21delayLoop(CPU_WAIT_CYCLE_AFTER_SSEL_LOW);

    // Send Read Frame Buffer Command and get Status Byte (see r21 datasheet 35.4 Radio Transceiver Status Information)
    g_trxStatus.reg = samr21TrxSpiTransceiveByteRaw(AT86RF233_CMD_FRAMEBUFFER_READ);

    // First Byte is the msg Lenght
    samr21delayLoop(CPU_WAIT_CYCLE_BETWEEN_BYTES);
    buffer->inboundFrame.header.lenght = samr21TrxSpiTransceiveByteRaw(SPI_DUMMY_BYTE);
    buffer->downloadedSize = 1;

    PORT->Group[0].OUTSET.reg = PORT_PA16;
    // Download Recived Frame Till FCF
    while (buffer->downloadedSize <  4) //1Byte PhyHeader, 2Byte FCF, 1Byte Sequenz Number
    { 
        
        //(see r21 datasheet, 40.7 Frame Buffer Empty Indicator)
        samr21delayLoop(CPU_WAIT_CYCLE_FOR_FRAME_BUFFER_EMPTY_FLAG);
        for(uint32_t timeout = 0; timeout < 0x0FFFFF; timeout++){
            if(!(PORT->Group[1].IN.reg & PORT_PB00)){
                break;
            }
        }
        //  while (PORT->Group[1].IN.reg & PORT_PB00);

        buffer->inboundFrame.raw[buffer->downloadedSize++] =
            samr21TrxSpiTransceiveByteRaw(SPI_DUMMY_BYTE);
    }
    PORT->Group[0].OUTCLR.reg = PORT_PA16;

    //Extract Address Postion Infromation from recived Frame FCS
    uint8_t posSourceAddr, posDestinationAddr, posSourcePanId, posDestinationPanId;
    uint8_t curPosOffset = 4; // 1Byte PhyHeader, 2Byte FCF, 1Byte Sequenz Number

    //Destination PAN ID
    posDestinationPanId = curPosOffset;
    curPosOffset += sizeof(uint16_t);
    
    //Destination Addr
    if(buffer->inboundFrame.header.frameControlField2.destinationAddressingMode){
        posDestinationAddr = curPosOffset;
    }
    curPosOffset += 
        (( buffer->inboundFrame.header.frameControlField2.destinationAddressingMode & 0b10 ) ? sizeof(uint16_t) : 0 );
    curPosOffset += 
        (( buffer->inboundFrame.header.frameControlField2.destinationAddressingMode & 0b01 ) ? ( sizeof(uint64_t) - sizeof(uint16_t) ) : 0 );


    //Source PAN ID
    if(!(buffer->inboundFrame.header.frameControlField1.panIdCompression)){
        posSourceAddr = curPosOffset++;
    }
    //Source Addr
    if(buffer->inboundFrame.header.frameControlField2.sourceAddressingMode){
        posSourceAddr = curPosOffset;
    }
    curPosOffset += 
        (( buffer->inboundFrame.header.frameControlField2.sourceAddressingMode & 0b10 ) ? sizeof(uint16_t) : 0 );
    curPosOffset += 
        (( buffer->inboundFrame.header.frameControlField2.sourceAddressingMode & 0b01 ) ? ( sizeof(uint64_t) - sizeof(uint16_t) ) : 0 );

    PORT->Group[0].OUTSET.reg = PORT_PA16;
    // Download The Address Information data of the Frame 
    while (buffer->downloadedSize < curPosOffset)
    {
        //(see r21 datasheet, 40.7 Frame Buffer Empty Indicator)
        samr21delayLoop(CPU_WAIT_CYCLE_FOR_FRAME_BUFFER_EMPTY_FLAG);
        for(uint32_t timeout = 0; timeout < 0x0FFFFF; timeout++){
            if(!(PORT->Group[1].IN.reg & PORT_PB00)){
                break;
            }
        }
        //  while (PORT->Group[1].IN.reg & PORT_PB00);

        buffer->inboundFrame.raw[buffer->downloadedSize++] =
            samr21TrxSpiTransceiveByteRaw(SPI_DUMMY_BYTE);
    }
    PORT->Group[0].OUTCLR.reg = PORT_PA16;

    if ((buffer->inboundFrame.header.frameControlField2.destinationAddressingMode == IEEE_802_15_4_ADDR_SHORT))
    {
        for (uint8_t i = 0; i < sizeof(uint16_t); i++)
        {
            if(((uint8_t*) &s_panId)[i] != buffer->inboundFrame.raw[posDestinationPanId + i]){ //Little Endian Order
                samr21RadioFsmQueueSoftEvent(RADIO_SOFTEVENT_MSG_INVALID);
                samr21delaySysTick(CPU_WAIT_CYCLE_BEFORE_SSEL_HIGH);
                samr21TrxSetSSel(false);
                PORT->Group[0].OUTCLR.reg = PORT_PA15;
                __enable_irq();
                return;
            }
        } 


        for (uint8_t i = 0; i < sizeof(uint16_t); i++)
        {
            if(((uint8_t*) &s_shortAddr)[i] != buffer->inboundFrame.raw[posDestinationAddr + i]){ //Little Endian Order
                samr21RadioFsmQueueSoftEvent(RADIO_SOFTEVENT_MSG_INVALID);
                samr21delaySysTick(CPU_WAIT_CYCLE_BEFORE_SSEL_HIGH);
                samr21TrxSetSSel(false);
                PORT->Group[0].OUTCLR.reg = PORT_PA15;
                __enable_irq();
                return;
            }
        } 
    }

    if (buffer->inboundFrame.header.frameControlField2.destinationAddressingMode == IEEE_802_15_4_ADDR_IEEE)
    {
        for (uint8_t i = 0; i < sizeof(uint64_t); i++)
        {
            if(((uint8_t*) &s_ieeeAddr)[i] != buffer->inboundFrame.raw[posDestinationAddr + i]){ //Little Endian Order
                samr21RadioFsmQueueSoftEvent(RADIO_SOFTEVENT_MSG_INVALID);
                samr21delaySysTick(CPU_WAIT_CYCLE_BEFORE_SSEL_HIGH);
                samr21TrxSetSSel(false);
                PORT->Group[0].OUTCLR.reg = PORT_PA15;
                __enable_irq();
                return;
            }
        }   
    }

    if(buffer->inboundFrame.header.frameControlField2.sourceAddressingMode == IEEE_802_15_4_ADDR_SHORT){

        uint16_t shortAddr = buffer->inboundFrame.raw[posSourceAddr];//Little Endian Order
        shortAddr += (buffer->inboundFrame.raw[posSourceAddr + 1]) << 8;

        buffer->outboundFrame.header.frameControlField1.framePending = sf_findShortAddrInPendingFrameTable(shortAddr, false);

        //Does not work Beacause of no support for unaligned accesses on the Cortex-M0 processor.
        //(
            
            // sf_findShortAddrInPendingFrameTable(*((uint16_t *) &buffer->inboundFrame.raw[posSourceAddr]),  false ) ?
            // 1 : 0
        //);
    }

    if(buffer->inboundFrame.header.frameControlField2.sourceAddressingMode == IEEE_802_15_4_ADDR_IEEE){
        
        uint64_t ieeeAddr = buffer->inboundFrame.raw[posSourceAddr];//Little Endian Order
        ieeeAddr += (buffer->inboundFrame.raw[posSourceAddr+1]) << 8;
        ieeeAddr += (buffer->inboundFrame.raw[posSourceAddr+2]) << 16;
        ieeeAddr += (buffer->inboundFrame.raw[posSourceAddr+3]) << 24;
        ieeeAddr += (buffer->inboundFrame.raw[posSourceAddr+4]) << 32;
        ieeeAddr += (buffer->inboundFrame.raw[posSourceAddr+5]) << 40;
        ieeeAddr += (buffer->inboundFrame.raw[posSourceAddr+6]) << 48;
        ieeeAddr += (buffer->inboundFrame.raw[posSourceAddr+7]) << 56;
        
        buffer->outboundFrame.header.frameControlField1.framePending = sf_findIeeeAddrInPendingFrameTable(ieeeAddr, false);

        //Does not work Because of no support for unaligned accesses on the Cortex-M0 processor.
        // (
            //sf_findIeeeAddrInPendingFrameTable( *((uint64_t *) &buffer->inboundFrame.raw[posSourceAddr]), false ) ?
            //1 : 0
        // );
    }

    PORT->Group[0].OUTSET.reg = PORT_PA16;
    // Download The Remaining data of the Frame 
    while (buffer->downloadedSize <= buffer->inboundFrame.header.lenght)
    {
        samr21delayLoop(CPU_WAIT_CYCLE_FOR_FRAME_BUFFER_EMPTY_FLAG);

        //(see r21 datasheet, 40.7 Frame Buffer Empty Indicator)
        for(uint32_t timeout = 0; timeout < 0x0FFFFF; timeout++){
            if(!(PORT->Group[1].IN.reg & PORT_PB00)){
                break;
            }
        }
        //while (PORT->Group[1].IN.reg & PORT_PB00);

        buffer->inboundFrame.raw[buffer->downloadedSize++] =
            samr21TrxSpiTransceiveByteRaw(SPI_DUMMY_BYTE);
    }
    PORT->Group[0].OUTCLR.reg = PORT_PA16;

    // 3 Byte after the msg Frame are LQI,RSSI and CRC Informations (see r21 datasheet 35.3.2 Frame Buffer Access Mode)
    samr21delayLoop(CPU_WAIT_CYCLE_BETWEEN_BYTES);
    buffer->rxLQI = samr21TrxSpiTransceiveByteRaw(SPI_DUMMY_BYTE);

    samr21delayLoop(CPU_WAIT_CYCLE_BETWEEN_BYTES);
    buffer->rxRSSI = AT86RF233_RSSI_BASE_VAL + samr21TrxSpiTransceiveByteRaw(SPI_DUMMY_BYTE);

    AT86RF233_REG_RX_STATUS_t rxStatus = (AT86RF233_REG_RX_STATUS_t)samr21TrxSpiTransceiveByteRaw(SPI_DUMMY_BYTE);

    // Disable Slave Select
    samr21delaySysTick(CPU_WAIT_CYCLE_BEFORE_SSEL_HIGH);
    samr21TrxSetSSel(false);
    __enable_irq();

    samr21RadioChangeState(TRX_CMD_FORCE_PLL_ON);

    if (!rxStatus.bit.crcValid)
    {
        samr21RadioChangeState(TRX_CMD_RX_ON);
        samr21RadioFsmQueueSoftEvent(RADIO_SOFTEVENT_MSG_INVALID);
        PORT->Group[0].OUTCLR.reg = PORT_PA15;
        return;
    }

    // Check if Ack is needed
    if (!sf_ringBufferGetCurrent()->inboundFrame.header.frameControlField1.ackRequest)
    {
        samr21RadioChangeState(TRX_CMD_RX_ON);
        samr21RadioFsmQueueSoftEvent(RADIO_SOFTEVENT_MSG_VALID);
        PORT->Group[0].OUTCLR.reg = PORT_PA15;
        return;
    }

    // Prepre TRX

    samr21RadioFsmQueueSoftEvent(RADIO_SOFTEVENT_ACK_REQUESTED);
    PORT->Group[0].OUTCLR.reg = PORT_PA15;
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
    PORT->Group[0].OUTSET.reg = PORT_PA14;
    samr21TrxSetSLP_TR(true);
    // add tx timestamp
    TransmissionBuffer_t * buffer = sf_ringBufferGetCurrent();

    buffer->txTimestamp = samr21RtcGetTimestamp();

    // Enable SPI Slave Select, to start uploading the Frame Payload in parallel while SHR is still being send
    samr21TrxSetSSel(true);
    samr21delayLoop(CPU_WAIT_CYCLE_AFTER_SSEL_LOW);

    // Transmission should have started here allrdy so trigger can be disabled
    samr21TrxSetSLP_TR(false);

    // Send Write Frame Buffer Command and get Status Byte (see r2 datasheet 35.4 Radio Transceiver Status Information)
    g_trxStatus.reg = samr21TrxSpiTransceiveByteRaw(AT86RF233_CMD_FRAMEBUFFER_WRITE);

    if (g_trxStatus.bit.trxStatus != TRX_STATUS_BUSY_TX)
    {
        samr21delaySysTick(CPU_WAIT_CYCLE_BEFORE_SSEL_HIGH);
        samr21TrxSetSSel(false);

        samr21RadioChangeState(TRX_CMD_RX_ON);
        samr21RadioFsmQueueSoftEvent(RADIO_EVENT_ERROR);
        PORT->Group[0].OUTCLR.reg = PORT_PA14;
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

    // Leave the Last 2 Bytes empty cause CRC is generated by at86rf233
    for (int16_t i = 0; i <= (sf_ringBufferGetCurrent()->outboundFrame.header.lenght - IEEE_802_15_4_CRC_SIZE); i++)
    {
        samr21delayLoop(CPU_WAIT_CYCLE_BETWEEN_BYTES);
        samr21TrxSpiTransceiveByteRaw(sf_ringBufferGetCurrent()->outboundFrame.raw[i]);
    }

    // Disable Slave Select
    samr21delaySysTick(CPU_WAIT_CYCLE_BEFORE_SSEL_HIGH);
    samr21TrxSetSSel(false);
    __enable_irq();

    // Queue Move to RX
    samr21TrxWriteRegister(TRX_STATE_REG, TRX_CMD_RX_ON);
    PORT->Group[0].OUTCLR.reg = PORT_PA14;
}

void fsm_func_samr21RadioAbortLiveRxParser()
{
    __NOP();
}
