//Author Eric Härtel @ dresden elektronik ingenieurtechnik gmbh © 2022
#include "samr21RadioRxHandler.h"

static RxBuffer s_rxBuffer[NUM_SAMR21_RX_BUFFER];
static uint8_t  s_activeRxBuffer = 0;

volatile bool           s_rxSlottedActive = false;
uint32_t                s_slottedListeningDuration_us;
uint8_t                 s_slottedListeningChannel;

volatile bool                    s_rxAbort = false;
volatile bool                    s_rxHandlerActive = false;

static uint8_t          s_ackNonce[AES_BLOCK_SIZE];
static uint8_t          s_cbcBlock[AES_BLOCK_SIZE];

static uint8_t s_ackEncryptionMask[(IEEE_802_15_4_FRAME_SIZE / AES_BLOCK_SIZE)][AES_BLOCK_SIZE];


bool samr21RadioRxBusy(){
    return s_rxHandlerActive; 
}

bool samr21RadioRxAbort(){
    if(samr21RadioRxBusy()){
        s_rxAbort = true;

        while (s_rxHandlerActive);
        return true;
    }
    return false;
}


void samr21RadioRxResetBuffer(){
    for(uint16_t i = 0; i < NUM_SAMR21_RX_BUFFER; i++)
    {
        s_rxBuffer[i].status = RX_STATUS_IDLE;
        s_rxBuffer[i].otFrame.mPsdu = &s_rxBuffer[i].rxFramePsdu[1];
    }

    s_activeRxBuffer = 0;
}

RxBuffer* samr21RadioRxGetPendingRxBuffer(){
    for(uint16_t i = 0; i < NUM_SAMR21_RX_BUFFER; i++){
        if (s_rxBuffer[i].status >= RX_STATUS_DONE)
        {
            return &s_rxBuffer[i];
        }
    }
    return NULL;
}

bool samr21RadioRxSetup(uint8_t channel, uint32_t duration, uint32_t startTime)
{
    if(duration){
        s_slottedListeningChannel = channel;
        s_slottedListeningDuration_us = duration;
        samr21RtcSetWakeUpTimestamp(startTime);
        return;
    }

    samr21RadioRxStart(channel);
    return true;
}



void samr21RadioRxStart(uint8_t channel){

    //Set Timer for listening duration
    if(s_slottedListeningDuration_us){
        s_rxSlottedActive = true;
        samr21RtcSetWakeUpTimestamp(samr21RtcGetTimestamp() + s_slottedListeningDuration_us);
    }

    //Abort the current Transmisson (prevent retrys)
    samr21RadioTxAbort();
        

    //Check if theres already an ongoing Reception
    if(s_rxHandlerActive){
        if(channel == g_phyCcCcaReg.bit.channel){
            return; //TRX is allrdy setup for reception on the desired channel
        }
        samr21RadioRxAbort();
    }

    g_irqMask = (AT86RF233_REG_IRQ_MASK_t){
        .bit.pllLock = 0,
        .bit.pllUnlock = 0,
        .bit.rxStart = 1,
        .bit.trxEnd = 1,
        .bit.ccaEdDone = 0,
        .bit.addressMatch = 0,
        .bit.bufferUnderRun = 1,
        .bit.batteryLow = 0
    };
    samr21TrxWriteRegister(IRQ_MASK_REG, g_irqMask.reg);

    //Set Channel of TRX
    g_phyCcCcaReg.bit.channel = channel;
    samr21TrxWriteRegister(PHY_CC_CCA_REG, g_phyCcCcaReg.reg);

    //Prepare RX-Buffer
    if(s_rxBuffer[s_activeRxBuffer].status != RX_STATUS_IDLE)
    {
        __disable_irq();
        uint16_t nextBuffer = ( s_activeRxBuffer + 1 ) % NUM_SAMR21_RX_BUFFER;
        s_rxBuffer[nextBuffer].status = RX_STATUS_IDLE;
        s_activeRxBuffer = nextBuffer;
        __enable_irq();
    }
    
    // Check and enforce that the Transciver is in recive state
    if (g_trxStatus.bit.trxStatus != TRX_STATUS_RX_ON)
    {
        samr21TrxWriteRegister(TRX_STATE_REG, TRX_CMD_RX_ON);
        while (g_trxStatus.bit.trxStatus != TRX_STATUS_RX_ON){
            samr21TrxUpdateStatus();
        }
    }

    void samr21RadioSetEventHandler(&samr21RadioRxEventHandler);

    s_rxHandlerActive = true;
    s_rxAbort = false;
}

void samr21RadioRxReceptionStarted()
{
    s_rxBuffer[s_activeRxBuffer].status = RX_STATUS_RECIVING_FCF;
    s_rxBuffer[s_activeRxBuffer].otFrame.mInfo.mRxInfo.mTimestamp = 
        samr21RtcGesamr21RtcGetTimestamp()
    ;
    s_rxBuffer[s_activeRxBuffer].otFrame.mChannel = g_phyCcCcaReg.bit.channel;
    samr21Timer4Set(RX_BACKOFF_BEFORE_FIRST_FRAMEBUFFER_ACCESS);
}

void samr21RadioRxDownloadFCF()
{
    samr21TrxSpiStartAccess(AT86RF233_CMD_FRAMEBUFFER_READ, 0);

    RxBuffer * buffer = &s_rxBuffer[s_activeRxBuffer];

#ifdef __CONSERVATIVE_TRX_SPI_TIMING__    
    samr21delaySysTick(CPU_WAIT_CYCLES_BETWEEN_BYTES);
#endif    
    buffer->otFrame.mLength = samr21TrxSpiReadByteRaw();
    uint8_t numDownloadedPdsuBytes = 0;

    if(buffer->otFrame.mLength > IEEE_802_15_4_PDSU_SIZE){
        samr21TrxSpiStopAccess();
        samr21RadioRxCleanup(false);
        return;
    }

    //Dowload FCS + DSN Field First
    while (numDownloadedPdsuBytes < IEEE_802_15_4_STATIC_HEADER_SIZE)
    { 
        uint32_t timeout = 0xFFFF;
        //(see r21 datasheet, 40.7 Frame Buffer Empty Indicator)
        samr21delaySysTick(CPU_WAIT_CYCLES_FOR_FRAME_BUFFER_EMPTY_FLAG);
        while ( ( PORT->Group[1].IN.reg & PORT_PB00 ) && timeout){
            timeout--;
        }

        if(timeout){
            buffer->otFrame.mPsdu[numDownloadedPdsuBytes++] =
                samr21TrxSpiReadByteRaw();
            ;
        } else {
            samr21RadioRxCleanup(false);
            return;
        }
    }
    samr21TrxSpiCloseAccess();

    uint8_t relevantOctets = 0;

    otMacAddress srcAddr;
    otMacAddress dstAddr;

    otMacFrameGetSrcAddr(&(buffer->otFrame) , &srcAddr);
    otMacFrameGetDstAddr(&(buffer->otFrame) , &dstAddr);

    if(dstAddr.mType != OT_MAC_ADDRESS_TYPE_NONE){
        relevantOctets += 
            (dstAddr.mType == OT_MAC_ADDRESS_TYPE_EXTENDED ? 
                sizeof(uint64_t) 
                : sizeof(uint16_t)
            )
        ;
    }
    
    if(srcAddr.mType == OT_MAC_ADDRESS_TYPE_EXTENDED){
        relevantOctets += sizeof(uint64_t);
        if(utilsSoftSrcMatchExtFindEntry(srcAddr.mAddress.mExtAddress.m8) >= 0){
            buffer->framePending = true;
        } else {
            buffer->framePending = false;
        }
    }

    if(srcAddr.mType == OT_MAC_ADDRESS_TYPE_SHORT){
        relevantOctets += sizeof(uint16_t);
        if(utilsSoftSrcMatchShortFindEntry(srcAddr.mAddress.mShortAddress) >= 0){
            buffer->framePending = true;
        } else {
            buffer->framePending = false;
        }
    }            

    relevantOctets += 
        (otMacFrameIsSrcPanIdPresent(&(buffer->otFrame)) ? sizeof(uint16_t) : 0)            )
    ;

    relevantOctets += 
        (otMacFrameIsDstPanIdPresent(&(buffer->otFrame)) ? sizeof(uint16_t) : 0)            )
    ;

    if( relevantOctets ){
        
        uint32_t nextAction_us = 
            ( relevantOctets * IEEE_802_15_4_24GHZ_TIME_PER_OCTET_us )
            - ( AT86RF233_SPI_INIT_TIME_FRAMEBUFFER_us + RX_FRAMEBUFFER_ACCESS_HEADROOM_us)
            - ( (relevantOctets + IEEE_802_15_4_STATIC_HEADER_SIZE ) * AT86RF233_SPI_TIME_PER_BYTE_us)
        ;

        buffer->neededPdsuSizeForNextAction = relevantOctets + IEEE_802_15_4_STATIC_HEADER_SIZE;
        buffer->status = RX_STATUS_RECIVING_ADDR_FIELD;

        if(nextAction_us > 0xFFFF || nextAction_us < RX_FRAMEBUFFER_ACCESS_HEADROOM_us){
            samr21RadioRxDownloadAddrField();
            return;
        }

        samr21Timer4Set(nextAction_us);
        return;
    }

    uint32_t nextAction_us = 
        ( ( buffer->otFrame.mLength - numDownloadedPdsuBytes ) * IEEE_802_15_4_24GHZ_TIME_PER_OCTET_us)
        - (AT86RF233_SPI_INIT_TIME_FRAMEBUFFER_us + RX_FRAMEBUFFER_ACCESS_HEADROOM_us)
        - ( (buffer->otFrame.mLength + AT86RF233_FRAMEBUFFER_MISC_SIZE) * AT86RF233_SPI_TIME_PER_BYTE_us)
    ;

    buffer->neededPdsuSizeForNextAction = buffer->otFrame.mLength ;
    buffer->status = RX_STATUS_RECIVING_REMAINING;

    if(nextAction_us > 0xFFFF || nextAction_us < RX_FRAMEBUFFER_ACCESS_HEADROOM_us){
        samr21RadioRxDownloadRemaining();
        return;
    }

    samr21Timer4Set(nextAction_us);
}


void samr21RadioRxDownloadAddrField()
{
    samr21TrxSpiStartAccess(AT86RF233_CMD_FRAMEBUFFER_READ, 0);

    RxBuffer * buffer = &s_rxBuffer[s_activeRxBuffer];

#ifdef __CONSERVATIVE_TRX_SPI_TIMING__    
    samr21delaySysTick(CPU_WAIT_CYCLES_BETWEEN_BYTES);
#endif    
    buffer->otFrame.mLength = samr21TrxSpiReadByteRaw();
    uint8_t numDownloadedPdsuBytes = 0;

    if(buffer->otFrame.mLength > IEEE_802_15_4_PDSU_SIZE){
        samr21TrxSpiStopAccess();
        samr21RadioRxCleanup(false);
        return;
    }

    //Dowload FCF+ADDR Field First
    while ( numDownloadedPdsuBytes < buffer->neededPdsuSizeForNextAction )
    { 
        uint32_t timeout = 0xFFFF;
        //(see r21 datasheet, 40.7 Frame Buffer Empty Indicator)
        samr21delaySysTick(CPU_WAIT_CYCLES_FOR_FRAME_BUFFER_EMPTY_FLAG);
        while ( ( PORT->Group[1].IN.reg & PORT_PB00 ) && timeout){
            timeout--;
        }

        if(timeout){
            buffer->otFrame.mPsdu[numDownloadedPdsuBytes++] =
                samr21TrxSpiReadByteRaw();
            ;
        } else {
            samr21RadioRxCleanup(false);
            return;
        }
    }
    samr21TrxSpiCloseAccess();

    if( !otMacFrameDoesAddrMatch(
            &(buffer->otFrame),
            g_panId,
            g_shortAddr,
            (const otExtAddress*)(&g_extAddr)
        )
    ){
        samr21RadioRxCleanup(false);
        return;
    }


    uint32_t nextAction_us = 
        ( ( buffer->otFrame.mLength - numDownloadedPdsuBytes ) * IEEE_802_15_4_24GHZ_TIME_PER_OCTET_us)
        - (AT86RF233_SPI_INIT_TIME_FRAMEBUFFER_us + RX_FRAMEBUFFER_ACCESS_HEADROOM_us)
        - ( (buffer->otFrame.mLength + AT86RF233_FRAMEBUFFER_MISC_SIZE) * AT86RF233_SPI_TIME_PER_BYTE_us)
    ;

    buffer->neededPdsuSizeForNextAction = buffer->otFrame.mLength ;
    buffer->status = RX_STATUS_RECIVING_REMAINING;

    if(nextAction_us > 0xFFFF || nextAction_us < RX_FRAMEBUFFER_ACCESS_HEADROOM_us){
        samr21RadioRxDownloadRemaining();
        return;
    }

    samr21Timer4Set(nextAction_us);    
}


void samr21RadioRxDownloadRemaining()
{
    samr21TrxSpiStartAccess(AT86RF233_CMD_FRAMEBUFFER_READ, 0);

    RxBuffer * buffer = &s_rxBuffer[s_activeRxBuffer];

#ifdef __CONSERVATIVE_TRX_SPI_TIMING__    
    samr21delaySysTick(CPU_WAIT_CYCLES_BETWEEN_BYTES);
#endif    
    buffer->otFrame.mLength = samr21TrxSpiReadByteRaw();
    uint8_t numDownloadedPdsuBytes = 0;

    if(buffer->otFrame.mLength > IEEE_802_15_4_PDSU_SIZE){
        samr21TrxSpiStopAccess();
        samr21RadioRxCleanup(false);
        return;
    }

    //Dowload FCF+ADDR Field First
    while ( numDownloadedPdsuBytes < buffer->otFrame.mLength )
    { 
        uint32_t timeout = 0xFFFF;
        //(see r21 datasheet, 40.7 Frame Buffer Empty Indicator)
        samr21delaySysTick(CPU_WAIT_CYCLES_FOR_FRAME_BUFFER_EMPTY_FLAG);
        while ( ( PORT->Group[1].IN.reg & PORT_PB00 ) && timeout){
            timeout--;
        }

        if(timeout){
            buffer->otFrame.mPsdu[numDownloadedPdsuBytes++] =
                samr21TrxSpiReadByteRaw();
            ;
        } else {
            samr21RadioRxCleanup(false);
            return;
        }
    }
    //Dowload LQI, RSSI and CRC Check (r21 Datasheet 35.3.2 -  Frame Buffer Access Mode)
#ifdef __CONSERVATIVE_TRX_SPI_TIMING__    
    samr21delaySysTick(CPU_WAIT_CYCLES_BETWEEN_BYTES);
#endif 
    buffer->otFrame.mInfo.mRxInfo.mLqi = samr21TrxSpiReadByteRaw();

#ifdef __CONSERVATIVE_TRX_SPI_TIMING__    
    samr21delaySysTick(CPU_WAIT_CYCLES_BETWEEN_BYTES);
#endif 
    buffer->otFrame.mInfo.mRxInfo.mRssi = AT86RF233_RSSI_BASE_VAL + samr21TrxSpiReadByteRaw();

#ifdef __CONSERVATIVE_TRX_SPI_TIMING__    
    samr21delaySysTick(CPU_WAIT_CYCLES_BETWEEN_BYTES);
#endif 
    AT86RF233_REG_RX_STATUS_t rxStatus = (AT86RF233_REG_RX_STATUS_t)samr21TrxSpiReadByteRaw();

    samr21TrxSpiCloseAccess();

    if(!rxStatus.bit.crcValid){
        samr21RadioRxCleanup(false);
        return;
    }
    
    if(otMacFrameIsAckRequested(&(buffer->otFrame))){

        //Prep TRX
        samr21TrxWriteRegister(TRX_STATE_REG, TRX_CMD_FORCE_PLL_ON);

        if( otMacFrameIsVersion2015( &( buffer->otFrame ) ) ){
            samr21RadioRxSendEnhAck();
            return;
        }

        samr21RadioRxSendAck();
    }
}


void samr21RadioRxSendEnhAck(){
    RxBuffer * buffer = &s_rxBuffer[s_activeRxBuffer];
    buffer->status=RX_STATUS_SENDING_ENH_ACK;

    otRadioFrame ackFrame = {.mPsdu = &(buffer->rxAckPsdu[1])};

    uint8_t ieData[IEEE_802_15_4_PDSU_SIZE];
    uint8_t ieDataLen = 0;

#if OPENTHREAD_CONFIG_MAC_CSL_RECEIVER_ENABLE
    if (g_cslPeriod > 0)
    {
        ieData[ieDataLen++] = CSL_IE_HEADER_BYTES_LO;
        ieData[ieDataLen++] = CSL_IE_HEADER_BYTES_HI;

        uint16_t cslPhase = samr21RadioCtrlCslGetPhase();
        memcpy(&ieData[ieDataLen],&cslPhase,sizeof(uint16_t));
        ieDataLen += sizeof(uint16_t);
        
        memcpy(&ieData[ieDataLen],&g_cslPeriod,sizeof(uint16_t));
        ieDataLen += sizeof(uint16_t);
    }
#endif

#if OPENTHREAD_CONFIG_MLE_LINK_METRICS_SUBJECT_ENABLE
    otMacAddress macAddress;
    otMacFrameGetSrcAddr(&(buffer->otFrame), &macAddress);
    uint8_t linkMetricsDataLen = otLinkMetricsEnhAckGetDataLen(&macAddress);

    if (linkMetricsDataLen)
    {
        ieData[ieDataLen++] = CSL_IE_HEADER_BYTES_LO;
        otLinkMetricsEnhAckGenData
    }

#endif
    
    otMacFrameGenerateEnhAck(
        &(buffer->otFrame),
        buffer->framePending,
        &ackFrame
    );
    buffer->rxAckPsdu[0]=ackFrame.mLength;

    while (g_trxStatus.bit.trxStatus != TRX_STATUS_PLL_ON){
        samr21TrxUpdateStatus();
    }

    // Start Trasmission in advance, there is some spare time while the Preamble and SFD is transmitted
    samr21TrxSetSLP_TR(true);

    // Timout-Timer
    samr21Timer4Set( (IEEE_802_15_4_FRAME_SIZE*2) * IEEE_802_15_4_24GHZ_TIME_PER_OCTET_us );

    samr21TrxSpiStartAccess(AT86RF233_CMD_FRAMEBUFFER_WRITE, NULL);
    samr21TrxSetSLP_TR(false);
    samr21TrxSpiTransceiveBytesRaw(ackFrame.mPsdu[-1], NULL, s_txFrame.mLength - 1); // -2 FCS +1 PhyLen
    samr21TrxSpiCloseSpiAccess();
}

void samr21RadioRxSendAck(){
    RxBuffer * buffer = &s_rxBuffer[s_activeRxBuffer];
    buffer->status=RX_STATUS_SENDING_ACK;

    otRadioFrame ackFrame = {.mPsdu = &(buffer->rxAckPsdu[1])};
    
    otMacFrameGenerateImmAck(
        &(buffer->otFrame),
        buffer->framePending,
        &ackFrame
    );
    buffer->rxAckPsdu[0]=ackFrame.mLength;


    while (g_trxStatus.bit.trxStatus != TRX_STATUS_PLL_ON){
        samr21TrxUpdateStatus();
    }

    // Start Trasmission in advance, there is some spare time while the Preamble and SFD is transmitted
    samr21TrxSetSLP_TR(true);

    // Timout-Timer
    samr21Timer4Set( (IEEE_802_15_4_FRAME_SIZE*2) * IEEE_802_15_4_24GHZ_TIME_PER_OCTET_us );

    samr21TrxSpiStartAccess(AT86RF233_CMD_FRAMEBUFFER_WRITE, NULL);
    samr21TrxSetSLP_TR(false);
    samr21TrxSpiTransceiveBytesRaw(ackFrame.mPsdu[-1], NULL, ackFrame.mLength - 1); // -2 FCS +1 PhyLen
    samr21TrxSpiCloseSpiAccess();
}


void RTC_Handler(){
    RTC->MODE0.INTENFLAG.bit.CMP0 = 1;
    RTC->MODE0.INTENCLR.bit.CMP0 = 1;
    NVIC_DisableIRQ(RTC_IRQn);

    if(s_rxBuffer[ (s_activeRxBuffer + 1 ) % NUM_SAMR21_RX_BUFFER ].status == RX_STATUS_WAIT_FOR_DELAYED_START){
        
    }

    
}