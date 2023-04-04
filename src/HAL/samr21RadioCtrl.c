/*
 * Copyright (c) 2023 dresden elektronik ingenieurtechnik gmbh.
 * All rights reserved.
 *
 * The software in this package is published under the terms of the BSD
 * style license a copy of which has been included with this distribution in
 * the LICENSE.txt file.
 *
 */
#include "samr21RadioCtrl.h"

#include "otUtilities_macFrame.h"
#include "otUtilities_sourceMatch.h"
#include "otUtilities_linkMetrics.h"

static struct radioVars_s
{
    bool        initiated;
    bool        rxActive;

    uint16_t    shortAddress;
    uint16_t    panId;
    uint64_t    extendedAddress;

    bool        txBusy;
    bool        rxBusy;
    bool        edBusy;

    bool        promiscuousMode;
    bool        framePendingSrcMatch;

    uint32_t    macCounter;

    uint8_t     currentMacKeyId;

    uint8_t     previousMacKey[IEEE_15_4_AES_CCM_KEY_SIZE];
    uint8_t     currentMacKey[IEEE_15_4_AES_CCM_KEY_SIZE];
    uint8_t     nextMacKey[IEEE_15_4_AES_CCM_KEY_SIZE];

#if OPENTHREAD_CONFIG_MAC_CSL_RECEIVER_ENABLE
    uint32_t    cslSampleTime;
    uint32_t    cslPeriod;
#endif

    uint32_t    receiveSlotDuration_us;
    uint8_t     receiveSlotChannel;
    uint16_t    numReceivedFramesDuringSlot;


    samr21RxState rxState;
    samr21TxState txState;

} s_radioVars;

/***************Timer-Handler for Mac(Lite)-Operational Timing***************************/
static volatile bool *s_timeoutFlag = NULL;
static volatile void (*s_currentTimerHandler_fktPtr)(void) = NULL;

static void samr21RadioTimeoutTriggered()
{
    *s_timeoutFlag = true;
    s_currentTimerHandler_fktPtr = NULL;
}

static void samr21RadioStartTimeoutTimer(uint16_t a_duration_us, bool *a_triggerFlag)
{
    s_timeoutFlag = a_triggerFlag;
    s_currentTimerHandler_fktPtr = &samr21RadioTimeoutTriggered;
    samr21Timer4Set(a_duration_us);
}

static void samr21RadioStopTimeoutTimer()
{
    samr21Timer4Stop();
    s_timeoutFlag = NULL;
    s_currentTimerHandler_fktPtr = NULL;
}

static void samr21RadioQueueDelayedAction(uint16_t a_delay_us, void (*a_queuedAction_fktPtr)(void))
{
    s_currentTimerHandler_fktPtr = a_queuedAction_fktPtr;
    samr21Timer4Set(a_delay_us);
}

static void samr21RadioRemoveQueuedAction()
{
    s_currentTimerHandler_fktPtr = NULL;
    samr21Timer4Stop();
}

// NVIC IRQ-Handler Function for Timer5 (TC5). Fkt-Addr is found in NVIC-IRQ-Vector-Table
void TC4_Handler()
{
    // Reset IRQ
    TC4->COUNT16.INTFLAG.bit.OVF = 1;

    if (s_currentTimerHandler_fktPtr)
    {
        // Invoke Handler
        (*s_currentTimerHandler_fktPtr)();
    }
}

static volatile void (*s_currentRtcHandler_fktPtr)(void) = NULL;

static void samr21RadioCtrlQueueDelayedEventAbsolute(uint32_t a_triggerTimestamp, void (*a_queuedAction_fktPtr)(void))
{
    s_currentRtcHandler_fktPtr = &a_queuedAction_fktPtr;
    samr21RtcSetAbsoluteAlarm(a_triggerTimestamp);
}

static void samr21RadioCtrlQueueDelayedEventRelative(uint32_t a_duration, void (*a_queuedAction_fktPtr)(void))
{
    s_currentRtcHandler_fktPtr = &a_queuedAction_fktPtr;
    samr21RtcSetRelativeAlarm(a_duration);
}

static void samr21RadioCtrlRemoveQueuedEvent()
{
    s_currentRtcHandler_fktPtr = NULL;
    samr21RtcStopAlarm();
}

// NVIC IRQ-Handler Function for RTC. Fkt-Addr is found in NVIC-IRQ-Vector-Table
void RTC_Handler()
{
    RTC->MODE0.INTFLAG.bit.CMP0 = 1;
    RTC->MODE0.INTENCLR.bit.CMP0 = 1;
    NVIC_DisableIRQ(RTC_IRQn);

    if (s_currentRtcHandler_fktPtr)
    {
        // Invoke Handler
        (*s_currentRtcHandler_fktPtr)();
        // Remove Handler
        s_currentRtcHandler_fktPtr = NULL;
    }
}

void samr21RadioCtrlEnable()
{
    samr21TrxInterruptInit();

    // Enable TRX-IRQ in NVIC
    __NVIC_EnableIRQ(EIC_IRQn);

    // Enable Radio Timer
    samr21Timer4Init(0); // 1MHz / (2^0) -> 1us resolution

    // HardwareTrx is always initiated cause the clk is sourced from the TRX
    s_radioVars.initiated = true;
}

void samr21RadioCtrlDisable()
{   
    // Disable IRQ in NVIC
    __NVIC_DisableIRQ(EIC_IRQn);

    samr21RadioRemoveQueuedAction();
    samr21TrxRemoveAllHandler();
    samr21TrxDisableAllIrq();

    // HardwareTrx is always initiated cause the clk is sourced from the TRX
    s_radioVars.initiated = false;
    s_radioVars.txBusy = false;
    s_radioVars.edBusy = false;
    s_radioVars.rxBusy = false;
    s_radioVars.initiated = false;
    s_radioVars.rxActive = false;
}

void samr21RadioCtrlSleep()
{
    s_radioVars.rxActive = false;
    
    samr21RadioRemoveQueuedAction();
    samr21TrxRemoveAllHandler();

    s_radioVars.txBusy = false;
    s_radioVars.edBusy = false;
    s_radioVars.rxBusy = false;
}

otRadioState samr21RadioGetOtState(){
    if(!s_radioVars.initiated){
        return OT_RADIO_STATE_DISABLED;
    }

    if(s_radioVars.txBusy){
        return OT_RADIO_STATE_TRANSMIT;
    }

    if(s_radioVars.rxActive || s_radioVars.edBusy){
        return OT_RADIO_STATE_RECEIVE;
    }

    return OT_RADIO_STATE_SLEEP;
}

void samr21RadioCtrlSetPromiscuousMode(bool a_enable)
{
    s_radioVars.promiscuousMode = a_enable;
}
bool samr21RadioCtrlGetPromiscuousMode()
{
    return s_radioVars.promiscuousMode;
}

void samr21RadioCtrlSetMacFrameCounter(uint32_t a_macFrameCounter)
{
    s_radioVars.macCounter = a_macFrameCounter;
}
uint32_t samr21RadioCtrlGetMacFrameCounter()
{
    return s_radioVars.macCounter;
}

#if OPENTHREAD_CONFIG_MAC_CSL_RECEIVER_ENABLE

void samr21RadioCtrlSetCslPeriod(uint32_t a_cslPeriod)
{
    s_radioVars.cslPeriod = a_cslPeriod;
}

void samr21RadioCtrlUpdateCslSampleTime(uint32_t a_cslSampleTime)
{
    s_radioVars.cslSampleTime = a_cslSampleTime;
}

uint16_t samr21RadioCtrlCslGetPhase()
{
    uint32_t curTime = samr21RtcGetTimestamp();
    uint32_t cslPeriodInUs = s_radioVars.cslPeriod * OT_US_PER_TEN_SYMBOLS;
    uint32_t diff = (cslPeriodInUs - (curTime % cslPeriodInUs) + (s_radioVars.cslSampleTime % cslPeriodInUs)) % cslPeriodInUs;
    return (uint16_t)(diff / OT_US_PER_TEN_SYMBOLS + 1);
}

#endif

void samr21RadioCtrlSetMacKeys(

    uint8_t a_keyId,
    const uint8_t *a_previousKey_1D,
    const uint8_t *a_currentKey_1D,
    const uint8_t *a_nextKey_1D)
{
    __disable_irq();

    s_radioVars.currentMacKeyId = a_keyId;

     memcpy(s_radioVars.previousMacKey, a_previousKey_1D, IEEE_15_4_AES_CCM_KEY_SIZE);
     memcpy(s_radioVars.currentMacKey, a_currentKey_1D, IEEE_15_4_AES_CCM_KEY_SIZE);
     memcpy(s_radioVars.nextMacKey, a_nextKey_1D, IEEE_15_4_AES_CCM_KEY_SIZE);

    __enable_irq();

    samr21TrxAesKeySetup(s_radioVars.currentMacKey);
}

void samr21RadioCtrlSetPanId(uint16_t a_panId){
    s_radioVars.panId = a_panId;
}
uint16_t samr21RadioCtrlGetPanId(){
    return s_radioVars.panId;
}

void samr21RadioCtrlSetShortAddress(uint16_t a_shortAddress){
    s_radioVars.shortAddress = a_shortAddress;
}
uint16_t samr21RadioCtrlGetShortAddr(){
    return s_radioVars.shortAddress;
}

void samr21RadioCtrlSetExtendedAddress(uint8_t* a_extendedAddress_1D){

    uint8_t * extendedAddress_p = (uint8_t*) &s_radioVars.extendedAddress;

    for(uint8_t i = 0; i < IEEE_15_4_EXTENDED_ADDR_SIZE; i++){
        extendedAddress_p[i] = a_extendedAddress_1D[(IEEE_15_4_EXTENDED_ADDR_SIZE-1)-i];
    }
}

uint8_t* samr21RadioCtrlGetExtendedAddress(){
    return (uint8_t*) &s_radioVars.extendedAddress;
}


void samr21RadioCtrlSetFramePendingSrcMatch(bool a_enable)
{
    s_radioVars.framePendingSrcMatch = a_enable;
}




/******************************RX Operation***************************************/
//Prototype
static void samr21RadioAbortReception();
static void samr21RadioFinishReception();
static void samr21RadioPrepareImmediateAck();
static void samr21RadioPrepareEnhancedAck();
static void samr21RadioRxDownloadAndHandleRemaining();
static void samr21RadioRxDownloadAndHandleFcf();
static void samr21RadioRxDownloadAndHandleAddrField();
static void samr21RadioSendAck();
static void samr21RadioStartQueuedReceiveSlot();




static struct rxBuffer_s
{
    bool done;

    uint8_t frameBuffer[IEEE_15_4_FRAME_SIZE];

    otRadioFrame otFrame;

    uint8_t neededPdsuSizeForNextAction;
    bool framePending;
} s_rxBuffer[SAMR21_NUM_RX_BUFFER];
static uint8_t s_activeRxBuffer = 0;


//Software Ack
static uint8_t s_rxAckFrameBuffer[IEEE_15_4_FRAME_SIZE];
static uint8_t s_rxAckNonce[IEEE_15_4_AES_CCM_NONCE_SIZE];
static otRadioFrame s_rxOtAckFrame = {.mPsdu = &s_rxAckFrameBuffer[1]};

static uint8_t *s_rxAckPayload;
static uint8_t *s_rxAckFooter;
static uint8_t *s_rxAckAesKey;
static uint8_t s_rxAckSecurityLevel;

static void samr21RadioAbortReception()
{
    PORT->Group[0].OUTCLR.reg = PORT_PA09;
    s_radioVars.rxState = SAMR21_RADIO_RX_STATE_IDLE;

    s_radioVars.rxBusy = false;

    // Stop Timeout
    samr21RadioRemoveQueuedAction();
    

    s_rxBuffer[s_activeRxBuffer].done = false;
    

    // Put TRX in receive again
    samr21TrxQueueMoveToRx(false);

    // Remove TrxEnd Handler
    samr21TrxSetIrqHandler(TRX_IRQ_TRX_END, NULL);
}

static void samr21RadioFinishReception()
{
    PORT->Group[0].OUTCLR.reg = PORT_PA09;
    s_radioVars.rxState = SAMR21_RADIO_RX_STATE_IDLE;

    s_radioVars.rxBusy = false;
    s_rxBuffer[s_activeRxBuffer].done = true;

    s_activeRxBuffer = (s_activeRxBuffer + 1) % SAMR21_NUM_RX_BUFFER;
    s_rxBuffer[s_activeRxBuffer].otFrame.mPsdu = &s_rxBuffer[s_activeRxBuffer].frameBuffer[1];
    s_rxBuffer[s_activeRxBuffer].done = false;

    // Stop Timeout
    samr21RadioRemoveQueuedAction();

    // Remove TrxEnd Handler
    samr21TrxSetIrqHandler(TRX_IRQ_TRX_END, NULL);

    // Put TRX in receive again
    samr21TrxQueueMoveToRx(false);

    cb_samr21RadioReceptionDone();
}

static void samr21RadioSendAck()
{
    s_radioVars.rxState = SAMR21_RADIO_RX_STATE_SENDING_ACK;
    // Start Transmission with a empty Framebuffer, cause the there is time to fill it while Preamble and sfd is transmitted first
    samr21TrxSetSLP_TR(true);

    if (s_rxAckSecurityLevel)
    {
        samr21TrxSendFrameAndApplyMacSecurity(
            &s_rxAckFrameBuffer[0],
            s_rxAckPayload,
            s_rxAckFooter,
            s_rxAckSecurityLevel,
            s_radioVars.currentMacKey,
            s_rxAckNonce
        );
    }
    else
    {
        samr21TrxUploadToFramebuffer(s_rxAckFrameBuffer, s_rxAckFrameBuffer[0], 0);
    }

    samr21TrxSetSLP_TR(false);

    s_radioVars.rxState = SAMR21_RADIO_RX_STATE_WAIT_ACK_END;

    // Set a handler for when the Ack is successfully transmitted
    samr21TrxSetIrqHandler(TRX_IRQ_TRX_END, samr21RadioFinishReception);

    // Set a Timeout in case the Trx never get triggered
    samr21RadioQueueDelayedAction(IEEE_15_4_FRAME_SIZE * IEEE_15_4_24GHZ_TIME_PER_OCTET_us, samr21RadioAbortReception);
}

// IEEE 802.15.4 2006
static void samr21RadioPrepareImmediateAck()
{

    otMacFrameGenerateImmAck(
        &(s_rxBuffer[s_activeRxBuffer].otFrame),
        s_rxBuffer[s_activeRxBuffer].framePending,
        &s_rxOtAckFrame
    );

    s_rxAckFrameBuffer[0] = s_rxOtAckFrame.mLength;
}

// IEEE 802.15.4 2015+
static void samr21RadioPrepareEnhancedAck()
{
    s_rxAckSecurityLevel = 0;

    // Prepare Data for Header-IE
    uint8_t ieData[IEEE_15_4_PDSU_SIZE]; // TODO can be smaller
    uint8_t ieDataLen = 0;

#if OPENTHREAD_CONFIG_MAC_CSL_RECEIVER_ENABLE
    if (s_radioVars.cslPeriod > 0)
    {
        ieData[ieDataLen++] = CSL_IE_HEADER_BYTES_LO;
        ieData[ieDataLen++] = CSL_IE_HEADER_BYTES_HI;

        uint16_t cslPhase = samr21RadioCtrlCslGetPhase();
        memcpy(&ieData[ieDataLen], &cslPhase, sizeof(uint16_t));
        ieDataLen += sizeof(uint16_t);

        memcpy(&ieData[ieDataLen], &s_radioVars.cslPeriod, sizeof(uint16_t));
        ieDataLen += sizeof(uint16_t);
    }
#endif

#if OPENTHREAD_CONFIG_MLE_LINK_METRICS_SUBJECT_ENABLE
    otMacAddress macAddress;
    otMacFrameGetSrcAddr(&(s_rxBuffer[s_activeRxBuffer].otFrame), &macAddress);
    uint8_t linkMetricsDataLen = otLinkMetricsEnhAckGenData(
        &macAddress,
        s_rxBuffer[s_activeRxBuffer].otFrame.mInfo.mRxInfo.mLqi,
        s_rxBuffer[s_activeRxBuffer].otFrame.mInfo.mRxInfo.mRssi,
        &ieData[ieDataLen + 4] // IE VENDOR HEADER LENGTH
    );

    if (linkMetricsDataLen)
    {
        ieData[ieDataLen++] = ENH_ACK_PROBING_IE;

        ieData[ieDataLen++] = (VENDOR_OUI_THREAD_COMPANY_ID & 0x0000FF) >> 0;
        ieData[ieDataLen++] = (VENDOR_OUI_THREAD_COMPANY_ID & 0x00FF00) >> 8;
        ieData[ieDataLen++] = (VENDOR_OUI_THREAD_COMPANY_ID & 0xFF0000) >> 16;

        ieDataLen += linkMetricsDataLen;
    }
#endif

    otMacFrameGenerateEnhAck(
        &(s_rxBuffer[s_activeRxBuffer].otFrame),
        s_rxBuffer[s_activeRxBuffer].framePending,
        ieData,
        ieDataLen,
        &s_rxOtAckFrame);

    s_rxAckFrameBuffer[0] = s_rxOtAckFrame.mLength;
    s_rxBuffer[s_activeRxBuffer].otFrame.mInfo.mRxInfo.mAckedWithFramePending = s_rxBuffer[s_activeRxBuffer].framePending;

    // Check if the Ack needs Security
    if (otMacFrameIsSecurityEnabled(&s_rxOtAckFrame))
    {
        s_rxBuffer[s_activeRxBuffer].otFrame.mInfo.mRxInfo.mAckedWithSecEnhAck = true;

        // If Security is needed the FrameCounter and KeyID need to be inserted
        __disable_irq();
        otMacFrameSetFrameCounter(&s_rxOtAckFrame, ++s_radioVars.macCounter);
        s_rxBuffer[s_activeRxBuffer].otFrame.mInfo.mRxInfo.mAckFrameCounter = s_radioVars.macCounter;

        if (otMacFrameIsKeyIdMode1(&s_rxOtAckFrame))
        {
            otMacFrameSetKeyId(&s_rxOtAckFrame, s_radioVars.currentMacKeyId);
            s_rxBuffer[s_activeRxBuffer].otFrame.mInfo.mRxInfo.mAckKeyId = s_radioVars.currentMacKeyId;
        }
        __enable_irq();

        // Generate Nonce
        otMacGenerateNonce(&s_rxOtAckFrame, s_radioVars.extendedAddress, s_rxAckNonce);
        
        //Set some Makers for AES-CCM
        s_rxAckFooter = otMacFrameGetFooter(&s_rxOtAckFrame);
        s_rxAckSecurityLevel = otMacFrameGetSecurityLevel(&s_rxOtAckFrame);
    }
}

static void samr21RadioRxDownloadAndHandleRemaining()
{

    s_radioVars.rxState = SAMR21_RADIO_RX_STATE_PARSE_REMAINING;

    if (!samr21TrxDownloadFramebuffer(
            &s_rxBuffer[s_activeRxBuffer].otFrame.mLength,
            s_rxBuffer[s_activeRxBuffer].otFrame.mPsdu,
            &s_rxBuffer[s_activeRxBuffer].otFrame.mInfo.mRxInfo.mLqi,
            &s_rxBuffer[s_activeRxBuffer].otFrame.mInfo.mRxInfo.mRssi 
        )
    ){
        // Timeout or invalid Frame Length
        samr21RadioAbortReception();
        return;
    }

    bool ackRequested  = otMacFrameIsAckRequested(&(s_rxBuffer[s_activeRxBuffer].otFrame));

    // A Message has been fully received at this point
    if (s_radioVars.promiscuousMode || !ackRequested)
    {
        // No Acknowledgment was requested
        samr21RadioFinishReception();
        return;
    }

    // A Acknowledgment was requested
    // Send after Ack-InterFrame-Spacing delay
    samr21RadioQueueDelayedAction(IEEE_15_4_ADJUSTED_AIFS_us, samr21RadioSendAck);

    s_radioVars.rxState = SAMR21_RADIO_RX_STATE_PREP_ACK;
    // Move TRX to Tx, so PLL is already dialed in when ack is about to be Transmitted
    samr21TrxForceMoveToTx(false);

    if (otMacFrameIsVersion2015(&(s_rxBuffer[s_activeRxBuffer].otFrame)))
    {
        samr21RadioPrepareEnhancedAck();
    }
    else
    {
        samr21RadioPrepareImmediateAck();
    }
    s_radioVars.rxState = SAMR21_RADIO_RX_STATE_WAIT_ACK_START;
}

static void samr21RadioRxDownloadAndHandleAddrField()
{

    s_radioVars.rxState = SAMR21_RADIO_RX_STATE_PARSE_ADDR;

    if (!samr21TrxLiveFramebufferDownload(
            s_rxBuffer[s_activeRxBuffer].frameBuffer,
            s_rxBuffer[s_activeRxBuffer].neededPdsuSizeForNextAction))
    {
        // Timeout or invalid Frame Length
        samr21RadioAbortReception();
        return;
    }

    bool isRecipient = otMacFrameDoesAddrMatch(
                                  &(s_rxBuffer[s_activeRxBuffer].otFrame),
                                  s_radioVars.panId,
                                  s_radioVars.shortAddress,
                                  (uint8_t*) &s_radioVars.extendedAddress
                        );

    if (!s_radioVars.promiscuousMode && !isRecipient)
    {
        // Device is not recipient
        samr21RadioAbortReception();
        return;
    }

    if(s_radioVars.framePendingSrcMatch){

        otMacAddress srcAddr;
        otMacFrameGetSrcAddr(&(s_rxBuffer[s_activeRxBuffer].otFrame), &srcAddr);

        s_rxBuffer[s_activeRxBuffer].framePending = false;

        if(srcAddr.mType == OT_MAC_ADDRESS_TYPE_EXTENDED){
            if(utilsSoftSrcMatchExtFindEntry(&srcAddr.mAddress.mExtAddress) > 0){
                s_rxBuffer[s_activeRxBuffer].framePending = true;
            }
        }

        if(srcAddr.mType == OT_MAC_ADDRESS_TYPE_SHORT){
            if(utilsSoftSrcMatchShortFindEntry(srcAddr.mAddress.mShortAddress) > 0){
                s_rxBuffer[s_activeRxBuffer].framePending = true;
            }
        }
    }
    else
    {
        s_rxBuffer[s_activeRxBuffer].framePending = true;
    }
    
    uint32_t nextAction_us =
        ((s_rxBuffer[s_activeRxBuffer].otFrame.mLength - s_rxBuffer[s_activeRxBuffer].neededPdsuSizeForNextAction) * (IEEE_15_4_24GHZ_TIME_PER_OCTET_us - SAMR21_SPI_TIME_PER_BYTE_us)) - SAMR21_SPI_INIT_TIME_FRAMEBUFFER_us - (s_rxBuffer[s_activeRxBuffer].neededPdsuSizeForNextAction * SAMR21_SPI_TIME_PER_BYTE_us);

    s_rxBuffer[s_activeRxBuffer].neededPdsuSizeForNextAction = s_rxBuffer[s_activeRxBuffer].otFrame.mLength;

    if(nextAction_us > SAMR21_SOFTWARE_RADIO_MIN_DOWNLOAD_BACKOFF_us){
        s_radioVars.rxState = SAMR21_RADIO_RX_STATE_WAIT_REMAINING;
        samr21RadioQueueDelayedAction(nextAction_us, samr21RadioRxDownloadAndHandleRemaining);
        return;
    }

    samr21RadioRxDownloadAndHandleRemaining();
}

static void samr21RadioRxDownloadAndHandleFcf()
{
    s_radioVars.rxBusy = true;
    s_radioVars.rxState = SAMR21_RADIO_RX_STATE_PARSE_FCF;
    
    PORT->Group[0].OUTSET.reg = PORT_PA09;

    s_rxBuffer[s_activeRxBuffer].otFrame.mInfo.mRxInfo.mTimestamp = samr21RtcGetTimestamp();
    s_rxBuffer[s_activeRxBuffer].otFrame.mChannel = samr21TrxGetChannel(); 

    // +4 => 1Byte phyLen, 2Byte FCF, 1Byte DSN
    if (!samr21TrxLiveFramebufferDownload(s_rxBuffer[s_activeRxBuffer].frameBuffer, 4))
    { 
        // Timeout or invalid Frame Length
        samr21RadioAbortReception();
        return;
    }

    if(s_rxBuffer[s_activeRxBuffer].frameBuffer[0] > IEEE_15_4_FRAME_SIZE){
        //Can't be a valid frame
        samr21RadioAbortReception();
        return;
    }

    s_rxBuffer[s_activeRxBuffer].otFrame.mPsdu = &s_rxBuffer[s_activeRxBuffer].frameBuffer[1]; // phyLen not part of psdu
    s_rxBuffer[s_activeRxBuffer].otFrame.mLength = s_rxBuffer[s_activeRxBuffer].frameBuffer[0];

    // Calculate the size of the Address Field
    otMacAddress srcAddr;
    otMacAddress dstAddr;

    otMacFrameGetSrcAddr(&(s_rxBuffer[s_activeRxBuffer].otFrame), &srcAddr);
    otMacFrameGetDstAddr(&(s_rxBuffer[s_activeRxBuffer].otFrame), &dstAddr);

    uint8_t addrFieldSize = 0;

    if (dstAddr.mType != OT_MAC_ADDRESS_TYPE_NONE)
    {
        addrFieldSize +=
            (dstAddr.mType == OT_MAC_ADDRESS_TYPE_EXTENDED ? sizeof(uint64_t)
                                                           : sizeof(uint16_t));
    }

    if (srcAddr.mType == OT_MAC_ADDRESS_TYPE_EXTENDED)
    {
        addrFieldSize += sizeof(uint64_t);

        if (utilsSoftSrcMatchExtFindEntry(srcAddr.mAddress.mExtAddress.m8) >= 0)
        {
            s_rxBuffer[s_activeRxBuffer].framePending = true;
        }
        else
        {
            s_rxBuffer[s_activeRxBuffer].framePending = false;
        }
    }

    if (srcAddr.mType == OT_MAC_ADDRESS_TYPE_SHORT)
    {
        addrFieldSize += sizeof(uint16_t);
        if (utilsSoftSrcMatchShortFindEntry(srcAddr.mAddress.mShortAddress) >= 0)
        {
            s_rxBuffer[s_activeRxBuffer].framePending = true;
        }
        else
        {
            s_rxBuffer[s_activeRxBuffer].framePending = false;
        }
    }

    addrFieldSize +=
        (otMacFrameIsSrcPanIdPresent(&(s_rxBuffer[s_activeRxBuffer].otFrame)) ? sizeof(uint16_t) : 0);

    addrFieldSize +=
        (otMacFrameIsDstPanIdPresent(&(s_rxBuffer[s_activeRxBuffer].otFrame)) ? sizeof(uint16_t) : 0);

    if (addrFieldSize)
    {
        uint32_t nextAction_us =
            ((addrFieldSize) * (IEEE_15_4_24GHZ_TIME_PER_OCTET_us - SAMR21_SPI_TIME_PER_BYTE_us)) - SAMR21_SPI_INIT_TIME_FRAMEBUFFER_us - (4 * SAMR21_SPI_TIME_PER_BYTE_us);

        s_rxBuffer[s_activeRxBuffer].neededPdsuSizeForNextAction = addrFieldSize + 4;

        if(nextAction_us > SAMR21_SOFTWARE_RADIO_MIN_DOWNLOAD_BACKOFF_us){
            s_radioVars.rxState = SAMR21_RADIO_RX_STATE_WAIT_ADDR;
            samr21RadioQueueDelayedAction(nextAction_us, samr21RadioRxDownloadAndHandleAddrField);
            return;
        }
        samr21RadioRxDownloadAndHandleAddrField();
        return;
    }

    // Calculate the time when to start the next read Access to get the Addr Field
    uint32_t nextAction_us = s_rxBuffer[s_activeRxBuffer].otFrame.mLength * (IEEE_15_4_24GHZ_TIME_PER_OCTET_us - SAMR21_SPI_TIME_PER_BYTE_us);
    nextAction_us -= SAMR21_SPI_INIT_TIME_FRAMEBUFFER_us;

    s_rxBuffer[s_activeRxBuffer].neededPdsuSizeForNextAction = s_rxBuffer[s_activeRxBuffer].otFrame.mLength;

    if(nextAction_us > SAMR21_SOFTWARE_RADIO_MIN_DOWNLOAD_BACKOFF_us){
        s_radioVars.rxState = SAMR21_RADIO_RX_STATE_WAIT_REMAINING;
        samr21RadioQueueDelayedAction(nextAction_us, samr21RadioRxDownloadAndHandleRemaining);
        return;
    }

    samr21RadioRxDownloadAndHandleRemaining();
}


void samr21RadioReceive(uint8_t a_channel)
{
    //Cleanup from previous Mode
    samr21TrxRemoveAllHandler();
    samr21RadioRemoveQueuedAction();

    s_radioVars.rxActive = 1;
    s_radioVars.rxBusy = 0;

    s_radioVars.rxState = SAMR21_RADIO_RX_STATE_IDLE;
    
    s_rxBuffer[s_activeRxBuffer].otFrame.mPsdu = &s_rxBuffer[s_activeRxBuffer].frameBuffer[1];
    s_rxBuffer[s_activeRxBuffer].done = false;

    if (a_channel)
    {
        samr21TrxSetChannel(a_channel);
    }

    samr21TrxSetIrqHandler(TRX_IRQ_RX_START, samr21RadioRxDownloadAndHandleFcf);

    // Enable Rx Start IRQ for reception
    samr21TrxEnableIrq(TRX_IRQ_RX_START);

    // Enable Trx End IRQ to confirm Ack Transmission
    samr21TrxEnableIrq(TRX_IRQ_TRX_END);

    // Enable Buffer Underrun for LIVE RX Parser
    samr21TrxEnableIrq(TRX_IRQ_TRX_UR);

    //Move to Rx
    samr21TrxQueueMoveToRx(true);
}

otRadioFrame* samr21RadioGetReceivedFrame(){
    for(uint8_t i = 0; i < SAMR21_NUM_RX_BUFFER; i++){
        if(s_rxBuffer[i].done){
            s_rxBuffer[i].done = false;
            return &(s_rxBuffer[i].otFrame);
        }
    }
    //No Finished Buffer available
    return NULL;
}

static void samr21RadioFinishQueuedReceiveSlot(){

    //Wait for Current Reception to finish
    while(s_radioVars.rxBusy);

    samr21RadioCtrlSleep();

    if(s_radioVars.numReceivedFramesDuringSlot == 0){
        cb_samr21RadioNoMessagesDuringSlot();
    }

    s_radioVars.numReceivedFramesDuringSlot = 0;
    s_radioVars.receiveSlotChannel = 0;
    s_radioVars.receiveSlotDuration_us = 0;
}

static void samr21RadioStartQueuedReceiveSlot(){

    s_radioVars.numReceivedFramesDuringSlot = 0;
    samr21RadioReceive(s_radioVars.receiveSlotChannel);

    if(s_radioVars.receiveSlotDuration_us){
        samr21RadioCtrlQueueDelayedEventRelative(s_radioVars.receiveSlotDuration_us, samr21RadioFinishQueuedReceiveSlot);
    }

}

bool samr21RadioQueueReceiveSlot(uint32_t a_startPointTimestamp, uint8_t a_channel, uint32_t a_duration){
    
    if(s_currentRtcHandler_fktPtr){

        //There already is a Queued Event
        return false;
    }

    s_radioVars.receiveSlotDuration_us = a_duration;
    s_radioVars.receiveSlotChannel = a_channel;

    samr21RadioCtrlQueueDelayedEventAbsolute(a_startPointTimestamp, samr21RadioStartQueuedReceiveSlot);

    return true;
}


/******************************TX Operation***************************************/
//Prototypes
static void samr21RadioFinishTransmission();
static void samr21RadioAbortTransmission();
static void samr21RadioEvaluateAck();
static void samr21RadioPrepareForAckReception();
static void samr21RadioRetryTransmission();
static void samr21RadioStartTransmission();
static void samr21RadioEvaluateCcaResult();
static void samr21RadioStartCca();

static uint8_t s_txFrameBuffer[IEEE_15_4_FRAME_SIZE];
static uint8_t a_txAckFrameBuffer[IEEE_15_4_FRAME_SIZE];
static otRadioFrame s_txOtFrame = { .mPsdu = &s_txFrameBuffer[1]};
static otRadioFrame s_txAckOtFrame = { .mPsdu = &a_txAckFrameBuffer[1]};

static uint8_t s_txNumTransmissionRetries;
static uint8_t s_txNumCsmaBackoff;


//Transmission Security
static uint8_t s_txNonce[IEEE_15_4_AES_CCM_NONCE_SIZE];
static uint8_t *s_txPayload;
static uint8_t *s_txFooter;
static uint8_t *s_txAesKey;
static uint8_t s_txSecurityLevel;


static void samr21RadioFinishTransmission(){

    s_radioVars.txState = SAMR21_RADIO_TX_STATE_IDLE;
    PORT->Group[0].OUTCLR.reg = PORT_PA08;

    //Stop Timeout
    samr21RadioRemoveQueuedAction();

    s_radioVars.txBusy = false;

    //Move back to Receive
    samr21RadioReceive(0);

    //Inform UpperLayer
    cb_samr21RadioTransmissionDone(RADIO_TRANSMISSION_SUCCESSFUL);
}

static void samr21RadioAbortTransmission(){
    
    PORT->Group[0].OUTCLR.reg = PORT_PA08;
    s_radioVars.txState = SAMR21_RADIO_TX_STATE_IDLE;
    
    //Stop Timeout
    samr21RadioRemoveQueuedAction();

    s_radioVars.txBusy = false;

    if(s_txNumCsmaBackoff >= s_txOtFrame.mInfo.mTxInfo.mMaxCsmaBackoffs)
    {
        cb_samr21RadioTransmissionDone(RADIO_TRANSMISSION_CHANNEL_ACCESS_FAILED);
    }
    else if(s_txNumTransmissionRetries >= s_txOtFrame.mInfo.mTxInfo.mMaxFrameRetries)
    {
        cb_samr21RadioTransmissionDone(RADIO_TRANSMISSION_NO_ACK);
    }
    else 
    {
        cb_samr21RadioTransmissionDone(RADIO_TRANSMISSION_UNDEFINED_ERROR);
    }

    //Move back to Receive
    samr21RadioReceive(0);
}

static void samr21RadioEvaluateAck(){

    s_radioVars.txState = SAMR21_RADIO_TX_STATE_EVAL_ACK;

    //Stop Timeout
    samr21RadioRemoveQueuedAction();

    //Download Ack
    bool validFrame = samr21TrxDownloadFramebuffer(
        &s_txAckOtFrame.mLength,
        s_txAckOtFrame.mPsdu,
        &s_txAckOtFrame.mInfo.mRxInfo.mLqi,
        &s_txAckOtFrame.mInfo.mRxInfo.mRssi
    );

    if (
        otMacFrameIsAck(&s_txAckOtFrame) 
        && ( otMacFrameGetSequence(&s_txAckOtFrame) == otMacFrameGetSequence(&s_txOtFrame) ) 
        && validFrame
    ){
        //Ack is Valid
        samr21RadioFinishTransmission();
        return;
    }

    //Ack is Invalid
    samr21RadioRetryTransmission();
}

static void samr21RadioAckReceptionStarted(){

    s_radioVars.txState = SAMR21_RADIO_TX_STATE_WAIT_FOR_ACK_END;

    //Add a Timestamp for the Ack reception started
    s_txAckOtFrame.mInfo.mRxInfo.mTimestamp = samr21RtcGetTimestamp();

    //Add a Handler for when Ack is fully received
    samr21TrxSetIrqHandler(TRX_IRQ_TRX_END, samr21RadioEvaluateAck);

}

static void samr21RadioPrepareForAckReception(){

    PORT->Group[0].OUTCLR.reg = PORT_PA08;

    s_radioVars.txState = SAMR21_RADIO_TX_STATE_WAIT_FOR_ACK_START;

    //Stop Timeout
    samr21RadioRemoveQueuedAction();

    //Change Radio Mode to RX
    samr21TrxQueueMoveToRx(true);

    //Add a Handler for when Ack is received
    samr21TrxSetIrqHandler(TRX_IRQ_RX_START, samr21RadioAckReceptionStarted);

    //Add a Timeout in case there is no Ack
    samr21RadioQueueDelayedAction(SAMR21_SOFTWARE_RADIO_ACK_TIMEOUT_us, samr21RadioRetryTransmission);
}

    
static void samr21RadioRetryTransmission(){
    if(s_txNumTransmissionRetries < s_txOtFrame.mInfo.mTxInfo.mMaxFrameRetries)
    {
        //Reset CSMA Counter
        s_txNumCsmaBackoff = 0;

        //Increase Retry Counter
        s_txNumTransmissionRetries++;

        //Start over
        samr21RadioStartCca();
        return;
    }

    //No Retry Attempts left
    samr21RadioAbortTransmission();
}

static void samr21RadioStartTransmission(){

    s_radioVars.txState = SAMR21_RADIO_TX_STATE_SENDING;

    samr21TrxForceMoveToTx(true);
    PORT->Group[0].OUTSET.reg = PORT_PA08; 

    if (!s_txOtFrame.mInfo.mTxInfo.mIsHeaderUpdated)
    {

#if OPENTHREAD_CONFIG_MAC_CSL_RECEIVER_ENABLE
        // Add CSL-IE
        if ((s_radioVars.cslPeriod > 0) && !s_txOtFrame.mInfo.mTxInfo.mIsARetx && s_txOtFrame.mInfo.mTxInfo.mCslPresent)
        {
            otMacFrameSetCslIe(&s_txOtFrame, (uint16_t)s_radioVars.cslPeriod, samr21RadioCtrlCslGetPhase());
        }
#endif

#if OPENTHREAD_CONFIG_TIME_SYNC_ENABLE
        // Add Time-IE
        if (s_txOtFrame.mInfo.mTxInfo.mIeInfo->mTimeIeOffset != 0)
        {
            uint8_t *timeIe = s_txOtFrame.mPsdu + s_txOtFrame.mInfo.mTxInfo.mIeInfo->mTimeIeOffset;
            uint64_t time = samr21RtcGetTimestamp() + s_txOtFrame.mInfo.mTxInfo.mIeInfo->mNetworkTimeOffset;

            *timeIe = s_txOtFrame.mInfo.mTxInfo.mIeInfo->mTimeSyncSeq;

            *(++timeIe) = (uint8_t)(time & 0xff);
            for (uint8_t i = 1; i < sizeof(uint64_t); i++)
            {
                time = time >> 8;
                *(++timeIe) = (uint8_t)(time & 0xff);
            }
        }
#endif // OPENTHREAD_CONFIG_TIME_SYNC_ENABLE
    }

    
    if (s_txSecurityLevel && otMacFrameIsKeyIdMode1(&s_txOtFrame) && !s_txOtFrame.mInfo.mTxInfo.mIsSecurityProcessed)
    {
        s_radioVars.txState = SAMR21_RADIO_TX_STATE_SENDING_SECURITY;
        samr21TrxSendFrameAndApplyMacSecurity(
            &s_txFrameBuffer[0],
            s_txPayload,
            s_txFooter,
            s_txSecurityLevel,
            s_txAesKey,
            s_txNonce
        );
    }
    else
    {   
        s_radioVars.txState = SAMR21_RADIO_TX_STATE_SENDING_RAW;

        //Start Transmission
        samr21TrxSetSLP_TR(true);
        samr21TrxUploadToFramebuffer(s_txFrameBuffer, s_txFrameBuffer[0], 0);

        //Transmission Should have started by now
        samr21TrxSetSLP_TR(false);
    }

    s_radioVars.txState = SAMR21_RADIO_TX_STATE_WAIT_SENDING_END;
    // Set a handler for when the Frame is fully transmitted
    if(otMacFrameIsAckRequested(&s_txOtFrame))
    {
        samr21TrxSetIrqHandler(TRX_IRQ_TRX_END, samr21RadioPrepareForAckReception);
    }
    else
    {
        samr21TrxSetIrqHandler(TRX_IRQ_TRX_END, samr21RadioFinishTransmission);
    }
    
    // Set a Timeout in case the TrxEnd IRQ never happens
    samr21RadioQueueDelayedAction(IEEE_15_4_FRAME_SIZE * IEEE_15_4_24GHZ_TIME_PER_OCTET_us, samr21RadioRetryTransmission);

}

static void samr21RadioEvaluateCcaResult(){

    s_radioVars.txState = SAMR21_RADIO_TX_STATE_EVAL_CCA_RESULT;    
    //Clear Timeout
    samr21RadioRemoveQueuedAction();

    if (samr21TrxGetCcaResult())
    {
        //Channel is free
        samr21RadioStartTransmission();
        return;
    }
    
    //Channel is busy
    if(s_txNumCsmaBackoff < s_txOtFrame.mInfo.mTxInfo.mMaxCsmaBackoffs)
    {
        //Calculate Backoff Time and Queue Action
        uint32_t backoffTime_us =
            ((samr21TrxGetRandomByte() >> (8 - (IEEE_15_4_MIN_BACKOFF_EXPONENT + s_txNumCsmaBackoff > IEEE_15_4_MAX_BACKOFF_EXPONENT ? IEEE_15_4_MAX_BACKOFF_EXPONENT : IEEE_15_4_MIN_BACKOFF_EXPONENT + s_txNumCsmaBackoff))) - 1) * (20 * IEEE_15_4_24GHZ_TIME_PER_OCTET_us);
        
        s_txNumCsmaBackoff++;

        s_radioVars.txState = SAMR21_RADIO_TX_STATE_CSMA_BACKOFF;
        samr21RadioQueueDelayedAction(backoffTime_us, samr21RadioStartCca);
        return;
    }

    //No CSMA Attempts left
    samr21RadioAbortTransmission();
}

static void samr21RadioStartCca(){

    s_radioVars.txState = SAMR21_RADIO_TX_STATE_CCA;

    // Put Transceiver is in receive Mode
    samr21TrxQueueMoveToRx(true);

    // Set a Timeout in Case something goes wrong 
    samr21RadioQueueDelayedAction(SAMR21_SOFTWARE_RADIO_CCA_TIMEOUT_us, samr21RadioEvaluateCcaResult);

    //Start the measurement
    samr21TrxStartCca();

    s_radioVars.txState = SAMR21_RADIO_TX_STATE_WAIT_CCA_RESULT;
}

void samr21RadioTransmit(otRadioFrame *a_otFrame)
{

    s_txNumTransmissionRetries = 0;
    s_txNumCsmaBackoff = 0;
    s_txSecurityLevel = 0;
    s_txFrameBuffer[0] = s_txOtFrame.mLength;
    
    //Setup IRQ Handler for Transmit-Operation
    samr21TrxRemoveAllHandler();
    samr21RadioRemoveQueuedAction();

    s_radioVars.txBusy = true;

    //Change Channel to desired one
    samr21TrxSetChannel(s_txOtFrame.mChannel);

    // Enable CCA_ED IRQ for Channel Clear Assessment
    samr21TrxSetIrqHandler(TRX_IRQ_CCA_ED_DONE, samr21RadioEvaluateCcaResult);
    samr21TrxEnableIrq(TRX_IRQ_CCA_ED_DONE);

    // Enable Trx End IRQ to confirm Transmission
    samr21TrxEnableIrq(TRX_IRQ_TRX_END);

    // Enable Rx-Start IQ to indicate Ack Reception
    samr21TrxEnableIrq(TRX_IRQ_RX_START);


    //Check if Security needs to be applied by the Radio Driver
    if (    !s_txOtFrame.mInfo.mTxInfo.mIsARetx 
            && !s_txOtFrame.mInfo.mTxInfo.mIsHeaderUpdated 
            && otMacFrameIsSecurityEnabled(&s_txOtFrame)
    ){
        __disable_irq();
        otMacFrameSetFrameCounter(&s_txOtFrame, ++s_radioVars.macCounter);

        if (otMacFrameIsKeyIdMode1(&s_txOtFrame))
        {
            otMacFrameSetKeyId(&s_txOtFrame, s_radioVars.currentMacKeyId);
            s_txOtFrame.mInfo.mTxInfo.mAesKey = (const otMacKeyMaterial *)s_radioVars.currentMacKey;
            s_txAesKey = s_radioVars.currentMacKey;
        }
        __enable_irq();

        // Generate Nonce
        otMacGenerateNonce(&s_txOtFrame, (uint8_t*) &s_radioVars.extendedAddress, s_txNonce);

        //Set some Makers for AES-CCM
        s_txPayload = otMacFrameGetPayload(&s_txOtFrame);
        s_txFooter = otMacFrameGetFooter(&s_txOtFrame);
        s_txSecurityLevel = otMacFrameGetSecurityLevel(&s_txOtFrame);
    }


    //Inform Upper Layer that Transmission started
    cb_samr21RadioTransmissionStarted();

    //Check if Transmission needs to be delayed
    if(s_txOtFrame.mInfo.mTxInfo.mTxDelay){

        s_radioVars.txState = SAMR21_RADIO_TX_STATE_WAIT_FOR_TX_TIMING;

        samr21RadioCtrlQueueDelayedEventAbsolute(
            s_txOtFrame.mInfo.mTxInfo.mTxDelayBaseTime + s_txOtFrame.mInfo.mTxInfo.mTxDelay,
            s_txOtFrame.mInfo.mTxInfo.mCsmaCaEnabled ? samr21RadioStartCca : samr21RadioStartTransmission
        );

        return;
    }



    if(s_txOtFrame.mInfo.mTxInfo.mCsmaCaEnabled){
        samr21RadioStartCca();
        return;
    }

    samr21RadioStartTransmission();
}

otRadioFrame* samr21RadioGetOtTxBuffer(){

    while(s_radioVars.txBusy);

    return &s_txOtFrame;
}

otRadioFrame* samr21RadioGetLastReceivedAckFrame(){
    if(otMacFrameGetSequence(&s_txAckOtFrame) == otMacFrameGetSequence(&s_txOtFrame)){
        return &s_txAckOtFrame;
    }

    return NULL;
}


/******************************ED Operation***************************************/
static uint8_t s_maxEnergyDetectionLevel;
static uint32_t s_edScansLeft;

static void samr21RadioEvaluateEnergyScan(){
    
    //Remove Timeout
    samr21RadioRemoveQueuedAction();

    uint8_t edReading = samr21TrxReadRegister(PHY_ED_LEVEL_REG_ADDR);

    if(s_maxEnergyDetectionLevel < edReading){
        s_maxEnergyDetectionLevel = edReading;
    }

    if(--s_edScansLeft){
        //Set a Timeout if ED IRQ fails to trigger
        samr21RadioQueueDelayedAction((1000 / ( SAMR21_SOFTWARE_RADIO_ED_SCANS_PER_MS / 2 )), samr21RadioEvaluateEnergyScan);

        //Start ED
        samr21TrxStartEd(); 
        return;
    }

    cb_samr21RadioEnergyDetectionDone(AT86RF233_RSSI_BASE_VAL_dBm + s_maxEnergyDetectionLevel);

    s_radioVars.edBusy = false;
    //Move back to Receive
    samr21RadioReceive(0);
}


bool samr21RadioStartEnergyDetection(uint8_t a_channel, uint32_t a_duration_ms){

#ifdef _DEBUG
    assert(a_duration_ms > 0);
#endif

    if(s_radioVars.edBusy){
        //A Scan is already ongoing
        return false;
    }

    //Cleanup from previous Mode
    samr21TrxRemoveAllHandler();
    samr21RadioRemoveQueuedAction();

    s_radioVars.edBusy = true;

    s_maxEnergyDetectionLevel = 0;
    s_edScansLeft = a_duration_ms * SAMR21_SOFTWARE_RADIO_ED_SCANS_PER_MS;


    if (a_channel)
    {
        samr21TrxSetChannel(a_channel);
    }

    //Set a Handler for when the ED is done
    samr21TrxSetIrqHandler(TRX_IRQ_CCA_ED_DONE, samr21RadioEvaluateEnergyScan);
    samr21TrxEnableIrq(TRX_IRQ_CCA_ED_DONE);

    //Set a Timeout if ED IRQ fails to trigger
    samr21RadioQueueDelayedAction((1000 / ( SAMR21_SOFTWARE_RADIO_ED_SCANS_PER_MS / 2 )),samr21RadioEvaluateEnergyScan);

    //Start ED
    samr21TrxStartEd();

}

int8_t samr21RadioGetLastEdResult(){
    return AT86RF233_RSSI_BASE_VAL_dBm + s_maxEnergyDetectionLevel;
}
