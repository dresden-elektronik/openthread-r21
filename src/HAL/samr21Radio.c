/*
 * Copyright (c) 2023 dresden elektronik ingenieurtechnik gmbh.
 * All rights reserved.
 *
 * The software in this package is published under the terms of the BSD
 * style license a copy of which has been included with this distribution in
 * the LICENSE.txt file.
 *
 */
#include "samr21Radio.h"

#include "otUtilities_macFrame.h"
#include "otUtilities_sourceMatch.h"
#include "otUtilities_linkMetrics.h"



static struct radioVars_s
{
    bool        initiated;
    bool        rxActive;

    otShortAddress      shortAddress;
    otExtAddress          extendedAddress;
    otPanId                     panId;

    bool        txBusy;
    bool        rxBusy;
    bool        edBusy;

    bool        promiscuousMode;
    bool        framePendingSrcMatch;

    uint32_t    macCounter;

    uint8_t     currentMacKeyId;

    otMacKeyMaterial    previousMacKey;
    otMacKeyMaterial    currentMacKey;
    otMacKeyMaterial    nextMacKey;


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
static void (*s_currentTimerHandler_fktPtr)(void) = NULL;

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

/***************RTC-Handler for Operational Schedule-Timing***************************/
static void (*s_currentRtcHandler_fktPtr)(void) = NULL;

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


static void radio_timeoutHandler()
{
    *s_timeoutFlag = true;
    s_currentTimerHandler_fktPtr = NULL;
}

static void radio_startTimeoutTimer(uint16_t a_duration_us, bool *a_triggerFlag)
{
    s_timeoutFlag = a_triggerFlag;
    s_currentTimerHandler_fktPtr = &radio_timeoutHandler;
    samr21Timer4_startOneshot(a_duration_us);
}

static void radio_stopTimeoutTimer()
{
    samr21Timer4_stop();
    s_timeoutFlag = NULL;
    s_currentTimerHandler_fktPtr = NULL;
}

static void radio_queueDelayedAction(uint16_t a_delay_us, void (*a_queuedAction_fktPtr)(void))
{
    s_currentTimerHandler_fktPtr = a_queuedAction_fktPtr;
    samr21Timer4_startOneshot(a_delay_us);
}

static void radio_removeQueuedAction()
{
    s_currentTimerHandler_fktPtr = NULL;
    samr21Timer4_stop();
}


static void queueDelayedEventAbsolute(uint32_t a_triggerTimestamp, void (*a_queuedAction_fktPtr)(void))
{
    s_currentRtcHandler_fktPtr = &a_queuedAction_fktPtr;
    samr21Rtc_setAbsoluteAlarm(a_triggerTimestamp);
}

static void queueDelayedEventRelative(uint32_t a_duration, void (*a_queuedAction_fktPtr)(void))
{
    s_currentRtcHandler_fktPtr = &a_queuedAction_fktPtr;
    samr21Rtc_setRelativeAlarm(a_duration);
}

static void removeQueuedEvent()
{
    s_currentRtcHandler_fktPtr = NULL;
    samr21Rtc_disableAlarm();
}


void samr21Radio_enable()
{
    samr21Trx_initInterrupts();

    // Enable TRX-IRQ in NVIC
    __NVIC_EnableIRQ(EIC_IRQn);

    // Enable Radio Timer
    samr21Timer4_init(0,true,true); // 1MHz / (2^0) -> 1us resolution

    // HardwareTrx is always initiated cause the clk is sourced from the TRX
    s_radioVars.initiated = true;
}

void samr21Radio_disable()
{   
    // Disable IRQ in NVIC
    __NVIC_DisableIRQ(EIC_IRQn);

    radio_removeQueuedAction();
    samr21Trx_removeAllInterruptHandler();
    samr21Trx_disableAllInterrupts();

    // HardwareTrx is always initiated cause the clk is sourced from the TRX
    s_radioVars.initiated = false;
    s_radioVars.txBusy = false;
    s_radioVars.edBusy = false;
    s_radioVars.rxBusy = false;
    s_radioVars.initiated = false;
    s_radioVars.rxActive = false;
}

void samr21Radio_sleep()
{
    s_radioVars.rxActive = false;
    
    radio_removeQueuedAction();
    samr21Trx_removeAllInterruptHandler();

    s_radioVars.txBusy = false;
    s_radioVars.edBusy = false;
    s_radioVars.rxBusy = false;
}

otRadioState samr21Radio_getOtState(){
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

void samr21Radio_setPromiscuousMode(bool a_enable)
{
    s_radioVars.promiscuousMode = a_enable;
}

bool samr21Radio_getPromiscuousMode()
{
    return s_radioVars.promiscuousMode;
}

void samr21Radio_setMacFrameCounter(uint32_t a_macFrameCounter)
{
    s_radioVars.macCounter = a_macFrameCounter;
}
uint32_t samr21Radio_getMacFrameCounter()
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
    uint32_t curTime = samr21Rtc_getTimestamp();
    uint32_t cslPeriodInUs = s_radioVars.cslPeriod * OT_US_PER_TEN_SYMBOLS;
    uint32_t diff = (cslPeriodInUs - (curTime % cslPeriodInUs) + (s_radioVars.cslSampleTime % cslPeriodInUs)) % cslPeriodInUs;
    return (uint16_t)(diff / OT_US_PER_TEN_SYMBOLS + 1);
}

#endif

void samr21Radio_setMacKeys
(
    uint8_t a_keyId,
    const otMacKeyMaterial *a_previousKey_p,
    const otMacKeyMaterial *a_currentKey_p,
    const otMacKeyMaterial *a_nextKey_p
)
{
    __disable_irq();

    s_radioVars.currentMacKeyId = a_keyId;

    s_radioVars.previousMacKey= *a_previousKey_p;
    s_radioVars.currentMacKey= *a_currentKey_p;
    s_radioVars.nextMacKey= *a_nextKey_p;

    __enable_irq();

}

void samr21Radio_setPanId(otPanId a_panId){
    s_radioVars.panId = a_panId;
}
uint16_t samr21Radio_getPanId(){
    return s_radioVars.panId;
}

void samr21Radio_setShortAddress(otShortAddress a_shortAddress){
    s_radioVars.shortAddress = a_shortAddress;
}
uint16_t samr21Radio_getShortAddr(){
    return s_radioVars.shortAddress;
}

void samr21Radio_setExtendedAddress(const otExtAddress* a_extendedAddress_p){

   __disable_irq();
   for(uint8_t i = 0; i < sizeof(uint64_t); i++)
   {
        s_radioVars.extendedAddress.m8[i] = a_extendedAddress_p->m8[sizeof(uint64_t)-1-i];
   }
   __enable_irq();
}

uint8_t* samr21Radio_getExtendedAddress(){
    return (uint8_t*) &s_radioVars.extendedAddress.m8;
}


void samr21Radio_setFramePendingSrcMatch(bool a_enable)
{
    s_radioVars.framePendingSrcMatch = a_enable;
}




/******************************RX Operation***************************************/
//Prototype
static void rx_abortReception();
static void rx_finishReception();
static void rx_prepareImmediateAck();
static void rx_prepareEnhancedAck();
static void rx_downloadAndHandleRemaining();
static void rx_handleFcf();
static void rx_handleAddrField();
static void rx_sendAck();
static void rx_handleQueuedReceiveSlot();




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

static void rx_abortReception()
{

    s_radioVars.rxState = SAMR21_RADIO_RX_STATE_IDLE;

    s_radioVars.rxBusy = false;

    // Stop Timeout
    radio_removeQueuedAction();
    

    s_rxBuffer[s_activeRxBuffer].done = false;
    

    // Put TRX in receive again
    samr21Trx_queueMoveToRx(false);

    // Remove TrxEnd Handler
    samr21Trx_setInterruptHandler(TRX_IRQ_TRX_END, NULL);
}

static void rx_finishReception()
{
    s_radioVars.rxState = SAMR21_RADIO_RX_STATE_IDLE;

    s_radioVars.rxBusy = false;
    s_rxBuffer[s_activeRxBuffer].done = true;

    s_activeRxBuffer = (s_activeRxBuffer + 1) % SAMR21_NUM_RX_BUFFER;
    s_rxBuffer[s_activeRxBuffer].otFrame.mPsdu = &s_rxBuffer[s_activeRxBuffer].frameBuffer[1];
    s_rxBuffer[s_activeRxBuffer].done = false;

    // Stop Timeout
    radio_removeQueuedAction();

    // Remove TrxEnd Handler
    samr21Trx_setInterruptHandler(TRX_IRQ_TRX_END, NULL);

    // Put TRX in receive again
    samr21Trx_queueMoveToRx(false);

    samr21Radio_receptionDone_cb();
}

static void rx_sendAck()
{
    s_radioVars.rxState = SAMR21_RADIO_RX_STATE_SENDING_ACK;
    
    // Set a handler for when the Ack is successfully transmitted
    samr21Trx_setInterruptHandler(TRX_IRQ_TRX_END, rx_finishReception);

     //Start Transmission
    samr21Trx_setSleepTransmitPin(true);
    samr21Trx_startJustInTimeUploadToFramebuffer(s_rxAckFrameBuffer, s_rxAckFrameBuffer[0] + IEEE_15_4_PHR_SIZE , 0);

     //Transmission Should have started by now
    samr21Trx_setSleepTransmitPin(false);

    if (s_rxAckSecurityLevel)
    {
        otMacFrameProcessTransmitAesCcm(&s_rxOtAckFrame, &s_radioVars.extendedAddress);
        s_rxOtAckFrame.mInfo.mTxInfo.mIsSecurityProcessed = true;
    }

    s_radioVars.rxState = SAMR21_RADIO_RX_STATE_WAIT_ACK_END;

    // Set a Timeout in case the Trx never get triggered
    radio_queueDelayedAction(IEEE_15_4_FRAME_SIZE * IEEE_15_4_24GHZ_TIME_PER_OCTET_us, rx_abortReception);
}

// IEEE 802.15.4 2006
static void rx_prepareImmediateAck()
{

    otMacFrameGenerateImmAck(
        &(s_rxBuffer[s_activeRxBuffer].otFrame),
        s_rxBuffer[s_activeRxBuffer].framePending,
        &s_rxOtAckFrame
    );

    s_rxAckFrameBuffer[0] = s_rxOtAckFrame.mLength;
}

// IEEE 802.15.4 2015+
static void rx_prepareEnhancedAck()
{
    // Prepare Data for Header-IE
    uint8_t ieData[OT_ACK_IE_MAX_SIZE]; 
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

    otMacFrameGenerateEnhAck
    (
        &(s_rxBuffer[s_activeRxBuffer].otFrame),
        s_rxBuffer[s_activeRxBuffer].framePending,
        ieData,
        ieDataLen,
        &s_rxOtAckFrame
    );

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
            s_rxOtAckFrame.mInfo.mTxInfo.mAesKey = &s_radioVars.currentMacKey;
            s_rxBuffer[s_activeRxBuffer].otFrame.mInfo.mRxInfo.mAckKeyId = s_radioVars.currentMacKeyId;
        }
        __enable_irq();
    }
}


static void rx_downloadAndHandleRemaining()
{

    s_radioVars.rxState = SAMR21_RADIO_RX_STATE_PARSE_REMAINING;

    if (!samr21Trx_downloadReceivedFramebuffer(
            &s_rxBuffer[s_activeRxBuffer].otFrame.mLength,
            s_rxBuffer[s_activeRxBuffer].otFrame.mPsdu,
            &s_rxBuffer[s_activeRxBuffer].otFrame.mInfo.mRxInfo.mLqi,
            &s_rxBuffer[s_activeRxBuffer].otFrame.mInfo.mRxInfo.mRssi,
            true 
        )
    ){
        // Timeout or invalid Frame Length
        rx_abortReception();
        return;
    }



    bool ackRequested  = otMacFrameIsAckRequested(&(s_rxBuffer[s_activeRxBuffer].otFrame));

    // A Message has been fully received at this point
    if (s_radioVars.promiscuousMode || !ackRequested)
    {
        // No Acknowledgment was requested
        rx_finishReception();
        return;
    }

    // A Acknowledgment was requested
    // Send after Ack-InterFrame-Spacing delay
    radio_queueDelayedAction(IEEE_15_4_ADJUSTED_AIFS_us, rx_sendAck);


    s_radioVars.rxState = SAMR21_RADIO_RX_STATE_PREP_ACK;
    // Move TRX to Tx, so PLL is already dialed in when ack is about to be Transmitted
    samr21Trx_forceMoveToTx(false);

    if (otMacFrameIsVersion2015(&(s_rxBuffer[s_activeRxBuffer].otFrame)))
    {
        rx_prepareEnhancedAck();
    }
    else
    {
        rx_prepareImmediateAck();
    }
    s_radioVars.rxState = SAMR21_RADIO_RX_STATE_WAIT_ACK_START;
}

static void rx_handleAddrField()
{

    s_radioVars.rxState = SAMR21_RADIO_RX_STATE_PARSE_ADDR;

    if (!samr21Trx_realtimeFramebufferDownload(
            s_rxBuffer[s_activeRxBuffer].frameBuffer,
            s_rxBuffer[s_activeRxBuffer].neededPdsuSizeForNextAction))
    {
        // Timeout or invalid Frame Length
        rx_abortReception();
        return;
    }

    bool isRecipient = otMacFrameDoesAddrMatch(
                                  &(s_rxBuffer[s_activeRxBuffer].otFrame),
                                  s_radioVars.panId,
                                  s_radioVars.shortAddress,
                                  &s_radioVars.extendedAddress
                        );

    if (!s_radioVars.promiscuousMode && !isRecipient)
    {
        // Device is not recipient
        rx_abortReception();
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
        radio_queueDelayedAction(nextAction_us, rx_downloadAndHandleRemaining);
        return;
    }

    rx_downloadAndHandleRemaining();
}

static void rx_handleFcf()
{
    s_radioVars.rxBusy = true;
    s_radioVars.rxState = SAMR21_RADIO_RX_STATE_PARSE_FCF;
    

    s_rxBuffer[s_activeRxBuffer].otFrame.mInfo.mRxInfo.mTimestamp = samr21Rtc_getTimestamp();
    s_rxBuffer[s_activeRxBuffer].otFrame.mChannel = samr21Trx_getAktiveChannel(); 

    // +4 => 1Byte phyLen, 2Byte FCF, 1Byte DSN
    if (!samr21Trx_realtimeFramebufferDownload(s_rxBuffer[s_activeRxBuffer].frameBuffer, 4))
    { 
        // Timeout or invalid Frame Length
        rx_abortReception();
        return;
    }

    if(s_rxBuffer[s_activeRxBuffer].frameBuffer[0] > IEEE_15_4_FRAME_SIZE){
        //Can't be a valid frame
        rx_abortReception();
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
            radio_queueDelayedAction(nextAction_us, rx_handleAddrField);
            return;
        }
        rx_handleAddrField();
        return;
    }

    // Calculate the time when to start the next read Access to get the Addr Field
    uint32_t nextAction_us = s_rxBuffer[s_activeRxBuffer].otFrame.mLength * (IEEE_15_4_24GHZ_TIME_PER_OCTET_us - SAMR21_SPI_TIME_PER_BYTE_us);
    nextAction_us -= SAMR21_SPI_INIT_TIME_FRAMEBUFFER_us;

    s_rxBuffer[s_activeRxBuffer].neededPdsuSizeForNextAction = s_rxBuffer[s_activeRxBuffer].otFrame.mLength;

    if(nextAction_us > SAMR21_SOFTWARE_RADIO_MIN_DOWNLOAD_BACKOFF_us){
        s_radioVars.rxState = SAMR21_RADIO_RX_STATE_WAIT_REMAINING;
        radio_queueDelayedAction(nextAction_us, rx_downloadAndHandleRemaining);
        return;
    }

    rx_downloadAndHandleRemaining();
}


void samr21Radio_startReceiving(uint8_t a_channel)
{
    //Cleanup from previous Mode
    samr21Trx_removeAllInterruptHandler();
    radio_removeQueuedAction();

    s_radioVars.rxActive = 1;
    s_radioVars.rxBusy = 0;

    s_radioVars.rxState = SAMR21_RADIO_RX_STATE_IDLE;
    
    s_rxBuffer[s_activeRxBuffer].otFrame.mPsdu = &s_rxBuffer[s_activeRxBuffer].frameBuffer[1];
    s_rxBuffer[s_activeRxBuffer].done = false;

    if (a_channel)
    {
        samr21Trx_setActiveChannel(a_channel);
    }

    samr21Trx_setInterruptHandler(TRX_IRQ_RX_START, rx_handleFcf);

    // Enable Rx Start IRQ for reception
    samr21Trx_enableInterrupt(TRX_IRQ_RX_START);

    // Enable Trx End IRQ to confirm Ack Transmission
    samr21Trx_enableInterrupt(TRX_IRQ_TRX_END);

    // Enable Buffer Underrun for LIVE RX Parser
    samr21Trx_enableInterrupt(TRX_IRQ_TRX_UR);

    //Move to Rx
    samr21Trx_queueMoveToRx(true);
}

otRadioFrame* samr21Radio_getReceivedOtFrame(){
    for(uint8_t i = 0; i < SAMR21_NUM_RX_BUFFER; i++){
        if(s_rxBuffer[i].done){
            s_rxBuffer[i].done = false;
            return &(s_rxBuffer[i].otFrame);
        }
    }
    //No Finished Buffer available
    return NULL;
}

static void rx_handleReceiveSlotDone(){

    //Wait for Current Reception to finish
    while(s_radioVars.rxBusy);

    //Put Radio to sleep
    samr21Radio_sleep();

    if(s_radioVars.numReceivedFramesDuringSlot == 0){
        samr21Radio_noMessagesDuringSlot_cb();
    }

    s_radioVars.numReceivedFramesDuringSlot = 0;
    s_radioVars.receiveSlotChannel = 0;
    s_radioVars.receiveSlotDuration_us = 0;
}

static void rx_handleQueuedReceiveSlot(){

    s_radioVars.numReceivedFramesDuringSlot = 0;
    samr21Radio_startReceiving(s_radioVars.receiveSlotChannel);

    if(s_radioVars.receiveSlotDuration_us){
        queueDelayedEventRelative(s_radioVars.receiveSlotDuration_us, rx_handleReceiveSlotDone);
    }

}

bool samr21Radio_queueReceiveSlot(uint32_t a_startPointTimestamp, uint8_t a_channel, uint32_t a_duration){
    
    if(s_currentRtcHandler_fktPtr){

        //There already is a Queued Event
        return false;
    }

    s_radioVars.receiveSlotDuration_us = a_duration;
    s_radioVars.receiveSlotChannel = a_channel;

    queueDelayedEventAbsolute(a_startPointTimestamp, rx_handleQueuedReceiveSlot);

    return true;
}


/******************************TX Operation***************************************/
//Prototypes
static void tx_doneHandler();
static void tx_abort();
static void tx_evaluateAck();
static void tx_prepareForAckReception();
static void tx_retry();
static void tx_start();
static void tx_evalCca();
static void tx_startCca();

static uint8_t s_txFrameBuffer[IEEE_15_4_FRAME_SIZE];
static uint8_t s_txAckFrameBuffer[IEEE_15_4_FRAME_SIZE];
static otRadioFrame s_txOtFrame = { .mPsdu = &s_txFrameBuffer[1]};
static otRadioFrame s_txAckOtFrame = { .mPsdu = &s_txAckFrameBuffer[1]};

static uint8_t s_txNumTransmissionRetries;
static uint8_t s_txNumCsmaBackoff;


//Transmission Security
static uint8_t s_txNonce[IEEE_15_4_AES_CCM_NONCE_SIZE];
static uint8_t *s_txPayload;
static uint8_t *s_txFooter;
static uint8_t *s_txAesKey;
static uint8_t s_txSecurityLevel;


static void tx_doneHandler(){

    s_radioVars.txState = SAMR21_RADIO_TX_STATE_IDLE;

    //Stop Timeout
    radio_removeQueuedAction();

    s_radioVars.txBusy = false;

    //Move back to Receive
    samr21Radio_startReceiving(0);

    //Inform UpperLayer
    samr21Radio_transmissionDone_cb(RADIO_TRANSMISSION_SUCCESSFUL, &s_txOtFrame);
}

static void tx_abort(){
    
    s_radioVars.txState = SAMR21_RADIO_TX_STATE_IDLE;
    
    //Stop Timeout
    radio_removeQueuedAction();

    s_radioVars.txBusy = false;

    if(s_txNumCsmaBackoff >= s_txOtFrame.mInfo.mTxInfo.mMaxCsmaBackoffs)
    {
        samr21Radio_transmissionDone_cb(RADIO_TRANSMISSION_CHANNEL_ACCESS_FAILED, &s_txOtFrame);
    }
    else if(s_txNumTransmissionRetries >= s_txOtFrame.mInfo.mTxInfo.mMaxFrameRetries)
    {
        samr21Radio_transmissionDone_cb(RADIO_TRANSMISSION_NO_ACK, &s_txOtFrame);
    }
    else 
    {
        samr21Radio_transmissionDone_cb(RADIO_TRANSMISSION_UNDEFINED_ERROR, &s_txOtFrame);
    }

    //Move back to Receive
    samr21Radio_startReceiving(0);
}

static void tx_evaluateAck(){

    s_radioVars.txState = SAMR21_RADIO_TX_STATE_EVAL_ACK;

    //Stop Timeout
    radio_removeQueuedAction();

    //Download Ack
    bool validFrame = samr21Trx_downloadReceivedFramebuffer(
        &s_txAckOtFrame.mLength,
        s_txAckOtFrame.mPsdu,
        &s_txAckOtFrame.mInfo.mRxInfo.mLqi,
        &s_txAckOtFrame.mInfo.mRxInfo.mRssi,
        false
    );

    if (
        otMacFrameIsAck(&s_txAckOtFrame) 
        && ( otMacFrameGetSequence(&s_txAckOtFrame) == otMacFrameGetSequence(&s_txOtFrame) ) 
        && validFrame
    ){
        //Ack is Valid
        tx_doneHandler();
        return;
    }

    //Ack is Invalid
    tx_retry();
}

static void samr21RadioAckReceptionStarted(){

    //Add a Handler for when Ack is fully received
    samr21Trx_setInterruptHandler(TRX_IRQ_TRX_END, tx_evaluateAck);

    s_radioVars.txState = SAMR21_RADIO_TX_STATE_WAIT_FOR_ACK_END;

    //Add a Timestamp for the Ack reception started
    s_txAckOtFrame.mInfo.mRxInfo.mTimestamp = samr21Rtc_getTimestamp();
}


    
static void tx_retry(){
    if(s_txNumTransmissionRetries < s_txOtFrame.mInfo.mTxInfo.mMaxFrameRetries)
    {
        //Reset CSMA Counter
        s_txNumCsmaBackoff = 0;

        //Increase Retry Counter
        s_txNumTransmissionRetries++;

        //Start over
        tx_startCca();
        return;
    }

    //No Retry Attempts left
    tx_abort();
}





static void tx_start(){

    s_radioVars.txState = SAMR21_RADIO_TX_STATE_SENDING;

    samr21Trx_forceMoveToTx(true);

    //Start Transmission

    //Enter Time Critical Section
    __disable_irq();

    samr21Trx_setSleepTransmitPin(true);
    samr21Trx_startJustInTimeUploadToFramebuffer(s_txFrameBuffer, s_txFrameBuffer[0] + IEEE_15_4_PHR_SIZE , 0);

     //Transmission Should have started by now
    samr21Trx_setSleepTransmitPin(false);

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
            uint64_t time = samr21Rtc_getTimestamp() + s_txOtFrame.mInfo.mTxInfo.mIeInfo->mNetworkTimeOffset;

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

    
    if (otMacFrameIsSecurityEnabled(&s_txOtFrame) && otMacFrameIsKeyIdMode1(&s_txOtFrame) && !s_txOtFrame.mInfo.mTxInfo.mIsSecurityProcessed)
    {
        s_radioVars.txState = SAMR21_RADIO_TX_STATE_SENDING_SECURITY;
        otMacFrameProcessTransmitAesCcm(&s_txOtFrame, &s_radioVars.extendedAddress);
        s_txOtFrame.mInfo.mTxInfo.mIsSecurityProcessed = true;
    }
    else
    {   
        s_radioVars.txState = SAMR21_RADIO_TX_STATE_SENDING_RAW;
    }

    //Leaving Time Critical Section
    __enable_irq();

    // Set a handler for when the Frame is fully transmitted
    if(otMacFrameIsAckRequested(&s_txOtFrame))
    {   
        //Add a Handler for when Ack is received
        samr21Trx_setInterruptHandler(TRX_IRQ_RX_START, samr21RadioAckReceptionStarted);
        //Queue Radio Mode Change to RX after Transmission is completed
        samr21Trx_queueMoveToRx(false);

        //Add a Timeout in case there is no Ack
        radio_queueDelayedAction(SAMR21_SOFTWARE_RADIO_ACK_TIMEOUT_us, tx_retry);
        s_radioVars.rxState = SAMR21_RADIO_RX_STATE_WAIT_ACK_START;
    }
    else
    {
        samr21Trx_setInterruptHandler(TRX_IRQ_TRX_END, tx_doneHandler);
        // Set a Timeout in case the TrxEnd IRQ never happens
        radio_queueDelayedAction(SAMR21_SOFTWARE_RADIO_TRX_END_TIMEOUT_us, tx_retry);
        s_radioVars.txState = SAMR21_RADIO_TX_STATE_WAIT_SENDING_END;
    }

    //Inform Upper Layer that Transmission started
    samr21Radio_transmissionStarted_cb( &s_txOtFrame);
}

static void tx_evalCca(){

    s_radioVars.txState = SAMR21_RADIO_TX_STATE_EVAL_CCA_RESULT;    
    //Clear Timeout
    radio_removeQueuedAction();

    if (samr21Trx_getCcaResult())
    {
        //Channel is free
        tx_start();
        return;
    }
    
    //Channel is busy
    if(s_txNumCsmaBackoff < s_txOtFrame.mInfo.mTxInfo.mMaxCsmaBackoffs)
    {
        //Calculate Backoff Time and Queue Action
        uint32_t backoffTime_us =
            ((samr21Trx_getRandomByte() >> (8 - (IEEE_15_4_MIN_BACKOFF_EXPONENT + s_txNumCsmaBackoff > IEEE_15_4_MAX_BACKOFF_EXPONENT ? IEEE_15_4_MAX_BACKOFF_EXPONENT : IEEE_15_4_MIN_BACKOFF_EXPONENT + s_txNumCsmaBackoff))) - 1) * (20 * IEEE_15_4_24GHZ_TIME_PER_OCTET_us);
        
        s_txNumCsmaBackoff++;

        s_radioVars.txState = SAMR21_RADIO_TX_STATE_CSMA_BACKOFF;
        radio_queueDelayedAction(backoffTime_us, tx_startCca);
        return;
    }

    //No CSMA Attempts left
    tx_abort();
}

static void tx_startCca(){

    s_radioVars.txState = SAMR21_RADIO_TX_STATE_CCA;

    // Put Transceiver is in receive Mode
    samr21Trx_queueMoveToRx(true);

    // Set a Timeout in Case something goes wrong 
    radio_queueDelayedAction(SAMR21_SOFTWARE_RADIO_CCA_TIMEOUT_us, tx_evalCca);

    //Start the measurement
    samr21Trx_startCca();

    s_radioVars.txState = SAMR21_RADIO_TX_STATE_WAIT_CCA_RESULT;
}

void samr21radio_transmit(otRadioFrame *a_otFrame)
{

    s_txNumTransmissionRetries = 0;
    s_txNumCsmaBackoff = 0;
    s_txSecurityLevel = 0;
    s_txFrameBuffer[0] = s_txOtFrame.mLength;
    
    //Setup IRQ Handler for Transmit-Operation
    samr21Trx_removeAllInterruptHandler();
    radio_removeQueuedAction();

    s_radioVars.txBusy = true;

    //Change Channel to desired one
    samr21Trx_setActiveChannel(s_txOtFrame.mChannel);

    // Enable CCA_ED IRQ for Channel Clear Assessment
    samr21Trx_setInterruptHandler(TRX_IRQ_CCA_ED_DONE, tx_evalCca);
    samr21Trx_enableInterrupt(TRX_IRQ_CCA_ED_DONE);

    // Enable Trx End IRQ to confirm Transmission
    samr21Trx_enableInterrupt(TRX_IRQ_TRX_END);

    // Enable Rx-Start IRQ to indicate Ack Reception
    samr21Trx_enableInterrupt(TRX_IRQ_RX_START);


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
            s_txOtFrame.mInfo.mTxInfo.mAesKey =  &s_radioVars.currentMacKey;
        }
        __enable_irq();
    }

    //Check if Transmission needs to be delayed
    if(s_txOtFrame.mInfo.mTxInfo.mTxDelay){

        s_radioVars.txState = SAMR21_RADIO_TX_STATE_WAIT_FOR_TX_TIMING;

        queueDelayedEventAbsolute(
            s_txOtFrame.mInfo.mTxInfo.mTxDelayBaseTime + s_txOtFrame.mInfo.mTxInfo.mTxDelay,
            s_txOtFrame.mInfo.mTxInfo.mCsmaCaEnabled ? tx_startCca : tx_start
        );
        return;
    }



    if(s_txOtFrame.mInfo.mTxInfo.mCsmaCaEnabled){
        tx_startCca();
        return;
    }

    tx_start();
}

otRadioFrame* samr21RadioGetOtTxBuffer(){

    while(s_radioVars.txBusy);

    return &s_txOtFrame;
}

otRadioFrame* samr21RadioGetLastReceivedAckFrame()
{
    if(otMacFrameGetSequence(&s_txAckOtFrame) == otMacFrameGetSequence(&s_txOtFrame)){
        return &s_txAckOtFrame;
    }

    return NULL;
}


/******************************ED Operation***************************************/
static uint8_t s_maxEnergyDetectionLevel;
static uint32_t s_edScansLeft;

static void ed_evaluateScan(){
    
    //Remove Timeout
    radio_removeQueuedAction();

    uint8_t edReading = samr21Trx_readRegister(PHY_ED_LEVEL_REG_ADDR);

    if(s_maxEnergyDetectionLevel < edReading){
        s_maxEnergyDetectionLevel = edReading;
    }

    if(--s_edScansLeft){
        //Set a Timeout if ED IRQ fails to trigger
        radio_queueDelayedAction((1000 / ( SAMR21_SOFTWARE_RADIO_ED_SCANS_PER_MS / 2 )), ed_evaluateScan);

        //Start ED
        samr21Trx_startEd(); 
        return;
    }

    samr21Radio_energyDetectionDone_cb(AT86RF233_RSSI_BASE_VAL_dBm + s_maxEnergyDetectionLevel);

    s_radioVars.edBusy = false;
    //Move back to Receive
    samr21Radio_startReceiving(0);
}


bool samr21Radio_startEnergyDetection(uint8_t a_channel, uint32_t a_duration_ms){

#ifdef _DEBUG
    assert(a_duration_ms > 0);
#endif

    if(s_radioVars.edBusy){
        //A Scan is already ongoing
        return false;
    }

    //Cleanup from previous Mode
    samr21Trx_removeAllInterruptHandler();
    radio_removeQueuedAction();

    s_radioVars.edBusy = true;

    s_maxEnergyDetectionLevel = 0;
    s_edScansLeft = a_duration_ms * SAMR21_SOFTWARE_RADIO_ED_SCANS_PER_MS;


    if (a_channel)
    {
        samr21Trx_setActiveChannel(a_channel);
    }

    //Set a Handler for when the ED is done
    samr21Trx_setInterruptHandler(TRX_IRQ_CCA_ED_DONE, ed_evaluateScan);
    samr21Trx_enableInterrupt(TRX_IRQ_CCA_ED_DONE);

    //Set a Timeout if ED IRQ fails to trigger
    radio_queueDelayedAction((1000 / ( SAMR21_SOFTWARE_RADIO_ED_SCANS_PER_MS / 2 )),ed_evaluateScan);

    //Start ED
    samr21Trx_startEd();

}

int8_t samr21Radio_getLastEdResult(){
    return AT86RF233_RSSI_BASE_VAL_dBm + s_maxEnergyDetectionLevel;
}
