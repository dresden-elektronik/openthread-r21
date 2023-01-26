// Author Eric Härtel @ dresden elektronik ingenieurtechnik gmbh © 2022

#include "openthread/platform/radio.h"

#include "samr21RadioCtrl.h"
#include "samr21RadioRxHandler.h"
#include "samr21RadioTxHandler.h"
#include "samr21RadioEdHandler.h"
#include "samr21RadioAddrMatch.h"
#include "soft_source_match_table.h"
#include "samr21Nvm.h"


static otRadioState s_radioState = OT_RADIO_STATE_DISABLED;
static int8_t s_lastEdResult = INT8_MAX;

static bool s_pendingRxBuffer;

otInstance * s_instance = NULL;

void otPlatRadioGetIeeeEui64(otInstance *aInstance, uint8_t *aIeeeEui64)
{
    uint64_t ieeeAddr = samr21NvmGetIeeeAddr();

    for (uint8_t i = 0; i < sizeof(uint64_t); i++)
    {
        aIeeeEui64[i] = ((uint8_t *)&ieeeAddr)[i];
    }
}

otRadioState otPlatRadioGetState(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);
    return (otRadioState)samr21RadioCtrlGetState();
}

void otPlatRadioSetExtendedAddress(otInstance *aInstance, const otExtAddress *aAddress)
{
    OT_UNUSED_VARIABLE(aInstance);
    samr21RadioSetIeeeAddr(aAddress->m8);
}

void otPlatRadioSetShortAddress(otInstance *aInstance, uint16_t aAddress)
{
    OT_UNUSED_VARIABLE(aInstance);

    samr21RadioSetShortAddr(aAddress);
}

void otPlatRadioSetPanId(otInstance *aInstance, uint16_t aPanId)
{
    OT_UNUSED_VARIABLE(aInstance);

    samr21RadioSetPanId(aPanId);

#if RADIO_CONFIG_SRC_MATCH_SHORT_ENTRY_NUM || RADIO_CONFIG_SRC_MATCH_EXT_ENTRY_NUM
    utilsSoftSrcMatchSetPanId(aPanId);
#endif // RADIO_CONFIG_SRC_MATCH_SHORT_ENTRY_NUM || RADIO_CONFIG_SRC_MATCH_EXT_ENTRY_NUM}
}

otError otPlatRadioEnable(otInstance *aInstance)
{
    s_instance = aInstance;
    // Always on, cause the TRX-Clock is used as the ref Clock for the MCU-Clocks
    samr21RadioCtrlSetState(SAMR21_RADIO_STATE_SLEEP);

    return OT_ERROR_NONE;
}

otError otPlatRadioDisable(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);

    // Always on, cause the TRX-Clock is used as the ref Clock for the MCU-Clocks
    samr21RadioCtrlSetState(SAMR21_RADIO_STATE_DISABLED);

    return OT_ERROR_NONE;
}

otError otPlatRadioSetCcaEnergyDetectThreshold(otInstance *aInstance, int8_t aThreshold)
{
    OT_UNUSED_VARIABLE(aInstance);

    samr21RadioCtrlSetCcaThreshold(aThreshold);

    return OT_ERROR_NONE;
}

otError otPlatRadioGetCcaEnergyDetectThreshold(otInstance *aInstance, int8_t *aThreshold)
{
    OT_UNUSED_VARIABLE(aInstance);

    *(aThreshold) = samr21RadioCtrlGetCurrentCcaThreshold();

    return OT_ERROR_NONE;
}

otError otPlatRadioGetTransmitPower(otInstance *aInstance, int8_t *aPower)
{
    OT_UNUSED_VARIABLE(aInstance);

    *(aPower) = samr21RadioCtrlGetTxPower();
}

otError otPlatRadioSetTransmitPower(otInstance *aInstance, int8_t aPower)
{
    OT_UNUSED_VARIABLE(aInstance);

    samr21RadioCtrlSetTxPower(aPower);

    return OT_ERROR_NONE;
}

int8_t otPlatRadioGetReceiveSensitivity(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);

    return AT86RF233_RSSI_BASE_VAL;
}

bool otPlatRadioIsEnabled(otInstance *aInstance)
{
    Samr21RadioState state = samr21RadioCtrlGetState();

    if (
        state == SAMR21_RADIO_STATE_DISABLED || state == SAMR21_RADIO_STATE_INVALID)
    {
        return false;
    }
    return true;
}

otRadioCaps otPlatRadioGetCaps(otInstance *aInstance)
{

    OT_UNUSED_VARIABLE(aInstance);

    return OT_RADIO_CAPS_ACK_TIMEOUT | OT_RADIO_CAPS_ENERGY_SCAN | OT_RADIO_CAPS_TRANSMIT_RETRIES | OT_RADIO_CAPS_CSMA_BACKOFF | OT_RADIO_CAPS_SLEEP_TO_TX | OT_RADIO_CAPS_TRANSMIT_SEC | OT_RADIO_CAPS_TRANSMIT_TIMING | OT_RADIO_CAPS_RECEIVE_TIMING;
}

otError otPlatRadioSetFemLnaGain(otInstance *aInstance, int8_t aGain)
{

    OT_UNUSED_VARIABLE(aInstance);
    OT_UNUSED_VARIABLE(aGain);

    return OT_ERROR_NOT_IMPLEMENTED;
}

otError otPlatRadioGetFemLnaGain(otInstance *aInstance, int8_t *aGain)
{

    OT_UNUSED_VARIABLE(aInstance);
    OT_UNUSED_VARIABLE(aGain);

    return OT_ERROR_NOT_IMPLEMENTED;
}

void otPlatRadioSetPromiscuous(otInstance *aInstance, bool aEnable)
{

    OT_UNUSED_VARIABLE(aInstance);

    samr21RadioCtrlSetPromiscuousMode(aEnable);
}

bool otPlatRadioGetPromiscuous(otInstance *aInstance)
{

    OT_UNUSED_VARIABLE(aInstance);

    return samr21RadioCtrlGetPromiscuousMode();
}

#if OPENTHREAD_CONFIG_MAC_CSL_RECEIVER_ENABLE

otError otPlatRadioEnableCsl(otInstance *aInstance,
                             uint32_t aCslPeriod,
                             otShortAddress aShortAddr,
                             const otExtAddress *aExtAddr)
{

    OT_UNUSED_VARIABLE(aInstance);
    OT_UNUSED_VARIABLE(aInstance);
    OT_UNUSED_VARIABLE(aInstance);

    g_cslPeriod = aCslPeriod;
}

void otPlatRadioUpdateCslSampleTime(otInstance *aInstance, uint32_t aCslSampleTime)
{
    OT_UNUSED_VARIABLE(aInstance);

    g_cslSampleTime = aCslSampleTime;
}

uint8_t otPlatRadioGetCslAccuracy(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);

    return 30; //
}

uint8_t otPlatRadioGetCslClockUncertainty(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);

    return 1;
}

#endif // OPENTHREAD_CONFIG_MAC_CSL_RECEIVER_ENABLE

otError otPlatRadioSetChannelMaxTransmitPower(otInstance *aInstance, uint8_t aChannel, int8_t aMaxPower)
{

    OT_UNUSED_VARIABLE(aInstance);
    OT_UNUSED_VARIABLE(aChannel);
    OT_UNUSED_VARIABLE(aMaxPower);

    return OT_ERROR_NOT_IMPLEMENTED;
}

otError otPlatRadioSetRegion(otInstance *aInstance, uint16_t aRegionCode)
{

    OT_UNUSED_VARIABLE(aInstance);
    OT_UNUSED_VARIABLE(aRegionCode);

    return OT_ERROR_NOT_IMPLEMENTED;
}

otError otPlatRadioGetRegion(otInstance *aInstance, uint16_t *aRegionCode)
{

    OT_UNUSED_VARIABLE(aInstance);
    OT_UNUSED_VARIABLE(aRegionCode);

    return OT_ERROR_NOT_IMPLEMENTED;
}


uint64_t otPlatRadioGetNow(otInstance *aInstance){
    
    OT_UNUSED_VARIABLE(aInstance);

    return samr21RtcGetTimestamp();
}

uint32_t otPlatRadioGetBusSpeed(otInstance *aInstance){

    OT_UNUSED_VARIABLE(aInstance);

    return 2000000; //actully 8Mbits but bus is shared with AES Engine.
}

#if OPENTHREAD_CONFIG_MLE_LINK_METRICS_SUBJECT_ENABLE
otError otPlatRadioConfigureEnhAckProbing(otInstance *aInstance,
                                          otLinkMetrics aLinkMetrics,
                                          const otShortAddress aShortAddress,
                                          const otExtAddress *aExtAddress)
{
    OT_UNUSED_VARIABLE(aInstance);
    OT_UNUSED_VARIABLE(aLinkMetrics);
    OT_UNUSED_VARIABLE(aShortAddress);
    OT_UNUSED_VARIABLE(aExtAddress);

    otError error = OT_ERROR_NONE;

    error = otLinkMetricsConfigureEnhAckProbing(aShortAddress, aExtAddress, aLinkMetrics);
    otEXPECT(error == OT_ERROR_NONE);

exit:
    return error;
}
#endif

#if OPENTHREAD_CONFIG_THREAD_VERSION >= OT_THREAD_VERSION_1_2
void otPlatRadioEnableSrcMatch(otInstance *aInstance, bool aEnable)
{
    // UTILS SOFT-TABLE USED
    OT_UNUSED_VARIABLE(aInstance);
    OT_UNUSED_VARIABLE(aEnable);

    return;
}

void otPlatRadioSetMacKey(otInstance *aInstance,
                          uint8_t aKeyIdMode,
                          uint8_t aKeyId,
                          const otMacKeyMaterial *aPrevKey,
                          const otMacKeyMaterial *aCurrKey,
                          const otMacKeyMaterial *aNextKey,
                          otRadioKeyType aKeyType)
{
    OT_UNUSED_VARIABLE(aInstance);
    OT_UNUSED_VARIABLE(aKeyIdMode);

//    assert(aKeyType == OT_KEY_TYPE_LITERAL_KEY);
//    assert(aPrevKey != NULL && aCurrKey != NULL && aNextKey != NULL);

    samr21RadioCtrlSetMacKeys(aKeyId,
                              aPrevKey->mKeyMaterial.mKey.m8,
                              aCurrKey->mKeyMaterial.mKey.m8,
                              aNextKey->mKeyMaterial.mKey.m8);
}

void otPlatRadioSetMacFrameCounter(otInstance *aInstance, uint32_t aMacFrameCounter){
    
    OT_UNUSED_VARIABLE(aInstance);
    samr21RadioCtrlSetMacFrameCounter(aMacFrameCounter);
}

otError otPlatRadioReceiveAt(otInstance *aInstance, uint8_t aChannel, uint32_t aStart, uint32_t aDuration){
    
    OT_UNUSED_VARIABLE(aInstance);
    
    if(samr21RadioRxIsReceiveSlotPlanned()){
        return OT_ERROR_FAILED;
    }

    samr21RadioRxSetup(aChannel, aDuration, aStart);
    
    return OT_ERROR_NONE;
}
#endif

static bool s_edScanDone;
static bool s_edScanDone;
static bool s_edScanDone;

otError otPlatRadioEnergyScan(otInstance *aInstance, uint8_t aScanChannel, uint16_t aScanDuration)
{

    OT_UNUSED_VARIABLE(aInstance);

    if (samr21RadioEdStart(aScanChannel, aScanDuration))
    {
        return OT_ERROR_NONE;
        s_lastEdResult = INT8_MAX;
    }

    return OT_ERROR_BUSY;
}


int8_t otPlatRadioGetRssi(otInstance *aInstance){
    
    OT_UNUSED_VARIABLE(aInstance);

    
}

otError otPlatRadioReceive(otInstance *aInstance, uint8_t aChannel){

    OT_UNUSED_VARIABLE(aInstance);

    samr21RadioCtrlSetState(SAMR21_RADIO_STATE_RECEIVE);
    samr21RadioRxSetup(aChannel, 0, 0);
    
    return OT_ERROR_NONE;
}

otError otPlatRadioSleep(otInstance *aInstance){

    OT_UNUSED_VARIABLE(aInstance);

    samr21RadioCtrlSetState(SAMR21_RADIO_STATE_SLEEP);
    samr21RadioRxAbort(); 
    samr21RadioCtrlReturnToLastHandler();
    
    return OT_ERROR_NONE;
}

otRadioFrame *otPlatRadioGetTransmitBuffer(otInstance *aInstance){

    OT_UNUSED_VARIABLE(aInstance);

    return samr21RadioTxGetOtBuffer();
}

otError otPlatRadioTransmit(otInstance *aInstance, otRadioFrame *aFrame){
    
    OT_UNUSED_VARIABLE(aInstance);
    
    samr21RadioCtrlSetState(SAMR21_RADIO_STATE_TRANSMIT);
    samr21RadioTxSetup();

    return OT_ERROR_NONE;
}


void cb_samr21RadioEdDone(int8_t rssi)
{
    s_lastEdResult=rssi;
}

void cb_samr21RadioRxDone(RxBuffer* buffer){
    s_pendingRxBuffer=true;
}

void cb_samr21RadioRxRecivedNothing(){

}

void cb_samr21RadioTxDone(otRadioFrame* txFrameBuffer, otRadioFrame* txAckFrameBuffer){
    otPlatRadioTxDone(s_instance, txFrameBuffer, txAckFrameBuffer, OT_ERROR_NONE);
}

void cb_samr21RadioTxStarted(otRadioFrame* txFrameBuffer){
    otPlatRadioTxStarted(s_instance, txFrameBuffer);
}

void cb_samr21RadioTxFailed(otRadioFrame* txFrameBuffer, TxStatus failedAt){

    otError error = OT_ERROR_ABORT;

    if(failedAt == TX_STATUS_CCA){
        error = OT_ERROR_CHANNEL_ACCESS_FAILURE;
    }

    if(failedAt == TX_STATUS_WAIT_FOR_ACK || failedAt == TX_STATUS_RECIVING_ACK){
        error = OT_ERROR_NO_ACK;
    }
    
    otPlatRadioTxDone(s_instance, txFrameBuffer, NULL, error);
}

void samr21OtPlatRadioTask(){
    if(s_lastEdResult != INT8_MAX){
        otPlatRadioEnergyScanDone(s_instance, s_lastEdResult);
        s_lastEdResult = INT8_MAX;
    }

    if(s_pendingRxBuffer){
        RxBuffer * buffer = samr21RadioRxGetPendingRxBuffer();

        if(buffer == NULL){
           s_pendingRxBuffer = 0;
           return;
        }

        otPlatRadioReceiveDone(s_instance,&buffer->otFrame,OT_ERROR_NONE);
        buffer->status = RX_STATUS_IDLE;
    }
}