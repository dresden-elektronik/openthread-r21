// Author Eric Härtel @ dresden elektronik ingenieurtechnik gmbh © 2022

#include "openthread/platform/radio.h"

#include "otUtilities_macFrame.h"
#include "otUtilities_sourceMatch.h"
#include "otUtilities_linkMetrics.h"

#include "samr21Nvm.h"
#include "samr21Radio.h"
#include "samr21Trx.h"
#include "samr21Rtc.h"

static otInstance *s_instance_p = NULL;

// Radio Configuration

otRadioCaps otPlatRadioGetCaps(otInstance *a_instance_p)
{

    OT_UNUSED_VARIABLE(a_instance_p);

    return OT_RADIO_CAPS_ACK_TIMEOUT | OT_RADIO_CAPS_ENERGY_SCAN | OT_RADIO_CAPS_TRANSMIT_RETRIES | OT_RADIO_CAPS_CSMA_BACKOFF | OT_RADIO_CAPS_SLEEP_TO_TX | OT_RADIO_CAPS_TRANSMIT_SEC | OT_RADIO_CAPS_TRANSMIT_TIMING | OT_RADIO_CAPS_RECEIVE_TIMING;
}

// TODO
//  const char *otPlatRadioGetVersionString(otInstance *a_instance_p){
//     OT_UNUSED_VARIABLE(a_instance_p);
//  }

int8_t otPlatRadioGetReceiveSensitivity(otInstance *a_instance_p)
{
    OT_UNUSED_VARIABLE(a_instance_p);

#if defined(TARGET_DEVICE) && ((TARGET_DEVICE == CONBEE2) || (TARGET_DEVICE == RASPBEE2))
    // Frontend gives 5dBm Rx Sensitivity
    return AT86RF233_RSSI_BASE_VAL_dBm - 5;
#else
    return AT86RF233_RSSI_BASE_VAL_dBm;
#endif
}

void otPlatRadioGetIeeeEui64(otInstance *a_instance_p, uint8_t *a_ieeeEui64_p)
{
    OT_UNUSED_VARIABLE(a_instance_p);

    uint8_t wrongEndianIeeeAddr[IEEE_15_4_EXTENDED_ADDR_SIZE];
    samr21Nvm_getIeeeAddr(wrongEndianIeeeAddr);

    for (unsigned int  i = 0; i < IEEE_15_4_EXTENDED_ADDR_SIZE; i++)
    {
        a_ieeeEui64_p[IEEE_15_4_EXTENDED_ADDR_SIZE - 1 - i] = wrongEndianIeeeAddr[i];
    }
}

void otPlatRadioSetPanId(otInstance *a_instance_p, otPanId a_panId)
{
    OT_UNUSED_VARIABLE(a_instance_p);

    samr21Radio_setPanId(a_panId);

#if RADIO_CONFIG_SRC_MATCH_SHORT_ENTRY_NUM || RADIO_CONFIG_SRC_MATCH_EXT_ENTRY_NUM
    utilsSoftSrcMatchSetPanId(a_panId);
#endif // RADIO_CONFIG_SRC_MATCH_SHORT_ENTRY_NUM || RADIO_CONFIG_SRC_MATCH_EXT_ENTRY_NUM}
}

void otPlatRadioSetExtendedAddress(otInstance *a_instance_p, const otExtAddress *a_address_p)
{
    OT_UNUSED_VARIABLE(a_instance_p);

    samr21Radio_setExtendedAddress(a_address_p);
}

void otPlatRadioSetShortAddress(otInstance *a_instance_p, uint16_t a_address_p)
{
    OT_UNUSED_VARIABLE(a_instance_p);

    samr21Radio_setShortAddress(a_address_p);
}

otError otPlatRadioGetTransmitPower(otInstance *a_instance_p, int8_t *a_power_p)
{
    OT_UNUSED_VARIABLE(a_instance_p);

    *a_power_p = samr21Trx_getTxPower();

    return OT_ERROR_NONE;
}

otError otPlatRadioSetTransmitPower(otInstance *a_instance_p, int8_t a_power)
{
    OT_UNUSED_VARIABLE(a_instance_p);

    samr21Trx_setTxPower(a_power);

    return OT_ERROR_NONE;
}

otError otPlatRadioGetCcaEnergyDetectThreshold(otInstance *a_instance_p, int8_t *a_threshold_p)
{
    OT_UNUSED_VARIABLE(a_instance_p);

    *a_threshold_p = samr21Trx_getCcaThreshold();

    return OT_ERROR_NONE;
}

otError otPlatRadioSetCcaEnergyDetectThreshold(otInstance *a_instance_p, int8_t a_threshold)
{
    OT_UNUSED_VARIABLE(a_instance_p);

    samr21Trx_setCcaThreshold(a_threshold);

    return OT_ERROR_NONE;
}

// TODO
otError otPlatRadioGetFemLnaGain(otInstance *a_instance_p, int8_t *a_gain_p)
{
    OT_UNUSED_VARIABLE(a_instance_p);
    OT_UNUSED_VARIABLE(a_gain_p);

    return OT_ERROR_NOT_IMPLEMENTED;
}

// TODO
otError otPlatRadioSetFemLnaGain(otInstance *a_instance_p, int8_t a_gain_p)
{
    OT_UNUSED_VARIABLE(a_instance_p);
    OT_UNUSED_VARIABLE(a_gain_p);

    return OT_ERROR_NOT_IMPLEMENTED;
}

bool otPlatRadioGetPromiscuous(otInstance *a_instance_p)
{
    OT_UNUSED_VARIABLE(a_instance_p);

    return samr21Radio_getPromiscuousMode();
}

void otPlatRadioSetPromiscuous(otInstance *a_instance_p, bool a_enable)
{
    OT_UNUSED_VARIABLE(a_instance_p);

    samr21Radio_setPromiscuousMode(a_enable);
}

void otPlatRadioSetMacKey(otInstance *a_instance_p,
                          uint8_t a_keyIdMode,
                          uint8_t a_keyId,
                          const otMacKeyMaterial *a_prevKey_p,
                          const otMacKeyMaterial *a_currKey_p,
                          const otMacKeyMaterial *a_nextKey_p,
                          otRadioKeyType a_keyType)
{
    OT_UNUSED_VARIABLE(a_instance_p);

#ifdef _DEBUG
    assert(a_keyIdMode == 1);
    assert(a_keyType == OT_KEY_TYPE_LITERAL_KEY);
    assert(a_prevKey != NULL && a_currKey != NULL && a_nextKey != NULL);
#else
    OT_UNUSED_VARIABLE(a_keyIdMode);
    OT_UNUSED_VARIABLE(a_keyType);
#endif

    samr21Radio_setMacKeys
    (
        a_keyId,
        a_prevKey_p,
        a_currKey_p,
        a_nextKey_p
    );
}

void otPlatRadioSetMacFrameCounter(otInstance *a_instance_p, uint32_t a_macFrameCounter)
{
    OT_UNUSED_VARIABLE(a_instance_p);

    samr21Radio_setMacFrameCounter(a_macFrameCounter);
}

void otPlatRadioSetMacFrameCounterIfLarger(otInstance *a_instance_p, uint32_t a_macFrameCounter)
{
    OT_UNUSED_VARIABLE(a_instance_p);

    if (samr21Radio_getMacFrameCounter() < a_macFrameCounter)
    {
        samr21Radio_setMacFrameCounter(a_macFrameCounter);
    }
}

uint64_t otPlatRadioGetNow(otInstance *a_instance_p)
{
    OT_UNUSED_VARIABLE(a_instance_p);
    return samr21Rtc_getTimestamp();
}

uint32_t otPlatRadioGetBusSpeed(otInstance *a_instance_p)
{
    OT_UNUSED_VARIABLE(a_instance_p);

    // Return 0 when the MAC and above layer and Radio layer resides on the same chip.
    return 0;
}

// Radio Operation

otRadioState otPlatRadioGetState(otInstance *a_instance_p)
{
    OT_UNUSED_VARIABLE(a_instance_p);

    return samr21Radio_getOtState();
}

otError otPlatRadioEnable(otInstance *a_instance_p)
{
    s_instance_p = a_instance_p;

    samr21Radio_enable();

    return OT_ERROR_NONE;
}

otError otPlatRadioDisable(otInstance *a_instance_p)
{
    OT_UNUSED_VARIABLE(a_instance_p);

    samr21Radio_disable();

    return OT_ERROR_NONE;
}

bool otPlatRadioIsEnabled(otInstance *a_instance_p)
{
    OT_UNUSED_VARIABLE(a_instance_p);

    return (samr21Radio_getOtState() > 0 ? true : false);
}

otError otPlatRadioSleep(otInstance *a_instance_p)
{
    OT_UNUSED_VARIABLE(a_instance_p);

    otRadioState currentState = samr21Radio_getOtState();

    if (currentState == OT_RADIO_STATE_TRANSMIT)
    {
        return OT_ERROR_BUSY;
    }
    if (currentState == OT_RADIO_STATE_DISABLED)
    {
        return OT_ERROR_INVALID_STATE;
    }

    samr21Radio_sleep();
    return OT_ERROR_NONE;
}

otError otPlatRadioReceive(otInstance *a_instance_p, uint8_t a_channel)
{
    OT_UNUSED_VARIABLE(a_instance_p);

    otRadioState currentState = samr21Radio_getOtState();

    if (currentState == OT_RADIO_STATE_TRANSMIT || currentState == OT_RADIO_STATE_DISABLED)
    {
        return OT_ERROR_INVALID_STATE;
    }

    samr21Radio_startReceiving(a_channel);

    return OT_ERROR_NONE;
}

otError otPlatRadioReceiveAt(otInstance *a_instance_p, uint8_t a_channel, uint32_t a_start, uint32_t a_duration)
{
    OT_UNUSED_VARIABLE(a_instance_p);

    if (samr21Radio_queueReceiveSlot(a_start, a_channel, a_duration))
    {
        return OT_ERROR_NONE;
    }

    return OT_ERROR_FAILED;
}

otRadioFrame *otPlatRadioGetTransmitBuffer(otInstance *a_instance_p)
{
    OT_UNUSED_VARIABLE(a_instance_p);

    return samr21Radio_getOtTxBuffer();
}

otError otPlatRadioTransmit(otInstance *a_instance_p, otRadioFrame *a_frame)
{
    OT_UNUSED_VARIABLE(a_instance_p);

    otRadioState currentState = samr21Radio_getOtState();

    if (currentState == OT_RADIO_STATE_TRANSMIT || currentState == OT_RADIO_STATE_DISABLED)
    {
        return OT_ERROR_INVALID_STATE;
    }

    samr21radio_transmit(a_frame);

    return OT_ERROR_NONE;
}

int8_t otPlatRadioGetRssi(otInstance *a_instance_p)
{
    OT_UNUSED_VARIABLE(a_instance_p);

    return samr21Trx_getLastRssiValue();
}

otError otPlatRadioEnergyScan(otInstance *a_instance_p, uint8_t a_scanChannel, uint16_t a_scanDuration)
{
    OT_UNUSED_VARIABLE(a_instance_p);

    if (samr21Radio_startEnergyDetection(a_scanChannel, a_scanDuration))
    {
        return OT_ERROR_NONE;
    }

    return OT_ERROR_BUSY;
}

void otPlatRadioEnableSrcMatch(otInstance *a_instance_p, bool a_enable)
{
    OT_UNUSED_VARIABLE(a_instance_p);

    samr21Radio_setFramePendingSrcMatch(a_enable);
}

uint32_t otPlatRadioGetSupportedChannelMask(otInstance *a_instance_p)
{
    OT_UNUSED_VARIABLE(a_instance_p);

    return 0b00000011111111111111110000000000;
}

uint32_t otPlatRadioGetPreferredChannelMask(otInstance *a_instance_p)
{
    OT_UNUSED_VARIABLE(a_instance_p);

    // return 0b00000001000010000100010000000000;
    return 0b00000011111111111111110000000000;
}

// TODO
// otError otPlatRadioSetCoexEnabled(otInstance *aInstance, bool aEnabled);
// bool otPlatRadioIsCoexEnabled(otInstance *aInstance);
// otError otPlatRadioGetCoexMetrics(otInstance *aInstance, otRadioCoexMetrics *aCoexMetrics);

#if OPENTHREAD_CONFIG_MAC_CSL_RECEIVER_ENABLE

otError otPlatRadioEnableCsl(otInstance *a_instance_p,
                             uint32_t a_cslPeriod,
                             otShortAddress a_shortAddr,
                             const otExtAddress *a_extAddr)
{
    OT_UNUSED_VARIABLE(a_instance_p);

    samr21RadioCtrlSetCslPeriod(uint32_t a_cslPeriod);
}

void otPlatRadioUpdateCslSampleTime(otInstance *a_instance_p, uint32_t a_cslSampleTime)
{

    OT_UNUSED_VARIABLE(a_instance_p);

    samr21RadioCtrlSetCslSampleTime(uint32_t a_cslSampleTime);
}

uint8_t otPlatRadioGetCslUncertainty(otInstance *a_instance_p)
{
    OT_UNUSED_VARIABLE(a_instance_p);
    return 20;
}

uint8_t otPlatRadioGetCslAccuracy(otInstance *a_instance_p)
{
    return 10;
}

#endif

otError otPlatRadioSetChannelMaxTransmitPower(otInstance *a_instance_p, uint8_t a_channel, int8_t a_maxPower)
{
    OT_UNUSED_VARIABLE(a_instance_p);
    OT_UNUSED_VARIABLE(a_channel);
    OT_UNUSED_VARIABLE(a_maxPower);

    return OT_ERROR_NOT_IMPLEMENTED;
}

otError otPlatRadioSetRegion(otInstance *a_instance_p, uint16_t a_regionCode)
{
    OT_UNUSED_VARIABLE(a_instance_p);
    OT_UNUSED_VARIABLE(a_regionCode);

    return OT_ERROR_NOT_IMPLEMENTED;
}

otError otPlatRadioGetRegion(otInstance *a_instance_p, uint16_t *a_regionCode_p)
{
    OT_UNUSED_VARIABLE(a_instance_p);
    OT_UNUSED_VARIABLE(a_regionCode_p);

    return OT_ERROR_NOT_IMPLEMENTED;
}

otError otPlatRadioConfigureEnhAckProbing(otInstance *a_instance_p,
                                          otLinkMetrics a_linkMetrics,
                                          otShortAddress a_shortAddress,
                                          const otExtAddress *a_extAddress_p)
{
    OT_UNUSED_VARIABLE(a_instance_p);
    OT_UNUSED_VARIABLE(a_linkMetrics);
    OT_UNUSED_VARIABLE(a_shortAddress);
    OT_UNUSED_VARIABLE(a_extAddress_p);

    return OT_ERROR_NOT_IMPLEMENTED;
}

otError otPlatRadioAddCalibratedPower(otInstance *a_instance_p,
                                      uint8_t a_channel,
                                      int16_t a_actualPower,
                                      const uint8_t *a_rawPowerSetting_p,
                                      uint16_t a_rawPowerSettingLength)
{
    OT_UNUSED_VARIABLE(a_instance_p);
    OT_UNUSED_VARIABLE(a_channel);
    OT_UNUSED_VARIABLE(a_actualPower);
    OT_UNUSED_VARIABLE(a_rawPowerSetting_p);

    return OT_ERROR_NOT_IMPLEMENTED;
}

otError otPlatRadioSetChannelTargetPower(otInstance *a_instance_p,
                                         uint8_t a_channel,
                                         int16_t a_actualPower)
{
    OT_UNUSED_VARIABLE(a_instance_p);
    OT_UNUSED_VARIABLE(a_channel);
    OT_UNUSED_VARIABLE(a_actualPower);

    return OT_ERROR_NOT_IMPLEMENTED;
}

static volatile bool s_pendingRxBuffer = false;
static void receiveTask()
{
    if (s_pendingRxBuffer)
    {
        otRadioFrame *receivedFrame_p = samr21Radio_getReceivedOtFrame();

        if (!receivedFrame_p)
        {
            // No new Frame available
            s_pendingRxBuffer = false;
        }
        else
        {
            // Inform upper Layer
            otPlatRadioReceiveDone(s_instance_p, receivedFrame_p, OT_ERROR_NONE);
        }
    }
}

static bool s_pendingTxStartedEvent = false;
static bool s_pendingTxFinishedEvent = false;
static otRadioFrame* s_txOtFrame_p = NULL;

static transmissionStatus s_currentTransmissionStatus;
static void transmitTask()
{
    if (s_pendingTxStartedEvent)
    {
        s_pendingTxStartedEvent = false;

        otPlatRadioTxStarted(s_instance_p, s_txOtFrame_p);
    }

    if (s_pendingTxFinishedEvent)
    {
        s_pendingTxFinishedEvent = false;

        switch (s_currentTransmissionStatus)
        {
        case RADIO_TRANSMISSION_SUCCESSFUL:
            otPlatRadioTxDone(
                s_instance_p,
                s_txOtFrame_p,
                samr21Radio_getLastReceivedAckOtFrame(),
                OT_ERROR_NONE);
            return;
        case RADIO_TRANSMISSION_CHANNEL_ACCESS_FAILED:
            otPlatRadioTxDone(
                s_instance_p,
                s_txOtFrame_p,
                NULL,
                OT_ERROR_CHANNEL_ACCESS_FAILURE);
            return;
        case RADIO_TRANSMISSION_NO_ACK:
            otPlatRadioTxDone(
                s_instance_p,
                s_txOtFrame_p,
                NULL,
                OT_ERROR_NO_ACK);
            return;
        default:
            otPlatRadioTxDone(
                s_instance_p,
                s_txOtFrame_p,
                NULL,
                OT_ERROR_ABORT);
            break;
        }
    }
}

void samr21OtPlat_radioTick()
{
    receiveTask();
    transmitTask();
}

void samr21Radio_energyDetectionDone_cb(int8_t a_rssi)
{
    otPlatRadioEnergyScanDone(s_instance_p, a_rssi);
}

void samr21Radio_receptionDone_cb()
{
    s_pendingRxBuffer = true;
}

void samr21Radio_transmissionDone_cb(transmissionStatus a_status,  otRadioFrame* a_frame_p)
{

    s_pendingTxFinishedEvent = true;
    s_currentTransmissionStatus = a_status;
    s_txOtFrame_p = a_frame_p;
}

void samr21Radio_transmissionStarted_cb(otRadioFrame* a_frame_p)
{
    s_pendingTxStartedEvent = true;
    s_txOtFrame_p = a_frame_p;
}

void samr21Radio_noMessagesDuringSlot_cb()
{
}