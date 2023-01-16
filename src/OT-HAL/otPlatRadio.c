//Author Eric Härtel @ dresden elektronik ingenieurtechnik gmbh © 2022

#include "openthread/platform/radio.h"

#include "samr21RadioCtrl.h"
#include "samr21RadioRxHandler.h"
#include "samr21RadioTxHandler.h"
#include "samr21RadioEdHandler.h"
#include "samr21RadioAddrMatch.h"
#include "samr21Nvm.h"

extern AT86RF233_REG_TRX_STATUS_t   g_trxStatus;  // from samr21trx.c

static otRadioState s_radioState = OT_RADIO_STATE_DISABLED;

void otPlatRadioGetIeeeEui64(otInstance *aInstance, uint8_t *aIeeeEui64)
{
    uint64_t ieeeAddr = samr21NvmGetIeeeAddr();

    for(uint8_t i = 0; i < sizeof(uint64_t); i++){
        aIeeeEui64[i] = ((uint8_t *) &ieeeAddr)[i];
    }
}


otRadioState otPlatRadioGetState(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);
    return (otRadioState) samr21RadioCtrlGetState();
}

void otPlatRadioSetExtendedAddress(otInstance *aInstance, const otExtAddress *aAddress)
{
    OT_UNUSED_VARIABLE(aInstance);
    samr21RadioSetIeeeAddr(aAddress);
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
}

otError otPlatRadioEnable(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);

    //Always on, cause the TRX-Clock is used as the ref Clock for the MCU-Clocks
    samr21RadioCtrlSetState(SAMR21_RADIO_STATE_SLEEP);

    return OT_ERROR_NONE;
}

otError otPlatRadioDisable(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);

    //Always on, cause the TRX-Clock is used as the ref Clock for the MCU-Clocks
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

    samr21RadioSetTxPower(aPower);

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
        state == SAMR21_RADIO_STATE_DISABLED
        || state == SAMR21_RADIO_STATE_INVALID
    ){
        return false;
    }
    return true;
}

otRadioCaps otPlatRadioGetCaps(otInstance *aInstance){
    
    OT_UNUSED_VARIABLE(aInstance);

    return 
        OT_RADIO_CAPS_ACK_TIMEOUT 
        | OT_RADIO_CAPS_ENERGY_SCAN 
        | OT_RADIO_CAPS_TRANSMIT_RETRIES
        | OT_RADIO_CAPS_CSMA_BACKOFF   
        | OT_RADIO_CAPS_SLEEP_TO_TX    
        | OT_RADIO_CAPS_TRANSMIT_SEC   
        | OT_RADIO_CAPS_TRANSMIT_TIMING
        | OT_RADIO_CAPS_RECEIVE_TIMING
    ;
} 

otError otPlatRadioSetFemLnaGain(otInstance *aInstance, int8_t aGain){

    OT_UNUSED_VARIABLE(aInstance);
    OT_UNUSED_VARIABLE(aGain);

    return OT_ERROR_NOT_IMPLEMENTED;
}

otError otPlatRadioGetFemLnaGain(otInstance *aInstance, int8_t *aGain){

    OT_UNUSED_VARIABLE(aInstance);
    OT_UNUSED_VARIABLE(aGain);

    return OT_ERROR_NOT_IMPLEMENTED;
}

void otPlatRadioSetPromiscuous(otInstance *aInstance, bool aEnable){

    OT_UNUSED_VARIABLE(aInstance);

    samr21RadioCtrlSetPromiscuousMode(aEnable);
}


bool otPlatRadioSetPromiscuous(otInstance *aInstance){

    OT_UNUSED_VARIABLE(aInstance);

    return samr21RadioCtrlGetPromiscuousMode();
}

