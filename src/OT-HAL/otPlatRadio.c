//Author Eric Härtel @ dresden elektronik ingenieurtechnik gmbh © 2022

#include "openthread/platform/radio.h"

#include "samr21Trx.h"
#include "samr21Radio.h"
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

    uint8_t status = samr21RadioGetStatus().bit.trxStatus;

    switch (status){
        case TRX_STATUS_P_ON:
            return OT_RADIO_STATE_DISABLED;

        case TRX_STATUS_TRX_OFF:
            return OT_RADIO_STATE_DISABLED;
        
        case TRX_STATUS_BUSY_RX:
            return OT_RADIO_STATE_RECEIVE;

        case TRX_STATUS_RX_ON:
            return OT_RADIO_STATE_RECEIVE;

        case TRX_STATUS_PLL_ON:
            return OT_RADIO_STATE_TRANSMIT;
        
        case TRX_STATUS_BUSY_TX:
            return OT_RADIO_STATE_TRANSMIT;
        
        default:
            return OT_RADIO_STATE_INVALID;
    }
}

void otPlatRadioSetExtendedAddress(otInstance *aInstance, const otExtAddress *aAddress)
{
    OT_UNUSED_VARIABLE(aInstance);

    samr21RadioTurnTrxOff();

    samr21RadioSetIeeeAddr(aAddress->m8);

    samr21RadioTurnTrxOn();
}

void otPlatRadioSetShortAddress(otInstance *aInstance, uint16_t aAddress)
{
    OT_UNUSED_VARIABLE(aInstance);

    samr21RadioTurnTrxOff();

    samr21RadioSetShortAddr(&aAddress);

    samr21RadioTurnTrxOn();
}

void otPlatRadioSetPanId(otInstance *aInstance, uint16_t aPanId)
{
    OT_UNUSED_VARIABLE(aInstance);

    samr21RadioTurnTrxOff();

    samr21RadioSetPanId(&aPanId);

    samr21RadioTurnTrxOn();
}

otError otPlatRadioEnable(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);

    samr21RadioTurnTrxOn();

    return OT_ERROR_NONE;
}

otError otPlatRadioDisable(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);

    samr21RadioTurnTrxOff();

    return OT_ERROR_NONE;
}

otError otPlatRadioSetCcaEnergyDetectThreshold(otInstance *aInstance, int8_t aThreshold)
{
    OT_UNUSED_VARIABLE(aInstance);

    samr21RadioSetCcaThreshold(aThreshold);

    return OT_ERROR_NONE;
}

otError otPlatRadioGetCcaEnergyDetectThreshold(otInstance *aInstance, int8_t *aThreshold)
{
    OT_UNUSED_VARIABLE(aInstance);

    *(aThreshold) = samr21RadioGetCurrentCcaThreshold();

    return OT_ERROR_NONE;
}

otError otPlatRadioGetTransmitPower(otInstance *aInstance, int8_t *aPower)
{
    OT_UNUSED_VARIABLE(aInstance);

    *(aPower) = samr21RadioGetTxPower();
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
    if (
        g_trxStatus.bit.trxStatus == TRX_STATUS_TRX_OFF
        || g_trxStatus.bit.trxStatus == TRX_STATUS_P_ON
    ){
        return false;
    }
    return true;
}
