//Author Eric Härtel @ dresden elektronik ingenieurtechnik gmbh © 2022

#include "openthread/platform/radio.h"

#include "samr21Trx.h"
#include "samr21Radio.h"
#include "samr21Nvm.h"

static otRadioState s_radioState = OT_RADIO_STATE_DISABLED;
























void otPlatRadioGetIeeeEui64(otInstance *aInstance, uint8_t *aIeeeEui64)
{
    uint64_t ieeeAddr = samr21NvmGetIeeeAddr();

    for(uint8_t i = 0; i < sizeof(uint64_t); i++){
        aIeeeEui64[i] = ((uint8_t *) &ieeeAddr)[i];
    }
}

void otPlatRadioSetPanId(otInstance *aInstance, uint16_t aPanId)
{
    OT_UNUSED_VARIABLE(aInstance);

    otLogDebgPlat("Set Pan ID: 0x%04X", aPanId);

    radioTrxOff();

    PHY_SetPanId(aPanId);

    radioRestore();
}


otRadioState otPlatRadioGetState(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);

    uint8_t status = samr21RadioGetStatus().bit.trxStatus

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

    samr21RadioSetPanID(&aPanId);

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

    samr21RadioChangeCCAThreshold(aThreshold);

    return OT_ERROR_NONE;
}

otError otPlatRadioGetCcaEnergyDetectThreshold(otInstance *aInstance, int8_t *aThreshold)
{
    OT_UNUSED_VARIABLE(aInstance);

    *(aThreshold) = samr21RadioGetCurrentCCAThreshold();

    return OT_ERROR_NONE;
}

otError otPlatRadioGetTransmitPower(otInstance *aInstance, int8_t *aPower)
{
    OT_UNUSED_VARIABLE(aInstance);

    *(aPower) = samr21RadioGetTXPower();
}

otError otPlatRadioSetTransmitPower(otInstance *aInstance, int8_t aPower)
{
    OT_UNUSED_VARIABLE(aInstance);

    samr21RadioChangeTXPower(aPower);

    return OT_ERROR_NONE;
}

