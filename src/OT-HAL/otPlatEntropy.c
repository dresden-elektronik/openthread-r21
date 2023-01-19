//Author Eric Härtel @ dresden elektronik ingenieurtechnik gmbh © 2022
#include "openthread/platform/entropy.h"

#include "samr21Trx.h"

otError otPlatEntropyGet(uint8_t *aOutput, uint16_t aOutputLength)
{
    for (uint16_t i = 0; i < aOutputLength; i++)
    {
        aOutput[i] = samr21TrxGetRandomByte();
    }
    return OT_ERROR_NONE;
}
