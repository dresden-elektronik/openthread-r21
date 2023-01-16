/*
 * Copyright (c) 2023 dresden elektronik ingenieurtechnik gmbh.
 * All rights reserved.
 *
 * The software in this package is published under the terms of the BSD
 * style license a copy of which has been included with this distribution in
 * the LICENSE.txt file.
 *
 */
#include "samr21RadioAes.h"

void samr21RadioAesKeySetup(uint8_t *key)
{

    samr21TrxSpiStartAccess(AT86RF233_CMD_SRAM_WRITE, AES_CTRL_REG);

    AT86RF233_SRAM_REG_AES_CTRL_t temp_aesCtrl = {
        .bit.dir = 0,
        .bit.mode = AT86RF233_AES_MODE_KEY,
        .bit.request = 0};
    samr21TrxSpiTransceiveByteRaw(temp_aesCtrl.reg);
    samr21TrxSpiTransceiveBytesRaw(key, NULL, AES_BLOCK_SIZE);
    samr21TrxSpiCloseAccess();
}

void samr21RadioAesEcbEncrypt(uint8_t *inDataBlock, uint8_t *outDataBlock)
{

    samr21TrxSpiStartAccess(AT86RF233_CMD_SRAM_WRITE, AES_CTRL_REG);

    AT86RF233_SRAM_REG_AES_CTRL_t aesCtrl = {
        .bit.dir = 0,
        .bit.mode = AT86RF233_AES_MODE_ECB,
        .bit.request = 0};

#ifdef __CONSERVATIVE_TRX_SPI_TIMING__
    samr21delaySysTick(CPU_WAIT_CYCLES_BETWEEN_BYTES_FAST_ACCESS);
#endif
    samr21TrxSpiTransceiveByteRaw(aesCtrl.reg);

    for (uint8_t i = 0; i < AES_BLOCK_SIZE; i++)
    {
        // Send New Plaintext and Retrive last Ciphertext
#ifdef __CONSERVATIVE_TRX_SPI_TIMING__
        samr21delaySysTick(CPU_WAIT_CYCLES_BETWEEN_BYTES_FAST_ACCESS);
#endif
        if (outDataBlock)
        {
            if (i != 0)
            {
                outDataBlock[i - 1] = samr21TrxSpiTransceiveByteRaw(inDataBlock == NULL ? 0x00 : inDataBlock[i]);
            }
            else
            {
                samr21TrxSpiTransceiveByteRaw(inDataBlock == NULL ? 0x00 : inDataBlock[i]);
            }
        }
        else
        {
            samr21TrxSpiTransceiveByteRaw(inDataBlock == NULL ? 0x00 : inDataBlock[i]);
        }
    }

    aesCtrl.bit.request = 1;

#ifdef __CONSERVATIVE_TRX_SPI_TIMING__
    samr21delaySysTick(CPU_WAIT_CYCLES_BETWEEN_BYTES);
#endif

    if (outDataBlock)
    {
        outDataBlock[AES_BLOCK_SIZE - 1] = samr21TrxSpiTransceiveByteRaw(aesCtrl.reg);
    }
    else
    {
        samr21TrxSpiTransceiveByteRaw(aesCtrl.reg);
    }

    samr21TrxSpiCloseAccess();
}

void samr21RadioAesCbcEncrypt(uint8_t *inDataBlock, uint8_t inDataLength, uint8_t *outDataBlock)
{

    samr21TrxSpiStartAccess(AT86RF233_CMD_SRAM_WRITE, AES_CTRL_REG);

    // Send ECB encription Command
    AT86RF233_SRAM_REG_AES_CTRL_t aesCtrl = {
        .bit.dir = 0,
        .bit.mode = AT86RF233_AES_MODE_CBC,
        .bit.request = 0};

#ifdef __CONSERVATIVE_TRX_SPI_TIMING__
    samr21delaySysTick(CPU_WAIT_CYCLES_BETWEEN_BYTES);
#endif
    samr21TrxSpiTransceiveByteRaw(aesCtrl.reg);

    for (uint8_t i = 0; i < AES_BLOCK_SIZE; i++)
    {
        // Send New Plaintext and Retrive last Ciphertext
#ifdef __CONSERVATIVE_TRX_SPI_TIMING__
        samr21delaySysTick(CPU_WAIT_CYCLES_BETWEEN_BYTES_FAST_ACCESS);
#endif
        if (outDataBlock)
        {
            if (i != 0)
            {
                outDataBlock[i - 1] = samr21TrxSpiTransceiveByteRaw(inDataBlock == NULL ? 0x00 : inDataBlock[i]);
            }
            else
            {
                samr21TrxSpiTransceiveByteRaw(inDataBlock == NULL ? 0x00 : inDataBlock[i]);
            }
        }
        else
        {

            if (i < inDataLength)
            {
                samr21TrxSpiTransceiveByteRaw(inDataBlock == NULL ? 0x00 : inDataBlock[i]);
            }
            else
            {
                samr21TrxSpiTransceiveByteRaw(0x00);
            }
        }
    }

    aesCtrl.bit.request = 1;

#ifdef __CONSERVATIVE_TRX_SPI_TIMING__
    samr21delaySysTick(CPU_WAIT_CYCLES_BETWEEN_BYTES);
#endif
    if (outDataBlock)
    {
        outDataBlock[AES_BLOCK_SIZE - 1] = samr21TrxSpiTransceiveByteRaw(aesCtrl.reg);
    }
    else
    {
        samr21TrxSpiTransceiveByteRaw(aesCtrl.reg);
    }

    samr21TrxSpiCloseAccess();
}

void samr21RadioAesReadBuffer(uint8_t *pBuffer, uint8_t offset, uint8_t lenght)
{

    samr21TrxSpiStartAccess(AT86RF233_CMD_SRAM_READ, AES_KEY_REG + offset);
    samr21TrxSpiTransceiveBytesRaw(NULL, pBuffer, lenght);
    samr21TrxSpiCloseAccess();
}

bool samr21RadioAesBusy()
{

    samr21TrxSpiStartAccess(AT86RF233_CMD_SRAM_READ, AES_STATUS_REG);
    uint8_t aesStatus = samr21TrxSpiReadByteRaw();
    samr21TrxSpiCloseAccess();

    return !(aesStatus & 0x01);
}
