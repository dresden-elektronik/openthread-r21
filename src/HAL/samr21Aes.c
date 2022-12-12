//Author Eric Härtel @ dresden elektronik ingenieurtechnik gmbh © 2022
#include "samr21Aes.h"

extern AT86RF233_REG_TRX_STATUS_t   g_trxStatus;  // from samr21trx.c

void samr21AesKeySetup(uint8_t* key){

    __disable_irq();
    samr21TrxSetSSel(true);
    samr21delayLoop(CPU_WAIT_CYCLE_AFTER_SSEL_LOW);

    //Send Read SRAM Command and get Status Byte (see r2 datasheet 35.4 Radio Transceiver Status Information)
    g_trxStatus.reg = samr21TrxSpiTransceiveByteRaw( AT86RF233_CMD_SRAM_WRITE );
    samr21delayLoop(CPU_WAIT_CYCLE_BETWEEN_BYTES_FAST_ACCESS);
    samr21TrxSpiTransceiveByteRaw( AES_CTRL_REG );

    for (uint8_t i = 0; i < SIZE_AES_KEY_BYTE; i++)
    {
        samr21delayLoop(CPU_WAIT_CYCLE_BETWEEN_BYTES_FAST_ACCESS);
        samr21TrxSpiTransceiveByteRaw( key[i] );
    }

    samr21delayLoop(CPU_WAIT_CYCLE_BEFORE_SSEL_HIGH);
    samr21TrxSetSSel(false);

    __enable_irq();
}

void samr21AesEncrypt(uint8_t* data, uint8_t lenght){

    __disable_irq();
    samr21TrxSetSSel(true);
    samr21delayLoop(CPU_WAIT_CYCLE_AFTER_SSEL_LOW);

    //Send Read SRAM Command and get Status Byte (see r2 datasheet 35.4 Radio Transceiver Status Information)
    g_trxStatus.reg = samr21TrxSpiTransceiveByteRaw( AT86RF233_CMD_SRAM_WRITE );
    samr21delayLoop(CPU_WAIT_CYCLE_BETWEEN_BYTES_FAST_ACCESS);
    samr21TrxSpiTransceiveByteRaw( AES_CTRL_REG );

    for (uint8_t i = 0; i < SIZE_AES_TEXT_BYTE; i++)
    {
        samr21delayLoop(CPU_WAIT_CYCLE_BETWEEN_BYTES_FAST_ACCESS);

        if(i < lenght){
            samr21TrxSpiTransceiveByteRaw( key[i] );
            continue;
        }
        samr21TrxSpiTransceiveByteRaw( 0x00 ); 
    }

    samr21delayLoop(CPU_WAIT_CYCLE_BEFORE_SSEL_HIGH);
    samr21TrxSetSSel(false);

    __enable_irq();
}

void samr21AesDecrypt(uint8_t* data, uint8_t lenght){
    
}