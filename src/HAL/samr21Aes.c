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

    //Send KEY-MODE Command
    samr21delayLoop(CPU_WAIT_CYCLE_BETWEEN_BYTES);
    AT86RF233_SRAM_REG_AES_CTRL_t temp_aesCtrl = {
        .bit.dir = 0,
        .bit.mode = AT86RF233_AES_MODE_KEY,
        .bit.request = 0
    };
    samr21TrxSpiTransceiveByteRaw( temp_aesCtrl.reg );

    //Send Key
    for (uint8_t i = 0; i < AES_BLOCK_SIZE; i++)
    {
        samr21delayLoop(CPU_WAIT_CYCLE_BETWEEN_BYTES_FAST_ACCESS);
        samr21TrxSpiTransceiveByteRaw( key[i] );
    }

    samr21delayLoop(CPU_WAIT_CYCLE_BEFORE_SSEL_HIGH);
    samr21TrxSetSSel(false);

    __enable_irq();
}

void samr21AesEcbEncrypt(uint8_t* inDataBlock, uint8_t* outDataBlock, bool readOnly){
    __disable_irq();
    samr21TrxSetSSel(true);
    samr21delayLoop(CPU_WAIT_CYCLE_AFTER_SSEL_LOW);

    //Send Read SRAM Command and get Status Byte (see r2 datasheet 35.4 Radio Transceiver Status Information)
    g_trxStatus.reg = samr21TrxSpiTransceiveByteRaw( AT86RF233_CMD_SRAM_WRITE );
     
    //Send Addr
    samr21delayLoop(CPU_WAIT_CYCLE_BETWEEN_BYTES);
    samr21TrxSpiTransceiveByteRaw( AES_CTRL_REG );

    //Send ECB encription Command
    samr21delayLoop(CPU_WAIT_CYCLE_BETWEEN_BYTES);
    AT86RF233_SRAM_REG_AES_CTRL_t aesCtrl = {
        .bit.dir = 0,
        .bit.mode = AT86RF233_AES_MODE_ECB,
        .bit.request = 0
    };
    samr21TrxSpiTransceiveByteRaw( aesCtrl.reg );

    for (uint8_t i = 0; i < AES_BLOCK_SIZE; i++)
    {
        //Send New Plaintext and Retrive last Ciphertext
        //samr21delayLoop(CPU_WAIT_CYCLE_BETWEEN_BYTES_FAST_ACCESS);
        if(outDataBlock){
            if(i != 0){
                outDataBlock[i-1] = samr21TrxSpiTransceiveByteRaw( inDataBlock == NULL ? 0x00 : inDataBlock[i] );
            } 
            else {
                samr21TrxSpiTransceiveByteRaw( inDataBlock == NULL ? 0x00 : inDataBlock[i] );
            }
        } 
        else {
            samr21TrxSpiTransceiveByteRaw( inDataBlock == NULL ? 0x00 : inDataBlock[i] );
        }
    }

    aesCtrl.bit.request = !readOnly;

    samr21delayLoop(CPU_WAIT_CYCLE_BETWEEN_BYTES);
    if(outDataBlock){
        outDataBlock[AES_BLOCK_SIZE-1] = samr21TrxSpiTransceiveByteRaw( aesCtrl.reg );
    } else {
        samr21TrxSpiTransceiveByteRaw( aesCtrl.reg );
    }
        
    samr21delayLoop(CPU_WAIT_CYCLE_BEFORE_SSEL_HIGH);
    samr21TrxSetSSel(false);

    __enable_irq();
}

void samr21AesEcbEncryptBlocking(uint8_t* dataBlock){

    samr21AesEcbEncrypt(dataBlock, NULL, false);

    samr21delaySysTick(350); // AES ECB takes 21 us

    samr21AesEcbEncrypt(NULL, dataBlock, true);
}