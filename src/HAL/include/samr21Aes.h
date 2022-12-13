//Author Eric Härtel @ dresden elektronik ingenieurtechnik gmbh © 2022

#ifndef _SAMR21_AES_H_
#define _SAMR21_AES_H_

#define AES_ECB_ENCRYPTION_CMD  0x00
#define AES_START_CMD           0x00

#include "samr21.h"
#include "samr21NopDelay.h"
#include "samr21Trx.h"

#include "at86rf233_Bitfield.h"
#include "at86rf233.h"

#include <stddef.h>

void samr21AesKeySetup(uint8_t* key);
void samr21AesEcbEncrypt(uint8_t* inDataBlock, uint8_t* outDataBlock, bool readOnly);
void samr21AesEcbEncryptBlocking(uint8_t* dataBlock);

#endif //_SAMR21_AES_H_