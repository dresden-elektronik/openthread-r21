#ifndef _SAMR21_RADIO_AES_H_
#define _SAMR21_RADIO_AES_H_

#include "samr21RadioTrxRegCopy.h"
#include "samr21RadioVars.h"
#include "samr21Trx.h"
#include "samr21.h"


void samr21RadioAesCbcEncrypt(uint8_t* inDataBlock, uint8_t inDataLength ,uint8_t* outDataBlock);
void samr21RadioAesEcbEncrypt(uint8_t* inDataBlock, uint8_t* outDataBlock);
void samr21RadioAesKeySetup(uint8_t* key);

void samr21RadioAesReadBuffer(uint8_t* pBuffer, uint8_t offset, uint8_t length);

bool samr21RadioAesBusy();
#endif //_SAMR21_RADIO_AES_H_