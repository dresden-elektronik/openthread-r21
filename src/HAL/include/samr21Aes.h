//Author Eric Härtel @ dresden elektronik ingenieurtechnik gmbh © 2022

#ifndef _SAMR21_AES_H_
#define _SAMR21_AES_H_

#define SIZE_AES_KEY_BIT    128
#define SIZE_AES_KEY_BYTE   16

#define SIZE_AES_TEXT_BIT    128
#define SIZE_AES_TEXT_BYTE   16

#include "samr21.h"
#include "samr21NopDelay.h"

#include "at86rf233_Bitfield.h"
#include "at86rf233.h"

void samr21AesKeySetup(uint8_t* key);

void samr21AesEncrypt(uint8_t* data, uint8_t lenght);

void samr21AesDecrypt(uint8_t* data, uint8_t lenght);

#endif //_SAMR21_AES_H_