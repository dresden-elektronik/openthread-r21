//Author Eric Härtel @ dresden elektronik ingenieurtechnik gmbh © 2022

#ifndef _SAMR21_AES_H_
#define _SAMR21_AES_H_

#include "samr21.h"
#include "samr21NopDelay.h"

#include "at86rf233_Bitfield.h"
#include "at86rf233.h"

void samr21AesKeySetup(uint8_t* key);

void samr21AesEncrypt(uint8_t* data, uint8_t lenght);

void samr21AesDecrypt(uint8_t* data, uint8_t lenght);

#endif //_SAMR21_AES_H_