/*
 * Copyright (c) 2023 dresden elektronik ingenieurtechnik gmbh.
 * All rights reserved.
 *
 * The software in this package is published under the terms of the BSD
 * style license a copy of which has been included with this distribution in
 * the LICENSE.txt file.
 *
 */
#ifndef _SAMR21_RADIO_ADDRESS_MATCH_H_
#define _SAMR21_RADIO_ADDRESS_MATCH_H_


#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>


#include "samr21.h"
#include "samr21RadioVars.h"
#include "802_15_4_Helper.h"

//For Self-Radio
void samr21RadioSetIeeeAddr(uint8_t* addr);
void samr21RadioSetShortAddr(uint16_t addr);
void samr21RadioSetPanId(uint16_t panid);

uint8_t* samr21RadioGetIeeeAddr();
uint16_t samr21RadioGetShortAddr();
uint16_t samr21RadioGetPanId();

bool samr21RadioMatchesIeeeAddr(uint8_t* addr);
bool samr21RadioMatchesShortAddr(uint8_t* addr);
bool samr21RadioMatchesPanId(uint8_t* panid);

bool samr21RadioMatchesBroadcastIeeeAddr(uint8_t* addr);
bool samr21RadioMatchesBroadcastShortAddr(uint8_t* addr);
bool samr21RadioMatchesBroadcastPanId(uint8_t* panid);

#endif //_SAMR21_RADIO_ADDRESS_MATCH_H_