/*
 * Copyright (c) 2023 dresden elektronik ingenieurtechnik gmbh.
 * All rights reserved.
 *
 * The software in this package is published under the terms of the BSD
 * style license a copy of which has been included with this distribution in
 * the LICENSE.txt file.
 *
 */
//Frame Pending Table Short Addr
#include "samr21RadioAddrMatch.h"


//Self Device Addr
static uint64_t g_ieeeAddr;
static uint16_t g_shortAddr;
static uint16_t g_panId;

void samr21RadioSetIeeeAddr(uint8_t* addr){
    for(uint8_t i = 0; i < IEEE_802_15_4_EXTENDED_ADDR_SIZE_BYTES; i++){
        ((uint8_t *)(&g_ieeeAddr))[i] = addr[i];
    }
}

void samr21RadioSetShortAddr(uint16_t addr){
    g_shortAddr = addr;
}

void samr21RadioSetPanId(uint16_t addr){
    g_panId = addr;
}

uint8_t* samr21RadioGetIeeeAddr(){
    return ((uint8_t *)(&g_ieeeAddr));
}
uint16_t samr21RadioGetShortAddr(){
    return g_shortAddr;
}
uint16_t samr21RadioGetPanId(){
    return g_panId;
}

bool samr21RadioMatchesIeeeAddr(uint8_t* addr){
    for(uint8_t i = 0; i < IEEE_802_15_4_EXTENDED_ADDR_SIZE_BYTES; i++){
        if(((uint8_t *)(&g_ieeeAddr))[i] != addr[i]){
            return false;
        }
    }
    return true;
}
bool samr21RadioMatchesShortAddr(uint8_t* addr){
    if(
        (((uint8_t *)(&g_shortAddr))[0] != addr[0])
        || (((uint8_t *)(&g_shortAddr))[1] != addr[1])
    ){
        return false;
    }
    return true;
}
bool samr21RadioMatchesPanId(uint8_t* panid){
    if(
        (((uint8_t *)(&s_panId))[0] != panid[0])
        || (((uint8_t *)(&s_panId))[1] != panid[1])
    ){
        return false;
    }
    return true;
}

bool samr21RadioMatchesBroadcastIeeeAddr(uint8_t* addr){
    for(uint8_t i = 0; i < IEEE_802_15_4_EXTENDED_ADDR_SIZE_BYTES; i++){
        if(addr[i] != 0xFF){
            return false;
        }
    }
    return true;
}
bool samr21RadioMatchesBroadcastShortAddr(uint8_t* addr){
    if(
        (((uint8_t *)(&addr))[0] != 0xFF)
        || (((uint8_t *)(&addr))[1] != 0xFF)
    ){
        return false;
    }
    return true;
}
bool samr21RadioMatchesBroadcastPanId(uint8_t* panid){
    if(
        (((uint8_t *)(&panid))[0] != 0xFF)
        || (((uint8_t *)(&panid))[1] != 0xFF)
    ){
        return false;
    }
    return true;
}

 