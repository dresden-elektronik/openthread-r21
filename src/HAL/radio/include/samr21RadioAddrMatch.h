//Author Eric Härtel @ dresden elektronik ingenieurtechnik gmbh © 2022
#ifndef _SAMR21_RADIO_ADDRESS_MATCH_H_
#define _SAMR21_RADIO_ADDRESS_MATCH_H_

#include <stdint.h>
#include <stdbool.h>

#include "samr21.h"
#include "802_15_4_Helper.h"

//For Self-Radio
void samr21RadioSetIeeeAddr(uint8_t* addr);
void samr21RadioSetShortAddr(uint8_t* addr);
void samr21RadioSetPanId(uint8_t* panid);

uint8_t* samr21RadioGetIeeeAddr();
uint8_t* samr21RadioGetShortAddr();
uint8_t* samr21RadioGetPanId();

bool samr21RadioMatchesIeeeAddr(uint8_t* addr);
bool samr21RadioMatchesShortAddr(uint8_t* addr);
bool samr21RadioMatchesPanId(uint8_t* panid);

bool samr21RadioMatchesBroadcastIeeeAddr(uint8_t* addr);
bool samr21RadioMatchesBroadcastShortAddr(uint8_t* addr);
bool samr21RadioMatchesBroadcastPanId(uint8_t* panid);

//For Incoming Frames
#ifndef SIZE_TABLE_FRAME_PENDING_SHORT_ADDR
    #define SIZE_TABLE_FRAME_PENDING_SHORT_ADDR 20
#endif

typedef struct 
{
    uint16_t addr[SIZE_TABLE_FRAME_PENDING_SHORT_ADDR];
    uint8_t size;
}framePendingTableShortAddr;

#ifndef SIZE_TABLE_FRAME_PENDING_IEEE_ADDR
    #define SIZE_TABLE_FRAME_PENDING_IEEE_ADDR 20
#endif

typedef struct 
{
    uint64_t addr[SIZE_TABLE_FRAME_PENDING_IEEE_ADDR];
    uint8_t size;
} framePendingTableIeeeAddr;

bool samr21RadioAddShortAddrToPendingFrameTable(uint8_t * shortAddr);
bool samr21RadioFindShortAddrInPendingFrameTable(uint8_t * shortAddr, bool remove);
void samr21RadioClearShortAddrPendingFrameTable();

bool samr21RadioAddIeeeAddrToPendingFrameTable(uint8_t * ieeeAddr);
bool samr21RadioFindIeeeAddrInPendingFrameTable(uint8_t * ieeeAddr, bool remove);
void samr21RadioClearIeeeAddrPendingFrameTable();


#endif //_SAMR21_RADIO_ADDRESS_MATCH_H_