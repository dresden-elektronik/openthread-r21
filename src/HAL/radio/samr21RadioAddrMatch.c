//Author Eric Härtel @ dresden elektronik ingenieurtechnik gmbh © 2022
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
void samr21RadioSetShortAddr(uint8_t* addr){
    ((uint8_t *)(&g_shortAddr))[0] = addr[0];
    ((uint8_t *)(&g_shortAddr))[1] = addr[1];
}
void samr21RadioSetPanId(uint8_t* panid){
    ((uint8_t *)(&g_panId))[0] = panid[0];
    ((uint8_t *)(&g_panId))[1] = panid[1];
}

uint8_t* samr21RadioGetIeeeAddr(){
    return ((uint8_t *)(&g_ieeeAddr));
}
uint8_t* samr21RadioGetShortAddr(){
    return ((uint8_t *)(&g_shortAddr));
}
uint8_t* samr21RadioGetPanId(){
    return ((uint8_t *)(&g_panId));
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
        (((uint8_t *)(&s_panid))[0] != panid[0])
        || (((uint8_t *)(&s_panid))[1] != panid[1])
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

//Source Address Matching

static framePendingTableShortAddr s_framePendingTableShortAddr;
static framePendingTableIeeeAddr s_framePendingTableIeeeAddr;


bool samr21RadioAddShortAddrToPendingFrameTable(uint8_t* shortAddr){
    if(s_framePendingTableShortAddr.size >= SIZE_TABLE_FRAME_PENDING_SHORT_ADDR){
        //No Space left
        return false;
    }

    ((uint8_t*) &(s_framePendingTableShortAddr.addr[s_framePendingTableShortAddr.size]))[0] = shortAddr[0];
    ((uint8_t*) &(s_framePendingTableShortAddr.addr[s_framePendingTableShortAddr.size++]))[1] = shortAddr[1];
    return true;
} 
bool samr21RadioFindShortAddrInPendingFrameTable(uint8_t* shortAddr, bool remove){
    for (uint8_t i = 0; i < s_framePendingTableShortAddr.size; i++)
    {
        if(((uint8_t*) &(s_framePendingTableShortAddr.addr[i]))[0] != shortAddr[0]){
            continue;
        }

        if(((uint8_t*) &(s_framePendingTableShortAddr.addr[i]))[1] != shortAddr[1]){
            if(remove){
                s_framePendingTableShortAddr.addr[i] = s_framePendingTableShortAddr.addr[s_framePendingTableShortAddr.size--];
            }   
            return true;
        }
    }
    //Address not found, can't be removed
    return false;
} 
void samr21RadioClearShortAddrPendingFrameTable(){
    s_framePendingTableShortAddr.size = 0;
} 

bool samr21RadioAddIeeeAddrToPendingFrameTable(uint8_t * ieeeAddr){
    if(s_framePendingTableIeeeAddr.size >= SIZE_TABLE_FRAME_PENDING_IEEE_ADDR){
        //No Space left
        return false;
    }

    for (uint8_t i = 0; i < sizeof(uint64_t); i++)
    {
        ((uint8_t*) &(s_framePendingTableIeeeAddr.addr[s_framePendingTableIeeeAddr.size]))[i] = ieeeAddr[i];
    }
    s_framePendingTableIeeeAddr.size++;
    return true;
} 
bool samr21RadioFindIeeeAddrInPendingFrameTable(uint8_t* ieeeAddr, bool remove){
    uint8_t i = 0;
    while(i < s_framePendingTableIeeeAddr.size)
    {
        for (uint8_t j = 0; j < (sizeof(uint64_t)); j++)
        {
            if(((uint8_t*) &(s_framePendingTableIeeeAddr.addr[i]))[j] != ieeeAddr[j]){
                goto nextEntry;
            }
        }

        if(remove){
            s_framePendingTableIeeeAddr.addr[i] = s_framePendingTableIeeeAddr.addr[s_framePendingTableIeeeAddr.size--];
        }
        return true;

nextEntry:
        i++;
    }

    //Address not found
    return false;
} 
void samr21RadioClearIeeeAddrPendingFrameTable(){
    s_framePendingTableIeeeAddr.size = 0;
} 