#ifndef _SAMR21_RADIO_VARS_H_
#define _SAMR21_RADIO_VARS_H_

#include <stdint.h>
#include <stdbool.h>

#include "at86rf233.h"

extern uint64_t   g_extAddr;
extern uint16_t   g_shortAddr;
extern uint16_t   g_panId;

extern bool       g_promiscuousMode;

extern uint32_t   g_macFrameCounter;

extern uint32_t   g_cslSampleTime;
extern uint32_t   g_cslPeriod;


extern uint8_t    g_currKeyId; 
extern uint8_t    *g_prevKey; 
extern uint8_t    *g_currKey;
extern uint8_t    *g_nextKey;

extern uint8_t    g_numMaxTransmissionRetrys;
extern uint8_t    g_numMaxCsmaBackoffs;

extern uint32_t   g_csmaBackoffExponentMin;
extern uint32_t   g_csmaBackoffExponentMax; 

extern uint16_t   g_ackMaxWaitDuration_us;
extern uint16_t   g_transmissionTimeout_us;

extern uint32_t   g_txAckTimeout_us;


#endif //_SAMR21_RADIO_VARS_H_