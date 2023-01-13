//Author Eric Härtel @ dresden elektronik ingenieurtechnik gmbh © 2022
#ifndef _SAMR21_RADIO_CTRL_H_
#define _SAMR21_RADIO_CTRL_H_

#include "samr21.h"
#include "802_15_4_Helper.h"
#include "samr21Trx.h"
#include "samr21RadioIrqHandler.h"
#include "samr21RadioTrxRegCopy.h"
#include "samr21RadioRxHandler.h"
#include "samr21RadioTxHandler.h"
#include "samr21RadioVars.h"

#define OT_US_PER_TEN_SYMBOLS 160 ///< The microseconds per 10 symbols.
#define OT_AES_KEY_SIZE       16

typedef enum 
{
    SAMR21_RADIO_STATE_DISABLED = 0,
    SAMR21_RADIO_STATE_SLEEP    = 1,
    SAMR21_RADIO_STATE_RECEIVE  = 2,
    SAMR21_RADIO_STATE_TRANSMIT = 3,
    SAMR21_RADIO_STATE_INVALID  = 255,
} Samr21RadioState;

void samr21RadioCtrlSetState(Samr21RadioState State);
Samr21RadioState samr21RadioCtrlGetState();

void samr21RadioCtrlInitIrq();
void samr21RadioCtrlDeinitIrq();

void samr21RadioCtrlSetTxPower(int8_t txPower);
int8_t samr21RadioCtrlGetTxPower();

void samr21RadioCtrlSetCCAMode(uint8_t newCcaMode);
uint8_t samr21RadioCtrlGetCCAMode();

void samr21RadioCtrlSetCcaThreshold(int8_t threshold);
int8_t samr21RadioCtrlGetCurrentCcaThreshold();

void samr21RadioCtrlSetPromiscuousMode(bool enable);
bool samr21RadioCtrlGetPromiscuousMode();

void samr21RadioCtrlSetMacFrameCounter(uint32_t macFrameCounter);
uint32_t samr21RadioCtrlGetMacFrameCounter();

void samr21RadioCtrlUpdateCslSampleTime(uint32_t cslSampleTime);
uint16_t samr21RadioCtrlCslGetPhase();