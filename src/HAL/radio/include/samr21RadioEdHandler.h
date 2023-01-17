/*
 * Copyright (c) 2023 dresden elektronik ingenieurtechnik gmbh.
 * All rights reserved.
 *
 * The software in this package is published under the terms of the BSD
 * style license a copy of which has been included with this distribution in
 * the LICENSE.txt file.
 *
 */
#ifndef _SAMR21_RADIO_ED_HANDLER_H_
#define _SAMR21_RADIO_ED_HANDLER_H_

#include <stdint.h>
#include <stdbool.h>
#include "samr21.h"

#include "802_15_4_Helper.h"
#include "samr21RadioIrqHandler.h"
#include "samr21Trx.h"
#include "samr21RadioTrxRegCopy.h"
#include "samr21RadioCtrl.h"
#include "samr21Timer.h"

#ifndef NUM_ENERGY_DETECTION_SCANS_PER_MS
    #define NUM_ENERGY_DETECTION_SCANS_PER_MS 4
#endif

#define TIME_UNTIL_NEXT_ENERGY_DETECTION_SCAN_us (1000 / NUM_ENERGY_DETECTION_SCANS_PER_MS)

typedef enum 
{
    ED_STATUS_IDLE                  = 0x00,
    ED_STATUS_WAIT_FOR_RESULT       = 0x02,
    ED_STATUS_WAIT_FOR_NEXT_SCAN    = 0x03,
    ED_STATUS_DONE                  = 0xFF
} EdStatus;


int8_t samr21RadioEdGetLastResult();
bool samr21RadioEdStart(uint8_t channel, uint16_t duration_ms);


void samr21RadioEdEventHandler(IrqEvent event);

//Callback
void cb_samr21RadioEdDone(int8_t rssi);

#endif //_SAMR21_RADIO_ED_HANDLER_H_