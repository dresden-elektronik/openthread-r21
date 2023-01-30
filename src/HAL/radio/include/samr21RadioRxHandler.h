/*
 * Copyright (c) 2023 dresden elektronik ingenieurtechnik gmbh.
 * All rights reserved.
 *
 * The software in this package is published under the terms of the BSD
 * style license a copy of which has been included with this distribution in
 * the LICENSE.txt file.
 *
 */
#ifndef _SAMR21_RADIO_RX_HANDLER_H_
#define _SAMR21_RADIO_RX_HANDLER_H_


#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#include "samr21.h"

#include "802_15_4_Helper.h"
#include "samr21RadioIrqHandler.h"
#include "samr21Trx.h"
#include "samr21RadioTrxRegCopy.h"
#include "samr21RadioVars.h"
#include "samr21RadioCtrl.h"
#include "samr21RadioAes.h"
#include "samr21Timer.h"

#include "openthread/platform/radio.h"
#include "otUtilWrapper_macFrame.h"

#define NUM_SAMR21_RX_BUFFER 4

#define RX_BACKOFF_BEFORE_FIRST_FRAMEBUFFER_ACCESS 100
#define RX_FRAMEBUFFER_ACCESS_HEADROOM_us 64

#ifdef OPENTHREAD_CONFIG_MLE_LINK_METRICS_SUBJECT_ENABLE
#define VENDOR_OUI_THREAD_COMPANY_ID 0xeab89b
#define ENH_ACK_PROBING_IE 0x00
#endif

typedef enum RxStatus
{
    RX_STATUS_IDLE = 0x00,

    RX_STATUS_RECIVING_FCF,
    RX_STATUS_RECIVING_ADDR_FIELD,
    RX_STATUS_RECIVING_REMAINING,

    RX_STATUS_SENDING_ACK,
    RX_STATUS_SENDING_ENH_ACK,
    RX_STATUS_SENDING_ACK_WAIT_TRX_END,
    RX_STATUS_DONE = 0xF0,
} RxStatus;

typedef struct RxBuffer
{
    RxStatus status;
    otRadioFrame otFrame;
    uint8_t rxFramePsdu[IEEE_802_15_4_FRAME_SIZE];
    otRadioFrame otAck;
    uint8_t rxAckPsdu[IEEE_802_15_4_FRAME_SIZE];
    uint8_t neededPdsuSizeForNextAction;
    bool framePending;
} RxBuffer;

// Init
void samr21RadioRxResetAllBuffer();
void samr21RadioRxResetBuffer(RxBuffer * buffer);
void samr21RadioRxPrepareBuffer();



bool samr21RadioRxBusy();

bool samr21RadioRxIsReceiveSlotPlanned();

// Interface to Openthread
RxBuffer *samr21RadioRxGetPendingRxBuffer();
void samr21RadioRxSetup(uint8_t channel, uint32_t duration, uint32_t startTime);


// IRQ-Handler
void samr21RadioRxEventHandler(IrqEvent event);

// Callback
void cb_samr21RadioRxDone(RxBuffer* buffer);
void cb_samr21RadioRxRecivedNothing();

#endif //_SAMR21_RADIO_RX_HANDLER_H_