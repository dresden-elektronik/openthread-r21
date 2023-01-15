// Author Eric Härtel @ dresden elektronik ingenieurtechnik gmbh © 2022
#ifndef _SAMR21_RADIO_RX_HANDLER_H_
#define _SAMR21_RADIO_RX_HANDLER_H_

#include <stdint.h>
#include <stdbool.h>
#include "samr21.h"

#include "802_15_4_Helper.h"
#include "samr21RadioIrqHandler.h"
#include "samr21Trx.h"
#include "samr21RadioTrxRegCopy.h"
#include "samr21RadioCtrl.h"
#include "samr21RadioAes.h"
#include "samr21Timer.h"

#include "openthread/platform/radio.h"
#include "otUltilWrapper_macFrame.h"

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
    RX_STATUS_SETUP,
    RX_STATUS_WAIT_FOR_DELAYED_START,

    RX_STATUS_RECIVING_FCF,
    RX_STATUS_RECIVING_ADDR_FIELD,
    RX_STATUS_RECIVING_REMAINING,

    RX_STATUS_SENDING_ACK,
    RX_STATUS_SENDING_ENH_ACK,
    RX_STATUS_SENDING_ACK_WAIT_TRX_END,
    RX_STATUS_DONE = 0xF0,
    RX_STATUS_FAILED
} RxStatus;

typedef struct RxBuffer
{
    RxStatus status;
    otRadioFrame otFrame;
    uint8_t rxFramePsdu[IEEE_802_15_4_FRAME_SIZE];
    uint8_t rxAckPsdu[IEEE_802_15_4_FRAME_SIZE];
    uint8_t neededPdsuSizeForNextAction;
    bool framePending;
} RxBuffer;

// Init
void samr21RadioRxResetBuffer();

// Interface to Openthread
RxBuffer *samr21RadioRxGetPendingRxBuffer();
bool samr21RadioRxSetup(uint8_t channel, uint32_t duration, uint32_t startTime);

// Operation Functions
void samr21RadioTxStart();
void samr21RadioTxStartCCA();
void samr21RadioTxEvalCCA();
void samr21RadioTxStartTransmission();
void samr21RadioTxPrepareAckReception();
void samr21RadioTxAckReceptionStarted();
void samr21RadioTxAckReceptionDone();

// IRQ-Handler
void samr21RadioTxEventHandler();

// Callback
void cb_samr21RadioTxDone(otRadioFrame *txFrameBuffer, otRadioFrame *txAckFrameBuffer);
void cb_samr21RadioTxStarted(otRadioFrame *txFrameBuffer);
void cb_samr21RadioTxFailed(otRadioFrame *txFrameBuffer, RxStatus failedAt);

#endif //_SAMR21_RADIO_RX_HANDLER_H_