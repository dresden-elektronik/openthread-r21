/*
 * Copyright (c) 2023 dresden elektronik ingenieurtechnik gmbh.
 * All rights reserved.
 *
 * The software in this package is published under the terms of the BSD
 * style license a copy of which has been included with this distribution in
 * the LICENSE.txt file.
 *
 */
#ifndef _SAMR21_RADIO_TX_HANDLER_H_
#define _SAMR21_RADIO_TX_HANDLER_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "samr21.h"

#include "802_15_4_Helper.h"
#include "samr21RadioIrqHandler.h"
#include "samr21Trx.h"
#include "samr21RadioTrxRegCopy.h"
#include "samr21RadioCtrl.h"
#include "samr21RadioVars.h"
#include "samr21RadioAes.h"
#include "samr21Timer.h"

#include "openthread/platform/radio.h"
#include "otUtilWrapper_macFrame.h"

#define TIMEOUT_CCA_us 1000
#define TIMEOUT_RETRY_us 3000


typedef enum TxStatus
{
    TX_STATUS_IDLE              =0x0,
    TX_STATUS_SETUP                 ,
    TX_STATUS_WAIT_FOR_DELAYED_START,
    TX_STATUS_CCA                   ,
    TX_STATUS_CSMA_BACKOFF          ,
    TX_STATUS_SENDING_UPLOADED_HEADER,
    TX_STATUS_SENDING_UPLOADED_PAYLOAD,
    TX_STATUS_SENDING_BUSY_MIC      ,
    TX_STATUS_SENDING_WAIT_FOR_MIC  ,
    TX_STATUS_SENDING_WAIT_TRX_END  ,
    TX_STATUS_WAIT_FOR_ACK          ,
    TX_STATUS_RECIVING_ACK          ,
    TX_STATUS_DONE             =0xF0,
    TX_STATUS_FAILED                
} TxStatus;


//Interface to other RadioHandler
bool samr21RadioTxBusy();
void samr21RadioTxAbortRetrys();


//Interface to Openthread
otRadioFrame* samr21RadioTxGetOtBuffer();
otRadioFrame* samr21RadioTxGetAckOtBuffer();
bool samr21RadioTxSetup();


//IRQ-Handler
void samr21RadioTxEventHandler(IrqEvent event);

//Callback
 void cb_samr21RadioTxDone(otRadioFrame* txFrameBuffer, otRadioFrame* txAckFrameBuffer);
 void cb_samr21RadioTxStarted(otRadioFrame* txFrameBuffer);
 void cb_samr21RadioTxFailed(otRadioFrame* txFrameBuffer, TxStatus failedAt);

#endif //_SAMR21_RADIO_TX_HANDLER_H_