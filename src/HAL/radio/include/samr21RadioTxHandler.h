//Author Eric Härtel @ dresden elektronik ingenieurtechnik gmbh © 2022
#ifndef _SAMR21_RADIO_TX_HANDLER_H_
#define _SAMR21_RADIO_TX_HANDLER_H_

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

#define TIMEOUT_CCA_us 100

typedef enum TxStatus
{
    TX_STATUS_IDLE              =0x0,
    TX_STATUS_SETUP                 ,
    TX_STATUS_WAIT_FOR_DELAYED_START,
    TX_STATUS_CCA                   ,
    TX_STATUS_CSMA_BACKOFF          ,
    TX_STATUS_SENDING_UPLOADING     ,
    TX_STATUS_SENDING_WAIT_TRX_END  ,
    TX_STATUS_WAIT_FOR_ACK          ,
    TX_STATUS_RECIVING_ACK          ,
    TX_STATUS_DONE             =0xF0,
    TX_STATUS_FAILED                
} TxStatus;


//Interface to other RadioHandler
bool samr21RadioTxBusy();
bool samr21RadioTxAbortRetrys();

//Interface to Openthread
otRadioFrame* samr21RadioTxGetOtBuffer();
otRadioFrame* samr21RadioTxGetAckOtBuffer();
bool samr21RadioTxSetup();

//Operation Functions
void samr21RadioTxStart();
void samr21RadioTxStartCCA();
void samr21RadioTxEvalCCA();
void samr21RadioTxStartTransmission();
void samr21RadioTxPrepareAckReception();
void samr21RadioTxAckReceptionStarted();
void samr21RadioTxAckReceptionDone();

//IRQ-Handler
void samr21RadioTxEventHandler();

//Callback
 void cb_samr21RadioTxDone(otRadioFrame* txFrameBuffer, otRadioFrame* txAckFrameBuffer);
 void cb_samr21RadioTxStarted(otRadioFrame* txFrameBuffer);
 void cb_samr21RadioTxFailed(otRadioFrame* txFrameBuffer);

#endif //_SAMR21_RADIO_TX_HANDLER_H_