/*
 * Copyright (c) 2023 dresden elektronik ingenieurtechnik gmbh.
 * All rights reserved.
 *
 * The software in this package is published under the terms of the BSD
 * style license a copy of which has been included with this distribution in
 * the LICENSE.txt file.
 *
 */
#ifndef _SAMR21_RADIO_CTRL_H_
#define _SAMR21_RADIO_CTRL_H_

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stddef.h>

#include "samr21.h"
#include "samr21Trx.h"
#include "samr21Rtc.h"

#include "openthread/platform/radio.h"


#ifndef SAMR21_NUM_RX_BUFFER
    #define SAMR21_NUM_RX_BUFFER 3
#endif



#define US_PER_TEN_SYMBOLS 160 ///< The microseconds per 10 symbols.

#define SAMR21_SOFTWARE_RADIO_ACK_TIMEOUT_us 5000
#define SAMR21_SOFTWARE_RADIO_CCA_TIMEOUT_us 1000
#define SAMR21_SOFTWARE_RADIO_TRX_END_TIMEOUT_us 4096

#define SAMR21_SOFTWARE_RADIO_MIN_DOWNLOAD_BACKOFF_us 300

#define SAMR21_SOFTWARE_RADIO_ED_SCANS_PER_MS 4

typedef enum transmissionStatus_e
{
    RADIO_TRANSMISSION_SUCCESSFUL               = 0x0,
    RADIO_TRANSMISSION_CHANNEL_ACCESS_FAILED    ,
    RADIO_TRANSMISSION_NO_ACK                   ,
    RADIO_TRANSMISSION_UNDEFINED_ERROR
} transmissionStatus;



typedef enum samr21Radio_RxBufferState_e
{
    SAMR21_RADIO_RX_STATE_IDLE,
    SAMR21_RADIO_RX_STATE_PARSE_FCF,
    SAMR21_RADIO_RX_STATE_WAIT_ADDR,
    SAMR21_RADIO_RX_STATE_PARSE_ADDR,
    SAMR21_RADIO_RX_STATE_WAIT_REMAINING,
    SAMR21_RADIO_RX_STATE_PARSE_REMAINING,
    SAMR21_RADIO_RX_STATE_PREP_ACK,
    SAMR21_RADIO_RX_STATE_WAIT_ACK_START,
    SAMR21_RADIO_RX_STATE_SENDING_ACK,
    SAMR21_RADIO_RX_STATE_WAIT_ACK_END
} samr21RxState;

typedef enum samr21Radio_TxBufferState_e
{
    SAMR21_RADIO_TX_STATE_IDLE,
    SAMR21_RADIO_TX_STATE_WAIT_FOR_TX_TIMING,
    SAMR21_RADIO_TX_STATE_CCA,
    SAMR21_RADIO_TX_STATE_WAIT_CCA_RESULT,
    SAMR21_RADIO_TX_STATE_EVAL_CCA_RESULT,
    SAMR21_RADIO_TX_STATE_CSMA_BACKOFF,
    SAMR21_RADIO_TX_STATE_SENDING,
    SAMR21_RADIO_TX_STATE_SENDING_RAW,
    SAMR21_RADIO_TX_STATE_SENDING_SECURITY,
    SAMR21_RADIO_TX_STATE_WAIT_SENDING_END,
    SAMR21_RADIO_TX_STATE_WAIT_FOR_ACK_START,
    SAMR21_RADIO_TX_STATE_WAIT_FOR_ACK_END,
    SAMR21_RADIO_TX_STATE_EVAL_ACK
} samr21TxState;



/**
 * 
 * Enable the Transceiver IRQs Handler
 */
void samr21Radio_enable();

/**
 * Disables all Transceiver IRQs 
 */
void samr21Radio_disable();

/**
 * Disables all Transceiver IRQs 
 */
void samr21Radio_disable();

/**
 * Puts Transceiver To sleep
 */
void samr21Radio_sleep();

/**
 * Returns the OpenThread equivalent State of the Radio
 * 
 * @returns OpenThread equivalent State of the Radio
 */
otRadioState samr21Radio_getOtState();

/**
 * Changes Promiscuous Mode
 *
 * @param[in]  enable       true = enable, false = disable
 */
void samr21Radio_setPromiscuousMode(bool enable);

/**
 * Enables or Disables Soft Source Addr Match table
 *
 * @param[in]  enable       true = enable, false = disable
 */
void samr21Radio_setFramePendingSrcMatch(bool enable);

/**
 * Gets current Promiscuous State
 *
 * @returns true = enabled, false = disabled
 */
bool samr21Radio_getPromiscuousMode();

/**
 * Updates the current Mac Frame Counter
 *
 * @param[in]  macFrameCounter  new Value for Mac Frame Counter
 *
 */
void samr21Radio_setMacFrameCounter(uint32_t macFrameCounter);

/**
 * Gets the current Mac Frame Counter
 *
 * @return      current Mac Frame Counter
 *
 */
uint32_t samr21Radio_getMacFrameCounter();

#if OPENTHREAD_CONFIG_MAC_CSL_RECEIVER_ENABLE

/**
 * Updates the current CSL Sample Time
 *
 * @param[in]  cslPeriod  new Value for CSL Sample Time
 *
 */
void samr21RadioCtrlSetCslPeriod(uint32_t cslPeriod);

/**
 * Updates the current CSL Sample Time
 *
 * @param[in]  cslSampleTime  new Value for CSL Sample Time
 *
 */
void samr21RadioCtrlUpdateCslSampleTime(uint32_t cslSampleTime);

/**
 * Calculates the current CSL Phase
 *
 * @return      current CSL Phase
 *
 */
uint16_t samr21RadioCtrlCslGetPhase();

#endif

/**
 * Updates the MacKeys for Ack and Tx encryption
 *
 * @param[in]  keyId  id of current Key
 *
 * @param[in]  previousKey  pointer to 16 Byte Array of the last AES-Key
 * @param[in]  currentKey  pointer to 16 Byte Array of the current AES-Key
 * @param[in]  nextKey  pointer to 16 Byte Array of the next AES-Key
 *
 */
void samr21Radio_setMacKeys(uint8_t keyId, const otMacKeyMaterial*previousKey, const otMacKeyMaterial *currentKey, const otMacKeyMaterial *nextKey);

/**
 * Sets a new Pan ID
 *
 * @param[in]  panId  new Pan ID 
 *
 */
void samr21Radio_setPanId(otPanId panId);

/**
 * Sets a new Short Address
 *
 * @param[in]  shortAddress  new Short Address
 *
 */
void samr21Radio_setShortAddress(otShortAddress shortAddress);

/**
 * Sets a new extended Address
 *
 * @param[in]  extendedAddress  pointer to new Extended Address
 *
 */
void samr21Radio_setExtendedAddress(const otExtAddress* extendedAddress);

/**
 * Returns the current Pan ID
 *
 * @return     current Pan ID
 *
 */
uint16_t samr21Radio_getPanId();

/**
 * Returns the current Short Address
 *
 * @return     current Short Address
 *
 */
uint16_t samr21Radio_getShortAddr();

/**
 * Returns pointer to rhe current Extended Address
 *
 * @return     Pointer to current Extended Address
 *
 */
uint8_t* samr21Radio_getExtendedAddress();

/**
 * Puts Radio into Receive State on given channel
 *
 * @param[in]  channel  channel to receive on ( 0 means stay on current channel )
 *
 */
void samr21Radio_startReceiving(uint8_t channel);

/**
 * Queues a Receive-Slot for a specified duration, channel and starttime
 *
 * @param[in]  startPointTimestamp  microsecond Timestamp when the Slot is should start
 * @param[in]  channel  channel on which the slot takes place
 * @param[in]  duration  duration for how long the Radio should listen
 * 
 * @returns true if Slot was successfully Queued, false if the Slot couldn't be queued
 *
 */
bool samr21Radio_queueReceiveSlot(uint32_t startPointTimestamp, uint8_t channel, uint32_t duration);

/**
 * Returns a pending received Frame from the RX-Ringbuffer
 *
* @returns   pointer to the otFrame struct of a received Frame or NULL if none is available
 *
 */
otRadioFrame* samr21Radio_getReceivedOtFrame();



/**
 * Starts Transmission of a given Frame
 *
 * @param[in]  otFrame  Pointer to OpenThread FrameBuffer about to Transmitted
 *
 */
void samr21radio_transmit(otRadioFrame *otFrame);

/**
 * Get the Transmission Buffer
 *
 * @returns Pointer to OpenThread FrameBuffer for Transmission purposes 
 *
 */
otRadioFrame* samr21Radio_getOtTxBuffer();

/**
 * Gets the Ack of the Last Transmission
 *
 * @returns Pointer to OpenThread FrameBuffer for last received Transmission acknowledgment 
 *
 */
otRadioFrame* samr21Radio_getLastReceivedAckOtFrame();

/**
 * Starts a Energy Detect
 *
 * @param[in]  channel  channel to perform Energy Detection on
 *
 * @param[in]  duration  duration of EnergyScan in Milli Seconds
 * 
 * @returns True if EnergyScan started, False if a EnergyScan is already ongoing 
 *
 */
bool samr21Radio_startEnergyDetection(uint8_t channel, uint32_t duration);

/**
 * Returns the Maximum RSSI Value of the Last Energy Detection Scan 
 *
 * @returns result of last Energy Detection Measurement 
 *
 */
int8_t samr21Radio_getLastEdResult();


//CALLBACKS
void samr21Radio_energyDetectionDone_cb(int8_t a_rssi);
void samr21Radio_receptionDone_cb();

void samr21Radio_noMessagesDuringSlot_cb();

void samr21Radio_transmissionDone_cb(transmissionStatus status, otRadioFrame* a_frame_p);
void samr21Radio_transmissionStarted_cb(otRadioFrame* a_frame_p);

#endif