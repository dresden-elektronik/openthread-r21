/*
 * Copyright (c) 2024 dresden elektronik ingenieurtechnik gmbh.
 * All rights reserved.
 *
 * The software in this package is published under the terms of the BSD
 * style license a copy of which has been included with this distribution in
 * the LICENSE.txt file.
 *
 */

#ifndef _SAMR21_DMA_H_
#define _SAMR21_DMA_H_

#include "samr21.h"
#include <stdbool.h>
#include <string.h>
#include <stdint.h>

#ifndef SAMR21_NUM_DMA_CHANNEL
#define SAMR21_NUM_DMA_CHANNEL 2
#endif

typedef void (*samr21Dma_done_cb)(void);



/**
 * Inits the DMA Module
 * 
 * Must be called before any DMA Action can take place
 *
 */
void samr21Dma_init(void);

/**
 * Sets up a DMA Channel for Asynchronous memcpy to a Peripheral  
 *
 * @param[in]  channel  channel used for the DMA Action (lower Channel have higher Priority)
 * @param[in]  targetAddress  address to the desired Peripheral DATA Register
 * @param[in]  triggerSource trigger that initiates the transfer of a Single Byte 
 * @param[in]  callbackFunc function called after a DMA-Transaction is finished (can be NULL if none is used)
 *
 * @returns True if Channel was initted correctly
 *
 */
bool samr21Dma_initChannel(uint8_t channel, uint32_t targetAddress, uint8_t triggerSource, samr21Dma_done_cb callbackFunc);


/**
 * Sets DMA Channel as Active, so it reacts to Trigger Events
 *
 * @param[in]  channel  channel used for the DMA Action (lower Channel have higher Priority)
 * @param[in]  data  pointer to the ByteArray the supposed to be copied
 * @param[in]  dataLength  length of the ByteArray supposed to be copied
 * @param[in]  linkedDescriptor  DMA Descriptor to a queued DMA-Job
 *
 * @returns True if Channel was activated 
 *
 */
bool samr21Dma_activateChannel(uint8_t channel, uint8_t * data, uint32_t dataLength, DmacDescriptor * linkedDescriptor);


/**
 * Forces a Trigger Event on specified DMA Channel
 * 
 * Used to Jump-Start Transfers
 *
 * @param[in]  channel  channel thats supposed to be triggered
 */
void samr21Dma_triggerChannelAction(uint8_t channel);

#endif //_SAMR21_DMA_H_