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

void samr21Dma_init(void);
bool samr21Dma_initChannel(uint8_t channel, uint32_t targetAddress, uint8_t triggerSource, samr21Dma_done_cb callbackFunc);
bool samr21Dma_start(uint8_t channel, uint8_t * data, uint32_t dataLength, DmacDescriptor * linkedDescriptor);
void samr21Dma_triggerChannelAction(uint8_t channel);

#endif //_SAMR21_DMA_H_