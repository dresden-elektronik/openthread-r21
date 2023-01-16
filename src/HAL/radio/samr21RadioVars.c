/*
 * Copyright (c) 2023 dresden elektronik ingenieurtechnik gmbh.
 * All rights reserved.
 *
 * The software in this package is published under the terms of the BSD
 * style license a copy of which has been included with this distribution in
 * the LICENSE.txt file.
 *
 */
#include "samr21RadioVars.h"

volatile uint64_t   g_ieeeAddr;
volatile uint16_t   g_shortAddr;
volatile uint16_t   g_panId;

volatile bool       g_promiscuousMode = false;

volatile uint32_t   g_macFrameCounter;

volatile uint32_t   g_cslSampleTime;
volatile uint32_t   g_cslPeriod;


volatile uint8_t    g_currKeyId; 
volatile uint8_t    g_prevKey[AES_BLOCK_SIZE]; 
volatile uint8_t    g_currKey[AES_BLOCK_SIZE];
volatile uint8_t    g_nextKey[AES_BLOCK_SIZE];

volatile uint8_t    g_numMaxTransmissionRetrys   = 3;
volatile uint8_t    g_numMaxCsmaBackoffs         = 4;

volatile uint32_t   g_csmaBackoffExponentMin     = 3;
volatile uint32_t   g_csmaBackoffExponentMax     = 7;

volatile uint32_t   g_txAckTimeout_us            = 250;

