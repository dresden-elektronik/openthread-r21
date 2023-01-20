/*
 * Copyright (c) 2023 dresden elektronik ingenieurtechnik gmbh.
 * All rights reserved.
 *
 * The software in this package is published under the terms of the BSD
 * style license a copy of which has been included with this distribution in
 * the LICENSE.txt file.
 *
 */
#include "at86rf233_Bitfield.h"
#include "at86rf233.h"
//Local Register Copys of At86rf233 to save some Read Acceses
volatile AT86RF233_REG_TRX_STATUS_t     g_trxStatus;   //used as external var
volatile AT86RF233_REG_IRQ_STATUS_t     g_trxLastIrq;  //used as external var         
volatile AT86RF233_REG_TRX_CTRL_0_t     g_trxCtrl0;    //used as external var

volatile AT86RF233_REG_PHY_CC_CCA_t     g_phyCcCcaReg;
volatile AT86RF233_REG_PHY_RSSI_t       g_phyRssi;
volatile AT86RF233_REG_CCA_THRES_t      g_ccaThres;

volatile AT86RF233_REG_TRX_CTRL_1_t     g_trxCtrl1 =
    {
        .bit.irqPolarity = 0,
        .bit.irqMaskMode = 0,
        .bit.spiCmdMode = 0x1,
        .bit.rxBlCtrl = 1,
        .bit.txAutoCrcOn = 1,
        .bit.irq2ExtEn = 0,
        .bit.paExtnEn = 0
    };

volatile AT86RF233_REG_PHY_TX_PWR_t     g_phyTxPwr =
    {
        .bit.txPwr = 0x7
    };

volatile AT86RF233_REG_IRQ_MASK_t       g_irqMask =
    {
        .bit.pllLock = 1,
        .bit.pllUnlock = 0,
        .bit.rxStart = 1,
        .bit.trxEnd = 1,
        .bit.ccaEdDone = 1,
        .bit.addressMatch = 0,
        .bit.bufferUnderRun = 1,
        .bit.batteryLow = 0
    };