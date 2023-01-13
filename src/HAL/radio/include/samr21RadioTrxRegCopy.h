//Author Eric Härtel @ dresden elektronik ingenieurtechnik gmbh © 2022

#ifndef _SAMR21_TRX_REG_COPY_H_
#define _SAMR21_TRX_REG_COPY_H_

#include "at86rf233_Bitfield.h"
#include "at86rf233.h"

//Local Register Copys of At86rf233 to save some Read Acceses
extern AT86RF233_REG_TRX_STATUS_t     g_trxStatus; 
extern AT86RF233_REG_IRQ_STATUS_t     g_trxLastIrq;         
extern AT86RF233_REG_TRX_CTRL_0_t     g_trxCtrl0;  
extern AT86RF233_REG_PHY_CC_CCA_t     g_phyCcCcaReg;
extern AT86RF233_REG_PHY_RSSI_t       g_phyRssi;
extern AT86RF233_REG_CCA_THRES_t      g_ccaThres;
extern AT86RF233_REG_TRX_CTRL_1_t     g_trxCtrl1;
extern AT86RF233_REG_PHY_TX_PWR_t     g_phyTxPwr;
extern AT86RF233_REG_IRQ_MASK_t       g_irqMask;

#endif //_SAMR21_TRX_REG_COPY_H_