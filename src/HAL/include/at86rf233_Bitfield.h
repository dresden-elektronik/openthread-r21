//Author Eric Härtel @ dresden elektronik ingenieurtechnik gmbh © 2022
#ifndef _AT86RF233_BITFIELD_H_
#define _AT86RF233_BITFIELD_H_

#include "at86rf233.h"
#include <stdint.h>

typedef union {
    struct{
        uint8_t  reserved:4;
        uint8_t  tracStatus:3;
        uint8_t  crcValid:1;
    }bit;
    uint8_t reg;
}AT86RF233_REG_RX_STATUS_t;


typedef union {
    struct{
        uint8_t  trxStatus:5;
        uint8_t  reserved:1;
        uint8_t  ccaStatus:1;
        uint8_t  ccaDone:1;
    }bit;
    uint8_t reg;
}AT86RF233_REG_TRX_STATUS_t;

typedef struct{
    uint8_t  trxCmd:5;
    uint8_t  tracStatus:3;
}AT86RF233_REG_TRX_STATE_t;

typedef union {
    struct{
        uint8_t  clkmCtrl:3;
        uint8_t  clkmShaSel:1;
        uint8_t  pmuIfInverse:1;
        uint8_t  pmuEn:1;
        uint8_t  reserved:1;
        uint8_t  tomEn:1;
    }bit;
    uint8_t reg;
}AT86RF233_REG_TRX_CTRL_0_t;

typedef union {
    struct{
        uint8_t  irqPolarity:1;
        uint8_t  irqMaskMode:1;
        uint8_t  spiCmdMode:2;
        uint8_t  rxBlCtrl:1;
        uint8_t  txAutoCrcOn:1;
        uint8_t  irq2ExtEn:1;
        uint8_t  paExtnEn:1;
    }bit;
    uint8_t reg;
}AT86RF233_REG_TRX_CTRL_1_t;

typedef union {
    struct{
        uint8_t  txPwr:4;
        uint8_t  reserved:4;
        }bit;
    uint8_t reg;
}AT86RF233_REG_PHY_TX_PWR_t;

typedef union {
    struct{
        uint8_t  rssi:5;
        uint8_t  rndValue:2;
        uint8_t  rxCrcValid:1;
    }bit;
    uint8_t reg;
}AT86RF233_REG_PHY_RSSI_t;

typedef union {
    struct{
        uint8_t edLevel;
    }bit;
    uint8_t reg;
}AT86RF233_REG_PHY_ED_LEVEL_t;

typedef union {
    struct{
        uint8_t  channel:5;
        uint8_t  ccaMode:2;
        uint8_t  ccaRequest:1;
    }bit;
    uint8_t reg;
}AT86RF233_REG_PHY_CC_CCA_t;

typedef union {
    struct{
        uint8_t  ccaEdThres:4;
        uint8_t  reserved:4;
    }bit;
    uint8_t reg;
}AT86RF233_REG_CCA_THRES_t;

typedef union {
    struct{
        uint8_t  pdtThres:4;
        uint8_t  reserved:2;
        uint8_t  pelShiftValue:2;
    }bit;
    uint8_t reg;
}AT86RF233_REG_RX_CTRL_t;

typedef union {
    struct{
        uint8_t  sfdValue;
    }bit;
    uint8_t reg;
}AT86RF233_REG_SFD_VALUE_t;

//TODO
    //TRX_CTRL_2_t
    //ANT_DIV_t

typedef union {
    struct{
        uint8_t  pllLock:1;
        uint8_t  pllUnlock:1;
        uint8_t  rxStart:1;
        uint8_t  trxEnd:1;
        uint8_t  ccaEdDone:1;
        uint8_t  addressMatch:1;
        uint8_t  bufferUnderRun:1;
        uint8_t  batteryLow:1;
    }bit;
    uint8_t reg;
}AT86RF233_REG_IRQ_STATUS_t;

typedef union {
    struct{
        uint8_t  pllLock:1;
        uint8_t  pllUnlock:1;
        uint8_t  rxStart:1;
        uint8_t  trxEnd:1;
        uint8_t  ccaEdDone:1;
        uint8_t  addressMatch:1;
        uint8_t  bufferUnderRun:1;
        uint8_t  batteryLow:1;
    }bit;
    uint8_t reg;
}AT86RF233_REG_IRQ_MASK_t;

//TODO
    //VREG_CTRL_t
    //BATMON_t
    //XOSC_CTRL_t
    //CC_CTRL_0_t
    //CC_CTRL_1_t
    //RX_SYN_t
    //TRX_RPC_1_t
    //XAH_CTRL_1_t
    //FTN_CTRL_t
    //XAH_CTRL_2_t
    //PLL_CF_t
    //PLL_DCU_t
    //.....


#endif //_AT86RF233_BITFIELD_H_