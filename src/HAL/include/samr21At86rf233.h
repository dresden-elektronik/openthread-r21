/*
 * Copyright (c) 2023 dresden elektronik ingenieurtechnik gmbh.
 * All rights reserved.
 *
 * The software in this package is published under the terms of the BSD
 * style license a copy of which has been included with this distribution in
 * the LICENSE.txt file.
 *
 */
#ifndef _AT86RF233_H_
#define _AT86RF233_H_

#include <stdint.h>

#define IEEE_15_4_FRAME_SIZE                128
#define IEEE_15_4_PDSU_SIZE                 127
#define IEEE_15_4_PHR_SIZE                 1
#define IEEE_15_4_CRC_SIZE                  2
#define IEEE_15_4_FCF_SIZE                  2
#define IEEE_15_4_DSN_SIZE                  1
#define IEEE_15_4_PHY_LEN_SIZE              1
#define IEEE_15_4_EXTENDED_ADDR_SIZE        8
#define IEEE_15_4_SHORT_ADDR_SIZE           2
#define IEEE_15_4_PAN_ID_SIZE               2

#define IEEE_15_5_STATIC_HEADER_SIZE        (IEEE_15_4_FCF_SIZE + IEEE_15_4_PHY_LEN_SIZE + IEEE_15_4_DSN_SIZE)

#define IEEE_15_4_24GHZ_TIME_PER_OCTET_us   32
#define IEEE_15_4_AIFS_us                   192
#define IEEE_15_4_ADJUSTED_AIFS_us          180

#define IEEE_15_4_MIN_BACKOFF_EXPONENT      3
#define IEEE_15_4_MAX_BACKOFF_EXPONENT      7

#define AT86RF233_RANDOM_NUMBER_INTERVAL_us 1

#define AT86RF233_RSSI_BASE_VAL_dBm         -94

#define AT86RF233_CMD_REG_READ_MASK                 0b10000000   
#define AT86RF233_CMD_REG_WRITE_MASK                0b11000000

#define AT86RF233_CMD_FRAMEBUFFER_READ              0b00100000   
#define AT86RF233_CMD_FRAMEBUFFER_WRITE             0b01100000 
  
#define AT86RF233_CMD_SRAM_READ                     0b00000000
#define AT86RF233_CMD_SRAM_WRITE                    0b01000000

#define SAMR21_NUM_CPU_WAIT_CYCLES_FOR_FRAME_BUFFER_EMPTY_FLAG      36  // 750ns

#ifdef __CONSERVATIVE_TRX_SPI_TIMING__
    #define SAMR21_NUM_CPU_WAIT_CYCLES_BETWEEN_SPI_BYTES                12  // 250ns
    #define SAMR21_NUM_CPU_WAIT_CYCLES_BETWEEN_SPI_BYTES_FAST_ACCESS    24  // 500ns
#endif

enum{
    TRX_STATUS_REG_ADDR    =0x01,
    TRX_STATE_REG_ADDR     =0x02,
    TRX_CTRL_0_REG_ADDR    =0x03,
    TRX_CTRL_1_REG_ADDR    =0x04,
    PHY_TX_PWR_REG_ADDR    =0x05,
    PHY_RSSI_REG_ADDR      =0x06,
    PHY_ED_LEVEL_REG_ADDR  =0x07,
    PHY_CC_CCA_REG_ADDR    =0x08,
    CCA_THRES_REG_ADDR     =0x09,
    RX_CTRL_REG_ADDR       =0x0a,
    SFD_VALUE_REG_ADDR     =0x0b,
    TRX_CTRL_2_REG_ADDR    =0x0c,
    ANT_DIV_REG_ADDR       =0x0d,
    IRQ_MASK_REG_ADDR      =0x0e,
    IRQ_STATUS_REG_ADDR    =0x0f,
    VREG_CTRL_REG_ADDR     =0x10,
    BATMON_REG_ADDR        =0x11,
    XOSC_CTRL_REG_ADDR     =0x12,
    RX_SYN_REG_ADDR        =0x15,
    TRX_RPC_REG_ADDR       =0x16,
    XAH_CTRL_1_REG_ADDR    =0x17,
    FTN_CTRL_REG_ADDR      =0x18,
    XAH_CTRL_2_REG_ADDR    =0x19,
    PLL_CF_REG_ADDR        =0x1a,
    PLL_DCU_REG_ADDR       =0x1b,
    PART_NUM_REG_ADDR      =0x1c,
    VERSION_NUM_REG_ADDR   =0x1d,
    MAN_ID_0_REG_ADDR      =0x1e,
    MAN_ID_1_REG_ADDR      =0x1f,
    SHORT_ADDR_0_REG_ADDR  =0x20,
    SHORT_ADDR_1_REG_ADDR  =0x21,
    PAN_ID_0_REG_ADDR      =0x22,
    PAN_ID_1_REG_ADDR      =0x23,
    IEEE_ADDR_0_REG_ADDR   =0x24,
    IEEE_ADDR_1_REG_ADDR   =0x25,
    IEEE_ADDR_2_REG_ADDR   =0x26,
    IEEE_ADDR_3_REG_ADDR   =0x27,
    IEEE_ADDR_4_REG_ADDR   =0x28,
    IEEE_ADDR_5_REG_ADDR   =0x29,
    IEEE_ADDR_6_REG_ADDR   =0x2a,
    IEEE_ADDR_7_REG_ADDR   =0x2b,
    XAH_CTRL_0_REG_ADDR    =0x2c,
    CSMA_SEED_0_REG_ADDR   =0x2d,
    CSMA_SEED_1_REG_ADDR   =0x2e,
    CSMA_BE_REG_ADDR       =0x2f,
    TST_CTRL_DIGI_REG_ADDR =0x36,
    PHY_TX_TIME_REG_ADDR   =0x3b,
    PHY_PMU_VALUE_REG_ADDR =0x3b,
    TST_AGC_REG_ADDR       =0x3c,
    TST_SDM_REG_ADDR       =0x3d
};

enum{
    TRX_STATUS_P_ON               =0,
    TRX_STATUS_BUSY_RX            =1,
    TRX_STATUS_BUSY_TX            =2,
    TRX_STATUS_RX_ON              =6,
    TRX_STATUS_TRX_OFF            =8,
    TRX_STATUS_PLL_ON             =9,
    TRX_STATUS_SLEEP              =15,
    TRX_STATUS_BUSY_RX_AACK       =17,
    TRX_STATUS_BUSY_TX_ARET       =18,
    TRX_STATUS_RX_AACK_ON         =22,
    TRX_STATUS_TX_ARET_ON         =25,
    TRX_STATUS_RX_ON_NOCLK        =28,
    TRX_STATUS_RX_AACK_ON_NOCLK   =29,
    TRX_STATUS_BUSY_RX_AACK_NOCLK =30,
    TRX_STATUS_STATE_TRANSITION   =31
};

enum{
    TRX_CMD_NOP             =0,
    TRX_CMD_TX_START        =2,
    TRX_CMD_FORCE_TRX_OFF   =3,
    TRX_CMD_FORCE_PLL_ON    =4,
    TRX_CMD_RX_ON           =6,
    TRX_CMD_TRX_OFF         =8,
    TRX_CMD_PLL_ON          =9,
    TRX_CMD_RX_AACK_ON      =22,
    TRX_CMD_TX_ARET_ON      =25
};

typedef struct {

    //TRX_STATUS 0x01
    union {
        struct{
            uint8_t  trxStatus:5;
            uint8_t  reserved:1;
            uint8_t  ccaStatus:1;
            uint8_t  ccaDone:1;
        }bit;
        uint8_t reg;
    }trxStatus;

    //TRX_STATE 0x02
    union {
        struct{
            uint8_t  trxCmd:5;
            uint8_t  tracStatus:3;
        }bit;
        uint8_t reg;
    }trxState;

    //TRX_CTRL_0 0x03
    union {
        struct{
            uint8_t  clkmCtrl:3;
            uint8_t  clkmShaSel:1;
            uint8_t  pmuIfInverse:1;
            uint8_t  pmuEn:1;
            uint8_t  reserved:1;
            uint8_t  tomEn:1;
        }bit;
        uint8_t reg;
    }trxCtrl0;

    //TRX_CTRL_1 0x04
    union {
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
    }trxCtrl1;

    //PHY_TX_PWR 0x05
    union {
        struct{
            uint8_t  txPwr:3;
            uint8_t  reserved:5;
        }bit;
        uint8_t reg;
    }phyTxPwr;

    //PHY_RSSI 0x06
    union {
        struct{
            uint8_t  rssi:5;
            uint8_t  rndValue:2;
            uint8_t  rxCrcValid:1;
        }bit;
        uint8_t reg;
    }phyRssi;

    //PHY_ED_LEVEL 0x07
    union {
        struct{
            uint8_t  edLevel;
        }bit;
        uint8_t reg;
    }phyEdLevel;

    //PHY_CC_CCA 0x08
    union {
        struct{
            uint8_t  channel:5;
            uint8_t  ccaMode:2;
            uint8_t  ccaRequest:1;
        }bit;
        uint8_t reg;
    }phyCcCca;

    //CCA_THRES 0x09
    union {
        struct{
            uint8_t  ccaEdThres:4;
            uint8_t  reserved:4;
        }bit;
        uint8_t reg;
    }ccaThres;

    //RX_CTRL 0x0A
    union {
        struct{
            uint8_t  pdtThres:4;
            uint8_t  reserved:2;
            uint8_t  pelShiftValue:2;
        }bit;
        uint8_t reg;
    }rxCtrl;

    //SFD_VALUE 0x0B
    union {
        struct{
            uint8_t  sfdValue;
        }bit;
        uint8_t reg;
    }sfdValue;

    //TRX_CTRL_2 0x0C
    union {
        struct{
            uint8_t  oqpskDataRate:3;
            uint8_t  reserved:4;
            uint8_t  rxSafeMode:1;
        }bit;
        uint8_t reg;
    }trxCtrl2;

    //ANT_DIV 0x0D
    union {
        struct{
            uint8_t  antCtrl:2;
            uint8_t  antExtSwEn:1;
            uint8_t  antDivEn:1;
            uint8_t  reserved:3;
            uint8_t  antSel:1;
        }bit;
        uint8_t reg;
    }antDiv;

    //IRQ_MASK 0x0E
    union {
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
    }irqMask;

    //IRQ_STATUS 0x0F
    union {
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
    }irqStatus;

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

}ramCopyTrxRegister_t;

#define IEEE_15_4_AES_CCM_BLOCK_SIZE        16
#define IEEE_15_4_AES_CCM_KEY_SIZE          16
#define IEEE_15_4_AES_CCM_NONCE_SIZE        13

#define AT86RF233_AES_BUSY_TIME_us          24
#define AT86RF233_ADJUSTED_AES_BUSY_TIME_us 15

#define AES_STATUS_SRAM_ADDR                0x82
#define AES_CTRL_SRAM_ADDR                  0x83
#define AES_CTRL_MIRROR_SRAM_ADDR           0x94

#define AES_BLOCK_IO_SRAM_ADDR              0x84
#define AES_KEY_SRAM_ADDR                   0x84


enum {
    AT86RF233_AES_MODE_ECB     = 0x00,
    AT86RF233_AES_MODE_KEY     = 0x01,
    AT86RF233_AES_MODE_CBC     = 0x02
};

typedef union {
    struct{
        uint8_t  reserved:3;
        uint8_t  dir:1;
        uint8_t  mode:3;
        uint8_t  request:1;
    }bit;
    uint8_t reg;
}ramCopyTrxAesCtrl_t;

typedef union {
    struct{
        uint8_t  done:1;
        uint8_t  reserved:6;
        uint8_t  error:1;
    }bit;
    uint8_t reg;
}ramCopyTrxAesStatus_t;

#endif //_AT86RF233_BITFIELD_H_