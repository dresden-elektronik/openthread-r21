/*
 * Copyright (c) 2023 dresden elektronik ingenieurtechnik gmbh.
 * All rights reserved.
 *
 * The software in this package is published under the terms of the BSD
 * style license a copy of which has been included with this distribution in
 * the LICENSE.txt file.
 *
 */
#include "samr21Trx.h"


static struct trxVars_s
{
    volatile bool spiActive;
} s_trxVars =
    {
        .spiActive = false,
    };

static ramCopyTrxRegister_t s_localTrxRegisterCopy =
    {
        
        // Default Config
        .trxCtrl0.bit.clkmCtrl = 0x1, // 1MHz ClockOutput 
        .trxCtrl0.bit.clkmShaSel = 1, // MCLK Changes take immediate effect

        .trxCtrl1.bit.irqPolarity = 0,
        .trxCtrl1.bit.irqMaskMode = 0,
        .trxCtrl1.bit.spiCmdMode = 0x1,
        .trxCtrl1.bit.rxBlCtrl = 1,     //Needed for Live Download while RX
        .trxCtrl1.bit.txAutoCrcOn = 1, //Stop uploading when the CRC is due, TRX inserts the correct one
        .trxCtrl1.bit.irq2ExtEn = 0,

#ifdef _GCF_RELEASE_
        .trxCtrl1.bit.paExtnEn = 1,
        .phyTxPwr.bit.txPwr = 0x7
#else
        .trxCtrl1.bit.paExtnEn = 0,
        .phyTxPwr.bit.txPwr = 0x0
#endif
};

static void trx_dmaDone_cb(void);

void samr21Trx_initInterface()
{
    // Setup Ports for TRX
    // Setup PIN PC19 as MISO <-> At86rf233 via SERCOM4
    // Make Input
    PORT->Group[2].DIRCLR.reg = PORT_PC19;

    // Setup Mux Settings
    PORT->Group[2].WRCONFIG.reg =
        PORT_WRCONFIG_HWSEL | PORT_WRCONFIG_WRPINCFG | PORT_WRCONFIG_WRPMUX | PORT_WRCONFIG_PMUX(MUX_PC19F_SERCOM4_PAD0)
        //|PORT_WRCONFIG_PULLEN
        | PORT_WRCONFIG_INEN | PORT_WRCONFIG_PMUXEN | PORT_WRCONFIG_PINMASK(PORT_PC19 >> 16) // upper Halfword
        ;

    // Setup PIN PB30 as MOSI <-> At86rf233 via SERCOM4
    // Make Output
    PORT->Group[1].DIRSET.reg = PORT_PB30;

    // Setup Mux Settings
    PORT->Group[1].WRCONFIG.reg =
        PORT_WRCONFIG_HWSEL | PORT_WRCONFIG_WRPINCFG | PORT_WRCONFIG_WRPMUX | PORT_WRCONFIG_PMUX(MUX_PB30F_SERCOM4_PAD2)
        //|PORT_WRCONFIG_PULLEN
        //|PORT_WRCONFIG_INEN
        | PORT_WRCONFIG_PMUXEN | PORT_WRCONFIG_PINMASK(PORT_PB30 >> 16) // upper Halfword
        ;

    // Setup PIN PC18 as SCLK <-> At86rf233 via SERCOM4
    // Make Output
    PORT->Group[2].DIRSET.reg = PORT_PC18;

    // Setup Mux Settings
    PORT->Group[2].WRCONFIG.reg =
        PORT_WRCONFIG_HWSEL | PORT_WRCONFIG_WRPINCFG | PORT_WRCONFIG_WRPMUX | PORT_WRCONFIG_PMUX(MUX_PC18F_SERCOM4_PAD3)
        //|PORT_WRCONFIG_PULLEN
        //|PORT_WRCONFIG_INEN
        | PORT_WRCONFIG_PMUXEN | PORT_WRCONFIG_PINMASK(PORT_PC18 >> 16) // upper Halfword
        ;

    // Setup PIN PB31 as SEL <-> At86rf233
    // SEL is configreud as Mnual PORT, cause we need Control over ist for
    // Realtime Buffer Read
    // Make Output
    PORT->Group[1].DIRSET.reg = PORT_PB31;

    // Setup Mux Settings
    PORT->Group[1].WRCONFIG.reg =
        PORT_WRCONFIG_HWSEL | PORT_WRCONFIG_WRPINCFG
        //|PORT_WRCONFIG_WRPMUX
        //|PORT_WRCONFIG_PMUX(0)
        //|PORT_WRCONFIG_PULLEN
        //|PORT_WRCONFIG_INEN
        //|PORT_WRCONFIG_PMUXEN
        | PORT_WRCONFIG_PINMASK(PORT_PB31 >> 16) // upper Halfword
        ;

    // Set Default High
    PORT->Group[1].OUT.reg |= ((uint32_t)1 & 0x1) << 31;

    // Setup PIN PB15 as RSTN <-> At86rf233
    // Make Output
    PORT->Group[1].DIRSET.reg = PORT_PB15;

    // Setup Mux Settings
    PORT->Group[1].WRCONFIG.reg =
        // PORT_WRCONFIG_HWSEL
        PORT_WRCONFIG_WRPINCFG
        //|PORT_WRCONFIG_WRPMUX
        //|PORT_WRCONFIG_PMUX(0)
        //|PORT_WRCONFIG_PULLEN
        //|PORT_WRCONFIG_INEN
        //|PORT_WRCONFIG_PMUXEN
        | PORT_WRCONFIG_PINMASK(PORT_PB15) // lower Halfword
        ;

    // Set Default High
    PORT->Group[1].OUT.reg |= ((uint32_t)1 & 0x1) << 15;
    // Setup PIN PB00 as IRQ <-> At86rf233 via EXTINT[0]
    // Make Input
    PORT->Group[1].DIRCLR.reg = PORT_PB00;

    // Setup Mux Settings
    PORT->Group[1].WRCONFIG.reg =
        // PORT_WRCONFIG_HWSEL
        PORT_WRCONFIG_WRPINCFG | PORT_WRCONFIG_WRPMUX | PORT_WRCONFIG_PMUX(MUX_PB00A_EIC_EXTINT0)
        //|PORT_WRCONFIG_PULLEN
        | PORT_WRCONFIG_INEN | PORT_WRCONFIG_PMUXEN | PORT_WRCONFIG_PINMASK(PORT_PB00) // lower Halfword
        ;
    // Setup PIN PA20 as SLP_TR <-> At86rf233
    // Make Output
    PORT->Group[0].DIRSET.reg = PORT_PA20;

    // Setup Mux Settings
    PORT->Group[0].WRCONFIG.reg =
        PORT_WRCONFIG_HWSEL | PORT_WRCONFIG_WRPINCFG
        //|PORT_WRCONFIG_WRPMUX
        //|PORT_WRCONFIG_PMUX(0)
        //|PORT_WRCONFIG_PULLEN
        //|PORT_WRCONFIG_INEN
        //|PORT_WRCONFIG_PMUXEN
        | PORT_WRCONFIG_PINMASK(PORT_PA20 >> 16) // upper Halfword
        ;

    // Set Default Low
    PORT->Group[0].OUT.reg |= ((uint32_t)0 & 0x1) << 20;

    // Enable EIC in Power Manager
    PM->APBCMASK.bit.SERCOM4_ = 1;
    
    // Reset SERCOM4
    SERCOM4->SPI.CTRLA.bit.SWRST = 1;

    // Wait for SERCOM4 reset to finish
    while (SERCOM4->SPI.CTRLA.bit.SWRST || SERCOM4->SPI.SYNCBUSY.bit.SWRST)
        ;

    // Setup SERCOM4

    // F_ref = 8MHz (or 1 MHz on fallback Clocktree) F_baud = F_ref / 2*(BAUD+1) ---> BAUD = 0 F_baud = 4MBauds/s (or 500kBaud/s on fallback Clocktree)
    SERCOM4->SPI.BAUD.reg =
        SERCOM_SPI_BAUD_BAUD(0x0);

    SERCOM4->SPI.CTRLB.reg =
        SERCOM_SPI_CTRLB_RXEN
        //|SERCOM_SPI_CTRLB_AMODE(0)
        //|SERCOM_SPI_CTRLB_MSSEN
        //|SERCOM_SPI_CTRLB_SSDE
        //|SERCOM_SPI_CTRLB_PLOADEN
        | SERCOM_SPI_CTRLB_CHSIZE(0x0) // 8Bit
        ;
    // Wait for SERCOM4 Sync
    while (SERCOM4->SPI.SYNCBUSY.bit.CTRLB)
        ;

    SERCOM4->SPI.CTRLA.reg =
        //SERCOM_SPI_CTRLA_RUNSTDBY 
        SERCOM_SPI_CTRLA_MODE(SERCOM_SPI_CTRLA_MODE_SPI_MASTER_Val) | SERCOM_SPI_CTRLA_ENABLE
        //|SERCOM_SPI_CTRLA_SWRST
        //|SERCOM_SPI_CTRLA_IBON
        | SERCOM_SPI_CTRLA_DOPO(1) // MOSI IS ON PB30 SERCOM4/PAD[2] , SCLK ON PB30 SERCOM4/PAD[3]
        | SERCOM_SPI_CTRLA_DIPO(0) // MISO IS ON PC19 SERCOM4/PAD[0]
        | SERCOM_SPI_CTRLA_FORM(0)
        //|SERCOM_SPI_CTRLA_CPHA
        //|SERCOM_SPI_CTRLA_CPOL
        //|SERCOM_SPI_CTRLA_DORD
        ;
    // Wait for SERCOM4 to setup
    while (SERCOM4->SPI.SYNCBUSY.reg)
        ;

    // Write predefined Operating-Values to Config Registers of the AT86rf233

    // Note: this should actully be written, but the Trx resets to desired Values anyways
    //       making changes to this causes wierd clock glitches not worth dealing with
    //
    samr21Trx_writeRegister(TRX_CTRL_0_REG_ADDR, s_localTrxRegisterCopy.trxCtrl0.reg);
    
    samr21Trx_writeRegister(TRX_CTRL_1_REG_ADDR, s_localTrxRegisterCopy.trxCtrl1.reg);
    
    //
    samr21Trx_writeRegister(PHY_TX_PWR_REG_ADDR, s_localTrxRegisterCopy.phyTxPwr.reg);
}

void samr21Trx_initLocalDriver()
{
    // Enable Timer 5 for Trx Orchestration
    samr21Timer5_init(0, true, true); // 1MHz / (2^0) -> 1us resolution

    // Enable Timer 3 for DMA Paceing
    samr21Timer3_init(0, false, false); // 1MHz / (2^0) -> 1us resolution

    samr21Dma_initChannel(
        0, //Channel 0 cause it has the highest Priority
        (uint32_t)(&SERCOM4->SPI.DATA.reg), //Data input of SERCOM4 Register
        0x18, // TC3 Overflow Trigger
        trx_dmaDone_cb
    );
}

void samr21Trx_initInterrupts()
{
    // Get TRX into IDLE (STATUS_TRX_OFF) State
    samr21Trx_forceMoveToIdle(true);

    // Enable EIC in Power Manager
    PM->APBAMASK.bit.EIC_ = 1;

    // Reset first and wait for reset to finish
    EIC->CTRL.bit.SWRST = 1;
    while (EIC->STATUS.bit.SYNCBUSY)
        ;
    while (EIC->CTRL.bit.SWRST)
        ;

    // Enable EXINT[0]
    EIC->CONFIG[0].bit.FILTEN0 = 0;
    EIC->CONFIG[0].bit.SENSE0 = EIC_CONFIG_SENSE0_RISE_Val;
    EIC->INTENSET.bit.EXTINT0 = 1;

    // Enable Module
    EIC->CTRL.bit.ENABLE = 1;
    while (EIC->STATUS.bit.SYNCBUSY)
        ;
    while (!(EIC->CTRL.bit.ENABLE))
        ;

    // Read Once to clear all pending Interrupts
    samr21Trx_readRegister(IRQ_STATUS_REG_ADDR);

    // Clear Irqflag once before returning
    EIC->INTFLAG.bit.EXTINT0 = 1;
}

void samr21Trx_deinitInterrupts(void)
{
    if (EIC->CTRL.bit.ENABLE)
    {
        // Disable EIC Module
        EIC->CTRL.bit.ENABLE = 0;
        while ((EIC->CTRL.bit.ENABLE))
            ;

        // Reset after
        EIC->CTRL.bit.SWRST = 1;
        while (EIC->STATUS.bit.SYNCBUSY)
            ;
        while (EIC->CTRL.bit.SWRST)
            ;
    }


    // Disable EIC in Power Manager
    if (PM->APBAMASK.bit.EIC_)
    {
        PM->APBAMASK.bit.EIC_ = 0;
    }
}

void samr21Trx_setupMClk(uint8_t a_clk)
{

    // Write desired Value into the CLKM Register ( e.g.: 0x05 -> 16Mhz)
    s_localTrxRegisterCopy.trxCtrl0.bit.clkmCtrl = a_clk;

    samr21Trx_writeRegister(TRX_CTRL_0_REG_ADDR, s_localTrxRegisterCopy.trxCtrl0.reg);

    // Wait a bit for the  the Freq.-change to take place
    samr21SysTick_delayTicks(50000);
}

uint8_t samr21Trx_readRegister(uint8_t a_addr)
{

    samr21Trx_spiStartAccess(AT86RF233_CMD_REG_READ_MASK, a_addr);

#ifdef __CONSERVATIVE_TRX_SPI_TIMING__
    samr21SysTick_delayTicks(CPU_WAIT_CYCLES_BETWEEN_BYTES);
#endif

    uint8_t rVal = samr21Trx_spiReadByteRaw();

    samr21Trx_spiCloseAccess();

    return rVal;
}

void samr21Trx_writeRegister(uint8_t a_addr, uint8_t a_data)
{

    samr21Trx_spiStartAccess(AT86RF233_CMD_REG_WRITE_MASK, a_addr);

#ifdef __CONSERVATIVE_TRX_SPI_TIMING__
    samr21SysTick_delayTicks(CPU_WAIT_CYCLES_BETWEEN_BYTES);
#endif

    samr21Trx_spiTransceiveByteRaw(a_data);

    samr21Trx_spiCloseAccess();
}

void samr21Trx_spiStartAccess(uint8_t a_command, uint8_t a_addr)
{
    // Wait for other ongoing SPI Access to End
    while (s_trxVars.spiActive)
        ;

    NVIC_DisableIRQ(EIC_IRQn);
    NVIC_DisableIRQ(TC4_IRQn);

    s_trxVars.spiActive = true;
    PORT->Group[1].OUTCLR.reg = 1 << 31; // SSel Low Active

#ifdef __CONSERVATIVE_TRX_SPI_TIMING__
    samr21SysTick_delayTicks(CPU_WAIT_CYCLES_AFTER_SSEL_LOW);
#endif

    // Register Read/Write
    if (a_command & AT86RF233_CMD_REG_READ_MASK)
    {
        //There is always the trxStatus in the first byte of a SPI transaction (See 35.4 Radio Transceiver Status Information SAMR21 - Datasheet)
        s_localTrxRegisterCopy.trxStatus.reg = samr21Trx_spiTransceiveByteRaw((a_addr & 0x3F) | a_command);
        return;
    }

    // Framebuffer Access
    if (a_command & AT86RF233_CMD_FRAMEBUFFER_READ)
    {
        //There is always the trxStatus in the first byte of a SPI transaction (See 35.4 Radio Transceiver Status Information SAMR21 - Datasheet)
        s_localTrxRegisterCopy.trxStatus.reg = samr21Trx_spiTransceiveByteRaw(a_command);
        return;
    }

    // SRAM Access
        //There is always the trxStatus in the first byte of a SPI transaction (See 35.4 Radio Transceiver Status Information SAMR21 - Datasheet)
    s_localTrxRegisterCopy.trxStatus.reg = samr21Trx_spiTransceiveByteRaw(a_command);

#ifdef __CONSERVATIVE_TRX_SPI_TIMING__
    samr21SysTick_delayTicks(CPU_WAIT_CYCLES_BETWEEN_BYTES);
#endif
    samr21Trx_spiTransceiveByteRaw(a_addr);
}

void samr21Trx_spiCloseAccess()
{
    PORT->Group[1].OUTSET.reg = 1 << 31; // SSel Low Active

#ifdef __CONSERVATIVE_TRX_SPI_TIMING__
    samr21SysTick_delayTicks(CPU_WAIT_CYCLES_AFTER_SSEL_LOW);
#endif
    s_trxVars.spiActive = false;

    NVIC_EnableIRQ(EIC_IRQn);
    NVIC_EnableIRQ(TC4_IRQn);
}

void samr21Trx_updateStatusRegister()
{

    //Manual samr21Trx_spiStartAccess
    while (s_trxVars.spiActive)
        ;
    NVIC_DisableIRQ(EIC_IRQn);
    NVIC_DisableIRQ(TC4_IRQn);
    PORT->Group[1].OUTCLR.reg = 1 << 31; // SSel Low Active
    s_trxVars.spiActive = true;

    //There is always the trxStatus in the first byte of a SPI transaction (See 35.4 Radio Transceiver Status Information SAMR21 - Datasheet)
    s_localTrxRegisterCopy.trxStatus.reg = samr21Trx_spiTransceiveByteRaw(SPI_DUMMY_BYTE);

    samr21Trx_spiCloseAccess();
}

void samr21Trx_setResetPin(bool a_enable)
{
    // Pin PB15 == RSTN
    // Active Low
    if (a_enable)
    {
        PORT->Group[1].OUTCLR.reg = 1 << 15;
        return;
    }
    PORT->Group[1].OUTSET.reg = 1 << 15;
}

void samr21Trx_setSleepTransmitPin(bool a_enable)
{
    // Pin PA20 == SLP_TR
    // Active High
    if (a_enable)
    {
        PORT->Group[0].OUTSET.reg = 1 << 20;
        return;
    }
    PORT->Group[0].OUTCLR.reg = 1 << 20;
}

uint8_t samr21Trx_spiTransceiveByteRaw(uint8_t a_data)
{
    while (!SERCOM4->SPI.INTFLAG.bit.DRE)
        ;
    // Put data into the tranmitt buffer to start transmission
    SERCOM4->SPI.DATA.bit.DATA = a_data;

    while (!SERCOM4->SPI.INTFLAG.bit.TXC)
        ;
    while (!SERCOM4->SPI.INTFLAG.bit.RXC)
        ;

    // return recived Answerto
    return SERCOM4->SPI.DATA.bit.DATA;
}

void samr21Trx_spiSendByteRawIgnoreResponse(uint8_t a_data)
{
    // Put data into the tranmitt buffer to start transmission
    SERCOM4->SPI.DATA.bit.DATA = a_data;

    // Wait for Transmit to Complete
    while (!SERCOM4->SPI.INTFLAG.bit.TXC)
        ;
}

void samr21Trx_spiTransceiveBytesRaw(uint8_t *a_inData_1D, uint8_t *a_outData_1D, uint8_t a_len)
{
    for (uint8_t i = 0; i < a_len; i++)
    {

#ifdef __CONSERVATIVE_TRX_SPI_TIMING__
        samr21SysTick_delayTicks(SAMR21_NUM_CPU_WAIT_CYCLES_BETWEEN_SPI_BYTES);
#endif

        if (!a_outData_1D)
        {
            samr21Trx_spiSendByteRawIgnoreResponse(a_inData_1D[i]);
            continue;
        }

        a_outData_1D[i] = samr21Trx_spiTransceiveByteRaw(a_inData_1D != NULL ? a_inData_1D[i] : SPI_DUMMY_BYTE);
    }
}

uint8_t samr21Trx_spiReadByteRaw()
{
    return samr21Trx_spiTransceiveByteRaw(SPI_DUMMY_BYTE);
}

/****************STATE CHANGES************************/
static const uint8_t sc_validRxTrxStates[] =
    {
        TRX_STATUS_BUSY_RX,
        TRX_STATUS_RX_ON};

static const uint8_t sc_validTxTrxStates[] =
    {
        TRX_STATUS_PLL_ON,
        TRX_STATUS_BUSY_TX};

static const uint8_t sc_validIdleTrxStates[] =
    {
        TRX_STATUS_TRX_OFF,
        TRX_STATUS_SLEEP,
        TRX_STATUS_P_ON};

static void samr21Trx_blockingStateChange(uint8_t a_cmd, const uint8_t *a_desiredStates, uint8_t a_numDesiredStates)
{
    samr21Trx_writeRegister(TRX_STATE_REG_ADDR, a_cmd);

    while (true)
    {
        uint8_t currentState = samr21Trx_readRegister(TRX_STATUS_REG_ADDR);

        for (uint8_t i = 0; i < a_numDesiredStates; i++)
        {
            if ((currentState & 0b00011111) == a_desiredStates[i])
            {
                return;
            }
        }
    }
}

void samr21Trx_queueMoveToRx(bool a_blocking)
{
    if (!a_blocking)
    {
        samr21Trx_writeRegister(TRX_STATE_REG_ADDR, TRX_CMD_RX_ON);
        return;
    }

    samr21Trx_blockingStateChange(TRX_CMD_RX_ON, sc_validRxTrxStates, sizeof(sc_validRxTrxStates));
}

void samr21Trx_queueMoveToTx(bool a_blocking)
{
    if (!a_blocking)
    {
        samr21Trx_writeRegister(TRX_STATE_REG_ADDR, TRX_CMD_PLL_ON);
        return;
    }

    samr21Trx_blockingStateChange(TRX_CMD_PLL_ON, sc_validTxTrxStates, sizeof(sc_validTxTrxStates));
}

void samr21Trx_forceMoveToTx(bool a_blocking)
{
    if (!a_blocking)
    {
        samr21Trx_writeRegister(TRX_STATE_REG_ADDR, TRX_CMD_FORCE_PLL_ON);
        return;
    }

    samr21Trx_blockingStateChange(TRX_CMD_FORCE_PLL_ON, sc_validTxTrxStates, sizeof(sc_validTxTrxStates));
}

void samr21Trx_queueMoveToIdle(bool a_blocking)
{
    if (!a_blocking)
    {
        samr21Trx_writeRegister(TRX_STATE_REG_ADDR, TRX_CMD_TRX_OFF);
        return;
    }

    samr21Trx_blockingStateChange(TRX_CMD_TRX_OFF, sc_validIdleTrxStates, sizeof(sc_validIdleTrxStates));
}

void samr21Trx_forceMoveToIdle(bool a_blocking)
{
    if (!a_blocking)
    {
        samr21Trx_writeRegister(TRX_STATE_REG_ADDR, TRX_CMD_FORCE_TRX_OFF);
        return;
    }

    samr21Trx_blockingStateChange(TRX_CMD_FORCE_TRX_OFF, sc_validIdleTrxStates, sizeof(sc_validIdleTrxStates));
}

bool samr21Trx_getCcaResult()
{
    return s_localTrxRegisterCopy.trxStatus.bit.ccaStatus;
}

/*************************Channel Control*************************************/
void samr21Trx_setActiveChannel(uint8_t a_channel)
{
#ifdef _DEBUG
    assert(a_channel >= 0x0B && a_channel <= 0x1A);
#endif
    if (a_channel == s_localTrxRegisterCopy.phyCcCca.bit.channel)
    {
        return;
    }

    s_localTrxRegisterCopy.phyCcCca.bit.channel = a_channel;

    samr21Trx_writeRegister(PHY_CC_CCA_REG_ADDR, s_localTrxRegisterCopy.phyCcCca.reg);
}

uint8_t samr21Trx_getAktiveChannel()
{
    return s_localTrxRegisterCopy.phyCcCca.bit.channel;
}

void samr21Trx_startCca()
{
    // Prepare CCA
    s_localTrxRegisterCopy.phyCcCca.bit.ccaRequest = 1;

    // Start CCA
    samr21Trx_writeRegister(PHY_CC_CCA_REG_ADDR, s_localTrxRegisterCopy.phyCcCca.reg);

    // Reset local copy of ccaRequest Bit
    s_localTrxRegisterCopy.phyCcCca.bit.ccaRequest = 0;
}

void samr21Trx_startEd()
{
    // Start ED by writing any value to PHY_ED_LEVEL_REG_ADDR
    // See samr21 datasheet: 37.5 Energy Detection (ED)
    samr21Trx_writeRegister(PHY_ED_LEVEL_REG_ADDR, 0xE7);
}

int8_t samr21Trx_getLastRssiValue()
{

    if (
        s_localTrxRegisterCopy.trxStatus.bit.trxStatus != TRX_STATUS_RX_ON && s_localTrxRegisterCopy.trxStatus.bit.trxStatus != TRX_STATUS_RX_ON)
    {
        return INT8_MAX;
    }

    return AT86RF233_RSSI_BASE_VAL_dBm + (int8_t)samr21Trx_readRegister(PHY_ED_LEVEL_REG_ADDR);
}

/*************************CCA Control*************************************/

void samr21Trx_setCcaMode(uint8_t a_newCcaMode)
{
    s_localTrxRegisterCopy.phyCcCca.bit.ccaMode = a_newCcaMode;
    samr21Trx_writeRegister(PHY_CC_CCA_REG_ADDR, s_localTrxRegisterCopy.phyCcCca.reg);
}

uint8_t samr21Radio_getCCAMode()
{
    return s_localTrxRegisterCopy.phyCcCca.bit.ccaMode;
}

void samr21Trx_setCcaThreshold(int8_t a_threshold)
{
    int8_t diff = (AT86RF233_RSSI_BASE_VAL_dBm - a_threshold);
    s_localTrxRegisterCopy.ccaThres.bit.ccaEdThres = abs(diff) >> 1; // Devide by 2

    samr21Trx_writeRegister(CCA_THRES_REG_ADDR, s_localTrxRegisterCopy.ccaThres.reg);
}

int8_t samr21Trx_getCcaThreshold()
{
    return (AT86RF233_RSSI_BASE_VAL_dBm + (s_localTrxRegisterCopy.ccaThres.bit.ccaEdThres << 1)); // Multiply by 2
}

/*************************Tx Power Control*************************************/
// TX Power
#define SIZE_AT86RF233_TX_POWER_TABLE 16
#ifdef _GCF_RELEASE_
static const float sc_txPowerTable_dBm_1D[SIZE_AT86RF233_TX_POWER_TABLE] = {11.9, 11.7, 11.6, 11.5, 11.3, 11.2, 11, 10.7, 10.4, 10.1, 9.3, 8.2, 6.4, 3.9, 0.1, -5.8};
#else
static const float sc_txPowerTable_dBm_1D[SIZE_AT86RF233_TX_POWER_TABLE] = {4, 3.7, 3.4, 3, 2.5, 2, 1, 0, -1, -2, -3, -4, -6, -8, -12, -17};
#endif

void samr21Trx_setTxPower(int8_t a_power_dbm)
{
    for (uint8_t i = 0; i < SIZE_AT86RF233_TX_POWER_TABLE; i++)
    {
        if (
            (sc_txPowerTable_dBm_1D[i] >= a_power_dbm) && (sc_txPowerTable_dBm_1D[i + 1] <= a_power_dbm))
        {
            s_localTrxRegisterCopy.phyTxPwr.bit.txPwr = i;
            goto exit;
        }
    }

    // Default
    s_localTrxRegisterCopy.phyTxPwr.bit.txPwr = 0x7;

exit:
    samr21Trx_writeRegister(PHY_TX_PWR_REG_ADDR, s_localTrxRegisterCopy.phyTxPwr.reg);
}

int8_t samr21Trx_getTxPower()
{
    return sc_txPowerTable_dBm_1D[s_localTrxRegisterCopy.phyTxPwr.bit.txPwr];
}

/***************Timer-Handler for Trx-Operational Timing***************************/
static volatile bool *s_timeoutFlag = NULL;
static void (*s_currentTimerHandler_fktPtr)(void) = NULL;

// NVIC IRQ-Handler Function for Timer5 (TC5). Fkt-Addr is found in NVIC-IRQ-Vector-Table
void TC5_Handler()
{
    // Reset IRQ
    TC5->COUNT16.INTFLAG.bit.OVF = 1;

    if (s_currentTimerHandler_fktPtr)
    {
        // Invoke Handler
        (*s_currentTimerHandler_fktPtr)();
    }
}

static void trx_timeoutHandler()
{
    *s_timeoutFlag = true;
    s_currentTimerHandler_fktPtr = NULL;
}

static void trx_startTimeoutTimer(uint16_t a_duration_us, volatile bool *a_triggerFlag)
{
    s_timeoutFlag = a_triggerFlag;
    s_currentTimerHandler_fktPtr = &trx_timeoutHandler;
    samr21Timer5_startOneshot(a_duration_us);
}

static void trx_stopTimeoutTimer()
{
    samr21Timer5_stop();
    s_timeoutFlag = NULL;
    s_currentTimerHandler_fktPtr = NULL;
}

static void trx_queueDelayedAction(uint16_t a_delay_us, void (*a_queuedAction_fktPtr)(void))
{
    s_currentTimerHandler_fktPtr = a_queuedAction_fktPtr;
    samr21Timer5_startOneshot(a_delay_us);
}

static void trx_removeQueuedAction()
{
    s_currentTimerHandler_fktPtr = NULL;
    samr21Timer5_stop();
}


/***************True Random Engine***************************/
uint8_t samr21Trx_getRandomCrumb()
{
    return (samr21Trx_readRegister(PHY_RSSI_REG_ADDR) >> 5) & 0b00000011;
}
uint8_t samr21Trx_getRandomNibble()
{
    return ((samr21Trx_readRegister(PHY_RSSI_REG_ADDR) >> 5) & 0b00000011) | ((samr21Trx_readRegister(PHY_RSSI_REG_ADDR) >> 3) & 0b00001100);
}
uint8_t samr21Trx_getRandomByte()
{
    uint8_t rVal = (samr21Trx_readRegister(PHY_RSSI_REG_ADDR) & 0x60) >> 5;
    rVal |= (samr21Trx_readRegister(PHY_RSSI_REG_ADDR) & 0x60) >> 3;
    rVal |= (samr21Trx_readRegister(PHY_RSSI_REG_ADDR) & 0x60) >> 1;
    rVal |= (samr21Trx_readRegister(PHY_RSSI_REG_ADDR) & 0x60) << 1;
    return rVal;
}


/****************FRAMEBUFFER READ ACCESS************************/
bool samr21Trx_realtimeFramebufferDownload(uint8_t *a_data_p, uint8_t a_len)
{

#ifdef _DEBUG
    assert(a_len <= IEEE_802_15_4_FRAME_SIZE);
    assert(a_data_p);
#endif

    uint8_t numDownloadedBytes = 0;

    // Set a Timeout
    volatile bool abortActionFlag = false;
    trx_startTimeoutTimer((((uint16_t)(a_len)*2) * IEEE_15_4_24GHZ_TIME_PER_OCTET_us), &abortActionFlag);

    samr21Trx_spiStartAccess(AT86RF233_CMD_FRAMEBUFFER_READ, 0);

    a_data_p[numDownloadedBytes++] = samr21Trx_spiReadByteRaw();

    while (numDownloadedBytes < a_len)
    {
        //(see r21 datasheet, 40.7 Frame Buffer Empty Indicator)
        samr21SysTick_delayTicks(SAMR21_NUM_CPU_WAIT_CYCLES_FOR_FRAME_BUFFER_EMPTY_FLAG);
        while ((PORT->Group[1].IN.reg & PORT_PB00) && !abortActionFlag)
            ;

        // Abort if a Timeout happened
        if (abortActionFlag)
        {
            goto exit;
        }

        a_data_p[numDownloadedBytes++] = samr21Trx_spiReadByteRaw();
    }
    trx_stopTimeoutTimer();

exit:
    samr21Trx_spiCloseAccess();
    return !abortActionFlag;
}

bool samr21Trx_downloadReceivedFramebuffer(uint8_t *a_psduLen, uint8_t *a_psdu_p, uint8_t *a_LQI_p, int8_t *a_RSSI_p, bool a_live)
{

#ifdef _DEBUG
    assert(a_psduLen);
    assert(a_psdu_p);
    assert(a_LQI);
    assert(a_RSSI);
#endif

    uint8_t numDownloadedBytes = 0;
    bool validFrame = true;

    samr21Trx_spiStartAccess(AT86RF233_CMD_FRAMEBUFFER_READ, 0);

    // First Byte Downloaded is the psduLen
    *a_psduLen = samr21Trx_spiReadByteRaw();

    // Check if the Framelenght is within limits outlined by 802.15.4
    if (*a_psduLen > IEEE_15_4_PDSU_SIZE)
    {
        validFrame = false;
        goto exit;
    }

    // Set a Timeout
    volatile bool abortActionFlag = false;
    trx_startTimeoutTimer((IEEE_15_4_FRAME_SIZE * IEEE_15_4_24GHZ_TIME_PER_OCTET_us), &abortActionFlag); // Shift for cheap multiply x2

    while (numDownloadedBytes < *a_psduLen)
    {
        //(see r21 datasheet, 40.7 Frame Buffer Empty Indicator)
        samr21SysTick_delayTicks(SAMR21_NUM_CPU_WAIT_CYCLES_FOR_FRAME_BUFFER_EMPTY_FLAG);

        if (a_live)
        {
            while ((PORT->Group[1].IN.reg & PORT_PB00) && !abortActionFlag)
                ;
        }

        if (abortActionFlag)
        {
            validFrame = false;
            goto exit;
        }

        a_psdu_p[(numDownloadedBytes++)] = samr21Trx_spiReadByteRaw();
    }
    trx_stopTimeoutTimer();

    // Download LQI, RSSI and CRC Check (r21 Datasheet 35.3.2 -  Frame Buffer Access Mode)
    *a_LQI_p = samr21Trx_spiReadByteRaw();
    *a_RSSI_p = AT86RF233_RSSI_BASE_VAL_dBm + samr21Trx_spiReadByteRaw();

    // Check if CRC is correct (samr21 Datasheet 35.3.2 Frame Buffer Access Mode - Structure of RX_STATUS)
    validFrame &= samr21Trx_spiReadByteRaw() >> 7;

exit:
    samr21Trx_spiCloseAccess();
    return validFrame;
}

/****************FRAMEBUFFER WRITE ACCESS************************/
void samr21Trx_uploadToFramebuffer(uint8_t *a_data_p, uint8_t a_len, uint8_t a_pos)
{

#ifdef _DEBUG
    assert(a_len <= IEEE_802_15_4_FRAME_SIZE);
    assert((a_pos + a_len) <= IEEE_802_15_4_FRAME_SIZE);
    assert(a_data_p);
#endif

    samr21Trx_spiStartAccess(AT86RF233_CMD_SRAM_WRITE, a_pos);
    samr21Trx_spiTransceiveBytesRaw(a_data_p, NULL, a_len); // phyLen not in psduLen
    samr21Trx_spiCloseAccess();
}



static DmacDescriptor s_framebufferDmaDescriptor =
{
    .BTCTRL.bit.VALID = 1, //Set to True when transfer starts
    .BTCTRL.bit.EVOSEL = DMAC_BTCTRL_EVOSEL_DISABLE_Val, // Don't use Event System
    .BTCTRL.bit.BLOCKACT = DMAC_BTCTRL_BLOCKACT_NOACT_Val, // Fulfill all Linked Descriptors without intervention 
    .BTCTRL.bit.BEATSIZE = DMAC_BTCTRL_BEATSIZE_BYTE_Val, // Always Single Byte
    .BTCTRL.bit.SRCINC = 1,
    .BTCTRL.bit.DSTINC = 0,   //SERCOM[X]->Data in all Cases
    .BTCTRL.bit.STEPSEL = DMAC_BTCTRL_STEPSEL_SRC_Val,
    .BTCTRL.bit.STEPSIZE = DMAC_BTCTRL_STEPSIZE_X1_Val, //Single Byte jumps in memory

    .DSTADDR.bit.DSTADDR = (uint32_t)(&SERCOM4->SPI.DATA.reg), //Data input of SERCOM4 Register
    
    .DESCADDR.bit.DESCADDR = (uint32_t)NULL //No DMA Linked Jobs this
};
static uint8_t s_sramAccessCommand[2] =  {AT86RF233_CMD_SRAM_WRITE, 0x00};

void samr21Trx_dmaUploadToFramebuffer(uint8_t *a_data_p, uint8_t a_len, uint8_t a_pos)
{
#ifdef _DEBUG
    assert(a_len <= IEEE_802_15_4_FRAME_SIZE);
    assert((a_pos + a_len) <= IEEE_802_15_4_FRAME_SIZE);
    assert(a_data_p);
#endif

    while (s_trxVars.spiActive)
        ;

    //Faster SPI Start Access (does ignore the inital status byte)
    NVIC_DisableIRQ(EIC_IRQn);
    NVIC_DisableIRQ(TC4_IRQn);
    PORT->Group[1].OUTCLR.reg = 1 << 31; // SSel Low Active
    s_trxVars.spiActive = true;

    //Adjust StartAddr of SRAM Write Command
    s_sramAccessCommand[1] = a_pos;

    //Adjust The Framebuffer DMA Job
    s_framebufferDmaDescriptor.BTCTRL.bit.VALID = 0;
    
    s_framebufferDmaDescriptor.BTCNT.bit.BTCNT = a_len;
    s_framebufferDmaDescriptor.SRCADDR.bit.SRCADDR = (uint32_t)a_data_p + a_len;

    s_framebufferDmaDescriptor.BTCTRL.bit.VALID = 1;
    
    //Start a DMA Job for the SRAM Acces Command itself and link the actual Framebuffer DMA after that
    samr21Dma_start(0,s_sramAccessCommand,sizeof(s_sramAccessCommand),&s_framebufferDmaDescriptor);
    //Jumpstart the DMA
    samr21Dma_triggerChannelAction(0);

    //Set a Periodic DMA-Trigger Timer
    samr21Timer3_setContinuousPeriod(IEEE_15_4_24GHZ_TIME_PER_OCTET_us - 5); //Some wiggle Room
}

static void trx_dmaDone_cb(void)
{
    //Invalidate the Framebuffer-DMA-Job
    s_framebufferDmaDescriptor.BTCTRL.bit.VALID = 0;

    //Stop the Periodic DMA-Trigger Timer
    samr21Timer3_Stop();

    //Orderly close the SPI-Access 
    samr21Trx_spiCloseAccess();
}

/**********TRX EVENT HANDLER********/

// External IRQs form TRX
static void (*s_currentIsrHandler_fktPtr[NUM_TRX_IRQS])(void) = {NULL};

void samr21Trx_setInterruptHandler(uint8_t a_irqType, void (*a_handler_fktPtr)(void))
{

#ifdef _DEBUG
    assert(a_irqType <= (NUM_TRX_IRQS - 1));
#endif

    s_currentIsrHandler_fktPtr[a_irqType] = a_handler_fktPtr;
}

void samr21Trx_removeAllInterruptHandler()
{
    for (uint8_t i = 0; i < NUM_TRX_IRQS; i++)
    {
        s_currentIsrHandler_fktPtr[i] = NULL;
    }
}

void samr21Trx_disableAllInterrupts()
{
    s_localTrxRegisterCopy.irqMask.reg = 0x00;

    samr21Trx_writeRegister(IRQ_MASK_REG_ADDR, s_localTrxRegisterCopy.irqMask.reg);
}

void samr21Trx_enableInterrupt(uint8_t a_irqType)
{

#ifdef _DEBUG
    assert(a_irqType <= (NUM_TRX_IRQS - 1));
#endif

    uint8_t shiftedIrqEnableBit = 1 << a_irqType;

    if (shiftedIrqEnableBit & s_localTrxRegisterCopy.irqMask.reg)
    {
        // Already enabled
        return;
    }

    s_localTrxRegisterCopy.irqMask.reg |= shiftedIrqEnableBit;

    samr21Trx_writeRegister(IRQ_MASK_REG_ADDR, s_localTrxRegisterCopy.irqMask.reg);
}

void samr21Trx_disableInterrupt(uint8_t a_irqType)
{

#ifdef _DEBUG
    assert(a_irqType <= (NUM_TRX_IRQS - 1));
#endif

    uint8_t shiftedIrqDisableBit = 1 << a_irqType;

    if (shiftedIrqDisableBit & ~s_localTrxRegisterCopy.irqMask.reg)
    {
        // Already disabled
        return;
    }

    s_localTrxRegisterCopy.irqMask.reg &= ~shiftedIrqDisableBit;

    samr21Trx_writeRegister(IRQ_MASK_REG_ADDR, s_localTrxRegisterCopy.irqMask.reg);
}

void EIC_Handler()
{
    // Clear IRQ
    EIC->INTFLAG.bit.EXTINT0 = 1;

    s_localTrxRegisterCopy.irqStatus.reg = samr21Trx_readRegister(IRQ_STATUS_REG_ADDR);

    if (s_localTrxRegisterCopy.irqStatus.bit.rxStart && (s_currentIsrHandler_fktPtr[TRX_IRQ_RX_START] != NULL))
    {
        (*s_currentIsrHandler_fktPtr[TRX_IRQ_RX_START])();
    }

    if (s_localTrxRegisterCopy.irqStatus.bit.trxEnd && (s_currentIsrHandler_fktPtr[TRX_IRQ_TRX_END] != NULL))
    {
        (*s_currentIsrHandler_fktPtr[TRX_IRQ_TRX_END])();
    }

    if (s_localTrxRegisterCopy.irqStatus.bit.pllLock && (s_currentIsrHandler_fktPtr[TRX_IRQ_PLL_LOCK] != NULL))
    {
        (*s_currentIsrHandler_fktPtr[TRX_IRQ_TRX_END])();
    }

    if (s_localTrxRegisterCopy.irqStatus.bit.pllUnlock && (s_currentIsrHandler_fktPtr[TRX_IRQ_PLL_UNLOCK] != NULL))
    {
        (*s_currentIsrHandler_fktPtr[TRX_IRQ_PLL_UNLOCK])();
    }

    if (s_localTrxRegisterCopy.irqStatus.bit.ccaEdDone && (s_currentIsrHandler_fktPtr[TRX_IRQ_CCA_ED_DONE] != NULL))
    {
        (*s_currentIsrHandler_fktPtr[TRX_IRQ_CCA_ED_DONE])();
    }

    if (s_localTrxRegisterCopy.irqStatus.bit.addressMatch && (s_currentIsrHandler_fktPtr[TRX_IRQ_AMI] != NULL))
    {
        (*s_currentIsrHandler_fktPtr[TRX_IRQ_AMI])();
    }

    if (s_localTrxRegisterCopy.irqStatus.bit.bufferUnderRun && (s_currentIsrHandler_fktPtr[TRX_IRQ_TRX_UR] != NULL))
    {
        (*s_currentIsrHandler_fktPtr[TRX_IRQ_TRX_UR])();
    }

    if (s_localTrxRegisterCopy.irqStatus.bit.batteryLow && (s_currentIsrHandler_fktPtr[TRX_IRQ_BAT_LOW] != NULL))
    {
        (*s_currentIsrHandler_fktPtr[TRX_IRQ_BAT_LOW])();
    }
}