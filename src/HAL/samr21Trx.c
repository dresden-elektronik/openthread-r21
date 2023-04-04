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

static void samr21DebugPortsInit()
{

    PORT->Group[0].DIRSET.reg = PORT_PA06;

    // Setup Mux Settings
    PORT->Group[0].WRCONFIG.reg =
        // PORT_WRCONFIG_HWSEL
        PORT_WRCONFIG_WRPINCFG
        //|PORT_WRCONFIG_WRPMUX
        //|PORT_WRCONFIG_PMUX(MUX_PC16F_GCLK_IO1)
        // PORT_WRCONFIG_PULLEN
        //|PORT_WRCONFIG_INEN
        //|PORT_WRCONFIG_PMUXEN
        | PORT_WRCONFIG_PINMASK(PORT_PA06) // lower Halfword
        ;

    PORT->Group[0].DIRSET.reg = PORT_PA07;

    // Setup Mux Settings
    PORT->Group[0].WRCONFIG.reg =
        // PORT_WRCONFIG_HWSEL
        PORT_WRCONFIG_WRPINCFG
        //|PORT_WRCONFIG_WRPMUX
        //|PORT_WRCONFIG_PMUX(MUX_PC16F_GCLK_IO1)
        // PORT_WRCONFIG_PULLEN
        //|PORT_WRCONFIG_INEN
        //|PORT_WRCONFIG_PMUXEN
        | PORT_WRCONFIG_PINMASK(PORT_PA07) // lower Halfword
        ;

    PORT->Group[0].DIRSET.reg = PORT_PA08;

    // Setup Mux Settings
    PORT->Group[0].WRCONFIG.reg =
        // PORT_WRCONFIG_HWSEL
        PORT_WRCONFIG_WRPINCFG
        //|PORT_WRCONFIG_WRPMUX
        //|PORT_WRCONFIG_PMUX(MUX_PC16F_GCLK_IO1)
        // PORT_WRCONFIG_PULLEN
        //|PORT_WRCONFIG_INEN
        //|PORT_WRCONFIG_PMUXEN
        | PORT_WRCONFIG_PINMASK(PORT_PA08) // lower Halfword
        ;

    PORT->Group[0].DIRSET.reg = PORT_PA09;

    // Setup Mux Settings
    PORT->Group[0].WRCONFIG.reg =
        // PORT_WRCONFIG_HWSEL
        PORT_WRCONFIG_WRPINCFG
        //|PORT_WRCONFIG_WRPMUX
        //|PORT_WRCONFIG_PMUX(MUX_PC16F_GCLK_IO1)
        // PORT_WRCONFIG_PULLEN
        //|PORT_WRCONFIG_INEN
        //|PORT_WRCONFIG_PMUXEN
        | PORT_WRCONFIG_PINMASK(PORT_PA09) // lower Halfword
        ;

    PORT->Group[0].DIRSET.reg = PORT_PA09;

    // Setup Mux Settings
    PORT->Group[0].WRCONFIG.reg =
        // PORT_WRCONFIG_HWSEL
        PORT_WRCONFIG_WRPINCFG
        //|PORT_WRCONFIG_WRPMUX
        //|PORT_WRCONFIG_PMUX(MUX_PC16F_GCLK_IO1)
        // PORT_WRCONFIG_PULLEN
        //|PORT_WRCONFIG_INEN
        //|PORT_WRCONFIG_PMUXEN
        | PORT_WRCONFIG_PINMASK(PORT_PA09) // lower Halfword
        ;

    PORT->Group[0].DIRSET.reg = PORT_PA16;

    // Setup Mux Settings
    PORT->Group[0].WRCONFIG.reg =
        PORT_WRCONFIG_HWSEL
        |PORT_WRCONFIG_WRPINCFG
        //| PORT_WRCONFIG_WRPMUX 
        //| PORT_WRCONFIG_PMUX(MUX_PC16F_GCLK_IO1)
        //| PORT_WRCONFIG_PULLEN
        //| PORT_WRCONFIG_INEN
        //| PORT_WRCONFIG_PMUXEN 
        | PORT_WRCONFIG_PINMASK(PORT_PA16) // lower Halfword
        ;
}

static ramCopyTrxRegister_t s_ramCopyTrxRegister = {

    // Default Config
    .trxCtrl1.bit.irqPolarity = 0,
    .trxCtrl1.bit.irqMaskMode = 0,
    .trxCtrl1.bit.spiCmdMode = 0x1,
    .trxCtrl1.bit.rxBlCtrl = 1,
    .trxCtrl1.bit.txAutoCrcOn = 1,
    .trxCtrl1.bit.irq2ExtEn = 0,

#ifdef _GCF_RELEASE_
    .trxCtrl1.bit.paExtnEn = 1,
    .phyTrxPwr.bit.txPwr = 0x7
#else
    .trxCtrl1.bit.paExtnEn = 0,
    .phyTxPwr.bit.txPwr = 0x0
#endif
};

void samr21TrxInterfaceInit()
{
    // Setup Clocks for TRX-SPI
    // Use GCLKGEN1 as core Clock for SPI At86rf233 (SERCOM4, Synchronous)
    GCLK->CLKCTRL.reg =
        // GCLK_CLKCTRL_WRTLOCK
        GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN(1) // GCLKGEN1 (Sourced from At86rf233 mClk)
        | GCLK_CLKCTRL_ID(GCLK_CLKCTRL_ID_SERCOM4_CORE_Val);

    // Wait for synchronization
    while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY)
        ;

    GCLK->CLKCTRL.reg =
        // GCLK_CLKCTRL_WRTLOCK
        GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN(0) // GCLKGEN1
        | GCLK_CLKCTRL_ID(GCLK_CLKCTRL_ID_SERCOMX_SLOW_Val);

    // Wait for synchronization
    while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY)
        ;

    // Enable in Power manager
    PM->APBCMASK.bit.SERCOM4_ = 1;

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

    // Reset SERCOM4
    SERCOM4->SPI.CTRLA.bit.SWRST = 1;

    // Wait for SERCOM4 reset to finish
    while (SERCOM4->SPI.CTRLA.bit.SWRST || SERCOM4->SPI.SYNCBUSY.bit.SWRST)
        ;

    // Setup SERCOM4

    // F_ref = 16MHz (if At86r233 ist setup correctly) F_baud = F_ref / 2*(BAUD+1) ---> BAUD = 0 F_baud = 8Mhz
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
        SERCOM_SPI_CTRLA_RUNSTDBY | SERCOM_SPI_CTRLA_MODE(SERCOM_SPI_CTRLA_MODE_SPI_MASTER_Val) | SERCOM_SPI_CTRLA_ENABLE
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

    // Write predefined Operating-Values to Config Registers of AT86rf233
    samr21TrxWriteRegister(TRX_CTRL_1_REG_ADDR, s_ramCopyTrxRegister.trxCtrl1.reg);
    samr21TrxWriteRegister(PHY_TX_PWR_REG_ADDR, s_ramCopyTrxRegister.phyTxPwr.reg);

    // Enable Timer for Trx
    samr21Timer5Init(0); // 1MHz / (2^0) -> 1us resolution

    samr21DebugPortsInit();
}

void samr21TrxInterruptInit()
{

    // Get TRX into IDLE (STATUS_TRX_OFF) State
    samr21TrxForceMoveToIdle(true);

    // Enable EIC in Power Manager
    PM->APBAMASK.bit.EIC_ = 1;

    // Enable IRQ via EIC
    // Use GCLKGEN0 as core Clock for EIC (At86rf233, IRQ_Detect)
    GCLK->CLKCTRL.reg =
        // GCLK_CLKCTRL_WRTLOCK
        GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN(0) // GCLKGEN1
        | GCLK_CLKCTRL_ID(GCLK_CLKCTRL_ID_EIC_Val);
    // Wait for synchronization
    while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY)
        ;
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
    samr21TrxReadRegister(IRQ_STATUS_REG_ADDR);

    // cLEAR irq BEFORE ENABLING
    EIC->INTFLAG.bit.EXTINT0 = 1;
}

void samr21TrxInterruptDeinit()
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

        // Disable GCLKGEN0 for EIC (At86rf233, IRQ_Detect)
        GCLK->CLKCTRL.reg =
            // GCLK_CLKCTRL_WRTLOCK
            GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN(0) // GCLKGEN1
            | GCLK_CLKCTRL_ID(GCLK_CLKCTRL_ID_EIC_Val);
        // Wait for synchronization
        while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY)
            ;
    }

    // Disable EIC in Power Manager
    if (PM->APBAMASK.bit.EIC_)
    {
        PM->APBAMASK.bit.EIC_ = 0;
    }
}

void samr21TrxSetupMClk(uint8_t a_clk)
{

    // Write desired Value into the CLKM Register (0x05 -> 16Mhz)
    s_ramCopyTrxRegister.trxCtrl0.bit.clkmCtrl = a_clk;

    samr21TrxWriteRegister(TRX_CTRL_0_REG_ADDR, s_ramCopyTrxRegister.trxCtrl0.reg);

    // Wait a bit for the  the Freq.-change to take place
    samr21delaySysTick(50);
}

uint8_t samr21TrxReadRegister(uint8_t a_addr)
{

    samr21TrxSpiStartAccess(AT86RF233_CMD_REG_READ_MASK, a_addr);

#ifdef __CONSERVATIVE_TRX_SPI_TIMING__
    samr21delaySysTick(CPU_WAIT_CYCLES_BETWEEN_BYTES);
#endif

    uint8_t rVal = samr21TrxSpiReadByteRaw();

    samr21TrxSpiCloseAccess();

    return rVal;
}

void samr21TrxWriteRegister(uint8_t a_addr, uint8_t a_data)
{

    samr21TrxSpiStartAccess(AT86RF233_CMD_REG_WRITE_MASK, a_addr);

#ifdef __CONSERVATIVE_TRX_SPI_TIMING__
    samr21delaySysTick(CPU_WAIT_CYCLES_BETWEEN_BYTES);
#endif

    samr21TrxSpiTransceiveByteRaw(a_data);

    samr21TrxSpiCloseAccess();
}

static volatile bool s_spiActive;
void samr21TrxSpiStartAccess(uint8_t a_command, uint8_t a_addr)
{
    // Wait for other ongoing SPI Access to End
    while (s_spiActive)
        ;

    __disable_irq();
    // NVIC_DisableIRQ(EIC_IRQn);
    // NVIC_DisableIRQ(TC4_IRQn);

    s_spiActive = true;
    PORT->Group[1].OUTCLR.reg = 1 << 31; // SSel Low Active

#ifdef __CONSERVATIVE_TRX_SPI_TIMING__
    samr21delaySysTick(CPU_WAIT_CYCLES_AFTER_SSEL_LOW);
#endif

    // Register Read/Write
    if (a_command & AT86RF233_CMD_REG_READ_MASK)
    {
        s_ramCopyTrxRegister.trxStatus.reg = samr21TrxSpiTransceiveByteRaw((a_addr & 0x3F) | a_command);
        return;
    }

    // Framebuffer Access
    if (a_command & AT86RF233_CMD_FRAMEBUFFER_READ)
    {
        s_ramCopyTrxRegister.trxStatus.reg = samr21TrxSpiTransceiveByteRaw(a_command);
        return;
    }

    // SRAM Access
    s_ramCopyTrxRegister.trxStatus.reg = samr21TrxSpiTransceiveByteRaw(a_command);

#ifdef __CONSERVATIVE_TRX_SPI_TIMING__
    samr21delaySysTick(CPU_WAIT_CYCLES_BETWEEN_BYTES);
#endif
    samr21TrxSpiTransceiveByteRaw(a_addr);
}

void samr21TrxSpiCloseAccess()
{
    PORT->Group[1].OUTSET.reg = 1 << 31; // SSel Low Active

#ifdef __CONSERVATIVE_TRX_SPI_TIMING__
    samr21delaySysTick(CPU_WAIT_CYCLES_AFTER_SSEL_LOW);
#endif
    s_spiActive = false;

    // NVIC_EnableIRQ(EIC_IRQn);
    // NVIC_EnableIRQ(TC4_IRQn);
    __enable_irq();
}

void samr21TrxUpdateStatusRegister()
{

    while (s_spiActive)
        ;
    __disable_irq();
    PORT->Group[1].OUTCLR.reg = 1 << 31; // SSel Low Active
    s_spiActive = true;

    s_ramCopyTrxRegister.trxStatus.reg = samr21TrxSpiTransceiveByteRaw(SPI_DUMMY_BYTE);

    samr21TrxSpiCloseAccess();
}

void samr21TrxSetRSTN(bool a_enable)
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

void samr21TrxSetSLP_TR(bool a_enable)
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

uint8_t samr21TrxSpiTransceiveByteRaw(uint8_t a_data)
{
    while (!SERCOM4->SPI.INTFLAG.bit.DRE)
        ;
    // Put data into the tranmitt buffer to start transmission
    // PORT->Group[0].OUTTGL.reg= PORT_PA07;
    SERCOM4->SPI.DATA.bit.DATA = a_data;

    while (!SERCOM4->SPI.INTFLAG.bit.TXC)
        ;
    while (!SERCOM4->SPI.INTFLAG.bit.RXC)
        ;
    // PORT->Group[0].OUTTGL.reg= PORT_PA07;

    // return recived Answerto
    return SERCOM4->SPI.DATA.bit.DATA;
}

void samr21TrxSpiSendByteRawIgnoreResponse(uint8_t a_data)
{
    // Put data into the tranmitt buffer to start transmission
    SERCOM4->SPI.DATA.bit.DATA = a_data;

    // Wait for Transmit to Complete
    while (!SERCOM4->SPI.INTFLAG.bit.TXC)
        ;
}

void samr21TrxSpiTransceiveBytesRaw(uint8_t *a_inData_1D, uint8_t *a_outData_1D, uint8_t a_len)
{
    for (uint8_t i = 0; i < a_len; i++)
    {

#ifdef __CONSERVATIVE_TRX_SPI_TIMING__
        samr21delaySysTick(SAMR21_NUM_CPU_WAIT_CYCLES_BETWEEN_SPI_BYTES);
#endif

        if (!a_outData_1D)
        {
            samr21TrxSpiSendByteRawIgnoreResponse(a_inData_1D[i]);
            continue;
        }

        a_outData_1D[i] = samr21TrxSpiTransceiveByteRaw(a_inData_1D != NULL ? a_inData_1D[i] : SPI_DUMMY_BYTE);
    }
}

uint8_t samr21TrxSpiReadByteRaw()
{
    return samr21TrxSpiTransceiveByteRaw(SPI_DUMMY_BYTE);
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

static void samr21TrxChangeStateBlocking(uint8_t a_cmd, uint8_t *a_desiredStates, uint8_t a_numDesiredStates)
{
    samr21TrxWriteRegister(TRX_STATE_REG_ADDR, a_cmd);

    while (true)
    {
        uint8_t currentState = samr21TrxReadRegister(TRX_STATUS_REG_ADDR);

        for (uint8_t i = 0; i < a_numDesiredStates; i++)
        {
            if ((currentState & 0b00011111) == a_desiredStates[i])
            {
                return;
            }
        }
    }
}

void samr21TrxQueueMoveToRx(bool a_blocking)
{
    if (!a_blocking)
    {
        samr21TrxWriteRegister(TRX_STATE_REG_ADDR, TRX_CMD_RX_ON);
        return;
    }

    samr21TrxChangeStateBlocking(TRX_CMD_RX_ON, sc_validRxTrxStates, sizeof(sc_validRxTrxStates));
}

void samr21TrxQueueMoveToTx(bool a_blocking)
{
    if (!a_blocking)
    {
        samr21TrxWriteRegister(TRX_STATE_REG_ADDR, TRX_CMD_PLL_ON);
        return;
    }

    samr21TrxChangeStateBlocking(TRX_CMD_PLL_ON, sc_validTxTrxStates, sizeof(sc_validTxTrxStates));
}

void samr21TrxForceMoveToTx(bool a_blocking)
{
    if (!a_blocking)
    {
        samr21TrxWriteRegister(TRX_STATE_REG_ADDR, TRX_CMD_FORCE_PLL_ON);
        return;
    }

    samr21TrxChangeStateBlocking(TRX_CMD_FORCE_PLL_ON, sc_validTxTrxStates, sizeof(sc_validTxTrxStates));
}

void samr21TrxQueueMoveToIdle(bool a_blocking)
{
    if (!a_blocking)
    {
        samr21TrxWriteRegister(TRX_STATE_REG_ADDR, TRX_CMD_TRX_OFF);
        return;
    }

    samr21TrxChangeStateBlocking(TRX_CMD_TRX_OFF, sc_validIdleTrxStates, sizeof(sc_validIdleTrxStates));
}

void samr21TrxForceMoveToIdle(bool a_blocking)
{
    if (!a_blocking)
    {
        samr21TrxWriteRegister(TRX_STATE_REG_ADDR, TRX_CMD_FORCE_TRX_OFF);
        return;
    }

    samr21TrxChangeStateBlocking(TRX_CMD_FORCE_TRX_OFF, sc_validIdleTrxStates, sizeof(sc_validIdleTrxStates));
}

bool samr21TrxGetCcaResult()
{
    return s_ramCopyTrxRegister.trxStatus.bit.ccaStatus;
}

/*************************Channel Control*************************************/
void samr21TrxSetChannel(uint8_t a_channel)
{
#ifdef _DEBUG
    assert(a_channel >= 0x0B && a_channel <= 0x1A);
#endif
    if (a_channel == s_ramCopyTrxRegister.phyCcCca.bit.channel)
    {
        return;
    }

    s_ramCopyTrxRegister.phyCcCca.bit.channel = a_channel;

    samr21TrxWriteRegister(PHY_CC_CCA_REG_ADDR, s_ramCopyTrxRegister.phyCcCca.reg);
}

uint8_t samr21TrxGetChannel()
{
    return s_ramCopyTrxRegister.phyCcCca.bit.channel;
}

void samr21TrxStartCca()
{
    // Prepare CCA
    s_ramCopyTrxRegister.phyCcCca.bit.ccaRequest = 1;

    // Start CCA
    samr21TrxWriteRegister(PHY_CC_CCA_REG_ADDR, s_ramCopyTrxRegister.phyCcCca.reg);

    // Reset local copy of ccaRequest Bit
    s_ramCopyTrxRegister.phyCcCca.bit.ccaRequest = 0;
}

void samr21TrxStartEd()
{
    // Start ED by writing any value to PHY_ED_LEVEL_REG_ADDR
    // See samr21 datasheet: 37.5 Energy Detection (ED)
    samr21TrxWriteRegister(PHY_ED_LEVEL_REG_ADDR, 0xE7);
}

int8_t samr21TrxGetLastRssiValue()
{

    if (
        s_ramCopyTrxRegister.trxStatus.bit.trxStatus != TRX_STATUS_RX_ON && s_ramCopyTrxRegister.trxStatus.bit.trxStatus != TRX_STATUS_RX_ON)
    {
        return INT8_MAX;
    }

    return AT86RF233_RSSI_BASE_VAL_dBm + (int8_t)samr21TrxReadRegister(PHY_ED_LEVEL_REG_ADDR);
}

/*************************CCA Control*************************************/

void samr21RadioCtrlSetCCAMode(uint8_t a_newCcaMode)
{
    s_ramCopyTrxRegister.phyCcCca.bit.ccaMode = a_newCcaMode;
    samr21TrxWriteRegister(PHY_CC_CCA_REG_ADDR, s_ramCopyTrxRegister.phyCcCca.reg);
}

uint8_t samr21RadioCtrlGetCCAMode()
{
    return s_ramCopyTrxRegister.phyCcCca.bit.ccaMode;
}

void samr21RadioCtrlSetCcaThreshold(int8_t a_threshold)
{
    int8_t diff = (AT86RF233_RSSI_BASE_VAL_dBm - a_threshold);
    s_ramCopyTrxRegister.ccaThres.bit.ccaEdThres = abs(diff) >> 1; // Devide by 2

    samr21TrxWriteRegister(CCA_THRES_REG_ADDR, s_ramCopyTrxRegister.ccaThres.reg);
}

int8_t samr21RadioCtrlGetCcaThreshold()
{
    return (AT86RF233_RSSI_BASE_VAL_dBm + (s_ramCopyTrxRegister.ccaThres.bit.ccaEdThres << 1)); // Multiply by 2
}

/*************************Tx Power Control*************************************/
// TX Power
#define SIZE_AT86RF233_TX_POWER_TABLE 16
#ifdef _GCF_RELEASE_
static const float sc_txPowerTable_dBm_1D[SIZE_AT86RF233_TX_POWER_TABLE] = {11.9, 11.8, 11.7, 11.6, 11.5, 11.4, 11.3, 11.2, 11, 10.7, 10.4, 10.1, 9.3, 8.2, 6.4, 3.9, 0.1, -5.8};
#else
static const float sc_txPowerTable_dBm_1D[SIZE_AT86RF233_TX_POWER_TABLE] = {4, 3.7, 3.4, 3, 2.5, 2, 1, 0, -1, -2, -3, -4, -6, -8, -12, -17};
#endif

void samr21TrxSetTxPower(int8_t a_power_dbm)
{
    for (uint8_t i = 0; i < SIZE_AT86RF233_TX_POWER_TABLE; i++)
    {
        if (
            (sc_txPowerTable_dBm_1D[i] >= a_power_dbm) && (sc_txPowerTable_dBm_1D[i + 1] <= a_power_dbm))
        {
            s_ramCopyTrxRegister.phyTxPwr.bit.txPwr = i;
            goto exit;
        }
    }

    // Default
    s_ramCopyTrxRegister.phyTxPwr.bit.txPwr = 0x7;

exit:
    samr21TrxWriteRegister(PHY_TX_PWR_REG_ADDR, s_ramCopyTrxRegister.phyTxPwr.reg);
}

int8_t samr21TrxGetTxPower()
{
    return sc_txPowerTable_dBm_1D[s_ramCopyTrxRegister.phyTxPwr.bit.txPwr];
}

/***************Timer-Handler for Trx-Operational Timing***************************/
static volatile bool *s_timeoutFlag = NULL;
static volatile void (*s_currentTimerHandler_fktPtr)(void) = NULL;

static void samr21TrxTimeoutTriggered()
{
    *s_timeoutFlag = true;
    s_currentTimerHandler_fktPtr = NULL;
}

static void samr21TrxStartTimeoutTimer(uint16_t a_duration_us, bool *a_triggerFlag)
{
    s_timeoutFlag = a_triggerFlag;
    s_currentTimerHandler_fktPtr = &samr21TrxTimeoutTriggered;
    samr21Timer5Set(a_duration_us);
}

static void samr21TrxStopTimeoutTimer()
{
    samr21Timer5Stop();
    s_timeoutFlag = NULL;
    s_currentTimerHandler_fktPtr = NULL;
}

static void samr21TrxQueueDelayedAction(uint16_t a_delay_us, void (*a_queuedAction_fktPtr)(void))
{
    s_currentTimerHandler_fktPtr = a_queuedAction_fktPtr;
    samr21Timer5Set(a_delay_us);
}

static void samr21TrxRemoveQueuedAction()
{
    s_currentTimerHandler_fktPtr = NULL;
    samr21Timer5Stop();
}

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

/***************True Random Engine***************************/
uint8_t samr21TrxGetRandomCrumb()
{
    return (samr21TrxReadRegister(PHY_RSSI_REG_ADDR) >> 5) & 0b00000011;
}
uint8_t samr21TrxGetRandomNibble()
{
    return ((samr21TrxReadRegister(PHY_RSSI_REG_ADDR) >> 5) & 0b00000011) | ((samr21TrxReadRegister(PHY_RSSI_REG_ADDR) >> 3) & 0b00001100);
}
uint8_t samr21TrxGetRandomByte()
{
    uint8_t rVal = (samr21TrxReadRegister(PHY_RSSI_REG_ADDR) & 0x60) >> 5;
    rVal |= (samr21TrxReadRegister(PHY_RSSI_REG_ADDR) & 0x60) >> 3;
    rVal |= (samr21TrxReadRegister(PHY_RSSI_REG_ADDR) & 0x60) >> 1;
    rVal |= (samr21TrxReadRegister(PHY_RSSI_REG_ADDR) & 0x60) << 1;
    return rVal;
}

/*********************AES Engine*******************************/
static s_aesBusy = false;

void samr21TrxAesKeySetup(uint8_t *a_key_1D)
{
    while (s_aesBusy)
        ;

    PORT->Group[0].OUTSET.reg = PORT_PA07;
    samr21TrxSpiStartAccess(AT86RF233_CMD_SRAM_WRITE, AES_CTRL_SRAM_ADDR);

    ramCopyTrxAesCtrl_t temp_aesCtrl = {
        .bit.dir = 0,
        .bit.mode = AT86RF233_AES_MODE_KEY,
        .bit.request = 0};
    samr21TrxSpiTransceiveByteRaw(temp_aesCtrl.reg);
    samr21TrxSpiTransceiveBytesRaw(a_key_1D, NULL, IEEE_15_4_AES_CCM_BLOCK_SIZE);
    samr21TrxSpiCloseAccess();
    PORT->Group[0].OUTCLR.reg = PORT_PA07;
}

static void samr21TrxAesEcbEncrypt(uint8_t *inDataBlock_1D, uint8_t *outDataBlock_1D)
{
    PORT->Group[0].OUTSET.reg = PORT_PA07;
    samr21TrxSpiStartAccess(AT86RF233_CMD_SRAM_WRITE, AES_CTRL_SRAM_ADDR);

    ramCopyTrxAesCtrl_t aesCtrl = {
        .bit.dir = 0,
        .bit.mode = AT86RF233_AES_MODE_ECB,
        .bit.request = 0};

#ifdef __CONSERVATIVE_TRX_SPI_TIMING__
    samr21delaySysTick(SAMR21_NUM_CPU_WAIT_CYCLES_BETWEEN_SPI_BYTES_FAST_ACCESS);
#endif
    samr21TrxSpiTransceiveByteRaw(aesCtrl.reg);

    for (uint8_t i = 0; i < IEEE_15_4_AES_CCM_BLOCK_SIZE; i++)
    {
        // Send New Plaintext and Retrive last Ciphertext
#ifdef __CONSERVATIVE_TRX_SPI_TIMING__
        samr21delaySysTick(SAMR21_NUM_CPU_WAIT_CYCLES_BETWEEN_SPI_BYTES_FAST_ACCESS);
#endif
        if (outDataBlock_1D)
        {
            if (i != 0)
            {
                outDataBlock_1D[i - 1] = samr21TrxSpiTransceiveByteRaw(inDataBlock_1D == NULL ? 0x00 : inDataBlock_1D[i]);
            }
            else
            {
                samr21TrxSpiTransceiveByteRaw(inDataBlock_1D == NULL ? 0x00 : inDataBlock_1D[i]);
            }
        }
        else
        {
            samr21TrxSpiTransceiveByteRaw(inDataBlock_1D == NULL ? 0x00 : inDataBlock_1D[i]);
        }
    }

    aesCtrl.bit.request = 1;

    if (outDataBlock_1D)
    {
        outDataBlock_1D[IEEE_15_4_AES_CCM_BLOCK_SIZE - 1] = samr21TrxSpiTransceiveByteRaw(aesCtrl.reg);
    }
    else
    {
        samr21TrxSpiTransceiveByteRaw(aesCtrl.reg);
    }

    samr21TrxSpiCloseAccess();
    PORT->Group[0].OUTCLR.reg = PORT_PA07;
}

static void samr21TrxAesCbcEncrypt(uint8_t *inDataBlock_1D, uint8_t *outDataBlock_1D)
{
    PORT->Group[0].OUTSET.reg = PORT_PA07;
    samr21TrxSpiStartAccess(AT86RF233_CMD_SRAM_WRITE, AES_CTRL_SRAM_ADDR);

    // Send ECB encription Command
    ramCopyTrxAesCtrl_t aesCtrl = {
        .bit.dir = 0,
        .bit.mode = AT86RF233_AES_MODE_CBC,
        .bit.request = 0};

#ifdef __CONSERVATIVE_TRX_SPI_TIMING__
    samr21delaySysTick(CPU_WAIT_CYCLES_BETWEEN_BYTES);
#endif
    samr21TrxSpiTransceiveByteRaw(aesCtrl.reg);

    for (uint8_t i = 0; i < IEEE_15_4_AES_CCM_BLOCK_SIZE; i++)
    {
        // Send New Plaintext and Retrive last Ciphertext
#ifdef __CONSERVATIVE_TRX_SPI_TIMING__
        samr21delaySysTick(CPU_WAIT_CYCLES_BETWEEN_BYTES_FAST_ACCESS);
#endif
        if (outDataBlock_1D)
        {
            if (i != 0)
            {
                outDataBlock_1D[i - 1] = samr21TrxSpiTransceiveByteRaw(inDataBlock_1D == NULL ? 0x00 : inDataBlock_1D[i]);
            }
            else
            {
                samr21TrxSpiTransceiveByteRaw(inDataBlock_1D == NULL ? 0x00 : inDataBlock_1D[i]);
            }
        }
        else
        {
            samr21TrxSpiSendByteRawIgnoreResponse(inDataBlock_1D == NULL ? 0x00 : inDataBlock_1D[i]);
        }
    }

    aesCtrl.bit.request = 1;

#ifdef __CONSERVATIVE_TRX_SPI_TIMING__
    samr21delaySysTick(CPU_WAIT_CYCLES_BETWEEN_BYTES);
#endif
    if (outDataBlock_1D)
    {
        outDataBlock_1D[IEEE_15_4_AES_CCM_BLOCK_SIZE - 1] = samr21TrxSpiTransceiveByteRaw(aesCtrl.reg);
    }
    else
    {
        samr21TrxSpiTransceiveByteRaw(aesCtrl.reg);
    }

    samr21TrxSpiCloseAccess();
    PORT->Group[0].OUTCLR.reg = PORT_PA07;
}

/****************FRAMEBUFFER READ ACCESS************************/
bool samr21TrxLiveFramebufferDownload(uint8_t *a_data_1D, uint8_t a_len)
{

#ifdef _DEBUG
    assert(a_len <= IEEE_802_15_4_FRAME_SIZE);
    assert(a_data_1D);
#endif

    uint8_t numDownloadedBytes = 0;

    // Set a Timeout
    volatile bool abortActionFlag = false;
    samr21TrxStartTimeoutTimer((((uint16_t)(a_len)*2) * IEEE_15_4_24GHZ_TIME_PER_OCTET_us), &abortActionFlag); // Shift for cheap multiply x2

    samr21TrxSpiStartAccess(AT86RF233_CMD_FRAMEBUFFER_READ, NULL);

    a_data_1D[numDownloadedBytes++] = samr21TrxSpiReadByteRaw();

    while (numDownloadedBytes < a_len)
    {
        //(see r21 datasheet, 40.7 Frame Buffer Empty Indicator)
        samr21delaySysTick(SAMR21_NUM_CPU_WAIT_CYCLES_FOR_FRAME_BUFFER_EMPTY_FLAG);
        while ((PORT->Group[1].IN.reg & PORT_PB00) && !abortActionFlag)
            ;

        // Abort if a Timeout happened
        if (abortActionFlag)
        {
            goto exit;
        }

        a_data_1D[numDownloadedBytes++] = samr21TrxSpiReadByteRaw();
    }
    samr21TrxStopTimeoutTimer();

exit:
    samr21TrxSpiCloseAccess();
    return !abortActionFlag;
}

bool samr21TrxDownloadFramebuffer(uint8_t *a_psduLen, uint8_t *a_psdu_1D, uint8_t *a_LQI_p, int8_t *a_RSSI_p)
{

#ifdef _DEBUG
    assert(a_psduLen);
    assert(a_psdu_1D);
    assert(a_LQI);
    assert(a_RSSI);
#endif

    uint8_t numDownloadedBytes = 0;
    bool validFrame = true;

    samr21TrxSpiStartAccess(AT86RF233_CMD_FRAMEBUFFER_READ, NULL);

    // First Byte Downloaded is the psduLen
    *a_psduLen = samr21TrxSpiReadByteRaw();

    // Check if the Framelenght is within limits outlined by 802.15.4
    if (*a_psduLen > IEEE_15_4_PDSU_SIZE)
    {
        validFrame = false;
        goto exit;
    }

    // Set a Timeout
    volatile bool abortActionFlag = false;
    samr21TrxStartTimeoutTimer((IEEE_15_4_FRAME_SIZE * IEEE_15_4_24GHZ_TIME_PER_OCTET_us), &abortActionFlag); // Shift for cheap multiply x2

    while (numDownloadedBytes < *a_psduLen)
    {
        //(see r21 datasheet, 40.7 Frame Buffer Empty Indicator)
        samr21delaySysTick(SAMR21_NUM_CPU_WAIT_CYCLES_FOR_FRAME_BUFFER_EMPTY_FLAG);
        while ((PORT->Group[1].IN.reg & PORT_PB00) && !abortActionFlag)
            ;

        if (abortActionFlag)
        {
            validFrame = false;
            goto exit;
        }

        a_psdu_1D[(numDownloadedBytes++)] = samr21TrxSpiReadByteRaw();
    }
    samr21TrxStopTimeoutTimer();

    // Download LQI, RSSI and CRC Check (r21 Datasheet 35.3.2 -  Frame Buffer Access Mode)
    *a_LQI_p = samr21TrxSpiReadByteRaw();
    *a_RSSI_p = AT86RF233_RSSI_BASE_VAL_dBm + samr21TrxSpiReadByteRaw();

    // Check if CRC is correct (samr21 Datasheet 35.3.2 Frame Buffer Access Mode - Structure of RX_STATUS)
    validFrame &= samr21TrxSpiReadByteRaw() >> 7;

exit:
    samr21TrxSpiCloseAccess();
    return validFrame;
}

/****************FRAMEBUFFER WRITE ACCESS************************/
void samr21TrxUploadToFramebuffer(uint8_t *a_data_1D, uint8_t a_len, uint8_t a_pos)
{

#ifdef _DEBUG
    assert(a_len <= IEEE_802_15_4_FRAME_SIZE);
    assert((a_pos + a_len) <= IEEE_802_15_4_FRAME_SIZE);
    assert(a_data_1D);
#endif
    PORT->Group[0].OUTSET.reg = PORT_PA06;
    samr21TrxSpiStartAccess(AT86RF233_CMD_SRAM_WRITE, a_pos);
    samr21TrxSpiTransceiveBytesRaw(a_data_1D, NULL, a_len); // phyLen not in psduLen
    samr21TrxSpiCloseAccess();
    PORT->Group[0].OUTCLR.reg = PORT_PA06;
}

/*********FRAMEBUFFER WRITE ACCESS (with Security)**************/
static struct trxSecurity_s
{
    uint8_t securityLevel;

    uint8_t numBytesUploaded;

    uint8_t numEncryptionBlocksDone;
    uint8_t numAuthenticationPayloadBlocksDone;

    union
    {
        uint8_t numAuthenticationPayloadBlocksNeeded;
        uint8_t numEncryptionBlocksNeeded;
    };

    uint8_t numAuthenticationHeaderBlocksDone;
    uint8_t numAuthenticationHeaderBlocksNeeded;

    uint8_t currentSizeCbcBlock;

    uint8_t *header_p;
    uint8_t headerLen;
    uint8_t *payload_p;
    uint8_t payloadLen;
    uint8_t *footer_p;
    uint8_t micSize; // Footer also includes the FCS but this is automatically calculated by TRX (samr21 datasheet 37.3.3 Automatic FCS Generation)

    union
    {
        uint8_t aesCbcBlock_1D[IEEE_15_4_AES_CCM_BLOCK_SIZE];
        uint8_t aesCtrBlock_1D[IEEE_15_4_AES_CCM_BLOCK_SIZE];
        uint8_t micBlock_1D[IEEE_15_4_AES_CCM_BLOCK_SIZE];
    };

    uint8_t encryptionMask_1D[IEEE_15_4_AES_CCM_BLOCK_SIZE];

} s_trxSecurity;

static void samr21TrxUploadEncryptedToFramebuffer(uint8_t *a_data_1D, uint8_t *a_xorMask_1D, uint8_t a_len, uint8_t a_pos)
{

#ifdef _DEBUG
    assert(a_len <= IEEE_15_4_AES_CCM_BLOCK_SIZE);
    assert((a_pos + a_len) <= IEEE_15_4_FRAME_SIZE);
    assert(a_data_1D);
    assert(a_xorMask_1D);
#endif

    PORT->Group[0].OUTSET.reg = PORT_PA06;
    samr21TrxSpiStartAccess(AT86RF233_CMD_SRAM_WRITE, a_pos);

    for (uint8_t i = 0; i < a_len; i++)
    {
        samr21TrxSpiTransceiveByteRaw(a_data_1D[i] ^ a_xorMask_1D[i]);
    }

    samr21TrxSpiCloseAccess();
    PORT->Group[0].OUTCLR.reg = PORT_PA06;
}

static void samr21TrxAuthenticateFinish()
{
    // Start a DummyRound to retrieve the result of the last Round
    // TODO remove Dummy round!
    samr21TrxAesCbcEncrypt(NULL, s_trxSecurity.micBlock_1D);

    // Last AES Action took place, the engine can be unlocked again
    s_aesBusy = false;

    if (s_trxSecurity.micSize)
    {
        samr21TrxUploadEncryptedToFramebuffer(
            s_trxSecurity.micBlock_1D,
            s_trxSecurity.encryptionMask_1D,
            s_trxSecurity.micSize,
            s_trxSecurity.numBytesUploaded);
        s_trxSecurity.numBytesUploaded += s_trxSecurity.micSize;
    }

    // FCS is done by TRX (samr21 datasheet 37.3.3 Automatic FCS Generation)

#ifdef _DEBUG
    assert(s_trxSecurity.numBytesUploaded == (s_trxSecurity.headerLen + s_trxSecurity.payloadLen + s_trxSecurity.micSize + 1)) // psduLen not part of headerLength
#endif
}

static void samr21TrxAuthenticateLoop()
{

    // Start the AES Engine
    if (s_trxSecurity.numAuthenticationHeaderBlocksDone < s_trxSecurity.numAuthenticationHeaderBlocksNeeded)
    {
        // start the next CBC-Round for a header Block
        samr21TrxAesCbcEncrypt(s_trxSecurity.aesCbcBlock_1D, NULL); // only the last round CBC output is relevant
        // memcpy(AESCBCINPUT[AEStemp++],s_trxSecurity.aesCbcBlock_1D,IEEE_15_4_AES_CCM_BLOCK_SIZE);

        s_trxSecurity.numAuthenticationHeaderBlocksDone++;
    }
    else if (s_trxSecurity.numAuthenticationPayloadBlocksDone < s_trxSecurity.numAuthenticationPayloadBlocksNeeded)
    {
        // start the next CBC-Round for a Payload
        samr21TrxAesCbcEncrypt(s_trxSecurity.aesCbcBlock_1D, NULL); // only the last round CBC output is relevant
        // memcpy(AESCBCINPUT[AEStemp++],s_trxSecurity.aesCbcBlock_1D,IEEE_15_4_AES_CCM_BLOCK_SIZE);

        s_trxSecurity.numAuthenticationPayloadBlocksDone++;
    }

    if (s_trxSecurity.numAuthenticationPayloadBlocksDone == s_trxSecurity.numAuthenticationPayloadBlocksNeeded && s_trxSecurity.numAuthenticationHeaderBlocksDone == s_trxSecurity.numAuthenticationHeaderBlocksNeeded)
    {
        // Start a Timer for when the AES-Engine is done
        samr21TrxQueueDelayedAction(AT86RF233_ADJUSTED_AES_BUSY_TIME_us, samr21TrxAuthenticateFinish);
        return;
    }

    // Start a Timer for when the AES-Engine is done
    samr21TrxQueueDelayedAction(AT86RF233_ADJUSTED_AES_BUSY_TIME_us, samr21TrxAuthenticateLoop);

    // Prepare Memory for next Action when AES-Engine is done/ready
    if (s_trxSecurity.numAuthenticationHeaderBlocksDone < s_trxSecurity.numAuthenticationHeaderBlocksNeeded)
    {

        // Calculate current Header Offset
        uint8_t headerOffset = (IEEE_15_4_AES_CCM_BLOCK_SIZE * s_trxSecurity.numAuthenticationHeaderBlocksDone) - 2; // 2Bytes HeaderLen in first CBC Round

        // Fill a new Header Block
        for (uint8_t i = 0; i < IEEE_15_4_AES_CCM_BLOCK_SIZE; i++)
        {
            if ((headerOffset + i) < s_trxSecurity.headerLen)
            {
                s_trxSecurity.aesCbcBlock_1D[i] = s_trxSecurity.header_p[headerOffset + i];
                continue;
            }

            s_trxSecurity.aesCbcBlock_1D[i] = 0x00; // Fill up rest with 0
        }
    }
    else if (s_trxSecurity.numAuthenticationPayloadBlocksDone < s_trxSecurity.numAuthenticationPayloadBlocksNeeded)
    {

        // Calculate current Payload Offset
        uint8_t payloadOffset = (IEEE_15_4_AES_CCM_BLOCK_SIZE * s_trxSecurity.numAuthenticationPayloadBlocksDone);

        // Fill a new Header Block
        for (uint8_t i = 0; i < IEEE_15_4_AES_CCM_BLOCK_SIZE; i++)
        {
            if ((payloadOffset + i) < s_trxSecurity.payloadLen)
            {
                s_trxSecurity.aesCbcBlock_1D[i] = s_trxSecurity.payload_p[payloadOffset + i];
                continue;
            }

            s_trxSecurity.aesCbcBlock_1D[i] = 0x00; // Fill up rest with 0
        }
    }
}

static void samr21TrxAuthenticateStart()
{
    // Form init Vector (See IEEE 802.15.4-2015 Appendix B for good examples)
    // Nonce is already in CBC block because of union within the s_trxSecurity struct
    s_trxSecurity.aesCbcBlock_1D[0] =
        (s_trxSecurity.headerLen != 0 ? 0b01000000 : 0x0) | (((s_trxSecurity.micSize - 2) >> 1) << 3) | 1; // L always 2
    s_trxSecurity.aesCbcBlock_1D[IEEE_15_4_AES_CCM_BLOCK_SIZE - 2] = 0;                                    // always 0 cause maxlenght is 127
    s_trxSecurity.aesCbcBlock_1D[IEEE_15_4_AES_CCM_BLOCK_SIZE - 1] = s_trxSecurity.payloadLen;

    // start the first CBC-Round with init vector (plain ECB, r21 datasheet 40.1.4.2 Cipher Block Chaining (CBC))
    // simultaneously retrieve the MIC encryptionBlock from the CTR-Section
    samr21TrxAesEcbEncrypt(s_trxSecurity.aesCbcBlock_1D, s_trxSecurity.encryptionMask_1D);

    // Fill first CBC Block to be authenticated
    // First 2 Bytes are U16 representation of Header length
    s_trxSecurity.aesCbcBlock_1D[0] = s_trxSecurity.headerLen >> 8;
    s_trxSecurity.aesCbcBlock_1D[1] = s_trxSecurity.headerLen >> 0;

    // Catch the Case of a really small header Size
    if (s_trxSecurity.headerLen <= (IEEE_15_4_AES_CCM_BLOCK_SIZE - 2))
    {
        memcpy(&s_trxSecurity.aesCbcBlock_1D[2], s_trxSecurity.header_p, s_trxSecurity.headerLen);
        memset(&s_trxSecurity.aesCbcBlock_1D[2], 0, IEEE_15_4_AES_CCM_BLOCK_SIZE - 2 - s_trxSecurity.headerLen);
    }
    else
    {
        memcpy(&s_trxSecurity.aesCbcBlock_1D[2], s_trxSecurity.header_p, (IEEE_15_4_AES_CCM_BLOCK_SIZE - 2));
    }

    // Prep rest of the Memory for the next Actions
    s_trxSecurity.numAuthenticationHeaderBlocksDone = 0;
    s_trxSecurity.numAuthenticationHeaderBlocksNeeded = (s_trxSecurity.headerLen + 2) / IEEE_15_4_AES_CCM_BLOCK_SIZE; // First Header Block starts with headerLen
    if ((s_trxSecurity.headerLen + 2) % IEEE_15_4_AES_CCM_BLOCK_SIZE)
    {
        s_trxSecurity.numAuthenticationHeaderBlocksNeeded++;
    }

    s_trxSecurity.currentSizeCbcBlock = IEEE_15_4_AES_CCM_BLOCK_SIZE;
    s_trxSecurity.numAuthenticationPayloadBlocksDone = 0;

    // numAuthenticationPayloadBlocksNeeded is already calculated because of union with EncryptionBlockNeeded

    // Start a Timer for when the AES-Engine is done
    samr21TrxQueueDelayedAction(AT86RF233_ADJUSTED_AES_BUSY_TIME_us, samr21TrxAuthenticateLoop);
}

static void samr21TrxEncryptAndUploadPayload()
{
    while (s_trxSecurity.numEncryptionBlocksDone < s_trxSecurity.numEncryptionBlocksNeeded)
    {
        if ((s_trxSecurity.numEncryptionBlocksDone + 1) < s_trxSecurity.numEncryptionBlocksNeeded)
        {
            // increase Nonce Counter for next AES-CTR-Block
            s_trxSecurity.aesCtrBlock_1D[(IEEE_15_4_AES_CCM_BLOCK_SIZE - 1)]++;
            // start uploading next AES-CTR Block, while simultaneously retrieving (SPI SRAM-Fast-Access) the result of the last AES-CTR-Round
            samr21TrxAesEcbEncrypt(s_trxSecurity.aesCtrBlock_1D, s_trxSecurity.encryptionMask_1D);
            // Write via SRAM starting from Addr of the current Byte to the Framebuffer
            samr21TrxUploadEncryptedToFramebuffer(
                (s_trxSecurity.payload_p + (IEEE_15_4_AES_CCM_BLOCK_SIZE * s_trxSecurity.numEncryptionBlocksDone)),
                s_trxSecurity.encryptionMask_1D,
                IEEE_15_4_AES_CCM_BLOCK_SIZE,
                s_trxSecurity.numBytesUploaded);
            s_trxSecurity.numBytesUploaded += IEEE_15_4_AES_CCM_BLOCK_SIZE;
        }
        else
        {

            uint8_t lastEncryptedPayloadBlockSize = ((s_trxSecurity.payloadLen % IEEE_15_4_AES_CCM_BLOCK_SIZE) ? (s_trxSecurity.payloadLen % IEEE_15_4_AES_CCM_BLOCK_SIZE) : IEEE_15_4_AES_CCM_BLOCK_SIZE);

            // set Nonce Counter to 0 for later encryption of MIC
            s_trxSecurity.aesCtrBlock_1D[(IEEE_15_4_AES_CCM_BLOCK_SIZE - 1)] = 0;
            // start encrypting the AES-CTR Block for MIC encryption, while simultaneously retrieving (SPI SRAM-Fast-Access) the result of the final Payload AES-CTR-Round
            samr21TrxAesEcbEncrypt(s_trxSecurity.aesCtrBlock_1D, s_trxSecurity.encryptionMask_1D);
            // Write via SRAM starting from Addr of the current Byte to the Framebuffer
            samr21TrxUploadEncryptedToFramebuffer(
                (s_trxSecurity.payload_p + (IEEE_15_4_AES_CCM_BLOCK_SIZE * s_trxSecurity.numEncryptionBlocksDone)),
                s_trxSecurity.encryptionMask_1D,
                lastEncryptedPayloadBlockSize,
                s_trxSecurity.numBytesUploaded);

            s_trxSecurity.numBytesUploaded += lastEncryptedPayloadBlockSize;
        }

        s_trxSecurity.numEncryptionBlocksDone++;
    }

    // Delay is needed here cause if the last PayloadBlock is pretty short
    // There could be a Edge Case where the time it take to upload this last Block
    // Is shorter then the AES Cycle Time
    if (s_trxSecurity.micSize)
    {
        samr21TrxAuthenticateStart();
    }
}

void samr21TrxSendFrameAndApplyMacSecurity(uint8_t *a_header_p, uint8_t *a_payload_p, uint8_t *a_footer_p, uint8_t a_secLevel, uint8_t *a_key_1D, uint8_t *a_nonce_1D)
{

#ifdef _DEBUG
    assert(a_secLevel >= 0b100);
#endif

    // Block AES Engine, so no Key Change takes places while frame is being transmitted
    s_aesBusy = true;

    // Upload the unecrypted Part of the Message first. This is done o the Framebuffer doesn't get empty, while AES-CCM* is ongoing
    // Frame Transmission and AES-CCM* is done in Parallel
    s_trxSecurity.headerLen = (uint8_t)(a_payload_p - a_header_p);
    samr21TrxUploadToFramebuffer(a_header_p, s_trxSecurity.headerLen, 0);

    // Start Transmission
    samr21TrxSetSLP_TR(true);

    // Prep CTR Block (input for AES-Engine, Output get XORed with payload)
    s_trxSecurity.aesCtrBlock_1D[0] = 1; // Always 1 cause L is always 2
    memcpy(&s_trxSecurity.aesCtrBlock_1D[1], a_nonce_1D, IEEE_15_4_AES_CCM_NONCE_SIZE);
    s_trxSecurity.aesCtrBlock_1D[IEEE_15_4_AES_CCM_BLOCK_SIZE - 2] = 0; // Always 0 cause ctr-Value never exceeds 0xFF
    s_trxSecurity.aesCtrBlock_1D[IEEE_15_4_AES_CCM_BLOCK_SIZE - 1] = 1; // Start with ctr = 1, because timing for payload encryption is crucial

    // Start Encryption of first CTR Block
    samr21TrxAesEcbEncrypt(s_trxSecurity.aesCtrBlock_1D, NULL);
    // Start Timer for when result AES-Engine is done
    samr21TrxQueueDelayedAction(AT86RF233_ADJUSTED_AES_BUSY_TIME_us, samr21TrxEncryptAndUploadPayload);

    // Prep part of the Memory for the next Actions
    s_trxSecurity.numBytesUploaded = s_trxSecurity.headerLen;
    s_trxSecurity.securityLevel = a_secLevel;

    // Copy Frame inicies pointer
    s_trxSecurity.payload_p = a_payload_p;
    s_trxSecurity.footer_p = a_footer_p;

    // Remove psdu-Framelenght octet form header. Only mac-frame content is of intrest for AES
    s_trxSecurity.headerLen--;
    s_trxSecurity.header_p = a_header_p + 1;

    s_trxSecurity.payloadLen = (uint8_t)(a_footer_p - a_payload_p);

    s_trxSecurity.numEncryptionBlocksDone = 0;
    s_trxSecurity.numEncryptionBlocksNeeded = s_trxSecurity.payloadLen / IEEE_15_4_AES_CCM_BLOCK_SIZE;
    if (s_trxSecurity.payloadLen % IEEE_15_4_AES_CCM_BLOCK_SIZE)
    {
        s_trxSecurity.numEncryptionBlocksNeeded++;
    }

    switch (s_trxSecurity.securityLevel & 0b11)
    {
    case 0b00:
        s_trxSecurity.micSize = 0;
        break;

    case 0b01:
        s_trxSecurity.micSize = 4;
        break;

    case 0b10:
        s_trxSecurity.micSize = 8;
        break;

    case 0b11:
        s_trxSecurity.micSize = 16;
        break;
    }

    // Transmission Should have started by now
    samr21TrxSetSLP_TR(false);
}

/**********TRX EVENT HANDLER********/

// External IRQs form TRX
static void (*s_currentIsrHandler_fktPtr[NUM_TRX_IRQS])(void) = {NULL};

void samr21TrxSetIrqHandler(uint8_t a_irqType, void (*a_handler_fktPtr)(void))
{

#ifdef _DEBUG
    assert(a_irqType <= (NUM_TRX_IRQS - 1));
#endif

    s_currentIsrHandler_fktPtr[a_irqType] = a_handler_fktPtr;
}

void samr21TrxRemoveAllHandler()
{
    for (uint8_t i = 0; i < NUM_TRX_IRQS; i++)
    {
        s_currentIsrHandler_fktPtr[i] = NULL;
    }
}

void samr21TrxDisableAllIrq()
{
    s_ramCopyTrxRegister.irqMask.reg = 0x00;

    samr21TrxWriteRegister(IRQ_MASK_REG_ADDR, s_ramCopyTrxRegister.irqMask.reg);
}

void samr21TrxEnableIrq(uint8_t a_irqType)
{

#ifdef _DEBUG
    assert(a_irqType <= (NUM_TRX_IRQS - 1));
#endif

    uint8_t shiftedIrqEnableBit = 1 << a_irqType;

    if (shiftedIrqEnableBit & s_ramCopyTrxRegister.irqMask.reg)
    {
        // Already enabled
        return;
    }

    s_ramCopyTrxRegister.irqMask.reg |= shiftedIrqEnableBit;

    samr21TrxWriteRegister(IRQ_MASK_REG_ADDR, s_ramCopyTrxRegister.irqMask.reg);
}

void samr21TrxDisableIrq(uint8_t a_irqType)
{

#ifdef _DEBUG
    assert(a_irqType <= (NUM_TRX_IRQS - 1));
#endif

    uint8_t shiftedIrqDisableBit = 1 << a_irqType;

    if (shiftedIrqDisableBit & ~s_ramCopyTrxRegister.irqMask.reg)
    {
        // Already disabled
        return;
    }

    s_ramCopyTrxRegister.irqMask.reg &= ~shiftedIrqDisableBit;

    samr21TrxWriteRegister(IRQ_MASK_REG_ADDR, s_ramCopyTrxRegister.irqMask.reg);
}

void EIC_Handler()
{
    // Clear IRQ
    EIC->INTFLAG.bit.EXTINT0 = 1;

    s_ramCopyTrxRegister.irqStatus.reg = samr21TrxReadRegister(IRQ_STATUS_REG_ADDR);

    if (s_ramCopyTrxRegister.irqStatus.bit.rxStart && (s_currentIsrHandler_fktPtr[TRX_IRQ_RX_START] != NULL))
    {
        (*s_currentIsrHandler_fktPtr[TRX_IRQ_RX_START])();
    }

    if (s_ramCopyTrxRegister.irqStatus.bit.trxEnd && (s_currentIsrHandler_fktPtr[TRX_IRQ_TRX_END] != NULL))
    {
        (*s_currentIsrHandler_fktPtr[TRX_IRQ_TRX_END])();
    }

    if (s_ramCopyTrxRegister.irqStatus.bit.pllLock && (s_currentIsrHandler_fktPtr[TRX_IRQ_PLL_LOCK] != NULL))
    {
        (*s_currentIsrHandler_fktPtr[TRX_IRQ_TRX_END])();
    }

    if (s_ramCopyTrxRegister.irqStatus.bit.pllUnlock && (s_currentIsrHandler_fktPtr[TRX_IRQ_PLL_UNLOCK] != NULL))
    {
        (*s_currentIsrHandler_fktPtr[TRX_IRQ_PLL_UNLOCK])();
    }

    if (s_ramCopyTrxRegister.irqStatus.bit.ccaEdDone && (s_currentIsrHandler_fktPtr[TRX_IRQ_CCA_ED_DONE] != NULL))
    {
        (*s_currentIsrHandler_fktPtr[TRX_IRQ_CCA_ED_DONE])();
    }

    if (s_ramCopyTrxRegister.irqStatus.bit.addressMatch && (s_currentIsrHandler_fktPtr[TRX_IRQ_AMI] != NULL))
    {
        (*s_currentIsrHandler_fktPtr[TRX_IRQ_AMI])();
    }

    if (s_ramCopyTrxRegister.irqStatus.bit.bufferUnderRun && (s_currentIsrHandler_fktPtr[TRX_IRQ_TRX_UR] != NULL))
    {
        (*s_currentIsrHandler_fktPtr[TRX_IRQ_TRX_UR])();
    }

    if (s_ramCopyTrxRegister.irqStatus.bit.batteryLow && (s_currentIsrHandler_fktPtr[TRX_IRQ_BAT_LOW] != NULL))
    {
        (*s_currentIsrHandler_fktPtr[TRX_IRQ_BAT_LOW])();
    }
}