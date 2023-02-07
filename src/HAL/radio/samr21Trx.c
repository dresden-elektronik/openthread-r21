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



static volatile bool s_spiActive;

void samr21TrxInterfaceInit(){
    //Setup Clocks for TRX-SPI
        //Use GCLKGEN1 as core Clock for SPI At86rf233 (SERCOM4)
        GCLK->CLKCTRL.reg =
            //GCLK_CLKCTRL_WRTLOCK
            GCLK_CLKCTRL_CLKEN
            |GCLK_CLKCTRL_GEN(1) // GCLKGEN1 (Sourced from trx clkm)
            |GCLK_CLKCTRL_ID(GCLK_CLKCTRL_ID_SERCOM4_CORE_Val)
        ;
        //Wait for synchronization 
        while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY );

        GCLK->CLKCTRL.reg =
            //GCLK_CLKCTRL_WRTLOCK
            GCLK_CLKCTRL_CLKEN
            |GCLK_CLKCTRL_GEN(1) // GCLKGEN1
            |GCLK_CLKCTRL_ID(GCLK_CLKCTRL_ID_SERCOMX_SLOW_Val)
        ;
        //Wait for synchronization 
        while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY );
    
    //Enable in Power manager
        PM->APBCMASK.bit.SERCOM4_ = 1;

    //Setup Ports for TRX
        //Setup PIN PC19 as MISO <-> At86rf233 via SERCOM4
            //Make Input
            PORT->Group[2].DIRCLR.reg= PORT_PC19;

            //Setup Mux Settings
            PORT->Group[2].WRCONFIG.reg =
                PORT_WRCONFIG_HWSEL
                |PORT_WRCONFIG_WRPINCFG
                |PORT_WRCONFIG_WRPMUX
                |PORT_WRCONFIG_PMUX(MUX_PC19F_SERCOM4_PAD0)
                //|PORT_WRCONFIG_PULLEN
                |PORT_WRCONFIG_INEN
                |PORT_WRCONFIG_PMUXEN
                |PORT_WRCONFIG_PINMASK(PORT_PC19 >> 16) //upper Halfword
            ;
        //Setup PIN PB30 as MOSI <-> At86rf233 via SERCOM4
            //Make Output
            PORT->Group[1].DIRSET.reg= PORT_PB30;

            //Setup Mux Settings
            PORT->Group[1].WRCONFIG.reg =
                PORT_WRCONFIG_HWSEL
                |PORT_WRCONFIG_WRPINCFG
                |PORT_WRCONFIG_WRPMUX
                |PORT_WRCONFIG_PMUX(MUX_PB30F_SERCOM4_PAD2)
                //|PORT_WRCONFIG_PULLEN
                //|PORT_WRCONFIG_INEN
                |PORT_WRCONFIG_PMUXEN
                |PORT_WRCONFIG_PINMASK(PORT_PB30 >> 16) //upper Halfword
            ;

        //Setup PIN PC18 as SCLK <-> At86rf233 via SERCOM4
            //Make Output
            PORT->Group[2].DIRSET.reg= PORT_PC18;

            //Setup Mux Settings
            PORT->Group[2].WRCONFIG.reg =
                PORT_WRCONFIG_HWSEL
                |PORT_WRCONFIG_WRPINCFG
                |PORT_WRCONFIG_WRPMUX
                |PORT_WRCONFIG_PMUX(MUX_PC18F_SERCOM4_PAD3)
                //|PORT_WRCONFIG_PULLEN
                //|PORT_WRCONFIG_INEN
                |PORT_WRCONFIG_PMUXEN
                |PORT_WRCONFIG_PINMASK(PORT_PC18 >> 16) //upper Halfword
            ;

        //Setup PIN PB31 as SEL <-> At86rf233
        //SEL is configreud as Mnual PORT, cause we need Control over ist for
        //Realtime Buffer Read
            //Make Output
            PORT->Group[1].DIRSET.reg= PORT_PB31;

            //Setup Mux Settings
            PORT->Group[1].WRCONFIG.reg =
                PORT_WRCONFIG_HWSEL
                |PORT_WRCONFIG_WRPINCFG
                //|PORT_WRCONFIG_WRPMUX
                //|PORT_WRCONFIG_PMUX(0)
                //|PORT_WRCONFIG_PULLEN
                //|PORT_WRCONFIG_INEN
                //|PORT_WRCONFIG_PMUXEN
                |PORT_WRCONFIG_PINMASK(PORT_PB31 >> 16) //upper Halfword
            ;
            
            //Set Default High
            PORT->Group[1].OUT.reg |= ((uint32_t)1 & 0x1) << 31;
        
        //Setup PIN PB15 as RSTN <-> At86rf233
            //Make Output
            PORT->Group[1].DIRSET.reg= PORT_PB15;

            //Setup Mux Settings
            PORT->Group[1].WRCONFIG.reg =
                //PORT_WRCONFIG_HWSEL
                PORT_WRCONFIG_WRPINCFG
                //|PORT_WRCONFIG_WRPMUX
                //|PORT_WRCONFIG_PMUX(0)
                //|PORT_WRCONFIG_PULLEN
                //|PORT_WRCONFIG_INEN
                //|PORT_WRCONFIG_PMUXEN
                |PORT_WRCONFIG_PINMASK(PORT_PB15) //lower Halfword
            ;

            //Set Default High
            PORT->Group[1].OUT.reg |= ((uint32_t)1 & 0x1) << 15;
        //Setup PIN PB00 as IRQ <-> At86rf233 via EXTINT[0]      
            //Make Input
            PORT->Group[1].DIRCLR.reg= PORT_PB00;

            //Setup Mux Settings
            PORT->Group[1].WRCONFIG.reg =
                //PORT_WRCONFIG_HWSEL
                PORT_WRCONFIG_WRPINCFG
                |PORT_WRCONFIG_WRPMUX
                |PORT_WRCONFIG_PMUX(MUX_PB00A_EIC_EXTINT0)
                //|PORT_WRCONFIG_PULLEN
                |PORT_WRCONFIG_INEN
                |PORT_WRCONFIG_PMUXEN
                |PORT_WRCONFIG_PINMASK(PORT_PB00) //lower Halfword
            ;
        //Setup PIN PA20 as SLP_TR <-> At86rf233
            //Make Output
            PORT->Group[0].DIRSET.reg= PORT_PA20;

            //Setup Mux Settings
            PORT->Group[0].WRCONFIG.reg =
                PORT_WRCONFIG_HWSEL
                |PORT_WRCONFIG_WRPINCFG
                //|PORT_WRCONFIG_WRPMUX
                //|PORT_WRCONFIG_PMUX(0)
                //|PORT_WRCONFIG_PULLEN
                //|PORT_WRCONFIG_INEN
                //|PORT_WRCONFIG_PMUXEN
                |PORT_WRCONFIG_PINMASK(PORT_PA20 >> 16) //upper Halfword
            ;

            //Set Default Low
            PORT->Group[0].OUT.reg |= ((uint32_t)0 & 0x1) << 20;


    //Reset SERCOM4
        SERCOM4->SPI.CTRLA.bit.SWRST = 1;

    // Wait for SERCOM4 reset to finish
        while ( SERCOM4->SPI.CTRLA.bit.SWRST || SERCOM4->SPI.SYNCBUSY.bit.SWRST );
    
    //Setup SERCOM4
        
        //F_ref = 16MHz (if At86r233 ist setup correctly) F_baud = F_ref / 2*(BAUD+1) ---> BAUD = 0 F_baud = 8Mhz
        SERCOM4->SPI.BAUD.reg=
            SERCOM_SPI_BAUD_BAUD(0x0) 
        ;

        SERCOM4->SPI.CTRLB.reg=
            SERCOM_SPI_CTRLB_RXEN
            //|SERCOM_SPI_CTRLB_AMODE(0)
            //|SERCOM_SPI_CTRLB_MSSEN
            //|SERCOM_SPI_CTRLB_SSDE
            //|SERCOM_SPI_CTRLB_PLOADEN
            |SERCOM_SPI_CTRLB_CHSIZE(0x0) //8Bit
        ;
        // Wait for SERCOM4 Sync
        while ( SERCOM4->SPI.SYNCBUSY.bit.CTRLB );


        SERCOM4->SPI.CTRLA.reg=
            SERCOM_SPI_CTRLA_RUNSTDBY
            |SERCOM_SPI_CTRLA_MODE(SERCOM_SPI_CTRLA_MODE_SPI_MASTER_Val)
            |SERCOM_SPI_CTRLA_ENABLE
            //|SERCOM_SPI_CTRLA_SWRST
            //|SERCOM_SPI_CTRLA_IBON
            |SERCOM_SPI_CTRLA_DOPO(1) //MOSI IS ON PB30 SERCOM4/PAD[2] , SCLK ON PB30 SERCOM4/PAD[3]
            |SERCOM_SPI_CTRLA_DIPO(0) //MISO IS ON PC19 SERCOM4/PAD[0]
            |SERCOM_SPI_CTRLA_FORM(0)
            //|SERCOM_SPI_CTRLA_CPHA
            //|SERCOM_SPI_CTRLA_CPOL
            //|SERCOM_SPI_CTRLA_DORD
        ;
        // Wait for SERCOM4 to setup
        while (SERCOM4->SPI.SYNCBUSY.reg);

    //Write predefined Operating-Values to Config Registers of AT86rf233 
        samr21TrxWriteRegister(TRX_CTRL_1_REG,  g_trxCtrl1.reg);
        samr21TrxWriteRegister(PHY_TX_PWR_REG,  g_phyTxPwr.reg);
        samr21TrxWriteRegister(IRQ_MASK_REG,    g_irqMask.reg);
}

void samr21TrxSetupMClk(uint8_t clk){

    //Write a 0x5 into the CLKM Register to get a 16Mhz Clock on CLKM
    g_trxCtrl0.bit.clkmCtrl=clk;

    samr21TrxWriteRegister(TRX_CTRL_0_REG, g_trxCtrl0.reg);

    //Wait a bit for the  the Freq.-change to take place
    samr21delaySysTick(50);  
}

uint8_t samr21TrxReadRegister(uint8_t addr){

    samr21TrxSpiStartAccess(AT86RF233_CMD_REG_READ_MASK, addr);

#ifdef __CONSERVATIVE_TRX_SPI_TIMING__
    samr21delaySysTick(CPU_WAIT_CYCLES_BETWEEN_BYTES);
#endif

    uint8_t rVal = samr21TrxSpiReadByteRaw();

    samr21TrxSpiCloseAccess();

    return rVal;
}

void samr21TrxWriteRegister(uint8_t addr, uint8_t data){

    samr21TrxSpiStartAccess(AT86RF233_CMD_REG_WRITE_MASK, addr);

#ifdef __CONSERVATIVE_TRX_SPI_TIMING__
    samr21delaySysTick(CPU_WAIT_CYCLES_BETWEEN_BYTES);
#endif

    samr21TrxSpiTransceiveByteRaw(data);

    samr21TrxSpiCloseAccess();
}

void samr21TrxUpdateStatus(){

    while (s_spiActive);
    PORT->Group[0].OUTSET.reg= PORT_PA07;
    __disable_irq();
    PORT->Group[1].OUTCLR.reg = 1 << 31; //SSel Low Active
    s_spiActive = true;

    g_trxStatus.reg = samr21TrxSpiTransceiveByteRaw(SPI_DUMMY_BYTE);

    samr21TrxSpiCloseAccess();
}

void samr21TrxSpiStartAccess(uint8_t command, uint8_t addr){
    //Wait for other ongoing SPI Access to End
    while (s_spiActive);
    PORT->Group[0].OUTSET.reg= PORT_PA07;
    __disable_irq();
    s_spiActive = true;
    PORT->Group[1].OUTCLR.reg = 1 << 31; //SSel Low Active

#ifdef __CONSERVATIVE_TRX_SPI_TIMING__
    samr21delaySysTick(CPU_WAIT_CYCLES_AFTER_SSEL_LOW);
#endif

    //Register Read/Write
    if(command & AT86RF233_CMD_REG_READ_MASK){
        g_trxStatus.reg = samr21TrxSpiTransceiveByteRaw( ( addr & 0x3F ) | command );
        return;
    }

    //Framebuffer Access
    if(command & AT86RF233_CMD_FRAMEBUFFER_READ){
        g_trxStatus.reg = samr21TrxSpiTransceiveByteRaw(command);
        return;
    }

    //SRAM Access
    g_trxStatus.reg = samr21TrxSpiTransceiveByteRaw(command);

#ifdef __CONSERVATIVE_TRX_SPI_TIMING__
    samr21delaySysTick(CPU_WAIT_CYCLES_BETWEEN_BYTES);
#endif
    samr21TrxSpiTransceiveByteRaw(addr);
}

void samr21TrxSpiCloseAccess(){
    PORT->Group[1].OUTSET.reg = 1 << 31;//SSel Low Active

#ifdef __CONSERVATIVE_TRX_SPI_TIMING__
    samr21delaySysTick(CPU_WAIT_CYCLES_AFTER_SSEL_LOW);
#endif
    s_spiActive = false;
    __enable_irq();
    PORT->Group[0].OUTCLR.reg= PORT_PA07;
}

bool samr21TrxSpiBusy(){
    return s_spiActive;
}

void samr21TrxSetRSTN(bool enabled){
    //Pin PB15 == RSTN
    //Active Low
    if(enabled){
        PORT->Group[1].OUTCLR.reg = 1 << 15;
        return;
    }
    PORT->Group[1].OUTSET.reg = 1 << 15;
}

void samr21TrxSetSLP_TR(bool enabled){
    //Pin PA20 == SLP_TR
    //Active High
    if(enabled){
        PORT->Group[0].OUTSET.reg = 1 << 20;
        return;
    }
    PORT->Group[0].OUTCLR.reg = 1 << 20;
}

uint8_t samr21TrxSpiTransceiveByteRaw(uint8_t data){
    while (!SERCOM4->SPI.INTFLAG.bit.DRE);
    //Put data into the tranmitt buffer to start transmission
    //PORT->Group[0].OUTTGL.reg= PORT_PA07;
    SERCOM4->SPI.DATA.bit.DATA = data;
    
    while (!SERCOM4->SPI.INTFLAG.bit.TXC);
    while (!SERCOM4->SPI.INTFLAG.bit.RXC);
    //PORT->Group[0].OUTTGL.reg= PORT_PA07;

    //return recived Answerto
    return SERCOM4->SPI.DATA.bit.DATA;
}

void samr21TrxSpiTransceiveBytesRaw(uint8_t *inData, uint8_t *outData, uint8_t len){
    for (uint8_t i = 0; i < len; i++)
    {

#ifdef __CONSERVATIVE_TRX_SPI_TIMING__
        samr21delaySysTick(CPU_WAIT_CYCLES_BETWEEN_BYTES);
#endif       

        if(!outData){
            samr21TrxSpiTransceiveByteRaw(inData[i]);
            continue;
        }
 
        outData[i] = samr21TrxSpiTransceiveByteRaw(inData != NULL ? inData[i] : SPI_DUMMY_BYTE);
    } 
}

uint8_t  samr21TrxSpiReadByteRaw(){
    return samr21TrxSpiTransceiveByteRaw(SPI_DUMMY_BYTE);
}


uint8_t samr21TrxGetRandomCrumb()
{
    return (samr21TrxReadRegister(PHY_RSSI_REG) >> 5) & 0b00000011;
}
uint8_t samr21TrxGetRandomNibble()
{
    return ((samr21TrxReadRegister(PHY_RSSI_REG) >> 5) & 0b00000011) | ((samr21TrxReadRegister(PHY_RSSI_REG) >> 3) & 0b00001100);
}
uint8_t samr21TrxGetRandomByte()
{
    uint8_t rVal = (samr21TrxReadRegister(PHY_RSSI_REG) & 0x60) >> 5;
    rVal |= (samr21TrxReadRegister(PHY_RSSI_REG) & 0x60) >> 3;
    rVal |= (samr21TrxReadRegister(PHY_RSSI_REG) & 0x60) >> 1;
    rVal |= (samr21TrxReadRegister(PHY_RSSI_REG) & 0x60) << 1;
    return rVal;
}

