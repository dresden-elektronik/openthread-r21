/*
 * Copyright (c) 2023 dresden elektronik ingenieurtechnik gmbh.
 * All rights reserved.
 *
 * The software in this package is published under the terms of the BSD
 * style license a copy of which has been included with this distribution in
 * the LICENSE.txt file.
 *
 */
//Inspired by https://github.com/arduino/ArduinoCore-samd

#include "samr21Clock.h"

static bool s_dfllClockSourceActive = false;

static bool s_gclk0DependsOnDfll = false; //CPU Clock

static bool s_gclk1Enabled = false;
static bool s_gclk1SourcedByTrx = false;

static bool s_gclk2Enabled = false; //Used by WDT, better don't touch this
static bool s_gclk2DependsOnDfll = false; 

static bool s_gclk3Enabled = false; //Used by RTC and Timer
static bool s_gclk3DependsOnDfll = false; 

static bool s_gclk4Enabled = false; //Used by TRX-SPI 
static bool s_gclk4DependsOnDfll = false; 

static bool isDfllShutdownSafe(void)
{
    return ((s_dfllClockSourceActive) && !(s_gclk0DependsOnDfll || s_gclk2DependsOnDfll || s_gclk3DependsOnDfll || s_gclk4DependsOnDfll));
}

static bool isDfllUsable(void)
{
    return s_dfllClockSourceActive && s_gclk1Enabled;
}

static void disablePeripheralClock(uint8_t peripheralClock)
{
    __disable_irq();
   	/* Select the requested generator channel */
	*((uint8_t*)&GCLK->CLKCTRL.reg) = peripheralClock;

	/* Sanity check WRTLOCK */
	while(GCLK->CLKCTRL.bit.WRTLOCK);

	/* Switch to known-working source so that the channel can be disabled */
	uint32_t prev_gen_id = GCLK->CLKCTRL.bit.GEN;
	GCLK->CLKCTRL.bit.GEN = 0;

	/* Disable the generic clock */
	GCLK->CLKCTRL.reg &= ~GCLK_CLKCTRL_CLKEN;
	while (GCLK->CLKCTRL.reg & GCLK_CLKCTRL_CLKEN) {
		/* Wait for clock to become disabled */
	}

	/* Restore previous configured clock generator */
	GCLK->CLKCTRL.bit.GEN = prev_gen_id; 
    __enable_irq();
}


static void enablePeripheralClock(uint8_t peripheralClock, uint8_t gClkGen)
{
    disablePeripheralClock(peripheralClock);

    __disable_irq();

	/* Write the new configuration */
	GCLK->CLKCTRL.reg = 
        GCLK_CLKCTRL_ID(GCLK_CLKCTRL_ID_RTC_Val)
        | GCLK_CLKCTRL_GEN(gClkGen)
    ;

    /* Select the requested peripheralClock */
	*((uint8_t*)&GCLK->CLKCTRL.reg) = peripheralClock;

	/* Enable the generic clock */
	GCLK->CLKCTRL.reg |= GCLK_CLKCTRL_CLKEN;

    __enable_irq();
}




static void enableAllPeripheralClocks(void)
{
    //CPU Clock (48Mhz)
    enablePeripheralClock(GCLK_CLKCTRL_ID_EIC_Val,0);
    enablePeripheralClock(GCLK_CLKCTRL_ID_USB_Val,0);

    //Timer Clock (1Mhz)
    enablePeripheralClock(GCLK_CLKCTRL_ID_RTC_Val,3);
    enablePeripheralClock(GCLK_CLKCTRL_ID_TCC0_TCC1_Val, 3);
    enablePeripheralClock(GCLK_CLKCTRL_ID_TCC2_TC3_Val, 3);
    enablePeripheralClock(GCLK_CLKCTRL_ID_TC4_TC5_Val, 3);

    //Sercom Slow Clock (1 MHz, DebugUart and APB Interface)
    enablePeripheralClock(GCLK_CLKCTRL_ID_SERCOMX_SLOW_Val, 3);
    enablePeripheralClock(GCLK_CLKCTRL_ID_SERCOM2_CORE_Val, 3);

    //Sercom Fast Clock (8 MHz TRX SPI)
    enablePeripheralClock(GCLK_CLKCTRL_ID_SERCOM4_CORE_Val, 4);
}

static void disableAllPeripheralClocks(void)
{
    disablePeripheralClock(GCLK_CLKCTRL_ID_EIC_Val);
    disablePeripheralClock(GCLK_CLKCTRL_ID_USB_Val);
    disablePeripheralClock(GCLK_CLKCTRL_ID_RTC_Val);
    disablePeripheralClock(GCLK_CLKCTRL_ID_TCC0_TCC1_Val);
    disablePeripheralClock(GCLK_CLKCTRL_ID_TCC2_TC3_Val);
    disablePeripheralClock(GCLK_CLKCTRL_ID_TC4_TC5_Val);
    disablePeripheralClock(GCLK_CLKCTRL_ID_SERCOMX_SLOW_Val);
    disablePeripheralClock(GCLK_CLKCTRL_ID_SERCOM2_CORE_Val);
    disablePeripheralClock(GCLK_CLKCTRL_ID_SERCOM4_CORE_Val);
}


bool samr21Clock_switchGen0Source(bool useDfll)
{
    //Setup GENDIV first (should output 8MHz when DFLL is used, ~1 when not)
    if(useDfll && !isDfllUsable())
    {
        return false;
    }

    //Should use the Clock Src 1:1 (1MHz or 48Mhz)
    GCLK->GENDIV.reg = 
        GCLK_GENDIV_ID(0)
        |GCLK_GENDIV_DIV(0)
    ;
    //Wait for synchronization 
    while ( GCLK->STATUS.bit.SYNCBUSY );


    //Put GCLK 0 (Main Clk) on the OSC8M or DFLL
    GCLK->GENCTRL.reg = 
        GCLK_GENCTRL_ID(0) 
        |GCLK_GENCTRL_SRC(useDfll ? GCLK_GENCTRL_SRC_DFLL48M_Val : GCLK_GENCTRL_SRC_OSC8M_Val)
        //|GCLK_GENCTRL_RUNSTDBY
        //|GCLK_GENCTRL_DIVSEL
        //|GCLK_GENCTRL_OE
        //|GCLK_GENCTRL_OOV
        |GCLK_GENCTRL_GENEN
    ;

    //Wait for synchronization 
    while ( GCLK->STATUS.bit.SYNCBUSY || !GCLK->GENCTRL.bit.GENEN);

    s_gclk0DependsOnDfll = useDfll; 
    return true;
}

bool samr21Clock_enableGen1(bool useTrxSource)
{
    if(s_dfllClockSourceActive)
    {
        //Dfll Uses this clock as a Ref, if Dlff is active we cant modify this clock
        return false;
    }

    if(useTrxSource)
    {
        //Setup PIN PC16 as Clock-Input from mClk-Pin from At86rf233
        //Make Input
        PORT->Group[2].DIRCLR.reg= PORT_PC16;

        //Setup Mux Settings
        PORT->Group[2].WRCONFIG.reg =
            PORT_WRCONFIG_HWSEL
            |PORT_WRCONFIG_WRPINCFG
            |PORT_WRCONFIG_WRPMUX
            |PORT_WRCONFIG_PMUX(MUX_PC16F_GCLK_IO1)
            //PORT_WRCONFIG_PULLEN
            |PORT_WRCONFIG_INEN
            |PORT_WRCONFIG_PMUXEN
            |PORT_WRCONFIG_PINMASK(PORT_PC16 >> 16) //upper Halfword
        ;
    }

    //Disable first
    GCLK->GENCTRL.reg = 
        GCLK_GENCTRL_ID(1)
        |GCLK_GENCTRL_SRC(useTrxSource ? GCLK_GENCTRL_SRC_GCLKIN_Val : GCLK_GENCTRL_SRC_OSC8M_Val)
        //|GCLK_GENCTRL_RUNSTDBY
        |GCLK_GENCTRL_DIVSEL
        //|GCLK_GENCTRL_OE
        //|GCLK_GENCTRL_OOV
        //|GCLK_GENCTRL_GENEN
    ;
    while ( GCLK->STATUS.bit.SYNCBUSY );

    //Setup GENDIV first (should output 31.250 kHz)
    GCLK->GENDIV.reg = 
        GCLK_GENDIV_ID(1) 
        |GCLK_GENDIV_DIV(4) //Uses exp Formate 2^4 (see GCLK_GENCTRL_DIVSEL)
    ;
    //Wait for synchronization 
    while ( GCLK->STATUS.bit.SYNCBUSY );

    //Put GCLK 1 (REF for DFLL) on the OSC8M or TRX_CLK_OUT (Both 1 MHz)
    GCLK->GENCTRL.reg = 
        GCLK_GENCTRL_ID(1)
        |GCLK_GENCTRL_SRC(useTrxSource ? GCLK_GENCTRL_SRC_GCLKIN_Val : GCLK_GENCTRL_SRC_OSC8M_Val)
        //|GCLK_GENCTRL_RUNSTDBY
        |GCLK_GENCTRL_DIVSEL
        //|GCLK_GENCTRL_OE
        //|GCLK_GENCTRL_OOV
        |GCLK_GENCTRL_GENEN
    ;
    //Wait for synchronization 
    while ( GCLK->STATUS.bit.SYNCBUSY);

    s_gclk1Enabled = true;
    s_gclk1SourcedByTrx = useTrxSource;
    return true;
}

bool samr21Clock_enableGen2(void)
{
    return true; //We don't mess with GCLK2 
} 

bool samr21Clock_enableGen3(bool useDfll)
{
    if(useDfll && !isDfllUsable())
    {
        return false;
    }

    //Disable GCLK 3 first
    GCLK->GENCTRL.reg = 
        GCLK_GENCTRL_ID(3)
        |GCLK_GENCTRL_SRC(useDfll ? GCLK_GENCTRL_SRC_DFLL48M_Val : GCLK_GENCTRL_SRC_OSC8M_Val)
        //|GCLK_GENCTRL_RUNSTDBY
        //|GCLK_GENCTRL_DIVSEL
        //|GCLK_GENCTRL_OE
        //|GCLK_GENCTRL_OOV
        //|GCLK_GENCTRL_GENEN
    ;

    //Wait for synchronization 
    while ( GCLK->STATUS.bit.SYNCBUSY);

    //Setup GENDIV first (should output ~1MHz)
    GCLK->GENDIV.reg = 
        GCLK_GENDIV_ID(3) 
        |GCLK_GENDIV_DIV(useDfll ? 48 : 0)
    ;
    //Wait for synchronization 
    while ( GCLK->STATUS.bit.SYNCBUSY );

    //Put GCLK 3 (Timer and RTC) on the OSC8M or DFLL
    GCLK->GENCTRL.reg = 
        GCLK_GENCTRL_ID(3)
        |GCLK_GENCTRL_SRC(useDfll ? GCLK_GENCTRL_SRC_DFLL48M_Val : GCLK_GENCTRL_SRC_OSC8M_Val)
        //|GCLK_GENCTRL_RUNSTDBY
        //|GCLK_GENCTRL_DIVSEL
        //|GCLK_GENCTRL_OE
        //|GCLK_GENCTRL_OOV
        |GCLK_GENCTRL_GENEN
    ;

    //Wait for synchronization 
    while ( GCLK->STATUS.bit.SYNCBUSY);

    s_gclk3Enabled = true;
    s_gclk3DependsOnDfll = useDfll;
    return true;
}

bool samr21Clock_enableGen4(bool useDfll)
{
    //Setup GENDIV first (should output 8MHz when DFLL is used, ~1Mhz when not)
    if(useDfll && !isDfllUsable())
    {
        return false;
    }
    
    //Disable first
    GCLK->GENCTRL.reg = 
        GCLK_GENCTRL_ID(4)
        |GCLK_GENCTRL_SRC(useDfll ? GCLK_GENCTRL_SRC_DFLL48M_Val : GCLK_GENCTRL_SRC_OSC8M_Val)
        //|GCLK_GENCTRL_RUNSTDBY
        //|GCLK_GENCTRL_DIVSEL
        //|GCLK_GENCTRL_OE
        //|GCLK_GENCTRL_OOV
        //|GCLK_GENCTRL_GENEN
    ;
    while ( GCLK->STATUS.bit.SYNCBUSY );

    GCLK->GENDIV.reg = 
        GCLK_GENDIV_ID(4) 
        |GCLK_GENDIV_DIV(useDfll ? 6 : 0)
    ;
    //Wait for synchronization 
    while ( GCLK->STATUS.bit.SYNCBUSY );


    //Put GCLK 4 (SERCOM SPI TRX) on the OSC8M or DFLL
    GCLK->GENCTRL.reg = 
        GCLK_GENCTRL_ID(4)
        |GCLK_GENCTRL_SRC(useDfll ? GCLK_GENCTRL_SRC_DFLL48M_Val : GCLK_GENCTRL_SRC_OSC8M_Val)
        //|GCLK_GENCTRL_RUNSTDBY
        //|GCLK_GENCTRL_DIVSEL
        //|GCLK_GENCTRL_OE
        //|GCLK_GENCTRL_OOV
        |GCLK_GENCTRL_GENEN
    ;

    //Wait for synchronization 
    while ( GCLK->STATUS.bit.SYNCBUSY );

    s_gclk4Enabled = true;
    s_gclk4DependsOnDfll = useDfll; 
    return true;
}

bool samr21Clock_shutdownDfllClockSource(void)
{
    if(!s_dfllClockSourceActive)
    {
        return true;
    }

    if(!isDfllShutdownSafe())
    {
        return false;
    }

    if(SYSCTRL->DFLLCTRL.bit.ENABLE)
    {
        SYSCTRL->DFLLCTRL.bit.ENABLE = 0;
        while(SYSCTRL->DFLLCTRL.bit.ENABLE);
    }

    SYSCTRL->DFLLCTRL.reg = 0x00;

    SYSCTRL->DFLLVAL.bit.COARSE = 0x00;
    SYSCTRL->DFLLVAL.bit.DIFF = 0x00;

    s_dfllClockSourceActive = false;

    return true;
}


bool samr21Clock_startupDfllClockSource(void)
{
    if(!s_gclk1Enabled)
    {
        return false;
    }

    if(isDfllUsable())
    {
        return true;
    }

    //Use GCLKGEN 1 (31.250 kHz) as Ref Freq for DFLL48M
    enablePeripheralClock(GCLK_CLKCTRL_ID_DFLL48_Val,1);

    uint32_t coarse = (*((uint32_t*)(FUSES_DFLL48M_COARSE_CAL_ADDR)) & FUSES_DFLL48M_COARSE_CAL_Msk) >> FUSES_DFLL48M_COARSE_CAL_Pos;
    uint32_t fine = (*((uint32_t*)(FUSES_DFLL48M_FINE_CAL_ADDR)) & FUSES_DFLL48M_FINE_CAL_Msk) >> FUSES_DFLL48M_FINE_CAL_Pos;

    SYSCTRL->DFLLVAL.bit.COARSE = coarse;
    SYSCTRL->DFLLVAL.bit.DIFF = fine;

    //Set Calibration Values for DFLL
    SYSCTRL->DFLLMUL.reg=
        SYSCTRL_DFLLMUL_CSTEP(0x1f / 4)// Coarse step is 31, half of the max value
        |SYSCTRL_DFLLMUL_FSTEP(0xff / 4)// Fine step is 511, half of the max value
        |SYSCTRL_DFLLMUL_MUL(1536ul) // 31.250kHz ref --> x1536 -> 48MHz
    ;

    //Enable DFLL48M clock 
    SYSCTRL->DFLLCTRL.reg =
        SYSCTRL_DFLLCTRL_WAITLOCK
        //|SYSCTRL_DFLLCTRL_BPLCKC
        //|SYSCTRL_DFLLCTRL_QLDIS
        //|SYSCTRL_DFLLCTRL_CCDIS
        //|SYSCTRL_DFLLCTRL_ONDEMAND
        //|SYSCTRL_DFLLCTRL_RUNSTDBY
        //|SYSCTRL_DFLLCTRL_USBCRM
        |SYSCTRL_DFLLCTRL_LLAW
        //|SYSCTRL_DFLLCTRL_STABLE
        |SYSCTRL_DFLLCTRL_MODE
        |SYSCTRL_DFLLCTRL_ENABLE
    ;

    while (!SYSCTRL->PCLKSR.bit.DFLLRDY || !SYSCTRL->PCLKSR.bit.DFLLLCKC || !SYSCTRL->PCLKSR.bit.DFLLLCKF);

    s_dfllClockSourceActive = true;
    return true;
}

bool samr21Clock_enableFallbackClockTree(void)
{
    //Switch All used Clocks to the internal RC Oscilator (OSC8M) 
    
    if(!SYSCTRL->OSC8M.bit.ENABLE){
        SYSCTRL->OSC8M.bit.ENABLE = 1;
    }

    //Wait for OSC8M to be availible
    while (!SYSCTRL->PCLKSR.bit.OSC8MRDY);

    bool failed = false;

    //CPU, AHB and APB (GCLK) depend on this clock Soure
    //Put it on OSC8M so we have stable clock, while configuring Clocks    
    failed |= !samr21Clock_switchGen0Source(false);

    disableAllPeripheralClocks();

    //Put them also on OSC8M, so comm with TRX still works
    failed |= !samr21Clock_enableGen3(false);
    failed |= !samr21Clock_enableGen4(false);

    //Shut down DFLL cause it is not used
    failed |= !samr21Clock_shutdownDfllClockSource();

    //Enable clocks for SPI-Comm with TRX
    enablePeripheralClock(GCLK_CLKCTRL_ID_SERCOMX_SLOW_Val, 3);
    enablePeripheralClock(GCLK_CLKCTRL_ID_SERCOM4_CORE_Val, 4);


    return !failed;
}

bool samr21Clock_enableOperatingClockTree(bool useTrxClock)
{
    if(!useTrxClock)
    {
        //Ensure that internal Oscillator is running
        if(!SYSCTRL->OSC8M.bit.ENABLE){
            SYSCTRL->OSC8M.bit.ENABLE = 1;
        }

        //Wait for OSC8M to be availible
        while (!SYSCTRL->PCLKSR.bit.OSC8MRDY);
    }

    bool failed = false;

    //Setup DFLL
    failed |= !samr21Clock_enableGen1(useTrxClock);
    failed |= !samr21Clock_startupDfllClockSource();
    
    //CPU, AHB and APB (GCLK) depend on this clock Soure
    failed |= !samr21Clock_switchGen0Source(true); //CPU Clock first
    

    //Swicht Timer to more percise Source
    failed |= !samr21Clock_enableGen3(true);

    //Switch SPI-TRX Clock to faster clock source (speeds up comm with TRX)
    failed |= !samr21Clock_enableGen4(true);

    if(failed)
    {
        samr21Clock_enableFallbackClockTree();
    }
    
    enableAllPeripheralClocks();

    return !failed;
}



void samr21Clock_init(void) 
{
    samr21Clock_enableFallbackClockTree();

#ifndef SAMR21_DONT_USE_TRX_CLOCK
    samr21Clock_enableOperatingClockTree(true);
#else
    samr21Clock_enableOperatingClockTree(false);
#endif
}
