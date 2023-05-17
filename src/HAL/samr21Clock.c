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


static void samr21ClockInitExternalClock(void)
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

    //Setup GENDIV
    GCLK->GENDIV.reg = 
        GCLK_GENDIV_ID(1)
        |GCLK_GENDIV_DIV(4) // 1MHz (Default MClk from At86rf233) --> 31.250 Khz (Max DFLL48M Reference clock frequency)
    ;

    //Wait for synchronization 
    while ( GCLK->STATUS.bit.SYNCBUSY );

    //Setup GENCTRL
    GCLK->GENCTRL.reg = 
        GCLK_GENCTRL_ID(1) // GCLKGEN1
        |GCLK_GENCTRL_SRC(GCLK_GENCTRL_SRC_GCLKIN_Val)
        |GCLK_GENCTRL_RUNSTDBY
        |GCLK_GENCTRL_DIVSEL
        //|GCLK_GENCTRL_OOV
        |GCLK_GENCTRL_GENEN
    ;

    //Wait for synchronization 
    while ( GCLK->STATUS.bit.SYNCBUSY );
}

static void samr21ClockDisableDfllClockGen(void)
{
    if(SYSCTRL->DFLLCTRL.bit.ENABLE)
    {
        SYSCTRL->DFLLCTRL.bit.ENABLE = 0;
        while(SYSCTRL->DFLLCTRL.bit.ENABLE);
    }

    SYSCTRL->DFLLCTRL.reg = 0x00;

    SYSCTRL->DFLLVAL.bit.COARSE = 0x00;
    SYSCTRL->DFLLVAL.bit.DIFF = 0x00;

}

static void samr21ClockInitDfllClockGen(void)
{
    //WA1 see R21 Datasheet ERRATA 47.1.5
    SYSCTRL->DFLLCTRL.bit.ONDEMAND = 0;
    while (!SYSCTRL->PCLKSR.bit.DFLLRDY);

     //Use GCLKGEN 1 (31.250 kHz) as Ref Freq for DFLL48M
    GCLK->CLKCTRL.reg =
        //GCLK_CLKCTRL_WRTLOCK
        GCLK_CLKCTRL_CLKEN
        |GCLK_CLKCTRL_GEN(1)
        |GCLK_CLKCTRL_ID(GCLK_CLKCTRL_ID_DFLL48_Val)
    ;

    //Wait for synchronization 
    while ( GCLK->STATUS.bit.SYNCBUSY );

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
        |SYSCTRL_DFLLCTRL_RUNSTDBY
        //|SYSCTRL_DFLLCTRL_USBCRM
        //|SYSCTRL_DFLLCTRL_LLAW
        //|SYSCTRL_DFLLCTRL_STABLE
        |SYSCTRL_DFLLCTRL_MODE
        |SYSCTRL_DFLLCTRL_ENABLE
    ;

    while (!SYSCTRL->PCLKSR.bit.DFLLRDY || !SYSCTRL->PCLKSR.bit.DFLLLCKC || !SYSCTRL->PCLKSR.bit.DFLLLCKF);
}

static void samr21ClockSwitchCpuClockToInternalOscillator(void)
{
    //Check that internal Oscillator is running
    if(!SYSCTRL->OSC8M.bit.ENABLE){
        SYSCTRL->OSC8M.bit.ENABLE = 1;
    }

    while (!SYSCTRL->PCLKSR.bit.OSC8MRDY);
    

    //Setup GENDIV first
    GCLK->GENDIV.reg = 
        GCLK_GENDIV_ID(0) // GCLKGEN0
        |GCLK_GENDIV_DIV(0x0)
    ;

    //Wait for synchronization 
    while ( GCLK->STATUS.bit.SYNCBUSY );

    //Setup GENCTRL after
    GCLK->GENCTRL.reg = 
        GCLK_GENCTRL_ID(0) // GCLKGEN0
        |GCLK_GENCTRL_SRC(GCLK_GENCTRL_SRC_OSC8M_Val)
        |GCLK_GENCTRL_RUNSTDBY
        //|GCLK_GENCTRL_DIVSEL
#ifdef _CLOCK_DEBUG_OUTPUT
        |GCLK_GENCTRL_OE
#endif
        //|GCLK_GENCTRL_OOV
        |GCLK_GENCTRL_GENEN
    ;

     //Wait for synchronization 
    while ( GCLK->STATUS.bit.SYNCBUSY );
}

static void samr21ClockSwitchCpuClockToDfll(void)
{
    //Setup GCLKGEN 0 (CPU Clock)to be sourced from DFLL48M

    //Setup GENDIV first
    GCLK->GENDIV.reg = 
        GCLK_GENDIV_ID(0) // GCLKGEN0
        |GCLK_GENDIV_DIV(0x0)
    ;

    //Wait for synchronization 
    while ( GCLK->STATUS.bit.SYNCBUSY );

    //Setup GENCTRL after
    GCLK->GENCTRL.reg = 
        GCLK_GENCTRL_ID(0) // GCLKGEN0
        |GCLK_GENCTRL_SRC(GCLK_GENCTRL_SRC_DFLL48M_Val)
        |GCLK_GENCTRL_RUNSTDBY
        //|GCLK_GENCTRL_DIVSEL
#ifdef _CLOCK_DEBUG_OUTPUT
        |GCLK_GENCTRL_OE
#endif
        //|GCLK_GENCTRL_OOV
        |GCLK_GENCTRL_GENEN
    ;

     //Wait for synchronization 
    while ( GCLK->STATUS.bit.SYNCBUSY );
}

static void samr21ClockInitPeriphialClocks(void){

    //Setup GCLKGEN 2 as a 8MHz Clock for the SPI-Interface to the At86Rf233 (SERCOM4)
    //Setup GENDIV first
    GCLK->GENDIV.reg = 
        GCLK_GENDIV_ID(2) 
        |GCLK_GENDIV_DIV(6ul) // 48MHz / 6 = 8MHz
    ;

    //Setup GENCTRL after
    GCLK->GENCTRL.reg = 
        GCLK_GENCTRL_ID(2) // GCLKGEN2
        |GCLK_GENCTRL_SRC(GCLK_GENCTRL_SRC_DFLL48M_Val)
        |GCLK_GENCTRL_RUNSTDBY
        //|GCLK_GENCTRL_DIVSEL
#ifdef _CLOCK_DEBUG_OUTPUT
        |GCLK_GENCTRL_OE
#endif
        //|GCLK_GENCTRL_OOV
        |GCLK_GENCTRL_GENEN
    ;


    //Setup GCLKGEN 3 as a 1Mhz Clock for Timer and RTC
    //Setup GENDIV first
    GCLK->GENDIV.reg = 
        GCLK_GENDIV_ID(3) // GCLKGEN0
        |GCLK_GENDIV_DIV(48ul) // 48MHz / 48 = 1MHz
    ;
    while(GCLK->STATUS.bit.SYNCBUSY);

    GCLK->GENCTRL.reg = 
        GCLK_GENCTRL_ID(3) 
        |GCLK_GENCTRL_SRC(GCLK_GENCTRL_SRC_DFLL48M_Val)
        |GCLK_GENCTRL_RUNSTDBY
        //|GCLK_GENCTRL_DIVSEL
#ifdef _CLOCK_DEBUG_OUTPUT
        |GCLK_GENCTRL_OE
#endif
        //|GCLK_GENCTRL_OOV
        |GCLK_GENCTRL_GENEN
    ;

    while(GCLK->STATUS.bit.SYNCBUSY);
}


void samr21ClockInit(void)
{
#ifdef _CLOCK_DEBUG_OUTPUT

    //Make Output
        PORT->Group[0].DIRSET.reg= PORT_PA17;
        PORT->Group[0].DIRSET.reg= PORT_PA16;
        PORT->Group[0].DIRSET.reg= PORT_PA14;

        //Setup Mux Settings
        PORT->Group[0].WRCONFIG.reg =
            PORT_WRCONFIG_HWSEL
            |PORT_WRCONFIG_WRPINCFG
            |PORT_WRCONFIG_WRPMUX
            |PORT_WRCONFIG_PMUX(MUX_PA17H_GCLK_IO3)
            //|PORT_WRCONFIG_INEN
            //|PORT_WRCONFIG_PULLEN
            |PORT_WRCONFIG_PMUXEN
            |PORT_WRCONFIG_PINMASK(PORT_PA17 >> 16) //upper Halfword
        ;

        PORT->Group[0].WRCONFIG.reg =
            PORT_WRCONFIG_HWSEL
            |PORT_WRCONFIG_WRPINCFG
            |PORT_WRCONFIG_WRPMUX
            |PORT_WRCONFIG_PMUX(MUX_PA16H_GCLK_IO2)
            //|PORT_WRCONFIG_INEN
            //|PORT_WRCONFIG_PULLEN
            |PORT_WRCONFIG_PMUXEN
            |PORT_WRCONFIG_PINMASK(PORT_PA16 >> 16) //upper Halfword
        ;

        PORT->Group[0].WRCONFIG.reg =
            //PORT_WRCONFIG_HWSEL
            PORT_WRCONFIG_WRPINCFG
            |PORT_WRCONFIG_WRPMUX
            |PORT_WRCONFIG_PMUX(MUX_PA14H_GCLK_IO0)
            //|PORT_WRCONFIG_INEN
            //|PORT_WRCONFIG_PULLEN
            |PORT_WRCONFIG_PMUXEN
            |PORT_WRCONFIG_PINMASK(PORT_PA14) //upper Halfword
        ;

#endif


    //samr21ClockReset();
    samr21ClockSwitchCpuClockToInternalOscillator();
    samr21ClockDisableDfllClockGen();

    samr21ClockInitExternalClock();
    samr21ClockInitDfllClockGen();
    samr21ClockInitPeriphialClocks();

    samr21ClockSwitchCpuClockToDfll();
}
