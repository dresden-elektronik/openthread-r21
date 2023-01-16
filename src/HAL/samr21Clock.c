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

//Setup GCLKGEN 1 to be sourced form At86rf233 MCLK
//Setup SERCOM4 (SPI <-> At86rf233) to use GCLKGEN 1 (MCLK) to enable synchronous Transfers
void samr21ClockTrxSrcInit(){

        //Deinit Timer and Counter first (to prevent an Error after a soft reset while modifying the clocksystem)
        samr21RtcDeinit();
        samr21TimerDeinit();
  
        //Setup GCLKGEN 0 (CPU Clock) to Use the internal OSC8M
        //This is needed so a reliable Clock for the CPU is available while Clocks are being Setup

        //Make sure OSC8M is enabled
        SYSCTRL->OSC8M.bit.ENABLE = 1;

        samr21delaySysTick(100);

        //Setup GENDIV first
        GCLK->GENDIV.reg = 
            GCLK_GENDIV_ID(0) // GCLKGEN0
            |GCLK_GENDIV_DIV(0x0)
        ;
        //Wait for synchronization 
        while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY );

        //Setup GENCTRL after
        GCLK->GENCTRL.reg = 
            GCLK_GENCTRL_ID(0) // GCLKGEN0
            |GCLK_GENCTRL_SRC(GCLK_GENCTRL_SRC_OSC8M_Val)
            |GCLK_GENCTRL_RUNSTDBY
            //|GCLK_GENCTRL_DIVSEL
#ifdef _DEBUG
            |GCLK_GENCTRL_OE
#endif
            //|GCLK_GENCTRL_OOV
            |GCLK_GENCTRL_GENEN
        ;
        while(GCLK->STATUS.bit.SYNCBUSY);

        samr21delaySysTick(100);

        GCLK->CTRL.reg = GCLK_CTRL_SWRST;

        samr21delaySysTick(100);
        //Wait for synchronization 
        while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY );


        //Setup PIN PC16 as Clockinput from MCLK from At86rf233
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

        // //Make PULLResistor -> Pulldown
        // PORT->Group[2].OUTCLR.reg= PORT_PC16;

        //Setup GENDIV
        GCLK->GENDIV.reg = 
            GCLK_GENDIV_ID(1)
            |GCLK_GENDIV_DIV(0x0)
        ;

        //Wait for synchronization 
        while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY );

        //Setup GENCTRL
        GCLK->GENCTRL.reg = 
            GCLK_GENCTRL_ID(1) // GCLKGEN1
            |GCLK_GENCTRL_SRC(GCLK_GENCTRL_SRC_GCLKIN_Val)
            |GCLK_GENCTRL_RUNSTDBY
            //|GCLK_GENCTRL_DIVSEL
#ifdef _DEBUG
            |GCLK_GENCTRL_OE
#endif
            //|GCLK_GENCTRL_OOV
            |GCLK_GENCTRL_GENEN
        ;

        //Wait for synchronization 
        while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY );

        //Setup GENDIV
        GCLK->GENDIV.reg = 
            GCLK_GENDIV_ID(2)
            |GCLK_GENDIV_DIV(16) //Use GCLKGEN 4 for RTC/Timer (16MHz -> 1Mhz (1us))
        ;

        //Wait for synchronization 
        while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY );

        //Setup GENCTRL
        GCLK->GENCTRL.reg = 
            GCLK_GENCTRL_ID(2) // GCLKGEN4
            |GCLK_GENCTRL_SRC(GCLK_GENCTRL_SRC_GCLKGEN1_Val)
            |GCLK_GENCTRL_RUNSTDBY
            //|GCLK_GENCTRL_DIVSEL
#ifdef _DEBUG
            |GCLK_GENCTRL_OE
#endif
            //|GCLK_GENCTRL_OOV
            |GCLK_GENCTRL_GENEN
        ;

        //Wait for synchronization 
        while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY );
}

//Setup GCLKGEN 3 as DFLL Ref Clock & RTC (divided down from 16MHz -> 1Mhz)
//Setup GCLKGEN 0 for DFFL48M for CPU and USB
void samr21ClockInitAfterTrxSetup(){
    
    //WA1 see R21 Datasheet ERRATA 47.1.5
    SYSCTRL->DFLLCTRL.bit.ONDEMAND = 0;
    while (!SYSCTRL->PCLKSR.bit.DFLLRDY);

    //WA1 use GCLKGEN 3 as Devider for MCLK 16MHz (if configured correctly) down to 31.250kHz 
        //Setup GENDIV first
        GCLK->GENDIV.reg = 
            GCLK_GENDIV_ID(3) // GCLKGEN3
            |GCLK_GENDIV_DIV(8)
        ;

        //Wait for synchronization 
        while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY );

        //Setup GENCTRL after
        GCLK->GENCTRL.reg = 
            GCLK_GENCTRL_ID(3) // GCLKGEN3
            |GCLK_GENCTRL_SRC(GCLK_GENCTRL_SRC_GCLKGEN1_Val)
            |GCLK_GENCTRL_RUNSTDBY
            |GCLK_GENCTRL_DIVSEL
#ifdef _DEBUG
            |GCLK_GENCTRL_OE
#endif
            |GCLK_GENCTRL_OOV
            |GCLK_GENCTRL_GENEN
        ;

        //Wait for synchronization 
        while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY );

    //Use GCLKGEN 3 as Ref Freq for DFLL48M
    GCLK->CLKCTRL.reg =
        //GCLK_CLKCTRL_WRTLOCK
        GCLK_CLKCTRL_CLKEN
        |GCLK_CLKCTRL_GEN(3) // GCLKGEN3
        |GCLK_CLKCTRL_ID(GCLK_CLKCTRL_ID_DFLL48_Val)
    ;
    //Wait for synchronization 
    while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY );

    //Enable DFLL48M clock 
    SYSCTRL->DFLLCTRL.reg =
        SYSCTRL_DFLLCTRL_WAITLOCK
        //|SYSCTRL_DFLLCTRL_BPLCKC
        |SYSCTRL_DFLLCTRL_QLDIS
        //|SYSCTRL_DFLLCTRL_CCDIS
        //|SYSCTRL_DFLLCTRL_ONDEMAND
        |SYSCTRL_DFLLCTRL_RUNSTDBY
        //|SYSCTRL_DFLLCTRL_USBCRM
        //|SYSCTRL_DFLLCTRL_LLAW
        //|SYSCTRL_DFLLCTRL_STABLE
        |SYSCTRL_DFLLCTRL_MODE
        |SYSCTRL_DFLLCTRL_ENABLE
    ;

    while (!SYSCTRL->PCLKSR.bit.DFLLRDY);

    SYSCTRL->DFLLMUL.reg=
        SYSCTRL_DFLLMUL_CSTEP(31) // Coarse step is 31, half of the max value
        |SYSCTRL_DFLLMUL_FSTEP(511) // Fine step is 511, half of the max value
        |SYSCTRL_DFLLMUL_MUL(1536) // 31.250kHz ref --> x1536 -> 48MHz
    ;

    while (!SYSCTRL->PCLKSR.bit.DFLLRDY || !SYSCTRL->PCLKSR.bit.DFLLLCKC || !SYSCTRL->PCLKSR.bit.DFLLLCKF);

    //Setup GCLKGEN 0 (CPU Clock) from DFLL48M
        //Setup GENDIV first
        GCLK->GENDIV.reg = 
            GCLK_GENDIV_ID(0) // GCLKGEN0
            |GCLK_GENDIV_DIV(0x0)
        ;

        //Wait for synchronization 
        while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY );

        //Setup GENCTRL after
        GCLK->GENCTRL.reg = 
            GCLK_GENCTRL_ID(0) // GCLKGEN0
            |GCLK_GENCTRL_SRC(GCLK_GENCTRL_SRC_DFLL48M_Val)
            |GCLK_GENCTRL_RUNSTDBY
            //|GCLK_GENCTRL_DIVSEL
#ifdef _DEBUG
            |GCLK_GENCTRL_OE
#endif
            //|GCLK_GENCTRL_OOV
            |GCLK_GENCTRL_GENEN
        ;

        //Wait for synchronization 
        while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY );
}
