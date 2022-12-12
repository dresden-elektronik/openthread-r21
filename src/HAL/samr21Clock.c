//Author Eric Härtel @ dresden elektronik ingenieurtechnik gmbh © 2022
//Inspired by https://github.com/arduino/ArduinoCore-samd

#include "samr21Clock.h"

uint32_t g_currentTrxClkCycle_ns = 1000; //1MHZ, used as external var
uint32_t g_currentSpiClkCycle_ns = 2000; //500kHZ, used as external var 
uint32_t g_currentCpuClkCycle_ns = 1000; //1MHZ, used as external var 


//Setup GCLKGEN 1 to be sourced form At86rf233 MCLK
//Setup SERCOM4 (SPI <-> At86rf233) to use GCLKGEN 1 (MCLK) to enable synchronous Transfers
void samr21ClockTrxSrcInit(){

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

        //Use GCLKGEN1 as core Clock for SPI At86rf233 (SERCOM4)
        GCLK->CLKCTRL.reg =
            //GCLK_CLKCTRL_WRTLOCK
            GCLK_CLKCTRL_CLKEN
            |GCLK_CLKCTRL_GEN(1) // GCLKGEN1
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

        //Use GCLKGEN 4 (1MHz) for RTC 
        GCLK->CLKCTRL.reg =
            //GCLK_CLKCTRL_WRTLOCK
            GCLK_CLKCTRL_CLKEN
            |GCLK_CLKCTRL_GEN(2) // GCLKGEN2
            |GCLK_CLKCTRL_ID(GCLK_CLKCTRL_ID_RTC_Val)
        ;
        //Wait for synchronization 
        while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY );

        //Use GCLKGEN 4 (1MHz) for TCC0 / TCC1
        GCLK->CLKCTRL.reg =
            //GCLK_CLKCTRL_WRTLOCK
            GCLK_CLKCTRL_CLKEN
            |GCLK_CLKCTRL_GEN(2) // GCLKGEN1
            |GCLK_CLKCTRL_ID(GCLK_CLKCTRL_ID_TCC0_TCC1_Val)
        ;
        //Wait for synchronization 
        while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY );

        //Use GCLKGEN 4 (1MHz) for TCC2 / TC3 
        GCLK->CLKCTRL.reg =
            //GCLK_CLKCTRL_WRTLOCK
            GCLK_CLKCTRL_CLKEN
            |GCLK_CLKCTRL_GEN(2) // GCLKGEN1
            |GCLK_CLKCTRL_ID(GCLK_CLKCTRL_ID_TCC2_TC3_Val)
        ;
        //Wait for synchronization 
        while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY );

        //Use GCLKGEN 4 (1MHz) for TC4 / TC5
        GCLK->CLKCTRL.reg =
            //GCLK_CLKCTRL_WRTLOCK
            GCLK_CLKCTRL_CLKEN
            |GCLK_CLKCTRL_GEN(2) // GCLKGEN1
            |GCLK_CLKCTRL_ID(GCLK_CLKCTRL_ID_TC4_TC5_Val)
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
        g_currentCpuClkCycle_ns = 21; //48Mhz = 20.833ns 

    //Use GCLKGEN0 as Ref Freq for USB
        GCLK->CLKCTRL.reg =
            //GCLK_CLKCTRL_WRTLOCK
            GCLK_CLKCTRL_CLKEN
            |GCLK_CLKCTRL_GEN(0) // GCLKGEN0
            |GCLK_CLKCTRL_ID(GCLK_CLKCTRL_ID_USB_Val)
        ;
        //Wait for synchronization 
        while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY );

    //Use GCLKGEN0 as core Clock for EIC (At86rf233, IRQ_Detect)
        GCLK->CLKCTRL.reg =
            //GCLK_CLKCTRL_WRTLOCK
            GCLK_CLKCTRL_CLKEN
            |GCLK_CLKCTRL_GEN(0) // GCLKGEN1
            |GCLK_CLKCTRL_ID(GCLK_CLKCTRL_ID_EIC_Val)
        ;
        //Wait for synchronization 
        while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY );  
}
