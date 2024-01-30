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


static void enableGClkGen(uint8_t gClkGen, uint8_t clkSrc, uint32_t divFactor )
{
    //Disable first
    GCLK->GENCTRL.reg = 
        GCLK_GENCTRL_ID(gClkGen)
        //|GCLK_GENCTRL_SRC()
        //|GCLK_GENCTRL_RUNSTDBY
        //|GCLK_GENCTRL_DIVSEL
        //|GCLK_GENCTRL_OE
        //|GCLK_GENCTRL_OOV
        //|GCLK_GENCTRL_GENEN
    ;
    while ( GCLK->STATUS.bit.SYNCBUSY || GCLK->GENCTRL.bit.GENEN);

    GCLK->GENDIV.reg = 
        GCLK_GENDIV_ID(gClkGen) 
        |GCLK_GENDIV_DIV(divFactor)
    ;
    //Wait for synchronization 
    while ( GCLK->STATUS.bit.SYNCBUSY );


    GCLK->GENCTRL.reg = 
        GCLK_GENCTRL_ID(gClkGen)
        |GCLK_GENCTRL_SRC(clkSrc)
        //|GCLK_GENCTRL_RUNSTDBY
        //|GCLK_GENCTRL_DIVSEL
#if ( defined(CLOCK_DEBUG_PIN_OUTPUT) && ( CLOCK_DEBUG_PIN_OUTPUT > 0 ) )
        |GCLK_GENCTRL_OE
#else   
        //|GCLK_GENCTRL_OE
#endif
        //|GCLK_GENCTRL_OOV
        |GCLK_GENCTRL_GENEN
    ;

    //Wait for synchronization 
    while ( GCLK->STATUS.bit.SYNCBUSY || !GCLK->GENCTRL.bit.GENEN);
}

static void disablePeripheralClock(uint8_t peripheralClock)
{
    while(GCLK->STATUS.bit.SYNCBUSY);
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
	//GCLK->CLKCTRL.bit.GEN = prev_gen_id; 
    __enable_irq();
}


static void enablePeripheralClock(uint8_t peripheralClock, uint8_t gClkGen)
{
    disablePeripheralClock(peripheralClock);

    __disable_irq();

	/* Write the new configuration */
	GCLK->CLKCTRL.reg =
        GCLK_CLKCTRL_ID(peripheralClock)
        | GCLK_CLKCTRL_GEN(gClkGen)
        | GCLK_CLKCTRL_CLKEN
    ;

    __enable_irq();
}

static void enableAllPeripheralClocks(void)
{
    //CPU Clock (48Mhz)
    enablePeripheralClock(GCLK_CLKCTRL_ID_EIC_Val,0);
    enablePeripheralClock(GCLK_CLKCTRL_ID_USB_Val,0);

    //Timer Clock (1Mhz)
    enablePeripheralClock(GCLK_CLKCTRL_ID_TCC0_TCC1_Val, 3);
    enablePeripheralClock(GCLK_CLKCTRL_ID_TCC2_TC3_Val, 3);
    enablePeripheralClock(GCLK_CLKCTRL_ID_TC4_TC5_Val, 3);
    enablePeripheralClock(GCLK_CLKCTRL_ID_RTC_Val,3);

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
    disablePeripheralClock(GCLK_CLKCTRL_ID_TCC0_TCC1_Val);
    disablePeripheralClock(GCLK_CLKCTRL_ID_TCC2_TC3_Val);
    disablePeripheralClock(GCLK_CLKCTRL_ID_TC4_TC5_Val);
    disablePeripheralClock(GCLK_CLKCTRL_ID_RTC_Val);
    disablePeripheralClock(GCLK_CLKCTRL_ID_SERCOM2_CORE_Val);
    disablePeripheralClock(GCLK_CLKCTRL_ID_SERCOM4_CORE_Val);
    disablePeripheralClock(GCLK_CLKCTRL_ID_SERCOMX_SLOW_Val);
}

bool switchGClkGen0Source(uint8_t clkSrc)
{
    GCLK->GENCTRL.bit.ID = 0;
    GCLK->GENCTRL.bit.SRC = clkSrc;

    //Wait for synchronization 
    while ( GCLK->STATUS.bit.SYNCBUSY || !GCLK->GENCTRL.bit.GENEN);

    return true;
}

static void setupGClkIn(void)
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


void shutdownDfllClockSource(void)
{
    if(SYSCTRL->DFLLCTRL.bit.ENABLE)
    {
        SYSCTRL->DFLLCTRL.reg = 0;
        while(SYSCTRL->DFLLCTRL.bit.ENABLE);
    }

    return true;
}


void startupDfllClockSource(bool useUsbClock, uint32_t refMultiplyFactor)
{
    uint32_t coarse = (*((uint32_t*)(FUSES_DFLL48M_COARSE_CAL_ADDR)) & FUSES_DFLL48M_COARSE_CAL_Msk) >> FUSES_DFLL48M_COARSE_CAL_Pos;
    uint32_t fine = (*((uint32_t*)(FUSES_DFLL48M_FINE_CAL_ADDR)) & FUSES_DFLL48M_FINE_CAL_Msk) >> FUSES_DFLL48M_FINE_CAL_Pos;

    //Errata 9905
    /* Disable ONDEMAND mode while writing configurations */
	SYSCTRL->DFLLCTRL.reg = SYSCTRL_DFLLCTRL_ENABLE;
    while(!SYSCTRL->DFLLCTRL.bit.ENABLE && !SYSCTRL->PCLKSR.bit.DFLLRDY );

	SYSCTRL->DFLLMUL.reg = 0;
	SYSCTRL->DFLLVAL.reg = 0;

	/* Write full configuration to DFLL control register */
	SYSCTRL->DFLLCTRL.reg = 0;
    while( SYSCTRL->DFLLCTRL.bit.ENABLE );

    //Set Calibration Values for DFLL
    SYSCTRL->DFLLMUL.reg=
        SYSCTRL_DFLLMUL_CSTEP(0x1f / 4)// Coarse step is 31, half of the max value
        |SYSCTRL_DFLLMUL_FSTEP(0xff / 4)// Fine step is 511, half of the max value
        |SYSCTRL_DFLLMUL_MUL(useUsbClock ?  0xBB80 : refMultiplyFactor ) // 31.250kHz ref --> x1536 -> 48MHz
    ;



    if(useUsbClock)
    {
        SYSCTRL->DFLLCTRL.reg |=
            SYSCTRL_DFLLCTRL_WAITLOCK
            //|SYSCTRL_DFLLCTRL_BPLCKC
            //|SYSCTRL_DFLLCTRL_QLDIS
            |SYSCTRL_DFLLCTRL_CCDIS
            //|SYSCTRL_DFLLCTRL_ONDEMAND
            //|SYSCTRL_DFLLCTRL_RUNSTDBY
            |SYSCTRL_DFLLCTRL_USBCRM
            //|SYSCTRL_DFLLCTRL_LLAW
            //|SYSCTRL_DFLLCTRL_STABLE
            |SYSCTRL_DFLLCTRL_MODE
            |SYSCTRL_DFLLCTRL_ENABLE
        ;
    }
    else
    {
        SYSCTRL->DFLLCTRL.reg |=
            SYSCTRL_DFLLCTRL_WAITLOCK
            //|SYSCTRL_DFLLCTRL_BPLCKC
            |SYSCTRL_DFLLCTRL_QLDIS
            //|SYSCTRL_DFLLCTRL_CCDIS
            //|SYSCTRL_DFLLCTRL_ONDEMAND
            //|SYSCTRL_DFLLCTRL_RUNSTDBY
            //|SYSCTRL_DFLLCTRL_USBCRM
            //|SYSCTRL_DFLLCTRL_LLAW
            //|SYSCTRL_DFLLCTRL_STABLE
            |SYSCTRL_DFLLCTRL_MODE
            |SYSCTRL_DFLLCTRL_ENABLE
        ;
    }

    while (!SYSCTRL->PCLKSR.bit.DFLLRDY && !SYSCTRL->PCLKSR.bit.DFLLLCKC && !SYSCTRL->PCLKSR.bit.DFLLLCKF);
}

void samr21Clock_enableFallbackClockTree(void)
{
    //Disable RTC cause clocksettings are reset persistent
    disablePeripheralClock(GCLK_CLKCTRL_ID_RTC_Val);
    
    //Start a Restart
    GCLK->CTRL.bit.SWRST = 1;

    uint32_t timeout = 0;
    while(GCLK->CTRL.bit.SWRST || GCLK->STATUS.bit.SYNCBUSY)
    {
        if(timeout++ > 10000)
        {
            //If The RTC was active before the clock system is Soft locked
            NVIC_SystemReset();
        }
    }
    
    if(!SYSCTRL->OSC8M.bit.ENABLE){
        SYSCTRL->OSC8M.bit.ENABLE = 1;
    }

    //Wait for OSC8M to be availible
    while (!SYSCTRL->PCLKSR.bit.OSC8MRDY);

    disableAllPeripheralClocks();

    //Enable clocks for SPI-Comm with TRX
    enablePeripheralClock(GCLK_CLKCTRL_ID_SERCOMX_SLOW_Val, 0);
    enablePeripheralClock(GCLK_CLKCTRL_ID_SERCOM4_CORE_Val, 0);

    //Switch CPU to OSC8M
    switchGClkGen0Source(GCLK_GENCTRL_SRC_OSC8M_Val);

}

void samr21Clock_enableOperatingClockTree()
{
    //Setup DFLL
    shutdownDfllClockSource();

#ifdef SAMR21_USE_USB_CLOCK
    startupDfllClockSource(true,0);
#else
    //Setup MCLK of ATRF233 as Reference to dfll 
    setupGClkIn();
    enableGClkGen(1,GCLK_GENCTRL_SRC_GCLKIN_Val,32);
    enablePeripheralClock(GCLK_CLKCTRL_ID_DFLL48_Val, 1);

    startupDfllClockSource(false,1536);
#endif
    
    enableGClkGen(3,GCLK_GENCTRL_SRC_DFLL48M_Val,48); //    1Mhz (1us Timer)
    enableGClkGen(4,GCLK_GENCTRL_SRC_DFLL48M_Val,6); //     8Mhz (fast TRX Comm)

    //CPU, AHB and APB (GCLK) depend on this clock Soure
    switchGClkGen0Source(GCLK_GENCTRL_SRC_DFLL48M_Val);
    
    enableAllPeripheralClocks();
}



void samr21Clock_init(void) 
{
    samr21Clock_enableFallbackClockTree();
    samr21Clock_enableOperatingClockTree();
}
