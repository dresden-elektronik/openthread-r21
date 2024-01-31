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


static void clock_enableGClkGen(uint8_t gClkGen, uint8_t clkSrc, uint32_t divFactor )
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

    //Setup Divider Factor
    GCLK->GENDIV.reg = 
        GCLK_GENDIV_ID(gClkGen) 
        |GCLK_GENDIV_DIV(divFactor)
    ;
    //Wait for synchronization 
    while ( GCLK->STATUS.bit.SYNCBUSY );


    //Enable again
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

static void clock_disablePeripheralClock(uint8_t peripheralClock)
{
    while(GCLK->STATUS.bit.SYNCBUSY);
    __disable_irq();
   	
    //Select the requested peripheral Clock
	*((uint8_t*)&GCLK->CLKCTRL.reg) = peripheralClock;

	//Sanity check for WRTLOCK 
	while(GCLK->CLKCTRL.bit.WRTLOCK);

	// Switch to known-working source so that the channel can be disabled 
	uint32_t prev_gen_id = GCLK->CLKCTRL.bit.GEN;
	GCLK->CLKCTRL.bit.GEN = 0;

	// Disable the peripheral Clock
	GCLK->CLKCTRL.reg &= ~GCLK_CLKCTRL_CLKEN;
	
    // Wait for clock to become disabled
	while (GCLK->CLKCTRL.reg & GCLK_CLKCTRL_CLKEN);

    __enable_irq();
}


static void clock_enablePeripheralClock(uint8_t peripheralClock, uint8_t gClkGen)
{
    clock_disablePeripheralClock(peripheralClock);

    __disable_irq();

	//Write the new configuration
	GCLK->CLKCTRL.reg =
        GCLK_CLKCTRL_ID(peripheralClock)
        | GCLK_CLKCTRL_GEN(gClkGen)
        | GCLK_CLKCTRL_CLKEN
    ;

    __enable_irq();
}

static void clock_enableAllPeripheralClocks(void)
{
    //CPU Clock (48Mhz)
    clock_enablePeripheralClock(GCLK_CLKCTRL_ID_EIC_Val,0);
    clock_enablePeripheralClock(GCLK_CLKCTRL_ID_USB_Val,0);

    //Timer Clock (1Mhz)
    clock_enablePeripheralClock(GCLK_CLKCTRL_ID_TCC0_TCC1_Val, 3);
    clock_enablePeripheralClock(GCLK_CLKCTRL_ID_TCC2_TC3_Val, 3);
    clock_enablePeripheralClock(GCLK_CLKCTRL_ID_TC4_TC5_Val, 3);
    clock_enablePeripheralClock(GCLK_CLKCTRL_ID_RTC_Val,3);

    //Sercom Slow Clock (1 MHz, DebugUart and APB Interface)
    clock_enablePeripheralClock(GCLK_CLKCTRL_ID_SERCOMX_SLOW_Val, 3);
    clock_enablePeripheralClock(GCLK_CLKCTRL_ID_SERCOM2_CORE_Val, 3);

    //Sercom Fast Clock (12 MHz TRX SPI)
    clock_enablePeripheralClock(GCLK_CLKCTRL_ID_SERCOM4_CORE_Val, 4);
}

static void clock_disableAllPeripheralClocks(void)
{
    clock_disablePeripheralClock(GCLK_CLKCTRL_ID_EIC_Val);
    clock_disablePeripheralClock(GCLK_CLKCTRL_ID_USB_Val);
    clock_disablePeripheralClock(GCLK_CLKCTRL_ID_TCC0_TCC1_Val);
    clock_disablePeripheralClock(GCLK_CLKCTRL_ID_TCC2_TC3_Val);
    clock_disablePeripheralClock(GCLK_CLKCTRL_ID_TC4_TC5_Val);
    clock_disablePeripheralClock(GCLK_CLKCTRL_ID_RTC_Val);
    clock_disablePeripheralClock(GCLK_CLKCTRL_ID_SERCOM2_CORE_Val);
    clock_disablePeripheralClock(GCLK_CLKCTRL_ID_SERCOM4_CORE_Val);
    clock_disablePeripheralClock(GCLK_CLKCTRL_ID_SERCOMX_SLOW_Val);
}

bool clock_switchGClkGen0Source(uint8_t clkSrc)
{
    GCLK->GENCTRL.bit.ID = 0;
    GCLK->GENCTRL.bit.SRC = clkSrc;

    //Wait for synchronization 
    while ( GCLK->STATUS.bit.SYNCBUSY || !GCLK->GENCTRL.bit.GENEN);

    return true;
}

static void clock_setupGClkIn(void)
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


void clock_shutdownDfllClockSource(void)
{
    if(SYSCTRL->DFLLCTRL.bit.ENABLE)
    {
        SYSCTRL->DFLLCTRL.reg = 0;
        while(SYSCTRL->DFLLCTRL.bit.ENABLE);
    }
}


void clock_startupDfllClockSource(bool useUsbClock, uint32_t refMultiplyFactor)
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

    while (!SYSCTRL->PCLKSR.bit.DFLLRDY || !SYSCTRL->PCLKSR.bit.DFLLLCKC || !SYSCTRL->PCLKSR.bit.DFLLLCKF);
}

void samr21Clock_enableFallbackClockTree(void)
{
    //RTC-BUG (PART 1)
    //
    // The RTC GCLK-Settings are Reset-Persitent.
    // This means it may still be set to GCLKGEN = 3 (which is disabled after a Reset) 
    // This will cause the GCLK SWRST to never finish.
    // While SWRST is ongoing GCLKs can't be modifyed, so we force disable the RTC before.
    // In case the SWRST Looks up we just force a NVIC-Reset and boot up again with the RTC now disabled
    clock_disablePeripheralClock(GCLK_CLKCTRL_ID_RTC_Val);
    
    //Start a Restart
    GCLK->CTRL.bit.SWRST = 1;

    uint32_t timeout = 0;
    while(GCLK->CTRL.bit.SWRST || GCLK->STATUS.bit.SYNCBUSY)
    {
        if(timeout++ > 10000)
        {
            //RTC-BUG (PART 2)
            //
            // The SWRST is looked up.
            // NVIC-Reset here, so after the reboot the RTC should be disabled
            NVIC_SystemReset();
        }
    }
    
    if(!SYSCTRL->OSC8M.bit.ENABLE){
        SYSCTRL->OSC8M.bit.ENABLE = 1;
    }

    //Wait for OSC8M to be availible
    while (!SYSCTRL->PCLKSR.bit.OSC8MRDY);

    clock_disableAllPeripheralClocks();

    //Enable clocks for SPI-Comm with TRX
    clock_enablePeripheralClock(GCLK_CLKCTRL_ID_SERCOMX_SLOW_Val, 0);
    clock_enablePeripheralClock(GCLK_CLKCTRL_ID_SERCOM4_CORE_Val, 0);

    //Switch CPU to OSC8M
    clock_switchGClkGen0Source(GCLK_GENCTRL_SRC_OSC8M_Val);

}

void samr21Clock_enableOperatingClockTree()
{
    //Shutdown the DFLL first
    clock_shutdownDfllClockSource();

#if defined(SAMR21_USE_USB_CLOCK) && (SAMR21_USE_USB_CLOCK > 0)
    //Use The Clock Reference coming form the SOF USB Signal
    clock_startupDfllClockSource(true,0);
#else
    //Setup MCLK of ATRF233 as Reference to dfll 
    clock_setupGClkIn();
    clock_enableGClkGen(1,GCLK_GENCTRL_SRC_GCLKIN_Val,32);
    clock_enablePeripheralClock(GCLK_CLKCTRL_ID_DFLL48_Val, 1);

    //Use The Clock Reference coming form the  AT86RF233
    clock_startupDfllClockSource(false,1536);
#endif
    
    //Enable clocks for Peripherals 
    clock_enableGClkGen(3,GCLK_GENCTRL_SRC_DFLL48M_Val,48); //   48Mhz / 48 = 1Mhz (used as 1us Timer)
    clock_enableGClkGen(4,GCLK_GENCTRL_SRC_DFLL48M_Val,4); //     48Mhz / 4 = 12MHz (used for fast TRX SPI-Communication)

    //Switch CPU to DFLL
    //CPU, AHB and APB (GCLK) depend on this clock Soure
    clock_switchGClkGen0Source(GCLK_GENCTRL_SRC_DFLL48M_Val);
    
    clock_enableAllPeripheralClocks();
}