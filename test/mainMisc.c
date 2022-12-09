//Author Eric Härtel @ dresden elektronik ingenieurtechnik gmbh © 2022
#include "samr21.h"
#include <stdbool.h>
#include <stdint.h>

#include "samr21Trx.h"
#include "samr21Radio.h"
#include "samr21Rtc.h"
#include "samr21NopDelay.h"
#include "samr21PowerManager.h"
#include "samr21Timer.h"
#include "samr21Nvm.h"
#include "samr21Usb.h"



void samr21DebugPortsInit(){
        PORT->Group[0].DIRSET.reg= PORT_PA06;

        //Setup Mux Settings
        PORT->Group[0].WRCONFIG.reg =
            //PORT_WRCONFIG_HWSEL
            PORT_WRCONFIG_WRPINCFG
            //|PORT_WRCONFIG_WRPMUX
            //|PORT_WRCONFIG_PMUX(MUX_PC16F_GCLK_IO1)
            //PORT_WRCONFIG_PULLEN
            //|PORT_WRCONFIG_INEN
            //|PORT_WRCONFIG_PMUXEN
            |PORT_WRCONFIG_PINMASK(PORT_PA06) //lower Halfword
        ;


        PORT->Group[0].DIRSET.reg= PORT_PA07;

        //Setup Mux Settings
        PORT->Group[0].WRCONFIG.reg =
            //PORT_WRCONFIG_HWSEL
            PORT_WRCONFIG_WRPINCFG
            //|PORT_WRCONFIG_WRPMUX
            //|PORT_WRCONFIG_PMUX(MUX_PC16F_GCLK_IO1)
            //PORT_WRCONFIG_PULLEN
            //|PORT_WRCONFIG_INEN
            //|PORT_WRCONFIG_PMUXEN
            |PORT_WRCONFIG_PINMASK(PORT_PA07) //lower Halfword
        ;

        PORT->Group[0].DIRSET.reg= PORT_PA08;

        //Setup Mux Settings
        PORT->Group[0].WRCONFIG.reg =
            //PORT_WRCONFIG_HWSEL
            PORT_WRCONFIG_WRPINCFG
            //|PORT_WRCONFIG_WRPMUX
            //|PORT_WRCONFIG_PMUX(MUX_PC16F_GCLK_IO1)
            //PORT_WRCONFIG_PULLEN
            //|PORT_WRCONFIG_INEN
            //|PORT_WRCONFIG_PMUXEN
            |PORT_WRCONFIG_PINMASK(PORT_PA08) //lower Halfword
        ;

        PORT->Group[0].DIRSET.reg= PORT_PA09;

        //Setup Mux Settings
        PORT->Group[0].WRCONFIG.reg = 
            //PORT_WRCONFIG_HWSEL
            PORT_WRCONFIG_WRPINCFG
            //|PORT_WRCONFIG_WRPMUX
            //|PORT_WRCONFIG_PMUX(MUX_PC16F_GCLK_IO1)
            //PORT_WRCONFIG_PULLEN
            //|PORT_WRCONFIG_INEN
            //|PORT_WRCONFIG_PMUXEN
            |PORT_WRCONFIG_PINMASK(PORT_PA09) //lower Halfword
        ;

        PORT->Group[0].DIRSET.reg= PORT_PA09;

        //Setup Mux Settings
        PORT->Group[0].WRCONFIG.reg = 
            //PORT_WRCONFIG_HWSEL
            PORT_WRCONFIG_WRPINCFG
            //|PORT_WRCONFIG_WRPMUX
            //|PORT_WRCONFIG_PMUX(MUX_PC16F_GCLK_IO1)
            //PORT_WRCONFIG_PULLEN
            //|PORT_WRCONFIG_INEN
            //|PORT_WRCONFIG_PMUXEN
            |PORT_WRCONFIG_PINMASK(PORT_PA09) //lower Halfword
        ;
}

extern AT86RF233_REG_IRQ_STATUS_t     g_trxLastIrq;               //from samr21trx.c

int main(int argc, char const *argv[])
{
    samr21NvmInit();
    samr21PowerManagerInit();
    samr21ClockInit();
    samr21TrxInterfaceInit();

    samr21TrxSetupMClk(0x5); //MCLK 1MHz -> 16 Mhz
    samr21ClockInitAfterTrxSetup();

    samr21TimerInit();

    samr21DebugPortsInit();
    samr21RadioInit();  

    samr21UsbInit();

    uint64_t ieeeAddr = samr21NvmGetIeeeAddr();
    uint16_t shortAddr = 0xA8A9;
    uint16_t panId = 0xCAFE;
    samr21RadioSetIeeeAddr(&ieeeAddr);
    samr21RadioSetShortAddr(&shortAddr);
    samr21RadioSetPanId(&shortAddr);
    samr21RadioChangeChannel(13);


    tempFrame.header.lenght = 26 + IEEE_802_15_4_CRC_SIZE;
    char edDone[23] = "\n\rED on Ch.:[  ] = -";
    char edFailed[23]  = "\n\rED on Ch.:[  ] FAILED";


    // Time to connect to the USB CDC DT
    for(uint32_t i = 0; i < 0xFFFFF; i++){
        samr21UsbEchoTask();
    }

    while (true)
    {   
        samr21UsbEchoTask();
    }
}

