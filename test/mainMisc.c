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
#include "samr21Aes.h"



void samr21DebugPortsInit(){

        PORT->Group[0].DIRSET.reg= PORT_PA16;

        //Setup Mux Settings
        PORT->Group[0].WRCONFIG.reg =
            PORT_WRCONFIG_HWSEL
            |PORT_WRCONFIG_WRPINCFG
            |PORT_WRCONFIG_WRPMUX
            |PORT_WRCONFIG_PMUX(MUX_PA16H_GCLK_IO2)
            //PORT_WRCONFIG_PULLEN
            //|PORT_WRCONFIG_INEN
            |PORT_WRCONFIG_PMUXEN
            |PORT_WRCONFIG_PINMASK(PORT_PA16 >> 16) //upper Halfword
        ;

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
volatile bool tempLock = false;

int main(int argc, char const *argv[])
{
    samr21NvmInit();
    samr21PowerManagerInit();
    samr21ClockTrxSrcInit();
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


    char edDone[23] = "\n\rED on Ch.:[  ] = -";
    char edFailed[23]  = "\n\rED on Ch.:[  ] FAILED";


    // Time to connect to the USB CDC DT
    for(uint32_t i = 0; i < 0xFFFF; i++){
        samr21UsbEchoTask();
    }

    uint8_t key[AES_BLOCK_SIZE] = {
        0x5a, 0x69, 0x67, 0x42, 0x65, 0x65, 0x41, 0x6c, 0x6c, 0x69, 0x61, 0x6e, 0x63, 0x65, 0x30, 0x39
    };

    uint8_t input[AES_BLOCK_SIZE] = {
        0xAA, 0xAA, 0xAA, 0xAA,
        0xAA, 0xAA, 0xAA, 0xAA,
        0xAA, 0xAA, 0xAA, 0xAA,
        0xAA, 0xAA, 0xAA, 0xAA
    };

    uint8_t output[AES_BLOCK_SIZE] = {0};
    
    samr21AesKeySetup(key);

    while (true)
    {  
        samr21Timer0Set(1500);
        tempLock=true;
        while (tempLock);

        uint8_t inout[AES_BLOCK_SIZE];
        memcpy(inout, input, AES_BLOCK_SIZE);

        PORT->Group[0].OUTSET.reg = PORT_PA06;
        samr21AesEcbEncryptBlocking(inout);
        PORT->Group[0].OUTCLR.reg = PORT_PA06;  
        
        samr21Timer1Set(500);
        tempLock=true;
        while (tempLock);
    }
}



void TCC0_Handler(){
    TCC0->INTFLAG.bit.OVF = 1;
    tempLock=false;
}

void TCC1_Handler(){
    TCC1->INTFLAG.bit.OVF = 1;
    tempLock=false;
}
