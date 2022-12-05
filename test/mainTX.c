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
    

    uint64_t ieeeAddr = 0xA0A1A2A3A4A5A6A7;
    uint16_t shortAddr = 0xA8A9;
    uint16_t panId = 0xCAFE;
    samr21RadioSetIEEEAddr(ieeeAddr);
    samr21RadioSetShortAddr(shortAddr);
    samr21RadioSetPanID(shortAddr);
    samr21RadioChangeChannel(13);

    FrameBuffer_t tempFrame;

    tempFrame.header.frameControlField1.frameType = IEEE_802_15_4_DATA_FRAME;
    tempFrame.header.frameControlField1.securityEnabled = 0;
    tempFrame.header.frameControlField1.framePending = 0;
    tempFrame.header.frameControlField1.ackRequest = 1;
    tempFrame.header.frameControlField1.panIdCompression = 1;
    tempFrame.header.frameControlField2.sequenceNumberSuppression = 0;
    tempFrame.header.frameControlField2.informationElementsPresent = 0;
    tempFrame.header.frameControlField2.destinationAddressingMode = IEEE_802_15_4_ADDR_IEEE;
    tempFrame.header.frameControlField2.frameVersion = IEEE_802_15_4_VERSION_2006;
    tempFrame.header.frameControlField2.sourceAddressingMode = IEEE_802_15_4_ADDR_SHORT;
    tempFrame.header.sequenceNumber = 1;

    tempFrame.raw[4] =  0xFE;
    tempFrame.raw[5] =  0xCA;

    tempFrame.raw[6] =  0xB7;
    tempFrame.raw[7] =  0xB6;
    tempFrame.raw[8] =  0xB5;
    tempFrame.raw[9] =  0xB4;
    tempFrame.raw[10] =  0xB3;
    tempFrame.raw[11] =  0xB2;
    tempFrame.raw[12] =  0xB1;
    tempFrame.raw[13] =  0xB0;

    tempFrame.raw[14] =  0xA9;
    tempFrame.raw[15] =  0xA8;

    tempFrame.raw[16] =  'H';
    tempFrame.raw[17] =  'A';
    tempFrame.raw[18] =  'L';
    tempFrame.raw[19] =  'L';
    tempFrame.raw[20] =  'O';
    tempFrame.raw[21] =  ' ';
    tempFrame.raw[22] =  'W';
    tempFrame.raw[23] =  'E';
    tempFrame.raw[24] =  'L';
    tempFrame.raw[25] =  'T';
    tempFrame.raw[26] =  '!';

    tempFrame.header.lenght = 26 + IEEE_802_15_4_CRC_SIZE;
    
    uint32_t tempI = 0x0FFFF;
    char buf[15] = "Sending Frame\n\r";

    while (true)
    {
        // uint8_t RegVal[0x3F];
        // for (uint16_t i = 1; i <= 0x3D; i++) 
        // {
        //     RegVal[i] = samr21TrxReadRegister(i);
        //     tud_task(); // device task
        // }
        if(tempI>0x00FF){
            if(samr21RadioSendFrame(&tempFrame)){
                tempFrame.header.sequenceNumber++;
            }
            tempI=0;
        }
        
        if(dtrFlag && g_trxLastIrq.reg){
            tud_cdc_write(g_trxLastIrq.reg, 1);
            tud_cdc_write_flush();
        }

        samr21UsbTask();
        tempI++; 
    }
}

