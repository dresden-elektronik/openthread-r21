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
    samr21ClockTrxSrcInit();
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
    samr21RadioSetIeeeAddr(&ieeeAddr);
    samr21RadioSetShortAddr(&shortAddr);
    samr21RadioSetPanId(&shortAddr);
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
    char edDone[23] = "\n\rED on Ch.:[  ] = -";
    char edFailed[23]  = "\n\rED on Ch.:[  ] FAILED";

    uint8_t curEdChannel = 13;

    // Time to connect to the USB CDC DT
    for(uint32_t i = 0; i < 0xFFFFF; i++){
        samr21UsbEchoTask();
    }

    samr21RadioStartEnergyDetection(curEdChannel, 50);

    while (true)
    {   
        if(samr21RadioGetNextFinishedJobBuffer()->jobState == RADIO_JOB_STATE_ED_DONE){
            edDone[13] =( curEdChannel / 10 ) + 48;
            edDone[14] =( curEdChannel % 10 ) + 48;

            uint8_t result = abs(AT86RF233_RSSI_BASE_VAL) - samr21RadioGetNextFinishedJobBuffer()->measuredEngeryLevel;

            samr21RadioGetNextFinishedJobBuffer()->jobState = RADIO_JOB_STATE_IDLE;

            edDone[20] = (result / 100) + 48;;
            edDone[21] = ((result % 100) / 10) + 48;
            edDone[22] = ((result % 100) % 10) + 48;

            tud_cdc_write(edDone, 23);
            tud_cdc_write_flush();

            if(++curEdChannel > 26){
                char buf[4] = "\n\r\n\r";
                curEdChannel = 11;
                tud_cdc_write(buf, 4);
            }

            samr21RadioStartEnergyDetection(curEdChannel, 50);
        }

        if(samr21RadioGetNextFinishedJobBuffer()->jobState == RADIO_JOB_STATE_ED_FAILED){
            edFailed[13] =( curEdChannel / 10 ) + 48;
            edFailed[14] =( curEdChannel % 10 ) + 48;

             samr21RadioGetNextFinishedJobBuffer()->jobState = RADIO_JOB_STATE_IDLE;

            tud_cdc_write(edFailed, 23);
            tud_cdc_write_flush();

            samr21RadioStartEnergyDetection(curEdChannel, 50);
        }

        samr21UsbEchoTask();
    }
}

