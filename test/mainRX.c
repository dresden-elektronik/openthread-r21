//Author Eric Härtel @ dresden elektronik ingenieurtechnik gmbh © 2022
#include "samr21.h"
#include <stdbool.h>
#include <stdint.h>

#include "samr21Trx.h"
#include "samr21Radio.h"
#include "samr21Rtc.h"
#include "samr21NopDelay.h"
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

        PORT->Group[0].DIRSET.reg= PORT_PA15;

        //Setup Mux Settings
        PORT->Group[0].WRCONFIG.reg = 
            //PORT_WRCONFIG_HWSEL
            PORT_WRCONFIG_WRPINCFG
            //|PORT_WRCONFIG_WRPMUX
            //|PORT_WRCONFIG_PMUX(MUX_PC16F_GCLK_IO1)
            //PORT_WRCONFIG_PULLEN
            //|PORT_WRCONFIG_INEN
            //|PORT_WRCONFIG_PMUXEN
            |PORT_WRCONFIG_PINMASK(PORT_PA15) //lower Halfword
        ;

        PORT->Group[0].DIRSET.reg= PORT_PA14;

        //Setup Mux Settings
        PORT->Group[0].WRCONFIG.reg = 
            //PORT_WRCONFIG_HWSEL
            PORT_WRCONFIG_WRPINCFG
            //|PORT_WRCONFIG_WRPMUX
            //|PORT_WRCONFIG_PMUX(MUX_PC16F_GCLK_IO1)
            //PORT_WRCONFIG_PULLEN
            //|PORT_WRCONFIG_INEN
            //|PORT_WRCONFIG_PMUXEN
            |PORT_WRCONFIG_PINMASK(PORT_PA14) //lower Halfword
        ;

        PORT->Group[0].DIRSET.reg= PORT_PA16;

        //Setup Mux Settings
        PORT->Group[0].WRCONFIG.reg = 
            PORT_WRCONFIG_HWSEL
            |PORT_WRCONFIG_WRPINCFG
            //|PORT_WRCONFIG_WRPMUX
            //|PORT_WRCONFIG_PMUX(MUX_PC16F_GCLK_IO1)
            //PORT_WRCONFIG_PULLEN
            //|PORT_WRCONFIG_INEN
            //|PORT_WRCONFIG_PMUXEN
            |PORT_WRCONFIG_PINMASK(PORT_PA16) //lower Halfword
        ;
}

extern AT86RF233_REG_IRQ_STATUS_t     g_trxLastIrq;               //from samr21trx.c

int main(int argc, char const *argv[])
{
    samr21Nvm_init();
    samr21ClockTrxSrcInit();
    samr21Trx_interfaceInit();
    samr21Trx_setupMClk(0x5); //MCLK 1MHz -> 16 Mhz
    samr21ClockInitAfterTrxSetup();

    samr21TimerInit();

    samr21DebugPortsInit();
    samr21RadioInit();  

    samr21Usb_init();

    uint64_t ieeeAddr = 0xB0B1B2B3B4B5B6B7;
    uint16_t shortAddr = 0xB8B9;
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
    tempFrame.header.frameControlField2.destinationAddressingMode = IEEE_802_15_4_ADDR_SHORT;
    tempFrame.header.frameControlField2.frameVersion = IEEE_802_15_4_VERSION_2006;
    tempFrame.header.frameControlField2.sourceAddressingMode = IEEE_802_15_4_ADDR_SHORT;
    tempFrame.header.sequenceNumber = 1;

    tempFrame.raw[4] =  0xFE;
    tempFrame.raw[5] =  0xCA;

    tempFrame.raw[6] =  0xB9;
    tempFrame.raw[7] =  0xB8;

    tempFrame.raw[8] =  0xA9;
    tempFrame.raw[9] =  0xA8;

    tempFrame.raw[10] =  'H';
    tempFrame.raw[11] =  'A';
    tempFrame.raw[12] =  'L';
    tempFrame.raw[13] =  'L';
    tempFrame.raw[14] =  'O';
    tempFrame.raw[15] =  ' ';
    tempFrame.raw[16] =  'W';
    tempFrame.raw[17] =  'E';
    tempFrame.raw[18] =  'L';
    tempFrame.raw[19] =  'T';
    tempFrame.raw[20] =  '!';

    tempFrame.header.lenght = 20 + IEEE_802_15_4_CRC_SIZE;
    
    uint32_t tempI = 0x0FFFF;

    char msgRcv[19] = "    Recived Frame: ";
    char msgAck[17] = "    Sending Ack: ";
    samr21Radio_statrtReceiving(13);

    while (true)
    {    
        JobBuffer_t * buffer = samr21RadioGetNextFinishedJobBuffer();
        if(buffer->jobState == RADIO_JOB_STATE_RX_DONE){
            char buf[170];

            memcpy(buf, msgRcv, 19);
            uint8_t len = 19;

            buf[0] = (buffer->inboundFrame.header.sequenceNumber / 100) + 48;;
            buf[1] = ((buffer->inboundFrame.header.sequenceNumber % 100) / 10) + 48;
            buf[2] = ((buffer->inboundFrame.header.sequenceNumber % 100) % 10) + 48;
            buf[3] = ' ';

            memcpy(&buf[len], buffer->inboundFrame.raw, buffer->inboundFrame.header.lenght+1);
            
            len += buffer->inboundFrame.header.lenght+1;

            buf[len++] = '\n';
            buf[len++] = '\r';

            tud_cdc_write(buf, len);
            tud_cdc_write_flush();

            buffer->jobState = RADIO_JOB_STATE_IDLE;
            
        }
        
        samr21Radio_statrtReceiving(13);
        samr21UsbEchoTask();
        tempI++; 
    }
}

void TCC0_Handler(){
    TCC0->INTFLAG.bit.OVF = 1;
}

void TCC1_Handler(){
    TCC1->INTFLAG.bit.OVF = 1;
}



