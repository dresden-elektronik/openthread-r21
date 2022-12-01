//Author Eric Härtel @ dresden elektronik ingenieurtechnik gmbh © 2022
#include "samr21.h"
#include <stdbool.h>
#include <stdint.h>

#include "HAL/samr21Trx.h"
#include "HAL/samr21Radio.h"
#include "HAL/samr21Rtc.h"
#include "HAL/samr21NopDelay.h"
#include "HAL/samr21PowerManager.h"
#include "HAL/samr21Timer.h"

#include "tusb.h"

void samr21NvmInit(){
    NVMCTRL->CTRLB.bit.RWS = 1;
}

bool dtrFlag;

void dcd_int_handler (uint8_t rhport);
void USB_Handler(){
    dcd_int_handler(0);
}

void samr21UsbInit(){
    //Setup Ports for USB
        //Setup PIN PA24 as USB D-
            //Make Input
            PORT->Group[0].DIRSET.reg= PORT_PA24;

            //Setup Mux Settings
            PORT->Group[0].WRCONFIG.reg =
                PORT_WRCONFIG_HWSEL
                |PORT_WRCONFIG_WRPINCFG
                |PORT_WRCONFIG_WRPMUX
                |PORT_WRCONFIG_PMUX(MUX_PA24G_USB_DM)
                //|PORT_WRCONFIG_PULLEN
                //|PORT_WRCONFIG_INEN
                |PORT_WRCONFIG_PMUXEN
                |PORT_WRCONFIG_PINMASK(PORT_PA24 >> 16) //upper Halfword
            ;
            PORT->Group[0].OUTCLR.reg= PORT_PA24;
            
        //Setup PIN PA25 as USB D+
            //Make Output
            PORT->Group[0].DIRSET.reg= PORT_PA25;

            //Setup Mux Settings
            PORT->Group[0].WRCONFIG.reg =
                PORT_WRCONFIG_HWSEL
                |PORT_WRCONFIG_WRPINCFG
                |PORT_WRCONFIG_WRPMUX
                |PORT_WRCONFIG_PMUX(MUX_PA25G_USB_DP)
                //|PORT_WRCONFIG_PULLEN
                //|PORT_WRCONFIG_INEN
                |PORT_WRCONFIG_PMUXEN
                |PORT_WRCONFIG_PINMASK(PORT_PA25 >> 16) //upper Halfword
            ;
            PORT->Group[0].OUTCLR.reg= PORT_PA25;
}

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
    tusb_init();

    uint64_t ieeeAddr = 0xB0B1B2B3B4B5B6B7;
    uint16_t shortAddr = 0xB8B9;
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
    char buf[15] = "Sending Frame\n\r";

    while (true)
    {   
        if(dtrFlag && g_trxLastIrq.reg){
            tud_cdc_write(g_trxLastIrq.reg, 1);
            tud_cdc_write_flush();
        }
    
        cdc_task(); // 
        tud_task(); // device task   
        tempI++; 
    }
}

// Invoked when device is mounted
void tud_mount_cb(void)
{
    __NOP();
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
    __NOP();
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
    __NOP();
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
    __NOP();
}


//--------------------------------------------------------------------+
// USB CDC
//--------------------------------------------------------------------+
void cdc_task(void)
{
  // connected() check for DTR bit
  // Most but not all terminal client set this when making connection
  // if ( tud_cdc_connected() )
  {
    // connected and there are data available
    if ( tud_cdc_available() )
    {
      // read datas
      char buf[64];
      uint32_t count = tud_cdc_read(buf, sizeof(buf));
      (void) count;

      // Echo back
      // Note: Skip echo by commenting out write() and write_flush()
      // for throughput test e.g
      //    $ dd if=/dev/zero of=/dev/ttyACM0 count=10000
      tud_cdc_write(buf, count);
      tud_cdc_write_flush();
    }
  }
}

// Invoked when cdc when line state changed e.g connected/disconnected


void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts)
{
  (void) itf;
  (void) rts;
  dtrFlag = dtr;
}

// Invoked when CDC interface received data from host
void tud_cdc_rx_cb(uint8_t itf)
{
  (void) itf;
}


