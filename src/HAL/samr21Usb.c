//Author Eric Härtel @ dresden elektronik ingenieurtechnik gmbh © 2022
#include "samr21Usb.h"

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

    tusb_init();
}

void samr21UsbEchoTask(){
    tud_task(); 
    if ( tud_cdc_available() )
    {
    // read datas
        char buf[64];
        uint32_t count = tud_cdc_read(buf, sizeof(buf));

        // Echo back
        // Note: Skip echo by commenting out write() and write_flush()
        // for throughput test e.g
        //    $ dd if=/dev/zero of=/dev/ttyACM0 count=10000
        tud_cdc_write(buf, count);
        tud_cdc_write_flush();
    }
}
//--------------------------------------------------------------------+
// TINY USB
//--------------------------------------------------------------------+

//IRQ Handler wrapper
void dcd_int_handler (uint8_t rhport);
void USB_Handler(){
    dcd_int_handler(0);
}
