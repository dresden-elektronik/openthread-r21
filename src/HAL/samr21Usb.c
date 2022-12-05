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

    //Init Tiny Usb
    tusb_init();
}

void samr21UsbTask(){
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

// Invoked when cdc when line state changed e.g connected/disconnected
void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts)
{
  (void) itf;
  (void) rts;
  (void) dtr;
}

// Invoked when CDC interface received data from host
void tud_cdc_rx_cb(uint8_t itf)
{
  (void) itf;
}

