/*
 * Copyright (c) 2023 dresden elektronik ingenieurtechnik gmbh.
 * All rights reserved.
 *
 * The software in this package is published under the terms of the BSD
 * style license a copy of which has been included with this distribution in
 * the LICENSE.txt file.
 *
 */
#include "samr21Usb.h"

void samr21UsbInit(){

    //Use GCLKGEN0 as Ref Freq for USB
        GCLK->CLKCTRL.reg =
            //GCLK_CLKCTRL_WRTLOCK
            GCLK_CLKCTRL_CLKEN
            |GCLK_CLKCTRL_GEN(0) // GCLKGEN0
            |GCLK_CLKCTRL_ID(GCLK_CLKCTRL_ID_USB_Val)
        ;
        //Wait for synchronization 
        while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY );

    //Enable in Power Manger 
        PM->APBBMASK.bit.USB_ = 1;
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

#ifdef _GCF_RELEASE_

    USB->DEVICE.CTRLA.bit.SWRST = 1;

    while(USB->DEVICE.CTRLA.bit.SWRST || USB->DEVICE.CTRLA.bit.ENABLE);
    
    samr21delaySysTick(0xFFFFFF);
  
#endif
    
    tusb_init();
}


void samr21UsbDeinit(){
    
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

