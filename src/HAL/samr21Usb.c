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

void samr21Usb_init(){

    //Enable in Power Manger 
    PM->APBBMASK.bit.USB_ = 1;
    
    //Setup Ports for USB

    //Setup PIN PA24 as USB D- 
    //Make Output
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

    __NVIC_EnableIRQ(USB_IRQn);
}

void samr21Usb_deinit(){

    // Disable IRQ in NVIC
    __NVIC_DisableIRQ(USB_IRQn);

    USB->DEVICE.CTRLA.bit.ENABLE = 0;
    while (USB->DEVICE.CTRLA.bit.ENABLE);

    USB->DEVICE.CTRLA.bit.SWRST = 1;
    while (USB->DEVICE.SYNCBUSY.bit.SWRST);
    
    PM->APBBMASK.bit.USB_ = 0;
}


//--------------------------------------------------------------------+
// TINY USB
//--------------------------------------------------------------------+

//IRQ Handler wrapper
void dcd_int_handler (uint8_t rhport);
void USB_Handler(){
    dcd_int_handler(0);
}

