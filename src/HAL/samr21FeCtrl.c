/*
 * Copyright (c) 2023 dresden elektronik ingenieurtechnik gmbh.
 * All rights reserved.
 *
 * The software in this package is published under the terms of the BSD
 * style license a copy of which has been included with this distribution in
 * the LICENSE.txt file.
 *
 */
#include "samr21FeCtrl.h"


void samr21FeCtrl_init(void)
{
#if defined(TARGET_DEVICE) && ((TARGET_DEVICE == CONBEE2) || (TARGET_DEVICE == RASPBEE2))

    PM->APBCMASK.bit.RFCTRL_ = 1;

    PORT->Group[0].DIRSET.reg= PORT_PA08;
    //Setup Mux Settings
    PORT->Group[0].WRCONFIG.reg =
        //PORT_WRCONFIG_HWSEL
        PORT_WRCONFIG_WRPINCFG
        |PORT_WRCONFIG_WRPMUX
        |PORT_WRCONFIG_PMUX(MUX_PA08F_RFCTRL_FECTRL0)
        //PORT_WRCONFIG_PULLEN
        //|PORT_WRCONFIG_INEN
        |PORT_WRCONFIG_PMUXEN
        |PORT_WRCONFIG_PINMASK(PORT_PA08) //lower Halfword
    ;
    //Setup Mux Settings
    PORT->Group[0].DIRSET.reg= PORT_PA09;
    PORT->Group[0].WRCONFIG.reg =
        //PORT_WRCONFIG_HWSEL
        PORT_WRCONFIG_WRPINCFG
        |PORT_WRCONFIG_WRPMUX
        |PORT_WRCONFIG_PMUX(MUX_PA09F_RFCTRL_FECTRL1)
        //PORT_WRCONFIG_PULLEN
        //|PORT_WRCONFIG_INEN
        |PORT_WRCONFIG_PMUXEN
        |PORT_WRCONFIG_PINMASK(PORT_PA09) //lower Halfword
    ;
    
    //PA Bypass
    PORT->Group[0].DIRSET.reg= PORT_PA16;
    PORT->Group[0].OUTSET.reg= PORT_PA16;

    //PA Enable
    PORT->Group[0].DIRSET.reg= PORT_PA17;
    PORT->Group[0].OUTSET.reg= PORT_PA17;

    RFCTRL->FECFG.bit.F0CFG = 0x03;
	RFCTRL->FECFG.bit.F1CFG = 0x02;

#endif
}


void samr21FeCtrl_enable(void)
{

}


void samr21FeCtrl_disable(void)
{

}