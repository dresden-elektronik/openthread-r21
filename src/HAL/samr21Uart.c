#include "samr21Uart.h"


void samr21UartInit(){
//Setup Clocks for TRX-SPI
        //Use GCLKGEN0 as core Clock for the debug UART
        GCLK->CLKCTRL.reg =
            //GCLK_CLKCTRL_WRTLOCK
            GCLK_CLKCTRL_CLKEN
            |GCLK_CLKCTRL_GEN(0) // GCLKGEN1
            |GCLK_CLKCTRL_ID(GCLK_CLKCTRL_ID_SERCOM2_CORE_Val)
        ;
        //Wait for synchronization 
        while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY );

    
    //Enable in Power manager
        PM->APBCMASK.bit.SERCOM2_ = 1;

    //Setup Ports for TRX
        //Setup PIN PA14 as UART TX
            //Make Input
            PORT->Group[0].DIRSET.reg= PORT_PA14;

            //Setup Mux Settings
            PORT->Group[0].WRCONFIG.reg =
                //PORT_WRCONFIG_HWSEL
                PORT_WRCONFIG_WRPINCFG
                |PORT_WRCONFIG_WRPMUX
                |PORT_WRCONFIG_PMUX(MUX_PA14C_SERCOM2_PAD2)
                //|PORT_WRCONFIG_PULLEN
                //|PORT_WRCONFIG_INEN
                |PORT_WRCONFIG_PMUXEN
                |PORT_WRCONFIG_PINMASK(PORT_PA14) //lower Halfword
            ;
        //Setup PIN PA15 as UART TX
            //Make Output
            PORT->Group[0].DIRCLR.reg= PORT_PA15;

            //Setup Mux Settings
            PORT->Group[0].WRCONFIG.reg =
                //PORT_WRCONFIG_HWSEL
                PORT_WRCONFIG_WRPINCFG
                |PORT_WRCONFIG_WRPMUX
                |PORT_WRCONFIG_PMUX(MUX_PA15C_SERCOM2_PAD3)
                //|PORT_WRCONFIG_PULLEN
                |PORT_WRCONFIG_INEN
                |PORT_WRCONFIG_PMUXEN
                |PORT_WRCONFIG_PINMASK(PORT_PA15) //lower Halfword
            ;



    //Reset SERCOM4
        SERCOM2->USART.CTRLA.bit.SWRST = 1;

    // Wait for SERCOM2 reset to finish
        while ( SERCOM2->USART.CTRLA.bit.SWRST || SERCOM2->USART.SYNCBUSY.bit.SWRST );
    
    //Setup SERCOM2
        
        //F_ref = 16MHz (if At86r233 ist setup correctly) F_baud = F_ref / 2*(BAUD+1) ---> BAUD = 7 F_baud = 3MBAUD
        SERCOM2->USART.BAUD.reg=
            SERCOM_SPI_BAUD_BAUD(0) 
        ;

        SERCOM2->USART.CTRLB.reg=
            SERCOM_USART_CTRLB_TXEN
            //|SERCOM_USART_CTRLB_RXEN
            //|SERCOM_USART_CTRLB_COLDEN
            //|SERCOM_USART_CTRLB_ENC
            //|SERCOM_USART_CTRLB_SFDE
            //|SERCOM_USART_CTRLB_SBMODE
            |SERCOM_USART_CTRLB_CHSIZE(0x0)
        ;
        // Wait for SERCOM2 Sync
        while ( SERCOM4->SPI.SYNCBUSY.bit.CTRLB );


        SERCOM2->USART.CTRLA.reg=
            SERCOM_USART_CTRLA_RUNSTDBY
            |SERCOM_USART_CTRLA_MODE(SERCOM_USART_CTRLA_MODE_USART_INT_CLK_Val)
            //|SERCOM_USART_CTRLA_CMODE
            |SERCOM_USART_CTRLA_ENABLE
            //|SERCOM_USART_CTRLA_IBON
            //|SERCOM_USART_CTRLA_DORD
            //|SERCOM_USART_CTRLA_CPOL
            |SERCOM_USART_CTRLA_FORM(0x1)
            //SERCOM_USART_CTRLA_SAMPR()
            |SERCOM_USART_CTRLA_RXPO(0x3)
            |SERCOM_USART_CTRLA_TXPO(0x1)
        ;
        // Wait for SERCOM2 to setup
        while (SERCOM2->USART.SYNCBUSY.reg);
}

void samr21UartDeinit(){
    //Disable
    SERCOM2->USART.CTRLA.bit.ENABLE = 0;
    while ( SERCOM2->USART.CTRLA.bit.ENABLE || SERCOM2->USART.SYNCBUSY.bit.ENABLE );

    //Disable RTC In Power Manger
    PM->APBAMASK.bit.RTC_ = 0;

    //Disable CLKGEN
    GCLK->CLKCTRL.reg =
        //GCLK_CLKCTRL_WRTLOCK
        //GCLK_CLKCTRL_CLKEN
        GCLK_CLKCTRL_GEN(0) // GCLKGEN2
        |GCLK_CLKCTRL_ID(GCLK_CLKCTRL_ID_SERCOM2_CORE_Val)
    ;
}

void samr21UartSend(uint8_t a_data){
    while (!SERCOM2->USART.INTFLAG.bit.DRE);
    //Put data into the tranmitt buffer to start transmission
    SERCOM2->USART.DATA.bit.DATA = a_data;
}