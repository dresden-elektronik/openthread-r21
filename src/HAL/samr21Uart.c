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
        //Setup PIN PA14 as UART RX
            //Make Input
            PORT->Group[0].DIRCLR.reg= PORT_PA14;

            //Setup Mux Settings
            PORT->Group[0].WRCONFIG.reg =
                //PORT_WRCONFIG_HWSEL
                PORT_WRCONFIG_WRPINCFG
                |PORT_WRCONFIG_WRPMUX
                |PORT_WRCONFIG_PMUX(MUX_PA14C_SERCOM2_PAD2)
                //|PORT_WRCONFIG_PULLEN
                |PORT_WRCONFIG_INEN
                |PORT_WRCONFIG_PMUXEN
                |PORT_WRCONFIG_PINMASK(PORT_PA14) //lower Halfword
            ;
        //Setup PIN PA15 as UART TX
            //Make Output
            PORT->Group[0].DIRSET.reg= PORT_PA15;

            //Setup Mux Settings
            PORT->Group[0].WRCONFIG.reg =
                //PORT_WRCONFIG_HWSEL
                PORT_WRCONFIG_WRPINCFG
                |PORT_WRCONFIG_WRPMUX
                |PORT_WRCONFIG_PMUX(MUX_PA15C_SERCOM2_PAD3)
                //|PORT_WRCONFIG_PULLEN
                //|PORT_WRCONFIG_INEN
                |PORT_WRCONFIG_PMUXEN
                |PORT_WRCONFIG_PINMASK(PORT_PA15) //lower Halfword
            ;



    //Reset SERCOM4
        SERCOM2->USART.CTRLA.bit.SWRST = 1;

    // Wait for SERCOM4 reset to finish
        while ( SERCOM2->USART.CTRLA.bit.SWRST || SERCOM2->USART.SYNCBUSY.bit.SWRST );
    
    //Setup SERCOM4
        
        //F_ref = 16MHz (if At86r233 ist setup correctly) F_baud = F_ref / 2*(BAUD+1) ---> BAUD = 0 F_baud = 8Mhz
        SERCOM2->USART.BAUD.reg=
            SERCOM_SPI_BAUD_BAUD(0x0) 
        ;

        SERCOM4->SPI.CTRLB.reg=
            SERCOM_SPI_CTRLB_RXEN
            //|SERCOM_SPI_CTRLB_AMODE(0)
            //|SERCOM_SPI_CTRLB_MSSEN
            //|SERCOM_SPI_CTRLB_SSDE
            //|SERCOM_SPI_CTRLB_PLOADEN
            |SERCOM_SPI_CTRLB_CHSIZE(0x0) //8Bit
        ;
        // Wait for SERCOM4 Sync
        while ( SERCOM4->SPI.SYNCBUSY.bit.CTRLB );


        SERCOM4->SPI.CTRLA.reg=
            SERCOM_SPI_CTRLA_RUNSTDBY
            |SERCOM_SPI_CTRLA_MODE(SERCOM_SPI_CTRLA_MODE_SPI_MASTER_Val)
            |SERCOM_SPI_CTRLA_ENABLE
            //|SERCOM_SPI_CTRLA_SWRST
            //|SERCOM_SPI_CTRLA_IBON
            |SERCOM_SPI_CTRLA_DOPO(1) //MOSI IS ON PB30 SERCOM4/PAD[2] , SCLK ON PB30 SERCOM4/PAD[3]
            |SERCOM_SPI_CTRLA_DIPO(0) //MISO IS ON PC19 SERCOM4/PAD[0]
            |SERCOM_SPI_CTRLA_FORM(0)
            //|SERCOM_SPI_CTRLA_CPHA
            //|SERCOM_SPI_CTRLA_CPOL
            //|SERCOM_SPI_CTRLA_DORD
        ;
        // Wait for SERCOM4 to setup
        while (SERCOM4->SPI.SYNCBUSY.reg);
}

void samr21UartDeinit(){

}

void samr21UartSend(){

}