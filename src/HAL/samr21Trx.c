//Author Eric Härtel @ dresden elektronik ingenieurtechnik gmbh © 2022
#include "samr21Trx.h"

//Local Register Copys of At86rf233 to save some Read Acceses
volatile AT86RF233_REG_TRX_STATUS_t    g_trxStatus;   //used as external var
volatile AT86RF233_REG_IRQ_STATUS_t    g_trxLastIrq;  //used as external var         
volatile AT86RF233_REG_TRX_CTRL_0_t    g_trxCtrl0;    //used as external var


void samr21TrxInterfaceInit(){
    //Setup Ports for TRX
        //Setup PIN PC19 as MISO <-> At86rf233 via SERCOM4
            //Make Input
            PORT->Group[2].DIRCLR.reg= PORT_PC19;

            //Setup Mux Settings
            PORT->Group[2].WRCONFIG.reg =
                PORT_WRCONFIG_HWSEL
                |PORT_WRCONFIG_WRPINCFG
                |PORT_WRCONFIG_WRPMUX
                |PORT_WRCONFIG_PMUX(MUX_PC19F_SERCOM4_PAD0)
                //|PORT_WRCONFIG_PULLEN
                |PORT_WRCONFIG_INEN
                |PORT_WRCONFIG_PMUXEN
                |PORT_WRCONFIG_PINMASK(PORT_PC19 >> 16) //upper Halfword
            ;
        //Setup PIN PB30 as MOSI <-> At86rf233 via SERCOM4
            //Make Output
            PORT->Group[1].DIRSET.reg= PORT_PB30;

            //Setup Mux Settings
            PORT->Group[1].WRCONFIG.reg =
                PORT_WRCONFIG_HWSEL
                |PORT_WRCONFIG_WRPINCFG
                |PORT_WRCONFIG_WRPMUX
                |PORT_WRCONFIG_PMUX(MUX_PB30F_SERCOM4_PAD2)
                //|PORT_WRCONFIG_PULLEN
                //|PORT_WRCONFIG_INEN
                |PORT_WRCONFIG_PMUXEN
                |PORT_WRCONFIG_PINMASK(PORT_PB30 >> 16) //upper Halfword
            ;

        //Setup PIN PC18 as SCLK <-> At86rf233 via SERCOM4
            //Make Output
            PORT->Group[2].DIRSET.reg= PORT_PC18;

            //Setup Mux Settings
            PORT->Group[2].WRCONFIG.reg =
                PORT_WRCONFIG_HWSEL
                |PORT_WRCONFIG_WRPINCFG
                |PORT_WRCONFIG_WRPMUX
                |PORT_WRCONFIG_PMUX(MUX_PC18F_SERCOM4_PAD3)
                //|PORT_WRCONFIG_PULLEN
                //|PORT_WRCONFIG_INEN
                |PORT_WRCONFIG_PMUXEN
                |PORT_WRCONFIG_PINMASK(PORT_PC18 >> 16) //upper Halfword
            ;

        //Setup PIN PB31 as SEL <-> At86rf233
        //SEL is configreud as Mnual PORT, cause we need Control over ist for
        //Realtime Buffer Read
            //Make Output
            PORT->Group[1].DIRSET.reg= PORT_PB31;

            //Setup Mux Settings
            PORT->Group[1].WRCONFIG.reg =
                PORT_WRCONFIG_HWSEL
                |PORT_WRCONFIG_WRPINCFG
                //|PORT_WRCONFIG_WRPMUX
                //|PORT_WRCONFIG_PMUX(0)
                //|PORT_WRCONFIG_PULLEN
                //|PORT_WRCONFIG_INEN
                //|PORT_WRCONFIG_PMUXEN
                |PORT_WRCONFIG_PINMASK(PORT_PB31 >> 16) //upper Halfword
            ;
            
            //Set Default High
            PORT->Group[1].OUT.reg |= ((uint32_t)1 & 0x1) << 31;
        
        //Setup PIN PB15 as RSTN <-> At86rf233
            //Make Output
            PORT->Group[1].DIRSET.reg= PORT_PB15;

            //Setup Mux Settings
            PORT->Group[1].WRCONFIG.reg =
                //PORT_WRCONFIG_HWSEL
                PORT_WRCONFIG_WRPINCFG
                //|PORT_WRCONFIG_WRPMUX
                //|PORT_WRCONFIG_PMUX(0)
                //|PORT_WRCONFIG_PULLEN
                //|PORT_WRCONFIG_INEN
                //|PORT_WRCONFIG_PMUXEN
                |PORT_WRCONFIG_PINMASK(PORT_PB15) //lower Halfword
            ;

            //Set Default High
            PORT->Group[1].OUT.reg |= ((uint32_t)1 & 0x1) << 15;
        //Setup PIN PB00 as IRQ <-> At86rf233 via EXTINT[0]      
            //Make Input
            PORT->Group[1].DIRCLR.reg= PORT_PB00;

            //Setup Mux Settings
            PORT->Group[1].WRCONFIG.reg =
                //PORT_WRCONFIG_HWSEL
                PORT_WRCONFIG_WRPINCFG
                |PORT_WRCONFIG_WRPMUX
                |PORT_WRCONFIG_PMUX(MUX_PB00A_EIC_EXTINT0)
                //|PORT_WRCONFIG_PULLEN
                |PORT_WRCONFIG_INEN
                |PORT_WRCONFIG_PMUXEN
                |PORT_WRCONFIG_PINMASK(PORT_PB00) //lower Halfword
            ;
        //Setup PIN PA20 as SLP_TR <-> At86rf233
            //Make Output
            PORT->Group[0].DIRSET.reg= PORT_PA20;

            //Setup Mux Settings
            PORT->Group[0].WRCONFIG.reg =
                PORT_WRCONFIG_HWSEL
                |PORT_WRCONFIG_WRPINCFG
                //|PORT_WRCONFIG_WRPMUX
                //|PORT_WRCONFIG_PMUX(0)
                //|PORT_WRCONFIG_PULLEN
                //|PORT_WRCONFIG_INEN
                //|PORT_WRCONFIG_PMUXEN
                |PORT_WRCONFIG_PINMASK(PORT_PA20 >> 16) //upper Halfword
            ;

            //Set Default Low
            PORT->Group[0].OUT.reg |= ((uint32_t)0 & 0x1) << 20;


    //Reset SERCOM4
        SERCOM4->SPI.CTRLA.bit.SWRST = 1;

    // Wait for SERCOM4 reset to finish
        while ( SERCOM4->SPI.CTRLA.bit.SWRST || SERCOM4->SPI.SYNCBUSY.bit.SWRST );
    
    //Setup SERCOM4
        
        //F_ref = 16MHz (if At86r233 ist setup correctly) F_baud = F_ref / 2*(BAUD+1) ---> BAUD = 0 F_baud = 8Mhz
        SERCOM4->SPI.BAUD.reg=
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

void samr21TrxSetupMClk(uint8_t clk){

    //Write a 0x5 into the CLKM Register to get a 16Mhz Clock on CLKM
    g_trxCtrl0.bit.clkmCtrl=clk;

    samr21TrxWriteRegister(TRX_CTRL_0_REG, g_trxCtrl0.reg);

    //Wait a bit for the  the Freq.-change to take place
    samr21delayLoop(50);  
}

uint8_t samr21TrxReadRegister(uint8_t addr){
    __disable_irq();
    
    //Enable Slave Select
    samr21TrxSetSSel(true);
    samr21delayLoop(CPU_WAIT_CYCLE_AFTER_SSEL_LOW);

    //Buffer for return Value
    uint8_t rVal;
        
    //Send Addr and get Status Byte (see r2 datasheet 35.4 Radio Transceiver Status Information)
    g_trxStatus.reg = samr21TrxSpiTransceiveByteRaw( (addr & 0x3F) | AT86RF233_CMD_REG_READ_MASK );
    samr21delayLoop(CPU_WAIT_CYCLE_BETWEEN_BYTES);

    //Send Dummy Data
    rVal = samr21TrxSpiTransceiveByteRaw(0x0);

    //Disable Slave Select
    samr21delayLoop(CPU_WAIT_CYCLE_BEFORE_SSEL_HIGH);
    samr21TrxSetSSel(false);

    __enable_irq();
    return rVal;
}

void samr21TrxWriteRegister(uint8_t addr, uint8_t data){
    __disable_irq();
    //Enable Slave Select
    samr21TrxSetSSel(true);
    samr21delayLoop(CPU_WAIT_CYCLE_AFTER_SSEL_LOW);
      
    //Send Addr and get Status Byte (see r2 datasheet 35.4 Radio Transceiver Status Information)
    g_trxStatus.reg = samr21TrxSpiTransceiveByteRaw( (addr & 0x3F ) | AT86RF233_CMD_REG_WRITE_MASK );
    samr21delayLoop(CPU_WAIT_CYCLE_BETWEEN_BYTES);

    //Send  Data
    samr21TrxSpiTransceiveByteRaw(data);

    //Disable Slave Select
    samr21delayLoop(CPU_WAIT_CYCLE_BEFORE_SSEL_HIGH);
    samr21TrxSetSSel(false);
    __enable_irq();
}

void samr21TrxReadFromSRam(uint8_t addr, uint8_t* readBuffer, uint8_t lenght){
    
    __disable_irq();
    //Enable Slave Select
    samr21TrxSetSSel(true);
    samr21delayLoop(CPU_WAIT_CYCLE_AFTER_SSEL_LOW);
    
    //Send Read SRAM Command and get Status Byte (see r2 datasheet 35.4 Radio Transceiver Status Information)
    g_trxStatus.reg = samr21TrxSpiTransceiveByteRaw( AT86RF233_CMD_SRAM_READ );

    //Send Address
    samr21delayLoop(CPU_WAIT_CYCLE_BETWEEN_BYTES_FAST_ACCESS);
    samr21TrxSpiTransceiveByteRaw( addr );

    //Download data
    for (uint8_t i = 0; i < lenght ; i++)
    {
        //wait a sec
        samr21delayLoop(CPU_WAIT_CYCLE_BETWEEN_BYTES_FAST_ACCESS);
        readBuffer[i] = samr21TrxSpiTransceiveByteRaw(SPI_DUMMY_BYTE); 
    }

    //Disable Slave Select
    samr21delayLoop(CPU_WAIT_CYCLE_BEFORE_SSEL_HIGH);
    samr21TrxSetSSel(false);
    __enable_irq();
}

void samr21TrxWriteToSRam(uint8_t addr, uint8_t* writeBuffer, uint8_t lenght){
    
    __disable_irq();
    //Enable Slave Select
    samr21TrxSetSSel(true);
    samr21delayLoop(CPU_WAIT_CYCLE_AFTER_SSEL_LOW);
    
    //Send Write SRAM Command and get Status Byte (see r21 datasheet 35.4 Radio Transceiver Status Information)
    g_trxStatus.reg = samr21TrxSpiTransceiveByteRaw( AT86RF233_CMD_SRAM_WRITE );

    //Send Address
    samr21delayLoop(CPU_WAIT_CYCLE_BETWEEN_BYTES_FAST_ACCESS);
    samr21TrxSpiTransceiveByteRaw( addr );

    //Upload data
    for (uint8_t i = 0; i < lenght ; i++)
    {
        samr21delayLoop(CPU_WAIT_CYCLE_BETWEEN_BYTES_FAST_ACCESS);
        samr21TrxSpiTransceiveByteRaw(writeBuffer[i]);
    }


    //Disable Slave Select
    samr21delayLoop(CPU_WAIT_CYCLE_BEFORE_SSEL_HIGH);
    samr21TrxSetSSel(false);
    __enable_irq();
}

void samr21TrxUpdateStatus(){
    __disable_irq();
    //Enable Slave Select
    samr21TrxSetSSel(true);
    samr21delayLoop(CPU_WAIT_CYCLE_AFTER_SSEL_LOW);    
    
    //Send Dummy Data to get Status Byte (see r2 datasheet 35.4 Radio Transceiver Status Information)
    g_trxStatus.reg = samr21TrxSpiTransceiveByteRaw( 0xFF );

    //Disable Slave Select
    samr21delayLoop(CPU_WAIT_CYCLE_BEFORE_SSEL_HIGH);
    samr21TrxSetSSel(false);
    __enable_irq();
}

void samr21TrxSetSSel(bool enabled){
    //Pin PB31 == SSEL
    //Active Low
    if(enabled){
        PORT->Group[1].OUTCLR.reg = 1 << 31;
        return;
    }
    PORT->Group[1].OUTSET.reg = 1 << 31;
}

void samr21TrxSetRSTN(bool enabled){
    //Pin PB15 == RSTN
    //Active Low
    if(enabled){
        PORT->Group[1].OUTCLR.reg = 1 << 15;
        return;
    }
    PORT->Group[1].OUTSET.reg = 1 << 15;
}

void samr21TrxSetSLP_TR(bool enabled){
    //Pin PA20 == SLP_TR
    //Active High
    if(enabled){
        PORT->Group[0].OUTSET.reg = 1 << 20;
        return;
    }
    PORT->Group[0].OUTCLR.reg = 1 << 20;
}

uint8_t samr21TrxSpiTransceiveByteRaw(uint8_t data){
    while (!SERCOM4->SPI.INTFLAG.bit.DRE);
    //Put data into the tranmitt buffer to start transmission
    SERCOM4->SPI.DATA.bit.DATA = data;
    
    while (!SERCOM4->SPI.INTFLAG.bit.TXC);
    while (!SERCOM4->SPI.INTFLAG.bit.RXC);

    //return recived Answerto
    return SERCOM4->SPI.DATA.bit.DATA;

}




