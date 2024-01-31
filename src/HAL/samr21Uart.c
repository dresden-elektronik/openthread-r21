#include "samr21Uart.h"
#include "samr21Dma.h"

//Template Include
#define FIFO_BUFFER_ENTRY_DATA_TYPE uartBuffer_t
#include "utils/fifoBuffer_t.h"

static uartBuffer_t s_uartBuffer[NUM_UART_BUFFER];

static fifoBuffer_t s_uartDmaFifoBuffer =
{
    .buffer = s_uartBuffer,
    .bufferSize = sizeof(s_uartBuffer) / sizeof(s_uartBuffer[0]),
    .readHeadPos = 0,
    .writeHeadPos = 0
};

static bool s_dmaBusy = false;

void dma_callback(void);

void samr21Uart_init(){
    
    //Enable in Power manager
        PM->APBCMASK.bit.SERCOM2_ = 1;

    //Setup Ports for UART
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



    //Reset SERCOM2
        SERCOM2->USART.CTRLA.bit.SWRST = 1;

    // Wait for SERCOM2 reset to finish
        while ( SERCOM2->USART.CTRLA.bit.SWRST || SERCOM2->USART.SYNCBUSY.bit.SWRST );
    
    //Setup SERCOM2
        
        //F_ref = 1MHz (F_baud = F_ref / 2*(BAUD+1)  = 500 kBAUD)
        SERCOM2->USART.BAUD.reg=
            SERCOM_SPI_BAUD_BAUD(0) 
        ;

        //Only enable Output
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
        while (SERCOM2->USART.SYNCBUSY.reg);


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


    //Reset the dma Fifo Buffer
    fifo_reset(&s_uartDmaFifoBuffer);

    //Init DMA for tx
    samr21Dma_initChannel(
        1,
        (uint32_t)(&SERCOM2->USART.DATA.reg),
        0x06, //SERCOM2 TX Trigger
        dma_callback
    );
}

void samr21Uart_deinit(){
    //Disable
    SERCOM2->USART.CTRLA.bit.ENABLE = 0;
    while ( SERCOM2->USART.CTRLA.bit.ENABLE || SERCOM2->USART.SYNCBUSY.bit.ENABLE );

    //Disable RTC In Power Manger
    PM->APBAMASK.bit.RTC_ = 0;

        //GCLK_CLKCTRL_WRTLOCK
        //GCLK_CLKCTRL_CLKEN
        GCLK_CLKCTRL_GEN(0) // GCLKGEN2
        |GCLK_CLKCTRL_ID(GCLK_CLKCTRL_ID_SERCOM2_CORE_Val)
    ;

    //Reset the dma Fifo Buffer
    fifo_reset(&s_uartDmaFifoBuffer);
}

void samr21Uart_sendByte(uint8_t a_data){
    while (!SERCOM2->USART.INTFLAG.bit.DRE);
    //Put data into the tranmitt buffer to start transmission
    SERCOM2->USART.DATA.bit.DATA = a_data;
}


void samr21Uart_checkForPendingTransmit(void)
{
    if (!s_dmaBusy)
    {
        uartBuffer_t * pendingBuffer_p = fifo_peak(&s_uartDmaFifoBuffer);

        if(( pendingBuffer_p != NULL) && (pendingBuffer_p->length > 0)) //Dont't start while the Buffer is in setup (marked by length == -1)
        {
            //Transmit the next Buffer
            samr21Dma_activateChannel(1, pendingBuffer_p->data, pendingBuffer_p->length, NULL);

            //Jumpstart DMA
            samr21Dma_triggerChannelAction(0);
        }
    }
}

uartBuffer_t * samr21Uart_allocTransmitBuffer(void)
{
    uartBuffer_t * buffer = fifo_alloc(&s_uartDmaFifoBuffer);
    
    if (buffer)
    {
        buffer->length = 0;
    }

    return buffer;
}

uint16_t samr21Uart_write(uint8_t * data, uint16_t length)
{
    uint16_t bytesSend = 0;

    while (bytesSend < length)
    {
        uartBuffer_t * newBuffer = fifo_alloc(&s_uartDmaFifoBuffer);

        if(!newBuffer)
        {
            //No Buffer Available
            goto exit;
        }

        newBuffer->length = -1; //prevent DMA callback from staring this preemptively

        uint16_t bufferFillLength = (length > UART_BUFFER_SIZE ? UART_BUFFER_SIZE : length); 

        memcpy(newBuffer->data, &data[bytesSend],  bufferFillLength);

        newBuffer->length = bufferFillLength; 

        bytesSend += bufferFillLength;
    }

exit:
    
    samr21Uart_checkForPendingTransmit();
    return bytesSend;
}

void dma_callback(void)
{
    //discard completed Transmission;
    fifo_discard(&s_uartDmaFifoBuffer);

    //Check if there is more data
    uartBuffer_t * pendingBuffer_p = fifo_peak(&s_uartDmaFifoBuffer);

    if((pendingBuffer_p != NULL) && (pendingBuffer_p->length > 0)) //Dont't start while the Buffer is in setup (marked by length == -1)
    {
        //Transmit the next Buffer
        samr21Dma_activateChannel(1, pendingBuffer_p->data, pendingBuffer_p->length, NULL);

        //Jumpstart DMA
        // Note: not necessary cause last Tx Byte should still be in transmit 
        //samr21Dma_triggerChannelAction(0);
    }
    else
    {
        s_dmaBusy = false;
    }
}
