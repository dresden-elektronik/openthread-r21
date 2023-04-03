#include "otUtilities_uart.h"
#include "samr21Usb.h"
    
volatile static bool s_dtr = false;

struct waitForHostBuffer_s
{
    uint16_t len;
    uint8_t  buf[];
} s_waitForHostBuffer;


#ifdef _GCF_RELEASE_
static const uint8_t gcfResetCommand[] =
{
    0x0B,
    0x03,
    0x00,
    0x0C,
    0x00,
    0x05,
    0x00,
    0x26,
    0x02,
    0x00,
    0x00,
    0x00
};
extern volatile bool g_keepAlive;
#endif

void samr21OtPlatCommReceiveTask(){

    tud_task(); 

    //Check for Available RX-Data
    if ( tud_cdc_available() )
    {
        char buf[64];
        uint32_t count = tud_cdc_read(buf, sizeof(buf));

        otPlatUartReceived(buf,count);        
    
#ifdef _GCF_RELEASE_
        if( ! strncmp( buf, gcfResetCommand, sizeof(gcfResetCommand) ) ){
            NVIC_SystemReset();
        }
#endif
    }
}

otError otPlatUartEnable(void){
    tusb_init();
    return OT_ERROR_NONE;
}

otError otPlatUartDisable(void)
{
    return OT_ERROR_NONE;
}

otError otPlatUartFlush(void)
{
    tud_cdc_write_flush();
    return OT_ERROR_NONE;
}

otError otPlatUartSend(const uint8_t *aBuf, uint16_t aBufLength)
{
    if( 
        ( tud_cdc_write_available() < aBufLength )
        || !s_dtr 
    ){
        memcpy(s_waitForHostBuffer.buf, aBuf, aBufLength);
        s_waitForHostBuffer.len = aBufLength;
        return OT_ERROR_NONE;
    }

    tud_cdc_write(aBuf, aBufLength);
    tud_cdc_write_flush();
    otPlatUartSendDone();
    return OT_ERROR_NONE;
}

otError otPlatWakeHost(){
   //TODO 
}

void tud_cdc_tx_complete_cb(uint8_t itf){
    otPlatUartSendDone();
}

void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts){
    (void)itf;
    (void)rts;
    s_dtr = dtr;

    if(s_waitForHostBuffer.len && dtr){
        tud_cdc_write(s_waitForHostBuffer.buf, s_waitForHostBuffer.len);
        //s_waitForHostBuffer.len = 0;
        otPlatUartSendDone();
    }


    //Dirty WA
    if(!dtr){
        NVIC_SystemReset();
    }
}