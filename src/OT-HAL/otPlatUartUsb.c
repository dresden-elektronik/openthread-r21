#include "uart.h"
#include "samr21Usb.h"
    
volatile static bool s_dtr = false;

void samr21OtPlatUsbTask(){

    tud_task(); 

    //Check for Available RX-Data
    if ( tud_cdc_available() )
    {
        char buf[64];
        uint32_t count = tud_cdc_read(buf, sizeof(buf));

        otPlatUartReceived(buf,count);        
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
        return OT_ERROR_FAILED;
    }

    tud_cdc_write(aBuf, aBufLength);
    tud_cdc_write_flush();

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
}