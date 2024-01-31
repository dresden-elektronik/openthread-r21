#include "otUtilities_uart.h"

#include "tusb.h"
#include "tusb_config.h"
volatile static bool s_dtr = false;

#define SIZE_WAIT_FOR_HOST_BUFFER 1024

static struct 
{
    const uint8_t*      pendingTxBuffer;
    uint16_t            pendingTxBufferLength;

    bool                current_otUartEnabled;
    bool                current_dataTerminalReady;

    bool                last_otUartEnabled;
    bool                last_dataTerminalReady;

    bool                ongoingTransmit;
    bool                finishedTransmit;


}s_otPlatUartUsbVars;


#ifdef GCF_BUILD
static const uint8_t s_gcfResetCommand[] =
    {
        0xC0, //Slip END Flag
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

static uint8_t s_gcfResetCommandMatchLen = 0;

#endif

static void uart_receiveTask()
{
    // Check for Available RX-Data
    if (tud_cdc_available())
    {
        char buf[64];
        uint32_t count = tud_cdc_read(buf, sizeof(buf));

        otPlatUartReceived(buf, count);

#ifdef GCF_BUILD
        for(uint8_t i = 0; i < count; i++){
            if(buf[i] == s_gcfResetCommand[s_gcfResetCommandMatchLen])
            {
                if(++s_gcfResetCommandMatchLen == sizeof(s_gcfResetCommand))
                {
                    otPlatReset();
                }
            }
            else
            {
                s_gcfResetCommandMatchLen = 0;
                break;
            }
        }

#endif
    }
}

static void uart_transmitTask()
{
    if(s_otPlatUartUsbVars.current_dataTerminalReady && ( s_otPlatUartUsbVars.pendingTxBuffer != NULL ) ){

        tud_cdc_write(s_otPlatUartUsbVars.pendingTxBuffer, s_otPlatUartUsbVars.pendingTxBufferLength);

        s_otPlatUartUsbVars.ongoingTransmit         = true;
        s_otPlatUartUsbVars.pendingTxBuffer         = NULL;
        s_otPlatUartUsbVars.pendingTxBufferLength   = 0;

        tud_cdc_write_flush();
        return;
    }

    if(s_otPlatUartUsbVars.ongoingTransmit && s_otPlatUartUsbVars.finishedTransmit){
        
        s_otPlatUartUsbVars.ongoingTransmit       = false;
        s_otPlatUartUsbVars.finishedTransmit      = false;

        otPlatUartSendDone();
        return;
    }

    if( (CFG_TUD_CDC_TX_BUFSIZE == tud_cdc_write_available()) && s_otPlatUartUsbVars.ongoingTransmit ){
        
        s_otPlatUartUsbVars.ongoingTransmit       = false;
        s_otPlatUartUsbVars.finishedTransmit      = false;

        otPlatUartSendDone();
    }
}

void samr21OtPlat_uartCommTask(){
    tud_task();

    s_otPlatUartUsbVars.current_dataTerminalReady = tud_cdc_connected();
    uart_receiveTask();
    uart_transmitTask();


    s_otPlatUartUsbVars.last_dataTerminalReady = s_otPlatUartUsbVars.current_dataTerminalReady;
    s_otPlatUartUsbVars.last_otUartEnabled = s_otPlatUartUsbVars.current_otUartEnabled;
}

otError otPlatUartEnable(void)
{
    s_otPlatUartUsbVars.last_otUartEnabled = true;
    return OT_ERROR_NONE;
}

otError otPlatUartDisable(void)
{
    s_otPlatUartUsbVars.last_otUartEnabled = false;
    return OT_ERROR_NONE;
}

otError otPlatUartFlush(void)
{
    while(tud_cdc_write_flush());

    s_otPlatUartUsbVars.ongoingTransmit       = false;
    s_otPlatUartUsbVars.finishedTransmit      = false;

    otPlatUartSendDone();

    return OT_ERROR_NONE;
}

otError otPlatUartSend(const uint8_t *a_buf_p, uint16_t a_bufLength)
{
    if(s_otPlatUartUsbVars.ongoingTransmit){
        otPlatUartFlush();
    }

#ifdef _DEBUG
    assert(s_otPlatUartUsbVars.ongoingTransmit == false);
    assert(s_otPlatUartUsbVars.finishedTransmit == false);
#endif

    if( s_otPlatUartUsbVars.current_dataTerminalReady ){
        tud_cdc_write(a_buf_p, a_bufLength);

        s_otPlatUartUsbVars.ongoingTransmit = true;

        tud_cdc_write_flush();
    }
    else
    {
        s_otPlatUartUsbVars.pendingTxBuffer = a_buf_p;
        s_otPlatUartUsbVars.pendingTxBufferLength = a_bufLength;
    }

    return OT_ERROR_NONE;
}

otError otPlatWakeHost()
{
    tud_remote_wakeup();
}