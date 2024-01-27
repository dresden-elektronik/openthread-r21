#include <otUtilities_codeUtils.h>
#include <openthread/platform/logging.h>
#include <openthread/platform/toolchain.h>

#include "samr21Uart.h"
 
#define LOG_PARSE_BUFFER_SIZE 128
char sLogString[LOG_PARSE_BUFFER_SIZE + 1];

static void logOutput(otLogLevel aLogLevel, otLogRegion aLogRegion, const char *aFormat, va_list ap)
{
    uartBuffer_t * buffer = samr21Uart_allocTransmitBuffer();

    if (!buffer)
    {
        //No Output Buffer Available
        return;
    }
    
    buffer->length = vsnprintf(buffer->data, UART_BUFFER_SIZE-1, aFormat, ap);


    samr21Uart_checkForPendingTransmit();
}

OT_TOOL_WEAK void otPlatLog(otLogLevel aLogLevel, otLogRegion aLogRegion, const char *aFormat, ...)
{
    va_list ap;

    va_start(ap, aFormat);

    logOutput(aLogLevel, aLogRegion, aFormat, ap);

    va_end(ap);


    OT_UNUSED_VARIABLE(aLogLevel);
    OT_UNUSED_VARIABLE(aLogRegion);
    OT_UNUSED_VARIABLE(aFormat);
}