#include <otUtilities_codeUtils.h>
#include <openthread/platform/logging.h>
#include <openthread/platform/toolchain.h>

#include "samr21Uart.h"
 
#define LOG_PARSE_BUFFER_SIZE 128
char sLogString[LOG_PARSE_BUFFER_SIZE + 1];

void samr21LogInit(void)
{
    samr21Uart_init();
}

static void logOutput(otLogLevel aLogLevel, otLogRegion aLogRegion, const char *aFormat, va_list ap)
{
    int len = 0;

    len = vsnprintf(sLogString, LOG_PARSE_BUFFER_SIZE, aFormat, ap);

    otEXPECT(len >= 0);

exit:

    if (len >= LOG_PARSE_BUFFER_SIZE)
    {
        len = LOG_PARSE_BUFFER_SIZE - 1;
    }

    sLogString[len++] = '\n';

    for(int i = 0; i<len; i++ ){
        samr21Uart_send(sLogString[i]);
    }
}

OT_TOOL_WEAK void otPlatLog(otLogLevel aLogLevel, otLogRegion aLogRegion, const char *aFormat, ...)
{
    va_list ap;

    va_start(ap, aFormat);

    //logOutput(aLogLevel, aLogRegion, aFormat, ap);

    va_end(ap);


    OT_UNUSED_VARIABLE(aLogLevel);
    OT_UNUSED_VARIABLE(aLogRegion);
    OT_UNUSED_VARIABLE(aFormat);
}