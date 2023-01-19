//Author Eric Härtel @ dresden elektronik ingenieurtechnik gmbh © 2022
#include "otPlatSystemHeader.h"
#include "openthread/platform/misc.h"

#include "samr21Clock.h"
#include "samr21Trx.h"
#include "samr21RadioCtrl.h"
#include "samr21Rtc.h"
#include "samr21Nvm.h"
#include "samr21NopDelay.h"
#include "samr21Timer.h"
#include "samr21Usb.h"


void otSysInit(int argc, char *argv[])
{
    samr21NvmInit();
    samr21ClockTrxSrcInit();
    samr21TrxInterfaceInit();
    samr21TrxSetupMClk(0x5); //MCLK 1MHz -> 16 Mhz
    samr21ClockInitAfterTrxSetup();

    samr21TimerInit();

    samr21RadioInitIrq();  

    samr21UsbInit();

}

bool otSysPseudoResetWasRequested(void)
{
    return false;
}

void otSysDeinit(void)
{
    __NOP();
}

void otSysProcessDrivers(otInstance *aInstance)
{
    samr21OtPlatUsbTask();
}

otPlatResetReason otPlatGetResetReason(otInstance *aInstance){
    return OT_PLAT_RESET_REASON_POWER_ON; //TODO
}

void otPlatReset(otInstance *aInstance){
    __NOP();
}