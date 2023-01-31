//Author Eric Härtel @ dresden elektronik ingenieurtechnik gmbh © 2022
#include "otPlatSystemHeader.h"
#include "openthread/platform/misc.h"

#include "samr21.h"
#include "samr21Clock.h"
#include "samr21Trx.h"
#include "samr21RadioCtrl.h"
#include "samr21Rtc.h"
#include "samr21Nvm.h"
#include "samr21NopDelay.h"
#include "samr21Timer.h"
#include "samr21Usb.h"

#include "samr21debugPins.h"

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

    samr21DebugPortsInit();
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
    samr21OtPlatRadioTask();
}

otPlatResetReason otPlatGetResetReason(otInstance *aInstance){
    switch (PM->RCAUSE.reg)
    {
    case PM_RCAUSE_POR:
        return OT_PLAT_RESET_REASON_POWER_ON;
    
    case PM_RCAUSE_BOD12:
        return OT_PLAT_RESET_REASON_FAULT;
        
    case PM_RCAUSE_BOD33:
        return OT_PLAT_RESET_REASON_FAULT;

    case PM_RCAUSE_EXT:
        return OT_PLAT_RESET_REASON_EXTERNAL;

    case PM_RCAUSE_WDT:
        return OT_PLAT_RESET_REASON_WATCHDOG;

    case PM_RCAUSE_SYST:
        return OT_PLAT_RESET_REASON_SOFTWARE;

    default:
        return OT_PLAT_RESET_REASON_OTHER; //TODO
    }
}

void otPlatReset(otInstance *aInstance){
   NVIC_SystemReset();
}