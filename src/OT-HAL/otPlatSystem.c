//Author Eric Härtel @ dresden elektronik ingenieurtechnik gmbh © 2022
#include "openthread-system.h"

#include "otPlatSystemHeader.h"

#include "samr21Clock.h"
#include "samr21Trx.h"
#include "samr21Radio.h"
#include "samr21Rtc.h"
#include "samr21NopDelay.h"
#include "samr21PowerManager.h"
#include "samr21Timer.h"
#include "samr21Usb.h"

void samr21NvmInit(){
    NVMCTRL->CTRLB.bit.RWS = 1;
}

void otSysInit(int argc, char *argv[])
{
    samr21PowerManagerInit();
    samr21NvmInit();
    
    samr21ClockInit();

    samr21TrxInterfaceInit();
    samr21TrxSetupMClk(0x5); //MCLK 1MHz -> 16 Mhz

    samr21ClockInitAfterTrxSetup();

    samr21TimerInit();

    samr21UsbInit();

    samr21RadioInit(); 
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
    samr21OtPlatAlarmTask(aInstance);
    samr21OtPlatUsbTask();
}