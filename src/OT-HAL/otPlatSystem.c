//Author Eric Härtel @ dresden elektronik ingenieurtechnik gmbh © 2022
#include "otPlatSystemHeader.h"
#include "openthread/platform/misc.h"

#include "samr21.h"
#include "samr21Clock.h"
#include "samr21Trx.h"
#include "samr21RadioCtrl.h"
#include "samr21RadioFeCtrl.h"
#include "samr21Rtc.h"
#include "samr21Nvm.h"
#include "samr21NopDelay.h"
#include "samr21Timer.h"
#include "samr21Usb.h"
#include "samr21Uart.h"


static void samr21TickleWatchdog()
{
#ifdef _GCF_RELEASE_
    WDT->CLEAR.reg = WDT_CLEAR_CLEAR_KEY_Val;
#endif
}

static void samr21InitIrqPriority()
{
    //192 Lowest, 0 Highest
    NVIC_SetPriority(TCC0_IRQn, 192); //Unused 
    NVIC_SetPriority(TCC1_IRQn, 5); //Used by OT Micro Alarm
    NVIC_SetPriority(TCC2_IRQn, 4); //Used by OT Millis Alarm
    NVIC_SetPriority(TC3_IRQn, 1); //Timer for DMA-Pace while uploading JIT to framebuffer
    NVIC_SetPriority(DMAC_IRQn, 0); //Timer for DMA-Pace while uploading to framebuffer
    NVIC_SetPriority(TC4_IRQn, 3); //Timer For Mac-Orchestration 
    NVIC_SetPriority(TC5_IRQn, 0); //Critical Timer for Mac-Security Feature
    NVIC_SetPriority(EIC_IRQn, 2); //IRQs from AT86RF233
    NVIC_SetPriority(USB_IRQn, 5); //For Communication with USB-Host
    NVIC_SetPriority(RTC_IRQn, 4); //For timed Transmission
}


void otSysInit(int argc, char *argv[])
{
    samr21InitIrqPriority();
    samr21TickleWatchdog();

    samr21NvmInit();
    samr21TickleWatchdog();

    samr21ClockInit();
    samr21TickleWatchdog();

    samr21TickleWatchdog();
    samr21TrxInterfaceInit();
    
    samr21TickleWatchdog();
    samr21RtcInit();

    //Give Some Slack (~0.2sec) before new USB enumeration
    __disable_irq();
    for (uint32_t i = 0; i < 1000; i++)
    {
        samr21delaySysTick(10000);
        samr21TickleWatchdog();
    }
    __enable_irq();
    
    samr21UsbInit();

#ifdef _GCF_RELEASE_
    samr21TickleWatchdog();
    samr21FeCtrlInit();

    //Confirm the App Started to Bootloader
    uint8_t confirmedBtlFlag = 0x77;
    samr21NvmWriteWithinRow(0x4FFF, &confirmedBtlFlag, sizeof(uint8_t));
#endif

    //samr21TickleWatchdog();
    //samr21LogInit();

    samr21TickleWatchdog();
    samr21OtPlatAlarmInit();
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
    samr21OtPlatCommTask();
    samr21OtPlatRadioTask();
    samr21OtPlatAlarmTask();

    samr21TickleWatchdog();
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