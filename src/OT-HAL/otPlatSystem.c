//Author Eric Härtel @ dresden elektronik ingenieurtechnik gmbh © 2022
#include "otPlatSystemHeader.h"
#include "openthread/platform/misc.h"

#include "samr21.h"
#include "samr21Clock.h"
#include "samr21Trx.h"
#include "samr21Radio.h"
#include "samr21FeCtrl.h"
#include "samr21Rtc.h"
#include "samr21Nvm.h"
#include "samr21SysTick.h"
#include "samr21Timer.h"
#include "samr21Usb.h"
#include "samr21Dma.h"
#include "samr21Uart.h"

#include "tusb.h"
#include "tusb_config.h"


static void samr21_tickleWatchdog()
{
#ifdef GCF_BUILD
    WDT->CLEAR.reg = WDT_CLEAR_CLEAR_KEY_Val;
#endif
}

static void samr21_initIrqPriority()
{
    //192 Lowest, 0 Highest
    NVIC_SetPriority(TCC0_IRQn, 192); //Unused 
    NVIC_SetPriority(TCC1_IRQn, 5); //Used by OT Micro Alarm
    NVIC_SetPriority(TCC2_IRQn, 4); //Used by OT Millis Alarm
    NVIC_SetPriority(TC3_IRQn, 1); //Timer for DMA-Pace while uploading JIT to framebuffer
    NVIC_SetPriority(DMAC_IRQn, 0); //Timer for DMA-Pace while uploading to framebuffer
    NVIC_SetPriority(TC4_IRQn, 1); //Timer For Mac-Orchestration 
    NVIC_SetPriority(TC5_IRQn, 0); //Critical Timer for Mac-Security Feature
    NVIC_SetPriority(EIC_IRQn, 2); //IRQs from AT86RF233
    NVIC_SetPriority(USB_IRQn, 5); //For Communication with USB-Host
    NVIC_SetPriority(RTC_IRQn, 4); //For timed Transmission
}


void otSysInit(int argc, char *argv[])
{
    samr21_tickleWatchdog();
    samr21_initIrqPriority();

    samr21Nvm_init();

#ifdef GCF_BUILD
    //Confirm the App Started to Bootloader
    uint8_t confirmedBtlFlag = 0x77;
    samr21Nvm_writeWithinRow(0x4FFF, &confirmedBtlFlag, sizeof(uint8_t));
#endif

    samr21_tickleWatchdog();
    samr21Clock_enableFallbackClockTree(); //Not depending on MCLK of AT86RF233
    
    samr21_tickleWatchdog();
    samr21Dma_init();

    samr21_tickleWatchdog();
    samr21Trx_initInterface(); //Also inits Clock Output of the AT86RF233, so we can switch to a Crystal based clock Domain

#if defined(SAMR21_USE_USB_CLOCK) && (SAMR21_USE_USB_CLOCK > 0)
    samr21Usb_init(); 
    samr21Clock_enableOperatingClockTree(); //Depending on receiving USB-SOF Signals
#else
    samr21Clock_enableOperatingClockTree(); //Depending on correct output freq of AT86RF233 MCLK
    samr21Usb_init();
#endif

    samr21_tickleWatchdog();
    samr21Trx_initDriver();
    samr21FeCtrl_init();

    samr21_tickleWatchdog();
    samr21Rtc_init();

    samr21_tickleWatchdog();
    tusb_init();


    samr21_tickleWatchdog();
    samr21Uart_init();

    samr21_tickleWatchdog();
    samr21OtPlat_alarmInit();
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
    samr21OtPlat_uartCommTask();
    samr21OtPlat_radioTick();
    samr21OtPlat_alarmTask();

    samr21_tickleWatchdog();
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