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

#define _DEBUG 1

#ifdef _DEBUG
static void samr21DebugPortsInit()
{

    PORT->Group[0].DIRSET.reg = PORT_PA06;

    // Setup Mux Settings
    PORT->Group[0].WRCONFIG.reg =
        // PORT_WRCONFIG_HWSEL
        PORT_WRCONFIG_WRPINCFG
        //|PORT_WRCONFIG_WRPMUX
        //|PORT_WRCONFIG_PMUX(MUX_PC16F_GCLK_IO1)
        // PORT_WRCONFIG_PULLEN
        //|PORT_WRCONFIG_INEN
        //|PORT_WRCONFIG_PMUXEN
        | PORT_WRCONFIG_PINMASK(PORT_PA06) // lower Halfword
        ;

    PORT->Group[0].DIRSET.reg = PORT_PA07;

    // Setup Mux Settings
    PORT->Group[0].WRCONFIG.reg =
        // PORT_WRCONFIG_HWSEL
        PORT_WRCONFIG_WRPINCFG
        //|PORT_WRCONFIG_WRPMUX
        //|PORT_WRCONFIG_PMUX(MUX_PC16F_GCLK_IO1)
        // PORT_WRCONFIG_PULLEN
        //|PORT_WRCONFIG_INEN
        //|PORT_WRCONFIG_PMUXEN
        | PORT_WRCONFIG_PINMASK(PORT_PA07) // lower Halfword
        ;

    PORT->Group[0].DIRSET.reg = PORT_PA08;

    // Setup Mux Settings
    PORT->Group[0].WRCONFIG.reg =
        // PORT_WRCONFIG_HWSEL
        PORT_WRCONFIG_WRPINCFG
        //|PORT_WRCONFIG_WRPMUX
        //|PORT_WRCONFIG_PMUX(MUX_PC16F_GCLK_IO1)
        // PORT_WRCONFIG_PULLEN
        //|PORT_WRCONFIG_INEN
        //|PORT_WRCONFIG_PMUXEN
        | PORT_WRCONFIG_PINMASK(PORT_PA08) // lower Halfword
        ;

    PORT->Group[0].DIRSET.reg = PORT_PA09;

    // Setup Mux Settings
    PORT->Group[0].WRCONFIG.reg =
        // PORT_WRCONFIG_HWSEL
        PORT_WRCONFIG_WRPINCFG
        //|PORT_WRCONFIG_WRPMUX
        //|PORT_WRCONFIG_PMUX(MUX_PC16F_GCLK_IO1)
        // PORT_WRCONFIG_PULLEN
        //|PORT_WRCONFIG_INEN
        //|PORT_WRCONFIG_PMUXEN
        | PORT_WRCONFIG_PINMASK(PORT_PA09) // lower Halfword
        ;

    PORT->Group[0].DIRSET.reg = PORT_PA09;

    // Setup Mux Settings
    PORT->Group[0].WRCONFIG.reg =
        // PORT_WRCONFIG_HWSEL
        PORT_WRCONFIG_WRPINCFG
        //|PORT_WRCONFIG_WRPMUX
        //|PORT_WRCONFIG_PMUX(MUX_PC16F_GCLK_IO1)
        // PORT_WRCONFIG_PULLEN
        //|PORT_WRCONFIG_INEN
        //|PORT_WRCONFIG_PMUXEN
        | PORT_WRCONFIG_PINMASK(PORT_PA09) // lower Halfword
        ;

    PORT->Group[0].DIRSET.reg = PORT_PA16;

    // Setup Mux Settings
    PORT->Group[0].WRCONFIG.reg =
        PORT_WRCONFIG_HWSEL
        |PORT_WRCONFIG_WRPINCFG
        //| PORT_WRCONFIG_WRPMUX 
        //| PORT_WRCONFIG_PMUX(MUX_PC16F_GCLK_IO1)
        //| PORT_WRCONFIG_PULLEN
        //| PORT_WRCONFIG_INEN
        //| PORT_WRCONFIG_PMUXEN 
        | PORT_WRCONFIG_PINMASK(PORT_PA16) // lower Halfword
        ;

    PORT->Group[0].DIRSET.reg = PORT_PA17;

    // Setup Mux Settings
    PORT->Group[0].WRCONFIG.reg =
        PORT_WRCONFIG_HWSEL
        |PORT_WRCONFIG_WRPINCFG
        //| PORT_WRCONFIG_WRPMUX 
        //| PORT_WRCONFIG_PMUX(MUX_PC16F_GCLK_IO1)
        //| PORT_WRCONFIG_PULLEN
        //| PORT_WRCONFIG_INEN
        //| PORT_WRCONFIG_PMUXEN 
        | PORT_WRCONFIG_PINMASK(PORT_PA17) // lower Halfword
        ;
}
#endif

void samr21TickleWatchdog()
{
#ifdef _GCF_RELEASE_
    WDT->CLEAR.reg = WDT_CLEAR_CLEAR_KEY_Val;
#endif
}



void otSysInit(int argc, char *argv[])
{

#ifdef _DEBUG
    samr21DebugPortsInit();
#endif

    samr21NvmInit();

    samr21TickleWatchdog();
    samr21ClockTrxSrcInit();

    samr21TickleWatchdog();
    samr21TrxInterfaceInit();
    
    samr21TickleWatchdog();
    samr21TrxSetupMClk(0x5); //MCLK 1MHz -> 16 Mhz
    
    samr21TickleWatchdog();
    samr21ClockInitAfterTrxSetup();
    
    samr21TickleWatchdog();
    samr21RtcInit();

    samr21TickleWatchdog();
    samr21UsbInit();

#ifdef _GCF_RELEASE_
    samr21TickleWatchdog();
    samr21FeCtrlInit();

    //Confirm the App Started to Bootloader
    uint8_t confirmedBtlFlag = 0x77;
    samr21NvmWriteWithinRow(0x4FFF, &confirmedBtlFlag, sizeof(uint8_t));
#endif

    samr21TickleWatchdog();

    //TCC1 Used by OT Micros Alarm
    samr21Timer1Init(0); // 1MHz / (2^0) -> 1us resolution
    //TCC2 Used by OT Millis Alarm
    samr21Timer2Init(7); // 1MHz / (2^7) -> ~1ms resolution


    //192 Lowest, 0 Highest
    NVIC_SetPriority(TCC0_IRQn, 192); //Unused 
    NVIC_SetPriority(TCC1_IRQn, 5); //Used by OT Micro Alarm
    NVIC_SetPriority(TCC2_IRQn, 4); //Used by OT Millis Alarm
    NVIC_SetPriority(TC3_IRQn, 192); //Unused
    NVIC_SetPriority(TC4_IRQn, 2); //Timer For Mac-Orchestration 
    NVIC_SetPriority(TC5_IRQn, 0); //Critical Timer for Mac-Security Feature
    NVIC_SetPriority(EIC_IRQn, 1); //IRQs from AT86RF233
    NVIC_SetPriority(USB_IRQn, 4); //For Communication with USB-Host
    NVIC_SetPriority(RTC_IRQn, 3); //For timed Transmission

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

#ifdef _GCF_RELEASE_   
    samr21TickleWatchdog();
#endif

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
    
    //Deinit all modules feed by MCLK from AT86RF233
    samr21RtcDeinit();
    samr21TimerDeinitAll();
    samr21UsbDeinit();
    samr21TrxInterruptDeinit();

    //Deinit all Clock-Domains derived from MCLK AT86RF233
    samr21ClockRemoveExternalSource();

    NVIC_SystemReset();
}