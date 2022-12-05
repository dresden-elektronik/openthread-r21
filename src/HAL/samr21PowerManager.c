//Author Eric Härtel @ dresden elektronik ingenieurtechnik gmbh © 2022
#include "samr21PowerManager.h"

void samr21PowerManagerInit(){
    
        PM->APBAMASK.reg = 
            PM_APBAMASK_GCLK        // Turn on the interface to GCLK 
            |PM_APBAMASK_SYSCTRL    // Turn on the interface to SYSCTRL 
            |PM_APBAMASK_RTC        // Turn on the interface to RTC
            |PM_APBAMASK_EIC        // Turn on the interface to EIC
            |PM_APBAMASK_WDT        // Turn on the interface to WDT   
            |PM_APBAMASK_PM         // Turn on the interface to PM      
        ;
    
        PM->APBBMASK.reg = 
            PM_APBBMASK_PORT        // Turn on the interface to PORT 
            |PM_APBBMASK_NVMCTRL    // Turn on the interface to NVMCTRL
            |PM_APBBMASK_USB        // Turn on the interface to USB  
        ;
    
        PM->APBCMASK.reg = 
            PM_APBCMASK_SERCOM4     // Turn on the interface to SERCOM4 
            |PM_APBCMASK_TC3        // Turn on the interface to  TC4
            |PM_APBCMASK_TC4        // Turn on the interface to  TC4
            |PM_APBCMASK_TC5        // Turn on the interface to  TC5
        ;
}