//Author Eric Härtel @ dresden elektronik ingenieurtechnik gmbh © 2022
#include "samr21NopDelay.h"

extern uint32_t g_currentCpuClkCycle_ns;



void samr21delaySysTick(uint32_t delayCycles){
    //Sets the trigger value
    SysTick->LOAD = delayCycles;
    //Clear current value register
    SysTick->VAL = 0;
    //Enable Systick
    SysTick->CTRL = 
        SysTick_CTRL_CLKSOURCE_Msk 
        |SysTick_CTRL_ENABLE_Msk
    ;

    //Delay Loop
    while(!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk));

    //Disable Systick
    SysTick->CTRL = 0;
}

void samr21delayLoop(uint32_t delayCycles){
    delayCycles = delayCycles >> 3; //Devide by 8
    for (uint32_t i = 0; i < delayCycles; i++)
    {
        __NOP();
    }
}