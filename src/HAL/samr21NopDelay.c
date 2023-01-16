/*
 * Copyright (c) 2023 dresden elektronik ingenieurtechnik gmbh.
 * All rights reserved.
 *
 * The software in this package is published under the terms of the BSD
 * style license a copy of which has been included with this distribution in
 * the LICENSE.txt file.
 *
 */
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