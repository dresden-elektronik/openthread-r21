/*
 * Copyright (c) 2023 dresden elektronik ingenieurtechnik gmbh.
 * All rights reserved.
 *
 * The software in this package is published under the terms of the BSD
 * style license a copy of which has been included with this distribution in
 * the LICENSE.txt file.
 *
 */
#include "samr21Timer.h"



static struct 
{
    bool timer0Active;
    bool timer1Active;
    bool timer2Active;
    bool timer3Active;
    bool timer4Active;
    bool timer5Active;

}s_timerVars;


//TCC0
void samr21Timer0_init(uint8_t a_divider, bool a_oneshot, bool a_interrupt)
{
    if(s_timerVars.timer0Active){
        return;
    }
    // Enable In Power Manger
    PM->APBCMASK.bit.TCC0_ = 1;

    // Disable Modules First
    TCC0->CTRLA.bit.ENABLE = 0;
    while (TCC0->SYNCBUSY.bit.ENABLE)
        ;

    // Reset after
    TCC0->CTRLA.bit.SWRST = 1;
    while (TCC0->CTRLA.bit.SWRST || TCC0->SYNCBUSY.bit.SWRST)
        ;

    // Setup TC Modules
    
    // Enable IRQ in NVIC
    TCC0->INTENSET.bit.OVF = 1;
    
    TCC0->CTRLA.reg =
        TCC_CTRLA_ENABLE | TCC_CTRLA_RESOLUTION(TCC_CTRLA_RESOLUTION_NONE_Val) | TCC_CTRLA_PRESCALER(a_divider) | TCC_CTRLA_RUNSTDBY | TCC_CTRLA_PRESCSYNC(TCC_CTRLA_PRESCSYNC_GCLK_Val)
        //|TCC_CTRLA_ALOCK
        // TCC_CTRLA_CPTEN0
        // TCC_CTRLA_CPTEN1
        // TCC_CTRLA_CPTEN2
        // TCC_CTRLA_CPTEN3
        ;
    while (TCC0->SYNCBUSY.bit.ENABLE)
        ;

    TCC0->CTRLBSET.reg =
        TCC_CTRLBSET_CMD_STOP 
        | TCC_CTRLBSET_DIR 
        | ( a_oneshot ? TCC_CTRLBSET_ONESHOT : 0x00)
    ;

    while (TCC0->SYNCBUSY.bit.CTRLB)
        ;

    TCC0->WAVE.reg =
        TCC_WAVE_WAVEGEN(TCC_WAVE_WAVEGEN_MFRQ_Val);
    while (TCC0->SYNCBUSY.bit.WAVE)
        ;

    // clear pending interrupt in TC Module
    TCC0->INTFLAG.bit.OVF = 1;
    NVIC_ClearPendingIRQ(TCC0_IRQn);

    if(a_interrupt)
    {
            // Enable IRQ in NVIC
            NVIC_EnableIRQ(TCC0_IRQn);
    }


    s_timerVars.timer0Active = true;
}

void samr21Timer0_set(uint32_t a_timerTicks)
{
    while (TCC0->SYNCBUSY.reg)
        ;
    TCC0->COUNT.reg = a_timerTicks;

    TCC0->CTRLBSET.reg =
        TCC_CTRLBSET_CMD_RETRIGGER;
}

void samr21Timer0_stop()
{
    while (TCC0->SYNCBUSY.reg)
        ;

    TCC0->CTRLBSET.reg =
        TCC_CTRLBSET_CMD_STOP;
}

//TCC1 Used by OT Micros Alarm
void samr21Timer1_init(uint8_t a_divider, bool a_oneshot, bool a_interrupt)
{
    if(s_timerVars.timer1Active){
        return;
    }

    // Enable In Power Manger
    PM->APBCMASK.bit.TCC1_ = 1;

    // Disable Modules First
    TCC1->CTRLA.bit.ENABLE = 0;
    while (TCC1->SYNCBUSY.bit.ENABLE)
        ;

    // Reset after
    TCC1->CTRLA.bit.SWRST = 1;
    while (TCC1->CTRLA.bit.SWRST || TCC1->SYNCBUSY.bit.SWRST)
        ;

    // Setup TC Modules

    TCC1->INTENSET.bit.OVF = 1;


    TCC1->CTRLA.reg =
        TCC_CTRLA_ENABLE | TCC_CTRLA_RESOLUTION(TCC_CTRLA_RESOLUTION_NONE_Val) | TCC_CTRLA_PRESCALER(a_divider) | TCC_CTRLA_RUNSTDBY | TCC_CTRLA_PRESCSYNC(TCC_CTRLA_PRESCSYNC_GCLK_Val)
        //|TCC_CTRLA_ALOCK
        // TCC_CTRLA_CPTEN0
        // TCC_CTRLA_CPTEN1
        // TCC_CTRLA_CPTEN2
        // TCC_CTRLA_CPTEN3
        ;
    while (TCC1->SYNCBUSY.bit.ENABLE)
        ;

    TCC1->WAVE.reg =
        TCC_WAVE_WAVEGEN(TCC_WAVE_WAVEGEN_MFRQ_Val);
    while (TCC1->SYNCBUSY.bit.WAVE)
        ;

    TCC1->CTRLBSET.reg =
        TCC_CTRLBSET_CMD_STOP 
        | TCC_CTRLBSET_DIR 
        |(a_oneshot ? TCC_CTRLBSET_ONESHOT : 0x00)
    ;

    while (TCC1->SYNCBUSY.bit.CTRLB)
        ;

    // clear pending interrupt in TC Module
    TCC1->INTFLAG.bit.OVF = 1;
    NVIC_ClearPendingIRQ(TCC1_IRQn);
    
    if(a_interrupt)
    {
        // Enable IRQ in NVIC
        NVIC_EnableIRQ(TCC1_IRQn);
    }

    s_timerVars.timer1Active = true;
}

void samr21Timer1_startOneshot(uint32_t a_timerTicks)
{
    while (TCC1->SYNCBUSY.reg)
        ;
    TCC1->COUNT.reg = a_timerTicks;

    TCC1->CTRLBSET.reg =
        TCC_CTRLBSET_CMD_RETRIGGER;
}

void samr21Timer1_stop()
{
    while (TCC1->SYNCBUSY.reg)
        ;

    TCC1->CTRLBSET.reg =
        TCC_CTRLBSET_CMD_STOP;
}

//TCC2 Used by OT Millis Alarm
void samr21Timer2_init(uint8_t a_divider , bool a_oneshot, bool a_interrupt)
{
    if(s_timerVars.timer2Active){
        return;
    }

    // Enable In Power Manger
    PM->APBCMASK.bit.TCC2_ = 1;

    // Disable Modules First
    TCC2->CTRLA.bit.ENABLE = 0;
    while (TCC2->SYNCBUSY.bit.ENABLE)
        ;

    // Reset after
    TCC2->CTRLA.bit.SWRST = 1;
    while (TCC2->CTRLA.bit.SWRST || TCC2->SYNCBUSY.bit.SWRST)
        ;

    // Setup TC Modules

    TCC2->INTENSET.bit.OVF = 1;

    TCC2->CTRLA.reg =
        TCC_CTRLA_ENABLE | TCC_CTRLA_RESOLUTION(TCC_CTRLA_RESOLUTION_NONE_Val) | TCC_CTRLA_PRESCALER(a_divider) | TCC_CTRLA_RUNSTDBY | TCC_CTRLA_PRESCSYNC(TCC_CTRLA_PRESCSYNC_GCLK_Val)
        //|TCC_CTRLA_ALOCK
        // TCC_CTRLA_CPTEN0
        // TCC_CTRLA_CPTEN1
        // TCC_CTRLA_CPTEN2
        // TCC_CTRLA_CPTEN3
        ;
    while (TCC2->SYNCBUSY.bit.ENABLE)
        ;

    TCC2->WAVE.reg =
        TCC_WAVE_WAVEGEN(TCC_WAVE_WAVEGEN_MFRQ_Val);
    while (TCC2->SYNCBUSY.bit.WAVE)
        ;

    TCC2->CTRLBSET.reg =
        TCC_CTRLBSET_CMD_STOP 
        | TCC_CTRLBSET_DIR 
        | (a_oneshot ? TCC_CTRLBSET_ONESHOT : 0x00)
    ;
    
    while (TCC2->SYNCBUSY.bit.CTRLB);

    // clear pending interrupt in TC Module
    TCC2->INTFLAG.bit.OVF = 1;
    NVIC_ClearPendingIRQ(TCC2_IRQn);

    if(a_interrupt)
    {
        // Enable IRQ in NVIC
        NVIC_EnableIRQ(TCC2_IRQn);
    }

    s_timerVars.timer2Active = true;
}

void samr21Timer2_startOneshot(uint16_t a_timerTicks)
{
    while (TCC0->SYNCBUSY.reg)
        ;

    TCC2->COUNT.reg = a_timerTicks;

    TCC2->CTRLBSET.reg =
        TCC_CTRLBSET_CMD_RETRIGGER;
}

void samr21Timer2_stop()
{
    while (TCC2->SYNCBUSY.reg)
        ;

    TCC2->CTRLBSET.reg =
        TCC_CTRLBSET_CMD_STOP;
}

//TC3
void samr21Timer3_init(uint8_t a_divider , bool a_oneshot, bool a_interrupt)
{
    if(s_timerVars.timer3Active){
        return;
    }

    // Enable In Power Manger
    PM->APBCMASK.bit.TC3_ = 1;

    // Disable Modules First
    TC3->COUNT16.CTRLA.bit.ENABLE = 0;
    while (TC3->COUNT16.STATUS.bit.SYNCBUSY)
        ;

    // Reset after
    TC3->COUNT16.CTRLA.bit.SWRST = 1;
    while (TC3->COUNT16.CTRLA.bit.SWRST || TC3->COUNT16.STATUS.bit.SYNCBUSY)
        ;

    // Setup TC Modules


    TC3->COUNT16.INTENSET.bit.OVF = 1;

    TC3->COUNT16.CTRLA.reg =
        TC_CTRLA_ENABLE | TC_CTRLA_MODE(TC_CTRLA_MODE_COUNT16_Val) | TC_CTRLA_WAVEGEN(TC_CTRLA_WAVEGEN_MFRQ_Val) | TC_CTRLA_PRESCALER(a_divider) // 1Mhz
        | TC_CTRLA_RUNSTDBY | TC_CTRLA_PRESCSYNC(TC_CTRLA_PRESCSYNC_GCLK);
    while (TC3->COUNT16.STATUS.bit.SYNCBUSY)
        ;
    
    TC3->COUNT16.CTRLBSET.reg =
        TC_CTRLBSET_CMD_STOP 
        | TC_CTRLBSET_DIR
        | ( a_oneshot  ?  TC_CTRLBSET_ONESHOT : 0x00 )
    ;

    while (TC3->COUNT16.STATUS.bit.SYNCBUSY)
        ;

    // clear pending interrupt in TC Module
    TC3->COUNT16.INTFLAG.bit.OVF = 1;
    NVIC_ClearPendingIRQ(TC3_IRQn);

    if(a_interrupt)
    {
        // Enable IRQ in NVIC
        NVIC_EnableIRQ(TC3_IRQn);
    }

    s_timerVars.timer3Active = true;
}

void samr21Timer3_startOneshot(uint16_t a_timerTicks)
{
    while (TC3->COUNT16.STATUS.bit.SYNCBUSY)
        ;

    TC3->COUNT16.COUNT.reg = a_timerTicks;

    TC3->COUNT16.CTRLBSET.reg =
        TC_CTRLBSET_CMD_RETRIGGER;
}

void samr21Timer3_setContinuousPeriod(uint16_t a_timerTicks)
{
    while (TC3->COUNT16.STATUS.bit.SYNCBUSY) ;

    TC3->COUNT16.CC[0].reg = a_timerTicks;
    TC3->COUNT16.COUNT.reg = a_timerTicks;

    TC3->COUNT16.CTRLBSET.reg =
        TC_CTRLBSET_CMD_RETRIGGER;

}

void samr21Timer3_Stop()
{
    while (TC3->COUNT16.STATUS.bit.SYNCBUSY)
        ;

    TC3->COUNT16.CTRLBSET.reg =
        TC_CTRLBSET_CMD_STOP;
}

//TC4 Used by Soft-Radio Driver
void samr21Timer4_init(uint8_t a_divider , bool a_oneshot, bool a_interrupt)
{
    if(s_timerVars.timer4Active){
        return;
    }

    // Enable In Power Manger
    PM->APBCMASK.bit.TC4_ = 1;

    // Disable Modules First
    TC4->COUNT16.CTRLA.bit.ENABLE = 0;
    while (TC4->COUNT16.STATUS.bit.SYNCBUSY)
        ;

    // Reset after
    TC4->COUNT16.CTRLA.bit.SWRST = 1;
    while (TC4->COUNT16.CTRLA.bit.SWRST || TC4->COUNT16.STATUS.bit.SYNCBUSY)
        ;
    
    //Enable interrupt 
    TC4->COUNT16.INTENSET.bit.OVF = 1;


    TC4->COUNT16.CTRLA.reg =
        TC_CTRLA_ENABLE | TC_CTRLA_MODE(TC_CTRLA_MODE_COUNT16_Val) | TC_CTRLA_WAVEGEN(TC_CTRLA_WAVEGEN_MFRQ_Val) | TC_CTRLA_PRESCALER(a_divider) // 1Mhz
        | TC_CTRLA_RUNSTDBY | TC_CTRLA_PRESCSYNC(TC_CTRLA_PRESCSYNC_GCLK);
    while (TC4->COUNT16.STATUS.bit.SYNCBUSY)
        ;
    
    TC4->COUNT16.CTRLBSET.reg =
        TC_CTRLBSET_CMD_STOP 
        | TC_CTRLBSET_DIR 
        | (a_oneshot ? TC_CTRLBSET_ONESHOT : 0x00)
    ;

    while (TC4->COUNT16.STATUS.bit.SYNCBUSY)
        ;

    // clear pending interrupt in TC Module
    TC4->COUNT16.INTFLAG.bit.OVF = 1;
    NVIC_ClearPendingIRQ(TC4_IRQn);

    if(a_interrupt)
    {
        // Enable IRQ in NVIC
        NVIC_EnableIRQ(TC4_IRQn);
    }

    s_timerVars.timer4Active = true;
}

void samr21Timer4_startOneshot(uint16_t a_timerTicks)
{
    while (TC4->COUNT16.STATUS.bit.SYNCBUSY)
        ;

    TC4->COUNT16.COUNT.reg = a_timerTicks;

    TC4->COUNT16.CTRLBSET.reg =
        TC_CTRLBSET_CMD_RETRIGGER;
}

void samr21Timer4_stop()
{
    while (TC4->COUNT16.STATUS.bit.SYNCBUSY)
        ;

    TC4->COUNT16.CTRLBSET.reg =
        TC_CTRLBSET_CMD_STOP;
}

//TC5 Used by TRX Driver
void samr21Timer5_init(uint8_t a_divider, bool a_oneshot, bool a_interrupt){
    if(s_timerVars.timer5Active){
        return;
    }

    // Enable In Power Manger
    PM->APBCMASK.bit.TC5_ = 1;

    // Disable Modules First
    TC5->COUNT16.CTRLA.bit.ENABLE = 0;
    while (TC5->COUNT16.STATUS.bit.SYNCBUSY)
        ;

    // Reset after
    TC5->COUNT16.CTRLA.bit.SWRST = 1;
    while (TC5->COUNT16.CTRLA.bit.SWRST);

    // Enable Interrupt
    TC5->COUNT16.INTENSET.bit.OVF = 1;

    TC5->COUNT16.CTRLA.reg =
        TC_CTRLA_ENABLE | TC_CTRLA_MODE(TC_CTRLA_MODE_COUNT16_Val) | TC_CTRLA_WAVEGEN(TC_CTRLA_WAVEGEN_MFRQ_Val) | TC_CTRLA_PRESCALER(a_divider) // 1Mhz
        | TC_CTRLA_RUNSTDBY | TC_CTRLA_PRESCSYNC(TC_CTRLA_PRESCSYNC_GCLK);
    while (TC5->COUNT16.STATUS.bit.SYNCBUSY)
        ;
    
    TC5->COUNT16.CTRLBSET.reg =
        TC_CTRLBSET_CMD_STOP 
        | TC_CTRLBSET_DIR
        | (a_oneshot ? TC_CTRLBSET_ONESHOT : 0x00)
    ;

    while (TC5->COUNT16.STATUS.bit.SYNCBUSY)
        ;

    // clear pending interrupt in TC Module
    TC5->COUNT16.INTFLAG.bit.OVF = 1;
    NVIC_ClearPendingIRQ(TC5_IRQn);
    
    if(a_interrupt)
    {
        // Enable IRQ in NVIC
        NVIC_EnableIRQ(TC5_IRQn);
    }

    s_timerVars.timer5Active = true;
}

void samr21Timer5_startOneshot(uint16_t a_timerTicks)
{
    while (TC5->COUNT16.STATUS.bit.SYNCBUSY)
        ;

    TC5->COUNT16.COUNT.reg = a_timerTicks;

    TC5->COUNT16.CTRLBSET.reg =
        TC_CTRLBSET_CMD_RETRIGGER;
}

void samr21Timer5_stop()
{
    while (TC5->COUNT16.STATUS.bit.SYNCBUSY)
        ;

    TC5->COUNT16.CTRLBSET.reg =
        TC_CTRLBSET_CMD_STOP;
}


//Deinit Timer for Soft Reset
void samr21Timer_deinitAll()
{

    // Stop All running timers
    samr21Timer0_stop();
    samr21Timer1_stop();
    samr21Timer2_stop();
    samr21Timer3_Stop();
    samr21Timer4_stop();
    samr21Timer5_stop();

    // Disable the Interrupts
    NVIC_DisableIRQ(TCC0_IRQn);
    NVIC_DisableIRQ(TCC1_IRQn);
    NVIC_DisableIRQ(TCC2_IRQn);
    NVIC_DisableIRQ(TC3_IRQn);
    NVIC_DisableIRQ(TC4_IRQn);
    NVIC_DisableIRQ(TC5_IRQn);

    // Disble TCC Modules
    TCC0->CTRLA.reg = TCC_CTRLA_RESETVALUE;
    while (TCC0->SYNCBUSY.bit.ENABLE)
        ;
    TCC1->CTRLA.reg = TCC_CTRLA_RESETVALUE;
    while (TCC0->SYNCBUSY.bit.ENABLE)
        ;
    TCC2->CTRLA.reg = TCC_CTRLA_RESETVALUE;
    while (TCC0->SYNCBUSY.bit.ENABLE)
        ;

    PM->APBCMASK.bit.TCC0_ = 0;
    PM->APBCMASK.bit.TCC1_ = 0;
    PM->APBCMASK.bit.TCC2_ = 0;
    
    s_timerVars.timer0Active = false;
    s_timerVars.timer1Active = false;
    s_timerVars.timer2Active = false;
    
    // Disble TC Modules
    TC3->COUNT16.CTRLA.reg = TC_CTRLA_RESETVALUE;
    while (TC3->COUNT16.STATUS.bit.SYNCBUSY)
        ;
    TC4->COUNT16.CTRLA.reg = TC_CTRLA_RESETVALUE;
    while (TC3->COUNT16.STATUS.bit.SYNCBUSY)
        ;
    TC5->COUNT16.CTRLA.reg = TC_CTRLA_RESETVALUE;
    while (TC3->COUNT16.STATUS.bit.SYNCBUSY)
        ;

    PM->APBCMASK.bit.TC3_ = 0;
    PM->APBCMASK.bit.TC4_ = 0;
    PM->APBCMASK.bit.TC5_ = 0;

    s_timerVars.timer3Active = false;
    s_timerVars.timer4Active = false;
    s_timerVars.timer5Active = false;
}





// MOVED TO TODO
//  void TCC0_Handler(){
//      TCC0->INTFLAG.bit.OVF = 1;
//      /*CODE*/
//  }
// MOVED TO TODO

// MOVED TO TODO
//  void TCC1_Handler(){
//      TCC1->INTFLAG.bit.OVF = 1;
//      /*CODE*/
//  }
// MOVED TO TODO

// MOVED TO TODO
//  void TCC2_Handler(){
//      TCC2->INTFLAG.bit.OVF = 1;
//      /*CODE*/
//  }
// MOVED TO TODO

// MOVED TO otPlatAlarm.c
//  void TC3_Handler(){
//      TC3->COUNT16.INTFLAG.bit.OVF = 1;
//      /*CODE*/
//  }
// MOVED TO otPlatAlarm.c

// MOVED TO samr21RadioFSM.c
//  void TC4_Handler(){
//      TC4->COUNT16.INTFLAG.bit.OVF = 1;
//      /*CODE*/
//  }
// MOVED TO samr21RadioFSM.c
//  void TC5_Handler(){
//      TC5->COUNT16.INTFLAG.bit.OVF = 1;
//      /*CODE*/
//  }
// MOVED TO samr21RadioFSM.c
