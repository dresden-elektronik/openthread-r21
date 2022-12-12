//Author Eric Härtel @ dresden elektronik ingenieurtechnik gmbh © 2022
#include "samr21Timer.h"

void samr21TimerInit(){

    //Disable Modules First
    TCC0->CTRLA.bit.ENABLE = 0;
    while (TCC0->SYNCBUSY.bit.ENABLE);
    TCC1->CTRLA.bit.ENABLE = 0;
    while (TCC1->SYNCBUSY.bit.ENABLE);
    TCC2->CTRLA.bit.ENABLE = 0;
    while (TCC2->SYNCBUSY.bit.ENABLE);
    TC3->COUNT16.CTRLA.bit.ENABLE = 0;
    while (TC3->COUNT16.STATUS.bit.SYNCBUSY);
    TC4->COUNT16.CTRLA.bit.ENABLE = 0;
    while (TC4->COUNT16.STATUS.bit.SYNCBUSY);
    TC5->COUNT16.CTRLA.bit.ENABLE = 0;
    while (TC5->COUNT16.STATUS.bit.SYNCBUSY);

    //Reset after
    TCC0->CTRLA.bit.SWRST = 1;
    while (TCC0->CTRLA.bit.SWRST || TCC0->SYNCBUSY.bit.SWRST);
    TCC1->CTRLA.bit.SWRST = 1;
    while (TCC1->CTRLA.bit.SWRST || TCC1->SYNCBUSY.bit.SWRST);
    TCC2->CTRLA.bit.SWRST = 1;
    while (TCC2->CTRLA.bit.SWRST || TCC2->SYNCBUSY.bit.SWRST);
    TC3->COUNT16.CTRLA.bit.SWRST = 1;
    while (TC3->COUNT16.CTRLA.bit.SWRST || TC3->COUNT16.STATUS.bit.SYNCBUSY);
    TC4->COUNT16.CTRLA.bit.SWRST = 1;
    while (TC4->COUNT16.CTRLA.bit.SWRST || TC4->COUNT16.STATUS.bit.SYNCBUSY);
    TC5->COUNT16.CTRLA.bit.SWRST = 1;
    while (TC5->COUNT16.CTRLA.bit.SWRST || TC5->COUNT16.STATUS.bit.SYNCBUSY);


    //Setup TC Modules
    TCC0->INTENSET.bit.OVF = 1;
    TCC0->CTRLA.reg =
        TCC_CTRLA_ENABLE
        |TCC_CTRLA_RESOLUTION(TCC_CTRLA_RESOLUTION_NONE_Val)
        |TCC_CTRLA_PRESCALER(TCC_CTRLA_PRESCALER_DIV1_Val)
        |TCC_CTRLA_RUNSTDBY
        |TCC_CTRLA_PRESCSYNC(TCC_CTRLA_PRESCSYNC_GCLK_Val)
        //|TCC_CTRLA_ALOCK
        //TCC_CTRLA_CPTEN0
        //TCC_CTRLA_CPTEN1
        //TCC_CTRLA_CPTEN2
        //TCC_CTRLA_CPTEN3
    ; 
    while (TCC0->SYNCBUSY.bit.ENABLE);

    TCC0->WAVE.reg =
        TCC_WAVE_WAVEGEN(TCC_WAVE_WAVEGEN_MFRQ_Val)
    ; 
    while (TCC0->SYNCBUSY.bit.WAVE);

    TCC1->INTENSET.bit.OVF = 1;
    TCC1->CTRLA.reg =
        TCC_CTRLA_ENABLE
        |TCC_CTRLA_RESOLUTION(TCC_CTRLA_RESOLUTION_NONE_Val)
        |TCC_CTRLA_PRESCALER(TCC_CTRLA_PRESCALER_DIV1_Val)
        |TCC_CTRLA_RUNSTDBY
        |TCC_CTRLA_PRESCSYNC(TCC_CTRLA_PRESCSYNC_GCLK_Val)
        //|TCC_CTRLA_ALOCK
        //TCC_CTRLA_CPTEN0
        //TCC_CTRLA_CPTEN1
        //TCC_CTRLA_CPTEN2
        //TCC_CTRLA_CPTEN3
    ; 
    while (TCC1->SYNCBUSY.bit.ENABLE);

    TCC1->WAVE.reg =
        TCC_WAVE_WAVEGEN(TCC_WAVE_WAVEGEN_MFRQ_Val)
    ; 
    while (TCC1->SYNCBUSY.bit.WAVE);

    TCC2->INTENSET.bit.OVF = 1;
    TCC2->CTRLA.reg =
        TCC_CTRLA_ENABLE
        |TCC_CTRLA_RESOLUTION(TCC_CTRLA_RESOLUTION_NONE_Val)
        |TCC_CTRLA_PRESCALER(TCC_CTRLA_PRESCALER_DIV1_Val)
        |TCC_CTRLA_RUNSTDBY
        |TCC_CTRLA_PRESCSYNC(TCC_CTRLA_PRESCSYNC_GCLK_Val)
        //|TCC_CTRLA_ALOCK
        //TCC_CTRLA_CPTEN0
        //TCC_CTRLA_CPTEN1
        //TCC_CTRLA_CPTEN2
        //TCC_CTRLA_CPTEN3
    ; 
    while (TCC2->SYNCBUSY.bit.ENABLE);

    TCC2->WAVE.reg =
        TCC_WAVE_WAVEGEN(TCC_WAVE_WAVEGEN_MFRQ_Val)
    ; 
    while (TCC2->SYNCBUSY.bit.WAVE);


    TC3->COUNT16.INTENSET.bit.OVF = 1;
    TC3->COUNT16.CTRLA.reg =
        TC_CTRLA_ENABLE
        |TC_CTRLA_MODE(TC_CTRLA_MODE_COUNT16_Val)
        |TC_CTRLA_WAVEGEN(TC_CTRLA_WAVEGEN_MFRQ_Val)
        |TC_CTRLA_PRESCALER(TC_CTRLA_PRESCALER_DIV1_Val) //1Mhz
        |TC_CTRLA_RUNSTDBY
        |TC_CTRLA_PRESCSYNC(TC_CTRLA_PRESCSYNC_GCLK)
    ; 
    while (TC3->COUNT16.STATUS.bit.SYNCBUSY);

    TC4->COUNT16.INTENSET.bit.OVF = 1;
    TC4->COUNT16.CTRLA.reg =
        TC_CTRLA_ENABLE
        |TC_CTRLA_MODE(TC_CTRLA_MODE_COUNT16_Val)
        |TC_CTRLA_WAVEGEN(TC_CTRLA_WAVEGEN_MFRQ_Val)
        |TC_CTRLA_PRESCALER(TC_CTRLA_PRESCALER_DIV1_Val) //1Mhz
        |TC_CTRLA_RUNSTDBY
        |TC_CTRLA_PRESCSYNC(TC_CTRLA_PRESCSYNC_GCLK)
    ; 
    while (TC4->COUNT16.STATUS.bit.SYNCBUSY);

    TC5->COUNT16.INTENSET.bit.OVF = 1;
    TC5->COUNT16.CTRLA.reg =
        TC_CTRLA_ENABLE
        |TC_CTRLA_MODE(TC_CTRLA_MODE_COUNT16_Val)
        |TC_CTRLA_WAVEGEN(TC_CTRLA_WAVEGEN_MFRQ_Val)
        |TC_CTRLA_PRESCALER(TC_CTRLA_PRESCALER_DIV1_Val) //1Mhz
        |TC_CTRLA_RUNSTDBY
        |TC_CTRLA_PRESCSYNC(TC_CTRLA_PRESCSYNC_GCLK)
    ; 
    while (TC5->COUNT16.STATUS.bit.SYNCBUSY);

    //Configure TC/TCC Module Mode 

    TCC0->CTRLBSET.reg =
        TCC_CTRLBSET_CMD_NONE
        |TCC_CTRLBSET_DIR
        |TCC_CTRLBSET_ONESHOT
    ; 
    while (TCC0->SYNCBUSY.bit.CTRLB);

    TCC1->CTRLBSET.reg =
        TCC_CTRLBSET_CMD_NONE
        |TCC_CTRLBSET_DIR
        |TCC_CTRLBSET_ONESHOT
    ; 
    while (TCC1->SYNCBUSY.bit.CTRLB);

    TCC2->CTRLBSET.reg =
        TCC_CTRLBSET_CMD_NONE
        |TCC_CTRLBSET_DIR
        |TCC_CTRLBSET_ONESHOT
    ; 
    while (TCC2->SYNCBUSY.bit.CTRLB);

    TC3->COUNT16.CTRLBSET.reg =
        TC_CTRLBSET_CMD_NONE
        |TC_CTRLBSET_DIR
        |TC_CTRLBSET_ONESHOT
    ;
    while (TC3->COUNT16.STATUS.bit.SYNCBUSY); 

    TC4->COUNT16.CTRLBSET.reg =
        TC_CTRLBSET_CMD_NONE
        |TC_CTRLBSET_DIR
        |TC_CTRLBSET_ONESHOT
    ;
    while (TC4->COUNT16.STATUS.bit.SYNCBUSY); 

    TC5->COUNT16.CTRLBSET.reg =
        TC_CTRLBSET_CMD_NONE
        |TC_CTRLBSET_DIR
        |TC_CTRLBSET_ONESHOT
    ;
    while (TC5->COUNT16.STATUS.bit.SYNCBUSY); 

    //clear pending interrupt in TC Module
    TCC0->INTFLAG.bit.OVF = 1;
    TCC1->INTFLAG.bit.OVF = 1;
    TCC2->INTFLAG.bit.OVF = 1;

    TC3->COUNT16.INTFLAG.bit.OVF = 1;
    TC4->COUNT16.INTFLAG.bit.OVF = 1;
    TC5->COUNT16.INTFLAG.bit.OVF = 1;
    //enable interrupt in NVIC


#ifdef __TESTBUILD__
    NVIC_EnableIRQ(TCC0_IRQn);
    NVIC_EnableIRQ(TCC1_IRQn);
#endif

//Used by Openthread Alarm
#ifndef __TESTBUILD__
    NVIC_EnableIRQ(TCC2_IRQn);
    NVIC_EnableIRQ(TC3_IRQn);
#endif

    NVIC_EnableIRQ(TC4_IRQn);
    NVIC_EnableIRQ(TC5_IRQn);
}


void samr21Timer0Set(uint32_t value_us){
    while (TCC0->SYNCBUSY.reg); 
    TCC0->COUNT.reg = value_us;
    
    TCC0->CTRLBSET.reg =
        TCC_CTRLBSET_CMD_RETRIGGER
    ;   
} 

void samr21Timer0Stop(){ 
    while (TCC0->SYNCBUSY.reg);

    TCC0->CTRLBSET.reg =
        TCC_CTRLBSET_CMD_STOP
    ;
}  

void samr21Timer1Set(uint16_t value_us){ 
   while (TCC1->SYNCBUSY.reg); 
    TCC1->COUNT.reg = value_us;

    TCC1->CTRLBSET.reg =
        TCC_CTRLBSET_CMD_RETRIGGER
    ;   
} 

void samr21Timer1Stop(){ 
    while (TCC1->SYNCBUSY.reg);

    TCC1->CTRLBSET.reg =
        TCC_CTRLBSET_CMD_STOP
    ;
} 

void samr21Timer2Set(uint16_t value_us){
    while (TCC0->SYNCBUSY.reg); 

    TCC2->COUNT.reg = value_us;
    
    TCC2->CTRLBSET.reg =
        TCC_CTRLBSET_CMD_RETRIGGER
    ;   
} 

void samr21Timer2Stop(){ 
    while (TCC2->SYNCBUSY.reg); 

    TCC2->CTRLBSET.reg =
        TCC_CTRLBSET_CMD_STOP
    ;
} 

void samr21Timer3Set(uint16_t value_us){ 
    while (TC3->COUNT16.STATUS.bit.SYNCBUSY);

    TC3->COUNT16.COUNT.reg = value_us;

    TC3->COUNT16.CTRLBSET.reg =
        TC_CTRLBSET_CMD_RETRIGGER
    ;   
}   

void samr21Timer3Stop(){ 
    while (TC3->COUNT16.STATUS.bit.SYNCBUSY);

    TC3->COUNT16.CTRLBSET.reg =
        TC_CTRLBSET_CMD_STOP
    ;
}  

void samr21Timer4Set(uint16_t value_us){ 
    while (TC4->COUNT16.STATUS.bit.SYNCBUSY);

    TC4->COUNT16.COUNT.reg = value_us;

    TC4->COUNT16.CTRLBSET.reg =
        TC_CTRLBSET_CMD_RETRIGGER
    ;   
}   

void samr21Timer4Stop(){ 
    while (TC4->COUNT16.STATUS.bit.SYNCBUSY);

    TC4->COUNT16.CTRLBSET.reg =
        TC_CTRLBSET_CMD_STOP
    ;
}  

void samr21Timer5Set(uint16_t value_us){ 
    while (TC5->COUNT16.STATUS.bit.SYNCBUSY);

    TC5->COUNT16.COUNT.reg = value_us;

    TC5->COUNT16.CTRLBSET.reg =
        TC_CTRLBSET_CMD_RETRIGGER
    ; 
}   

void samr21Timer5Stop(){ 
    while (TC5->COUNT16.STATUS.bit.SYNCBUSY);
    
    TC5->COUNT16.CTRLBSET.reg =
        TC_CTRLBSET_CMD_STOP
    ;
}

//MOVED TO TODO
// void TCC0_Handler(){
//     TCC0->INTFLAG.bit.OVF = 1;
//     /*CODE*/
// }
//MOVED TO TODO

//MOVED TO TODO
// void TCC1_Handler(){
//     TCC1->INTFLAG.bit.OVF = 1;
//     /*CODE*/
// }
//MOVED TO TODO

//MOVED TO TODO
// void TCC2_Handler(){
//     TCC2->INTFLAG.bit.OVF = 1;
//     /*CODE*/
// }
//MOVED TO TODO

//MOVED TO otPlatAlarm.c
// void TC3_Handler(){
//     TC3->COUNT16.INTFLAG.bit.OVF = 1;
//     /*CODE*/
// }
//MOVED TO otPlatAlarm.c

//MOVED TO samr21RadioFSM.c
// void TC4_Handler(){
//     TC4->COUNT16.INTFLAG.bit.OVF = 1;
//     /*CODE*/
// }
//MOVED TO samr21RadioFSM.c
// void TC5_Handler(){
//     TC5->COUNT16.INTFLAG.bit.OVF = 1;
//     /*CODE*/
// }
//MOVED TO samr21RadioFSM.c

