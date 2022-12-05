//Author Eric Härtel @ dresden elektronik ingenieurtechnik gmbh © 2022
#include "samr21Timer.h"

void samr21TimerInit(){

    //Disable First
    TC4->COUNT16.CTRLA.bit.ENABLE = 0;
    while (TC4->COUNT16.STATUS.bit.SYNCBUSY);
    TC5->COUNT16.CTRLA.bit.ENABLE = 0;
    while (TC5->COUNT16.STATUS.bit.SYNCBUSY);

    //Reset after
    TC4->COUNT16.CTRLA.bit.SWRST = 1;
    while (TC4->COUNT16.CTRLA.bit.SWRST || TC4->COUNT16.STATUS.bit.SYNCBUSY);
    TC5->COUNT16.CTRLA.bit.SWRST = 1;
    while (TC5->COUNT16.CTRLA.bit.SWRST || TC5->COUNT16.STATUS.bit.SYNCBUSY);


    //enable interrupt
    TC4->COUNT16.INTENSET.bit.OVF = 1;
    TC4->COUNT16.CTRLA.reg =
        TC_CTRLA_ENABLE
        |TC_CTRLA_MODE(TC_CTRLA_MODE_COUNT16_Val)
        |TC_CTRLA_WAVEGEN(TC_CTRLA_WAVEGEN_MFRQ_Val)
        |TC_CTRLA_PRESCALER(TC_CTRLA_PRESCALER_DIV1_Val) //16MHz -> 1Mhz
        |TC_CTRLA_RUNSTDBY
        |TC_CTRLA_PRESCSYNC(TC_CTRLA_PRESCSYNC_GCLK)
    ; 
    while (TC4->COUNT16.STATUS.bit.SYNCBUSY);

    TC5->COUNT16.INTENSET.bit.OVF = 1;
    TC5->COUNT16.CTRLA.reg =
        TC_CTRLA_ENABLE
        |TC_CTRLA_MODE(TC_CTRLA_MODE_COUNT16_Val)
        |TC_CTRLA_WAVEGEN(TC_CTRLA_WAVEGEN_MFRQ_Val)
        |TC_CTRLA_PRESCALER(TC_CTRLA_PRESCALER_DIV1_Val) //16MHz -> 1Mhz
        |TC_CTRLA_RUNSTDBY
        |TC_CTRLA_PRESCSYNC(TC_CTRLA_PRESCSYNC_GCLK)
    ; 
    while (TC5->COUNT16.STATUS.bit.SYNCBUSY);

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
    while (TC4->COUNT16.STATUS.bit.SYNCBUSY); 

    while (TC4->COUNT16.STATUS.bit.SYNCBUSY); 

    TC4->COUNT16.INTFLAG.bit.OVF = 1;
    TC5->COUNT16.INTFLAG.bit.OVF = 1;

    NVIC_EnableIRQ(TC4_IRQn);
    NVIC_EnableIRQ(TC5_IRQn);
}

void samr21Timer4Set(uint16_t value_us){ 
    while (TC4->COUNT16.STATUS.bit.SYNCBUSY);

    TC4->COUNT16.COUNT.reg = value_us;

    TC4->COUNT16.CTRLBSET.reg =
        TC_CTRLBSET_CMD_RETRIGGER
    ;   
}   

void samr21Timer4Stop(){ 
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
    TC5->COUNT16.CTRLBSET.reg =
        TC_CTRLBSET_CMD_STOP
    ;
}
  
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

