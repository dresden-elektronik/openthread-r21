//Author Eric Härtel @ dresden elektronik ingenieurtechnik gmbh © 2022
#include "samr21Timer.h"

void samr21TimerInit(){

    //Disable Module First
    TC3->COUNT16.CTRLA.bit.ENABLE = 0;
    while (TC3->COUNT16.STATUS.bit.SYNCBUSY);
    TC4->COUNT16.CTRLA.bit.ENABLE = 0;
    while (TC4->COUNT16.STATUS.bit.SYNCBUSY);
    TC5->COUNT16.CTRLA.bit.ENABLE = 0;
    while (TC5->COUNT16.STATUS.bit.SYNCBUSY);

    //Reset after
    TC3->COUNT16.CTRLA.bit.SWRST = 1;
    while (TC3->COUNT16.CTRLA.bit.SWRST || TC3->COUNT16.STATUS.bit.SYNCBUSY);
    TC4->COUNT16.CTRLA.bit.SWRST = 1;
    while (TC4->COUNT16.CTRLA.bit.SWRST || TC4->COUNT16.STATUS.bit.SYNCBUSY);
    TC5->COUNT16.CTRLA.bit.SWRST = 1;
    while (TC5->COUNT16.CTRLA.bit.SWRST || TC5->COUNT16.STATUS.bit.SYNCBUSY);


    //Setup TC Modules
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


    //Configure TC Module Mode 
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


    //enable interrupt in TC Module
    TC3->COUNT16.INTFLAG.bit.OVF = 1;
    TC4->COUNT16.INTFLAG.bit.OVF = 1;
    TC5->COUNT16.INTFLAG.bit.OVF = 1;
    //enable interrupt in NVIC
    NVIC_EnableIRQ(TC3_IRQn);
    NVIC_EnableIRQ(TC4_IRQn);
    NVIC_EnableIRQ(TC5_IRQn);
}

void samr21Timer3Set(uint16_t value_us){ 
    while (TC3->COUNT16.STATUS.bit.SYNCBUSY);

    TC3->COUNT16.COUNT.reg = value_us;

    TC3->COUNT16.CTRLBSET.reg =
        TC_CTRLBSET_CMD_RETRIGGER
    ;   
}   

void samr21Timer3Stop(){ 
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

