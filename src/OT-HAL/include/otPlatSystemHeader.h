//Author Eric Härtel @ dresden elektronik ingenieurtechnik gmbh © 2022

#ifndef _SAMR21_OT_PLATFORM_H_
#define _SAMR21_OT_PLATFORM_H_


//Handler that checks for new USB/UART Packages
void samr21OtPlat_uartCommTask();

//Handler that notifies upper layer about received Frames and the status of ongoing transmissions
void samr21OtPlat_radioTick();

//Inits Timer for OT Orchestration
void samr21OtPlat_alarmInit();

//Handles Timer for OT Orchestration
void samr21OtPlat_alarmTask();


//Handler that Transmit Frames to the Host device
#endif //_SAMR21_OT_PLATFORM_H_
