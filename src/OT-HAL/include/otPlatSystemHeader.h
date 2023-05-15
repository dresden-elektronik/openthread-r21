//Author Eric Härtel @ dresden elektronik ingenieurtechnik gmbh © 2022

#ifndef _SAMR21_OT_PLATFORM_H_
#define _SAMR21_OT_PLATFORM_H_


//Handler that checks for new USB/UART Packages
void samr21OtPlatCommTask();

//Handler that notifies upper layer about received Frames and the status of ongoing transmissions
void samr21OtPlatRadioTask();

//Inits Timer for OT Orchestration
void samr21OtPlatAlarmInit();

//Handles Timer for OT Orchestration
void samr21OtPlatAlarmTask();


//Handler that Transmit Frames to the Host device
#endif //_SAMR21_OT_PLATFORM_H_
