//Author Eric Härtel @ dresden elektronik ingenieurtechnik gmbh © 2022

#ifndef _SAMR21_OT_PLATFORM_H_
#define _SAMR21_OT_PLATFORM_H_


//Handler that checks for new USB/UART Packages
void samr21OtPlatCommReceiveTask();

//Handler that notifies upper layer about received Frames
void samr21OtPlatRadioReceiveTask();
#endif //_SAMR21_OT_PLATFORM_H_