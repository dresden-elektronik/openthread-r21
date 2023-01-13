//Author Eric Härtel @ dresden elektronik ingenieurtechnik gmbh © 2022
#ifndef _SAMR21_RADIO_IRQ_HANDLER_H_
#define _SAMR21_RADIO_IRQ_HANDLER_H_

#include "samr21.h"
#include "at86rf233_Bitfield.h"
#include "at86rf233.h"
#include "samr21RadioTrxRegCopy.h"

typedef enum{
    //From TRX (AT86rf233)
    TRX_EVENT_PLL_LOCK                  = 0x01,
    TRX_EVENT_PLL_UNLOCK                = 0x02,
    TRX_EVENT_RX_START                  = 0x03,
    TRX_EVENT_TRX_END                   = 0x04, 
    TRX_EVENT_CCA_ED_DONE               = 0x05,
    TRX_EVENT_RX_ADDRESS_MATCH          = 0x06,
    TRX_EVENT_BUFFER_READ_UNDERRUN      = 0x07,
    TRX_EVENT_BAT_LOW                   = 0x08,

    //From Timer Modules
    TIMER_EVENT_3_TRIGGER               = 0x11,
    TIMER_EVENT_4_TRIGGER               = 0x12,
    TIMER_EVENT_5_TRIGGER               = 0x13
} IrqEvent;


typedef void (*EventHandlerFunc)(IrqEvent);

void samr21RadioSetEventHandler(EventHandlerFunc eventHandlerFunc);
void samr21RadioRemoveEventHandler();


#endif //_SAMR21_RADIO_IRQ_HANDLER_H_