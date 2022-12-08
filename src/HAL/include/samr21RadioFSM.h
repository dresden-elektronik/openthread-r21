//Author Eric Härtel @ dresden elektronik ingenieurtechnik gmbh © 2022
#include "samr21.h"
#include "samr21Radio.h"
#include <stdbool.h>

#ifndef _SAMR21_RADIO_FSM_H_
#define _SAMR21_RADIO_FSM_H_

#ifndef NULL
	#define NULL 0
#endif


typedef enum RadioJobState{
    RADIO_JOB_STATE_IDLE                          = 0x00,
    RADIO_JOB_STATE_IN_SETUP                      = 0x01,

    //RX States
    MARKER_RADIO_JOB_STATES_BEGINN_RX             = 0x10,
    RADIO_JOB_STATE_RX_IDLE                       = 0x11,
    RADIO_JOB_STATE_RX_LIVE_MSG_PARSER            = 0x12,
    RADIO_JOB_STATE_RX_SENDING_ACK                = 0x13,
    MARKER_RADIO_JOB_STATES_END_RX                = 0x1F,
    
    //TX States
    MARKER_RADIO_JOB_STATES_BEGINN_TX             = 0x20,
    RADIO_JOB_STATE_TX_READY                      = 0x21,
    RADIO_JOB_STATE_TX_CCA                        = 0x22,
    RADIO_JOB_STATE_TX_CCA_EVAL                   = 0x23,  
    RADIO_JOB_STATE_TX_CCA_BACKOFF                = 0x24,
    RADIO_JOB_STATE_TX_SENDING                    = 0x25, 
    RADIO_JOB_STATE_TX_SENDING_ACK_REQUESTED      = 0x26,
    RADIO_JOB_STATE_TX_WAITING_FOR_ACK            = 0x27,
    RADIO_JOB_STATE_TX_RECEIVING_ACK              = 0x28,
    RADIO_JOB_STATE_TX_EVAL_ACK                   = 0x29,
    RADIO_JOB_STATE_TX_EVAL_RETRANSMISSIOM        = 0x2A,
    MARKER_RADIO_JOB_STATES_END_TX                = 0x2F,

    //ED States
    MARKER_RADIO_JOB_STATES_BEGINN_ED             = 0x30,
    RADIO_JOB_STATE_ED_READY                      = 0x31,
    RADIO_JOB_STATE_REQUEST_ED                    = 0x32,
    RADIO_JOB_STATE_WAIT_FOR_ED_RESULT            = 0x33,
    RADIO_JOB_STATE_EVAL_ED_RESULT                = 0x34,
    RADIO_JOB_STATE_EVAL_ED_CONTINUATION          = 0x35,
    MARKER_RADIO_JOB_STATES_END_ED                = 0x3F,

    //To Be Evaluated by Application States
    MARKER_RADIO_JOB_STATES_BEGINN_OTHER_DONE     = 0xF0,
    RADIO_JOB_STATE_RX_DONE                       = 0xF1,
    RADIO_JOB_STATE_TX_DONE                       = 0xF2,
    RADIO_JOB_STATE_ED_DONE                       = 0xF3,
    RADIO_JOB_STATE_TX_FAILED                     = 0xFA,
    RADIO_JOB_STATE_ED_FAILED                     = 0xFB,
    MARKER_RADIO_JOB_STATES_END_OTHER_DONE        = 0xFF,
} RadioJobState;

typedef enum RadioEvent{
    RADIO_EVENT_NONE                          = 0x00,
    RADIO_EVENT_ERROR                         = 0x0E,
    MARKER_RADIO_EVENTS_END_MISC              = 0x0F,

    //From TRX (AT86rf233)
    MARKER_RADIO_EVENTS_BEGINN_TRX_IRQ        = 0x10,
    RADIO_EVENT_IRQ_PLL_LOCK                  = 0x11,
    RADIO_EVENT_IRQ_PLL_UNLOCK                = 0x12,
    RADIO_EVENT_IRQ_RX_START                  = 0x13,
    RADIO_EVENT_IRQ_TRX_END                   = 0x14, 
    RADIO_EVENT_IRQ_CCA_ED_DONE               = 0x15,
    RADIO_EVENT_IRQ_RX_ADDRESS_MATCH          = 0x16,
    RADIO_EVENT_IRQ_BUFFER_READ_UNDERRUN      = 0x17,
    RADIO_EVENT_IRQ_BAT_LOW                   = 0x18,
    MARKER_RADIO_EVENTS_END_TRX_IRQ           = 0x1F,

    //From Timer Modules
    MARKER_RADIO_EVENTS_BEGINN_TIMER_IRQ      = 0x20,
    RADIO_EVENT_TIMER_TRIGGER                 = 0x21, //Timer4
    RADIO_EVENT_TIMEOUT_TRIGGER               = 0x22, //Timer5
    MARKER_RADIO_EVENTS_END_TIMMER_IRQ        = 0x2F,

    //Software Events
    MARKER_RADIO_EVENTS_BEGINN_SOFTEVENTS     = 0x30,
    RADIO_SOFTEVENT_START_TX                  = 0x31,
    RADIO_SOFTEVENT_NO_RETRYS_LEFT            = 0x32,
    RADIO_SOFTEVENT_ACK_VALID                 = 0x33,
    RADIO_SOFTEVENT_ACK_INVALID               = 0x34,
    RADIO_SOFTEVENT_MSG_VALID                 = 0x35,
    RADIO_SOFTEVENT_MSG_INVALID               = 0x36,
    RADIO_SOFTEVENT_CHANNEL_CLEAR             = 0x37,
    RADIO_SOFTEVENT_CHANNEL_BUSY              = 0x38,
    RADIO_SOFTEVENT_ACK_REQUESTED             = 0x39,
    RADIO_SOFTEVENT_START_ED                  = 0x3A,
    RADIO_SOFTEVENT_STOP_ED                   = 0x3B,
    MARKER_RADIO_EVENTS_END_SOFTEVENTS        = 0x3F,
} RadioEvent;

typedef struct {
	RadioJobState state;
	RadioEvent event;
	RadioJobState nextState;
	void (*transitionFunction)(void);
} fsmItem;


//Turns the FSM on or off
void samr21RadioFsmEnable(bool enable);

//Called from Event Handler Functions or by Application to start a TX procedure
void samr21RadioFsmHandleEvent(RadioEvent event);

//Called from within a transition function
void samr21RadioFsmQueueSoftEvent(RadioEvent event);
void samr21RadioFsmChangeJobStatePointer(RadioJobState* newActiveJobStatePtr);

#endif //_SAMR21_RADIO_FSM_H_