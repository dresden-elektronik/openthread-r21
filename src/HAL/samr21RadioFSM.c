//Author Eric Härtel @ dresden elektronik ingenieurtechnik gmbh © 2022
#include "samr21RadioFSM.h"

static const fsmItem s_fsmTxItemTable[]={

        {RADIO_STATE_TX_READY,                
            RADIO_SOFTEVENT_START_TX,     
                RADIO_STATE_TX_CCA,      
        samr21RadioStartCCA},  

            {RADIO_STATE_TX_CCA,                
                RADIO_EVENT_IRQ_CCA_ED_DONE,     
                    RADIO_STATE_TX_CCA_EVAL,      
            samr21RadioEvalCCA},  

                {RADIO_STATE_TX_CCA_EVAL,        
                    RADIO_SOFTEVENT_CHANNEL_BUSY,  
                        RADIO_STATE_TX_CCA_BACKOFF, 
                samr21RadioStartBackoffTimer},

                    {RADIO_STATE_TX_CCA_BACKOFF,        
                        RADIO_EVENT_TIMER_TRIGGER,  
                            RADIO_STATE_TX_CCA, 
            /*----*/samr21RadioStartCCA},    

                    {RADIO_STATE_TX_CCA_BACKOFF,        
                        RADIO_EVENT_TIMEOUT_TRIGGER,  
                            RADIO_STATE_TX_FAILED, 
    /*------------*/samr21RadioTransmissionCleanup}, 
                    
                {RADIO_STATE_TX_CCA_EVAL,        
                    RADIO_SOFTEVENT_CHANNEL_CLEAR,  
                        RADIO_STATE_TX_SENDING, 
                samr21RadioSendTXPayload},

                    {RADIO_STATE_TX_SENDING,        
                        RADIO_EVENT_IRQ_TRX_END,  
                            RADIO_STATE_TX_DONE, 
        /*--------*/samr21RadioTransmissionCleanup}, 

                    {RADIO_STATE_TX_SENDING,        
                        RADIO_EVENT_TIMEOUT_TRIGGER,  
                            RADIO_STATE_TX_FAILED, 
    /*------------*/samr21RadioTransmissionCleanup}, 

                    {RADIO_STATE_TX_SENDING,        
                        RADIO_SOFTEVENT_ACK_REQUESTED,  
                            RADIO_STATE_TX_SENDING_ACK_REQUESTED, 
                    NULL}, 

                        {RADIO_STATE_TX_SENDING_ACK_REQUESTED,        
                            RADIO_EVENT_IRQ_TRX_END,  
                                RADIO_STATE_TX_WAITING_FOR_ACK, 
                        samr21RadioWaitForAck}, 

                        {RADIO_STATE_TX_SENDING_ACK_REQUESTED,        
                            RADIO_EVENT_TIMEOUT_TRIGGER,  
                                RADIO_STATE_TX_FAILED, 
    /*----------------*/samr21RadioTransmissionCleanup},

                            {RADIO_STATE_TX_WAITING_FOR_ACK,        
                                RADIO_EVENT_TIMER_TRIGGER,  
                                    RADIO_STATE_TX_EVAL_RETRANSMISSIOM, 
                            samr21RadioEvalRetransmission}, 

                                {RADIO_STATE_TX_EVAL_RETRANSMISSIOM,        
                                    RADIO_SOFTEVENT_START_TX,  
                                        RADIO_STATE_TX_CCA, 
            /*----------------*/samr21RadioStartCCA}, 

                                {RADIO_STATE_TX_EVAL_RETRANSMISSIOM,        
                                    RADIO_SOFTEVENT_NO_RETRYS_LEFT,  
                                        RADIO_STATE_TX_FAILED, 
    /*------------------------*/samr21RadioTransmissionCleanup}, 

                                {RADIO_STATE_TX_EVAL_RETRANSMISSIOM,        
                                    RADIO_EVENT_TIMEOUT_TRIGGER,  
                                        RADIO_STATE_TX_FAILED, 
    /*------------------------*/samr21RadioTransmissionCleanup},
                
                            {RADIO_STATE_TX_WAITING_FOR_ACK,        
                                RADIO_EVENT_IRQ_RX_START,  
                                    RADIO_STATE_TX_RECEIVING_ACK, 
                            samr21RadioAckReceptionStarted},

                                {RADIO_STATE_TX_RECEIVING_ACK,        
                                    RADIO_EVENT_IRQ_TRX_END,  
                                        RADIO_STATE_TX_EVAL_ACK, 
                                samr21RadioEvalAck},

                                    {RADIO_STATE_TX_EVAL_ACK,        
                                        RADIO_SOFTEVENT_ACK_INVALID,  
                                            RADIO_STATE_TX_CCA, 
            /*--------------------*/samr21RadioStartCCA},

                                    {RADIO_STATE_TX_EVAL_ACK,        
                                        RADIO_SOFTEVENT_ACK_VALID,  
                                            RADIO_STATE_TX_DONE, 
    /*----------------------------*/samr21RadioTransmissionCleanup},

                                    {RADIO_STATE_TX_EVAL_ACK,        
                                        RADIO_SOFTEVENT_NO_RETRYS_LEFT,  
                                            RADIO_STATE_TX_FAILED, 
    /*----------------------------*/samr21RadioTransmissionCleanup},

                                    {RADIO_STATE_TX_EVAL_ACK,        
                                        RADIO_EVENT_TIMEOUT_TRIGGER,  
                                            RADIO_STATE_TX_FAILED, 
    /*---------------------------*/samr21RadioTransmissionCleanup},

                            {RADIO_STATE_TX_WAITING_FOR_ACK,        
                                    RADIO_EVENT_TIMEOUT_TRIGGER,  
                                        RADIO_STATE_TX_FAILED, 
    /*---------------------*/samr21RadioTransmissionCleanup},


                {RADIO_STATE_TX_CCA_EVAL,        
                    RADIO_SOFTEVENT_NO_RETRYS_LEFT,  
                        RADIO_STATE_TX_FAILED, 
    /*--------*/samr21RadioTransmissionCleanup},  


                {RADIO_STATE_TX_CCA_EVAL,        
                    RADIO_EVENT_TIMEOUT_TRIGGER,  
                        RADIO_STATE_TX_FAILED, 
    /*--------*/samr21RadioTransmissionCleanup}, 

            {RADIO_STATE_TX_CCA,        
                RADIO_EVENT_TIMEOUT_TRIGGER,  
                    RADIO_STATE_TX_FAILED, 
    /*----*/samr21RadioTransmissionCleanup}, 
    

    //Final States   
    {RADIO_STATE_TX_FAILED,                
        RADIO_EVENT_NONE,     
            RADIO_STATE_TX_FAILED,      
    NULL},

    {RADIO_STATE_TX_DONE,                
        RADIO_EVENT_NONE,     
            RADIO_STATE_TX_DONE,      
    NULL}
};
#define RADIO_STATE_TX_LAST RADIO_STATE_TX_DONE

static const fsmItem s_fsmRxItemTable[]={

        {RADIO_STATE_IDLE,                
            RADIO_EVENT_IRQ_RX_START,     
                RADIO_STATE_RX_LIVE_MSG_PARSER,      
        samr21RadioLiveRxParser},

            {RADIO_STATE_RX_LIVE_MSG_PARSER,                
                RADIO_SOFTEVENT_ACK_REQUESTED,     
                    RADIO_STATE_RX_SENDING_ACK,      
            samr21RadioSendAck},

                {RADIO_STATE_RX_SENDING_ACK,                
                    RADIO_EVENT_IRQ_TRX_END,     
                        RADIO_STATE_RX_DONE,      
    /*--------*/samr21RadioTransmissionCleanup},

            {RADIO_STATE_RX_LIVE_MSG_PARSER,                
                RADIO_EVENT_TIMER_TRIGGER,     
                    RADIO_STATE_IDLE,      
        /**/samr21RadioAbortLiveRxParser},

            {RADIO_STATE_RX_LIVE_MSG_PARSER,                
                RADIO_SOFTEVENT_MSG_INVALID,     
                    RADIO_STATE_IDLE,      
        /**/NULL},

            {RADIO_STATE_RX_LIVE_MSG_PARSER,                
                RADIO_SOFTEVENT_MSG_VALID,     
                    RADIO_STATE_RX_DONE,      
    /*----*/samr21RadioTransmissionCleanup},

    //Final States
    {RADIO_STATE_RX_DONE,                
        RADIO_EVENT_NONE,     
            RADIO_STATE_RX_DONE,      
    NULL}
};
#define RADIO_STATE_RX_LAST RADIO_STATE_RX_DONE

static volatile RadioEvent s_softEvent = RADIO_EVENT_NONE;
static volatile RadioJobState * s_currentActiveStatePtr = NULL;

void samr21RadioFsmHandleEvent(RadioEvent event)
{
    RadioJobState * currentStatePtr = s_currentActiveStatePtr;


    __NVIC_DisableIRQ(EIC_IRQn); 
    //Look for RX States first, cause timinig is more critical
    if (*(currentStatePtr) < MARKER_RADIO_STATES_END_RX){

beginFsmRxHandler:

        for (uint8_t i = 0; s_fsmRxItemTable[i].state != RADIO_STATE_RX_LAST; i++)
        {		
            if(s_fsmRxItemTable[i].state != *(currentStatePtr)){
                continue;
            }
            
            if(event != s_fsmRxItemTable[i].event){
                continue;
            }

            
            //Check for Transition Function 
            if(s_fsmRxItemTable[i].transitionFunction != NULL){
                s_fsmRxItemTable[i].transitionFunction();   
            }
                
            //Update State
            if(*(currentStatePtr) < MARKER_RADIO_STATES_BEGINN_OTHER_DONE){
                *(currentStatePtr) = s_fsmRxItemTable[i].nextState;
            }

            //Refresh State Pointer cause a change could have been invoked by a transition function
            currentStatePtr = s_currentActiveStatePtr;

            //Check for Soft Events
            if(s_softEvent){

                event = s_softEvent;
                s_softEvent = RADIO_EVENT_NONE;

                goto beginFsmRxHandler;
            }

            goto exitFsm;
        }
    }

beginFsmTxHandler:

    for (uint8_t i = 0; s_fsmTxItemTable[i].state != RADIO_STATE_TX_LAST; i++)
    {		
        if(s_fsmTxItemTable[i].state != *(currentStatePtr)){
            continue;
        }
        
        if(event != s_fsmTxItemTable[i].event){
            continue;
        }

        //Check for Transition Function       
        if(s_fsmTxItemTable[i].transitionFunction != NULL){
            s_fsmTxItemTable[i].transitionFunction();
        }  
        
        //Update State
        if(*(currentStatePtr) < MARKER_RADIO_STATES_BEGINN_OTHER_DONE){
            *(currentStatePtr) = s_fsmTxItemTable[i].nextState;
        }

        //Refresh State Pointer cause a change could have been invoked by a transition function
        currentStatePtr = s_currentActiveStatePtr;

        //Check for Soft Events
        if(s_softEvent){

            event = s_softEvent;
            s_softEvent = RADIO_EVENT_NONE;

            goto beginFsmTxHandler;
        }	

        goto exitFsm;
    }

exitFsm:

    __NVIC_EnableIRQ(EIC_IRQn);  
    return; 
}

void samr21RadioFsmQueueSoftEvent(RadioEvent event){
    s_softEvent = event;
}

void samr21RadioFsmChangeJobStatePointer(RadioJobState* newActiveJobStatePtr){
    s_currentActiveStatePtr = newActiveJobStatePtr;
}