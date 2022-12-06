//Author Eric Härtel @ dresden elektronik ingenieurtechnik gmbh © 2022
#include "samr21RadioFSM.h"

volatile static bool s_fsmEnabled = false;

void samr21RadioFsmEnable(bool enable){
    s_fsmEnabled = enable;
}


static const fsmItem s_fsmTxItemTable[]={

        {RADIO_STATE_TX_READY,                
            RADIO_SOFTEVENT_START_TX,     
                RADIO_STATE_TX_CCA,      
        fsm_func_samr21RadioStartCCA},  

            {RADIO_STATE_TX_CCA,                
                RADIO_EVENT_IRQ_CCA_ED_DONE,     
                    RADIO_STATE_TX_CCA_EVAL,      
            fsm_func_samr21RadioEvalCCA},  

                {RADIO_STATE_TX_CCA_EVAL,        
                    RADIO_SOFTEVENT_CHANNEL_BUSY,  
                        RADIO_STATE_TX_CCA_BACKOFF, 
                fsm_func_samr21RadioStartBackoffTimer},

                    {RADIO_STATE_TX_CCA_BACKOFF,        
                        RADIO_EVENT_TIMER_TRIGGER,  
                            RADIO_STATE_TX_CCA, 
            /*----*/fsm_func_samr21RadioStartCCA},    

                    {RADIO_STATE_TX_CCA_BACKOFF,        
                        RADIO_EVENT_TIMEOUT_TRIGGER,  
                            RADIO_STATE_TX_FAILED, 
    /*------------*/fsm_func_samr21RadioJobCleanup}, 
                    
                {RADIO_STATE_TX_CCA_EVAL,        
                    RADIO_SOFTEVENT_CHANNEL_CLEAR,  
                        RADIO_STATE_TX_SENDING, 
                fsm_func_samr21RadioSendTXPayload},

                    {RADIO_STATE_TX_SENDING,        
                        RADIO_EVENT_IRQ_TRX_END,  
                            RADIO_STATE_TX_DONE, 
        /*--------*/fsm_func_samr21RadioJobCleanup}, 

                    {RADIO_STATE_TX_SENDING,        
                        RADIO_EVENT_TIMEOUT_TRIGGER,  
                            RADIO_STATE_TX_FAILED, 
    /*------------*/fsm_func_samr21RadioJobCleanup}, 

                    {RADIO_STATE_TX_SENDING,        
                        RADIO_SOFTEVENT_ACK_REQUESTED,  
                            RADIO_STATE_TX_SENDING_ACK_REQUESTED, 
                    NULL}, 

                        {RADIO_STATE_TX_SENDING_ACK_REQUESTED,        
                            RADIO_EVENT_IRQ_TRX_END,  
                                RADIO_STATE_TX_WAITING_FOR_ACK, 
                        fsm_func_samr21RadioWaitForAck}, 

                        {RADIO_STATE_TX_SENDING_ACK_REQUESTED,        
                            RADIO_EVENT_TIMEOUT_TRIGGER,  
                                RADIO_STATE_TX_FAILED, 
    /*----------------*/fsm_func_samr21RadioJobCleanup},

                            {RADIO_STATE_TX_WAITING_FOR_ACK,        
                                RADIO_EVENT_TIMER_TRIGGER,  
                                    RADIO_STATE_TX_EVAL_RETRANSMISSIOM, 
                            fsm_func_samr21RadioEvalRetransmission}, 

                                {RADIO_STATE_TX_EVAL_RETRANSMISSIOM,        
                                    RADIO_SOFTEVENT_START_TX,  
                                        RADIO_STATE_TX_CCA, 
            /*----------------*/fsm_func_samr21RadioStartCCA}, 

                                {RADIO_STATE_TX_EVAL_RETRANSMISSIOM,        
                                    RADIO_SOFTEVENT_NO_RETRYS_LEFT,  
                                        RADIO_STATE_TX_FAILED, 
    /*------------------------*/fsm_func_samr21RadioJobCleanup}, 

                                {RADIO_STATE_TX_EVAL_RETRANSMISSIOM,        
                                    RADIO_EVENT_TIMEOUT_TRIGGER,  
                                        RADIO_STATE_TX_FAILED, 
    /*------------------------*/fsm_func_samr21RadioJobCleanup},
                
                            {RADIO_STATE_TX_WAITING_FOR_ACK,        
                                RADIO_EVENT_IRQ_RX_START,  
                                    RADIO_STATE_TX_RECEIVING_ACK, 
                            fsm_func_samr21RadioAckReceptionStarted},

                                {RADIO_STATE_TX_RECEIVING_ACK,        
                                    RADIO_EVENT_IRQ_TRX_END,  
                                        RADIO_STATE_TX_EVAL_ACK, 
                                fsm_func_samr21RadioEvalAck},

                                    {RADIO_STATE_TX_EVAL_ACK,        
                                        RADIO_SOFTEVENT_ACK_INVALID,  
                                            RADIO_STATE_TX_CCA, 
            /*--------------------*/fsm_func_samr21RadioStartCCA},

                                    {RADIO_STATE_TX_EVAL_ACK,        
                                        RADIO_SOFTEVENT_ACK_VALID,  
                                            RADIO_STATE_TX_DONE, 
    /*----------------------------*/fsm_func_samr21RadioJobCleanup},

                                    {RADIO_STATE_TX_EVAL_ACK,        
                                        RADIO_SOFTEVENT_NO_RETRYS_LEFT,  
                                            RADIO_STATE_TX_FAILED, 
    /*----------------------------*/fsm_func_samr21RadioJobCleanup},

                                    {RADIO_STATE_TX_EVAL_ACK,        
                                        RADIO_EVENT_TIMEOUT_TRIGGER,  
                                            RADIO_STATE_TX_FAILED, 
    /*---------------------------*/fsm_func_samr21RadioJobCleanup},

                            {RADIO_STATE_TX_WAITING_FOR_ACK,        
                                    RADIO_EVENT_TIMEOUT_TRIGGER,  
                                        RADIO_STATE_TX_FAILED, 
    /*---------------------*/fsm_func_samr21RadioJobCleanup},


                {RADIO_STATE_TX_CCA_EVAL,        
                    RADIO_SOFTEVENT_NO_RETRYS_LEFT,  
                        RADIO_STATE_TX_FAILED, 
    /*--------*/fsm_func_samr21RadioJobCleanup},  


                {RADIO_STATE_TX_CCA_EVAL,        
                    RADIO_EVENT_TIMEOUT_TRIGGER,  
                        RADIO_STATE_TX_FAILED, 
    /*--------*/fsm_func_samr21RadioJobCleanup}, 

            {RADIO_STATE_TX_CCA,        
                RADIO_EVENT_TIMEOUT_TRIGGER,  
                    RADIO_STATE_TX_FAILED, 
    /*----*/fsm_func_samr21RadioJobCleanup}, 
    

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
        fsm_func_samr21RadioLiveRxParser},

            {RADIO_STATE_RX_LIVE_MSG_PARSER,                
                RADIO_SOFTEVENT_ACK_REQUESTED,     
                    RADIO_STATE_RX_SENDING_ACK,      
            fsm_func_samr21RadioSendAck},

                {RADIO_STATE_RX_SENDING_ACK,                
                    RADIO_EVENT_IRQ_TRX_END,     
                        RADIO_STATE_RX_DONE,      
    /*--------*/fsm_func_samr21RadioJobCleanup},

            {RADIO_STATE_RX_LIVE_MSG_PARSER,                
                RADIO_EVENT_TIMER_TRIGGER,     
                    RADIO_STATE_IDLE,      
        /**/fsm_func_samr21RadioAbortLiveRxParser},

            {RADIO_STATE_RX_LIVE_MSG_PARSER,                
                RADIO_SOFTEVENT_MSG_INVALID,     
                    RADIO_STATE_IDLE,      
        /**/NULL},

            {RADIO_STATE_RX_LIVE_MSG_PARSER,                
                RADIO_SOFTEVENT_MSG_VALID,     
                    RADIO_STATE_RX_DONE,      
    /*----*/fsm_func_samr21RadioJobCleanup},

    //Final States
    {RADIO_STATE_RX_DONE,                
        RADIO_EVENT_NONE,     
            RADIO_STATE_RX_DONE,      
    NULL}
};
#define RADIO_STATE_RX_LAST RADIO_STATE_RX_DONE

static const fsmItem s_fsmEdItemTable[]={

        {RADIO_STATE_ED_READY,                
            RADIO_SOFTEVENT_START_ED,     
                RADIO_STATE_WAIT_FOR_ED_RESULT,      
        fsm_func_samr21StartEd},

            {RADIO_STATE_WAIT_FOR_ED_RESULT,                
                RADIO_EVENT_IRQ_CCA_ED_DONE,     
                    RADIO_STATE_EVAL_ED_RESULT,      
            fsm_func_samr21EvalEd},

                {RADIO_STATE_EVAL_ED_RESULT,                
                    RADIO_EVENT_TIMER_TRIGGER,     
                        RADIO_STATE_EVAL_ED_CONTINUATION,      
                fsm_func_samr21EvalEdContinuation},

                    {RADIO_STATE_EVAL_ED_CONTINUATION,                
                        RADIO_SOFTEVENT_START_ED,     
                            RADIO_STATE_WAIT_FOR_ED_RESULT,      
        /*--------*/fsm_func_samr21StartEd},

                    {RADIO_STATE_EVAL_ED_CONTINUATION,                
                        RADIO_SOFTEVENT_STOP_ED,     
                            RADIO_STATE_ED_DONE,      
    /*------------*/fsm_func_samr21RadioJobCleanup},

    //Final States
    {RADIO_STATE_ED_DONE,                
        RADIO_EVENT_NONE,     
            RADIO_STATE_ED_DONE,      
    NULL}
};
#define RADIO_STATE_ED_LAST RADIO_STATE_ED_DONE

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

    if (*(currentStatePtr) < MARKER_RADIO_STATES_END_TX){
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
    }

    if (*(currentStatePtr) < MARKER_RADIO_STATES_END_ED){
beginFsmEdHandler:
        for (uint8_t i = 0; s_fsmEdItemTable[i].state != RADIO_STATE_ED_LAST; i++)
        {		
            if(s_fsmEdItemTable[i].state != *(currentStatePtr)){
                continue;
            }
            
            if(event != s_fsmEdItemTable[i].event){
                continue;
            }

            //Check for Transition Function       
            if(s_fsmEdItemTable[i].transitionFunction != NULL){
                s_fsmEdItemTable[i].transitionFunction();
            }  
            
            //Update State
            if(*(currentStatePtr) < MARKER_RADIO_STATES_BEGINN_OTHER_DONE){
                *(currentStatePtr) = s_fsmEdItemTable[i].nextState;
            }

            //Refresh State Pointer cause a change could have been invoked by a transition function
            currentStatePtr = s_currentActiveStatePtr;

            //Check for Soft Events
            if(s_softEvent){

                event = s_softEvent;
                s_softEvent = RADIO_EVENT_NONE;

                goto beginFsmEdHandler;
            }	

            goto exitFsm;
        }
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

/*------------------*/
// Event Functions
/*------------------*/

// timer via TC4
void TC4_Handler()
{
    // Reset IRQ
    TC4->COUNT16.INTFLAG.bit.OVF = 1;

    if(s_fsmEnabled){
        samr21RadioFsmHandleEvent(RADIO_EVENT_TIMER_TRIGGER);
    } 
}

// timer via TC5
void TC5_Handler()
{
    // Reset IRQ
    TC5->COUNT16.INTFLAG.bit.OVF = 1;

    if(s_fsmEnabled){
        samr21RadioFsmHandleEvent(RADIO_EVENT_TIMEOUT_TRIGGER);
    } 
}

// irq from AT86RF233
extern AT86RF233_REG_IRQ_STATUS_t g_trxLastIrq;
void EIC_Handler()
{
    // Clear IRQ
    EIC->INTFLAG.bit.EXTINT0 = 1;
    

    g_trxLastIrq = (AT86RF233_REG_IRQ_STATUS_t)samr21TrxReadRegister(IRQ_STATUS_REG);

    if (g_trxLastIrq.reg == 0x00 || !s_fsmEnabled)
    {
        return;
    }

    if (g_trxLastIrq.bit.pllLock)
    {
        samr21RadioFsmHandleEvent(RADIO_EVENT_IRQ_PLL_LOCK);
    }

    if (g_trxLastIrq.bit.pllUnlock)
    {
        samr21RadioFsmHandleEvent(RADIO_EVENT_IRQ_PLL_UNLOCK);
    }

    if (g_trxLastIrq.bit.rxStart)
    {
        samr21RadioFsmHandleEvent(RADIO_EVENT_IRQ_RX_START);
    }

    if (g_trxLastIrq.bit.trxEnd)
    {
        samr21RadioFsmHandleEvent(RADIO_EVENT_IRQ_TRX_END);
    }

    if (g_trxLastIrq.bit.ccaEdDone)
    {
        samr21RadioFsmHandleEvent(RADIO_EVENT_IRQ_CCA_ED_DONE);
    }

    if (g_trxLastIrq.bit.addressMatch)
    {
        samr21RadioFsmHandleEvent(RADIO_EVENT_IRQ_RX_ADDRESS_MATCH);
    }

    if (g_trxLastIrq.bit.bufferUnderRun)
    {
        samr21RadioFsmHandleEvent(RADIO_EVENT_IRQ_BUFFER_READ_UNDERRUN);
    }

    if (g_trxLastIrq.bit.batteryLow)
    {
        samr21RadioFsmHandleEvent(RADIO_EVENT_IRQ_BAT_LOW);
    }
}