//Author Eric Härtel @ dresden elektronik ingenieurtechnik gmbh © 2022
#ifndef _SAMR21_RADIO_H_
#define _SAMR21_RADIO_H_


#include "samr21.h"
#include "samr21RadioFSM.h"
#include "samr21NopDelay.h"
#include "samr21Timer.h"
#include "samr21Rtc.h"
#include "samr21Trx.h"
#include <stdbool.h>
#include <string.h>

#include "at86rf233_Bitfield.h"
#include "802_15_4_Helper.h"

#ifndef NUM_RADIO_JOB_BUFFER
    #define NUM_RADIO_JOB_BUFFER 4
#endif

#ifndef SIZE_TABLE_FRAME_PENDING_SHORT_ADDR
    #define SIZE_TABLE_FRAME_PENDING_SHORT_ADDR 10
#endif

#ifndef SIZE_TABLE_FRAME_PENDING_IEEE_ADDR
    #define SIZE_TABLE_FRAME_PENDING_IEEE_ADDR 10
#endif

#ifndef SYMBOL_DURATION_802_15_4_us
    #define SYMBOL_DURATION_802_15_4_us 16
#endif

typedef enum RadioState{
    RADIO_STATE_IDLE                          = 0x00,
    RADIO_STATE_SLEEP                         = 0x01,
    RADIO_STATE_RX                            = 0x11,
    RADIO_STATE_TX                            = 0x12
} RadioState;

typedef union
{
    uint8_t raw[128]; 

    struct{
        uint8_t                 lenght;
        macFrameControlField1_t frameControlField1;
        macFrameControlField2_t frameControlField2;
        uint8_t                 sequenceNumber;
    }header;

}FrameBuffer_t;



typedef struct{
    FrameBuffer_t           outboundFrame;
    FrameBuffer_t           inboundFrame;

    
    uint32_t                txTimestamp;

    union{
        uint32_t            rxTimestamp;
        uint32_t            edTimeleft;
    };

    int8_t                  rxRSSI;
    uint8_t                 rxLQI;
    uint8_t                 channel;

    union{
        //RX Operation
        uint8_t             downloadedSize;

        //TX Operation
        struct         
        {
            uint8_t csma:4;
            uint8_t transmission:4;
        }                   retrysLeft;

        //ED Operation
        uint8_t              measuredEngeryLevel;
    };      

    RadioJobState           jobState;
}JobBuffer_t;

//Config Functions
    void samr21RadioInit();

//Config Functions
    void samr21RadioChangeChannel(uint8_t newChannel);
    void samr21RadioChangeCCAMode(uint8_t newCcaMode);

    void samr21RadioSetTxPower(int8_t txPower);
    int8_t samr21RadioGetTxPower();

    void samr21RadioSetCcaThreshold(int8_t threshold);
    int8_t samr21RadioGetCurrentCcaThreshold();

    void samr21RadioSetShortAddr(uint8_t* shortAddr);
    void samr21RadioSetPanId(uint8_t* panId);
    void samr21RadioSetIeeeAddr(uint8_t* ieeeAddr);
    void samr21RadioChangeCsmaBackoffExponent(uint8_t minBE, uint8_t maxBE);
    void samr21RadioChangeNumTransmitRetrys(uint8_t numRetrys);
    void samr21RadioChangeNumBackoffsCsma(uint8_t numBackoffs);

    void samr21RadioChangeState(RadioState newState, uint8_t channel);
    RadioState samr21RadioGetStatus();



//M.I.S.C Functions
    uint8_t samr21RadioGetRandomCrumb();
    uint8_t samr21RadioGetRandomNibble();
    uint8_t samr21RadioGetRandomByte();

//Interface Function
    bool samr21RadioSendFrame(FrameBuffer_t * frame, uint8_t channel);
    bool samr21RadioReceive(uint8_t channel);

    bool samr21RadioStartEnergyDetection(uint8_t channel, uint16_t duration);

    JobBuffer_t* samr21RadioGetNextFinishedJobBuffer();

    bool samr21RadioAddShortAddrToPendingFrameTable(uint16_t shortAddr);
    bool samr21RadioFindShortAddrInPendingFrameTable(uint16_t shortAddr, bool remove);
    void samr21RadioClearShortAddrPendingFrameTable();

    bool samr21RadioAddIeeeAddrToPendingFrameTable(uint64_t ieeeAddr);
    bool samr21RadioFindIeeeAddrInPendingFrameTable(uint64_t ieeeAddr, bool remove);
    void samr21RadioClearIeeeAddrPendingFrameTable();


//TX Operations
    //CSMA
        void fsm_func_samr21RadioStartCCA();
        void fsm_func_samr21RadioEvalCCA();
        void fsm_func_samr21RadioStartBackoffTimer();
    
    //Transmit
        void fsm_func_samr21RadioSendTXPayload();

    //Ack Handaling
        void fsm_func_samr21RadioWaitForAck();
        void fsm_func_samr21RadioEvalRetransmission();
        void fsm_func_samr21RadioEvalAck();
    
        void fsm_func_samr21RadioTxAbort();


//RX Operations
    //Recive Stage
        void fsm_func_samr21RadioLiveRxParser();
        void fsm_func_samr21RadioAbortLiveRxParser();
    //Sending Ack
        void fsm_func_samr21RadioSendAck();
        void fsm_func_samr21RadioAckReceptionStarted();

//BOTH RX and TX
        void fsm_func_samr21RadioJobCleanup();

//Energy Detection Operations
        void fsm_func_samr21StartEd();
        void fsm_func_samr21EvalEd();
        void fsm_func_samr21EvalEdContinuation();

#endif // _SAMR21_RADIO_H_
