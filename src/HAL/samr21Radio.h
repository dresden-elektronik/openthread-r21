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

#include "include/at86rf233_Bitfield.h"
#include "include/802_15_4_Helper.h"

#ifndef NUM_RADIO_BUFFER
#define NUM_RADIO_BUFFER 4
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

    uint32_t                rxTimestamp; 
    int8_t                  rxRSSI;
    uint8_t                 rxLQI;


    union{
        //RX Operation
        uint8_t             downloadedSize;

        //TX Operation
        struct         
        {
            uint8_t csma:4;
            uint8_t transmission:4;
        }retrysLeft;
    };      

    RadioJobState           currentJobState;
}TransmissionBuffer_t;

//Config Functions
    void samr21RadioInit();

//Config Functions
    void samr21RadioChangeState(uint8_t newState);
    void samr21RadioChangeChannel(uint8_t newChannel);
    void samr21RadioChangeCCAMode(uint8_t newCcaMode);
    void samr21RadioChangeTXPower(uint8_t txPower);
    void samr21RadioChangeCCAThreshold(uint8_t threshold);
    void samr21RadioSetShortAddr(uint16_t shortAddr);
    void samr21RadioSetPanID(uint16_t panId);
    void samr21RadioSetIEEEAddr(uint64_t ieeeAddr);
    void samr21RadioChangeCSMABackoffExponent(uint8_t minBE, uint8_t maxBE);
    void samr21RadioChangeNumTransmitRetrys(uint8_t numRetrys);
    void samr21RadioChangeNumBackoffsCSMA(uint8_t numBackoffs);

//M.I.S.C Functions
    uint8_t samr21RadioGetRandom2Bit();
    uint8_t samr21RadioGetRandomNibble();
    uint8_t samr21RadioGetRandomByte();

//Interface Function
    bool samr21RadioSendFrame(FrameBuffer_t * frame);

//PROTOTYPE! MUST BE DEFINED IN APPLIKATION CODE
    void cbf_samr21RadioReceivedMsgFrame(FrameBuffer_t* psduMsg, FrameBuffer_t* psduAck);
    void cbf_samr21RadioTransmitMsgFrameDone(bool success, FrameBuffer_t* psduAck);

//TX Operations
    //CSMA
        void samr21RadioStartCCA();
        void samr21RadioEvalCCA();
        void samr21RadioStartBackoffTimer();
    
    //Transmit
        void samr21RadioSendTXPayload();

    //Ack Handaling
        void samr21RadioWaitForAck();
        void samr21RadioEvalRetransmission();
        void samr21RadioEvalAck();
    
        void samr21RadioTxAbort();


//RX Operations
    //Recive Stage
        void samr21RadioLiveRxParser();
        void samr21RadioAbortLiveRxParser();
    //Sending Ack
        void samr21RadioSendAck();
        void samr21RadioAckReceptionStarted();

//BOTH RX and TX
        void samr21RadioTransmissionCleanup();

//Helper Funcs


#endif // _SAMR21_RADIO_H_
