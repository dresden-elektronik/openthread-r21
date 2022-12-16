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

#define OCTET_DURATION_802_15_4_us ( 8 * SYMBOL_DURATION_802_15_4_us )

#ifndef NUM_ENERGY_DETECTION_SCANS_PER_MS
    #define NUM_ENERGY_DETECTION_SCANS_PER_MS 4;
#endif

#define TIME_UNTIL_NEXT_ENERGY_DETECTION_SCAN_us (1000 / NUM_ENERGY_DETECTION_SCANS_PER_MS)

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

typedef struct
{
        uint8_t posDestinationPanId;
        uint8_t posDestinationAddr;
        uint8_t posSourcePanId;
        uint8_t posSourceAddr;
        uint8_t posSecurityControlField;
        uint8_t posSecurityFrameCounter;
        uint8_t posSecurityKeyIdentifier;
        union{
            uint8_t posBeginnPayload;
            uint8_t curParserTrailPos;
        };
}FrameIndex_t;      



typedef struct{
    
    FrameBuffer_t           outboundFrame;  //Holds the raw Data of the Outgoing Frame (Send Frame, Ack Frame)  
    FrameBuffer_t           inboundFrame;   //Holds the raw Data of the Incoming Frame (Recv Frame, Ack Frame)

    FrameIndex_t            frameIndex;     //Holds the relative Position of the Addressing and Securtity Data-Fields

    uint32_t                txTimestamp;    //Timestamp when Transmission of a frame or Ack started
    union{
        uint32_t            rxTimestamp;    //Timestamp when Reception of a frame or Ack started  
        uint32_t            edTimeleft;     //Holds the Number (and Time in ( 1 / NUM_ENERGY_DETECTION_SCANS_PER_MS ms steps ) ) of remaining EnergyScans
    };

    int8_t                  rxRSSI;         //Holds RSSI of the recived Frame
    uint8_t                 rxLQI;          //Holds LQI of the recived Frame
    uint8_t                 channel;        //Holds the channel where the Transaction is happening
    union{
        //RX Operation
        uint8_t             downloadedSize; //Holds the amount of RX-Data downloaded from the TRX

        //TX Operation
        struct         
        {
            uint8_t csma:4;                 //Holds the number of CSMA retrys left
            uint8_t transmission:4;         //Holds the number of Retransmission retrys left
        }                   retrysLeft;

        //ED Operation
        uint8_t             measuredEngeryLevel;    //Holds the number of Retransmission retrys left
    }; 
                    
    RadioJobState           jobState;       //Holds the current JobState of the Transmission in the Buffer

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
    void samr21RadioEnablePromiscuousMode(bool enable);

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

    bool samr21RadioAddShortAddrToPendingFrameTable(uint8_t * shortAddr);
    bool samr21RadioFindShortAddrInPendingFrameTable(uint8_t * shortAddr, bool remove);
    void samr21RadioClearShortAddrPendingFrameTable();

    bool samr21RadioAddIeeeAddrToPendingFrameTable(uint8_t * ieeeAddr);
    bool samr21RadioFindIeeeAddrInPendingFrameTable(uint8_t * ieeeAddr, bool remove);
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

//Filter and Parser funtions
bool samr21RadioFilterPanId(uint8_t * panID);
bool samr21RadioFilterShortAddr(uint8_t * ieeeAddr);
bool samr21RadioFilterIeeeAddr(uint8_t * ieeeAddr);
void samr21RadioParserGetRelativeMacHeaderPositions(FrameBuffer_t* frame, FrameIndex_t*  frameIndex);
#endif // _SAMR21_RADIO_H_
