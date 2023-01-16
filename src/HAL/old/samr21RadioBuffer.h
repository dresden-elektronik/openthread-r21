//Author Eric Härtel @ dresden elektronik ingenieurtechnik gmbh © 2022
#ifndef _SAMR21_RADIO_BUFFER_H_
#define _SAMR21_RADIO_BUFFER_H_

#include "802_15_4_Helper.h"

#include <stdbool.h>
#include <stddef.h>


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




#endif //_SAMR21_RADIO_BUFFER_H_