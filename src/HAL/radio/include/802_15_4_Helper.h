//Author Eric Härtel @ dresden elektronik ingenieurtechnik gmbh © 2022

#ifndef _802_15_4_HELPER_H_
#define _802_15_4_HELPER_H_

#include <stdint.h>

#define IEEE_802_15_4_EXTENDED_ADDR_SIZE_BYTES  8
#define IEEE_802_15_4_SHORT_ADDR_SIZE_BYTES     2
#define IEEE_802_15_4_PAN_ID_SIZE_BYTES         2

#define IEEE_802_15_4_24GHZ_TIME_PER_OCTET_us   32

#define IEEE_802_15_4_FRAME_SIZE                128
#define IEEE_802_15_4_PDSU_SIZE                 127
#define IEEE_802_15_4_CRC_SIZE                  2
#define IEEE_802_15_4_FCF_SIZE                  2
#define IEEE_802_15_4_DSN_SIZE                  1
#define IEEE_802_15_4_PHY_LEN_SIZE              1

#define IEEE_802_15_4_STATIC_HEADER_SIZE        (IEEE_802_15_4_FCF_SIZE + IEEE_802_15_4_DSN_SIZE)


typedef enum IEEE_802_15_4_FrameType{
    IEEE_802_15_4_BEACON_FRAME               = 0b000,
    IEEE_802_15_4_DATA_FRAME                 = 0b001,
    IEEE_802_15_4_ACK_FRAME                  = 0b010,
    IEEE_802_15_4_MAC_CMD_FRAME              = 0b011
}IEEE_802_15_4_FrameType;

typedef enum IEEE_802_15_4_FrameVersion{
    IEEE_802_15_4_VERSION_2003               = 0b00,
    IEEE_802_15_4_VERSION_2006               = 0b01
}IEEE_802_15_4_FrameVersion;

typedef enum IEEE_802_15_4_AddrMode{
    IEEE_802_15_4_ADDR_NONE                  = 0b00,
    IEEE_802_15_4_ADDR_SHORT                 = 0b10,
    IEEE_802_15_4_ADDR_IEEE                  = 0b11
}IEEE_802_15_4_AddrMode;

typedef struct
{
    uint8_t lenght:7;
    uint8_t reseverd:1;
}phyHeader_t;

typedef struct
{
    uint8_t frameType:3;
    uint8_t securityEnabled:1;
    uint8_t framePending:1;
    uint8_t ackRequest:1;
    uint8_t panIdCompression:1;
    uint8_t reserved:1;
}macFrameControlField1_t;

typedef struct
{
    uint8_t sequenceNumberSuppression:1;
    uint8_t informationElementsPresent:1;
    uint8_t destinationAddressingMode:2;
    uint8_t frameVersion:2;
    uint8_t sourceAddressingMode:2;
}macFrameControlField2_t;

typedef struct
{
    uint8_t securityLevel:3;
    uint8_t keyIndentifierMode:2;
    uint8_t frameCounterSuppression:1;
    uint8_t asnInNonce:1;
    uint8_t reserved:1;
}auxSecurityHeaderControlFiel_t;

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
}frameFieldPosInfo_t;



#endif //_802_15_4_HELPER_H_
