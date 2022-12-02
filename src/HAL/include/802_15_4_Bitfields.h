//Author Eric Härtel @ dresden elektronik ingenieurtechnik gmbh © 2022

#ifndef _802_15_4_BITFIELD_H_
#define _802_15_4_BITFIELD_H_

#define IEEE_802_15_4_FRAME_SIZE                128
#define IEEE_802_15_4_PDSU_SIZE                 127
#define IEEE_802_15_4_CRC_SIZE                  2


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

// typedef struct  
// {
//     uint8_t destinationPanId;
//     uint64_t destinationAddress;
// }macDestinationAddressingFieldsExtAddrWithPan_t;

// typedef struct  
// {
//     uint8_t destinationPanId;
//     uint8_t destinationAddress;
// }macDestinationAddressingFieldsShortAddrWithPan_t;

// typedef struct  
// {
//     uint8_t destinationAddress;
// }macDestinationAddressingFieldsShortAddrNoPan_t;

// typedef struct  
// {
//     uint64_t destinationAddress;
// }macDestinationAddressingFieldsExtAddrNoPan_t;


// typedef struct  
// {
//     uint8_t sourcePanId;
//     uint64_t sourceAddress;
// }macSourceAddressingFieldsExtAddrWithPan_t;

// typedef struct  
// {
//     uint8_t sourcePanId;
//     uint8_t sourceAddress;
// }macSourceAddressingFieldsShortAddrWithPan_t;

// typedef struct  
// {
//     uint8_t sourceAddress;
// }macSourceAddressingFieldsShortAddrNoPan_t;

// typedef struct  
// {
//     uint64_t sourceAddress;
// }macSourceAddressingFieldsExtAddrNoPan_t;


#endif //_802_15_4_BITFIELD_H_