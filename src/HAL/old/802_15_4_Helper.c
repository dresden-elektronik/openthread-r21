//Author Eric Härtel @ dresden elektronik ingenieurtechnik gmbh © 2022
#include "802_15_4_Helper.h"

uint8_t helper_getAddressFieldLength(uint8_t* pdsu){
        //Calc the len of the AddrField, to set the timer when to start reading the Framebuffer again
    uint8_t lenAddrField = 0;

    macFrameControlField1_t * fcf1 = (macFrameControlField1_t *)&pdsu[0];
    macFrameControlField2_t * fcf2 = (macFrameControlField2_t *)&pdsu[1];

    if(fcf2->destinationAddressingMode != IEEE_802_15_4_ADDR_NONE){
        //Destination Addr Size
        if(fcf2->destinationAddressingMode == IEEE_802_15_4_ADDR_SHORT){
            lenAddrField += sizeof(uint16_t);
        }
        if(fcf2->destinationAddressingMode == IEEE_802_15_4_ADDR_IEEE){
            lenAddrField += sizeof(uint64_t);
        }

        //Destination PAN-ID Size
        if( fcf2->sourceAddressingMode == IEEE_802_15_4_ADDR_NONE ){
            lenAddrField += (fcf1->panIdCompression ? 0 : sizeof(uint16_t));
            goto sourceAddr;
        } 
        if (fcf2->sourceAddressingMode == IEEE_802_15_4_ADDR_IEEE){
            if (fcf2->destinationAddressingMode == IEEE_802_15_4_ADDR_IEEE){
                lenAddrField += (fcf1->panIdCompression ? 0 : sizeof(uint16_t));
                goto sourceAddr;
            }
            lenAddrField += sizeof(uint16_t);
            goto sourceAddr;
        }
        if (fcf2->sourceAddressingMode == IEEE_802_15_4_ADDR_SHORT){
            lenAddrField += sizeof(uint16_t);
            goto sourceAddr;
        }
    }

sourceAddr:
    if(fcf2->sourceAddressingMode != IEEE_802_15_4_ADDR_NONE){
        //Source Addr Size
        if(fcf2->sourceAddressingMode == IEEE_802_15_4_ADDR_SHORT){
            lenAddrField += sizeof(uint16_t);
        }
        if(fcf2->sourceAddressingMode == IEEE_802_15_4_ADDR_IEEE){
            lenAddrField += sizeof(uint64_t);
        }

        //Source PAN-ID Size
        if( fcf2->destinationAddressingMode == IEEE_802_15_4_ADDR_NONE ){
            return lenAddrField + (fcf1->panIdCompression ? 0 : sizeof(uint16_t));
        } 
        if (fcf2->destinationAddressingMode == IEEE_802_15_4_ADDR_IEEE){
            if (fcf2->destinationAddressingMode == IEEE_802_15_4_ADDR_IEEE){
                return lenAddrField;
            }
            return lenAddrField + (fcf1->panIdCompression ? 0 : sizeof(uint16_t));
        }
        if (fcf2->destinationAddressingMode == IEEE_802_15_4_ADDR_SHORT){
            return lenAddrField + (fcf1->panIdCompression ? 0 : sizeof(uint16_t));
        }
    }

    if(!lenAddrField){
        return (fcf1->panIdCompression ? sizeof(uint16_t) : 0);
    }

    return lenAddrField;
}