//Author Eric Härtel @ dresden elektronik ingenieurtechnik gmbh © 2022
#include "samr21Usb.h"

extern g_UsbDiscriptorVirtualCom; //From usbDesc.c

uint8_t usbEndpointData0IN[64];
uint8_t usbEndpointData0OUT[64];

uint8_t usbEndpointData1IN[64];

//Recommended Size Defined by Spinel HDLC Lite
uint8_t usbEndpointData2IN[1300];
uint8_t usbEndpointData2OUT[1300];

UsbDeviceDescBank g_UsbDeviceDescBanks[]={
    //EP0 OUT USB CTRL
    {
        .ADDR = usbEndpointData0OUT,
        .PCKSIZE.bit.BYTE_COUNT = 0,
            .PCKSIZE.bit.MULTI_PACKET_SIZE = 1,
            .PCKSIZE.bit.SIZE = 0x3,
            .PCKSIZE.bit.AUTO_ZLP = 0
    },
    //EP0 IN USB CTRL
    {
        .ADDR = usbEndpointData0IN,
        .PCKSIZE.bit.BYTE_COUNT = 0,
            .PCKSIZE.bit.MULTI_PACKET_SIZE = 1,
            .PCKSIZE.bit.SIZE = 0x3,
            .PCKSIZE.bit.AUTO_ZLP = 1
    },
    //EP1 OUT UNUSED
    {
        .ADDR = 0,
        .PCKSIZE.bit.BYTE_COUNT = 0,
            .PCKSIZE.bit.MULTI_PACKET_SIZE = 1,
            .PCKSIZE.bit.SIZE = 0x3,
            .PCKSIZE.bit.AUTO_ZLP = 0
    },
    //EP1 IN USB CDC CTRL
    {
        .ADDR = usbEndpointData1IN,
        .PCKSIZE.bit.BYTE_COUNT = 0,
            .PCKSIZE.bit.MULTI_PACKET_SIZE = 1,
            .PCKSIZE.bit.SIZE = 0x3,
            .PCKSIZE.bit.AUTO_ZLP = 1
    },
    //EP2 OUT USB CDC DATA
    {
        .ADDR = usbEndpointData2OUT,
        .PCKSIZE.bit.BYTE_COUNT = 0,
            .PCKSIZE.bit.MULTI_PACKET_SIZE = 1,
            .PCKSIZE.bit.SIZE = 0x3,
            .PCKSIZE.bit.AUTO_ZLP = 0
    },
    //EP2 IN USB CDC DATA
    {
        .ADDR = usbEndpointData2IN,
        .PCKSIZE.bit.BYTE_COUNT = 0,
            .PCKSIZE.bit.MULTI_PACKET_SIZE = 1,
            .PCKSIZE.bit.SIZE = 0x3,
            .PCKSIZE.bit.AUTO_ZLP = 1
    }
};


void samr21UsbInit(){
    //Get Calibration Values from NVMCTRL
    USB->DEVICE.PADCAL.bit.TRANSP = (*((uint32_t *) USB_FUSES_TRANSP_ADDR) & USB_FUSES_TRANSP_Msk) >> USB_FUSES_TRANSP_Pos;
    USB->DEVICE.PADCAL.bit.TRANSN = (*((uint32_t *) USB_FUSES_TRANSN_ADDR) & USB_FUSES_TRANSN_Msk) >> USB_FUSES_TRANSN_Pos;
    USB->DEVICE.PADCAL.bit.TRIM = (*((uint32_t *) USB_FUSES_TRIM_ADDR) & USB_FUSES_TRIM_Msk) >> USB_FUSES_TRIM_Pos;

    //Reset USB Module
    USB->DEVICE.CTRLA.bit.SWRST = 1;
    //Wait for Reset to take Place
    while (USB->DEVICE.CTRLA.bit.SWRST || USB->DEVICE.SYNCBUSY.bit.SWRST);

    USB->DEVICE.CTRLB.bit.SPDCONF = USB_DEVICE_CTRLB_SPDCONF_FS_Val;

    USB->DEVICE.DESCADD.reg = &g_UsbDeviceDescBanks;

    //EP 0 (USB HOST CTRL)
        USB->DEVICE.DeviceEndpoint[0].EPCFG.bit.EPTYPE0=0x1;
        USB->DEVICE.DeviceEndpoint[0].EPCFG.bit.EPTYPE1=0x1;
        USB->DEVICE.DeviceEndpoint[0].EPINTENSET.bit.RXSTP = 1;

    //EP 1 (Virtual COM-Port DATA) BULK
        USB->DEVICE.DeviceEndpoint[0].EPCFG.bit.EPTYPE0=0x3;
        USB->DEVICE.DeviceEndpoint[0].EPCFG.bit.EPTYPE1=0x3;

    //EP 2 (Virtual COM-Port CTRL) INTERRUPT
        USB->DEVICE.DeviceEndpoint[0].EPCFG.bit.EPTYPE0=0x0;
        USB->DEVICE.DeviceEndpoint[0].EPCFG.bit.EPTYPE1=0x4;    

    //Enable USB Module
    USB->DEVICE.CTRLA.reg =
        USB_CTRLA_ENABLE
        |USB_CTRLA_MODE_DEVICE
        |USB_CTRLA_RUNSTDBY
    ;
    while (USB->DEVICE.SYNCBUSY.bit.ENABLE);

    __NVIC_EnableIRQ(USB_IRQn);
}

void samr21UsbHandleSetupToken(){
    
    usbDeviceRequest_t* data = (usbDeviceRequest_t*)usbEndpointData0OUT;

    switch (data->bRequest)
    {
    case B_REQUEST_SET_ADDRESS:
        USB->DEVICE.DADD.bit.DADD = data->wValue;
        USB->DEVICE.DADD.bit.ADDEN = 1;
        break;
    
    case B_REQUEST_GET_STATUS:
        uint16_t statusResponse = 0x0000;
        if(data->bmRequestType.recipient == 0){
            statusResponse = 0b10;
        } else if (data->bmRequestType.recipient == 2){
            //TODO Maybe Flip
            if(data->wIndex&0x80){
                statusResponse |= USB->DEVICE.DeviceEndpoint[data->wIndex&0x0f].EPSTATUS.bit.STALLRQ1;
            } else {
                statusResponse |= USB->DEVICE.DeviceEndpoint[data->wIndex&0x0f].EPSTATUS.bit.STALLRQ0;
            }
        } else{
            *((uint16_t *)usbEndpointData2OUT) = statusResponse;
        }

        g_UsbDeviceDescBanks[0].PCKSIZE.bit.MULTI_PACKET_SIZE = 0;
        g_UsbDeviceDescBanks[0].PCKSIZE.bit.BYTE_COUNT = 2;
        USB->DEVICE.DeviceEndpoint[1].EPSTATUSSET.bit.BK0RDY;
        break;

    case B_REQUEST_CLEAR_FEATURE:
        if(data->wValue == ENDPOINT_HALT){
            //TODO Maybe Flip
            if(data->wIndex&0x80){
                USB->DEVICE.DeviceEndpoint[data->wIndex&0x0f].EPSTATUSCLR.bit.STALLRQ1 = 1;
            } else {
                USB->DEVICE.DeviceEndpoint[data->wIndex&0x0f].EPSTATUSCLR.bit.STALLRQ0 = 1;
            }
        }
        break;
    
    case B_REQUEST_SET_FEATURE:
        if(data->wValue == ENDPOINT_HALT){
            //TODO Maybe Flip
            if(data->wIndex&0x80){
                USB->DEVICE.DeviceEndpoint[data->wIndex&0x0f].EPSTATUSSET.bit.STALLRQ1 = 1;
            } else {
                USB->DEVICE.DeviceEndpoint[data->wIndex&0x0f].EPSTATUSSET.bit.STALLRQ0 = 1;
            }
        }
        break;
    
    case B_REQUEST_GET_DESCRIPTOR:
        if(data->wValue == ENDPOINT_HALT){
            
            //TODO Maybe Flip

        }
        break;

    case B_REQUEST_SET_DESCRIPTOR:
        break;

    case B_REQUEST_GET_CONFIGURATION:
        break;

    case B_REQUEST_SET_CONFIGURATION:

        break;

    case B_REQUEST_GET_INTERFACE:
        break;

    case B_REQUEST_SET_INTERFACE:
        break;

    case B_REQUEST_SYNCH_FRAME:
        break;

    
    default:
        break;
    }
}


void USB_Handler(){
    //Check IRQ Reason

    //Generic
    if(USB->DEVICE.INTFLAG.reg){
        
    }

    //EP0
    if(USB->DEVICE.DeviceEndpoint[0].EPINTFLAG.reg){
        if(USB->DEVICE.DeviceEndpoint[0].EPINTFLAG.bit.RXSTP){
            samr21UsbHandleSetupToken();
            USB->DEVICE.DeviceEndpoint[0].EPINTFLAG.bit.RXSTP=1;
        }
    }

    //EP1
    if(USB->DEVICE.DeviceEndpoint[0].EPINTFLAG.reg){

    }

    //EP2
    if(USB->DEVICE.DeviceEndpoint[0].EPINTFLAG.reg){

    }
}