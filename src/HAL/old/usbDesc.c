#include "usbDesc.h"

UsbDiscriptorVirtualCom_t g_UsbDiscriptorVirtualCom = //external Var
{
    .usbDeviceDiscriptor =
    {
        .bLength=18,      
        .bDescriptorType=1,      
        .bcdUSB=0x0201,
        .bDeviceClass=2,
        .bDeviceSubClass=0,
        .bDeviceProtocol=0,
        .bMaxPacketSize=64,
        .idVendor=0x1cf1,
        .idProduct=0xee30,
        .bcdDevice=0x0100,
        .iManufacturer=1,
        .iProduct=2,
        .iSerialNumber=3,
        .bNumConfigurations=1
    },
        .usbConfigurationDiscriptor =
        {
            .bLength=9,
            .bDescriptorType=2,
            .wTotalLength=0x0043,
            .bNumInterfaces=2,
            .bConfigurationValue=1,
            .iConfiguration=0,
            .bmAttributes=0x80,
            .bMaxPower=50
        },
            .usbInterfaceDiscriptor_CDC =
            {
                .bLength=9,
                .bDescriptorType=4,
                .bInterfaceNumber=0,
                .bAlternateSetting=0,
                .bNumEndpoints=1,
                .bInterfaceClass=2,
                .bInterfaceSubClass=2,
                .bInterfaceProtocol=1,
                .iInterface=0
            },
                .usbCdcHeaderDescriptor =
                {
                    .bLength=5,
                    .bDescriptorType=0x24,
                    .bDescriptorSubType=0x00,
                    .bcdCDC=0x0110
                },
                .usbCdcAcmDescriptor =
                {
                    .bLength=4,
                    .bDescriptorType=0x24,
                    .bDescriptorSubType=0x02,
                    .bmCapabilities=0x02
                },
                .usbCdcUnionDescriptor =
                {
                    .bLength=5,
                    .bDescriptorType=0x24,
                    .bDescriptorSubType=0x02,
                    .bMasterInterface = 0,
                    .bSlaveInterface = 1
                },
                .usbCdcCallDescriptor =
                {
                    .bLength=4,
                    .bDescriptorType=0x24,
                    .bDescriptorSubType=0x02,
                    .bmCapabilities=0x02,
                    .bDataInterface=1
                },
                .usbEndpointDiscriptor_CDC =
                {
                    .bLength=7,
                    .bDescriptorType=5,
                    .bEndpointAddress=0x81,
                    .bmAttributes=3,
                    .wMaxPacketSize=0x0040,
                    .bInterval=16
                },
            .usbInterfaceDiscriptor_DATA =
            {
                .bLength=9,
                .bDescriptorType=4,
                .bInterfaceNumber=1,
                .bAlternateSetting=0,
                .bNumEndpoints=2,
                .bInterfaceClass=10,
                .bInterfaceSubClass=0,
                .bInterfaceProtocol=0,
                .iInterface=0
            },
                .UsbEndpointDiscriptor_DATAIN =
                {
                    .bLength=7,
                    .bDescriptorType=5,
                    .bEndpointAddress=0x82,
                    .bmAttributes=2,
                    .wMaxPacketSize=0x0040,
                    .bInterval=0
                },
                .UsbEndpointDiscriptor_DATAOUT =
                {
                    .bLength=7,
                    .bDescriptorType=5,
                    .bEndpointAddress=0x02,
                    .bmAttributes=2,
                    .wMaxPacketSize=0x0040,
                    .bInterval=0
                }
};
