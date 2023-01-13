#include <stdint.h> 

enum
{
    B_REQUEST_GET_STATUS=0,
    B_REQUEST_CLEAR_FEATURE=1,
    B_REQUEST_SET_FEATURE=3,
    B_REQUEST_SET_ADDRESS=5,
    B_REQUEST_GET_DESCRIPTOR=6,
    B_REQUEST_SET_DESCRIPTOR=7,
    B_REQUEST_GET_CONFIGURATION=8,
    B_REQUEST_SET_CONFIGURATION=9,
    B_REQUEST_GET_INTERFACE=10,
    B_REQUEST_SET_INTERFACE=11,
    B_REQUEST_SYNCH_FRAME=12,
}usbBRequests;

enum
{
    DEVICE=1,
    CONFIGURATION=2,
    STRING=3,
    INTERFACE=4,
    ENDPOINT=5,
    DEVICE_QUALIFIER=6,
    OTHER_SPEED_CONFIGURATION=7,
    INTERFACE_POWER=8
}usbRequestTypes;

enum
{
    DEVICE=1,
    CONFIGURATION=2,
    STRING=3,
    INTERFACE=4,
    ENDPOINT=5,
    DEVICE_QUALIFIER=6,
    OTHER_SPEED_CONFIGURATION=7,
    INTERFACE_POWER=8
}usbDescriptorTypes;

enum
{
    DEVICE_REMOTE_WAKEUP=1,
    ENDPOINT_HALT=2,
    TEST_MODE=3,
}usbFeatures;

typedef struct 
{
    uint8_t bLength; 
    uint8_t bDescriptorType;
    uint16_t bcdUSB;
    uint8_t bDeviceClass;
    uint8_t bDeviceSubClass;
    uint8_t bDeviceProtocol;
    uint8_t bMaxPacketSize;
    uint16_t idVendor;
    uint16_t idProduct;
    uint16_t bcdDevice;
    uint8_t iManufacturer;
    uint8_t iProduct;
    uint8_t iSerialNumber;
    uint8_t bNumConfigurations;
}UsbDeviceDiscriptor_t;

typedef struct 
{
    uint8_t bLength; 
    uint8_t bDescriptorType;
    uint16_t wTotalLength;
    uint8_t bNumInterfaces;
    uint8_t bConfigurationValue;
    uint8_t iConfiguration;
    uint8_t bmAttributes;
    uint8_t bMaxPower;
}UsbConfigurationDiscriptor_t; //Size = 9

typedef struct 
{
    uint8_t bLength; 
    uint8_t bDescriptorType;
    uint8_t bInterfaceNumber;
    uint8_t bAlternateSetting;
    uint8_t bNumEndpoints;
    uint8_t bInterfaceClass;
    uint8_t bInterfaceSubClass;
    uint8_t bInterfaceProtocol;
    uint8_t iInterface;
}UsbInterfaceDiscriptor_t; //Size = 9

typedef struct 
{
    uint8_t bLength; 
    uint8_t bDescriptorType;
    uint8_t bEndpointAddress;
    uint8_t bmAttributes;
    uint16_t wMaxPacketSize;
    uint8_t bInterval;
}UsbEndpointDiscriptor_t; //Size = 7

typedef struct 
{
    uint8_t bLength; 
    uint8_t bDescriptorType;
    uint8_t bDescriptorSubType;
    uint16_t bcdCDC;
}UsbCdcHeaderDescriptor_t; //Size = 5

typedef struct 
{
    uint8_t bLength; 
    uint8_t bDescriptorType;
    uint8_t bDescriptorSubType;
    uint8_t bmCapabilities;
}UsbCdcAcmDescriptor_t; //Size = 4

typedef struct 
{
    uint8_t bLength; 
    uint8_t bDescriptorType;
    uint8_t bDescriptorSubType;
    uint8_t bMasterInterface;
    uint8_t bSlaveInterface;
}UsbCdcUnionDescriptor_t; //Size = 5

typedef struct 
{
    uint8_t bLength; 
    uint8_t bDescriptorType;
    uint8_t bDescriptorSubType;
    uint8_t bmCapabilities;
    uint8_t bDataInterface;
}UsbCdcCallDescriptor_t; //Size = 5


typedef struct 
{
    UsbDeviceDiscriptor_t usbDeviceDiscriptor; 
        UsbConfigurationDiscriptor_t usbConfigurationDiscriptor;
            UsbInterfaceDiscriptor_t usbInterfaceDiscriptor_CDC;
                UsbCdcHeaderDescriptor_t    usbCdcHeaderDescriptor;
                UsbCdcAcmDescriptor_t       usbCdcAcmDescriptor;
                UsbCdcUnionDescriptor_t     usbCdcUnionDescriptor;
                UsbCdcCallDescriptor_t      usbCdcCallDescriptor;
                UsbEndpointDiscriptor_t     usbEndpointDiscriptor_CDC;
            UsbInterfaceDiscriptor_t usbInterfaceDiscriptor_DATA;
                UsbEndpointDiscriptor_t     UsbEndpointDiscriptor_DATAIN;
                UsbEndpointDiscriptor_t     UsbEndpointDiscriptor_DATAOUT;
}UsbDiscriptorVirtualCom_t;


typedef struct 
{
    struct
    {
        uint8_t recipient:5;
        uint8_t type:2;
        uint8_t direction:1;
    }bmRequestType;
    uint8_t bRequest;
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLenght;
}usbDeviceRequest_t;