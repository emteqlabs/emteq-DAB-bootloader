#include "usb.h"

#if 0 //< Can't use in C comppiler mode :(
usb_bos_webusb_descriptor_t make_usb_bos_webusb_descriptor(const uint8_t vendorCode, const uint8_t iLandingPage)
{
    return (usb_bos_webusb_descriptor_t) {
        {
            sizeof(usb_bos_webusb_descriptor_t)
                , USB_DEVICE_CAPABILITY_DESCRIPTOR
                , 0x05
                , 0x00
                , {0x38, 0xB6, 0x08, 0x34, 0xA9, 0x09, 0xA0, 0x47, 0x8B, 0xFD, 0xA0,0x76, 0x88, 0x15, 0xB6, 0x65}
        },
        {
              0x0100 ///<WebUSB descriptor version 1.0
            , vendorCode ///0x01 ///< bRequest value for WebUSB
            , iLandingPage ///0x01 ///<URL for landing page
        }
    };
}

usb_bos_ms_os_20_descriptor_t make_usb_bos_ms_os_20_descriptor(const uint16_t msOsDescriptorSetTotalLength, const uint8_t vendorCode)
{
    return (usb_bos_ms_os_20_descriptor_t) {
        {
            sizeof(usb_bos_webusb_descriptor_t)
                , USB_DEVICE_CAPABILITY_DESCRIPTOR
                , 0x05
                , 0x00
                // MS_OS_20_Platform_Capability_ID - {D8DD60DF-4589-4CC7-9CD2-659D9E648A9F}
                , {0xDF, 0x60, 0xDD, 0xD8, 0x89, 0x45, 0xC7, 0x4C, 0x9C, 0xD2, 0x65, 0x9D, 0x9E, 0x64, 0x8A, 0x9F}
        },
        {
              0x06030000 ///< Minimum Windows version (8.1) (0x06030000)
            , msOsDescriptorSetTotalLength
            , vendorCode ///< bRequest value for MS_OS
            , 0x00
        }
    };
}
#endif