/*
 * Copyright (c) 2016, Alex Taradov <alex@taradov.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


 /*- Includes ----------------------------------------------------------------*/
#include <sam.h>
#include "common.h"
#include "dfu.h"
#include "usb.h"
#include "usb_descriptors.h"

/*- Variables ---------------------------------------------------------------*/
usb_device_descriptor_t usb_device_descriptor __attribute__( (aligned( 4 )) ) = /* MUST BE IN RAM for USB peripheral */
{
  .bLength = sizeof( usb_device_descriptor_t ),
  .bDescriptorType = USB_DEVICE_DESCRIPTOR,

  .bcdUSB = 0x0210, // at least 2.1 or 3.x for BOS & webUSB
  .bDeviceClass = 0,
  .bDeviceSubClass = 0,
  .bDeviceProtocol = 0,

  .bMaxPacketSize0 = 64,
  .idVendor = usbVendorId,
  .idProduct = usbProductId,
  .bcdDevice = 0, //< @todo Cannot set here under c!

#if USE_STRING_DESCRIPTORS
  .iManufacturer = USB_STR_MANUFACTURER,
  .iProduct = USB_STR_PRODUCT,
  .iSerialNumber = USB_STR_SERIAL_NUMBER,
#else
  .iManufacturer = USB_STR_ZERO,
  .iProduct = USB_STR_ZERO,
  .iSerialNumber = USB_STR_ZERO,
#endif

  .bNumConfigurations = 1
};

usb_bos_descriptor_hierarchy_t usb_bos_descriptor_hierarchy __attribute__((aligned(4))) = /* MUST BE IN RAM for USB peripheral */
{
    // total length, number of device caps
    .header = (usb_bos_descriptor_header_t) {
          sizeof(usb_bos_descriptor_header_t)
        , USB_BINARY_OBJECT_STORE_DESCRIPTOR
        , sizeof(usb_bos_descriptor_hierarchy_t)
        , 2
    }

    // Vendor Code, iLandingPage
    , .webusb = /*(usb_bos_webusb_descriptor_t)*/ {
        .platform = (usb_desc_bos_platform_t){
            sizeof(usb_bos_webusb_descriptor_t)
            , USB_DEVICE_CAPABILITY_DESCRIPTOR
            , 0x05
            , 0x00
            , {0x38, 0xB6, 0x08, 0x34, 0xA9, 0x09, 0xA0, 0x47, 0x8B, 0xFD, 0xA0, 0x76, 0x88, 0x15, 0xB6, 0x65}
        }
        , .capability = (usb_bos_webusb_capability_t){
              0x0100 ///<WebUSB descriptor version 1.0
            , VENDOR_REQUEST_WEBUSB ///0x01 ///< bRequest value for WebUSB
            , 1 ///0x01 ///<URL for landing page
        }
    }

    // Microsoft OS 2.0 descriptor
    , .ms_os_20 = /*(usb_bos_ms_os_20_descriptor_t) */{
        .platform = (usb_desc_bos_platform_t){
              sizeof(usb_bos_ms_os_20_descriptor_t)
            , USB_DEVICE_CAPABILITY_DESCRIPTOR
            , 0x05
            , 0x00
            // MS_OS_20_Platform_Capability_ID - {D8DD60DF-4589-4CC7-9CD2-659D9E648A9F}
            , {0xDF, 0x60, 0xDD, 0xD8, 0x89, 0x45, 0xC7, 0x4C, 0x9C, 0xD2, 0x65, 0x9D, 0x9E, 0x64, 0x8A, 0x9F}
        }
        , .capability = (usb_bos_ms_os_20_capability_t){
              0x06030000 ///< Minimum Windows version (8.1) (0x06030000)
            , sizeof(usb_ms_os_20_descriptor_set_t)
            , VENDOR_REQUEST_MICROSOFT ///< bRequest value for MS_OS
            , 0x00
        }
    }
};
_Static_assert(sizeof(usb_bos_descriptor_hierarchy) == 57, "Incorrect size");

usb_ms_os_20_descriptor_set_t usb_ms_os_20_descriptor_set __attribute__((aligned(4))) = /* MUST BE IN RAM for USB peripheral */
{
    // Set header: length, type, windows version, total length
    (usb_ms_os_20_descriptor_set_header_t) {
          sizeof(usb_ms_os_20_descriptor_set_header_t)
        , MS_OS_20_SET_HEADER_DESCRIPTOR
        , 0x06030000
        , sizeof(usb_ms_os_20_descriptor_set_t)
    }

#if 0 /// NOT Composite!
    // Configuration subset header: length, type, configuration index, reserved, configuration total length
    , (usb_ms_os_20_configuration_subset_header_t) {
          sizeof(usb_ms_os_20_configuration_subset_header_t)
        , MS_OS_20_SUBSET_HEADER_CONFIGURATION
        ,  0 //< Configuration
        ,  0 //< resered
        , sizeof(usb_ms_os_20_configuration_subset_header_t)
          + sizeof(usb_ms_os_20_function_subset_header_t)
          + sizeof(usb_ms_os_20_compatible_id_descriptor_t)
          + sizeof(usb_ms_os_20_device_interface_guid_section_t) //<Subset length inc. this
    }

    // Function Subset header: length, type, first interface, reserved, subset length
    , (usb_ms_os_20_function_subset_header_t) {
          sizeof(usb_ms_os_20_function_subset_header_t)
        , MS_OS_20_SUBSET_HEADER_FUNCTION
        , 0 //ITF_NUM_VENDOR
        , 0 //< reservedsizeof(usb_ms_os_20_configuration_subset_header_t)
        , sizeof(usb_ms_os_20_function_subset_header_t)
          + sizeof(usb_ms_os_20_compatible_id_descriptor_t)
          + sizeof(usb_ms_os_20_device_interface_guid_section_t) //<Subset length inc. this
    }
#endif

    // MS OS 2.0 Compatible ID descriptor: length, type, compatible ID, sub compatible ID
    , (usb_ms_os_20_compatible_id_descriptor_t) {
          sizeof(usb_ms_os_20_compatible_id_descriptor_t)
        , MS_OS_20_FEATURE_COMPATBLE_ID
        , "WINUSB\0\0"
        , {0}
    }

    // MS OS 2.0 Registry property descriptor: length, type
    , (usb_ms_os_20_device_interface_guid_section_t) {
          sizeof(usb_ms_os_20_device_interface_guid_section_t)
        , MS_OS_20_FEATURE_REG_PROPERTY
        // wPropertyDataType, wPropertyNameLength and PropertyName "DeviceInterfaceGUIDs\0" in UTF-16
        , 0x0001 //< 1=A NULL-terminated Unicode String (REG_SZ)
        , sizeof(((usb_ms_os_20_device_interface_guid_section_t*)0)->bPropertyName)
        , u"DeviceInterfaceGUID\0"
        , sizeof(((usb_ms_os_20_device_interface_guid_section_t*)0)->bPropertyData)
        // bPropertyData: {3b9e82dd-cd7b-4133-bf88-4c65d4d84e20} <<Generated with GUID-Gen
        , u"{3b9e82dd-cd7b-4133-bf88-4c65d4d84e20}\0"
    }
};
//@TODO Check is valid for composite device only!
//_Static_assert(sizeof(usb_ms_os_20_descriptor_set) == 0xB2, "Incorrect size");


#define WEBUSB_URL  "deviceupdate.emteqlabs.com"
usb_desc_webusb_url_t usb_webusb_url_set =
{
  3 + sizeof(WEBUSB_URL) - 1,
  3, // WEBUSB URL type
  1, // 0: http, 1: https
  WEBUSB_URL
};

//_Static_assert(sizeof(usb_ms_os_20_descriptor_set) == 0xB2, "Incorrect size");

usb_configuration_hierarchy_t usb_configuration_hierarchy __attribute__( (aligned( 4 )) ) = /* MUST BE IN RAM for USB peripheral */
{
    .standard =
    {
        .configuration =
        {
            .bLength = sizeof( usb_configuration_descriptor_t ),
            .bDescriptorType = USB_CONFIGURATION_DESCRIPTOR,
            .wTotalLength = sizeof( usb_configuration_hierarchy_t ),
            .bNumInterfaces = 1,
            .bConfigurationValue = 1,
            .iConfiguration = USB_STR_ZERO,
            .bmAttributes = 0x80,
            .bMaxPower = 50, // 100 mA
        },

        .dfu =
        {
            .bLength = sizeof( usb_dfu_descriptor_t ),
            .bDescriptorType = 33,
            .bmAttributes = (USB_DFU_ATTR_CAN_DNLOAD /* | USB_DFU_ATTR_CAN_UPLOAD*/ | USB_DFU_ATTR_WILL_DETACH | USB_DFU_ATTR_MANIFESTATION_TOLERANT),
            .wDetachTimeout = 500,
            .wTransferSize = dfu_blockSize,
            .bcdDFU = 0x101,
        }
    },
#if DFU_BOOT 
    .boot = 
    {
        .dfuApp =
        {
            .bLength = sizeof( usb_interface_descriptor_t ),
            .bDescriptorType = USB_INTERFACE_DESCRIPTOR,
            .bInterfaceNumber = 0,
            .bAlternateSetting = USB_ALTERNATESETTING_App,
            .bNumEndpoints = 0,
            .bInterfaceClass = DFU_INTERFACE_CLASS,
            .bInterfaceSubClass = DFU_INTERFACE_SUBCLASS,
            .bInterfaceProtocol = DFU_INTERFACE_PROTOCOL,
            .iInterface = USB_STR_DFU_App,
        }
    },
#elif DFU_APP
    .app =
    {
        /// @todo reduce bootloader size by removing the duplication!.. how to send without being all in memory?
        .dfuBootloader =
        {
            .bLength = sizeof( usb_interface_descriptor_t ),
            .bDescriptorType = USB_INTERFACE_DESCRIPTOR,
            .bInterfaceNumber = 0,
            .bAlternateSetting = USB_ALTERNATESETTING_Bootloader,
            .bNumEndpoints = 0,
            .bInterfaceClass = DFU_INTERFACE_CLASS,
            .bInterfaceSubClass = DFU_INTERFACE_SUBCLASS,
            .bInterfaceProtocol = DFU_INTERFACE_PROTOCOL,
            .iInterface = USB_STR_DFU_Bootloader,
        },
        .dfuHardwareData =
        {
            .bLength = sizeof( usb_interface_descriptor_t ),
            .bDescriptorType = USB_INTERFACE_DESCRIPTOR,
            .bInterfaceNumber = 0,
            .bAlternateSetting = USB_ALTERNATESETTING_HardwareData,
            .bNumEndpoints = 0,
            .bInterfaceClass = DFU_INTERFACE_CLASS,
            .bInterfaceSubClass = DFU_INTERFACE_SUBCLASS,
            .bInterfaceProtocol = DFU_INTERFACE_PROTOCOL,
            .iInterface = USB_STR_DFU_HardwareData,
        },
        .dfuCalibrationData =
        {
            .bLength = sizeof( usb_interface_descriptor_t ),
            .bDescriptorType = USB_INTERFACE_DESCRIPTOR,
            .bInterfaceNumber = 0,
            .bAlternateSetting = USB_ALTERNATESETTING_CalibrationData,
            .bNumEndpoints = 0,
            .bInterfaceClass = DFU_INTERFACE_CLASS,
            .bInterfaceSubClass = DFU_INTERFACE_SUBCLASS,
            .bInterfaceProtocol = DFU_INTERFACE_PROTOCOL,
            .iInterface = USB_STR_DFU_CalibrationData,
        }
    }
#else
#error "Unknown DFU setup"
#endif
};

#if USE_STRING_DESCRIPTORS

#define STR_MANUFACTURER u"Emteq"
#define STR_PRODUCT u"EmteqDAB"
#define STR_SERIAL u"DAB\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0" ///< @note Pre-populated 'DAB' prefix
#define STR_DFU_App STR_PRODUCT u"-Application"
#define STR_DFU_CalibrationData STR_PRODUCT u"-Calibration"
#define STR_DFU_HardwareData STR_PRODUCT u"-HardwareData"
#define STR_DFU_Bootloader STR_PRODUCT  u"-Bootloader"

usb_string_descriptor_t usb_string_descriptor_langid __attribute__( (aligned( 4 )) ) = { sizeof( usb_string_descriptor_t ) + 2, USB_STRING_DESCRIPTOR, {0x0409 /*USB_LANGUAGE_EN_US*/} };
usb_string_descriptor_t usb_string_descriptor_manufacturer __attribute__( (aligned( 4 )) ) = { sizeof( usb_string_descriptor_t ) + sizeof( STR_MANUFACTURER ) - 2, USB_STRING_DESCRIPTOR, STR_MANUFACTURER };
usb_string_descriptor_t usb_string_descriptor_product __attribute__( (aligned( 4 )) ) = { sizeof( usb_string_descriptor_t ) + sizeof( STR_PRODUCT ) - 2, USB_STRING_DESCRIPTOR, STR_PRODUCT };
usb_string_descriptor_t usb_string_descriptor_serial __attribute__( (aligned( 4 )) ) = { sizeof( usb_string_descriptor_t ) + sizeof( STR_SERIAL ) - 2, USB_STRING_DESCRIPTOR, STR_SERIAL };
usb_string_descriptor_t usb_string_descriptor_dfu_app __attribute__( (aligned( 4 )) ) = { sizeof( usb_string_descriptor_t ) + sizeof( STR_DFU_App ) - 2, USB_STRING_DESCRIPTOR, STR_DFU_App };
usb_string_descriptor_t usb_string_descriptor_dfu_calibrationData __attribute__( (aligned( 4 )) ) = { sizeof( usb_string_descriptor_t ) + sizeof( STR_DFU_CalibrationData ) - 2, USB_STRING_DESCRIPTOR, STR_DFU_CalibrationData };
usb_string_descriptor_t usb_string_descriptor_dfu_bootloader __attribute__( (aligned( 4 )) ) = { sizeof( usb_string_descriptor_t ) + sizeof( STR_DFU_Bootloader ) - 2, USB_STRING_DESCRIPTOR, STR_DFU_Bootloader };
usb_string_descriptor_t usb_string_descriptor_dfu_hardwareData __attribute__( (aligned( 4 )) ) = { sizeof( usb_string_descriptor_t ) + sizeof( STR_DFU_HardwareData ) - 2, USB_STRING_DESCRIPTOR, STR_DFU_HardwareData };

usb_string_descriptor_t* getStringDescriptor( const uint8_t index )
{
    switch( index )
    {
        case USB_STR_ZERO: return &usb_string_descriptor_langid;
        case USB_STR_MANUFACTURER: return &usb_string_descriptor_manufacturer;
        case USB_STR_PRODUCT: return &usb_string_descriptor_product;
        case USB_STR_DFU_App: return &usb_string_descriptor_dfu_app;
        case USB_STR_DFU_CalibrationData: return &usb_string_descriptor_dfu_calibrationData;
        case USB_STR_DFU_Bootloader: return &usb_string_descriptor_dfu_bootloader;
        case USB_STR_DFU_HardwareData: return &usb_string_descriptor_dfu_hardwareData;
        case USB_STR_SERIAL_NUMBER: return &usb_string_descriptor_serial;
        default:
            return 0;
    }
}

#endif