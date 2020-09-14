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

#ifndef _USB_H_
#define _USB_H_

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "utils.h"

/*- Definitions -------------------------------------------------------------*/
enum
{
  USB_GET_STATUS        = 0,
  USB_CLEAR_FEATURE     = 1,
  USB_SET_FEATURE       = 3,
  USB_SET_ADDRESS       = 5,
  USB_GET_DESCRIPTOR    = 6,
  USB_SET_DESCRIPTOR    = 7,
  USB_GET_CONFIGURATION = 8,
  USB_SET_CONFIGURATION = 9,
  USB_GET_INTERFACE     = 10,
  USB_SET_INTERFACE     = 11,
  USB_SYNCH_FRAME       = 12,
};

enum
{
  USB_DEVICE_DESCRIPTOR                    = 1,
  USB_CONFIGURATION_DESCRIPTOR             = 2,
  USB_STRING_DESCRIPTOR                    = 3,
  USB_INTERFACE_DESCRIPTOR                 = 4,
  USB_ENDPOINT_DESCRIPTOR                  = 5,
  USB_DEVICE_QUALIFIER_DESCRIPTOR          = 6,
  USB_OTHER_SPEED_CONFIGURATION_DESCRIPTOR = 7,
  USB_INTERFACE_POWER_DESCRIPTOR           = 8,
  USB_OTG_DESCRIPTOR                       = 9,
  USB_DEBUG_DESCRIPTOR                     = 10,
  USB_INTERFACE_ASSOCIATION_DESCRIPTOR     = 11,
  USB_BINARY_OBJECT_STORE_DESCRIPTOR       = 15,
  USB_DEVICE_CAPABILITY_DESCRIPTOR         = 16,
};

enum
{
  USB_DEVICE_RECIPIENT     = 0,
  USB_INTERFACE_RECIPIENT  = 1,
  USB_ENDPOINT_RECIPIENT   = 2,
  USB_OTHER_RECIPIENT      = 3,
};

enum
{
  USB_STANDARD_REQUEST     = 0,
  USB_CLASS_REQUEST        = 1,
  USB_VENDOR_REQUEST       = 2,
};

enum
{
  USB_OUT_TRANSFER         = 0,
  USB_IN_TRANSFER          = 1,
};

enum
{
  USB_IN_ENDPOINT          = 0x80,
  USB_OUT_ENDPOINT         = 0x00,
  USB_INDEX_MASK           = 0x7f,
  USB_DIRECTION_MASK       = 0x80,
};

enum
{
  USB_CONTROL_ENDPOINT     = 0 << 0,
  USB_ISOCHRONOUS_ENDPOINT = 1 << 0,
  USB_BULK_ENDPOINT        = 2 << 0,
  USB_INTERRUPT_ENDPOINT   = 3 << 0,

  USB_NO_SYNCHRONIZATION   = 0 << 2,
  USB_ASYNCHRONOUS         = 1 << 2,
  USB_ADAPTIVE             = 2 << 2,
  USB_SYNCHRONOUS          = 3 << 2,

  USB_DATA_ENDPOINT        = 0 << 4,
  USB_FEEDBACK_ENDPOINT    = 1 << 4,
  USB_IMP_FB_DATA_ENDPOINT = 2 << 4,
};


typedef enum
{
    MS_OS_20_SET_HEADER_DESCRIPTOR = 0x00,
    MS_OS_20_SUBSET_HEADER_CONFIGURATION = 0x01,
    MS_OS_20_SUBSET_HEADER_FUNCTION = 0x02,
    MS_OS_20_FEATURE_COMPATBLE_ID = 0x03,
    MS_OS_20_FEATURE_REG_PROPERTY = 0x04,
    MS_OS_20_FEATURE_MIN_RESUME_TIME = 0x05,
    MS_OS_20_FEATURE_MODEL_ID = 0x06,
    MS_OS_20_FEATURE_CCGP_DEVICE = 0x07,
    MS_OS_20_FEATURE_VENDOR_REVISION = 0x08
} microsoft_os_20_type_t;

/*- Types -------------------------------------------------------------------*/
typedef struct PACK
{
  uint8_t   bmRequestType;
  uint8_t   bRequest;
  uint16_t  wValue;
  uint16_t  wIndex;
  uint16_t  wLength;
} usb_request_t;

typedef struct PACK
{
  uint8_t   bLength;
  uint8_t   bDescriptorType;
  uint16_t  bcdUSB;
  uint8_t   bDeviceClass;
  uint8_t   bDeviceSubClass;
  uint8_t   bDeviceProtocol;
  uint8_t   bMaxPacketSize0;
  uint16_t  idVendor;
  uint16_t  idProduct;
  uint16_t  bcdDevice;
  uint8_t   iManufacturer;
  uint8_t   iProduct;
  uint8_t   iSerialNumber;
  uint8_t   bNumConfigurations;
} usb_device_descriptor_t;

typedef struct PACK
{
  uint8_t   bLength;
  uint8_t   bDescriptorType;
  uint16_t  wTotalLength;
  uint8_t   bNumInterfaces;
  uint8_t   bConfigurationValue;
  uint8_t   iConfiguration;
  uint8_t   bmAttributes;
  uint8_t   bMaxPower;
} usb_configuration_descriptor_t;

typedef struct PACK
{
  uint8_t   bLength;
  uint8_t   bDescriptorType;
  uint8_t   bInterfaceNumber;
  uint8_t   bAlternateSetting;
  uint8_t   bNumEndpoints;
  uint8_t   bInterfaceClass;
  uint8_t   bInterfaceSubClass;
  uint8_t   bInterfaceProtocol;
  uint8_t   iInterface;
} usb_interface_descriptor_t;

#if 0
typedef struct PACK
{
    uint8_t   bLength;
    uint8_t   bDescriptorType;
    uint16_t  wLANGID;
} usb_string_descriptor_zero_t;
#endif

#if USE_STRING_DESCRIPTORS
typedef struct PACK
{
    uint8_t   bLength;
    uint8_t   bDescriptorType;
    uint16_t  bString[];
} usb_string_descriptor_t;
#endif

#if 0 //< MS v1.0 Descriptors

/** Microsoft WCID descriptor
* @see https://docs.microsoft.com/en-us/windows-hardware/drivers/usbcon/microsoft-defined-usb-descriptors
*/
typedef struct PACK
{
    uint8_t bFirstInterfaceNumber;
    uint8_t reserved1;
    uint8_t compatibleID[8];
    uint8_t subCompatibleID[8];
    uint8_t reserved2[6];
} USB_MicrosoftCompatibleDescriptor_Interface;

typedef struct PACK
{
    uint32_t dwLength;
    uint16_t bcdVersion;
    uint16_t wIndex;
    uint8_t bCount;
    uint8_t reserved[7];
    USB_MicrosoftCompatibleDescriptor_Interface interfaces[];
} usb_microsoft_compat_descriptor_t;

typedef struct PACK
{
    uint32_t dwSize;
    uint32_t dwPropertyDataType;
    uint16_t  wPropertyNameLength;
    uint16_t  bPropertyName[20];
    uint32_t dwPropertyDataLength;
    uint16_t  bPropertyData[39];
} USB_MicrosoftCustomProperty_Section;

typedef struct PACK
{
    // Header
    uint32_t dwLength;
    uint16_t bcdVersion;
    uint16_t wIndex;
    uint16_t wCount;
    USB_MicrosoftCustomProperty_Section sections[];
} usb_microsoft_extended_properties_t;

#else //< MS v2.0 Descriptors

// USB Binary Device Object Store (BOS)
// https://developers.google.com/web/fundamentals/native-hardware/build-for-webusb/
typedef struct PACK
{
    uint8_t bLength; ///< Size of this descriptor
    uint8_t bDescriptorType; ///<  = USB_DEVICE_CAPABILITY_DESCRIPTOR Device capability descriptor
    uint8_t bDevCapabilityType; ///<  = 0x05 Platform capability descriptor
    uint8_t bReserved; ///< = 0x00;
    uint8_t PlatformCapabilityUUID[16]; ///< WebUSB platform capability descriptor GUID in little-endian format
    //uint8_t CapabilityData[];
} usb_desc_bos_platform_t;

typedef struct PACK 
{
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint16_t wTotalLength;
    uint8_t bNumDeviceCaps;
} usb_bos_descriptor_header_t;


typedef struct PACK
{
    uint32_t dwWindowsVersion; ///< = 0x06030000 Windows version - Minimum Windows version (8.1) (0x06030000)
    uint16_t wMSOSDescriptorSetTotalLength; ///< The length, in bytes of the MS OS 2.0 descriptor set
    uint8_t bMS_VendorCode; ///< Vendor defined code to use to retrieve this version of the MS OS 2.0 descriptor and also to set alternate enumeration behavior on the device.
    uint8_t bAltEnumCode; ///< = 0x00 A non-zero value to send to the device to indicate that the device may return non-default USB descriptors for enumeration.  If the device does not support alternate enumeration, this value shall be 0.
} usb_bos_ms_os_20_capability_t;

typedef struct PACK
{
    usb_desc_bos_platform_t platform;
    usb_bos_ms_os_20_capability_t capability;
} usb_bos_ms_os_20_descriptor_t;

///Microsoft OS 2.0 descriptor set header
typedef struct PACK
{
    uint16_t wLength; ///< The length, in bytes, of this header. Shall be set to 10.
    uint16_t wDescriptorType; ///< MSOS20_SET_HEADER_DESCRIPTOR
    uint32_t dwWindowsVersion; ///< Windows version.
    uint16_t wTotalLength; ////< The size of entire MS OS 2.0 descriptor set. The value shall match the value in the descriptor set information structure.  
} usb_ms_os_20_descriptor_set_header_t;

///Microsoft OS 2.0 configuration subset header
typedef struct PACK
{
    uint16_t wLength; ///< The length, in bytes, of this subset header. Shall be set to 8.
    uint16_t wDescriptorType; ///< MS_OS_20_SUBSET_HEADER_CONFIGURATION
    uint8_t bConfigurationValue; ///< The configuration value for the USB configuration to which this subset applies
    uint8_t bReserved; ///< Shall be set to 0.
    uint16_t wTotalLength; ////< The size of entire MS OS 2.0 descriptor set. The value shall match the value in the descriptor set information structure.  
} usb_ms_os_20_configuration_subset_header_t;

/// Microsoft OS 2.0 function subset header
typedef struct PACK
{
    uint16_t wLength; ///< The length, in bytes, of this subset header. Shall be set to 8.
    uint16_t wDescriptorType; ///< MS_OS_20_SUBSET_HEADER_FUNCTION
    uint8_t bFirstInterface; ///< The interface number for the first interface of the function to which this subset applies.
    uint8_t bReserved; ///< Shall be set to 0.
    uint16_t wSubsetLength; ////< The size of entire function subset including this header.
} usb_ms_os_20_function_subset_header_t;

/// Microsoft OS 2.0 compatible ID descriptor
typedef struct PACK
{
    uint16_t wLength; ///< The length, bytes, of the compatible ID descriptor including value descriptors. Shall be set to 20.
    uint16_t wDescriptorType; ///< MS_OS_FEATURE_COMPATIBLE_ID
    uint8_t compatibleID[8]; ///< Compatible ID String
    uint8_t subCompatibleID[8]; ///< Sub-compatible ID String
} usb_ms_os_20_compatible_id_descriptor_t;

/// MS OS 2.0 Registry property descriptor: length, type
typedef struct PACK
{
    uint16_t wLength; ///< The length, in bytes, of this descriptor.
    uint16_t wDescriptorType; ///< MS_OS_20_FEATURE_REG_PROPERTY
    uint16_t  wPropertyDataType; ///< The type of registry property. See Table 15.
    uint16_t  wPropertyNameLength; ///< The length of the property name.
    uint16_t  bPropertyName[20];
    uint16_t dwPropertyDataLength; ///< The length of property data.
    uint16_t  bPropertyData[38];
} usb_ms_os_20_device_interface_guid_section_t;

typedef struct PACK
{
    uint16_t bcdVersion;/// = 0x0100 WebUSB descriptor version 1.0
    uint8_t bVendorCode;///	= 0x01 bRequest value for WebUSB
    uint8_t iLandingPage;/// = 0x01 URL for landing page
} usb_bos_webusb_capability_t;

typedef struct PACK
{
    usb_desc_bos_platform_t platform;
    usb_bos_webusb_capability_t capability;
} usb_bos_webusb_descriptor_t;

// USB WebuSB URL Descriptor
typedef struct PACK
{
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint8_t bScheme;
    char    url[];
} usb_desc_webusb_url_t;

#endif

/*- Prototypes --------------------------------------------------------------*/

#endif // _USB_H_
