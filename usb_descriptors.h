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

#ifndef _USB_DESCRIPTORS_H_
#define _USB_DESCRIPTORS_H_

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h> //< uint_least16_t ///@todo Why isn't char16_t defined!?
#include "usb.h"
#include "utils.h"

static const uint16_t cBcdVersion = 0x0101;  //< Binary-Coded-Decimal: Major[2].Minor[2]

/*- Definitions -------------------------------------------------------------*/
enum
{
    USB_STR_ZERO,
#if USE_STRING_DESCRIPTORS
    USB_STR_MANUFACTURER,
    USB_STR_PRODUCT,
    USB_STR_SERIAL_NUMBER,
    USB_STR_DFU_App,
    USB_STR_DFU_Bootloader,
    USB_STR_DFU_CalibrationData,
    USB_STR_DFU_HardwareData,
    USB_STR_WCID_Microsoft = 0xEE ///< @note Windows does not implicitly provide driver, WCID allows specifying the use of WinUSB default
#endif
};

typedef enum
{
#if DFU_BOOT 
      USB_ALTERNATESETTING_App
#elif DFU_APP
      USB_ALTERNATESETTING_Bootloader
    , USB_ALTERNATESETTING_HardwareData
    , USB_ALTERNATESETTING_CalibrationData
#else
#error "Unknown DFU setup"
#endif
    /// Sentinal
    , USB_ALTERNATESETTING_COUNT
} AlternateSettings;
    

/*- Types -------------------------------------------------------------------*/
typedef struct PACK
{
  uint8_t   bLength;
  uint8_t   bDescriptorType;
  uint8_t   bmAttributes;
  uint16_t  wDetachTimeout;
  uint16_t  wTransferSize;
  uint16_t  bcdDFU;
} usb_dfu_descriptor_t;

typedef struct PACK
{
    usb_configuration_descriptor_t  configuration;
    usb_dfu_descriptor_t            dfu;
} usb_configuration_hierarchy_standard_t;

typedef struct PACK
{
    usb_interface_descriptor_t      dfuApp;
} usb_configuration_hierarchy_boot_t;

typedef struct PACK
{
    usb_interface_descriptor_t      dfuBootloader;
    usb_interface_descriptor_t      dfuHardwareData;
    usb_interface_descriptor_t      dfuCalibrationData;
} usb_configuration_hierarchy_app_t;

typedef struct PACK
{
    usb_configuration_hierarchy_standard_t standard;
#if DFU_BOOT 
    usb_configuration_hierarchy_boot_t boot;
#elif DFU_APP
    usb_configuration_hierarchy_app_t app;
#else
#error "Unknown DFU setup"
#endif
} usb_configuration_hierarchy_t;

typedef struct PACK
{
    usb_bos_descriptor_header_t header;
    usb_bos_webusb_descriptor_t webusb;
    usb_bos_ms_os_20_descriptor_t ms_os_20;
} usb_bos_descriptor_hierarchy_t;

typedef struct PACK
{
    usb_ms_os_20_descriptor_set_header_t header;

#if 0 /// NOT Composite!
    usb_ms_os_20_configuration_subset_header_t config0_subset_header;
    usb_ms_os_20_function_subset_header_t func0_subset_header;
#endif

    usb_ms_os_20_compatible_id_descriptor_t compatible_id;
    usb_ms_os_20_device_interface_guid_section_t device_interface_guid;
} usb_ms_os_20_descriptor_set_t;

enum
{
    VENDOR_REQUEST_WEBUSB = 0x1,
    VENDOR_REQUEST_MICROSOFT = 0x2
};

//-----------------------------------------------------------------------------
extern usb_device_descriptor_t usb_device_descriptor;
extern usb_configuration_hierarchy_t usb_configuration_hierarchy;
extern usb_bos_descriptor_hierarchy_t usb_bos_descriptor_hierarchy;
extern usb_ms_os_20_descriptor_set_t usb_ms_os_20_descriptor_set;
extern usb_desc_webusb_url_t usb_webusb_url_set;

#if USE_STRING_DESCRIPTORS
usb_string_descriptor_t* getStringDescriptor(const  uint8_t index);

#endif

#endif // _USB_DESCRIPTORS_H_

