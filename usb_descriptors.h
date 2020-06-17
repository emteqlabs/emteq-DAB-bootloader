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
#endif
};

typedef enum
{
      USB_ALTERNATESETTING_App
    , USB_ALTERNATESETTING_Bootloader
    , USB_ALTERNATESETTING_HardwareData
    , USB_ALTERNATESETTING_CalibrationData

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
    usb_interface_descriptor_t      dfuApp;
} usb_configuration_hierarchy_standard_t;

typedef struct PACK
{
    usb_interface_descriptor_t      dfuBootloader;
    usb_interface_descriptor_t      dfuHardwareData;
    usb_interface_descriptor_t      dfuCalibrationData;
} usb_configuration_hierarchy_extended_t;

typedef struct PACK
{
    usb_configuration_hierarchy_standard_t standard;
    usb_configuration_hierarchy_extended_t extended;
} usb_configuration_hierarchy_t;

//-----------------------------------------------------------------------------
extern usb_device_descriptor_t usb_device_descriptor;
extern usb_configuration_hierarchy_t usb_configuration_hierarchy;

#if USE_STRING_DESCRIPTORS
typedef struct PACK
{
    uint8_t   bLength;
    uint8_t   bDescriptorType;
    uint16_t  bString[];
} usb_string_descriptor_t;

usb_string_descriptor_t* getStringDescriptor(const  uint8_t index);

extern uint_least16_t usb_serial_number[16];
#endif

#endif // _USB_DESCRIPTORS_H_

