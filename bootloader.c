/*
 * 1kByte USB DFU bootloader for Atmel SAMD11 microcontrollers
 *
 * Copyright (c) 2018-2020, Peter Lawrence
 * derived from https://github.com/ataradov/vcp Copyright (c) 2016, Alex Taradov <alex@taradov.com>
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

 /*
 NOTES:
 - anything pointed to by udc_mem[*].*.ADDR.reg *MUST* BE IN RAM and be 32-bit aligned... no exceptions
 */


 /*- Includes ----------------------------------------------------------------*/
#include <stdbool.h>
#include <string.h>
#include <sam.h>
#include "common.h"
#include "dfu.h"
#include "usb.h"
#include "nvm_data.h"
#include "usb_descriptors.h"

/*- Definitions -------------------------------------------------------------*/
#define USE_DBL_TAP /* comment out to use GPIO input for bootloader entry */
#define USB_CMD(dir, rcpt, type) ((USB_##dir##_TRANSFER << 7) | (USB_##type##_REQUEST << 5) | (USB_##rcpt##_RECIPIENT << 0))
#define SIMPLE_USB_CMD(rcpt, type) ((USB_##type##_REQUEST << 5) | (USB_##rcpt##_RECIPIENT << 0))

#ifndef F_CPU
# error "Macro F_CPU must be defined"
#endif

#if !defined(__SAMD11__) && !defined(__SAMD51__)
# error "Macro __SAMD11__ or  __SAMD51__ must be defined"
#endif


#ifdef USE_DBL_TAP
uint32_t resetMagic __attribute__((section( ".resetMagic" )));
#endif

/** Generated version tag from 'git --no-pager describe --tags --always --dirty'
@note Run update_version.ps1  (Linux users need install PowerShell to run this script)
*/
const char cVersionTag[] __attribute__( (section( ".versionTag" )) ) __attribute__( (__used__) ) =
#if __has_include("Version.generated.h")
#include "Version.generated.h"
#else
#error "Please run 'update_version.ps1' to generate version information"
#endif
;

/** @note "9.4 NVM User Page Mapping" The first eight 32-bit words (32 Bytes) of the Non Volatile Memory (NVM) User Page contain calibration data that are
automatically read at device power on.The remaining 480 Bytes can be used for storing custom parameters
*/
uint32_t userPageReserved[8] __attribute__((section( ".userPageReserved" )));

/*- Types -------------------------------------------------------------------*/
typedef struct
{
    UsbDeviceDescBank  out;
    UsbDeviceDescBank  in;
} udc_mem_t;

/*- Variables ---------------------------------------------------------------*/
static __attribute__( (aligned( 4 )) ) dfu_getstatus_t dfu_status =
{
      .bStatus = OK
    , .bState = dfuIDLE
    , .bwPollTimeout = {1,0,0}
};

static __attribute__( (aligned( 4 )) ) udc_mem_t udc_mem[USB_EPT_NUM];
static __attribute__( (aligned( 4 )) ) uint8_t udc_ctrl_in_buf[dfu_blockSize];
static __attribute__( (aligned( 4 )) ) uint8_t udc_ctrl_out_buf[dfu_blockSize];

extern char __origin_APP_FLASH[],  __length_APP_FLASH[]; ///< @note Defined in .ld linker
extern char __origin_BOOT_FLASH[], __length_BOOT_FLASH[]; ///< @note Defined in .ld linker
extern char __origin_USERPAGE_FLASH[], __length_USERPAGE_FLASH[]; ///< @note Defined in .ld linker
extern char __origin_CALDATA_FLASH[], __length_CALDATA_FLASH[]; ///< @note Defined in .ld linker
extern char __origin_HWDATA_FLASH[], __length_HWDATA_FLASH[]; ///< @note Defined in .ld linker
extern char __stack_start__[], __stack_end__[]; ///< @note Defined in .ld linker


static void nvmctrl_wait_ready()
{
#if __SAMD11__
    while( !NVMCTRL->INTFLAG.bit.READY );
#elif __SAMD51__
    while( !NVMCTRL->STATUS.bit.READY );
#else
#error "Unsupported processor class"
#endif
}

/*Before erasing the NVM User Page, ensure that the first 32 Bytes are read to a buffer and later written back
* to the same area unless a configuration change is intended
*/
void nvmctrl_erase_userpage()
{
    uint8_t userPageReservedBuffer[sizeof( userPageReserved )];

    memcpy( userPageReservedBuffer, userPageReserved, sizeof( userPageReserved ) );

    // Execute "EP" Erase Page (512Bytes region to 0xFFFFF)
    nvmctrl_wait_ready();
    NVMCTRL->ADDR.reg = (uint32_t)__origin_USERPAGE_FLASH;
    NVMCTRL->CTRLB.reg = NVMCTRL_CTRLB_CMDEX_KEY | NVMCTRL_CTRLB_CMD_EP;
    nvmctrl_wait_ready();

    //Restore the reserved data section contents
    memcpy( userPageReserved, userPageReservedBuffer, sizeof( userPageReserved ) );

#if 0 ///< @todo will this be necessary?
    //Commit the last quad-word in page-buffer to ensure the reserved data is commited
    nvmctrl_wait_ready();
    NVMCTRL->CTRLB.reg = NVMCTRL_CTRLB_CMDEX_KEY | NVMCTRL_CTRLB_CMD_WQW;
    nvmctrl_wait_ready();
#endif
}

/** Write of an incomplete quad-word (SAMD1) or row (SAMD11) must be flushed explicitly
*/
static void nvmctrl_write_flush()
{
    //Check if written up to a page boundary i.e. Automatic page-write shall have occurred
    if( 0 == (NVMCTRL->ADDR.bit.ADDR % NVMCTRL_PAGE_SIZE) )
        return;

    nvmctrl_wait_ready();

    //Page size: NVMCTRL_PAGE_SIZE==512 on SAMD51 and NVMCTRL_PAGE_SIZE==64 on SAMD11
#if __SAMD11__
    NVMCTRL->CTRLB.reg = NVMCTRL_CTRLA_CMDEX_KEY | NVMCTRL_CTRLA_CMD_WP;
#else
    NVMCTRL->CTRLB.reg = NVMCTRL_CTRLB_CMDEX_KEY | NVMCTRL_CTRLB_CMD_WP;
#endif
    nvmctrl_wait_ready();
}

#if __SAMD11__
static void nvmctrl_erase_row( uint32_t addr )
{
    nvmctrl_wait_ready();

    //Execute Erase-Row command
    // @note A row contains 4 pages
    NVMCTRL->ADDR.reg = addr / 2;
    NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY | NVMCTRL_CTRLA_CMD_ER;
    nvmctrl_wait_ready();
}
#elif __SAMD51__
static void nvmctrl_erase_block( uint32_t dst )
{
    nvmctrl_wait_ready();

    // Execute "EB" Erase block (8K region to 0xFFFF)
    NVMCTRL->ADDR.reg = dst;
    NVMCTRL->CTRLB.reg = NVMCTRL_CTRLB_CMDEX_KEY | NVMCTRL_CTRLB_CMD_EB;
    nvmctrl_wait_ready();
}
#endif


typedef bool (*fnNvmCtrlPreWrite)(const uint32_t nvm_addr, const uint32_t nvm_writeLength );

/** When writing the user-page the first 32-bytes are reserved by Samd specification.
*/
bool nvmctrl_userpage_PreWrite( const uint32_t nvm_addr, const uint32_t nvm_writeLength )
{
    (void)nvm_writeLength; //< unused

    // @note Sanity check that CalData is inside the UserPage!
   ///@todo assert( nvm_addr >= (uint32_t)__origin_CALDATA_FLASH && nvm_addr <= (uint32_t)__origin_CALDATA_FLASH + (uint32_t)__length_CALDATA_FLASH );

    //TODO; Check data to be written changes and 0's to 1's?

    if( nvm_addr == (uint32_t)__origin_CALDATA_FLASH )
        nvmctrl_erase_userpage();

    return true;
}

bool nvmctrl_bootloader_PreWrite( const uint32_t nvm_addr, const uint32_t nvm_writeLength )
{
    (void)nvm_addr; //< unused
    (void)nvm_writeLength; //< unused

   ///@todo assert( false == "Not implemented!" );
    return false;
}

bool nvmctrl_main_PreWrite( const uint32_t nvm_addr, const uint32_t nvm_writeLength )
{
    (void)nvm_writeLength; //< unused

    /// @todo MUST store in RAM before writing so we only do an erase if necessary
#if __SAMD11__ 
    /** The NVM is organized into rows, where each row contains four pages
        The NVM has a rowerase granularity, while the write granularity is by page. In other words, a single row erase will erase all four pages in
        the row, while four write operations are used to write the complete row.
        */
    if( 0 == (nvm_addr % NVMCTRL_ROW_SIZE) )
    {
        /// @todo Cache row and only erase if content differs to save wear
        nvmctrl_erase_row( nvm_addr );
    }
    // @note "WP" Write page and "PBC" Page Buffer Clear are not necessary while NVMCTRL->CTRLB.bit.MANW == 0;
#elif __SAMD51__
    /// erase at start of new block
    /// @todo Could erase at end of current block only/if block content changes etc
    if( 0 == (nvm_addr % NVMCTRL_BLOCK_SIZE) )
    {
        /// @todo Cache row and only erase if content differs to save wear
        nvmctrl_erase_block( nvm_addr );
    }
#else
#error "Unsupported processor class"
#endif

    return true;
}
typedef struct 
{
    uint32_t origin;
    uint32_t length;
    fnNvmCtrlPreWrite preWrite;
} Partition;

static const Partition partition[USB_ALTERNATESETTING_COUNT] =
{
     [USB_ALTERNATESETTING_App] = { (uint32_t)__origin_APP_FLASH, (uint32_t)__length_APP_FLASH, nvmctrl_main_PreWrite }
   , [USB_ALTERNATESETTING_Bootloader] = { (uint32_t)__origin_BOOT_FLASH, (uint32_t)__length_BOOT_FLASH, nvmctrl_bootloader_PreWrite }
   , [USB_ALTERNATESETTING_HardwareData] = { (uint32_t)__origin_HWDATA_FLASH, (uint32_t)__length_HWDATA_FLASH, nvmctrl_bootloader_PreWrite }
   , [USB_ALTERNATESETTING_CalibrationData] = { (uint32_t)__origin_CALDATA_FLASH, (uint32_t)__length_CALDATA_FLASH, nvmctrl_userpage_PreWrite }
};

static volatile uint32_t* userAppStackPointer = (volatile uint32_t*)(partition[USB_ALTERNATESETTING_App].origin + userAppStackVectorOffset);
static volatile uint32_t* userAppResetVector = (volatile uint32_t*)(partition[USB_ALTERNATESETTING_App].origin + userAppResetVectorOffset);
static volatile uint32_t* userAppCrcVector = (volatile uint32_t*)(partition[USB_ALTERNATESETTING_App].origin + userAppCrcEmbedOffset);
static volatile uint32_t* userAppLengthVector = (volatile uint32_t*)(partition[USB_ALTERNATESETTING_App].origin + userAppLengthEmbedOffset);

static uint8_t usb_status[2] = { 0, 0 };
static uint8_t usb_config = 0;
static uint8_t usb_alternateSetting = USB_ALTERNATESETTING_App; //, Sets the partition in DFU mode
static uint16_t dfu_writeBlockIndex = 0;
static uint16_t dfu_writeSize = 0;

/*- Implementations ---------------------------------------------------------*/

/// red LED
static void bootloaderStarted()
{
    // LEDs on PA07 & PA08
    PORT->Group[0].DIRSET.reg = (1 << 8) | (1 << 7); // Set pin to output mode
    PORT->Group[0].PINCFG[7].reg = PORT->Group[0].PINCFG[8].reg = (uint8_t)(PORT_PINCFG_INEN);

    PORT->Group[0].OUTSET.reg = (1 << 8); // OFF:Drive HIGH
    PORT->Group[0].OUTCLR.reg = (1 << 7); // ON: Drive low
}

/// Red LED
static void dfuStarted()
{
    PORT->Group[0].OUTSET.reg = (1 << 8); // OFF:Drive HIGH
    PORT->Group[0].OUTCLR.reg = (1 << 7); // ON: Drive low
}

/// Yellow LED
static void dfuUsbActive()
{
    PORT->Group[0].OUTCLR.reg = (1 << 7) | (1 << 8); // ON: Drive low
}

/// Red LED
static void dfuUsbDeactive()
{
    dfuStarted();
}


/// OFF LED
static void userAppStarted()
{
    PORT->Group[0].OUTSET.reg = (1 << 7) | (1 << 8); // OFF:Drive HIGH
}

//-----------------------------------------------------------------------------
static void udc_control_send( const uint8_t* const data, const uint32_t size )
{
    /* USB peripheral *only* reads valid data from 32-bit aligned RAM locations */
    udc_mem[0].in.ADDR.reg = (uint32_t)data;

    udc_mem[0].in.PCKSIZE.reg = USB_DEVICE_PCKSIZE_BYTE_COUNT( size )
        | USB_DEVICE_PCKSIZE_MULTI_PACKET_SIZE( 0 )
        | USB_DEVICE_PCKSIZE_SIZE( dfu_endpointSize /*64 SAMD11 or 512 SAMD51 */ );
    /// @ todo MULTI_PACKET_SIZE?

    USB->DEVICE.DeviceEndpoint[0].EPINTFLAG.reg = USB_DEVICE_EPINTFLAG_TRCPT1; //< clear
    USB->DEVICE.DeviceEndpoint[0].EPSTATUSSET.bit.BK1RDY = 1;

    while( !USB->DEVICE.DeviceEndpoint[0].EPINTFLAG.bit.TRCPT1 )
    {
        //if ( USB->DEVICE.DeviceEndpoint[0].EPINTFLAG.bit.TRFAIL1 )
        //  break;
    }
}



//-----------------------------------------------------------------------------
static void udc_control_send_zlp( void )
{
    udc_control_send( NULL, 0 ); /* peripheral can't read from NULL address, but size is zero and this value takes less space to compile */
}


static void dfuDownloadToNvm()
{
    const uint32_t nvm_addr = partition[usb_alternateSetting].origin + (dfu_writeBlockIndex * dfu_blockSize);

    // Make sure we don't write past the end of the partition
    const uint32_t nvm_writeLength = MIN( MIN( dfu_writeSize, sizeof( udc_ctrl_out_buf )), partition[usb_alternateSetting].length - (dfu_writeBlockIndex * dfu_blockSize) );

    partition[usb_alternateSetting].preWrite( nvm_addr, nvm_writeLength );

    typedef uint32_t NvmWord;
    ///@todo assert( nvm_writeLength % sizeof( NvmWord ) == 0 );// Can only write multiples of 32bit
    NvmWord* nvm_writeCursor = (NvmWord*)(nvm_addr);
    NvmWord* ram_readCursor = (NvmWord*)udc_ctrl_out_buf;
    for( unsigned i = 0; i < nvm_writeLength / sizeof( NvmWord ); ++i )
        *nvm_writeCursor++ = *ram_readCursor++;

    nvmctrl_wait_ready();
}

static bool checkCrcRegion( const uint32_t address, const uint32_t length )
{
#if __SAMD11__ 
    PAC1->WPCLR.reg = 2; /* clear DSU */
#elif __SAMD51__
    PAC->WRCTRL.reg = PAC_WRCTRL_PERID( ID_DSU ) | PAC_WRCTRL_KEY_CLR;
#else
#error "Unsupported processor class"
#endif

    /// @todo CRC calculate during flashing operation as async task
    DSU->ADDR.reg = address;
    DSU->LENGTH.reg = length; /* use length encoded into unused vector address in user app */

    /* ask DSU to compute CRC */
    DSU->STATUSA.bit.DONE = true;
    DSU->DATA.reg = 0xFFFFFFFF;
    DSU->CTRL.bit.CRC = 1;
    while( !DSU->STATUSA.bit.DONE );

    const uint32_t dsuCrcData = DSU->DATA.reg;

    return dsuCrcData == 0 && !(DSU->STATUSA.bit.PERR || DSU->STATUSA.bit.BERR);
}

static bool userImageValid()
{
    /// Check on the Stack-pointer address points somewhere in the RAM
    const uint32_t currentUserAppStackPointer = *userAppStackPointer;
    if( (currentUserAppStackPointer <= (uint32_t)__stack_start__)
        || (currentUserAppStackPointer > (uint32_t)__stack_end__ ) )
    {
        return false;
    }

    /// Check on the Reset_Handler address needs to point inside the Flash region for user-app
    const uint32_t currentUserAppResetVector = *userAppResetVector;
    if( (currentUserAppResetVector < partition[USB_ALTERNATESETTING_App].origin)
        || (currentUserAppResetVector >= partition[USB_ALTERNATESETTING_App].origin + partition[usb_alternateSetting].length) )
    {
        return false;
    }

    // If we don't use CRC then assume it must e correct!
    if( !userAppCrcEmbed )
        return true;

    //If user-app CRC is 0 then we assume invalid image
    const uint32_t userAppCrc = *userAppCrcVector;
    if( userAppCrc == 0 )
        return false;

    //If user-app is 0 bytes or extends past end of available flash then it must be invalid
    const uint32_t userAppLength = *userAppLengthVector;
    if( (userAppLength == 0)
        || (userAppLength > partition[usb_alternateSetting].length) )
        return false;

    return checkCrcRegion( partition[USB_ALTERNATESETTING_App].origin, userAppLength );
}

static bool USB_Service()
{
    if( USB->DEVICE.INTFLAG.bit.EORST ) /* End Of Reset */
    {
        USB->DEVICE.INTFLAG.reg = USB_DEVICE_INTFLAG_EORST;

        USB->DEVICE.DADD.reg = USB_DEVICE_DADD_ADDEN;

        for( int ep = 1; ep < USB_EPT_NUM; ep++ )
        {
            USB->DEVICE.DeviceEndpoint[ep].EPCFG.reg = 0;
        }

        USB->DEVICE.DeviceEndpoint[0].EPCFG.reg = USB_DEVICE_EPCFG_EPTYPE0( 1 /*CONTROL*/ ) | USB_DEVICE_EPCFG_EPTYPE1( 1 /*CONTROL*/ );
        USB->DEVICE.DeviceEndpoint[0].EPSTATUSSET.bit.BK0RDY = 1;
        USB->DEVICE.DeviceEndpoint[0].EPSTATUSCLR.bit.BK1RDY = 1;

        udc_mem[0].in.ADDR.reg = (uint32_t)udc_ctrl_in_buf;
        udc_mem[0].in.PCKSIZE.reg = USB_DEVICE_PCKSIZE_BYTE_COUNT( 0 ) | USB_DEVICE_PCKSIZE_MULTI_PACKET_SIZE( 0 ) | USB_DEVICE_PCKSIZE_SIZE( dfu_endpointSize );

        udc_mem[0].out.ADDR.reg = (uint32_t)udc_ctrl_out_buf;
        udc_mem[0].out.PCKSIZE.reg = USB_DEVICE_PCKSIZE_BYTE_COUNT(dfu_blockSize ) | USB_DEVICE_PCKSIZE_MULTI_PACKET_SIZE( 0 ) | USB_DEVICE_PCKSIZE_SIZE(dfu_endpointSize);

        USB->DEVICE.DeviceEndpoint[0].EPSTATUSCLR.bit.BK0RDY = 1;

        dfu_status.bState = dfuIDLE;
        dfu_status.bStatus = OK;
        usb_alternateSetting = USB_ALTERNATESETTING_App;
        usb_config = 0;
    }
    else
    if( USB->DEVICE.INTFLAG.bit.SUSPEND ) // SAMD doesn't distinguish between Suspend and Disconnect state.
    {
        USB->DEVICE.INTFLAG.reg = USB_DEVICE_INTFLAG_SUSPEND;
        USB->DEVICE.INTFLAG.reg = USB_DEVICE_INTFLAG_WAKEUP; // clear pending

        dfuUsbDeactive();
    }
    else
    if( USB->DEVICE.INTFLAG.bit.WAKEUP ) // SAMD doesn't distinguish between Suspend and Disconnect state.
    {
        USB->DEVICE.INTFLAG.reg = USB_DEVICE_INTFLAG_WAKEUP;

        dfuUsbActive();
    }

    switch( dfu_status.bState )
    {
        case appDETACH:
            {
                nvmctrl_wait_ready();

                /// @todo Enable DFU RT mode instead of disabling/resetting USB?
                USB->DEVICE.CTRLA.reg = USB_CTRLA_SWRST;
                while( USB->DEVICE.SYNCBUSY.bit.SWRST );
            }
            return false;  //< Exit DFU

        case dfuDNBUSY:
            if( USB->DEVICE.DeviceEndpoint[0].EPINTFLAG.bit.TRCPT0 ) /// Transmit Complete 0
            {
                ///< Protect against flash overflow
                const uint32_t writeEndIndex = (dfu_writeBlockIndex * dfu_blockSize) + dfu_writeSize;
                if( writeEndIndex < partition[usb_alternateSetting].length )
                {
                    dfuDownloadToNvm();
                    dfu_status.bState = dfuDNLOAD_IDLE;
                }
                else
                {
                    dfu_status.bState = dfuERROR;
                    dfu_status.bStatus = errADDRESS;
                }

                udc_control_send_zlp();
                USB->DEVICE.DeviceEndpoint[0].EPINTFLAG.reg = USB_DEVICE_EPINTFLAG_TRCPT0; //< clear
            }
            break;
        case dfuMANIFEST:
            {
                if( userImageValid() )
                {
                    dfu_status.bState = dfuMANIFEST_WAIT_RESET;
                    dfu_status.bStatus = OK;
                }
                else
                {
                    dfu_status.bState = dfuERROR;
                    dfu_status.bStatus = errVERIFY;
                }
                //udc_control_send_zlp();
            }
            break;
    }

    if( !USB->DEVICE.DeviceEndpoint[0].EPINTFLAG.bit.RXSTP ) /* Received Setup */
        return true;

    USB->DEVICE.DeviceEndpoint[0].EPINTFLAG.reg = USB_DEVICE_EPINTFLAG_RXSTP;
    USB->DEVICE.DeviceEndpoint[0].EPSTATUSCLR.bit.BK0RDY = 1;

    usb_request_t* request = (usb_request_t*)udc_ctrl_out_buf;
    const uint8_t type = request->wValue >> 8;
    const uint8_t index = request->wValue & 0xff;
    const uint16_t length = request->wLength;

    // handle Microsoft thing
    if (USB_CMD(IN, DEVICE, VENDOR) == request->bmRequestType) {
      // 0x20, since we put a whitespace (=0x20) after "MSFT100" String.
      if ((request->bRequest == 0x20) && (request->wIndex == 0x0004)) {
        udc_control_send(  (const uint8_t*)&usb_wcid_microsoft, MIN(usb_wcid_microsoft.dwLength, length));
      } else {
        USB->DEVICE.DeviceEndpoint[0].EPSTATUSSET.bit.STALLRQ1 = 1;
      }
      return true;
    }

    //http://www.usbmadesimple.co.uk/ums_4.htm
    /* for these "simple" USB requests, we can ignore the direction and use only bRequest */
    switch( request->bmRequestType & 0x7F )
    {
#if 0
        case SIMPLE_USB_CMD( DEVICE, VENDOR ):
            {
                switch( request->bRequest )
                {
                    case 0x20: //< @todo CHeck the WCID spec accordingly!?
                        if( request->wIndex == 0x0004 )
                        {
                            udc_control_send( (const uint8_t*)&usb_wcid_microsoft, MIN(usb_wcid_microsoft.dwLength, length) );
                        }
                        else
                            USB->DEVICE.DeviceEndpoint[0].EPSTATUSSET.bit.STALLRQ1 = 1;
                        break;

                    default:
                        USB->DEVICE.DeviceEndpoint[0].EPSTATUSSET.bit.STALLRQ1 = 1;
                        break;
                }
            }
            break;
#endif
        case SIMPLE_USB_CMD( DEVICE, STANDARD ):
        case SIMPLE_USB_CMD( INTERFACE, STANDARD ):
            switch( request->bRequest )
            {
                case USB_GET_DESCRIPTOR:
                    if( USB_DEVICE_DESCRIPTOR == type )
                    {
                        udc_control_send( (const uint8_t*)&usb_device_descriptor, MIN( length, usb_device_descriptor.bLength) );
                    }
                    else if( USB_CONFIGURATION_DESCRIPTOR == type )
                    {
#if 1
                        udc_control_send( (const uint8_t*)&usb_configuration_hierarchy, MIN( length, usb_configuration_hierarchy.standard.configuration.wTotalLength ) );
#else
                        udc_control_send( (const uint8_t*)&usb_configuration_hierarchy.standard, MIN( length, sizeof(usb_configuration_hierarchy.standard) ) );
                        udc_control_send( (const uint8_t*)&usb_configuration_hierarchy.extended, MIN( length, sizeof(usb_configuration_hierarchy.extended) ) );
#endif
                    }
#if USE_STRING_DESCRIPTORS
                    else if( USB_STRING_DESCRIPTOR == type )
                    {
                        const usb_string_descriptor_t* stringDescriptor = getStringDescriptor( index );
                        if( stringDescriptor )
                        {
                            udc_control_send( (const uint8_t*)stringDescriptor, MIN( length, stringDescriptor->bLength ) );
                        }
                        else
                        {
                            USB->DEVICE.DeviceEndpoint[0].EPSTATUSSET.bit.STALLRQ1 = 1;
                        }
                    }
                    else
#endif
                    {
                        USB->DEVICE.DeviceEndpoint[0].EPSTATUSSET.bit.STALLRQ1 = 1;
                    }
                    break;
                case USB_GET_CONFIGURATION:
                    udc_control_send( (const uint8_t*)&usb_config, sizeof( usb_config ) );
                    break;
                case USB_GET_STATUS:
                    udc_control_send( (const uint8_t*)&usb_status, sizeof( usb_status ) ); /* a 32-bit aligned zero in RAM is all we need */
                    break;
                case USB_SET_FEATURE:
                case USB_CLEAR_FEATURE:
                    USB->DEVICE.DeviceEndpoint[0].EPSTATUSSET.bit.STALLRQ1 = 1;
                    break;
                case USB_SET_ADDRESS:
                    udc_control_send_zlp();
                    USB->DEVICE.DADD.reg = USB_DEVICE_DADD_ADDEN | USB_DEVICE_DADD_DADD( request->wValue );
                    break;
                case USB_SET_CONFIGURATION:
                    usb_config = request->wValue;
                    udc_control_send_zlp();
                    break;
                case USB_SET_INTERFACE:
                    if( request->wValue < USB_ALTERNATESETTING_COUNT )
                    {
                        usb_alternateSetting = request->wValue;
                        udc_control_send_zlp();
                    }
                    else ///< Unexpected partition index
                    {
                        USB->DEVICE.DeviceEndpoint[0].EPSTATUSSET.bit.STALLRQ1 = 1;
                        dfu_status.bState = dfuERROR;
                        dfu_status.bStatus = errSTALLEDPKT;
                    }
                    break;
            }
            break;
        case SIMPLE_USB_CMD( INTERFACE, CLASS ):

            switch( request->bRequest )
            {
                case DFU_GETSTATUS:
                    udc_control_send( (const uint8_t*)&dfu_status, sizeof( dfu_status ) );
                    if( dfu_status.bState == dfuMANIFEST_SYNC )
                    {
                        dfu_status.bState = dfuMANIFEST;
                    }
                    break;
                case DFU_GETSTATE:
                    udc_control_send( (const uint8_t*)&dfu_status.bState, sizeof( dfu_status.bState ) );
                    break;
                case DFU_DNLOAD:
                    if( request->wLength ) //< "Download Request" 6.1.1 DFU_DNLOAD Request 
                    {
                        USB->DEVICE.DeviceEndpoint[0].EPINTFLAG.reg = USB_DEVICE_EPINTFLAG_TRCPT0; //< clear

                        dfu_status.bState = dfuDNBUSY;
                        dfu_writeSize = request->wLength;
                        dfu_writeBlockIndex = request->wValue;
                    }
                    else //< "Completion packet" 6.1.1.1 Zero Length DFU_DNLOAD Request 
                    {
                        nvmctrl_write_flush();

                        dfu_status.bState = dfuMANIFEST_SYNC;
                    }
                    break;
                case DFU_ABORT:
                    {
                        dfu_status.bState = dfuIDLE;
                        udc_control_send_zlp();
                    }
                    break;

                case DFU_CLRSTATUS:
                    {
                        dfu_status.bState = dfuIDLE;
                        dfu_status.bStatus = OK;
                        udc_control_send_zlp();
                    }
                    break;

                case DFU_DETACH:
                    dfu_status.bState = appDETACH;
                    udc_control_send_zlp();
                    break;

                case DFU_UPLOAD:
                    USB->DEVICE.DeviceEndpoint[0].EPSTATUSSET.bit.STALLRQ1 = 1;
                    dfu_status.bState = dfuERROR;
                    dfu_status.bStatus = errSTALLEDPKT;
                    break;

                default:
                    udc_control_send_zlp();
                    break;
            }
            break;
    }

    return true;
}

static void configureClock()
{
#if __SAMD11__
#if 1
    /*
    configure oscillator for crystal-free USB operation (USBCRM / USB Clock Recovery Mode)
    */

    SYSCTRL->OSC8M.bit.PRESC = 0;

    SYSCTRL->INTFLAG.reg = SYSCTRL_INTFLAG_BOD33RDY | SYSCTRL_INTFLAG_BOD33DET | SYSCTRL_INTFLAG_DFLLRDY;


    SYSCTRL->DFLLCTRL.reg = 0; // See Errata 9905
    while( !SYSCTRL->PCLKSR.bit.DFLLRDY );

    SYSCTRL->DFLLMUL.reg = SYSCTRL_DFLLMUL_MUL( 48000 );
    SYSCTRL->DFLLVAL.reg = SYSCTRL_DFLLVAL_COARSE( NVM_READ_CAL( NVM_DFLL48M_COARSE_CAL ) ) | SYSCTRL_DFLLVAL_FINE( NVM_READ_CAL( NVM_DFLL48M_FINE_CAL ) );

    SYSCTRL->DFLLCTRL.reg = SYSCTRL_DFLLCTRL_ENABLE | SYSCTRL_DFLLCTRL_USBCRM | SYSCTRL_DFLLCTRL_MODE | SYSCTRL_DFLLCTRL_BPLCKC | SYSCTRL_DFLLCTRL_CCDIS | SYSCTRL_DFLLCTRL_STABLE;

    while( !SYSCTRL->PCLKSR.bit.DFLLRDY );

    GCLK->GENCTRL.reg = GCLK_GENCTRL_ID( 0 ) | GCLK_GENCTRL_SRC( GCLK_SOURCE_DFLL48M ) | GCLK_GENCTRL_RUNSTDBY | GCLK_GENCTRL_GENEN;
    while( GCLK->STATUS.bit.SYNCBUSY );
#else
    /*
    configure oscillator for operation disciplined by external 32k crystal

    This can only be used on PCBs (such as Arduino Zero derived designs) that have these extra components populated.
    It *should* be wholly unnecessary to use this instead of the above USBCRM code.
    However, some problem (Sparkfun?) PCBs experience unreliable USB operation in USBCRM mode.
    */

    SYSCTRL->XOSC32K.reg = SYSCTRL_XOSC32K_STARTUP( 0x6u ) | SYSCTRL_XOSC32K_XTALEN | SYSCTRL_XOSC32K_EN32K;
    SYSCTRL->XOSC32K.reg |= SYSCTRL_XOSC32K_ENABLE;

    while( !(SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_XOSC32KRDY) );

    GCLK->GENDIV.reg = GCLK_GENDIV_ID( 1u /* XOSC32K */ );

    GCLK->GENCTRL.reg = GCLK_GENCTRL_ID( 1u /* XOSC32K */ ) | GCLK_GENCTRL_SRC_XOSC32K | GCLK_GENCTRL_GENEN;

    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID( 0u /* DFLL48M */ ) | GCLK_CLKCTRL_GEN_GCLK1 | GCLK_CLKCTRL_CLKEN;

    //  while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);

    SYSCTRL->DFLLCTRL.reg = 0; // See Errata 9905
  //  while (!SYSCTRL->PCLKSR.bit.DFLLRDY);

    SYSCTRL->DFLLMUL.reg = SYSCTRL_DFLLMUL_CSTEP( 31 ) | SYSCTRL_DFLLMUL_FSTEP( 511 ) | SYSCTRL_DFLLMUL_MUL( 48000000ul / 32768ul );

    SYSCTRL->DFLLCTRL.reg = SYSCTRL_DFLLCTRL_ENABLE | SYSCTRL_DFLLCTRL_MODE | SYSCTRL_DFLLCTRL_WAITLOCK | SYSCTRL_DFLLCTRL_QLDIS;

    while( !(SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLLCKC) || !(SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLLCKF) || !(SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLRDY) );

    GCLK->GENDIV.reg = GCLK_GENDIV_ID( 0u /* MAIN */ );

    GCLK->GENCTRL.reg = GCLK_GENCTRL_ID( 0u /* MAIN */ ) | GCLK_GENCTRL_SRC_DFLL48M | GCLK_GENCTRL_IDC | GCLK_GENCTRL_GENEN;

    while( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY );
#endif

    NVMCTRL->CTRLB.reg = NVMCTRL_CTRLB_RWS_DUAL; //< @note Automatic page Writes (CTRLB.MANW=0)

#elif __SAMD51__
    // Automatic wait states.
    NVMCTRL->CTRLA.bit.AUTOWS = 1;

    // Temporarily switch the CPU to the internal 32k oscillator while we reconfigure the DFLL.
    GCLK->GENCTRL[0].reg = GCLK_GENCTRL_SRC( GCLK_GENCTRL_SRC_OSCULP32K ) |
        GCLK_GENCTRL_OE |
        GCLK_GENCTRL_GENEN;

    while( GCLK->SYNCBUSY.bit.GENCTRL0 );
    OSCCTRL->DFLLCTRLA.reg = 0;// Configure the DFLL for USB clock recovery.

    OSCCTRL->DFLLMUL.reg = OSCCTRL_DFLLMUL_CSTEP( 0x1 ) |
        OSCCTRL_DFLLMUL_FSTEP( 0x1 ) |
        OSCCTRL_DFLLMUL_MUL( 0xBB80 );

    while( OSCCTRL->DFLLSYNC.bit.DFLLMUL );

    OSCCTRL->DFLLCTRLB.reg = 0;
    while( OSCCTRL->DFLLSYNC.bit.DFLLCTRLB );

    OSCCTRL->DFLLCTRLA.bit.ENABLE = true;
    while( OSCCTRL->DFLLSYNC.bit.ENABLE );

    OSCCTRL->DFLLVAL.reg = OSCCTRL->DFLLVAL.reg;
    while( OSCCTRL->DFLLSYNC.bit.DFLLVAL );

    OSCCTRL->DFLLCTRLB.reg = OSCCTRL_DFLLCTRLB_WAITLOCK |
        OSCCTRL_DFLLCTRLB_CCDIS | OSCCTRL_DFLLCTRLB_USBCRM;

    while( !OSCCTRL->STATUS.bit.DFLLRDY );

    // 5) Switch Generic Clock Generator 0 to DFLL48M. CPU will run at 48MHz.
    GCLK->GENCTRL[0].reg =
        GCLK_GENCTRL_SRC( GCLK_GENCTRL_SRC_DFLL ) |
        GCLK_GENCTRL_IDC |
        GCLK_GENCTRL_OE |
        GCLK_GENCTRL_GENEN;

    while( GCLK->SYNCBUSY.bit.GENCTRL0 );

    MCLK->CPUDIV.reg = MCLK_CPUDIV_DIV_DIV1;
#else
#error "Unsupported processor class"
#endif
}

#ifdef USE_DBL_TAP
uint32_t resetMagic __attribute__( (section( ".resetMagic" )) ) __attribute__( (__used__) );
#endif

static bool hasResetUserAppMagic()
{
    const bool hasQuickDfuMagic = (resetMagic == ResetMagic_UserApp);
    return hasQuickDfuMagic;
}

static bool hasResetBootloaderMagic()
{
#if __SAMD11__ 
    const bool isPowerOnReset = (PM->RCAUSE.bit.POR);
#elif __SAMD51__
    const bool isPowerOnReset = (RSTC->RCAUSE.bit.POR);
#else
#error "Unsupported processor class"
#endif

    const bool hasResetMagic = (resetMagic == ResetMagic_Bootloader);

    return !isPowerOnReset && hasResetMagic;
}

static void doubleTapResetDelay()
{
    /* postpone boot for a short period of time; if a second reset happens during this window, the "magic" value will remain */
    resetMagic = ResetMagic_Bootloader;
    /// @Note Default iOS double tap is .25 seconds so we use this here based on processor frequency
    const uint32_t delayCycles = F_CPU / 4; ///< Cycles to delay for boot 
    const uint32_t cyclesPerIteration = 3; ///<, Number of clocks per iteration (44,000,000 cycles on CortexM4)
    volatile uint32_t delayIterations = delayCycles / cyclesPerIteration; while( --delayIterations )
    {
        asm( "nop" );
    }
    /* however, if execution reaches this point, the window of opportunity has closed and the "magic" disappears  */
    resetMagic = 0;
}
/* pin PA15 grounded, so run bootloader */
static bool hasGroundedPA15()
{
    /* configure PA15 (bootloader entry pin used by SAM-BA) as input pull-up */
    PORT->Group[0].PINCFG[15].reg = PORT_PINCFG_PULLEN | PORT_PINCFG_INEN;
    PORT->Group[0].OUTSET.reg = (1UL << 15);

    return (!(PORT->Group[0].IN.reg & (1UL << 15)));
}

static bool hasBootloaderResetMagic()
{
#ifndef USE_DBL_TAP
    return hasGroundedPA15();
#else

    /// Bypass double-tap delay for fast user-app restart entry
    if( hasResetUserAppMagic() )
        return false;

    /// Enter bootloader via:
    /// - App requested quick bootloader entry by setting ResetMagic_Bootloader
    /// - User reset occured via double-tap external reset which means ResetMagic_Bootloader is set
    if( hasResetBootloaderMagic() )
    {
        return true;
    }

    doubleTapResetDelay();
    return false;
#endif
}

/* initialize USB
*/
static void initializeUsb()
{
#if __SAMD11__
    PORT->Group[0].PINCFG[24].reg |= PORT_PINCFG_PMUXEN;
    PORT->Group[0].PINCFG[25].reg |= PORT_PINCFG_PMUXEN;
    PORT->Group[0].PMUX[24 / 2].reg = PORT_PMUX_PMUXO( PORT_PMUX_PMUXE_G_Val ) | PORT_PMUX_PMUXE( PORT_PMUX_PMUXE_G_Val );

    PM->APBBMASK.reg |= PM_APBBMASK_USB;

    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_ID( USB_GCLK_ID ) | GCLK_CLKCTRL_GEN( 0 );

#elif __SAMD51__

    // Set up the USB DP/DN pins
    PORT->Group[0].PINCFG[PIN_PA24H_USB_DM].bit.PMUXEN = true;
    PORT->Group[0].PINCFG[PIN_PA25H_USB_DP].bit.PMUXEN = 1;
    PORT->Group[0].PMUX[PIN_PA24H_USB_DM / 2].reg = PORT_PMUX_PMUXO( MUX_PA25H_USB_DP ) | PORT_PMUX_PMUXE( MUX_PA24H_USB_DM );

    GCLK->PCHCTRL[USB_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK0_Val | GCLK_PCHCTRL_CHEN;

    /* Enable USB clock */
    MCLK->APBBMASK.bit.USB_ = true;

    while( GCLK->SYNCBUSY.bit.GENCTRL0 )
    {
    }

#else
#error "Unsupported processor class"
#endif

    USB->DEVICE.CTRLA.reg = USB_CTRLA_SWRST;
    while( USB->DEVICE.SYNCBUSY.bit.SWRST );


    /* Load Pad Calibration */
    uint32_t pad_transn = ((*((uint32_t*)USB_FUSES_TRANSN_ADDR)) & USB_FUSES_TRANSN_Msk) >> USB_FUSES_TRANSN_Pos;
    uint32_t pad_transp = ((*((uint32_t*)USB_FUSES_TRANSP_ADDR)) & USB_FUSES_TRANSP_Msk) >> USB_FUSES_TRANSP_Pos;
    uint32_t pad_trim = ((*((uint32_t*)USB_FUSES_TRIM_ADDR)) & USB_FUSES_TRIM_Msk) >> USB_FUSES_TRIM_Pos;


    USB->DEVICE.PADCAL.reg = USB_PADCAL_TRANSN( pad_transn ) | USB_PADCAL_TRANSP( pad_transp ) | USB_PADCAL_TRIM( pad_trim );

    USB->DEVICE.DESCADD.reg = (uint32_t)udc_mem;

    USB->DEVICE.CTRLA.reg = USB_CTRLA_MODE_DEVICE | USB_CTRLA_RUNSTDBY;
    USB->DEVICE.CTRLB.reg = USB_DEVICE_CTRLB_SPDCONF_FS;
    USB->DEVICE.CTRLA.reg |= USB_CTRLA_ENABLE;
}


void bootloader( void )
{
    bootloaderStarted();

    configureClock();

    // Check entry to DFU 
    const bool enterDfu = (!userImageValid()
        || hasBootloaderResetMagic());

#ifdef USE_DBL_TAP
    /* a 'double tap' has happened, so run bootloader */
    resetMagic = 0;
#endif

    if( enterDfu )
    {
        dfuStarted();

#ifdef __SAMD51__
        // Disable NVM caches, per errata.
       // NVMCTRL->CTRLA.bit.CACHEDIS0 = true;
       // NVMCTRL->CTRLA.bit.CACHEDIS1 = true;

        /// @todo Clean up! Automatic Page or Quad-word write?
        NVMCTRL->CTRLA.bit.WMODE = NVMCTRL_CTRLA_WMODE_AP_Val;
#endif

        initializeUsb();

        /*
        service USB
        */

        while( USB_Service() );
    }

    // After DFU we will start the user-app
    userAppStarted();

#if 0
    // Rebase the Stack Pointer
    __set_MSP( *(uint32_t*)appFlashPartition.origin);

    // Rebase the vector table base address
    SCB->VTOR = (appFlashPartition.origin + & SCB_VTOR_TBLOFF_Msk);

    /// Jump to application Reset Handler in the application
    const uint32_t currentUserAppResetVector = *userAppResetVector;
    asm( "bx %0" ::"r"(currentUserAppResetVector) );

    //ldr r0, = &SCB->VTOR /* VTOR register */
   // asm("ldr r1, =%0\n" : : "r"(appFlashPartition.origin)); /* origin of user app @todo Configure based on platform/variant etc*/
#endif
}
