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

/*- Types -------------------------------------------------------------------*/
typedef struct
{
    UsbDeviceDescBank  out;
    UsbDeviceDescBank  in;
} udc_mem_t;

/*- Variables ---------------------------------------------------------------*/
static const uint32_t dfu_status_choices[4] =
{ 
  0x00000000, 0x00000002, /* normal */
  0x00000000, 0x00000005, /* dl */
};
static const uint32_t* dfu_status = dfu_status_choices + 0;

static udc_mem_t udc_mem[USB_EPT_NUM];
static uint32_t udc_ctrl_in_buf[FLASH_PAGE_SIZE/sizeof(uint32_t)];
static uint32_t udc_ctrl_out_buf[FLASH_PAGE_SIZE/sizeof(uint32_t)];

static uint32_t usb_config = 0;
static uint32_t dfu_addr;

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
static void udc_control_send(const uint32_t* const data, const uint32_t size)
{
  /* USB peripheral *only* reads valid data from 32-bit aligned RAM locations */
  udc_mem[0].in.ADDR.reg = (uint32_t)data;

  udc_mem[0].in.PCKSIZE.reg = USB_DEVICE_PCKSIZE_BYTE_COUNT(size) | USB_DEVICE_PCKSIZE_MULTI_PACKET_SIZE(0) | USB_DEVICE_PCKSIZE_SIZE(3 /*64 Byte*/);

  USB->DEVICE.DeviceEndpoint[0].EPINTFLAG.reg = USB_DEVICE_EPINTFLAG_TRCPT1;
  USB->DEVICE.DeviceEndpoint[0].EPSTATUSSET.bit.BK1RDY = 1;

  while (0 == USB->DEVICE.DeviceEndpoint[0].EPINTFLAG.bit.TRCPT1);
}

//-----------------------------------------------------------------------------
static void udc_control_send_zlp(void)
{
  udc_control_send(NULL, 0); /* peripheral can't read from NULL address, but size is zero and this value takes less space to compile */
}

static void nvmctrl_wait_ready()
{
#if __SAMD11__
    while (!NVMCTRL->INTFLAG.bit.READY);
#elif __SAMD51__
    while (!NVMCTRL->STATUS.bit.READY);
#else
#error "Unsupported processor class"
#endif
}

#if __SAMD11__
static void nvmctrl_erase_row(uint32_t addr) 
{
    //@todo Necessary?
    //nvmctrl_wait_ready();
    //NVMCTRL->STATUS.reg = NVMCTRL_STATUS_MASK;

    //Execute Erase-Row command
    NVMCTRL->ADDR.reg = addr / 2;
    NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY | NVMCTRL_CTRLA_CMD_ER;
    nvmctrl_wait_ready();
}
#elif __SAMD51__
static void nvmctrl_erase_block(uint32_t dst) 
{
    //@todo Necessary?
    //nvmctrl_wait_ready();

    // Execute "ER" Erase Row
    NVMCTRL->ADDR.reg = dst;
    NVMCTRL->CTRLB.reg = NVMCTRL_CTRLB_CMDEX_KEY | NVMCTRL_CTRLB_CMD_EB;
    nvmctrl_wait_ready();
}
#endif


static void dfuDownloadToNvm()
{
#if __SAMD11__ 
    /** The NVM is organized into rows, where each row contains four pages
        The NVM has a rowerase granularity, while the write granularity is by page. In other words, a single row erase will erase all four pages in
        the row, while four write operations are used to write the complete row.
        */
    if (0 == (dfu_addr% NVMCTRL_ROW_SIZE))
    {
        /// @todo Cache row and only erase if content differs to save wear
        nvmctrl_erase_row(dfu_addr);
    }
    // @note "WP" Write page and "PBC" Page Buffer Clear are not necessary while NVMCTRL->CTRLB.bit.MANW == 0;
#elif __SAMD51__
    /// erase at start of new block
    if (0 == (dfu_addr % NVMCTRL_BLOCK_SIZE))
    {
        /// @todo Cache row and only erase if content differs to save wear
        nvmctrl_erase_block(dfu_addr);
    }
#else
#error "Unsupported processor class"s
#endif

    typedef uint16_t NvmWord;/// @todo Do we need to write in 16-bit chunks or 32bit okay?
    NvmWord* nvm_addr = (NvmWord*)(dfu_addr);
    NvmWord* ram_addr = (NvmWord*)udc_ctrl_out_buf;
    for (unsigned i = 0; i < sizeof(udc_ctrl_out_buf) / sizeof(NvmWord); ++i)
        *nvm_addr++ = *ram_addr++;

    nvmctrl_wait_ready();
}

static void USB_Service(void)
{
  if (USB->DEVICE.INTFLAG.bit.EORST) /* End Of Reset */
  {
    USB->DEVICE.INTFLAG.reg = USB_DEVICE_INTFLAG_EORST;
    USB->DEVICE.DADD.reg = USB_DEVICE_DADD_ADDEN;

    for (int ep = 0; ep < USB_EPT_NUM; ep++)
      USB->DEVICE.DeviceEndpoint[ep].EPCFG.reg = 0;

    USB->DEVICE.DeviceEndpoint[0].EPCFG.reg = USB_DEVICE_EPCFG_EPTYPE0(1 /*CONTROL*/) | USB_DEVICE_EPCFG_EPTYPE1(1 /*CONTROL*/);
    USB->DEVICE.DeviceEndpoint[0].EPSTATUSSET.bit.BK0RDY = 1;
    USB->DEVICE.DeviceEndpoint[0].EPSTATUSCLR.bit.BK1RDY = 1;

    udc_mem[0].in.ADDR.reg = (uint32_t)udc_ctrl_in_buf;
    udc_mem[0].in.PCKSIZE.reg = USB_DEVICE_PCKSIZE_BYTE_COUNT(0) | USB_DEVICE_PCKSIZE_MULTI_PACKET_SIZE(0) | USB_DEVICE_PCKSIZE_SIZE(3 /*64 Byte*/);

    udc_mem[0].out.ADDR.reg = (uint32_t)udc_ctrl_out_buf;
    udc_mem[0].out.PCKSIZE.reg = USB_DEVICE_PCKSIZE_BYTE_COUNT(64) | USB_DEVICE_PCKSIZE_MULTI_PACKET_SIZE(0) | USB_DEVICE_PCKSIZE_SIZE(3 /*64 Byte*/);

    USB->DEVICE.DeviceEndpoint[0].EPSTATUSCLR.bit.BK0RDY = 1;
  }

  if (USB->DEVICE.DeviceEndpoint[0].EPINTFLAG.bit.TRCPT0) /* Transmit Complete 0 */
  {
    if (dfu_addr)
    {
        dfuDownloadToNvm();

        udc_control_send_zlp();
        dfu_addr = 0;
    }

    USB->DEVICE.DeviceEndpoint[0].EPINTFLAG.reg = USB_DEVICE_EPINTFLAG_TRCPT0;
  }

  if (USB->DEVICE.DeviceEndpoint[0].EPINTFLAG.bit.RXSTP) /* Received Setup */
  {
    USB->DEVICE.DeviceEndpoint[0].EPINTFLAG.reg = USB_DEVICE_EPINTFLAG_RXSTP;
    USB->DEVICE.DeviceEndpoint[0].EPSTATUSCLR.bit.BK0RDY = 1;

    usb_request_t *request = (usb_request_t *)udc_ctrl_out_buf;
    const uint8_t type = request->wValue >> 8;
    const uint16_t length = request->wLength;

    /* for these other USB requests, we must examine all fields in bmRequestType */
    if (USB_CMD(OUT, INTERFACE, STANDARD) == request->bmRequestType)
    {
      udc_control_send_zlp();
      return;
    }

    /* for these "simple" USB requests, we can ignore the direction and use only bRequest */
    switch (request->bmRequestType & 0x7F)
    {
    case SIMPLE_USB_CMD(DEVICE, STANDARD):
    case SIMPLE_USB_CMD(INTERFACE, STANDARD):
      switch (request->bRequest)
      {
        case USB_GET_DESCRIPTOR:
          if (USB_DEVICE_DESCRIPTOR == type)
          {
            udc_control_send((uint32_t *)&usb_device_descriptor, length);
          }
          else if (USB_CONFIGURATION_DESCRIPTOR == type)
          {
            udc_control_send((uint32_t *)&usb_configuration_hierarchy, length);
          }
          else
          {
            USB->DEVICE.DeviceEndpoint[0].EPSTATUSSET.bit.STALLRQ1 = 1;
          }
          break;
        case USB_GET_CONFIGURATION:
          udc_control_send(&usb_config, 1);
          break;
        case USB_GET_STATUS:
          udc_control_send(dfu_status_choices + 0, 2); /* a 32-bit aligned zero in RAM is all we need */
          break;
        case USB_SET_FEATURE:
        case USB_CLEAR_FEATURE:
          USB->DEVICE.DeviceEndpoint[0].EPSTATUSSET.bit.STALLRQ1 = 1;
          break;
        case USB_SET_ADDRESS:
          udc_control_send_zlp();
          USB->DEVICE.DADD.reg = USB_DEVICE_DADD_ADDEN | USB_DEVICE_DADD_DADD(request->wValue);
          break;
        case USB_SET_CONFIGURATION:
          usb_config = request->wValue;
          udc_control_send_zlp();
          break;
      }
      break;
    case SIMPLE_USB_CMD(INTERFACE, CLASS):
      switch (request->bRequest)
      {
      case 0x03: // DFU_GETSTATUS
          udc_control_send(&dfu_status[0], 6);
          break;
      case 0x05: // DFU_GETSTATE
          udc_control_send(&dfu_status[1], 1);
          break;
      case 0x01: // DFU_DNLOAD
          dfu_status = dfu_status_choices + 0;
          if (request->wLength)
          {
              dfu_status = dfu_status_choices + 2;
            dfu_addr = 0x400 + request->wValue * 64;
          }
          /* fall through */
        default: // DFU_UPLOAD & others
          /* 0x00 == DFU_DETACH, 0x04 == DFU_CLRSTATUS, 0x06 == DFU_ABORT, and 0x01 == DFU_DNLOAD and 0x02 == DFU_UPLOAD */
          if (!dfu_addr)
            udc_control_send_zlp();
          break;
      }
      break;
    }
  }
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

    NVMCTRL->CTRLB.reg = NVMCTRL_CTRLB_RWS_DUAL;

    SYSCTRL->DFLLCTRL.reg = 0; // See Errata 9905
    while (!SYSCTRL->PCLKSR.bit.DFLLRDY);

    SYSCTRL->DFLLMUL.reg = SYSCTRL_DFLLMUL_MUL(48000);
    SYSCTRL->DFLLVAL.reg = SYSCTRL_DFLLVAL_COARSE( NVM_READ_CAL(NVM_DFLL48M_COARSE_CAL) ) | SYSCTRL_DFLLVAL_FINE( NVM_READ_CAL(NVM_DFLL48M_FINE_CAL) );

    SYSCTRL->DFLLCTRL.reg = SYSCTRL_DFLLCTRL_ENABLE | SYSCTRL_DFLLCTRL_USBCRM | SYSCTRL_DFLLCTRL_MODE | SYSCTRL_DFLLCTRL_BPLCKC | SYSCTRL_DFLLCTRL_CCDIS | SYSCTRL_DFLLCTRL_STABLE;

    while (!SYSCTRL->PCLKSR.bit.DFLLRDY);

    GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(0) | GCLK_GENCTRL_SRC(GCLK_SOURCE_DFLL48M) | GCLK_GENCTRL_RUNSTDBY | GCLK_GENCTRL_GENEN;
    while (GCLK->STATUS.bit.SYNCBUSY);
#else
    /*
    configure oscillator for operation disciplined by external 32k crystal

    This can only be used on PCBs (such as Arduino Zero derived designs) that have these extra components populated.
    It *should* be wholly unnecessary to use this instead of the above USBCRM code.
    However, some problem (Sparkfun?) PCBs experience unreliable USB operation in USBCRM mode.
    */

    NVMCTRL->CTRLB.reg = NVMCTRL_CTRLB_RWS_DUAL;

    SYSCTRL->XOSC32K.reg = SYSCTRL_XOSC32K_STARTUP( 0x6u ) | SYSCTRL_XOSC32K_XTALEN | SYSCTRL_XOSC32K_EN32K;
    SYSCTRL->XOSC32K.reg |= SYSCTRL_XOSC32K_ENABLE;

    while (!(SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_XOSC32KRDY));

    GCLK->GENDIV.reg = GCLK_GENDIV_ID( 1u /* XOSC32K */ );

    GCLK->GENCTRL.reg = GCLK_GENCTRL_ID( 1u /* XOSC32K */ ) | GCLK_GENCTRL_SRC_XOSC32K | GCLK_GENCTRL_GENEN;

    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID( 0u /* DFLL48M */ ) | GCLK_CLKCTRL_GEN_GCLK1 | GCLK_CLKCTRL_CLKEN;

    //  while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);

    SYSCTRL->DFLLCTRL.reg = 0; // See Errata 9905
  //  while (!SYSCTRL->PCLKSR.bit.DFLLRDY);

    SYSCTRL->DFLLMUL.reg = SYSCTRL_DFLLMUL_CSTEP( 31 ) | SYSCTRL_DFLLMUL_FSTEP( 511 ) | SYSCTRL_DFLLMUL_MUL(48000000ul / 32768ul);

    SYSCTRL->DFLLCTRL.reg = SYSCTRL_DFLLCTRL_ENABLE | SYSCTRL_DFLLCTRL_MODE | SYSCTRL_DFLLCTRL_WAITLOCK | SYSCTRL_DFLLCTRL_QLDIS;

    while ( !(SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLLCKC) || !(SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLLCKF) || !(SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLRDY) );

    GCLK->GENDIV.reg = GCLK_GENDIV_ID( 0u /* MAIN */ );

    GCLK->GENCTRL.reg = GCLK_GENCTRL_ID( 0u /* MAIN */ ) | GCLK_GENCTRL_SRC_DFLL48M | GCLK_GENCTRL_IDC | GCLK_GENCTRL_GENEN;

    while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);
#endif
#elif __SAMD51__
    // Automatic wait states.
    NVMCTRL->CTRLA.bit.AUTOWS = 1;

    // Temporarily switch the CPU to the internal 32k oscillator while we reconfigure the DFLL.
    GCLK->GENCTRL[0].reg = GCLK_GENCTRL_SRC(GCLK_GENCTRL_SRC_OSCULP32K) |
        GCLK_GENCTRL_OE |
        GCLK_GENCTRL_GENEN;

    while (GCLK->SYNCBUSY.bit.GENCTRL0);
    OSCCTRL->DFLLCTRLA.reg = 0;// Configure the DFLL for USB clock recovery.

    OSCCTRL->DFLLMUL.reg = OSCCTRL_DFLLMUL_CSTEP(0x1) |
        OSCCTRL_DFLLMUL_FSTEP(0x1) |
        OSCCTRL_DFLLMUL_MUL(0xBB80);

    while (OSCCTRL->DFLLSYNC.bit.DFLLMUL);

    OSCCTRL->DFLLCTRLB.reg = 0;
    while (OSCCTRL->DFLLSYNC.bit.DFLLCTRLB);

    OSCCTRL->DFLLCTRLA.bit.ENABLE = true;
    while (OSCCTRL->DFLLSYNC.bit.ENABLE);

    OSCCTRL->DFLLVAL.reg = OSCCTRL->DFLLVAL.reg;
    while (OSCCTRL->DFLLSYNC.bit.DFLLVAL);

    OSCCTRL->DFLLCTRLB.reg = OSCCTRL_DFLLCTRLB_WAITLOCK |
        OSCCTRL_DFLLCTRLB_CCDIS | OSCCTRL_DFLLCTRLB_USBCRM;

    while (!OSCCTRL->STATUS.bit.DFLLRDY);

    // 5) Switch Generic Clock Generator 0 to DFLL48M. CPU will run at 48MHz.
    GCLK->GENCTRL[0].reg =
        GCLK_GENCTRL_SRC(GCLK_GENCTRL_SRC_DFLL) |
        GCLK_GENCTRL_IDC |
        GCLK_GENCTRL_OE |
        GCLK_GENCTRL_GENEN;

    while (GCLK->SYNCBUSY.bit.GENCTRL0);

    MCLK->CPUDIV.reg = MCLK_CPUDIV_DIV_DIV1;
#else
#error "Unsupported processor class"
#endif
}

static bool userImageCrc()
{
#if __SAMD11__ 
    PAC1->WPCLR.reg = 2; /* clear DSU */
#elif __SAMD51__
    PAC->WRCTRL.reg = PAC_WRCTRL_PERID(ID_DSU) | PAC_WRCTRL_KEY_CLR;
#else
#error "Unsupported processor class"
#endif

    DSU->ADDR.reg = 0x400; /* start CRC check at beginning of user app */
    DSU->LENGTH.reg = *(volatile uint32_t*)0x410; /* use length encoded into unused vector address in user app */

    /* ask DSU to compute CRC */
    DSU->DATA.reg = 0xFFFFFFFF;
    DSU->CTRL.bit.CRC = 1;
    while (!DSU->STATUSA.bit.DONE);

    return !(DSU->DATA.reg);
}

#ifdef USE_DBL_TAP
  extern int __RAM_segment_used_end__;
  static volatile uint32_t *DBL_TAP_PTR = (volatile uint32_t *)(&__RAM_segment_used_end__);
  #define DBL_TAP_MAGIC 0xf02669ef
#endif

static bool hasResetDoubleTap()
{
#if __SAMD11__ 
    const bool isPowerOnReset = (PM->RCAUSE.bit.POR);
#elif __SAMD51__
    const bool isPowerOnReset = (RSTC->RCAUSE.bit.POR);
#else
#error "Unsupported processor class"
#endif

    const bool hasResetMagic = (*DBL_TAP_PTR == DBL_TAP_MAGIC);

    return !isPowerOnReset && hasResetMagic;
}

static void doubleTapResetDelay()
{
    /* postpone boot for a short period of time; if a second reset happens during this window, the "magic" value will remain */
    *DBL_TAP_PTR = DBL_TAP_MAGIC;
    /// @Note Default iOS double tap is .25 seconds so we use this here based on processor frequency
    volatile int wait = F_CPU/4; while (--wait);
    /* however, if execution reaches this point, the window of opportunity has closed and the "magic" disappears  */
    *DBL_TAP_PTR = 0;
}
/* pin PA15 grounded, so run bootloader */
static bool hasGroundedPA15()
{
    /* configure PA15 (bootloader entry pin used by SAM-BA) as input pull-up */
    PORT->Group[0].PINCFG[15].reg = PORT_PINCFG_PULLEN | PORT_PINCFG_INEN;
    PORT->Group[0].OUTSET.reg = (1UL << 15);

    return (!(PORT->Group[0].IN.reg & (1UL << 15)));
}

static bool bootloaderUserEntry()
{
#ifndef USE_DBL_TAP
    return hasGroundedPA15();
#else

    if (hasResetDoubleTap() )
        return true;

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
    PORT->Group[0].PMUX[24>>1].reg = PORT_PMUX_PMUXO(PORT_PMUX_PMUXE_G_Val) | PORT_PMUX_PMUXE(PORT_PMUX_PMUXE_G_Val);

    PM->APBBMASK.reg |= PM_APBBMASK_USB;

    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_ID(USB_GCLK_ID) | GCLK_CLKCTRL_GEN(0);

    USB->DEVICE.CTRLA.reg = USB_CTRLA_SWRST;
    while (USB->DEVICE.SYNCBUSY.bit.SWRST);

    USB->DEVICE.PADCAL.reg = USB_PADCAL_TRANSN( NVM_READ_CAL(NVM_USB_TRANSN) ) | USB_PADCAL_TRANSP( NVM_READ_CAL(NVM_USB_TRANSP) ) | USB_PADCAL_TRIM( NVM_READ_CAL(NVM_USB_TRIM) );

    USB->DEVICE.DESCADD.reg = (uint32_t)udc_mem;

    USB->DEVICE.CTRLA.reg = USB_CTRLA_MODE_DEVICE | USB_CTRLA_RUNSTDBY;
    USB->DEVICE.CTRLB.reg = USB_DEVICE_CTRLB_SPDCONF_FS;
    USB->DEVICE.CTRLA.reg |= USB_CTRLA_ENABLE;
#elif __SAMD51__

    /* Enable USB clock */
    MCLK->APBBMASK.reg |= MCLK_APBBMASK_USB;
    MCLK->AHBMASK.reg |= MCLK_AHBMASK_USB;

    // Set up the USB DP/DN pins
    PORT->Group[0].PINCFG[PIN_PA24H_USB_DM].bit.PMUXEN = 1;
    PORT->Group[0].PMUX[PIN_PA24H_USB_DM / 2].reg &= ~(0xF << (4 * (PIN_PA24H_USB_DM & 0x01u)));
    PORT->Group[0].PMUX[PIN_PA24H_USB_DM / 2].reg |= MUX_PA24H_USB_DM << (4 * (PIN_PA24H_USB_DM & 0x01u));
    PORT->Group[0].PINCFG[PIN_PA25H_USB_DP].bit.PMUXEN = 1;
    PORT->Group[0].PMUX[PIN_PA25H_USB_DP / 2].reg &= ~(0xF << (4 * (PIN_PA25H_USB_DP & 0x01u)));
    PORT->Group[0].PMUX[PIN_PA25H_USB_DP / 2].reg |= MUX_PA25H_USB_DP << (4 * (PIN_PA25H_USB_DP & 0x01u));

    GCLK->PCHCTRL[USB_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK1_Val | (1 << GCLK_PCHCTRL_CHEN_Pos);
#else
#error "Unsupported processor class"
#endif
}

void bootloader(void)
{
    if ( userImageCrc()
      && !bootloaderUserEntry() )
        return;

#ifdef USE_DBL_TAP
  /* a 'double tap' has happened, so run bootloader */
  *DBL_TAP_PTR = 0;
#endif

  configureClock();
  initializeUsb();


  /*
  service USB
  */

  while (1)
    USB_Service();
}
