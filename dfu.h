#ifndef _DFU_H_
#define _DFU_H_

#include <stdint.h>
#include <sam.h>

/// 64 SAMD11 or 512 SAMD51 
#define dfu_blockSize 64U
//NVMCTRL_PAGE_SIZE
/// @todo Blocks the size of a page? NVMCTRL_PAGE_SIZE; ///< 64 on SAMD11 and 512 on SAMD51

#define dfu_endpointSize (dfu_blockSize == 64U  ? 0x3U :  dfu_blockSize == 512U ? 0x6 : 0x0 )
/// @todo Use a pipe of size Page? 

/// @see https://usb.org/sites/default/files/DFU_1.1.pdf

/** @{ USB Class */
#define DFU_INTERFACE_CLASS 0xFE
#define DFU_INTERFACE_SUBCLASS 0x01
#define DFU_INTERFACE_PROTOCOL 0x02
/** @} */

/** "DFU class-specific requests are employed to accomplish the upgrade operations" Table 3.2 DFU Class-Specific Request Values
*/
enum DfuRequestValues
{
      DFU_DETACH = 0x00 ///< DFU-RT only
    , DFU_DNLOAD = 0x01
    , DFU_UPLOAD = 0x02
    , DFU_GETSTATUS = 0x03 ///< Optional for DFU-RT
    , DFU_CLRSTATUS = 0x04
    , DFU_GETSTATE = 0x05 ///< Optional for DFU-RT
    , DFU_ABORT = 0x06
};

/**  State that the device is in
*/
enum DfuState
{
    /** Device is running its normal application.
    */
    appIDLE = 0,

    /** Device is running its normal application, has received the  DFU_DETACH request,
    * and is waiting for a USB reset
    */
    appDETACH = 1,

    /** Device is operating in the DFU mode and is waiting for requests.
    */
    dfuIDLE = 2,

    /** Device has received a block and is waiting for the host to solicit the status via DFU_GETSTATUS.
    */
    dfuDNLOAD_SYNC = 3,

    /** Device is programming a control-write block into its nonvolatile memories.
    */
    dfuDNBUSY = 4,

    /** Device is processing a download operation. Expecting DFU_DNLOAD requests.
    */
    dfuDNLOAD_IDLE = 5,

    /** Device has received the final block of firmware from the host
    * and is waiting for receipt of DFU_GETSTATUS to begin the
    * Manifestation phase; or device has completed the
    * Manifestation phase and is waiting for receipt of
    * DFU_GETSTATUS. (Devices that can enter this state after
    *     the Manifestation phase set bmAttributes bit
    *     bitManifestationTolerant to 1.)
    */
    dfuMANIFEST_SYNC = 6,

    /** Device is in the Manifestation phase. (Not all devices will be able to respond to DFU_GETSTATUS when in this state.)
    */
    dfuMANIFEST = 7,

    /** Device has programmed its memories and is waiting for a USB reset or a power on reset. 
    * (Devices that must enter this state clear bitManifestationTolerant to 0.)
    */
    dfuMANIFEST_WAIT_RESET = 8,

    /** The device is processing an upload operation. Expecting
    * DFU_UPLOAD requests. 
    */
    dfuUPLOAD_IDLE = 9,

    /** An error has occurred. Awaiting the DFU_CLRSTATUS request.
    */
    dfuERROR = 10
};

/** "status resulting from the execution of the most recent request" 6.1.2 DFU_GETSTATUS Request 
*/
enum DfuStatus
{
    OK = 0x00, ///< No error condition is present.
    errTARGET = 0x01, ///< File is not targeted for use by this device.
    errFILE = 0x02, ///< File is for this device but fails some vendor - specific verification test.
    errWRITE = 0x03, ///< Device is unable to write memory.
    errERASE = 0x04, ///< Memory erase function failed.
    errCHECK_ERASED = 0x05, ///< Memory erase check failed.
    errPROG = 0x06, ///< Program memory function failed.
    errVERIFY = 0x07, ///< Programmed memory failed verification.
    errADDRESS = 0x08, ///< Cannot program memory due to received address that is out of range.
    errNOTDONE = 0x09, ///< Received DFU_DNLOAD with wLength = 0, but device does not think it has all of the data yet.
    errFIRMWARE = 0x0A, ///< Device’s firmware is corrupt. It cannot return to run - time (non - DFU) operations.
    errVENDOR = 0x0B, ///< iString indicates a vendor - specific error.
    errUSBR = 0x0C, ///< Device detected unexpected USB reset signaling.
    errPOR = 0x0D, ///< Device detected unexpected power on reset.
    errUNKNOWN = 0x0E, ///< Something went wrong, but the device does not know what it was.
    errSTALLEDPKT = 0x0F, ///<  Device stalled an unexpected request.
};

enum DfuAttributes
{
      USB_DFU_ATTR_CAN_DNLOAD = 1 << 0
    , USB_DFU_ATTR_CAN_UPLOAD = 1 << 1
    , USB_DFU_ATTR_MANIFESTATION_TOLERANT = 1 << 2
    , USB_DFU_ATTR_WILL_DETACH = 1 << 3
};

typedef struct PACK
{
    uint8_t bStatus;

    /** Minimum time, in milliseconds, that the host
    * should wait before sending a subsequent
    * DFU_GETSTATUS request. 
    */
    uint8_t bwPollTimeout[3];

    /** An indication of the state that the device is going to
    * enter immediately following transmission of this
    * response. (By the time the host receives this
    * information, this is the current state of the device.) 
    */
    uint8_t bState;

    /** Index of status description in string table
    */
    uint8_t iString;
} dfu_getstatus_t;

#endif