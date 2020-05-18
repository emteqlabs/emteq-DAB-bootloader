#ifndef DFUBOOTLOADER_COMMON_H
#define DFUBOOTLOADER_COMMON_H

#include <stdint.h> //< uint32_t
#include <stdbool.h> //< bool, true, false

///@todo Configure via parameter/platform!
static const uint16_t usbVendorId = 0x04D8;
static const uint16_t usbProductId = 0xEC5A;
static const uint32_t userAppAddress = 0x2000; //< 8KiB  /* origin of the application (first address available after the bootloader) */
static const bool userAppCrcEmbed = true; ///@todo Configure via parameter/platform!

///@{ Vector address'
static const uint32_t userAppResetVectorOffset = 0x04; ///< reset vector of application @APP_START_ADDRESS+4

/**@{ When userAppCrcEmbed==true we define some reserved vectors to store data within the image wihtout adding to the size:
* - SAMD11 0x10 - 0x2C
* - SAMD51 0x1C - 0x2C
*/
static const uint32_t userAppLengthEmbedOffset = 0x1C; /* reserved application vector where the application size is stored */
static const uint32_t userAppCrcEmbedOffset = 0x20; /* reserved application vector where the CRC value is stored */
/**@} userAppCrcEmbed==true */

///@} Vector address'

#endif