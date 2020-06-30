#ifndef DFUBOOTLOADER_COMMON_H
#define DFUBOOTLOADER_COMMON_H

#include <stdint.h> //< uint32_t
#include <stdbool.h> //< bool, true, false

/// @TODO This doesn't meet the recommendation from Microchip for a reset memory semaphore
enum ResetMagic
{
      ResetMagic_Bootloader = 0xf01669ef // Randomly selected, adjusted to have first and last bit set
    , ResetMagic_UserApp = 0xf02669ef
};

///@todo Configure via parameter/platform!
static const uint16_t usbVendorId = 0x04D8;
static const uint16_t usbProductId = 0xEC5A;
static const bool userAppCrcEmbed = true; ///@todo Configure via parameter/platform!

///@{ Vector address'
static const uint32_t userAppStackVectorOffset = 0x00;
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