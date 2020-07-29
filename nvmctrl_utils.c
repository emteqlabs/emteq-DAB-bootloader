#include "nvmctrl_utils.h"

#include <sam.h>
#include <string.h> //< memcpy

#include "partition.h"

/** NVM User page reserved word layout
The Eight 32 - bit words( 32 - bytes )
1) 0 - 31
2) 32 - 64
3) 63 - 95
4) 96 - 127 -- USER - Word
5) 128 - 159
6) 160 - 191
7) 192 - 223 -- USER - Word
8) 224 - 255 ++ USER - Hardware version
*/
typedef struct
{
    uint32_t reserved1[3];
    uint32_t userWord1;
    uint32_t reserved2[2];
    uint32_t userWord2;
    uint32_t hardwareVersion;
} UserPageReserved;

/** @note "9.4 NVM User Page Mapping" The first eight 32-bit words (32 Bytes) of the Non Volatile Memory (NVM) User Page contain calibration data that are
automatically read at device power on.The remaining 480 Bytes can be used for storing custom parameters.
*/
UserPageReserved userPageReserved __attribute__( (section( ".userPageReserved" )) );

void nvmctrl_wait_ready()
{
#if __SAMD11__
    while( !NVMCTRL->INTFLAG.bit.READY );
#elif __SAMD51__
    while( !NVMCTRL->STATUS.bit.READY );
#else
#error "Unsupported processor class"
#endif
}

void nvmctrl_erase_userpage()
{
    uint8_t userPageReservedBuffer[sizeof( userPageReserved )];

    memcpy( userPageReservedBuffer, (void*)&userPageReserved, sizeof( userPageReserved ) );

    // Execute "EP" Erase Page (512Bytes region to 0xFFFFF)
    nvmctrl_wait_ready();
    NVMCTRL->ADDR.reg = (uint32_t)__origin_USERPAGE_FLASH;
    NVMCTRL->CTRLB.reg = NVMCTRL_CTRLB_CMDEX_KEY | NVMCTRL_CTRLB_CMD_EP;
    nvmctrl_wait_ready();

    //Restore the reserved data section contents
    memcpy( (void*)&userPageReserved, userPageReservedBuffer, sizeof( userPageReserved ) );

#if 0 ///< @todo will this be necessary?
    //Commit the last quad-word in page-buffer to ensure the reserved data is commited
    nvmctrl_wait_ready();
    NVMCTRL->CTRLB.reg = NVMCTRL_CTRLB_CMDEX_KEY | NVMCTRL_CTRLB_CMD_WQW;
    nvmctrl_wait_ready();
#endif
}

/** Write of an incomplete quad-word (SAMD1) or row (SAMD11) must be flushed explicitly
*/
void nvmctrl_write_flush()
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
void nvmctrl_erase_row( uint32_t addr )
{
    nvmctrl_wait_ready();

    //Execute Erase-Row command
    // @note A row contains 4 pages
    NVMCTRL->ADDR.reg = addr / 2;
    NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY | NVMCTRL_CTRLA_CMD_ER;
    nvmctrl_wait_ready();
}
#elif __SAMD51__
void nvmctrl_erase_block( uint32_t dst )
{
    nvmctrl_wait_ready();

    // Execute "EB" Erase block (8K region to 0xFFFF)
    NVMCTRL->ADDR.reg = dst;
    NVMCTRL->CTRLB.reg = NVMCTRL_CTRLB_CMDEX_KEY | NVMCTRL_CTRLB_CMD_EB;
    nvmctrl_wait_ready();
}
#endif

bool nvmctrl_bootprot_disarm()
{
    NVMCTRL->CTRLB.reg = NVMCTRL_CTRLB_CMDEX_KEY | NVMCTRL_CTRLB_CMD_SBPDIS; //< Discard boot protection
    nvmctrl_wait_ready();
    return true;
}

bool nvmctrl_bootprot_rearm()
{
    NVMCTRL->CTRLB.reg = NVMCTRL_CTRLB_CMDEX_KEY | NVMCTRL_CTRLB_CMD_CBPDIS; //< Remove discard of boot protection
    nvmctrl_wait_ready();
    return true;
}