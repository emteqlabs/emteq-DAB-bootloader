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

void nvmctrl_bootprot_set( const uint32_t size )
{
    const uint32_t new_bootprot = 15 - (size / 8192); // 16k. See "Table 25-10 Boot Loader Size" in datasheet.
    nvmctrl_wait_ready();	

#if 1

    /** TODO:
    uint32_t userPage[__length_USERPAGE_FLASH / sizeof( uint32_t )];
    memcpy( userPage, (uint32_t*)__origin_USERPAGE_FLASH, sizeof( userPage ) );

    // Execute "EP" Erase Page (512Bytes region to 0xFFFFF)
    nvmctrl_wait_ready();
    NVMCTRL->ADDR.reg = (uint32_t)__origin_USERPAGE_FLASH;
    NVMCTRL->CTRLB.reg = NVMCTRL_CTRLB_CMDEX_KEY | NVMCTRL_CTRLB_CMD_EP;
    nvmctrl_wait_ready();
    */
#else
    uint32_t fuses[128];    // 512 bytes (whole user page)	

    std::memcpy( fuses, (uint32_t*)NVM_FUSE_ADDR, sizeof( fuses ) );

    // If it appears the fuses page was erased (all ones), replace fuses with reasonable values.	
    bool repair_fuses = (fuses[0] == 0xffffffff ||
                          fuses[1] == 0xffffffff ||
                          fuses[4] == 0xffffffff);

    if( repair_fuses )
    {
        // These canonical fuse values taken from working Adafruit boards.	
        // BOOTPROT is set to nothing in these values.	
        fuses[0] = 0xFE9A9239;
        fuses[1] = 0xAEECFF80;
        fuses[2] = 0xFFFFFFFF;
        // fuses[3] is for user use, so we don't change it.	
        fuses[4] = 0x00804010;
    }

    uint32_t current_bootprot = (fuses[0] & NVMCTRL_FUSES_BOOTPROT_Msk) >> NVMCTRL_FUSES_BOOTPROT_Pos;

    // logval("repair_fuses", repair_fuses);	
   //  logval("current_bootprot", current_bootprot);	
    // logval("new_bootprot", new_bootprot);	

     // Don't write if nothing will be changed.	
    if( current_bootprot == new_bootprot && !repair_fuses )
    {
        return;
    }

    // Update fuses BOOTPROT value with desired value.	
    fuses[0] = (fuses[0] & ~NVMCTRL_FUSES_BOOTPROT_Msk) | (new_bootprot << NVMCTRL_FUSES_BOOTPROT_Pos);

    // Write the fuses.	
    NVMCTRL->CTRLA.bit.WMODE = NVMCTRL_CTRLA_WMODE_MAN;
    nvmctrl_set_addr( NVM_FUSE_ADDR );  // Set address to user page.	
    nvmctrl_exec_cmd( NVMCTRL_CTRLB_CMD_EP );   // Erase user page.	
    nvmctrl_exec_cmd( NVMCTRL_CTRLB_CMD_PBC );  // Clear page buffer.	
    for( size_t i = 0; i < sizeof( fuses ) / sizeof( uint32_t ); i += 4 )
    {
        // Copy a quadword, one 32-bit word at a time. Writes to page	
        // buffer must be 16 or 32 bits at a time, so we use explicit	
        // word writes	
        NVM_FUSE_ADDR[i + 0] = fuses[i + 0];
        NVM_FUSE_ADDR[i + 1] = fuses[i + 1];
        NVM_FUSE_ADDR[i + 2] = fuses[i + 2];
        NVM_FUSE_ADDR[i + 3] = fuses[i + 3];
        nvmctrl_set_addr( &NVM_FUSE_ADDR[i] ); // Set write address to the current quad word.	
        nvmctrl_exec_cmd( NVMCTRL_CTRLB_CMD_WQW ); // Write quad word.	
    }

    Emteq::resetIntoApp();
#endif
}