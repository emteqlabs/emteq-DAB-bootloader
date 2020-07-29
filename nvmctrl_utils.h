#pragma once

#include <stdint.h> //< uint32_t
#include <stdbool.h> //< bool, true, false

void nvmctrl_wait_ready();

/*Before erasing the NVM User Page, ensure that the first 32 Bytes are read to a buffer and later written back
* to the same area unless a configuration change is intended
*/
void nvmctrl_erase_userpage();

/** Write of an incomplete quad-word (SAMD1) or row (SAMD11) must be flushed explicitly
*/
void nvmctrl_write_flush();

#if __SAMD11__
void nvmctrl_erase_row( uint32_t addr );
#elif __SAMD51__
void nvmctrl_erase_block( uint32_t dst );
#endif


bool nvmctrl_bootprot_disarm();

bool nvmctrl_bootprot_rearm();
