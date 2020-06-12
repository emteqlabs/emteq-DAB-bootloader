/*
modified from original (which can be found at https://github.com/ataradov/mcu-starter-projects):
+ formal stack declaration (rather than just use the end of RAM)
+ entry point of "Reset_Handler" instead of "irq_handler_reset"
+ used Rowley convention of section names (i.e. "data_load" instead of "etext")
+ BOOT_FLASH size reduced to 1kByte (since bootloader must fit in this)
*/

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

 
OUTPUT_FORMAT("elf32-littlearm", "elf32-littlearm", "elf32-littlearm")
OUTPUT_ARCH(arm)
SEARCH_DIR(.)

__BOOTPROTECT_SIZE__ = DEFINED(__BOOTPROTECT_SIZE__) ? __BOOTPROTECT_SIZE__ : 8K;

__origin_FLASH = 0x0;
__origin_RAM = 0x20000000;
__origin_BackupRAM = 0x47000000;
__origin_NvmUserPage = 0x00804000;

__length_FLASH = 512K; /** @todo Configure per device */
__length_RAM = 192K; /** @todo Configure per device */
__length_BackupRAM = 8K; /** @todo Configure per device */
__length_NvmUserPage = 512B; /** @todo Configure per device */

/** @{ PARTITIONS */
__BOOTLOADER_SIZE__ = DEFINED(__BOOTLOADER_SIZE__) ? __BOOTLOADER_SIZE__ : 4K;
__FACTORYCAL_SIZE__ = DEFINED(__FACTORYCAL_SIZE__) ? __FACTORYCAL_SIZE__ : 256;
__HWDATA_SIZE__ = DEFINED(__HWDATA_SIZE__) ? __HWDATA_SIZE__ : 64;
__APP_SIZE__ = DEFINED(__APP_SIZE__) ? __APP_SIZE__ : __length_FLASH - __BOOTPROTECT_SIZE__; /* Whole storage space after boot-protection */
/** @} */

/** @{ RAM */
__RESERVE_ETBSIZE__ = 0x0000; /* First 32K used by ETB trace */
/** @} */

/* Linker script to configure memory regions.
 * Need modifying for a specific board.
 *   APP_FLASH.ORIGIN: starting address of APP_FLASH
 *   APP_FLASH.LENGTH: length of APP_FLASH
 *   RAM.ORIGIN: starting address of RAM bank 0
 *   RAM.LENGTH: length of RAM bank 0
 */
MEMORY
{
  BOOT_FLASH (r) : ORIGIN = __origin_FLASH, LENGTH = __BOOTLOADER_SIZE__
  FACTORYCAL_FLASH (r) : ORIGIN = __BOOTPROTECT_SIZE__ - __HWDATA_SIZE__ - __FACTORYCAL_SIZE__, LENGTH = __FACTORYCAL_SIZE__
  HWDATA_FLASH (r) : ORIGIN = __BOOTPROTECT_SIZE__ - __HWDATA_SIZE__, LENGTH = __HWDATA_SIZE__
  APP_FLASH (rx) : ORIGIN = __BOOTPROTECT_SIZE__, LENGTH = __APP_SIZE__
  RAM (rw) : ORIGIN = __origin_RAM + __RESERVE_ETBSIZE__, LENGTH = __length_RAM - __RESERVE_ETBSIZE__ /* First 32K used by ETB trace */
}

ASSERT( ORIGIN(FACTORYCAL_FLASH) == 0x1EC0, "Factory Calibration Data address moved" )
ASSERT( ORIGIN(HWDATA_FLASH) == 0x1FC0, "Hardware Data address moved" )
ASSERT( ORIGIN(APP_FLASH) == 0x2000, "App-Image address moved" )

__origin_APP_FLASH = ORIGIN(APP_FLASH);     
__length_APP_FLASH = LENGTH(APP_FLASH);
__origin_BOOT_FLASH = ORIGIN(BOOT_FLASH);   
__length_BOOT_FLASH = LENGTH(BOOT_FLASH);
__origin_HWDATA_FLASH = ORIGIN(HWDATA_FLASH);   
__length_HWDATA_FLASH = LENGTH(HWDATA_FLASH);
__origin_FACTORYCAL_FLASH = ORIGIN(FACTORYCAL_FLASH);   
__length_FACTORYCAL_FLASH = LENGTH(FACTORYCAL_FLASH);

__end_RAM = ORIGIN(RAM) + LENGTH(RAM);

ENTRY(Reset_Handler)

SECTIONS
{
  .text : ALIGN(4)
  {
    FILL(0xff)
    KEEP(*(.vectors))
    *(.text*)
    *(.rodata)
    *(.rodata.*)
    . = ALIGN(4);
  } > BOOT_FLASH

  . = ALIGN(4);
  __data_load_start__ = .;

  .uninit_RESERVED : ALIGN(4)
  {
    KEEP(*(.bss.$RESERVED*))
  } > RAM

  .data : ALIGN(4)
  {
    FILL(0xff)
    __data_start__ = .;
    *(.ramfunc .ramfunc.*);
    *(vtable)
    *(.data*)
    . = ALIGN(4);
    __data_end__ = .;
  } > RAM AT > BOOT_FLASH

  .bss : ALIGN(4)
  {
    __bss_start__ = .;
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    __bss_end__ = .;
    PROVIDE(_end = .);
  } > RAM
  
    
	/* .stack_dummy section doesn't contains any symbols. It is only
	 * used for linker to calculate size of stack sections, and assign
	 * values to stack symbols later */
	.stack_dummy (COPY):
	{
		*(.stack*)
	} > RAM
    
	/* Set stack top to end of RAM, and stack limit move down by
	 * size of stack_dummy section */
	__stack_end__ = __end_RAM - SIZEOF(.resetMagic);
	__stack_start__ = __stack_end__ - SIZEOF(.stack_dummy) - SIZEOF(.resetMagic);
	PROVIDE(__stack = __stack_end__);  

    /** We place resetMagic at the very end of memory i.e. Before the stack
    */
    __RAM_reset_magic__ = __end_RAM - 4;
    .resetMagic __RAM_reset_magic__ (NOLOAD) :
    {
        . = ALIGN(4);
        KEEP(*(.resetMagic)) ;
    } > RAM

	__ram_end__ = __end_RAM;
}

