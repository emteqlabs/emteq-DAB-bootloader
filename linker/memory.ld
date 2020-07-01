
__BOOTPROTECT_SIZE__ = DEFINED(__BOOTPROTECT_SIZE__) ? __BOOTPROTECT_SIZE__ : 8K;

__origin_FLASH = 0x0;
__origin_RAM = 0x20000000;
__origin_BackupRAM = 0x47000000;
__origin_UserPage = 0x00804000;

__length_FLASH = 512K; /** @todo Configure per device */
__length_RAM = 192K; /** @todo Configure per device */
__length_BackupRAM = 8K; /** @todo Configure per device */
__length_UserPage = 512; /** @todo Configure per device */
__length_UserPageReserved = 32; /** First eight 32-bit words (32 Bytes) are reserved  @todo Configure per device @todo 3-words are unused on SAMD51 */

/** @{ PARTITIONS */
__BOOTLOADER_SIZE__ = DEFINED(__BOOTLOADER_SIZE__) ? __BOOTLOADER_SIZE__ : 6K;
__CALDATA_SIZE__ = DEFINED(__CALDATA_SIZE__) ? __CALDATA_SIZE__ : __length_UserPage - __length_UserPageReserved;
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
  /* - NVM - */
  BOOT_FLASH (rx) : ORIGIN = __origin_FLASH, LENGTH = __BOOTPROTECT_SIZE__ - __HWDATA_SIZE__
  HWDATA_FLASH (rw) : ORIGIN = __BOOTPROTECT_SIZE__ - __HWDATA_SIZE__, LENGTH = __HWDATA_SIZE__
  APP_FLASH (rw) : ORIGIN = __BOOTPROTECT_SIZE__, LENGTH = __APP_SIZE__

  /* - USER page - */
  USERPAGE_FLASH (rw) : ORIGIN = __origin_UserPage, LENGTH = __length_UserPage

  /* - RAM - */
  RAM (rw) : ORIGIN = __origin_RAM + __RESERVE_ETBSIZE__, LENGTH = __length_RAM - __RESERVE_ETBSIZE__ /* First 32K used by ETB trace */
}

__origin_APP_FLASH = ORIGIN(APP_FLASH);     
__length_APP_FLASH = LENGTH(APP_FLASH);
__origin_BOOT_FLASH = ORIGIN(BOOT_FLASH);   
__length_BOOT_FLASH = LENGTH(BOOT_FLASH);
__origin_HWDATA_FLASH = ORIGIN(HWDATA_FLASH);   
__length_HWDATA_FLASH = LENGTH(HWDATA_FLASH);
__origin_USERPAGE_FLASH = ORIGIN(USERPAGE_FLASH);   
__length_USERPAGE_FLASH = LENGTH(USERPAGE_FLASH);
__origin_CALDATA_FLASH = __origin_USERPAGE_FLASH + __length_UserPageReserved;
__length_CALDATA_FLASH = __length_USERPAGE_FLASH - __length_UserPageReserved;

__end_RAM = ORIGIN(RAM) + LENGTH(RAM);