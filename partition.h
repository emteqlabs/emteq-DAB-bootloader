#pragma once

#include <stdint.h> //< uint32_t
#include <stdbool.h> //< bool, true, false

extern char __origin_APP_FLASH[], __length_APP_FLASH[]; ///< @note Defined in .ld linker
extern char __origin_BOOT_FLASH[], __length_BOOT_FLASH[]; ///< @note Defined in .ld linker
extern char __origin_USERPAGE_FLASH[], __length_USERPAGE_FLASH[]; ///< @note Defined in .ld linker
extern char __origin_CALDATA_FLASH[], __length_CALDATA_FLASH[]; ///< @note Defined in .ld linker
extern char __origin_HWDATA_FLASH[], __length_HWDATA_FLASH[]; ///< @note Defined in .ld linker
extern char __stack_start__[], __stack_end__[]; ///< @note Defined in .ld linker


typedef enum
{
      Partition_Invalid = -1

    , Partition_App = 0
    , Partition_Bootloader
    , Partition_HardwareData
    , Partition_CalibrationData

    /// Sentinal
    , Partition_COUNT
} PartitionId;

typedef struct
{
    uint32_t origin;
    uint32_t length;
} PartitionAddress;

static const PartitionAddress partition[Partition_COUNT] =
{
     [Partition_App] = { (uint32_t)__origin_APP_FLASH, (uint32_t)__length_APP_FLASH }
   , [Partition_Bootloader] = { (uint32_t)__origin_BOOT_FLASH, (uint32_t)__length_BOOT_FLASH }
   , [Partition_HardwareData] = { (uint32_t)__origin_HWDATA_FLASH, (uint32_t)__length_HWDATA_FLASH }
   , [Partition_CalibrationData] = { (uint32_t)__origin_CALDATA_FLASH, (uint32_t)__length_CALDATA_FLASH }
};


inline bool isPartitionValid( const PartitionId partitionId )
{
    return partitionId > Partition_Invalid && partitionId < Partition_COUNT;
}
