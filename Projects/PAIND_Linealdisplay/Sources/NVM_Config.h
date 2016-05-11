/*
 * NVM_Config.h
 *
 *  Created on: 10.05.2016
 *      Author: Mario
 */

#include "PE_Types.h"

#ifndef SOURCES_NVM_CONFIG_H_
#define SOURCES_NVM_CONFIG_H_

#define NVMC_FLASH_START_ADDR    0x3C00 	/* NVRM_Config, start address of configuration data in flash */
#define NVMC_FLASH_ERASED_UINT8  0xFF
#define NVMC_FLASH_ERASED_UINT16 0xFFFF

#define NVMC_WATER_SPIRIT_LEVEL_OFFSET_DATA_START_ADDR  (NVMC_FLASH_START_ADDR)
#define NVMC_WATER_SPIRIT_LEVEL_OFFSET_DATA_SIZE        (16)
#define NVMC_WATER_SPIRIT_LEVEL_OFFSET_DATA_END_ADDR         (NVMC_WATER_SPIRIT_LEVEL_OFFSET_DATA_START_ADDR + NVMC_WATER_SPIRIT_LEVEL_OFFSET_DATA_SIZE)

/*
#define NVMC_SUMO_DATA_START_ADDR         (NVMC_REFLECTANCE_END_ADDR)
#define NVMC_SUMO_DATA_SIZE               (4) // 4 bytes of data
#define NVMC_SUMO_END_ADDR                (NVMC_SUMO_DATA_START_ADDR+NVMC_SUMO_DATA_SIZE)
*/

uint8_t NVMC_SaveWaterSpiritLevelOffsetData(void *data, uint16_t dataSize);
void *NVMC_GetWaterSpiritLevelOffset(void);

/*
uint8_t NVMC_SaveSumoData(void *data, uint16_t dataSize);
void *NVMC_GetSumoData(void);
*/



#endif /* SOURCES_NVM_CONFIG_H_ */
