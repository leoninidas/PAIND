/*
 * NVM_Config.c
 *
 *  Created on: 10.05.2016
 *      Author: Mario
 */

#include "NVM_Config.h"
#include "IFsh1.h"


static bool isErased(uint8_t *ptr, int nofBytes) {
  while (nofBytes>0) {
    if (*ptr!=0xFF) {
      return FALSE; /* byte not erased */
    }
    ptr++;
    nofBytes--;
  }
  return TRUE;
}

uint8_t NVMC_SaveWaterSpiritLevelOffsetData(void *data, uint16_t dataSize) {
  if (dataSize>NVMC_WATER_SPIRIT_LEVEL_OFFSET_DATA_SIZE) {
    return ERR_OVERFLOW;
  }
  return IFsh1_SetBlockFlash(data, (IFsh1_TAddress)(NVMC_WATER_SPIRIT_LEVEL_OFFSET_DATA_START_ADDR), dataSize);
}

void *NVMC_GetWaterSpiritLevelOffset(void) {
  if (isErased((uint8_t*)NVMC_WATER_SPIRIT_LEVEL_OFFSET_DATA_START_ADDR, NVMC_WATER_SPIRIT_LEVEL_OFFSET_DATA_SIZE)) {
    return NULL;
  }
  return (void*)NVMC_WATER_SPIRIT_LEVEL_OFFSET_DATA_START_ADDR;
}

/*
uint8_t NVMC_SaveSumoData(void *data, uint16_t dataSize) {
  if (dataSize>NVMC_SUMO_DATA_SIZE) {
    return ERR_OVERFLOW;
  }
  return IFsh1_SetBlockFlash(data, (IFsh1_TAddress)(NVMC_SUMO_DATA_START_ADDR), dataSize);
}

void *NVMC_GetSumoData(void) {
  if (isErased((uint8_t*)NVMC_SUMO_DATA_START_ADDR, NVMC_SUMO_DATA_SIZE)) {
    return NULL;
  }
  return (void*)NVMC_SUMO_DATA_START_ADDR;
}
*/



