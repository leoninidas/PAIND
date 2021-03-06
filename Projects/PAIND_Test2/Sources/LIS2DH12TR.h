/*
 * LIS2DH12TR.h
 *
 *  Created on: 18.03.2016
 *      Author: Mario
 */

#ifndef SOURCES_LIS2DH12TR_H_
#define SOURCES_LIS2DH12TR_H_

#include "PE_Types.h"
#include "PE_LDD.h"

typedef struct {
  volatile bool dataReceivedFlg; /* set to TRUE by the interrupt if we have received data */
  volatile bool dataTransmittedFlg; /* set to TRUE by the interrupt if we have set data */
  LDD_TDeviceData *handle; /* pointer to the device handle */
} LIS2DH12TR_TDataState;


void LIS2DH12TR_run(void);
uint8_t LIS2DH12TR_init(void);
int8_t getAccelValue(uint8_t*);
uint8_t LIS2DH12TR_WriteReg(uint8_t addr, uint8_t val);
uint8_t LIS2DH12TR_ReadReg(uint8_t addr, uint8_t *data, short dataSize);
void accelTest(void);

#endif /* SOURCES_LIS2DH12TR_H_ */
