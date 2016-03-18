/*
 * LIS2DH12TR.c
 *
 *  Created on: 18.03.2016
 *      Author: Mario
 */

#include "LIS2DH12TR.h"
#include "I2C1.h"
#include "LED1.h"
#include "LED2.h"
#include "LED3.h"

#define LIS2DH12TR_CTRL_REG_1	0x20


static LIS2DH12TR_TDataState deviceData;

uint8_t LIS2DH12TR_WriteReg(uint8_t addr, uint8_t val) {
  uint8_t buf[2], res;

  buf[0] = addr;
  buf[1] = val;
  res = I2C1_MasterSendBlock(deviceData.handle, &buf, 2U, LDD_I2C_SEND_STOP); /* Send OutData (3 bytes with address) on the I2C bus and generates not a stop condition to end transmission */
  if (res!=ERR_OK) {
    return ERR_FAILED;
  }
  while (!deviceData.dataTransmittedFlg) {}  /* Wait until date is sent */
  deviceData.dataTransmittedFlg = FALSE;
  return ERR_OK;
}

uint8_t LIS2DH12TR_ReadReg(uint8_t addr, uint8_t *data, short dataSize) {
  uint8_t res;

  /* Send I2C address plus register address to the I2C bus *without* a stop condition */
  res = I2C1_MasterSendBlock(deviceData.handle, &addr, 1U, LDD_I2C_NO_SEND_STOP);
  if (res!=ERR_OK) {
    return ERR_FAILED;
  }
  while (!deviceData.dataTransmittedFlg) {} /* Wait until data is sent */
  deviceData.dataTransmittedFlg = FALSE;

  /* Receive InpData (1 byte) from the I2C bus and generates a stop condition to end transmission */
  res = I2C1_MasterReceiveBlock(deviceData.handle, data, dataSize, LDD_I2C_SEND_STOP);
  if (res!=ERR_OK) {
    return ERR_FAILED;
  }
  while (!deviceData.dataReceivedFlg) {} /* Wait until data is received received */
  deviceData.dataReceivedFlg = FALSE;
  return ERR_OK;
}


static int8_t xyz[3];

void LIS2DH12TR_run(void)
{
	uint8_t res;
	uint8_t myValue;

	LED1_On();
	LED1_Off();
	LED3_On();

	deviceData.handle = I2C1_Init(&deviceData);

	res = LIS2DH12TR_ReadReg(0x0f, &myValue, 1); // nur zum testen

	res = LIS2DH12TR_WriteReg(0x23, 0b00110000);
	res = 0xff;
	res = LIS2DH12TR_WriteReg(0x20, 0x07);


	uint8_t a = xyz[0];
	if (res==ERR_OK)
	{
	    for(;;)
	    {
	      res = LIS2DH12TR_ReadReg(0x29, (uint8_t*)&xyz, 3); // X_MSB = 0x29
	      LED1_Put(xyz[0]>2);  //>50
	      //printf("%d\n",xyz[0]);
	    }
	}

	I2C1_Deinit(deviceData.handle);
	LED1_Off();
	LED2_Off();
	LED3_Off();
}
