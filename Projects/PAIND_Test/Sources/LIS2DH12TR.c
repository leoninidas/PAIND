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
  while (!deviceData.dataTransmittedFlg) {
	  __asm("nop");
  } /* Wait until data is sent */
  deviceData.dataTransmittedFlg = FALSE;

  /* Receive InpData (1 byte) from the I2C bus and generates a stop condition to end transmission */
  res = I2C1_MasterReceiveBlock(deviceData.handle, data, dataSize, LDD_I2C_SEND_STOP);
  if (res!=ERR_OK) {
    return ERR_FAILED;
  }
  while (!deviceData.dataReceivedFlg) {
	  __asm("nop");
  } /* Wait until data is received received */
  deviceData.dataReceivedFlg = FALSE;
  return ERR_OK;
}


//static int8_t xyz[3];

static bool run = TRUE;

void LIS2DH12TR_run(void)
{
	//uint8_t data = 0b01010101;
	//uint8_t data1 = 0;
	//uint8_t data_CTRL_REG1 = 0b01010101;
	//uint8_t data_CTRL_REG4 = 0b01010101;
	//uint8_t data_OUT_X_L = 0b01010101;
	int8_t data_OUT_X_H = 0b01010101;
	//uint16_t data_OUT_acc_X = 0;

	uint8_t res = ERR_OK;
	//uint8_t x_value = 0b01010101;
	//xyz[0] = 0;



	while(!run) {}

	LIS2DH12TR_init();



/*
	res = LIS2DH12TR_ReadReg(0x0f, &data, sizeof(data));	// I2C test line 1
	if (res!=ERR_OK) {
		return;
	}

	res = LIS2DH12TR_ReadReg(0x0f, &data1, sizeof(data));	// I2C test line 2
	if (res!=ERR_OK) {
		return;
	}

	res = LIS2DH12TR_ReadReg(0x20, &data_CTRL_REG1, 1);		// Erwartung: 0b00000111 => 0x07
	if (res!=ERR_OK) {
		return;
	}

	res = LIS2DH12TR_ReadReg(0x23, &data_CTRL_REG4, 1);		// Erwartung: 0b0000000	=> 0x00
	if (res!=ERR_OK) {
		return;
	}

	res = LIS2DH12TR_WriteReg(0x23, (0b00010000)|(data_CTRL_REG4));

	res = LIS2DH12TR_ReadReg(0x23, &data_CTRL_REG4, 1);		// Erwartung: 0b00010000	=> 0x10
	if (res!=ERR_OK) {
		return;
	}
	*/

	// Auslesen des Beschleunigungswertes der x-Achse
	//WAIT1_Waitms(1);
	WAIT1_WaitCycles(25);
	for(;;){

		res = LIS2DH12TR_ReadReg(0x29, &data_OUT_X_H, sizeof(data_OUT_X_H));
		while(res!=ERR_OK) {
			LED3_Neg();
			WAIT1_Waitms(100);
		}
/*
		res = LIS2DH12TR_ReadReg(0x28, &data_OUT_X_L, sizeof(data_OUT_X_L));
		if (res!=ERR_OK) {
			break;
		}

		data_OUT_acc_X = (data_OUT_X_H << 8) + data_OUT_X_L;
*/
		if (data_OUT_X_H > 50){
			LED1_On();
			//WAIT1_Waitms(100);
		}else{
			LED1_Off();
		}
		//WAIT1_Waitms(200);
		WAIT1_WaitCycles(25);

	}



	I2C1_Deinit(deviceData.handle);
	LED1_Off();
	LED2_Off();
	LED3_Off();
}

void LIS2DH12TR_init(void)
{
	uint8_t data, res, data_CTRL_REG1;

	deviceData.handle = I2C1_Init(&deviceData);

	res = LIS2DH12TR_ReadReg(0x0f, &data, sizeof(data));	// I2C test line 1
		if (res!=ERR_OK) {
			return;
		} else if(data != 0x33){
			return;
		}

	//WAIT1_WaitCycles(0);

	res = LIS2DH12TR_WriteReg(0x20, 0b01111111);
	res = LIS2DH12TR_WriteReg(0x23, 0b00100000);
	while(res!=ERR_OK) {
		LED3_Neg();
		WAIT1_Waitms(100);
	}

}
