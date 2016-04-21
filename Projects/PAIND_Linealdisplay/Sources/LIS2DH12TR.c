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

//#define LIS2DH12TR_CTRL_REG_1	0x20


static LIS2DH12TR_TDataState deviceData;

void accelTest(void){
	uint8_t res = ERR_OK;
	int8_t dataX = 0;

	deviceData.handle = I2C1_Init(&deviceData);

#if 1
	uint8_t data = 0;
	int8_t i = 2;




	WAIT1_WaitCycles(25);
	res = LIS2DH12TR_ReadReg(0x0f, &data, sizeof(data));
	WAIT1_WaitCycles(25);
	res = LIS2DH12TR_WriteReg(0x20, 0b01111111);
	WAIT1_WaitCycles(25);
	res = LIS2DH12TR_WriteReg(0x23, 0b00100000);

	for(;;){
		WAIT1_WaitCycles(25);
		res = LIS2DH12TR_ReadReg(0x29, &dataX, 1U);

		if(dataX > i){
			LED1_On();
			i = -1;
		} else if(dataX < i){
			LED1_Off();
			i = 1;
		}
	}
#else

#define MASK_LED1	0x04
#define MASK_LED2	0x02
#define MASK_LED3	0x01
#define PIXEL_TIME_MS	1

	uint8_t img[] = {7,2,7,0,7,7,7,0,7,4,4,0,7,4,7};

	WAIT1_WaitCycles(25);
	res = LIS2DH12TR_WriteReg(0x20, 0b01111111);
	WAIT1_WaitCycles(25);
	res = LIS2DH12TR_WriteReg(0x23, 0b00100000);
	WAIT1_WaitCycles(25);

	bool swipeRight = FALSE;
	bool swipeLeft = FALSE;

	uint8_t right = 0;
	uint8_t left = 0;

	uint8_t prevAccelValue = 0;
	uint8_t lengthOfImage = sizeof(img);

	while(1){
		//WAIT1_WaitCycles(25);
		WAIT1_Waitms(1);
		res = LIS2DH12TR_ReadReg(0x29, &dataX, 1U);

		if(dataX < prevAccelValue){
			right++;
			if(right == 3){
				swipeRight = TRUE;
				right = 0;
			}
		} /*else if(dataX > prevAccelValue){
			left++;
			if(left == 3){
				swipeLeft = TRUE;
				left = 0;
			}
		}*/
		prevAccelValue = dataX;

		if ((dataX > 40) & swipeRight){

			WAIT1_Waitms(10);

			for(int i=0; i<lengthOfImage; i++){
				LED1_Put(img[i] & MASK_LED1);
				LED2_Put(img[i] & MASK_LED2);
				LED3_Put(img[i] & MASK_LED3);

				WAIT1_Waitms(PIXEL_TIME_MS);

				LED1_Off();
				LED2_Off();
				LED3_Off();

				WAIT1_Waitms(PIXEL_TIME_MS);
			}
			swipeRight = FALSE;
		} /*else if ((dataX < 40) & swipeLeft){

			WAIT1_Waitms(10);

			for(int i=(lengthOfImage-1); i>=0; i--){
				LED1_Put(img[i] & MASK_LED1);
				LED2_Put(img[i] & MASK_LED2);
				LED3_Put(img[i] & MASK_LED3);

				WAIT1_Waitms(PIXEL_TIME_MS);

				LED1_Off();
				LED2_Off();
				LED3_Off();

				WAIT1_Waitms(PIXEL_TIME_MS);
			}
			swipeLeft = FALSE;
		}*/
		//prevAccelValue = 0;
	}
#endif
}

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
  } /* Wait until data is sent */
  deviceData.dataTransmittedFlg = FALSE;

  /* Receive InpData (1 byte) from the I2C bus and generates a stop condition to end transmission */
  res = I2C1_MasterReceiveBlock(deviceData.handle, data, dataSize, LDD_I2C_SEND_STOP);
  if (res!=ERR_OK) {
    return ERR_FAILED;
  }
  while (!deviceData.dataReceivedFlg) {
  } /* Wait until data is received received */
  deviceData.dataReceivedFlg = FALSE;
  return ERR_OK;
}


//static bool run = TRUE;

//void LIS2DH12TR_run(void){
	//while(!run) {}
	//LIS2DH12TR_init();
	//WAIT1_WaitCycles(25);
	//I2C1_Deinit(deviceData.handle);
//}

int8_t getAccelValue(uint8_t* data_OUT_X_H){
	//int8_t data_OUT_X_H = 0;
	uint8_t res = ERR_OK;
	res = LIS2DH12TR_ReadReg(0x29, &data_OUT_X_H, 1U);
	if(res != ERR_OK){
		return;
	}
}

uint8_t LIS2DH12TR_init(void){
	int8_t data, res;
	deviceData.handle = I2C1_Init(&deviceData);
	WAIT1_WaitCycles(25);
	res = LIS2DH12TR_ReadReg(0x0f, &data, sizeof(data));	// I2C test line 1
		if (res!=ERR_OK) {
			return res;
		} else if(data != 0x33){
			res = 0xff;	// eifach öppis
			return res;
		}
	//return res;

	WAIT1_WaitCycles(25);
	res = LIS2DH12TR_WriteReg(0x20, 0b10001111);
	if(res != ERR_OK){
		return res;
	}
	WAIT1_WaitCycles(25);
	res = LIS2DH12TR_WriteReg(0x23, 0b00100000);
	if(res!=ERR_OK) {
		return res;
	}
	WAIT1_WaitCycles(25);
	return res;

}
