/*
 * Application.h
 *
 *  Created on: 28.03.2016
 *      Author: Mario
 */

#ifndef SOURCES_APPLICATION_H_
#define SOURCES_APPLICATION_H_

#include "LIS2DH12TR.h"
#include "LED1.h"
#include "LED2.h"
#include "LED3.h"
#include "NVM_Config.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

typedef enum EventFlags {
	EVNT_STARTUP,
	EVNT_LED_HEARTBEAT,
	EVNT_LED_SHOW_IMAGE,
	EVNT_WORK,
	EVNT_ACCELERATION,
	EVNT_CHANGE_STATE,
	EVNT_CALIB_WATERSPIRITLEVEL_OFFSET,
	EVNT_RESET_ACCEL_SENSOR,
	EVNT_TIMEOUT,
	EVNT_NOF_EVENTS
} EventFlags;

typedef enum FSM_State {
	STATE_HSLU,
	//STATE_HELLO_WORLD,
	STATE_ELEKTRO,
	STATE_LUZERN,
	STATE_WATER_SPIRIT_LEVEL,
	//STATE_STEP_COUNTER,
	//STATE_CAR_ACCELERATION,
	STATE_SPINNING_WHEEL,
	//STATE_7,
	STATE_NOF_STATES
} FSM_State;

typedef struct LED_Pattern {
	uint8_t const* image;
	uint8_t size;
} LED_Pattern_t;

void startApplication(void);
void nextState(void);
void getAccelValue_8bit(void);
void getAccelValue_12bit(void);
void resetAccelSensor(void);
bool checkStateChange(uint16_t, uint8_t, int16_t);
void handleAccelerationForSpinningWheel(void);
void waterSpiritLevel(void);
void idleMode(FSM_State);
void handleAccelerationForImage(void);
void handleEvent(EventFlags);
void showImage(void);
void LEDHartbeat(void);
void LEDStartUp(void);
void addTick(void);
void setEvent(EventFlags);
void clearEvent(EventFlags);
void intiApplication(void);
void deinitApplication(void);
void sensorResolution_2g(void);
void sensorResolution_8g(void);
void sensorResolution_16g(void);
void sensorValueResolution_8bit(void);
void sensorValueResolution_12bit(void);




#endif /* SOURCES_APPLICATION_H_ */
