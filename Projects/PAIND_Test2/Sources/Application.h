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
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

typedef enum EventFlags {
	EVNT_STARTUP,
	EVNT_LED_HEARTBEAT,
	EVNT_LED_SHOW_IMAGE,
	EVNT_ACCELERATION,
	EVNT_CHANGE_STATE,
	EVNT_NOF_EVENTS
} EventFlags;

typedef enum FSM_State {
	STATE_HSLU,
	STATE_MARIO, //STATE_HELLO_WORLD,
	STATE_WATER_SPIRIT_LEVEL,
	//STATE_STEP_COUNTER,
	//STATE_CAR_ACCELERATION,
	//STATE_SPINNING_WHEEL,
	//STATE_7,
	STATE_NOF_STATES
} FSM_State;

typedef struct LED_Pattern {
	uint8_t* image;
	uint8_t size;
} LED_Pattern_t;

void startApplication(void);
void nextState(void);
void getAcceleration(void);
void resetAccelSensor(void);
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
void changeSensorResolutionForWaterSpiritLevel(void);
void changeSensorResolutionForShowingImage(void);




#endif /* SOURCES_APPLICATION_H_ */
