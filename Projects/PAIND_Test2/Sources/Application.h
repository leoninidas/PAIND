/*
 * Application.h
 *
 *  Created on: 28.03.2016
 *      Author: Mario
 */

#ifndef SOURCES_APPLICATION_H_
#define SOURCES_APPLICATION_H_

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
	/*STATE_STEP_COUNTER,
	STATE_WATER_SPIRIT_LEVEL,
	STATE_CAR_ACCELERATION,*/
	STATE_NOF_STATES
} FSM_State;

void startApplication(void);
void nextState(void);
void idleMode(FSM_State);
void handleAcceleration(void);
void handleEvent(EventFlags);
void showImage(void);
void LEDHartbeat(void);
void LEDStartUp(void);
void addTick(void);
void setEvent(EventFlags);
void clearEvent(EventFlags);
void intiApplication(void);




#endif /* SOURCES_APPLICATION_H_ */
