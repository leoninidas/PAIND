/*
 * Application.c
 *
 *  Created on: 28.03.2016
 *      Author: Mario
 */

#include "Application.h"
#include "LIS2DH12TR.h"
#include "LED1.h"
#include "LED2.h"
#include "LED3.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define ACCEL_MEAS_FREQ_MS	(1)
#define PIXEL_TIME_MS		(1)
#define LED_HEARTBEAT_FREQ_MS	(300)
#define MASK_LED1	(0x04)
#define MASK_LED2	(0x02)
#define MASK_LED3	(0x01)

static uint8_t events[EVNT_NOF_EVENTS];
//uint8_t* img;
uint8_t img[] = {7,2,7,0,7,7,7,0,7,4,4,0,7,4,7}; // HSLU
//uint8_t img[] = {7,3,7,0,7,3,7,0,7,3,5,0,7,0,7,5,7}; // MARIO
//int8_t xAxisValue = 0;
//int8_t zAxisValue = 0;
//int8_t prevXAxisValue = 0;
//int8_t prevZAxisValue = 0;
int8_t xyzValue[3];
//int8_t xyzValue_prev[3];
//uint8_t right = 0;
//uint8_t front = 0;
uint8_t res = 0;
bool appStarted = FALSE;
//bool newValuesAviable = FALSE;
bool imgShow = FALSE;
//bool swipeRight = FALSE;
//bool moveForwards = FALSE;
bool isIdle = TRUE;

uint16_t timeMotionRight = 0;	// ms
uint16_t timeMotionLeft = 0;	// ms
bool motionRight = FALSE;
bool motionLeft = FALSE;
//todo: swipeDetection, show on/off

void startApplication(void){
	intiApplication();

	uint8_t i = 0;
	for(;;){
		for (EventFlags event = 0; event < EVNT_NOF_EVENTS; event++){
			if(events[event]){
				handleEvent(event);
				clearEvent(event);
			}
		}
		//handleAcceleration();
		if(res != ERR_OK){
			LED1_On();
			for(;;){}
		}
	}
}



FSM_State currentState = STATE_MARIO;//STATE_HSLU;

void nextState(void){
	if((++currentState) == STATE_NOF_STATES){
		currentState = 0;
	}

	switch(currentState){
		case STATE_HSLU:
			//img = &img_HSLU;
			break;
		case STATE_MARIO:
			//img = &img_MARIO;
			break;/*
		case STATE_STEP_COUNTER:
			break;
		case STATE_WATER_SPIRIT_LEVEL:
			break;
		case STATE_CAR_ACCELERATION:
			break;*/
		default:
			break;
	}
}



//int16_t xyzIntegrator[] = {0,0,0};
bool countRight = FALSE;
bool countLeft = FALSE;
int8_t xValPrev = 0;


void idleMode(FSM_State state){
	LED1_Put((state+1) & MASK_LED1);
	LED2_Put((state+1) & MASK_LED2);
	LED3_Put((state+1) & MASK_LED3);
}


/*
 * Während einer genügend schnellen Wischbewegung wird die Zeit zwischen der Überschreitung des oberen und unteren Schwellwertes gemessen.
 * Daraus wird die Auslösezeit für den Schriftzug berechnet und mit der aktuellen Zeit verglichen. Sind die beiden Werte gleich, wird die Darstellung
 * des Schriftzuges gestartet.
 * Für die Berechnung der Auslösezeit, wird die Zeit für eine vollständige Bewegung der vorherigen Messung verwendet.
 */
void handleAcceleration(void){

#define ACC_LIMIT_RIGHT (126)	// Oberer Schwellwert für Messung
#define ACC_LIMIT_LEFT (-126)	// Unterer Schwellwert für Messung

//#if 1
	static uint16_t counterRight = 0;	// Zähler für Rechtsbewegung
	static uint16_t counterLeft = 0;	// Zähler für Linksbewegung

	if((xyzValue[0] < ACC_LIMIT_RIGHT) & (xValPrev >= ACC_LIMIT_RIGHT)){	// Aktueller Messwert kleiner und vorheriger Messwert grösser/gleich oberer Schwellwert => Flankendetektion
		counterRight = 0;	// Rechtszähler nullen
		countRight = TRUE;	// Rechtsbewegung wurde erkannt
	} else if(xyzValue[0] < ACC_LIMIT_LEFT){	// Messwert kleiner als unterer Schwellwert
		countRight = FALSE;		// Rechtszähler anhalten
	}
	if(countRight){		// Rechtsbewegung aktiv
		counterRight++;	// Rechtszähler inkrementieren
		//if((counterRight == timeMotionRight/2) & (counterRight > 0)){
		if((counterRight*ACCEL_MEAS_FREQ_MS) == ((timeMotionRight - 2*sizeof(img))/2)){	// Aktuelle Rechtslaufzeit == Auslösezeit für Schriftzug
			// /3, weil Mitte Zeit != Mitte Weg => 1/3 der Zeit, ev division in der klammer schon machen bei timeMotionRight/3
			imgShow = TRUE;	// Schriftzug auslösen
			//LED1_On();
		//} else {
			//LED1_Off();
		}
	}

	if((xyzValue[0] > ACC_LIMIT_LEFT) & (xValPrev <= ACC_LIMIT_LEFT)){
		counterLeft = 0;
		countLeft = TRUE;
	} else if(xyzValue[0] > ACC_LIMIT_RIGHT){
		countLeft = FALSE;
	}
	if(countLeft){
		counterLeft++;
		//if((counterLeft == timeMotionLeft/2) & (counterLeft > 0)){
		if((counterLeft*ACCEL_MEAS_FREQ_MS) == ((timeMotionLeft - 2*sizeof(img))/2)){
			imgShow = TRUE;
			//LED3_On();
		//} else {
			//LED3_Off();
		}
	}

	xValPrev = xyzValue[0];
	motionRight = countRight;
	motionLeft = countLeft;

	if(!countRight & (counterRight > 0)){
		timeMotionRight = counterRight * (ACCEL_MEAS_FREQ_MS);
		counterRight = 0;
	}
	if(!countLeft & (counterLeft > 0)){
		timeMotionLeft = counterLeft * (ACCEL_MEAS_FREQ_MS);
		counterLeft = 0;
	}
/*#else

	//if((xyzValue[0]>-5)&(xyzValue[0]<5)){ // Detektion der Position a = 0
	//if((xyzValue[0] > ACC_LIMIT_RIGHT) | (xyzValue[0] < ACC_LIMIT_LEFT)){ // Detektion der Minima und Maxima
		LED1_On();
	} else {
		LED1_Off();
	}
#endif*/









	/*
	static uint8_t idleCnt = 0;
	if(abs(xyzValue[0] - xValPrev) < 10){
		idleCnt++;
		if(idleCnt == 5){
			idleMode(currentState);
			idleCnt = 0;
		}

	}*/


	// check z-axis
	/*if((xyzValue[2] < xyzValue_prev[2]) & (xyzValue[2] < -20)){
		front++;
		if(front == 10){
			//setEvent(EVNT_CHANGE_STATE);
			front = 0;
		}
		//changeState();
	}*/




	//xyzIntegrator[0] += xyzValue[0];

	//if(xyzIntegrator[0] > )




	// x-axis
	//static uint8_t ix = 0;
	//if(!imgShow){
		/*if((++ix)<=4){
			xyzIntegrator[0] += xyzValue[0];
		} else {
			if(xyzIntegrator[0] > 112){
				swipeRight = TRUE;
				imgShow = TRUE;
				ix = 0;
			}
		}*/
	//}

	/*
	// Darstellung von img mittels Schwellwert
	if((xyzValue[0] < xyzValue_prev[0]) & (xyzValue[0] > 25)){ //todo: mit weniger xyz zugriffen
		right++;
		if(right == 3){
			swipeRight = TRUE;
			right = 0;
		}
		//if((xAxisValue > -20)&(xAxisValue < 20) & swipeRight){
		if((xyzValue[0] > 40) & swipeRight){
			imgShow = TRUE;
		}
	}
	xyzValue_prev[0] = xyzValue[0];
	xyzValue_prev[1] = xyzValue[1];
	xyzValue_prev[2] = xyzValue[3];
	*/
}


void handleEvent(EventFlags event){
	switch(event){
		case EVNT_STARTUP:
			LEDStartUp();
			break;
		case EVNT_LED_HEARTBEAT:
			//LEDHartbeat();
			break;
		case EVNT_LED_SHOW_IMAGE:
			showImage();
			break;
		case EVNT_ACCELERATION:
			//getAccelValue(&xAxisValue);
			res = LIS2DH12TR_ReadReg(0x29, &xyzValue[0], 1U);
			//res = LIS2DH12TR_ReadReg(0x2b, &xyzValue[1], 1U);
			//res = LIS2DH12TR_ReadReg(0x2d, &xyzValue[2], 1U);
			handleAcceleration();
			//motionLeft = TRUE;
			//imgShow = TRUE;
			break;
		case EVNT_CHANGE_STATE:
			//nextState();
			break;
		default:
			break;
	}
}

void showImage(void){
	static uint8_t i = 0;
	static bool ledsOn = FALSE;
	static bool iSet = FALSE;

	if(motionLeft & !iSet){
		i = sizeof(img);
		i--;
		iSet = TRUE;
	}

	if(!ledsOn){
		LED1_Put(img[i] & MASK_LED1);
		LED2_Put(img[i] & MASK_LED2);
		LED3_Put(img[i] & MASK_LED3);
		ledsOn = TRUE;
		if(motionRight){
			i++;
		} else {
			i--;
		}
	} else {
		LED1_Off();
		LED2_Off();
		LED3_Off();
		ledsOn = FALSE;
	}
	if(((i<sizeof(img)) & motionRight) | ((i != 0xff) & motionLeft)){
		imgShow = TRUE;
	} else {
		if(ledsOn){
			imgShow = TRUE;
		} else {
			//swipeRight = FALSE;
			imgShow = FALSE;
			i = 0;
			iSet = FALSE;
		}
	}
}

void LEDHartbeat(void){
	LED1_Neg();
}

void LEDStartUp(void){
	static uint16_t i = 0;

	if(i==0){
		LED1_On();
	} else if(i==333){
		LED2_On();
	} else if(i==666){
		LED3_On();
	} else if(i==999){
		LED1_Off();
		LED2_Off();
		LED3_Off();
	} else if(i>1332){
		appStarted = TRUE;
		i = 0;
		return;
	}
	i++;
}

void addTick(void){
	static uint16_t i = 0;

	if(appStarted){
		if(imgShow){
			setEvent(EVNT_LED_SHOW_IMAGE);
		}
		if((i % ACCEL_MEAS_FREQ_MS)==0){
			setEvent(EVNT_ACCELERATION);
		}
		/*if((i % LED_HEARTBEAT_FREQ_MS)==0){
			setEvent(EVNT_LED_HEARTBEAT);
		}*/
	} else {
		setEvent(EVNT_STARTUP);
	}
	i++;
}

void setEvent(EventFlags event){
	events[event] = 1;
}

void clearEvent(EventFlags event){
	events[event] = 0;
}

void intiApplication(void){
	// Initialisations
	LIS2DH12TR_init();
	LED1_Off();
	LED2_Off();
	LED3_Off();

	// set events
	setEvent(EVNT_STARTUP);
	//setEvent(EVNT_LED_HEARTBEAT);

	// default Values
	xyzValue[0] = 0;
	xyzValue[1] = 0;
	xyzValue[2] = 0;

	/*
	xyzValue_prev[0] = 0;
	xyzValue_prev[1] = 0;
	xyzValue_prev[2] = 0;
	*/
}


