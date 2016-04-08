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
int8_t xyzValue[3];
//int8_t xyzValue_prev[3];
//uint8_t right = 0;
//uint8_t front = 0;
uint8_t res = 0;
bool appStarted = FALSE;
bool imgShow = FALSE;
//bool moveForwards = FALSE;
bool isIdle = TRUE;

uint16_t timeMotionRight = 0;	// ms
uint16_t timeMotionLeft = 0;	// ms
bool motionRight = FALSE;
bool motionLeft = FALSE;
int8_t xValPrev = 0;

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
		if(res != ERR_OK){	// Schalte LED1 permanent ein, wenn I2C-Fehler auftritt
			LED1_On();
			LED3_On();
			for(;;){}
		}
	}
}



FSM_State currentState = STATE_HSLU;//STATE_HSLU;

void nextState(void){
	if((++currentState) == STATE_NOF_STATES){
		currentState = 0;
	}

	switch(currentState){
		case STATE_HSLU:
			//img = img_HSLU; // HSLU
			break;
		case STATE_MARIO:
			//img = img_MARIO;
			break;
		case STATE_STEP_COUNTER:
			break;
		case STATE_WATER_SPIRIT_LEVEL:
			break;
		case STATE_CAR_ACCELERATION:
			break;
		case STATE_6:
			break;
		case STATE_7:
			break;
		default:
			break;
	}
}





void idleMode(FSM_State state){
	LED1_Put((state+1) & MASK_LED3);
	LED2_Put((state+1) & MASK_LED2);
	LED3_Put((state+1) & MASK_LED1);
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
#define TIME_UNTIL_IDLE (800)	// Zeit bis zu Idle-Mode gewechselt wird

#if 1
	static uint16_t counterRight = 0;	// Zähler für Rechtsbewegung
	static uint16_t counterLeft = 0;	// Zähler für Linksbewegung
	static uint8_t cntrZ = 0;

	if((xyzValue[0] < ACC_LIMIT_RIGHT) & (xValPrev >= ACC_LIMIT_RIGHT)){	// Aktueller Messwert kleiner und vorheriger Messwert grösser/gleich oberer Schwellwert => Flankendetektion
		counterRight = 0;	// Rechtszähler nullen
		motionRight = TRUE;	// Rechtsbewegung wurde erkannt
	} else if(xyzValue[0] < ACC_LIMIT_LEFT){	// Messwert kleiner als unterer Schwellwert
		motionRight = FALSE;	// Rechtszähler anhalten
	}
	if(motionRight){	// Rechtsbewegung aktiv
		counterRight++;	// Rechtszähler inkrementieren
		//if((motionRight == timeMotionRight/2) & (counterRight > 0)){
		if((counterRight*ACCEL_MEAS_FREQ_MS) == ((timeMotionRight - 2*sizeof(img))/2)){	// Aktuelle Rechtslaufzeit == Auslösezeit für Schriftzug
			imgShow = TRUE;	// Schriftzug auslösen										// ev Auslösezeit bei 1/3 Rechtslaufzeit festlegen
			//LED1_On();
		//} else {
			//LED1_Off();
		}
	}

	if((xyzValue[0] > ACC_LIMIT_LEFT) & (xValPrev <= ACC_LIMIT_LEFT)){		// Aktueller Messwert grösser und vorheriger Messwert kleiner/gleich unterer Schwellwert => Flankendetektion
		counterLeft = 0;	// Linkszähler nullen
		motionLeft = TRUE;	// Linksbewegung wurde erkannt
	} else if(xyzValue[0] > ACC_LIMIT_RIGHT){	// Messwert grösser als oberer Schwellwert
		motionLeft = FALSE;		// Linkszähler anhalten
	}
	if(motionLeft){		// Linksbewegung aktiv
		counterLeft++;	// Linkszähler inkrementieren
		//if((motionLeft == timeMotionLeft/2) & (counterLeft > 0)){
		if((counterLeft*ACCEL_MEAS_FREQ_MS) == ((timeMotionLeft - 2*sizeof(img))/2)){	// Aktuelle Linkslaufzeit == Auslösezeit für Schriftzug
			imgShow = TRUE;	// Schriftzug auslösen										// ev Auslösezeit bei 2/3 Linkslaufzeit festlegen
			//LED3_On();
		//} else {
			//LED3_Off();
		}
	}

	xValPrev = xyzValue[0];		// Variable für vorherigen Wert gleich aktuellem Messwert setzen

	if(!motionRight & (counterRight > 0)){	// Zeitmessung rechts fertig
		timeMotionRight = counterRight * (ACCEL_MEAS_FREQ_MS);	// gemessene Zeit in Variable speichern
		counterRight = 0;	// Rechtszähler nullen
	}
	if(!motionLeft & (counterLeft > 0)){	// Zeitmessung links fertig
		timeMotionLeft = counterLeft * (ACCEL_MEAS_FREQ_MS);	//gemessene Zeit in Variable speichern
		counterLeft = 0;	// Linkszähler nullen
	}

	if(((counterRight*ACCEL_MEAS_FREQ_MS)>TIME_UNTIL_IDLE)|((counterLeft*ACCEL_MEAS_FREQ_MS)>TIME_UNTIL_IDLE)){
		idleMode(currentState);
		if(xyzValue[2] >= 100){
			if(++cntrZ == 10){
				nextState();
				idleMode(currentState);
			}
		} else {
			cntrZ = 0;
		}
	}

#else

	if((xyzValue[0]>-5)&(xyzValue[0]<5)){ // Detektion der Position a = 0
	//if((xyzValue[0] > ACC_LIMIT_RIGHT) | (xyzValue[0] < ACC_LIMIT_LEFT)){ // Detektion der Minima und Maxima
		LED1_On();
	} else {
		LED1_Off();
	}
#endif

// neu
//------------------------------------------------------------------------------------------------------------------------------------
// alt

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
}	// handleAcceleration ende


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
			res = LIS2DH12TR_ReadReg(0x29, &xyzValue[0], 1U);
			//res = LIS2DH12TR_ReadReg(0x2b, &xyzValue[1], 1U);
			res = LIS2DH12TR_ReadReg(0x2d, &xyzValue[2], 1U);
			handleAcceleration();
			break;
		case EVNT_CHANGE_STATE:
			//nextState();
			break;
		default:
			break;
	}
}

void showImage(void){
	static uint8_t i = 0;		// Zähler für Indices
	static bool ledsOn = FALSE;	// LEDs am leuchten?
	static bool iSet = FALSE;	// Variable, um zu Beginn einer Linksbewegung die Variable i zu setzen

	if(motionLeft & !iSet){		// Linksbewegung und i noch nicht gesetzt
		i = sizeof(img)-1;		// i auf Arraylänge-1 setzen
		iSet = TRUE;			// i wurde gesetzt
	}

	if(!ledsOn){				// LEDs leuchten noch nicht
		LED1_Put(img[i] & MASK_LED1);	// LED1 maskieren
		LED2_Put(img[i] & MASK_LED2);	// LED2 maskieren
		LED3_Put(img[i] & MASK_LED3);	// LED3 maskieren
		ledsOn = TRUE;					// LEDs leuchten
		if(motionRight){		// Bewegung nach rechts...
			i++;				// i inkrementieren
		} else {				// Bewegung nach links...
			i--;				// i dekrementieren
		}
	} else {			// LEDs leuchten bereits
		LED1_Off();		// LED1 aus
		LED2_Off();		// LED2 aus
		LED3_Off();		// LED3 aus
		ledsOn = FALSE;	// LEDs sind jetzt aus
	}
	if(((i<sizeof(img)) & motionRight) | ((i != 0xff) & motionLeft)){	// i kleiner Arraylänge bei Rechtsbewegung oder i nicht unter 0 gezählt bei Linksbewegung
		imgShow = TRUE;		// Schriftzug weiterhin anzeigen, da noch nicht vollsändig angezeigt
	} else {
		if(ledsOn){
			imgShow = TRUE;	// falls LEDs noch leuchten, aber Schriftzug zu Ende, Funktion nochmals ausführen bei nächstem Tick, um LEDs auszuschalten
		} else {			// falls LEDs jetzt aus sind...
			imgShow = FALSE;	// nichts mehr anzeigen
			i = 0;				// i nullen
			iSet = FALSE;		// iSet zurücksetzen
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
		idleMode(currentState);
		//i = 0;
		appStarted = TRUE;
		return;
	}
	i++;
}

void addTick(void){
	static uint16_t i = 0;		// Zählervariable

	if(appStarted){				// wenn Applikation läuft...
		if(imgShow){			// wenn Schriftzug angezeigt werden soll...
			setEvent(EVNT_LED_SHOW_IMAGE);	// Event setzen
		}
		if((i % ACCEL_MEAS_FREQ_MS)==0){	// Wenn Beschleunigungsmessung ausgeführt werden soll...
			setEvent(EVNT_ACCELERATION);	// Event setzen
		}
		/*if((i % LED_HEARTBEAT_FREQ_MS)==0){
			setEvent(EVNT_LED_HEARTBEAT);
		}*/
	} else {					// wenn Applikation noch nicht läuft
		setEvent(EVNT_STARTUP);	// Startupevent setzen
	}
	i++;	// Zähler inkrementieren
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


