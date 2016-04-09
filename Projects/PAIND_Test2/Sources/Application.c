/*
 * Application.c
 *
 *  Created on: 28.03.2016
 *      Author: Mario
 */

#include "Application.h"


// SCL Clock-Source Prescaler:	010 110 => 201.649kHz
//								000 000 => 1.048576MHz  -> geht auch :-)

#define ACC_LIMIT_RIGHT (126)	// Oberer Schwellwert für Messung
#define ACC_LIMIT_LEFT (-126)	// Unterer Schwellwert für Messung
#define TIME_UNTIL_IDLE (800)	// Zeit bis zu Idle-Mode gewechselt wird
#define ACCEL_MEAS_FREQ_MS	(1)
#define PIXEL_TIME_MS		(1)
#define LED_HEARTBEAT_FREQ_MS	(300)
#define MASK_LED1	(0x04) //(0x01)
#define MASK_LED2	(0x02) //(0x02)
#define MASK_LED3	(0x01) //(0x04)
#define MASK_LED4	(0x08)
#define MASK_LED5	(0x10)
#define MASK_LED6	(0x20)
#define MASK_LED7	(0x40)
#define MASK_LED8	(0x80)

uint8_t events[EVNT_NOF_EVENTS];
//uint8_t** img;
uint8_t img[] = {7,2,7,0,7,7,7,0,7,4,4,0,7,4,7}; // default: HSLU
uint8_t img_HSLU[] = {7,2,7,0,7,7,7,0,7,4,4,0,7,4,7}; // HSLU
//uint8_t img[] = {0x7e,0x08,0x08,0x7e,0x00,0x44,0x4a,0x4a,0x32,0x00,0x7e,0x40,0x40,0x40,0x00,0x3e,0x40,0x40,0x3e}; // HSLU 8LED
uint8_t img_MARIO[] = {7,3,7,0,7,3,7,0,7,3,5,0,7,0,7,5,7}; // MARIO

int8_t xyzValue[3];
//int8_t xyzValue_prev[3];
uint8_t res = 0;
bool appStarted = FALSE;
bool imgShow = FALSE;
bool isIdle = TRUE;
void (*fp_handleAcceleration)(void);

uint16_t timeMotionRight = 0;	// ms
uint16_t timeMotionLeft = 0;	// ms
bool motionRight = FALSE;
bool motionLeft = TRUE;
int8_t xValPrev = 0;
FSM_State currentState;

/*
 * Die Event-Flags werden mittels Polling gecheckt. Main-Loop.
 */
void startApplication(void){
	intiApplication();		// Initialisierung der Applikation
	uint8_t i = 0;
	for(;;){	// Main-Loop
		for (EventFlags event = 0; event < EVNT_NOF_EVENTS; event++){	// Iterieren durch Event-Flags
			if(events[event]){		// Event event ist gesetzt
				handleEvent(event);	// Event behandeln
				clearEvent(event);	// Event-Flag löschen
			}
		}
	}
}

/*
 * Wird ein Event gesetzt, werden hier die entsprechenden Aktionen ausgeführt.
 */
void handleEvent(EventFlags event){
	switch(event){
		case EVNT_STARTUP:
			LEDStartUp();
			break;
		case EVNT_LED_HEARTBEAT:
			//LEDHartbeat();
			break;
		case EVNT_LED_SHOW_IMAGE:
			showImage((uint8_t)currentState);
			break;
		case EVNT_ACCELERATION:
			getAcceleration();
			fp_handleAcceleration();
			break;
		case EVNT_CHANGE_STATE:
			nextState();
			break;
		default:
			res = 1;
			break;
	}
	if(res != ERR_OK){
		resetAccelSensor();		// Power-on reset des Beschleunigungssensors
	}
}

/*
 * Es wurd zum nächsten State gewechselt. Nach dem letzten State kommt wieder der erste.
 */
void nextState(void){
	if((++currentState) == STATE_NOF_STATES){
		currentState = 0;
	}

	switch(currentState){
		case STATE_HSLU:
			changeSensorResolutionForShowingImage();
			fp_handleAcceleration = &handleAccelerationForImage;
			//free(img);
			//img = malloc(sizeof(img_HSLU));
			//img = img_HSLU; // HSLU
			break;
		case STATE_MARIO:
			changeSensorResolutionForShowingImage();
			fp_handleAcceleration = &handleAccelerationForImage;
			//free(img);
			//img = malloc(sizeof(img_MARIO));
			//img = img_MARIO;
			break;
		case STATE_WATER_SPIRIT_LEVEL:
			changeSensorResolutionForWaterSpiritLevel();
			fp_handleAcceleration = &waterSpiritLevel;
			break;
			/*
		case STATE_STEP_COUNTER:
			changeSensorResolutionForShowingImage();
			fp_handleAcceleration = &handleAccelerationForImage;
			break;
		case STATE_CAR_ACCELERATION:
			changeSensorResolutionForShowingImage();
			fp_handleAcceleration = &handleAccelerationForImage;
			break;
		case STATE_SPINNING_WHEEL:
			changeSensorResolutionForShowingImage();
			fp_handleAcceleration = &handleAccelerationForImage;
			break;
		case STATE_7:
			changeSensorResolutionForShowingImage();
			fp_handleAcceleration = &handleAccelerationForImage;
			break;
			*/
		default:
			currentState = STATE_HSLU;
			break;
	}
}

/*
 * Beschleunigungswerte der drei Sensor-Achsen auslesen
 */
void getAcceleration(void){
	res = LIS2DH12TR_ReadReg(0x29, &xyzValue[0], 1U);
	res = LIS2DH12TR_ReadReg(0x2b, &xyzValue[1], 1U);
	res = LIS2DH12TR_ReadReg(0x2d, &xyzValue[2], 1U);
}

/*
 * Der Beschleunigungssensor wird eine Zeit lang von der Speisung getrennt (power-on reset). Via GPIO-Pin Nr. ...
 * Diese Funktion wird immer dann aufgerufen, wenn der Rückgabewert 'res' bei I2C-Aktionen != ERR_OK ist.
 */
void resetAccelSensor(void){
	LED1_On();	// Speisung für Sensor für ...ms ausschalten. Danach neu initialisieren
	LED3_On();
	for(;;){}
}

/*
 * Wird eine genügend schnelle Bewegung in z-Richtung detektiert, wechselt der aktuelle State der FSM.
 */
void checkStateChange(void){
	static uint8_t cntrZ = 0;
	if(xyzValue[2] >= 100){
		if(++cntrZ == 10){
			setEvent(EVNT_CHANGE_STATE);
		}
	} else {
		cntrZ = 0;
	}
}

/*
 * Dar aktuelle State wurd mit den LEDs binär dargestellt.
 */
void idleMode(FSM_State state){				// ev blinkend mit EVNT_LED_HEARTBEAT
	LED1_Put((state+1) & MASK_LED3);
	LED2_Put((state+1) & MASK_LED2);
	LED3_Put((state+1) & MASK_LED1);
	/*
	LED4_Put((state+1) & MASK_LED5);
	LED5_Put((state+1) & MASK_LED4);
	LED6_Put((state+1) & MASK_LED3);
	LED7_Put((state+1) & MASK_LED2);
	LED8_Put((state+1) & MASK_LED1);
	*/
}

#define WATER_SPIRIT_LEVEL_MEASURE_TIME_MS (20)

/*
 * Implementierung einer einfachen Wasserwaage.
 */
void waterSpiritLevel(void){
	static bool isIdle = TRUE;
	static uint8_t idleCntr = 0;
	static int16_t sumY= 0;
	static uint8_t i = 0;

	if(i < WATER_SPIRIT_LEVEL_MEASURE_TIME_MS){
		sumY += xyzValue[1];
	}

	if(!isIdle & (i == WATER_SPIRIT_LEVEL_MEASURE_TIME_MS)){
		int8_t meanYValue = sumY / WATER_SPIRIT_LEVEL_MEASURE_TIME_MS;
		LED1_Off();
		LED2_Off();
		LED3_Off();
		if(meanYValue == 0){//}> -1) & (meanYValue < 1)){
			LED2_On();
		} else if(meanYValue > 0){
			LED3_On();
		} else if(meanYValue < 0){
			LED1_On();
		}
		i = 0;
		sumY = 0;
	}
	i++;
	if((xyzValue[0] < -100) | (xyzValue[0] > 100)){	// X-Wert ist grösser oder kleiner als 100 -> ablegen auf lange Kante (wie eine Wasserwaage)
		if(++idleCntr == 3){
			isIdle = !isIdle;
		}
	} else {
		idleCntr = 0;
	}
	if(isIdle){
		idleMode(currentState);
	}
	checkStateChange();
}




/*
 * Berechnung der Auslösezeit für ein Schriftzug.
 * Während einer genügend schnellen Wischbewegung wird die Zeit zwischen der Überschreitung des oberen und unteren Schwellwertes gemessen.
 * Daraus wird die Auslösezeit für den Schriftzug berechnet und mit der aktuellen Zeit verglichen. Sind die beiden Werte gleich, wird die Darstellung
 * des Schriftzuges gestartet.
 * Für die Berechnung der Auslösezeit, wird die Zeit für eine vollständige Bewegung der vorherigen Messung verwendet.
 */
void handleAccelerationForImage(void){
#if 1
	static uint16_t counterRight = 0;	// Zähler für Rechtsbewegung
	static uint16_t counterLeft = TIME_UNTIL_IDLE+1;	// Zähler für Linksbewegung

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

	if(((counterRight*ACCEL_MEAS_FREQ_MS)>TIME_UNTIL_IDLE)|((counterLeft*ACCEL_MEAS_FREQ_MS)>TIME_UNTIL_IDLE)){
		idleMode(currentState);
		checkStateChange();
	}

	if(!motionRight & (counterRight > 0)){	// Zeitmessung rechts fertig
		timeMotionRight = counterRight * (ACCEL_MEAS_FREQ_MS);	// gemessene Zeit in Variable speichern
		counterRight = 0;	// Rechtszähler nullen
	}
	if(!motionLeft & (counterLeft > 0)){	// Zeitmessung links fertig
		timeMotionLeft = counterLeft * (ACCEL_MEAS_FREQ_MS);	//gemessene Zeit in Variable speichern
		counterLeft = 0;	// Linkszähler nullen
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
}	// handleAccelerationForImage ende


/*
 * Anzeigen eines Schriftzuges mit den LEDs.
 * Vorwärts und rückwärts.
 */
void showImage(uint8_t imgIndex){
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
		/*
		LED4_Put(img[i] & MASK_LED4);	// LED4 maskieren
		LED5_Put(img[i] & MASK_LED5);	// LED5 maskieren
		LED6_Put(img[i] & MASK_LED6);	// LED6 maskieren
		LED7_Put(img[i] & MASK_LED7);	// LED7 maskieren
		LED8_Put(img[i] & MASK_LED8);	// LED8 maskieren
		*/

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

/*
 *
 */
void LEDHartbeat(void){
	LED1_Neg();
}

/*
 * Zeigt an, dass das PCB soeben eingeschaltet wurde.
 */
void LEDStartUp(void){
#define STARTUP_WAIT_TIME_MS (150)
	static uint16_t i = 0;

	switch(i){
		case 0:
			LED1_On();	// LED8_On();
			break;
		case STARTUP_WAIT_TIME_MS:
			LED2_On();	// LED7_On();
			break;
		case 2*STARTUP_WAIT_TIME_MS:
			LED3_On();	// LED6_On();
			break;
			/*
		case 3*STARTUP_WAIT_TIME_MS:
			LED5_On();
			break;
		case 4*STARTUP_WAIT_TIME_MS:
			LED4_On();
			break;
		case 5*STARTUP_WAIT_TIME_MS:
			LED3_On();
			break;
		case 6*STARTUP_WAIT_TIME_MS:
			LED2_On();
			break;
		case 7*STARTUP_WAIT_TIME_MS:
			LED1_On();
			break;
		*/
		case 600://7*STARTUP_WAIT_TIME_MS+300:
			LED1_Off();
			LED2_Off();
			LED3_Off();
			/*
			LED4_Off();
			LED5_Off();
			LED6_Off();
			LED7_Off();
			LED8_Off();
			*/
			break;
		case 900://7*STARTUP_WAIT_TIME_MS+600:
			//idleMode(currentState);
			appStarted = TRUE;
			return;
	}
	i++;
}

/*
 * Wird vom Timer-Interrupt aufgerufen. (f=1kHz)
 * Setzt Events.
 */
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

/*
 * Setzt entsprechendes Event.
 */
void setEvent(EventFlags event){
	events[event] = 1;
}

/*
 * Löscht entsprechendes Event.
 */
void clearEvent(EventFlags event){
	events[event] = 0;
}

/*
 * Initialisierung der Applikation.
 */
void intiApplication(void){
	// Initialisations
	res = LIS2DH12TR_init();
	//res = initWaterSpiritLevel();
	currentState = STATE_HSLU;
	LED1_Off();
	LED2_Off();
	LED3_Off();

	// set events
	setEvent(EVNT_STARTUP);
	//setEvent(EVNT_LED_HEARTBEAT);

	fp_handleAcceleration = &handleAccelerationForImage;
	//img = malloc(sizeof(img_HSLU));
	//img = img_HSLU; // HSLU

	//img = malloc(sizeof(uint8_t*)*2);
	//img[0] = img_HSLU;
	//img[1] = img_MARIO;

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

//#define CTRL_REG4_MASK 0x30

/*
 * Sensoreinstellungen für Wasserwaage.
 */
void changeSensorResolutionForWaterSpiritLevel(void){
	// Beschleunigungssensor mit 2g-Auflösung
	//uint8_t value_CTRL_REG4;
	//res = LIS2DH12TR_ReadReg(0x23, &value_CTRL_REG4, 1U);
	res = LIS2DH12TR_WriteReg(0x23, 0b00000000);
}

/*
 * Sensoreinstellungen für Anzeige eines Schriftzuges mittels LEDs.
 */
void changeSensorResolutionForShowingImage(void){
	// Beschleunigungssensor mit 8g-Auflösung
	res = LIS2DH12TR_WriteReg(0x23, 0b00100000);
}

