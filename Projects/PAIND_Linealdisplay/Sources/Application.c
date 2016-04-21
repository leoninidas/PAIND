/*
 * Application.c
 *
 *  Created on: 28.03.2016
 *      Author: Mario
 */

#include "Application.h"


// SCL Clock-Source Prescaler:	010 110 => 201.649kHz
//								000 000 => 1.048576MHz  -> geht auch :-)

#define ACCEL_MEAS_FREQ_MS	(1)
#define PIXEL_TIME_MS		(1)
#define LED_HEARTBEAT_FREQ_MS	(300)
#define MASK_LED1	(0x01)
#define MASK_LED2	(0x02)
#define MASK_LED3	(0x04)
#define MASK_LED4	(0x08)
#define MASK_LED5	(0x10)
#define MASK_LED6	(0x20)
#define MASK_LED7	(0x40)
#define MASK_LED8	(0x80)

#define _A	0x7c,0x12,0x12,0x12,0x7c,0x00
#define _B
#define _C
#define _D	0x7e,0x42,0x42,0x42,0x3c,0x00
#define _E	0x7e,0x4a,0x4a,0x42,0x00
#define _F
#define _G
#define _H	0x7e,0x08,0x08,0x7e,0x00
#define _I	0x7e,0x00
#define _J
#define _K
#define _L	0x7e,0x40,0x40,0x40,0x00
#define _M	0x7e,0x04,0x08,0x04,0x7e,0x00
#define _N
#define _O	0x3c,0x42,0x42,0x42,0x3c,0x00
#define _P
#define _Q
#define _R	0x7e,0x12,0x32,0x4c,0x00
#define _S	0x44,0x4a,0x4a,0x32,0x00
#define _T
#define _U	0x3e,0x40,0x40,0x3e,0x00
#define _V
#define _W	0x3e,0x40,0x30,0x40,0x3e,0x00
#define _X
#define _Y
#define _Z
#define _BLANK	0x00,0x00

#define ACC_LIMIT_RIGHT (126)	// Oberer Schwellwert für Messung
#define ACC_LIMIT_LEFT (-126)	// Unterer Schwellwert für Messung
#define TIME_UNTIL_IDLE (500)	// Zeit bis zu Idle-Mode gewechselt wird

#define Z_TRESHOLD_8G	100
#define Z_TRESHOLD_2G	2047

uint8_t events[EVNT_NOF_EVENTS];
//uint8_t* img;
//uint8_t img[] = {7,2,7,0,7,7,7,0,7,4,4,0,7,4,7}; // default: HSLU
//uint8_t img_HSLU[] = {7,2,7,0,7,7,7,0,7,4,4,0,7,4,7}; // HSLU
//uint8_t img_HSLU[] = {0x7e,0x08,0x08,0x7e,0x00,0x44,0x4a,0x4a,0x32,0x00,0x7e,0x40,0x40,0x40,0x00,0x3e,0x40,0x40,0x3e}; // HSLU
uint8_t img_HSLU[] = {_H,_S,_L,_U};
//uint8_t img_MARIO[] = {0x7e,0x04,0x08,0x04,0x7e,0x00,0x7c,0x12,0x12,0x12,0x00,0x7e,0x12,0x32,0x4c,0x00,0x7e,0x00,0x3c,0x42,0x42,0x42,0x3c}; // MARIO
uint8_t img_MARIO[] = {_M,_A,_R,_I,_O};
//uint8_t img_HELLO_WORLD[] = {0x7e,0x08,0x08,0x7e,0x00,0x7e,0x4a,0x4a,0x42,0x00,0x7e,0x40,0x40,0x40,0x00,0x7e,0x40,0x40,0x40,0x00,0x3c,0x42,0x42,0x42,0x3c,0x00,0x00,0x00,0x3e,0x40,0x30,0x40,0x3e,0x00,0x3c,0x42,0x42,0x42,0x3c,0x00,0x7e,0x12,0x32,0x4c,0x00,0x7e,0x40,0x40,0x40,0x00,0x7e,0x42,0x42,0x42,0x3c}; // HELLO WORLD
//uint8_t img_HELLO_WORLD[] = {_H,_E,_L,_L,_O,_BLANK,_W,_O,_R,_L,_D};

LED_Pattern_t img;

int8_t xyzValue[3];
int16_t xyzValue_HR[3];
//int8_t xyzValue_prev[3];
uint8_t res = 0;
bool appStarted = FALSE;
bool imgShow = FALSE;
bool isIdle = TRUE;
void (*fp_handleAcceleration)(void);
void (*fp_getAccelValue)(void);

uint16_t timeMotionRight = 0;	// ms
uint16_t timeMotionLeft = 0;	// ms

uint16_t timeMotionRightMax = 0;	// ms
uint16_t timeMotionLeftMax = 0;	// ms


bool motionRight = FALSE;
bool motionLeft = TRUE;
int8_t xValPrev = 0;
FSM_State currentState;

/*
 * Die Event-Flags werden mittels Polling gecheckt. Main-Loop.
 */
void startApplication(void){
	intiApplication();				// Initialisierung der Applikation
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
			showImage();
			break;
		case EVNT_ACCELERATION:
			//getAccelValue_8bit();
			fp_getAccelValue();
			if(res != ERR_OK){
				resetAccelSensor();		// Power-on reset des Beschleunigungssensors
				intiApplication();
				return;
			}
			fp_handleAcceleration();
			break;
		case EVNT_CHANGE_STATE:
			nextState();
			break;
		default:
			res = 1;
			break;
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
			sensorResolution_8g();
			sensorValueResolution_8bit();
			fp_getAccelValue = &getAccelValue_8bit;
			fp_handleAcceleration = &handleAccelerationForImage;
			img.image = img_HSLU;
			img.size = sizeof(img_HSLU);
			break;
		case STATE_MARIO:
			sensorResolution_8g();
			sensorValueResolution_8bit();
			fp_getAccelValue = &getAccelValue_8bit;
			fp_handleAcceleration = &handleAccelerationForImage;
			img.image = img_MARIO;
			img.size = sizeof(img_MARIO);
			break;
		/*case STATE_MARIO:
			sensorResolution_8g();
			fp_handleAcceleration = &handleAccelerationForImage;

			break;*/
		case STATE_WATER_SPIRIT_LEVEL:
			sensorResolution_2g();
			sensorValueResolution_12bit();
			fp_getAccelValue = &getAccelValue_12bit;
			fp_handleAcceleration = &waterSpiritLevel;
			break;
			/*
		case STATE_STEP_COUNTER:
			sensorResolution_8g();
			fp_handleAcceleration = &handleAccelerationForImage;
			break;
		case STATE_CAR_ACCELERATION:
			sensorResolution_8g();
			fp_handleAcceleration = &handleAccelerationForImage;
			break;
		case STATE_SPINNING_WHEEL:
			sensorResolution_8g();
			fp_handleAcceleration = &handleAccelerationForImage;
			break;
		case STATE_7:
			sensorResolution_8g();
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
uint8_t getAccelValue_8bit(void){
	//int8_t tmp;
	res = LIS2DH12TR_ReadReg(0x29, &xyzValue[0], 1U);
	//res = LIS2DH12TR_ReadReg(0x28, &tmp, 1U);
	res = LIS2DH12TR_ReadReg(0x2b, &xyzValue[1], 1U);
	//res = LIS2DH12TR_ReadReg(0x2a, &tmp, 1U);
	res = LIS2DH12TR_ReadReg(0x2d, &xyzValue[2], 1U);
	//res = LIS2DH12TR_ReadReg(0x2c, &tmp, 1U);
	return res;
}

uint8_t getAccelValue_12bit(void){
	uint8_t tmp_L = 0;
	uint8_t tmp_H = 0;
	res = LIS2DH12TR_ReadReg(0x29, &tmp_H, 1U);
	res = LIS2DH12TR_ReadReg(0x28, &tmp_L, 1U);
	xyzValue_HR[0] = ((int16_t)((tmp_H << 8)|tmp_L))/16;

	res = LIS2DH12TR_ReadReg(0x2b, &tmp_H, 1U);
	res = LIS2DH12TR_ReadReg(0x2a, &tmp_L, 1U);
	xyzValue_HR[1] = ((int16_t)((tmp_H << 8)|tmp_L))/16;

	res = LIS2DH12TR_ReadReg(0x2d, &tmp_H, 1U);
	res = LIS2DH12TR_ReadReg(0x2c, &tmp_L, 1U);
	xyzValue_HR[2] = ((int16_t)((tmp_H << 8)|tmp_L))/16;
}

/*
 * Der Beschleunigungssensor wird eine Zeit lang von der Speisung getrennt (power-on reset). Via GPIO-Pin Nr. ...
 * Diese Funktion wird immer dann aufgerufen, wenn der Rückgabewert 'res' bei I2C-Aktionen != ERR_OK ist.
 */
void resetAccelSensor(void){
	CS1_CriticalVariable();
	LED1_On();LED2_On();LED3_On();LED4_On();
	CS1_EnterCritical();	// Es sollen keine Interrupts behandelt werden
	VDD_ACCEL_ClrVal();		// Speisung des Beschleunigungssensors aussschalten
	WAIT1_Waitms(500);		// 500ms warten
	VDD_ACCEL_SetVal();		// Speisung wieder einschalten
	CS1_ExitCritical();		// Interrupts wieder behandeln
	LED1_Off();LED2_Off();LED3_Off();LED4_Off();
}

/*
 * Wird eine genügend schnelle Bewegung in z-Richtung detektiert, wechselt der aktuelle State der FSM.
 */
void checkStateChange(uint16_t treshold, uint8_t nof_ticks, int16_t value){
	static uint8_t cntrZ = 0;
	if(value >= treshold){
		if(++cntrZ == nof_ticks){
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
	LED1_Put((state+1) & MASK_LED8);
	LED2_Put((state+1) & MASK_LED7);
	LED3_Put((state+1) & MASK_LED6);
	LED4_Put((state+1) & MASK_LED5);
	LED5_Put((state+1) & MASK_LED4);
	LED6_Put((state+1) & MASK_LED3);
	LED7_Put((state+1) & MASK_LED2);
	LED8_Put((state+1) & MASK_LED1);
}

#define WATER_SPIRIT_LEVEL_MEASURE_TIME_MS (50)
#define WATER_SPIRIT_LEVEL_OFFSET (0)

/*
 * Implementierung einer einfachen Wasserwaage.
 */
void waterSpiritLevel(void){
	static bool isIdle = TRUE;
	static bool isBlanking = FALSE;
	static uint8_t idleCntr = 0;
	static uint8_t blankCntr = 0;
	static int32_t sumY= 0;
	static uint8_t i = 0;

	if(i < WATER_SPIRIT_LEVEL_MEASURE_TIME_MS){
		sumY += xyzValue_HR[1];
	}

	if(!isIdle & (i == WATER_SPIRIT_LEVEL_MEASURE_TIME_MS)){
		int16_t meanYValue = (sumY / WATER_SPIRIT_LEVEL_MEASURE_TIME_MS) + WATER_SPIRIT_LEVEL_OFFSET;
		LED1_Off();
		LED2_Off();
		LED3_Off();
		LED4_Off();
		LED5_Off();
		LED6_Off();
		LED7_Off();
		LED8_Off();
		if(meanYValue >= 0){
			if(meanYValue < 2){
				LED4_On();
				LED5_On();
			} else if(meanYValue < 5){
				LED3_On();
				LED4_On();
			} else if(meanYValue < 10){
				LED2_On();
				LED3_On();
			} else {
				LED1_On();
				LED2_On();
			}
		} else {
			if(meanYValue > -2){
				LED4_On();
				LED5_On();
			} else if(meanYValue > -5){
				LED5_On();
				LED6_On();
			} else if(meanYValue > -10){
				LED6_On();
				LED7_On();
			} else {
				LED7_On();
				LED8_On();
			}
		}

		i = 0;
		sumY = 0;
	}
	i++;
	if((xyzValue_HR[0] < -1535) | (xyzValue_HR[0] > 1535)){	// X-Wert ist grösser oder kleiner als 100 -> ablegen auf lange Kante (wie eine Wasserwaage)
		if(++idleCntr == 3){//} && !isBlanking){
			isIdle = !isIdle;
			//isBlanking = TRUE;
		}
	} else {
		idleCntr = 0;
		isBlanking = FALSE;
	}
	if(isIdle){
		idleMode(currentState);
	}
	checkStateChange(Z_TRESHOLD_2G, 10, xyzValue_HR[2]);
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
		checkStateChange(Z_TRESHOLD_8G, 10, xyzValue[2]);
	}

	if(!motionRight & (counterRight > 0)){	// Zeitmessung rechts fertig
		timeMotionRight = counterRight * (ACCEL_MEAS_FREQ_MS);	// gemessene Zeit in Variable speichern
		counterRight = 0;	// Rechtszähler nullen
	}
	if(!motionLeft & (counterLeft > 0)){	// Zeitmessung links fertig
		timeMotionLeft = counterLeft * (ACCEL_MEAS_FREQ_MS);	//gemessene Zeit in Variable speichern
		counterLeft = 0;	// Linkszähler nullen
	}

	if(timeMotionLeft>timeMotionLeftMax){
		timeMotionLeftMax = timeMotionLeft;
	}

	if(timeMotionRight>timeMotionRightMax){
		timeMotionRightMax = timeMotionRight;
	}



#else
	#define TRESHOLD 2
	//if((xyzValue[0]>-TRESHOLD)&(xyzValue[0]<TRESHOLD)){ // Detektion der Position a = 0
	if((xyzValue[0] > ACC_LIMIT_RIGHT) | (xyzValue[0] < ACC_LIMIT_LEFT)){ // Detektion der Minima und Maxima
		LED1_On();
	} else {
		LED1_Off();
	}
#endif
}	// handleAccelerationForImage Ende


/*
 * Anzeigen eines Schriftzuges mit den LEDs.
 * Vorwärts und rückwärts.
 */
void showImage(void){
	static uint8_t i = 0;		// Zähler für Indices
	static bool ledsOn = FALSE;	// LEDs am leuchten?
	static bool iSet = FALSE;	// Variable, um zu Beginn einer Linksbewegung die Variable i zu setzen

	if(motionLeft & !iSet){		// Linksbewegung und i noch nicht gesetzt
		i = img.size-1;			// i auf Arraylänge-1 setzen
		iSet = TRUE;			// i wurde gesetzt
	}

	if(!ledsOn){				// LEDs leuchten noch nicht
		LED1_Put(img.image[i] & MASK_LED1);	// LED1 maskieren
		LED2_Put(img.image[i] & MASK_LED2);	// LED2 maskieren
		LED3_Put(img.image[i] & MASK_LED3);	// LED3 maskieren
		LED4_Put(img.image[i] & MASK_LED4);	// LED4 maskieren
		LED5_Put(img.image[i] & MASK_LED5);	// LED5 maskieren
		LED6_Put(img.image[i] & MASK_LED6);	// LED6 maskieren
		LED7_Put(img.image[i] & MASK_LED7);	// LED7 maskieren
		LED8_Put(img.image[i] & MASK_LED8);	// LED8 maskieren

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
		LED4_Off();		// LED4 aus
		LED5_Off();		// LED5 aus
		LED6_Off();		// LED6 aus
		LED7_Off();		// LED7 aus
		LED8_Off();		// LED8 aus
		ledsOn = FALSE;	// LEDs sind jetzt aus
	}
	if(((i<img.size) & motionRight) | ((i != 0xff) & motionLeft)){	// i kleiner Arraylänge bei Rechtsbewegung oder i nicht unter 0 gezählt bei Linksbewegung
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
#define STARTUP_WAIT_TIME_MS (50)
	static uint16_t i = 0;

	switch(i){
		case 0:
			LED8_On();
			break;
		case STARTUP_WAIT_TIME_MS:
			LED7_On();
			break;
		case 2*STARTUP_WAIT_TIME_MS:
			LED6_On();
			break;
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
		case 7*STARTUP_WAIT_TIME_MS+300:
			LED1_Off();
			LED2_Off();
			LED3_Off();
			LED4_Off();
			LED5_Off();
			LED6_Off();
			LED7_Off();
			LED8_Off();
			break;
		case 7*STARTUP_WAIT_TIME_MS+600:
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
	VDD_ACCEL_SetVal();
	WAIT1_Waitus(1000);
	res = LIS2DH12TR_init();
	/*WAIT1_Waitus(1000);
	sensorResolution_8g();
	WAIT1_Waitus(1000);
	sensorValueResolution_8bit();
	*/

	//res = initWaterSpiritLevel();
	currentState = STATE_HSLU;
	LED1_Off();
	LED2_Off();
	LED3_Off();
	LED4_Off();
	LED5_Off();
	LED6_Off();
	LED7_Off();
	LED8_Off();

	// set events
	setEvent(EVNT_STARTUP);
	//setEvent(EVNT_LED_HEARTBEAT);

	fp_handleAcceleration = &handleAccelerationForImage;
	fp_getAccelValue = &getAccelValue_8bit;
	img.image = img_HSLU;
	img.size = sizeof(img_HSLU);//= {img_HSLU, sizeof(img_HSLU)};
	//img = malloc(32);
	//img = img_HSLU;
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
void sensorResolution_2g(void){
	// Beschleunigungssensor mit 2g-Auflösung
	//res = LIS2DH12TR_WriteReg(0x23, 0b00000000);
	uint8_t tmp;
	res = LIS2DH12TR_ReadReg(0x23, &tmp, 1U);
	tmp &= 0b11001111;
	res = LIS2DH12TR_WriteReg(0x23, tmp);
}

/*
 * Sensoreinstellungen für Anzeige eines Schriftzuges mittels LEDs.
 */
void sensorResolution_8g(void){
	// Beschleunigungssensor mit 8g-Auflösung
	//res = LIS2DH12TR_WriteReg(0x23, 0b00100000);
	uint8_t tmp;
	res = LIS2DH12TR_ReadReg(0x23, &tmp, 1U);
	tmp &= 0b11101111;
	tmp |= 0b00100000;
	res = LIS2DH12TR_WriteReg(0x23, tmp);
}

void sensorValueResolution_8bit(void){
	uint8_t tmp;
	res = LIS2DH12TR_ReadReg(0x23, &tmp, 1U);
	tmp &= 0b11110111;
	res = LIS2DH12TR_WriteReg(0x23, tmp);
	res = LIS2DH12TR_ReadReg(0x20, &tmp, 1U);
	tmp &= 0b11101111;
	tmp |= 0b00001000;
	res = LIS2DH12TR_WriteReg(0x20, tmp);

}

void sensorValueResolution_12bit(void){
	uint8_t tmp;
	res = LIS2DH12TR_ReadReg(0x20, &tmp, 1U);
	tmp &= 0b11110111;
	tmp |= 0b00010000;
	res = LIS2DH12TR_WriteReg(0x20, tmp);
	res = LIS2DH12TR_ReadReg(0x23, &tmp, 1U);
	tmp |= 0b00001000;
	res = LIS2DH12TR_WriteReg(0x23, tmp);
}
