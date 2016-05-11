/*
 * Application.c
 *
 *  Created on: 28.03.2016
 *      Author: Mario
 */

#include "Application.h"


// SCL Clock-Source Prescaler:	010 110 => 201.649kHz
//								000 000 => 1.048576MHz  -> geht auch :-)

#define HAS_TIMEOUT				(0)

#define TICK_FREQ_US			(91)
#define ACCEL_MEAS_FREQ_US		(1000)
//#define PIXEL_TIME_US			(1000)
#define TIME_UNTIL_IDLE_US 		(500000)	// Zeit bis zu Idle-Mode gewechselt wird
#define LED_HEARTBEAT_FREQ_MS	(300)
#define TIMEOUT_S				(10)

#define MASK_LED1	(0x01)
#define MASK_LED2	(0x02)
#define MASK_LED3	(0x04)
#define MASK_LED4	(0x08)
#define MASK_LED5	(0x10)
#define MASK_LED6	(0x20)
#define MASK_LED7	(0x40)
#define MASK_LED8	(0x80)

#define LIS2DH12TR_CTRL_REG1	0x20
#define LIS2DH12TR_CTRL_REG4	0x23

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
#define _K	0x7e,0x18,0x24,0x42,0x00
#define _L	0x7e,0x40,0x40,0x40,0x00
#define _M	0x7e,0x04,0x08,0x04,0x7e,0x00
#define _N	0x7e,0x04,0x08,0x10,0x7e,0x00
#define _O	0x3c,0x42,0x42,0x42,0x3c,0x00
#define _P
#define _Q
#define _R	0x7e,0x12,0x32,0x4c,0x00
#define _S	0x44,0x4a,0x4a,0x32,0x00
#define _T	0x02,0x02,0x7e,0x02,0x02,0x00
#define _U	0x3e,0x40,0x40,0x3e,0x00
#define _V
#define _W	0x3e,0x40,0x30,0x40,0x3e,0x00
#define _X
#define _Y
#define _Z	0x62,0x52,0x4a,0x46,0x00
#define _BLANK	0x00,0x00
#define _DOT	0x40,0x00

#define ACC_LIMIT_RIGHT (126)	// Oberer Schwellwert für Messung
#define ACC_LIMIT_LEFT (-126)	// Unterer Schwellwert für Messung

#define Z_TRESHOLD_16G	50
#define Z_TRESHOLD_8G	100
#define Z_TRESHOLD_2G	2020

uint8_t t = sizeof(uint);

uint8_t events[EVNT_NOF_EVENTS];     //todo: beser hier Pointer definieren und Array im initApplication() inizialisieren uund nich uint_8 verwenden!

uint8_t const img_HSLU[] = {_H,_S,_L,_U};		//todo: gleiches hier
uint8_t const img_HELLO_WORLD[] = {_H,_E,_L,_L,_O,_BLANK,_W,_O,_R,_L,_D};
uint8_t const img_MARIO[] = {_M,_A,_R,_I,_O};
uint8_t const img_ELEKTRO[] = {_E,_L,_E,_K,_T,_R,_O};
uint8_t const img_LUZERN[] = {_L,_U,_Z,_E,_R,_N};

LED_Pattern_t img;

int8_t xyzValue[3];		//todo: und hier
int16_t xyzValue_HR[3];
uint8_t res = 0;		//todo: eigentlich alles so machen
bool appStarted = FALSE;
bool imgShow = FALSE;
bool isIdle = TRUE;

void (*fp_handleAcceleration)(void);
void (*fp_getAccelValue)(void);

uint timeMotionRight = 0;	// ms
uint timeMotionLeft = 0;	// ms

//uint32_t timeMotionRightMax = 0;	// ms
//uint32_t timeMotionLeftMax = 0;	// ms

bool isBlanking = TRUE;

bool motionRight = FALSE;
bool motionLeft = TRUE;
int8_t xValPrev = 0;

// Variablen für die Berechnung des Sensor-Offsets für die Wasserwaage
int32_t sumForWaterSpiritLevelCalibration = 0;
int16_t waterSpiritLevelOffset = 0;
uint8_t nofValuesForWaterSpiritLevelCalibration = 0;


FSM_State currentState;

uint pixelTime = 0;

/*
 * Die Event-Flags werden mittels Polling gecheckt. Main-Loop.
 */
void startApplication(void){
	intiApplication();				// Initialisierung der Applikation
	for(;;){	// Main-Loop
		for (EventFlags event = 0; event < EVNT_NOF_EVENTS; event++){	// Iterieren durch Event-Flags
			if(events[event]){		// Event event ist gesetzt
				clearEvent(event);	// Event-Flag löschen
				handleEvent(event);	// Event behandeln
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
		case EVNT_WORK:
			fp_handleAcceleration();
			break;
		case EVNT_ACCELERATION:
			//getAccelValue_8bit();
			fp_getAccelValue();
			if(res != ERR_OK){
				resetAccelSensor();		// Power-on reset des Beschleunigungssensors
				intiApplication();		//todo: das Resetten läuft noch nicht richtig---- ev ein EVNT hinzufügen, um den Sensor zu resetten
										// res gescheiter ausweretn
			}
			//fp_handleAcceleration();
			break;
		case EVNT_CHANGE_STATE:
			nextState();
			break;
		case EVNT_CALIB_WATERSPIRITLEVEL_OFFSET:
			waterSpiritLevelOffset = sumForWaterSpiritLevelCalibration/nofValuesForWaterSpiritLevelCalibration;
			CS1_CriticalVariable();
			CS1_EnterCritical();
			NVMC_SaveWaterSpiritLevelOffsetData(&waterSpiritLevelOffset, 8*sizeof(waterSpiritLevelOffset));
			CS1_ExitCritical();
			break;
		case EVNT_RESET_ACCEL_SENSOR:
			resetAccelSensor();		// Power-on reset des Beschleunigungssensors
			break;
		case EVNT_TIMEOUT:
			//uint16_t waterSpiritLevelOffset_tmp;
			Self_Supply_ClrVal();	// Selbsthaltung ausschalten
			break;
		default:
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

	void *ptr;
	CS1_CriticalVariable();

	switch(currentState){
		case STATE_HSLU:
			sensorResolution_16g();
			sensorValueResolution_8bit();
			fp_getAccelValue = &getAccelValue_8bit;
			fp_handleAcceleration = &handleAccelerationForImage;
			img.image = img_HSLU;
			img.size = sizeof(img_HSLU);
			break;
		case STATE_ELEKTRO:
			sensorResolution_16g();
			sensorValueResolution_8bit();
			fp_getAccelValue = &getAccelValue_8bit;
			fp_handleAcceleration = &handleAccelerationForImage;
			img.image = img_ELEKTRO;//img_MARIO;
			img.size = sizeof(img_ELEKTRO);
			break;
		case STATE_LUZERN:
			sensorResolution_16g();
			sensorValueResolution_8bit();
			fp_getAccelValue = &getAccelValue_8bit;
			fp_handleAcceleration = &handleAccelerationForImage;
			img.image = img_LUZERN;
			img.size = sizeof(img_LUZERN);
			break;
		case STATE_SPINNING_WHEEL:
			sensorResolution_16g();
			sensorValueResolution_8bit();
			fp_getAccelValue = &getAccelValue_8bit;
			fp_handleAcceleration = &handleAccelerationForSpinningWheel;
			break;
		case STATE_WATER_SPIRIT_LEVEL:
			CS1_EnterCritical();
			ptr = NVMC_GetWaterSpiritLevelOffset();
			CS1_ExitCritical();
			if(ptr != NULL){
				waterSpiritLevelOffset = *((int8_t*)ptr);
			}
			sumForWaterSpiritLevelCalibration = 0;
			nofValuesForWaterSpiritLevelCalibration = 0;
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
		case STATE_7:
			sensorResolution_8g();
			fp_handleAcceleration = &handleAccelerationForImage;
			break;
			*/
		default:
			currentState = STATE_HSLU;
			break;
	}
	idleMode(currentState);		// aktuellen State mit den LEDs anzeigen
	isIdle = TRUE;
	isBlanking = TRUE;			// nach State-Wechsel kurz warten, damit die FSM nicht gleich in den nächseten State springt (vor Allem bei der Wasserwage)
}

/*
 * Beschleunigungswerte der drei Sensor-Achsen auslesen
 */
void getAccelValue_8bit(void){
	res = LIS2DH12TR_ReadReg(0x29, &xyzValue[0], 1U);
	res = LIS2DH12TR_ReadReg(0x2b, &xyzValue[1], 1U);
	res = LIS2DH12TR_ReadReg(0x2d, &xyzValue[2], 1U);
}

void getAccelValue_12bit(void){
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
 * Wird eine genügend schnelle Bewegung in z-Richtung detektiert, wird ein Event gesetzt, um den Modus zu wechseln.
 */
bool checkStateChange(uint16_t treshold, uint8_t nof_ticks, int16_t value){
	static uint8_t cntrZ = 0;
	if(value >= treshold){
		if(++cntrZ == nof_ticks){
			setEvent(EVNT_CHANGE_STATE);
			return TRUE;
		}
	} else {
		cntrZ = 0;
		return FALSE;
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

uint8_t cntr = 0;

void handleAccelerationForSpinningWheel(void){


	LED1_Off();
	LED2_Off();
	LED3_Off();
	LED4_Off();
	LED5_Off();
	LED6_Off();
	LED7_Off();
	LED8_Off();

	if(xyzValue[1] < -64){
		cntr++;
		LED1_Put(cntr==1);
		LED2_Put(cntr==2);
		LED3_Put(cntr==3);
		LED4_Put(cntr==4);
		LED5_Put(cntr==5);
		LED6_Put(cntr==6);
		LED7_Put(cntr==7);
		LED8_Put(cntr==8);

		if(cntr == 8){
			cntr = 0;
		}
	}
	/*
	LED1_Put(xyzValue[1] < -16);
	LED2_Put(xyzValue[1] < -32);
	LED3_Put(xyzValue[1] < -48);
	LED4_Put(xyzValue[1] < -64);
	LED5_Put(xyzValue[1] < -80);
	LED6_Put(xyzValue[1] < -96);
	LED7_Put(xyzValue[1] < -112);
	LED8_Put(xyzValue[1] <= -126);
	*/
	checkStateChange(Z_TRESHOLD_16G, 10, xyzValue[2]);

}


#define WATER_SPIRIT_LEVEL_MEASURE_TIME_MS (500)//*TICK_FREQ_US)
#define WATER_SPIRIT_LEVEL_OFFSET (0)//-20 | 0

/*
 * Implementierung einer einfachen Wasserwaage.
 */
void waterSpiritLevel(void){
	static uint32_t idleCntr = 0;
	static int32_t sumY= 0;
	static int16_t meanYValue = 0;
	static int16_t meanYValuePerv = 0;
	static uint16_t i = 0;


	if(++i < WATER_SPIRIT_LEVEL_MEASURE_TIME_MS){
		sumY += xyzValue_HR[1];
	}

	if(!isIdle){
		if(i == WATER_SPIRIT_LEVEL_MEASURE_TIME_MS){
			meanYValue = (sumY / WATER_SPIRIT_LEVEL_MEASURE_TIME_MS) - waterSpiritLevelOffset;
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

			int16_t meanYValueDiff = meanYValuePerv - meanYValue;
				if((meanYValueDiff > -3) && (meanYValueDiff < 3)){
					sumForWaterSpiritLevelCalibration += meanYValue;
					nofValuesForWaterSpiritLevelCalibration++;
					if(++idleCntr == (2000000/WATER_SPIRIT_LEVEL_MEASURE_TIME_MS)/TICK_FREQ_US){
						isIdle = TRUE;
						idleCntr = 0;
						setEvent(EVNT_CALIB_WATERSPIRITLEVEL_OFFSET);
					}
				} else {
					idleCntr = 0;
					sumForWaterSpiritLevelCalibration = 0;
					nofValuesForWaterSpiritLevelCalibration = 0;
				}
			meanYValuePerv = meanYValue;
			i = 0;
			sumY = 0;

		}
	} else {
		idleMode(currentState);
		if(++idleCntr >= 1000000/TICK_FREQ_US){
			isIdle = FALSE;
			idleCntr = 0;
		}
	}
	if(checkStateChange(Z_TRESHOLD_2G, 10, xyzValue_HR[2])){
		//isIdle = TRUE;
	}
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
	static uint counterRight = 0;	// Zähler für Rechtsbewegung
	static uint counterLeft = (TIME_UNTIL_IDLE_US/TICK_FREQ_US)+1;	// Zähler für Linksbewegung
	static uint8_t i = 0;
	static uint sumR = 0;
	static uint sumL = 0;

	if((xyzValue[0] < ACC_LIMIT_RIGHT) & (xValPrev >= ACC_LIMIT_RIGHT)){	// Aktueller Messwert kleiner und vorheriger Messwert grösser/gleich oberer Schwellwert => Flankendetektion
		counterRight = 0;	// Rechtszähler nullen
		motionRight = TRUE;	// Rechtsbewegung wurde erkannt
	} else if(xyzValue[0] < ACC_LIMIT_LEFT){	// Messwert kleiner als unterer Schwellwert
		motionRight = FALSE;	// Rechtszähler anhalten
	}
	if(motionRight){	// Rechtsbewegung aktiv
		counterRight++;	// Rechtszähler inkrementieren
		if((counterRight == ((timeMotionRight - 2*img.size)/2)) && (timeMotionRight > 2*img.size*pixelTime)){	// Aktuelle Rechtslaufzeit == Auslösezeit für Schriftzug
			imgShow = TRUE;	// Schriftzug auslösen
			//pixelTime = timeMotionRight / (2*img.size);
		}

		/*if(counterRight == timeMotionRight/4){
			pixelTime = (timeMotionRight/2)/(2*img.size);
			imgShow = TRUE;
		}*/
	}

	if((xyzValue[0] > ACC_LIMIT_LEFT) & (xValPrev <= ACC_LIMIT_LEFT)){		// Aktueller Messwert grösser und vorheriger Messwert kleiner/gleich unterer Schwellwert => Flankendetektion
		counterLeft = 0;	// Linkszähler nullen
		motionLeft = TRUE;	// Linksbewegung wurde erkannt
	} else if(xyzValue[0] > ACC_LIMIT_RIGHT){	// Messwert grösser als oberer Schwellwert
		motionLeft = FALSE;		// Linkszähler anhalten
	}
	if(motionLeft){		// Linksbewegung aktiv
		counterLeft++;	// Linkszähler inkrementieren
		if((counterLeft == ((timeMotionLeft - 2*img.size)/2) && (timeMotionLeft > 2*img.size*pixelTime))){	// Aktuelle Linkslaufzeit == Auslösezeit für Schriftzug
			imgShow = TRUE;	// Schriftzug auslösen
			//pixelTime = timeMotionLeft / (2*img.size);
		}
		/*if(counterLeft == timeMotionLeft/4){
			pixelTime = (timeMotionLeft/2)/(2*img.size);
			imgShow = TRUE;
		}*/
	}


	xValPrev = xyzValue[0];		// Variable für vorherigen Wert gleich aktuellem Messwert setzen

	if(((counterRight/**ACCEL_MEAS_FREQ_US*/)>(TIME_UNTIL_IDLE_US/TICK_FREQ_US))|((counterLeft/**ACCEL_MEAS_FREQ_US*/)>(TIME_UNTIL_IDLE_US/TICK_FREQ_US))){
		idleMode(currentState);
		checkStateChange(Z_TRESHOLD_8G, 10, xyzValue[2]);
	} else {
		isIdle = FALSE;
	}

	uint time = 0;

	if(!motionRight & (counterRight > 0)){	// Zeitmessung rechts fertig
		timeMotionRight = counterRight;// * ACCEL_MEAS_FREQ_US;	// gemessene Zeit in Variable speichern
		time = timeMotionRight;
		counterRight = 0;	// Rechtszähler nullen
		/*sumR += timeMotionRight;
		if(++i == 10){
			__asm__("nop");
		}*/
	}
	if(!motionLeft & (counterLeft > 0)){	// Zeitmessung links fertig
		timeMotionLeft = counterLeft;// * ACCEL_MEAS_FREQ_US;	//gemessene Zeit in Variable speichern
		//sumL += timeMotionLeft;
		time = timeMotionLeft;
		counterLeft = 0;	// Linkszähler nullen
	}




	if(time){
		time *= TICK_FREQ_US;
		if(time < 9000){
			pixelTime = 500/TICK_FREQ_US;
		} else if(time < 10000){
			pixelTime = 600/TICK_FREQ_US;
		} else if(time < 20000){
			pixelTime = 700/TICK_FREQ_US;
		} else if(time < 30000){
			pixelTime = 800/TICK_FREQ_US;
		} else if(time < 40000){			// schneller
			pixelTime = 900/TICK_FREQ_US;	//-----------------------
		} else if(time < 50000){			// normal
			pixelTime = 1000/TICK_FREQ_US;	//-----------------------
		} else if(time < 60000){			// langsamer
			pixelTime = 960/TICK_FREQ_US;
		} else if(time < 70000){
			pixelTime = 920/TICK_FREQ_US;
		} else if(time < 80000){
			pixelTime = 880/TICK_FREQ_US;
		} else if(time < 90000){
			pixelTime = 840/TICK_FREQ_US;
		} else {
			pixelTime = 800/TICK_FREQ_US;
		}

	}
/*
	if(timeMotionLeft>timeMotionLeftMax){
		timeMotionLeftMax = timeMotionLeft;
	}

	if(timeMotionRight>timeMotionRightMax){
		timeMotionRightMax = timeMotionRight;
	}
*/


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
#define STARTUP_WAIT_TIME_MS (60)
	//Self_Supply_SetVal();		// Selbsthaltung einschalten???
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
		case 8*STARTUP_WAIT_TIME_MS:
			LED1_Off();
			LED2_Off();
			LED3_Off();
			LED4_Off();
			LED5_Off();
			LED6_Off();
			LED7_Off();
			LED8_Off();
			break;
		case 8*STARTUP_WAIT_TIME_MS + 600:
			appStarted = TRUE;
			return;
	}
	i++;
}



/*
 * Wird vom Timer-Interrupt aufgerufen.
 * Setzt Events.
 */
void addTick(void){
	static uint i = 0;		// Zählervariable
	static uint16_t blankCntr = 0;
	static uint timeOutCntr;

	if(appStarted && !isBlanking){				// wenn Applikation läuft...
		setEvent(EVNT_WORK);
		if(imgShow && !(i % pixelTime)){//*/(PIXEL_TIME_US/TICK_FREQ_US))){			// wenn Schriftzug angezeigt werden soll...
			setEvent(EVNT_LED_SHOW_IMAGE);	// Event setzen
		}
		if(!(i % (ACCEL_MEAS_FREQ_US/TICK_FREQ_US))){	// Wenn Beschleunigungsmessung ausgeführt werden soll...
			setEvent(EVNT_ACCELERATION);	// Event setzen
		}
		if(isIdle){
			if(++timeOutCntr == TIMEOUT_S*(1000000/TICK_FREQ_US)){
#if HAS_TIMEOUT
				setEvent(EVNT_TIMEOUT);
#endif
			}
		} else {
			timeOutCntr = 0;
		}
	} else if(!(i % 1000/TICK_FREQ_US)){					// wenn Applikation noch nicht läuft
		setEvent(EVNT_STARTUP);	// Startupevent setzen
	}
	i++;	// Zähler inkrementieren
	if(isBlanking){
		if(++blankCntr == (10*TICK_FREQ_US)){
			isBlanking = FALSE;
			blankCntr = 0;
		}
	}
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

	fp_handleAcceleration = &handleAccelerationForImage;
	fp_getAccelValue = &getAccelValue_8bit;
	img.image = img_HSLU;
	img.size = sizeof(img_HSLU);

	// default Values
	xyzValue[0] = 0;
	xyzValue[1] = 0;
	xyzValue[2] = 0;
}

void deinitApplication(void){
	// nichts zu tun
}



/*
 * Sensoreinstellungen für Wasserwaage.
 */
void sensorResolution_2g(void){
	uint8_t tmp;
	res = LIS2DH12TR_ReadReg(LIS2DH12TR_CTRL_REG4, &tmp, 1U);
	tmp &= 0b11001111;
	res = LIS2DH12TR_WriteReg(LIS2DH12TR_CTRL_REG4, tmp);
}

/*
 * Sensoreinstellungen für Anzeige eines Schriftzuges mittels LEDs.
 */
void sensorResolution_8g(void){
	uint8_t tmp;
	res = LIS2DH12TR_ReadReg(LIS2DH12TR_CTRL_REG4, &tmp, 1U);
	tmp &= 0b11101111;
	tmp |= 0b00100000;
	res = LIS2DH12TR_WriteReg(LIS2DH12TR_CTRL_REG4, tmp);
}

/*
 * Messbereich des Sensors einstellen: 16g
 */
void sensorResolution_16g(void){
	uint8_t tmp;
	res = LIS2DH12TR_ReadReg(LIS2DH12TR_CTRL_REG4, &tmp, 1U);
	tmp |= 0b00110000;
	res = LIS2DH12TR_WriteReg(LIS2DH12TR_CTRL_REG4, tmp);
}

void sensorValueResolution_8bit(void){
	uint8_t tmp;
	res = LIS2DH12TR_ReadReg(LIS2DH12TR_CTRL_REG4, &tmp, 1U);
	tmp &= 0b11110111;
	res = LIS2DH12TR_WriteReg(LIS2DH12TR_CTRL_REG4, tmp);
	res = LIS2DH12TR_ReadReg(LIS2DH12TR_CTRL_REG1, &tmp, 1U);
	tmp &= 0b11101111;
	tmp |= 0b00001000;
	res = LIS2DH12TR_WriteReg(LIS2DH12TR_CTRL_REG1, tmp);

}

void sensorValueResolution_12bit(void){
	uint8_t tmp;
	res = LIS2DH12TR_ReadReg(LIS2DH12TR_CTRL_REG1, &tmp, 1U);
	tmp &= 0b11110111;
	tmp |= 0b00010000;
	res = LIS2DH12TR_WriteReg(LIS2DH12TR_CTRL_REG1, tmp);
	res = LIS2DH12TR_ReadReg(LIS2DH12TR_CTRL_REG4, &tmp, 1U);
	tmp |= 0b00001000;
	res = LIS2DH12TR_WriteReg(LIS2DH12TR_CTRL_REG4, tmp);
}
