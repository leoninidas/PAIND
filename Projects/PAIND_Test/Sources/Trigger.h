/*
 * Trigger.h
 *
 *  Created on: 28.03.2016
 *      Author: Mario
 */

#ifndef SOURCES_TRIGGER_H_
#define SOURCES_TRIGGER_H_

#include <stdint.h>

typedef enum
{
	SHOW_IMAGE,			// show next LED row
	MEASURE_ACCEL,		// get next acceleration value
	NOF_TRIGGERS		// number of triggers
} TriggerKind;

typedef void* TriggerCallbackDataPointer;

typedef uint16_t TriggerTime;

typedef void (*TriggerCallback)(CallBackDataPtr);


uint8_t SetTrigger(TriggerKind trigger, TriggerTime ticks, TriggerCallback callback, TriggerCallbackDataPointer data);

/*! \brief Called from interrupt service routine with a period of TRG_TICKS_MS. */
void AddTriggerTick(void);

/*!\brief De-initializes the module. */
void Trigger_Deinit(void);

/*!\brief Initializes the module. */
void Trigger_Init(void);

#endif /* SOURCES_TRIGGER_H_ */
