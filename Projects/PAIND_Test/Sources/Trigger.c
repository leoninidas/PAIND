/*
 * Trigger.c
 *
 *  Created on: 28.03.2016
 *      Author: Mario
 */



#include "CS1.h"
#include "Trigger.h"


typedef struct TriggerDesc {
  TriggerTime ticks;    /*!< tick count until trigger */
  TriggerCallback callback;    /*!< callback function */
  TriggerCallbackDataPointer data; /*!< additional data pointer for callback */
} TriggerDesc;

static TriggerDesc Triggers[NOF_TRIGGERS];  /*!< Array of triggers */

uint8_t SetTrigger(TriggerKind trigger, TriggerTime ticks, TriggerCallback callback, TriggerCallbackDataPointer data){
	CS1_CriticalVariable();

	 CS1_EnterCritical();
	 Triggers[trigger].ticks = ticks;
	 Triggers[trigger].callback = callback;
	 Triggers[trigger].data = data;
	 CS1_ExitCritical();
	 return ERR_OK;
}

/*! \brief Called from interrupt service routine with a period of TRG_TICKS_MS. */
static bool CheckTriggerCallbacks(void){
	TriggerKind i;
	TriggerCallback callback;
	TriggerCallbackDataPointer data;
	bool calledCallBack = FALSE;
	CS1_CriticalVariable()

	for(i = (TriggerKind) 0; i < NOF_TRIGGERS; i++) {
		CS1_EnterCritical();
	    if (Triggers[i].ticks==0 && Triggers[i].callback != NULL) { /* trigger! */
	    	callback = Triggers[i].callback; /* get a copy */
	    	data = Triggers[i].data; /* get backup of data, as we overwrite it below */
	    	/* reset trigger structure, as callback might setup this trigger again */
	    	Triggers[i].callback = NULL; /* NULL callback prevents that we are called again */
	    	CS1_ExitCritical();
	    	callback(data);
	    	calledCallBack = TRUE; /* callback may have set a trigger at the current time: rescan trigger list */
	    } else {
	    	CS1_ExitCritical();
	    }
	  } /* for */
	return calledCallBack;
}

void AddTriggerTick(void) {
	TriggerKind i;
	CS1_CriticalVariable()

	CS1_EnterCritical();
	for(i = (TriggerKind) 0;  i < NOF_TRIGGERS; i++) {
		if (Triggers[i].ticks!=0) { /* prevent underflow */
			Triggers[i].ticks--;
		}
	} /* for */
	CS1_ExitCritical();
	while(CheckTriggerCallbacks()) {} /* while we have callbacks, re-iterate the list as this may have added new triggers at the current time */
}

/*!\brief De-initializes the module. */
void Trigger_Deinit(void);

/*!\brief Initializes the module. */
void Trigger_Init(void);
