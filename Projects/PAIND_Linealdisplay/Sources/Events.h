/* ###################################################################
**     Filename    : Events.h
**     Project     : PAIND_Linealdisplay
**     Processor   : MKL04Z16VLF4
**     Component   : Events
**     Version     : Driver 01.00
**     Compiler    : GNU C Compiler
**     Date/Time   : 2016-03-17, 06:42, # CodeGen: 0
**     Abstract    :
**         This is user's event module.
**         Put your event handler code here.
**     Contents    :
**         Cpu_OnNMIINT - void Cpu_OnNMIINT(void);
**
** ###################################################################*/
/*!
** @file Events.h
** @version 01.00
** @brief
**         This is user's event module.
**         Put your event handler code here.
*/         
/*!
**  @addtogroup Events_module Events module documentation
**  @{
*/         

#ifndef __Events_H
#define __Events_H
/* MODULE Events */

#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"
#include "LED1.h"
#include "LEDpin1.h"
#include "BitIoLdd1.h"
#include "WAIT1.h"
#include "Self_Supply.h"
#include "BitIoLdd2.h"
#include "LED2.h"
#include "LEDpin2.h"
#include "BitIoLdd3.h"
#include "LED3.h"
#include "LEDpin3.h"
#include "BitIoLdd4.h"
#include "LED4.h"
#include "LEDpin4.h"
#include "BitIoLdd5.h"
#include "LED5.h"
#include "LEDpin5.h"
#include "BitIoLdd6.h"
#include "LED6.h"
#include "LEDpin6.h"
#include "BitIoLdd7.h"
#include "LED7.h"
#include "LEDpin7.h"
#include "BitIoLdd8.h"
#include "LED8.h"
#include "LEDpin8.h"
#include "BitIoLdd9.h"
#include "I2C1.h"
#include "TI1.h"
#include "TimerIntLdd1.h"
#include "CS1.h"
#include "TU1.h"
#include "VDD_ACCEL.h"
#include "BitIoLdd10.h"
#include "IFsh1.h"
#include "IntFlashLdd1.h"

#ifdef __cplusplus
extern "C" {
#endif 

/*
** ===================================================================
**     Event       :  Cpu_OnNMIINT (module Events)
**
**     Component   :  Cpu [MKL04Z32LF4]
*/
/*!
**     @brief
**         This event is called when the Non maskable interrupt had
**         occurred. This event is automatically enabled when the [NMI
**         interrupt] property is set to 'Enabled'.
*/
/* ===================================================================*/
void Cpu_OnNMIINT(void);


/*
** ===================================================================
**     Event       :  TI1_OnInterrupt (module Events)
**
**     Component   :  TI1 [TimerInt]
**     Description :
**         When a timer interrupt occurs this event is called (only
**         when the component is enabled - <Enable> and the events are
**         enabled - <EnableEvent>). This event is enabled only if a
**         <interrupt service/event> is enabled.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
void TI1_OnInterrupt(void);

/*
** ===================================================================
**     Event       :  I2C1_OnMasterBlockSent (module Events)
**
**     Component   :  I2C1 [I2C_LDD]
*/
/*!
**     @brief
**         This event is called when I2C in master mode finishes the
**         transmission of the data successfully. This event is not
**         available for the SLAVE mode and if MasterSendBlock is
**         disabled. 
**     @param
**         UserDataPtr     - Pointer to the user or
**                           RTOS specific data. This pointer is passed
**                           as the parameter of Init method.
*/
/* ===================================================================*/
void I2C1_OnMasterBlockSent(LDD_TUserData *UserDataPtr);

/*
** ===================================================================
**     Event       :  I2C1_OnMasterBlockReceived (module Events)
**
**     Component   :  I2C1 [I2C_LDD]
*/
/*!
**     @brief
**         This event is called when I2C is in master mode and finishes
**         the reception of the data successfully. This event is not
**         available for the SLAVE mode and if MasterReceiveBlock is
**         disabled.
**     @param
**         UserDataPtr     - Pointer to the user or
**                           RTOS specific data. This pointer is passed
**                           as the parameter of Init method.
*/
/* ===================================================================*/
void I2C1_OnMasterBlockReceived(LDD_TUserData *UserDataPtr);

/* END Events */

#ifdef __cplusplus
}  /* extern "C" */
#endif 

#endif 
/* ifndef __Events_H*/
/*!
** @}
*/
/*
** ###################################################################
**
**     This file was created by Processor Expert 10.5 [05.21]
**     for the Freescale Kinetis series of microcontrollers.
**
** ###################################################################
*/
