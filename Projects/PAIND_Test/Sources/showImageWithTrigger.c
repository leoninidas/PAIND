/*
 * showImageWithTrigger.c
 *
 *  Created on: 18.03.2016
 *      Author: Mario
 */

#include "showImage.h"
#include "WAIT1.h"
#include "Trigger.h"
#include "LED1.h"
#include "LED2.h"
#include "LED3.h"

#define MASK_LED1	0x04
#define MASK_LED2	0x02
#define MASK_LED3	0x01
#define PIXEL_TIME_MS	1

uint8_t img[] = {7,2,7,0,7,7,7,0,7,4,4,0,7,4,7};

void ShowImageWithTrigger(void)
{
	static uint8_t i = 0;

	LED1_Put(img[i] & MASK_LED1);
	LED2_Put(img[i] & MASK_LED2);
	LED3_Put(img[i] & MASK_LED3);

	WAIT1_Waitms(PIXEL_TIME_MS);

	LED1_Off();
	LED2_Off();
	LED3_Off();

	i++;

	if(i < sizeof(img)){
		SetTrigger(SHOW_IMAGE, 1U, ShowImageWithTrigger, NULL);
	}
}

void initShowImageWithTrigger(void)
{
	SetTrigger(SHOW_IMAGE, 0, NULL, NULL);
	LED1_Off();
	LED1_Off();
	LED1_Off();
}

