/*
 * showImage.c
 *
 *  Created on: 18.03.2016
 *      Author: Mario
 */

#include "showImage.h"
#include "LED1.h"
#include "LED2.h"
#include "LED3.h"

#define MASK_LED1	0x04
#define MASK_LED2	0x02
#define MASK_LED3	0x01
#define PIXEL_TIME_MS	1
#define BLANK_TIME_MS	300

uint8_t img[] = {7,2,7,0,7,7,7,0,7,4,4,0,7,4,7};

void runShowImage(void)
{
	initShowImage();

	uint8_t lengthOfImage = sizeof(img);
	char a,b,c;

	for(;;)
	{
		for(int i=0; i<lengthOfImage; i++)
		{
			LED1_Put(img[i] & MASK_LED1);
			LED2_Put(img[i] & MASK_LED2);
			LED3_Put(img[i] & MASK_LED3);

			WAIT1_Waitms(PIXEL_TIME_MS);

			LED1_Off();
			LED2_Off();
			LED3_Off();

			WAIT1_Waitms(PIXEL_TIME_MS);
		}
		WAIT1_Waitms(BLANK_TIME_MS);
	}

}

void initShowImage(void)
{
	LED1_Off();
	LED1_Off();
	LED1_Off();
}

