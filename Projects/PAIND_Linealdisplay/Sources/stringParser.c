/*
 * stringParser.c
 *
 *  Created on: 18.03.2016
 *      Author: Mario
 */

#include "stringParser.h"

#define STRING_MAX_LENGTH	10

uint8_t letter_H[] = {0x7e, 0x08, 0x08, 0x7e};
uint8_t letter_L[] = {0x7e, 0x40, 0x40, 0x40};
uint8_t letter_S[] = {0x44, 0x4a, 0x4a, 0x32};
uint8_t letter_U[] = {0x3e, 0x40, 0x40, 0x3e};

uint8_t* pattern;


/*
 * Replaces the string "text" by the pattern, which will be shown by the LEDs
 */
parserStatus setText(char* text)
{
	char tmp = 0;
	uint8_t i = 0;

	while(text[i] != '\0')		// HSLUSLU\0   ??? whaaaat???
	{
		if(i > STRING_MAX_LENGTH)
		{
			return ERR_PARS_NOF_LETTERS_OF;
		}

		tmp = text[i];
		switch(tmp)
		{
			case 'H':
				connectArrays(pattern, letter_H);
				break;

			case 'L':
				connectArrays(pattern, letter_L);
				break;

			case 'S':
				connectArrays(pattern, letter_S);
				break;

			case 'U':
				connectArrays(pattern, letter_U);
				break;

			default:
				return ERR_PARS_INV_CHAR;
		}
		i++;
	}

	return ERR_PARS_OK;
}



/*
 * Concatenates two arrays
 * Puts "letterPattern" at the end of "string" and returns a new array
 */

void connectArrays(uint8_t* string, uint8_t* letterPattern)
{
	uint8_t tmp[sizeof(string)+sizeof(letterPattern)+1];		// two arrays + a blank
	uint8_t i,j;

	while(i<sizeof(string))
	{
		tmp[i] = string[i];		// adding the first array at the beginning of the new array
		i++;
	}

	tmp[++i] = 0;				// adding a blank between the letters

	while(j<sizeof(letterPattern))
	{
		tmp[i+j] = letterPattern[j];	// adding the second array after the blank
	}

	pattern = &tmp;
}


