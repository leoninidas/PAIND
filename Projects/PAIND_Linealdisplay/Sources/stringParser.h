/*
 * stringParser.h
 *
 *  Created on: 18.03.2016
 *      Author: Mario
 */

#ifndef SOURCES_STRINGPARSER_H_
#define SOURCES_STRINGPARSER_H_

#include "stdint.h"

typedef enum parserStatus {ERR_PARS_OK, ERR_PARS_INV_CHAR, ERR_PARS_NOF_LETTERS_OF} parserStatus;


parserStatus setText(char*);
void connectArrays(uint8_t*, uint8_t[]);


#endif /* SOURCES_STRINGPARSER_H_ */
