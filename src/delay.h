/*
 * delay.h
 *
 *  Created on: 22/06/2012
 *      Author: Kelvin
 */

#ifndef DELAY_H_
#define DELAY_H_
#include "LPC17xx.h"                        /* LPC13xx definitions */

void initDelay();
void wait(uint32_t timemicro);
uint32_t getTimeMicro();

#endif /* DELAY_H_ */
