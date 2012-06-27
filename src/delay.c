/*
 * delay.c
 *
 *  Created on: 22/06/2012
 *      Author: Kelvin
 */

#include "delay.h"

void initDelay(){
	LPC_TIM0->PR = 16;
	//enable timer 0
	LPC_TIM0->TCR = 1;
}
void wait(uint32_t timemicro){
	uint32_t currentCounter;

	currentCounter = LPC_TIM0->TC; // read current counter
		// Now loop until required number of ticks passes.
		while ((LPC_TIM0->TC - currentCounter) < timemicro);
}

uint32_t getTimeMicro(){
	return LPC_TIM0->TC;
}
