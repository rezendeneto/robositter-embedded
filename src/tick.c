#include "tick.h"
/*
 * tick.c
 *
 *  Created on: 28/05/2012
 *      Author: Kelvin
 */
volatile uint32_t tick;
uint8_t preempt;

void sys_tick_init(uint32_t time){
	if(tick == 0){
		preempt = 1;
	}
	while(!preempt);
	tick = 0;
	if (SysTick_Config(SystemFrequency / time)) {
		while (1)
			; // Capture error
	}
	NVIC_SetPriority(SysTick_IRQn,0);
}
void SysTick_Handler(void) {
	tick++;
}
void systick_delay(uint32_t delayTicks) {
	preempt = 0;
	uint32_t currentTicks;

	currentTicks = tick; // read current tick counter
	// Now loop until required number of ticks passes.
	while ((tick - currentTicks) < delayTicks)
		;
	preempt = 1;
}

extern void tickStop(){
	//para de contar
	SysTick->CTRL = 0;
	tick = 0;
}
