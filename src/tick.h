#include "LPC17xx.h"                        /* LPC13xx definitions */


volatile uint32_t tick;
extern void sys_tick_init(uint32_t time);
extern void tickStop();
extern void systick_delay(uint32_t delayTicks);
