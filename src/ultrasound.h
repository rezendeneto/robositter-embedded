/*
 *
 */
#ifndef __ULTRASOUND_H
#define __ULTRASOUND_H

#define TRIGGER_PIN (1 << 7)
#define ECHO_PIN (1 << 8)
#endif

volatile uint16_t distance;
volatile uint32_t t0, t1;//inicio da contagem
volatile uint8_t newValue;
void init_ultrasound();
int readDataUltraSound();
void sendTrigger();
