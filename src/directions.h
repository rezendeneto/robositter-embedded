/*
 * No sentido default do motor X, acionar o pwm 1 como 0 e o 2 com o valor da velocidade
 * */
#ifndef __DIRECTIONS_H
#define __DIRECTIONS_H

#include "LPC17xx.h"
#include "math.h"
#include "pwm.h"
#include "timer.h"

//MOTOR A
#define CHANNEL_PWM1			0
#define CHANNEL_PWM2			1
//MOTOR B
#define CHANNEL_PWM3			2
#define CHANNEL_PWM4			3
//#define CHANNEL_PWM3			3
//#define CHANNEL_PWM4			2
//MOTOR C
#define CHANNEL_PWM5			4
#define CHANNEL_PWM6			5

#define ENC1_PIN (1 << 6) //P2.6
#define ENC2_PIN (1 << 7) //P2.7
#define ENC3_PIN (1 << 8) //P2.8

//1s 25*
//100ms
#define TIMER0_INTERVAL	((3*(SystemFrequency/100)))

volatile uint32_t cenc1;
volatile uint32_t cenc2;
volatile uint32_t cenc3;

//velocidade de cada motor (de -100 até +100)
volatile int8_t speed1;
volatile int8_t speed2;
volatile int8_t speed3;
volatile int8_t speed_total;
volatile float_t angle_total;
volatile int8_t isRotate;
//volatile direction_current;
//direção 0 - normal; 1 - invertido motor 1 - bit 0, motor 2 bit 1, motor 3 bit 2
volatile int8_t dspeed;
volatile int8_t speedIncreased;

//speed(base_motor)/speed(outros motores) calculado
float_t prop1_ideal;
float_t prop2_ideal;

//enc(base_motor)/enc(outros motores) através do encoder
float_t prop1_real;
float_t prop2_real;

float_t dError1;
float_t currentError1;
float_t sumError1;
float_t lastError1;

float_t dError2;
float_t currentError2;
float_t sumError2;
float_t lastError2;

uint8_t base_motor; //motor base que não terá o PWM alterado

float_t values[20];
float_t values2[20];

int sp1[20];
int sp2[20];
int sp3[20];

uint8_t x;
uint8_t cont;

float_t Kp;
float_t Ki;
float_t Kd;

extern void init_pwm();
extern void move_robot(float_t angle, int speed);
extern void rotate_robot(int direction, int speed);
extern void stop_robot();
extern void encoder_init();
void moviment_start();
void update_motor();

#endif
