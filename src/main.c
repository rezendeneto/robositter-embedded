/*
===============================================================================
 Name        : main.c
 Author      : 
 Version     :
 Copyright   : Copyright (C) 
 Description : main definition
===============================================================================
 */

#ifdef __USE_CMSIS
#include "LPC17xx.h"
#endif
//#define PRINTF
#include <cr_section_macros.h>
#include <NXP/crp.h>

#include "directions.h"
#include "timer.h"
#include "ultrasound.h"
//#include "stdio.h"
#include "ssp.h"
#include "nRF24L01.h"
#include "adc.h"
#include "delay.h"
// Variable to store CRP value in. Will be placed automatically
// by the linker when "Enable Code Read Protect" selected.
// See crp.h header for more information
__CRP const unsigned int CRP_WORD = CRP_NO_CRP ;
/*			0			1
 *bit 0:	parado		movimento
 *bit 1:				frente
 *bit 2					trás
 *bit 3:				esquerda
 *bit 4:				direita
 *
 * 0° = 	90° =	180º =	270º =
 */
uint8_t direction;
uint8_t current_speed;
uint8_t current_angle;
uint8_t aut_mode;
uint8_t i,j;
uint8_t ignore_all;
uint8_t spi_data[SSP_BUFSIZE];
uint8_t lerultrassom, ultrassomlido;
uint8_t sliding;
uint8_t stopmoviment;

extern volatile uint32_t ADCValue[ADC_NUM];
uint32_t ADCColision[ADC_NUM];
uint32_t ADCBaseValue[ADC_NUM];
extern volatile uint32_t ADCIntDone;

#define mediaad 10
#define MIN_DIST 20
#define COLISION_TIMEOUT 1000
#define LED_PIN (1 << 9)
//1s
#define TIMER1_INTERVAL	((10*(SystemFrequency/100)))
#define TIMER2_INTERVAL	((25*(SystemFrequency/100)))
#define _360to255(x) x*255/360

void led_toogle(){
	int ledstate;
	// Read current state of GPIO P0_0..31, which includes LED
	ledstate = LPC_GPIO0->FIOPIN;
	// Turn off LED if it is on
	// (ANDing to ensure we only affect the LED output)
	LPC_GPIO0->FIOCLR = ledstate & LED_PIN;
	// Turn on LED if it is off
	// (ANDing to ensure we only affect the LED output)
	LPC_GPIO0->FIOSET = ((~ledstate) & LED_PIN);
}

void msg_handle(){
	//printf("Got packet: \n");
	/*
	 * Get load the packet into the buffer.
	 */
	led_toogle();
	wait(10000);

	getData(spi_data);

	//printf("%d %d %d\n",spi_data[0],spi_data[1],spi_data[2]);
	setTADDR((uint8_t *)"canel");
	//envia de volta a msg recebida (para a estação-base saber que o comando foi executado)
	send_rf(spi_data);

	switch (spi_data[0]){
	case MSG_PING:
		//	printf("ping\n");
		break;
	case MSG_MODO_AUT:
		//enable ultrasound timer
		aut_mode = 1;
		init_timer(1,TIMER1_INTERVAL);
		enable_timer(1);
		break;
	case MSG_MODO_MANUAL:
		//	printf("modo manual\n");
		aut_mode = 0;
		disable_timer(1);
		//disable ultrassom
		break;
	case MSG_MOVE:
		if(!ignore_all){
			current_speed = spi_data[2];
			current_angle = spi_data[1];
			move_robot(spi_data[1], spi_data[2]);
		}
		break;
	case MSG_ROTATE:
		if(!ignore_all){
			rotate_robot(spi_data[1], spi_data[2]);
			/*rotate_robot(spi_data[1], 65);
			wait(50000);
			rotate_robot(spi_data[1], 50);
			wait(75000);
			rotate_robot(spi_data[1], 37);*/
		}
		break;
	case MSG_STOP:
		current_speed = 0;
		current_angle = 0;
		stop_robot();
		//	printf("stop\n");
		break;
	default:
		break;
	}


	return;
}

void ultra_handler(){
	//return;
	//printf("colisao ultrassom\n");
	if(current_speed == 0 || distance < 8){
		move_robot(_360to255(135), 70);
		disable_timer(2);
		enable_timer(2);
	}else{
		//chuta um lado
		current_angle = (_360to255(90)+current_angle)/2;
		move_robot(current_angle,current_speed);
		disable_timer(2);
		enable_timer(2);
	}
}
/***************
 *  Interrupts -
 *************  */
void EINT3_IRQHandler (void){
	/****** encoders ******/
	if(LPC_GPIOINT->IO2IntStatF & ENC1_PIN){
		cenc1++;
		LPC_GPIOINT->IO2IntClr |= ENC1_PIN;
	}
	else if(LPC_GPIOINT->IO2IntStatF & ENC2_PIN){
		cenc2++;
		LPC_GPIOINT->IO2IntClr |= ENC2_PIN;
	}
	else if(LPC_GPIOINT->IO2IntStatF & ENC3_PIN){
		cenc3++;
		LPC_GPIOINT->IO2IntClr |= ENC3_PIN;
	}

	/****** Ultrasound ******/
	else if(LPC_GPIOINT->IO0IntStatR & ECHO_PIN){
		//começa a contar em us

		t0 = getTimeMicro();
		LPC_GPIOINT->IO0IntClr |= ECHO_PIN;
	}
	else if(LPC_GPIOINT->IO0IntStatF & ECHO_PIN){
		//para de contar
		t1 = getTimeMicro();
		if(t1 - t0 < 20000){
			distance = (t1 - t0)/58;
			//printf("dist = %d\n",distance);
		}else{
			distance = 0;
		}
		if(aut_mode && distance < MIN_DIST && distance > 1){
			ignore_all = 1;
			ultra_handler();
		}
		ultrassomlido = 1;

		LPC_GPIOINT->IO0IntClr |= ECHO_PIN;

	}
	/****** Transceiver ******/
	else if(LPC_GPIOINT->IO0IntStatF & nIRQ_PIN){
		//msg_handle();
		LPC_GPIOINT->IO0IntClr |= nIRQ_PIN;
	}
	else{
		//bugou?
		LPC_GPIOINT->IO0IntClr |= (1<<3);
	}
	return;
}

//timer para leitura ultrassom
void TIMER1_IRQHandler (void){
	lerultrassom = 1;
	LPC_TIM1->IR = 1;			/* clear interrupt flag */
	return;
}
//timer para ativação do motor (desvio de obstaculos)
void TIMER2_IRQHandler(){
	stop_robot();
	ignore_all = 0;
	disable_timer(2);
	LPC_TIM2->IR = 1;
	return;
}
void bump_handler(uint8_t adcNum){
	ADCColision[i] = 0;
	//printf("colisao bumper %d ADCColision = %d ADCBase = %d\n",adcNum,ADCValue[adcNum],ADCBaseValue[adcNum] );
	ignore_all = 1;
	stop_robot();
	switch(adcNum){
	case 0://45° -> 180
		move_robot(_360to255(180),75);
		break;
	case 1://90°  -> 225
		move_robot(_360to255(225),75);
		break;
	case 2://135° -> 270
		move_robot(_360to255(270),75);
		break;
	case 3://180° -> 315
		move_robot(_360to255(315),75);
		break;
	case 4://225° -> 0
		move_robot(_360to255(0),75);
		break;
	case 5://270° -> 45
		move_robot(_360to255(45),75);
		break;
	case 6://315° -> 90
		move_robot(_360to255(90),65);
		break;
	}
	disable_timer(2);
	enable_timer(2);
	return;
}

int main(void) {

	/* SystemClockUpdate() updates the SystemFrequency variable */
	SystemClockUpdate();

	/* incializa o sys_tick para contar de 1ms em 1ms */
	//sys_tick_init(1000);
	/* espera 10ms (settling time dos componentes) */
	initDelay();
	wait(100000);
	/* initialize SSP port */
	SSP0Init();
	/* inicializa pinos de PWM */
	init_pwm();
	init_ultrasound();
	/* Initialize ADC  */
	ADCInit( ADC_CLK );

	/*inicia led rf*/
	LPC_GPIO0->FIODIR |= LED_PIN;
	LPC_GPIO0->FIOSET |= LED_PIN;

	//initialize timer for ultrasound
	current_speed = 0;
	current_angle = 0;
	aut_mode = 0;
	ignore_all = 0;
	lerultrassom = 0;
	ultrassomlido = 0;
	sliding = 0;
	stopmoviment = 0;

	for(i = 0; i < SSP_BUFSIZE; i++){
		spi_data[i] = 0;
	}

	for(i = 0; i < ADC_NUM; i++){
		ADCBaseValue[i] = 0;
		ADCColision[i] = 0;
	}
	/* Inicializa dados dos IR : média dos 10 primeiros valores lidos */
	for(j = 0; j < mediaad; j++){
		for ( i = 0; i < ADC_NUM; i++ )
		{
			ADCRead( i );
			while ( !ADCIntDone );
			ADCIntDone = 0;
			ADCBaseValue[i] += ADCValue[i];

		}
	}
	for(i = 0; i < ADC_NUM; i++){
		ADCBaseValue[i] /= mediaad;
		//printf("if %d valor inicial = %d\n",i,ADCBaseValue[i]);
	}
	//systick_delay(100);
	/* configure rf deivce */
	config_rf();

	/* Manda sinal que está vivo */
	//setTADDR((uint8_t *)"canel");
	//uint8_t aux[4] = { MSG_ALIVE, 0, 0 ,0};
	//send_rf(aux);
	LPC_SC->PCONP |=  PCTIM2_POWERON | PCTIM3_POWERON;

	init_timer(1,TIMER1_INTERVAL);
	//enable_timer(1);

	init_timer(2,TIMER2_INTERVAL);



	//stop_robot();
	//systick_delay(10);//settling time da loucura
	//enable interrupts
	//NVIC_SetPriority(EINT3_IRQn, 10);
	NVIC_EnableIRQ(EINT3_IRQn);


	/*main loop*/

	while(1){
		if(lerultrassom){
			sendTrigger();
			lerultrassom = 0;
			uint32_t ti = getTimeMicro();
			while(!ultrassomlido && getTimeMicro() < ti + 50000);

			ultrassomlido = 0;
		}
		/* lê valores dos IR */
		for ( i = 0; i < ADC_NUM && aut_mode; i++ )
		{
			ADCRead( i );
			while ( !ADCIntDone );
			ADCIntDone = 0;
			if(ADCValue[i] < ADCBaseValue[i]/1.55){
				ADCColision[i]++;
				if(ADCColision[i] > 5){
					bump_handler(i);
				}
			}else{
				ADCColision[i] = 0;
			}

		}
		while(dataReady()){
			msg_handle();
		}

	}
	return 0 ;
}
