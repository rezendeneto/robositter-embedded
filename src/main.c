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
#include "stdio.h"
#include "ssp.h"
#include "nRF24L01.h"
#include "tick.h"
#include "adc.h"

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
uint8_t t1;
extern volatile uint32_t ADCValue[ADC_NUM];
uint32_t ADCColision[ADC_NUM];
uint32_t ADCBaseValue[ADC_NUM];
extern volatile uint32_t ADCIntDone;

#define MIN_DIST 30
#define COLISION_TIMEOUT 1000
//1s
#define TIMER1_INTERVAL	((10*(SystemFrequency/100)))
#define TIMER2_INTERVAL	((25*(SystemFrequency/100)))
#define norm(x) x*255/360

void msg_handle(){
	//printf("Got packet: \n");
	/*
	 * Get load the packet into the buffer.
	 */
	systick_delay(10);

	getData(spi_data);
	//printf("%d %d %d\n",spi_data[0],spi_data[1],spi_data[2]);
	setTADDR((uint8_t *)"canel");

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
			if(current_speed == 0 || !aut_mode){//primeira vez
				current_speed = spi_data[2];
				current_angle = spi_data[1];
			}else {
				//soma os vetores
				current_angle = (current_angle + spi_data[1])/2;
			}
			move_robot(spi_data[1], spi_data[2]);
		}
		break;
	case MSG_ROTATE:
		if(!ignore_all){
			rotate_robot(spi_data[1], spi_data[2]);
			//	printf("rotate\n");
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

	//envia de volta a msg recebida (para a estação-base saber que o comando foi executado)
	send_rf(spi_data);
	return;
}

void ultra_handler(){
	#ifdef PRINTF
	printf("colisao ultrassom\n");
	#endif
	if(current_speed == 0){
		move_robot(norm(180), 70);
		disable_timer(2);
		enable_timer(2);
	}else{
		//chuta um lado
		current_angle = (norm(90)+current_angle)/2;
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

		t0 = tick;
		LPC_GPIOINT->IO0IntClr |= ECHO_PIN;
	}
	else if(LPC_GPIOINT->IO0IntStatF & ECHO_PIN){
		//para de contar
		if(tick - t0 < 20000){
			distance = (tick - t0)/58;
			//printf("dist = %d\n",distance);
		}else{
			distance = 0;
		}
		if(aut_mode && distance < MIN_DIST && distance > 1){
			ignore_all = 1;
			ultra_handler();
		}
		ultrassomlido = 1;
#ifdef PRINTF
		//printf("desceu\n");
#endif
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
	//printf("timer 2 \n");
	stop_robot();
	ignore_all = 0;
	LPC_TIM2->IR = 1;
	disable_timer(2);
	return;
}
void bump_handler(uint8_t adcNum){
	ADCColision[i] = 0;
	#ifdef PRINTF
	printf("colisao bumper %d\n",adcNum);
	#endif
	ignore_all = 1;
	stop_robot();
	switch(adcNum){
	case 0://45° -> 180
		move_robot(norm(180),75);
		break;
	case 1://90°  -> 225
		move_robot(norm(225),75);
		break;
	case 2://135° -> 270
		move_robot(norm(270),75);
		break;
	case 3://180° -> 315
		move_robot(norm(315),75);
		break;
	case 4://225° -> 0
		move_robot(norm(0),75);
		break;
	case 5://270° -> 45
		move_robot(norm(45),75);
		break;
	case 6://315° -> 90
		move_robot(norm(90),75);
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
	sys_tick_init(1000);
	/* espera 10ms (settling time dos componentes) */
	systick_delay(100);
	/* initialize SSP port */
	SSP0Init();
	/* inicializa pinos de PWM */
	init_pwm();
	init_ultrasound();
	/* Initialize ADC  */
	ADCInit( ADC_CLK );

	//initialize timer for ultrasound

	current_speed = 0;
	current_angle = 0;
	aut_mode = 0;
	ignore_all = 0;
	lerultrassom = 0;
	ultrassomlido = 0;
	sliding = 0;

	for(i = 0; i < SSP_BUFSIZE; i++){
		spi_data[i] = 0;
	}

	for(i = 0; i < ADC_NUM; i++){
		ADCBaseValue[i] = 0;
		ADCColision[i] = 0;
	}
	/* Inicializa dados dos IR : média dos 10 primeiros valores lidos */
	for(j = 0; j < 5; j++){
		for ( i = 0; i < ADC_NUM; i++ )
		{
			ADCRead( i );
			while ( !ADCIntDone );
			ADCIntDone = 0;
			ADCBaseValue[i] += ADCValue[i];

		}
	}
	for(i = 0; i < ADC_NUM; i++){
		ADCBaseValue[i] /= 10;
	}
	//systick_delay(100);
	/* configure rf deivce */
	config_rf();

	/* Manda sinal que está vivo */
	setTADDR((uint8_t *)"canel");
	uint8_t aux[4] = { MSG_ALIVE, 0, 0 ,0};
	//send_rf(aux);


	//init_timer(1,TIMER1_INTERVAL);
	//enable_timer(1);

	//init_timer(2,TIMER2_INTERVAL);



	//systick_delay(10);//settling time da loucura
	//enable interrupts
	NVIC_SetPriority(EINT3_IRQn, 10);
	NVIC_EnableIRQ(EINT3_IRQn);

	rotate_robot(0, 45);
	/*main loop*/
	while(1){

		if(lerultrassom){
			sendTrigger();
			t1 = tick;
			lerultrassom = 0;
			while(!ultrassomlido/* && tick < t1 + 100000*/);
			/*if(tick >= t1 + 100000){
				printf("asd\n");

			}
			*/sys_tick_init(1000);
			ultrassomlido = 0;
		}
		/* lê valores dos IR */
		for ( i = 0; i < ADC_NUM && aut_mode; i++ )
		{
			ADCRead( i );
			while ( !ADCIntDone );
			ADCIntDone = 0;
			if(ADCValue[i] < ADCBaseValue[i]/1.75){
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
		systick_delay(10);
	}
	return 0 ;
}
