#include "LPC17xx.h"
#include "ultrasound.h"
#include "tick.h"




/*************************************************
 * init_ultrasound()
 *
 * inicializa os pinos do sensor de ultrassom
 *************************************************/
void init_ultrasound(){
	newValue = 0;
	distance = 0;
	t0 = 0;
	//trigger  p0.7 como output
	LPC_GPIO0->FIODIR |= TRIGGER_PIN;

	//eccho p0.6 como input
	LPC_GPIO0->FIODIR &= ~(ECHO_PIN);
	//enable interrupt faling and raise edge for eccho pin
	LPC_GPIOINT->IO0IntEnF |= (ECHO_PIN);
	LPC_GPIOINT->IO0IntEnR |= (ECHO_PIN);
}

/*************************************************
 * sendTrigger()
 *
 * envia o trigger para realizar uma leitura da distancia
 *************************************************/
void sendTrigger() {
	newValue = 0;
	//muda contador para us
	sys_tick_init(1000000);
	//seta saida como alta
	LPC_GPIO0->FIOSET |= TRIGGER_PIN;
	//espera 10us
	systick_delay(10);
	//seta pino em baixa
	LPC_GPIO0->FIOCLR |= TRIGGER_PIN;

}
