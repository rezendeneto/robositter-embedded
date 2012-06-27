#include "directions.h"

/*
 *        ____________
 *       /			  \
 *      /  /A		B/\\
 *     /  /			  \	\
 *    /	 /	   /\	   \ \
 *   |	\/		|		\ |
 *   |			|		  |
 *   |			|		  |
 *   |					  |
 *    \					  /
 *     \				 /
 *      \	C----->		/
 *       \_____________/
 */
/******************************************************************************
 ** Function name:		init
 **
 ** Descriptions:		Initialize pins and variables**
 **
 ** parameters:			None)
 ** Returned value:		None
 **
 ******************************************************************************/
void init_pwm() {

	//initialize interrupt pins for encoder
	encoder_init();
	//initialize PWM pins
	PWM_Init();
	Kp = 2.5;
	Ki = 0;
	Kd = 0;
}

void encoder_init() {

	//pin configuration as input (p2.6, p2.7, p2.8)
	LPC_GPIO2->FIODIR &= ~(ENC1_PIN | ENC2_PIN | ENC3_PIN);

	//falling edge interrupt
	LPC_GPIOINT->IO2IntEnF |= (ENC1_PIN | ENC2_PIN | ENC3_PIN);

}
/******************************************************************************
 ** Function name:		move
 **
 ** Descriptions:		Move the robot forward or backward with a defined speed forever**
 **
 ** parameters:			angle (0 - 360), speed (0-100)
 ** Returned value:		None
 **
 ******************************************************************************/
void move_robot(float_t angle, int speed) {
	speed_total = speed;
	angle_total = angle;
	//	isRotate = 0;
	/* tranforma o angulo para radianos */
	angle = (angle * 0.02464);

	//todos os motores na direção padrão
	dspeed = 0;
	//calcula a proporção de velocidade de cada motor para o angulo especificado
	speed1 = cosf(2.618 - angle) * speed;//150
	speed2 = cosf(0.5235 - angle) * speed;//30
	speed3 = cosf(4.712 - angle) * speed;//270

	//ajusta para a velocidade total definida

	//ativa os motores, seta as flags de direção de cada motor e deixa speed em modulo
	if (speed1 >= 0) {
		PWM_Set(CHANNEL_PWM1, 0);
		PWM_Set(CHANNEL_PWM2, speed1);
	} else {
		dspeed |= 0x01;
		speed1 *= -1;
		PWM_Set(CHANNEL_PWM1, speed1);
		PWM_Set(CHANNEL_PWM2, 0);
	}
	if (speed2 >= 0) {
		PWM_Set(CHANNEL_PWM3, 0);
		PWM_Set(CHANNEL_PWM4, speed2);
	} else {
		dspeed |= 0x02;
		speed2 *= -1;
		PWM_Set(CHANNEL_PWM3, speed2);
		PWM_Set(CHANNEL_PWM4, 0);
	}
	if (speed3 >= 0) {
		PWM_Set(CHANNEL_PWM5, 0);
		PWM_Set(CHANNEL_PWM6, speed3);
	} else {
		dspeed |= 0x04;
		speed3 *= -1;
		PWM_Set(CHANNEL_PWM5, speed3);
		PWM_Set(CHANNEL_PWM6, 0);
	}

	if (speed1 > speed2) {
		if (speed1 > speed3) {
			base_motor = 1; //1 é o maior
			prop1_ideal = (float_t) speed2 / speed1;
			prop2_ideal = (float_t) speed3 / speed1;
		} else {
			base_motor = 3; //3 é o maior
			prop1_ideal = (float_t) speed1 / speed3;
			prop2_ideal = (float_t) speed2 / speed3;
		}
	} else {
		if (speed2 > speed3) {
			base_motor = 2; //2 é o maior
			prop1_ideal = (float_t) speed1 / speed2;
			prop2_ideal = (float_t) speed3 / speed2;
		} else {
			base_motor = 3; //3 é o maior
			prop1_ideal = (float_t) speed1 / speed3;
			prop2_ideal = (float_t) speed2 / speed3;
		}
	}

	moviment_start();
}

/******************************************************************************
 ** Function name:		rotate
 **
 ** Descriptions:		Rotate the robot left or right with a defined speed forever**
 **
 ** parameters:			direction (0 - left, 1 - right), speed (0-100)
 ** Returned value:		None
 **
 ******************************************************************************/
void rotate_robot(int direction, int speed) {
	isRotate = 1;
	speed_total = speed;
	//	direction_current = direction;

	speed1 = speed;
	speed2 = speed;
	speed3 = speed;
	if (direction) {
		//direção default dos motores
		dspeed = 0;

		PWM_Set(CHANNEL_PWM1, 0);
		PWM_Set(CHANNEL_PWM2, speed1);
		PWM_Set(CHANNEL_PWM3, 0);
		PWM_Set(CHANNEL_PWM4, speed2);
		PWM_Set(CHANNEL_PWM5, 0);
		PWM_Set(CHANNEL_PWM6, speed3);
	} else {
		//direção invertida dos motores
		dspeed = 0x07;

		PWM_Set(CHANNEL_PWM1, speed1);
		PWM_Set(CHANNEL_PWM2, 0);
		PWM_Set(CHANNEL_PWM3, speed2);
		PWM_Set(CHANNEL_PWM4, 0);
		PWM_Set(CHANNEL_PWM5, speed3);
		PWM_Set(CHANNEL_PWM6, 0);
	}
	prop1_ideal = 1;
	prop2_ideal = 1;
	base_motor = 3;
	moviment_start();

}

void moviment_start() {

	cenc1 = 0;
	cenc2 = 0;
	cenc3 = 0;
	speedIncreased = 0;
	Kd = 0;
	PWM_Start();
	init_timer(3, TIMER0_INTERVAL);

	x = 0;
	//talvez precise de um wait aqui
	enable_timer(3);
	cont = 0;
}

void TIMER3_IRQHandler(void) {
	update_motor();
	LPC_TIM3->IR = 1; /* clear interrupt flag */
	return;
}

//método do controle, é chamado a cada 100ms quando esta em movimento
void update_motor() {
	//verificar se esta se movendo

	//se não estiver se movimentando (contadores dos encoders menor que 3) E estiver girando (isRotate)
	//cont++;
	if (isRotate && cenc1 < 3 && cenc2 < 3 && cenc3 < 3) {
		//aumenta a velocidade de cada motor em 5%
		speed1 *= 1.1;
		speed2 *= 1.1;
		speed3 *= 1.1;

		speedIncreased = 1;
	}

	// se o robô deveria se movimentar e realmente está, então define as velocidades como as velocidades originais
	else if (speedIncreased) {
		speed1 = speed_total;
		speed2 = speed_total;
		speed3 = speed_total;

		speedIncreased = 0;
	}

	else if (base_motor == 1) {
		currentError1 = (cenc1 * prop1_ideal - cenc2);
		currentError2 = (cenc1 * prop2_ideal - cenc3);
		if (speed2 > 0) {
			dError1 = currentError1 - lastError1;
			sumError1 += currentError1;
			lastError1 = currentError1;
			speed2 += Kp * currentError1 + Ki * dError1 + Kd * dError1;
		}
		if (speed3 > 0) {
			speed3 += Kp * (cenc1 * prop2_ideal - cenc3);
		}
	} else if (base_motor == 2) {
		if (speed1 > 0) {
			speed1 += Kp * (cenc2 * prop1_ideal - cenc1);
		}
		if (speed3 > 0) {
			speed3 += Kp * (cenc2 * prop2_ideal - cenc3);
		}
	} else {
		if (speed1 > 0) {
			speed1 += Kp * (cenc3 * prop1_ideal - cenc1);
		}
		if (speed2 > 0) {
			speed2 += Kp * (cenc3 * prop2_ideal - cenc2);
		}

		prop1_real = (float) cenc1 / cenc3;
		prop2_real = (float) cenc2 / cenc3;
	}

	//limitações de velocidade
	if (speed1 > 100) {
		speed1 = 100;
	} else if (speed1 < 0) {
		speed1 = 1;
	}
	if (speed2 > 100) {
		speed2 = 100;
	} else if (speed2 < 0) {
		speed2 = 1;
	}
	if (speed3 > 100) {
		speed3 = 100;
	} else if (speed3 < 0) {
		speed3 = 1;
	}

	//atualização do PWM
	if (dspeed & 0x01) {
		PWM_Set(CHANNEL_PWM1, speed1);
	} else {
		PWM_Set(CHANNEL_PWM2, speed1);
	}
	if (dspeed & 0x02) {
		PWM_Set(CHANNEL_PWM3, speed2);
	} else {
		PWM_Set(CHANNEL_PWM4, speed2);
	}
	if (dspeed & 0x04) {
		PWM_Set(CHANNEL_PWM5, speed3);
	} else {
		PWM_Set(CHANNEL_PWM6, speed3);
	}

	cenc1 = 0;
	cenc2 = 0;
	cenc3 = 0;

	//Kd = VALOR DO KD;
	PWM_Start();
}

/******************************************************************************
 ** Function name:		stop
 **
 ** Descriptions:		Stop the robot movements
 **
 ** parameters:			None
 ** Returned value:		None
 **
 ******************************************************************************/
void stop_robot() {
	disable_timer(3);
	speed1 = 0;
	speed2 = 0;
	speed3 = 0;
	speed_total = 0;
	isRotate = 0;
	PWM_Set(CHANNEL_PWM1, 0);
	PWM_Set(CHANNEL_PWM2, 0);
	PWM_Set(CHANNEL_PWM3, 0);
	PWM_Set(CHANNEL_PWM4, 0);
	PWM_Set(CHANNEL_PWM5, 0);
	PWM_Set(CHANNEL_PWM6, 0);
	PWM_Start();
}
