/****************************************************************************
 *   $Id:: pwm.c 5748 2010-11-30 23:48:37Z usb00423                         $
 *   Project: NXP LPC17xx PWM example
 *
 *   Description: **TESTE**
 *     This file contains PWM code example which include PWM initialization, 
 *     PWM interrupt handler, and APIs for PWM access.
 *
 ****************************************************************************
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * products. This software is supplied "AS IS" without any warranties.
 * NXP Semiconductors assumes no responsibility or liability for the
 * use of the software, conveys no license or title under any patent,
 * copyright, or mask work right to the product. NXP Semiconductors
 * reserves the right to make changes in the software without
 * notification. NXP Semiconductors also make no representation or
 * warranty that such application will be suitable for the specified
 * use without further testing or modification.
****************************************************************************/
#include "LPC17xx.h"
#include "type.h"
#include "pwm.h"

volatile uint32_t match_counter0, match_counter1;

/******************************************************************************
** Function name:		PWM_Init
**
** Descriptions:		PWM initialization, setup all GPIOs to PWM0~6,
**						reset counter, all latches are enabled, interrupt
**						on PWMMR0, install PWM interrupt to the VIC table.
**
** parameters:			None
** Returned value:		true or false, if VIC table is full, return false
** 
******************************************************************************/
void PWM_Init()
{

	match_counter1 = 0;
	LPC_PINCON->PINSEL4 |= 0x00001555;	/* set GPIOs for all PWM pins on PWM */

	LPC_PWM1->TCR = TCR_RESET;	/* Counter Reset */ 
	LPC_PWM1->PR = 0x00;		/* count frequency:Fpclk */


	LPC_PWM1->MR0 = PWM_CYCLE;		/* set PWM cycle (frequency for all PWM pins) */
	LPC_PWM1->MR1 = PWM_CYCLE;
	LPC_PWM1->MR2 = PWM_CYCLE;
	LPC_PWM1->MR3 = PWM_CYCLE;
	LPC_PWM1->MR4 = PWM_CYCLE;
	LPC_PWM1->MR5 = PWM_CYCLE;
	LPC_PWM1->MR6 = PWM_CYCLE;

	/* all PWM latch enabled */

	LPC_PWM1->LER |= LER0_EN | LER1_EN | LER2_EN | LER3_EN | LER4_EN | LER5_EN | LER6_EN;

}

/******************************************************************************
** Function name:		PWM_Set
**
** Descriptions:		PWM cycle setup
**
** parameters:			Channel number, PWM cycle
** Returned value:		None
** 
******************************************************************************/
void PWM_Set( int ChannelNum, float speed)
{
	/*o ciclo é invertido, portanto para speed = 0, velocidade máxima e speed = 100, parado*/

	speed = (100 - speed)/100;
	switch(ChannelNum){
	case 0:
		LPC_PWM1->MR1 = PWM_CYCLE*speed;
		LPC_PWM1->PCR |= PWMENA1;
		break;
	case 1:
		LPC_PWM1->MR2 = PWM_CYCLE*speed;
		LPC_PWM1->PCR |= PWMENA2;
		break;
	case 2:
		LPC_PWM1->MR3 = PWM_CYCLE*speed;
		LPC_PWM1->PCR |= PWMENA3;
		break;
	case 3:
		LPC_PWM1->MR4 = PWM_CYCLE*speed;
		LPC_PWM1->PCR |= PWMENA4;
		break;
	case 4:
		LPC_PWM1->MR5 = PWM_CYCLE*speed;
		LPC_PWM1->PCR |= PWMENA5;
		break;
	case 5:
		LPC_PWM1->MR6 = PWM_CYCLE*speed;
		LPC_PWM1->PCR |= PWMENA6;
		break;
	default:
		break;
	}

	/* The LER will be cleared when the Match 0 takes place, in order to
	load and execute the new value of match registers, all the PWMLERs need to
	reloaded. all PWM latch enabled */
	LPC_PWM1->LER |= LER0_EN | LER1_EN | LER2_EN | LER3_EN | LER4_EN | LER5_EN | LER6_EN;

}

/******************************************************************************
** Function name:		PWM_Start
**
** Descriptions:		Enable PWM by setting the PCR, PTCR registers
**
** parameters:			None
** Returned value:		None
** 
******************************************************************************/
void PWM_Start( )
{

	LPC_PWM1->LER |= LER0_EN | LER1_EN | LER2_EN | LER3_EN | LER4_EN | LER5_EN | LER6_EN;

	LPC_PWM1->TCR = TCR_CNT_EN | TCR_PWM_EN;	/* counter enable, PWM enable */

  return;
}

/******************************************************************************
** Function name:		PWM_Stop
**
** Descriptions:		Stop all PWM channels
**
** parameters:			channel number
** Returned value:		None
** 
******************************************************************************/
void PWM_Stop( )
{

	LPC_PWM1->PCR = 0;
	LPC_PWM1->TCR = 0x00;		/* Stop all PWMs */

  return;
}

/******************************************************************************
**                            End Of File
******************************************************************************/
