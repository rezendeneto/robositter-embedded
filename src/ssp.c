/****************************************************************************
 *   $Id:: ssp.c 5804 2010-12-04 00:32:12Z usb00423                         $
 *   Project: NXP LPC17xx SSP example
 *
 *   Description:
 *     This file contains SSP code example which include SSP initialization, 
 *     SSP interrupt handler, and APIs for SSP access.
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
#include "LPC17xx.h"			/* LPC17xx Peripheral Registers */
#include "ssp.h"
#include "nRF24L01.h"
#include <stdlib.h>

/* statistics of all the interrupts */
volatile uint32_t interrupt0RxStat = 0;
volatile uint32_t interrupt0OverRunStat = 0;
volatile uint32_t interrupt0RxTimeoutStat = 0;
volatile uint32_t interrupt1RxStat = 0;
volatile uint32_t interrupt1OverRunStat = 0;
volatile uint32_t interrupt1RxTimeoutStat = 0;


 uint8_t channel = 1;
 uint8_t payload = 3;
 uint8_t PTX = 0;
// uint8_t rx[4];
/*****************************************************************************
** Function name:		SSP_IRQHandler
**
** Descriptions:		SSP port is used for SPI communication.
**						SSP interrupt handler
**						The algorithm is, if RXFIFO is at least half full, 
**						start receive until it's empty; if TXFIFO is at least
**						half empty, start transmit until it's full.
**						This will maximize the use of both FIFOs and performance.
**
** parameters:			None
** Returned value:		None
** 
*****************************************************************************/
void SSP0_IRQHandler(void) 
{
  uint32_t regValue;

  regValue = LPC_SSP0->MIS;
  if ( regValue & SSPMIS_RORMIS )	/* Receive overrun interrupt */
  {
	interrupt0OverRunStat++;
	LPC_SSP0->ICR = SSPICR_RORIC;		/* clear interrupt */
  }
  if ( regValue & SSPMIS_RTMIS )	/* Receive timeout interrupt */
  {
	interrupt0RxTimeoutStat++;
	LPC_SSP0->ICR = SSPICR_RTIC;		/* clear interrupt */
  }

  /* please be aware that, in main and ISR, CurrentRxIndex and CurrentTxIndex
  are shared as global variables. It may create some race condition that main
  and ISR manipulate these variables at the same time. SSPSR_BSY checking (polling)
  in both main and ISR could prevent this kind of race condition */
  if ( regValue & SSPMIS_RXMIS )	/* Rx at least half full */
  {
	interrupt0RxStat++;		/* receive until it's empty */		
  }
  return;
}

void csnLow(){
	  LPC_GPIO0->FIOCLR |= SSP0_SEL;
}
void csnHi(){
	  LPC_GPIO0->FIOSET |= SSP0_SEL;
}
void ceLow(){
	  LPC_GPIO0->FIOCLR |= CE;

}
void ceHi(){
	  LPC_GPIO0->FIOSET |= CE;
}


/*****************************************************************************
** Function name:		SSPInit
**
** Descriptions:		SSP port initialization routine
**				
** parameters:			None
** Returned value:		None
** 
*****************************************************************************/
void SSP0Init( void )
{
  uint8_t i, Dummy=Dummy;

  /* Enable AHB clock to the SSP0. */
  LPC_SC->PCONP |= (0x1<<21);

  /* Further divider is needed on SSP0 clock. Using default divided by 4 */
  LPC_SC->PCLKSEL1 &= ~(0x3<<10);

  /* P0.15~0.18 as SSP0 */
  LPC_PINCON->PINSEL0 &= ~(0x3UL<<30);
  LPC_PINCON->PINSEL0 |= (0x2UL<<30);
  LPC_PINCON->PINSEL1 &= ~((0x3<<0)|(0x3<<2)|(0x3<<4));
  LPC_PINCON->PINSEL1 |= ((0x2<<0)|(0x2<<2)|(0x2<<4));
  
  LPC_GPIO0->FIODIR &= ~(nIRQ_PIN);//p0.22 nIRQ como input
  LPC_GPIOINT->IO0IntEnF |= nIRQ_PIN;

#if !USE_CS
  LPC_PINCON->PINSEL1 &= ~(0x3<<0);
  LPC_GPIO0->FIODIR |= (0x1<<16);		/* P0.16 defined as GPIO and Outputs */
#endif
		
  /* Set DSS data to 8-bit, Frame format SPI, CPOL = 0, CPHA = 0, and SCR is 15 */
  LPC_SSP0->CR0 = 0x0707;

  /* SSPCPSR clock prescale register, master mode, minimum divisor is 0x02 */
  LPC_SSP0->CPSR = 0x2;

  for ( i = 0; i < FIFOSIZE; i++ )
  {
	Dummy = LPC_SSP0->DR;		/* clear the RxFIFO */
  }

  LPC_GPIO0->FIODIR |= CE;
  /* Enable the SSP Interrupt */
 // NVIC_EnableIRQ(SSP0_IRQn);
	
  /* Device select as master, SSP Enabled */

  /* Master mode */
  LPC_SSP0->CR1 = SSPCR1_SSE;

  return;
}

/*****************************************************************************
** Function name:		SSPSend
**
** Descriptions:		Send a block of data to the SSP port, the 
**						first parameter is the buffer pointer, the 2nd 
**						parameter is the block length.
**
** parameters:			buffer pointer, and the block length
** Returned value:		None
** 
*****************************************************************************/
void SSPSend( uint8_t *buf, uint32_t Length )
{
  uint32_t i;
  uint8_t Dummy = Dummy;
    
  for ( i = 0; i < Length; i++ )
  {
      /* Move on only if NOT busy and TX FIFO not full. */
	  while ( (LPC_SSP0->SR & (SSPSR_TNF|SSPSR_BSY)) != SSPSR_TNF );
	  LPC_SSP0->DR = *buf ;
	  buf++;
#if !LOOPBACK_MODE
	  while ( (LPC_SSP0->SR & (SSPSR_BSY|SSPSR_RNE)) != SSPSR_RNE );
	  /* Whenever a byte is written, MISO FIFO counter increments, Clear FIFO 
	  on MISO. Otherwise, when SSP0Receive() is called, previous data byte
	  is left in the FIFO. */
	  Dummy = LPC_SSP0->DR;
#else
	  /* Wait until the Busy bit is cleared. */
	  while ( LPC_SSP0->SR & SSPSR_BSY );
#endif

  }
  return; 
}

/*****************************************************************************
** Function name:		SSPReceive
** Descriptions:		the module will receive a block of data from 
**						the SSP, the 2nd parameter is the block 
**						length.
** parameters:			buffer pointer, and block length
** Returned value:		None
** 
*****************************************************************************/
void SSPReceive( uint8_t *buf, uint32_t Length )
{
  uint32_t i;
 
  for ( i = 0; i < Length; i++ )
  {
	  /* As long as Receive FIFO is not empty, I can always receive. */
	  /* If it's a loopback test, clock is shared for both TX and RX,
	no need to write dummy byte to get clock to get the data */
	  /* if it's a peer-to-peer communication, SSPDR needs to be written
	before a read can take place. */

#if !LOOPBACK_MODE
#if SSP_SLAVE
	  while ( !(LPC_SSP0->SR & SSPSR_RNE) );
#else
	  LPC_SSP0->DR = 0xFF;
	  /* Wait until the Busy bit is cleared */
	  while ( (LPC_SSP0->SR & (SSPSR_BSY|SSPSR_RNE)) != SSPSR_RNE );
#endif
#else
	  while ( !(LPC_SSP0->SR & SSPSR_RNE) );
#endif
	  *buf++ = LPC_SSP0->DR;
  }


  return; 
}

void writeRegister(uint8_t reg, uint8_t * value, uint8_t len){
	uint8_t* rx = (uint8_t*)malloc(sizeof(uint8_t)*(len+1));
	rx[0] = W_REGISTER | (REGISTER_MASK & reg);
	uint8_t a;
	for(a = 1; a < len+1; a++){
		rx[a] = value[a-1];
	}
	csnLow();
	free(rx);
	SSPSend(rx, len+1);
	csnHi();

}

void setTADDR(uint8_t * adr)
// Sets the transmitting address
{
	/*
	 * RX_ADDR_P0 must be set to the sending addr for auto ack to work.
	 */

	writeRegister(RX_ADDR_P0,adr,mirf_ADDR_LEN);
	writeRegister(TX_ADDR,adr,mirf_ADDR_LEN);
}

void setRADDR(uint8_t * adr)
// Sets the receiving address
{
	ceLow();
	writeRegister(RX_ADDR_P1,adr,mirf_ADDR_LEN);
	ceHi();
}

void config_rf(){
	// Sets the important registers in the MiRF module and powers the module
	// in receiving mode
	// NB: channel and payload must be set now.

	/* set tranceiver address */
	setRADDR((uint8_t*)"robot");

	//uint8_t result = 0x07;
	uint8_t result = 0x27;
	writeRegister(RF_SETUP,&result,1);//1mbps

	// Set RF channel
	writeRegister(RF_CH,&channel,1);

	// Set length of incoming payload
	writeRegister(RX_PW_P0, &payload,1);
	writeRegister(RX_PW_P1, &payload,1);

	// Start receiver
	powerUpRx();
	flushRx();
}

void powerUpRx(){
	PTX = 0;
	ceLow();
	uint8_t result = mirf_CONFIG | ( (1<<PWR_UP) | (1<<PRIM_RX));
	result |= (1<<MASK_MAX_RT) | (1<<MASK_TX_DS); //ativa nIQR apenas quando tem algo para receber (tx mode)
	writeRegister(CONFIG, &result,1);
	ceHi();
	result = (1 << TX_DS) | (1 << MAX_RT);
	writeRegister(STATUS,&result,1);
}

void flushRx(){
	uint8_t result = FLUSH_RX;
	csnLow();
	SSPSend(&result, 1);
	csnHi();
}
void readRegister(uint8_t reg, uint8_t * value, uint8_t len){

	csnLow();
	SSPSend(&reg, 1);
	SSPReceive((uint8_t *)value, len);
	csnHi();
}
uint8_t getStatus(){
	uint8_t rv;
	readRegister(STATUS,&rv,1);
	return rv;
}

void send_rf(uint8_t * value)
// Sends a data package to the default address. Be sure to send the correct
// amount of bytes as configured as payload on the receiver.
{
    uint8_t status;
    status = getStatus();
    while (PTX) {
	    status = getStatus();
	    if((status & ((1 << TX_DS)  | (1 << MAX_RT)))){
		    PTX = 0;
		    break;
	    }
    }                  // Wait until last paket is send

    ceLow();

    powerUpTx();       // Set to transmitter mode , Power up

    csnLow();
    uint8_t result = FLUSH_TX;
    SSPSend(&result, 1);
    csnHi();


    uint8_t* rx = (uint8_t*)malloc(sizeof(uint8_t)*(payload+1));
    rx[0] = W_TX_PAYLOAD;
    uint8_t a;
    for(a = 1; a < payload+1; a++){
    	rx[a] = value[a-1];
    }
    csnLow();
    SSPSend(rx, payload+1);
    free(rx);
    csnHi();

    ceHi();                     // Start transmission
    while(isSending());
}


void powerUpTx(){
	PTX = 1;
	uint8_t result = mirf_CONFIG | ( (1<<PWR_UP) | (0<<PRIM_RX) );
	writeRegister(CONFIG, &result, 1);
}

uint8_t isSending(){
	uint8_t status;
	if(PTX){
		status = getStatus();

		/*
		 *  if sending successful (TX_DS) or max retries exceded (MAX_RT).
		 */

		if((status & ((1 << TX_DS)  | (1 << MAX_RT)))){
			powerUpRx();
			return 0;
		}

		return 1;
	}
	return 0;
}

uint8_t dataReady()
// Checks if data is available for reading
{
    // See note in getData() function - just checking RX_DR isn't good enough
	uint8_t status = getStatus();

    // We can short circuit on RX_DR, but if it's not set, we still need
    // to check the FIFO for any pending packets
    if ( status & (1 << RX_DR) ) return 1;
    return !rxFifoEmpty();
}

uint8_t rxFifoEmpty(){
	uint8_t fifoStatus;
	readRegister(FIFO_STATUS, &fifoStatus, 1);

	return (fifoStatus & (1 << RX_EMPTY));
}

void getData(uint8_t * data)
// Reads payload bytes into data array
{
	csnLow();
	//SSPSend(R_RX_PAYLOAD,1);
	//for(a = 0; a < payload; a++){
	readRegister(R_RX_PAYLOAD,data,payload);
	//}
	csnHi();

    // NVI: per product spec, p 67, note c:
    //  "The RX_DR IRQ is asserted by a new packet arrival event. The procedure
    //  for handling this interrupt should be: 1) read payload through SPI,
    //  2) clear RX_DR IRQ, 3) read FIFO_STATUS to check if there are more
    //  payloads available in RX FIFO, 4) if there are more data in RX FIFO,
    //  repeat from step 1)."
    // So if we're going to clear RX_DR here, we need to check the RX FIFO
    // in the dataReady() function
	uint8_t result = (1<<RX_DR);
  	writeRegister(STATUS, &result, 1);   // Reset status register
}

/******************************************************************************
**                            End Of File
******************************************************************************/

