/**
  \file spi.c

  \author G. Icking-Konert
  \date 2013-11-22
  \version 0.1

  \brief implementation of SPI functions/macros

  implementation of functions for SPI communication
  For SPI bus, see https://en.wikipedia.org/wiki/Serial_Peripheral_Interface_Bus


  2017-02-25 al177 - hack and slash to make SPI master MSB only
  */

/*-----------------------------------------------------------------------------
  INCLUDE FILES
  -----------------------------------------------------------------------------*/
#include <stdint.h>
#include "stm8as.h"
#include "spi.h"


/*----------------------------------------------------------
  FUNCTIONS
  ----------------------------------------------------------*/

/**
  \fn void spi_config(uint8_t pre, uint8_t phase, uint8_t idle)

  \brief configure SPI interface in master mode

  \param[in]  pre       baudrate prescaler (BR=16MHz/2^(pre+1))
  \param[in]  phase     clock phase (uC samples on falling(=0) or rising(=1) edge)
  \param[in]  idle      idle clock polarity low(=0) or high(=1)

  configure SPI interface in master mode for 4-wire communication

*/
void spi_config(uint8_t pre, uint8_t phase, uint8_t idle) {

	// disable SPI module
	SPI.CR1.reg.SPE = 0;


	// configure SPI for master mode
	SPI.CR2.reg.SSM  = 1;
	SPI.CR2.reg.SSI  = 1;
	SPI.CR1.reg.MSTR = 1;

	// configure PORT_C5(=SCK) as output
	PORT_C.ODR.bit.b5 = 1;   // default = high
	PORT_C.DDR.bit.b5 = 1;   // input(=0) or output(=1)
	PORT_C.CR1.bit.b5 = 1;   // input: 0=float, 1=pull-up; output: 0=open-drain, 1=push-pull
	PORT_C.CR2.bit.b5 = 1;   // input: 0=no exint, 1=exint; output: 0=2MHz slope, 1=10MHz slope  



	// set SPI type: 4-wire full-duplex(=0) or 3-wire half duplex(=1)
	SPI.CR2.reg.BDM = 0;

	PORT_C.CR2.bit.b6 = 1; 

	// set baudrate (=16MHz/2^(pre+1))
	SPI.CR1.reg.BR = (uint8_t) pre;

	// bit ordering (0=MSB first, 1=LSB first)
	SPI.CR1.reg.LSBFIRST = 0; 

	// set idle SCK polarity (0=low / 1=high)
	SPI.CR1.reg.CPOL = idle;

	// uC samples on falling (phase=0) or rising (phase=1) edge
	// -> STM8 convention: data CAPTURE edge (0=first clock transistion / 1=second clock transition)
	if (idle) {           // idle clock high
		if (phase)            // rising edge
			SPI.CR1.reg.CPHA = 1;
		else                  // falling edge
			SPI.CR1.reg.CPHA = 0;
	}
	else {                // idle clock low
		if (phase)            // rising edge
			SPI.CR1.reg.CPHA = 0;
		else                  // falling edge
			SPI.CR1.reg.CPHA = 1;
	}

	// Select SPI master mode
	SPI.CR1.reg.MSTR = 1;  

	// configure NSS control to SW-Mode
	SPI.CR2.reg.SSM = 1;
	SPI.CR2.reg.SSI = 1;

	// enable SPI module
	SPI.CR1.reg.SPE = 1;

} // spi_config


/**
  \fn void spi_send_receive(uint8_t numTx, uint8_t *buf)

  \brief send/receive via SPI in master mode

  \param[in]  numTx      number of bytes to send per frame
  \param[in]  buf        array of bytes [0..MAX_WIDTH_SPI-1] for MOSI and MISO

*/
void spi_send_receive(uint8_t numTx, uint8_t *buf) {

	uint8_t		i;

	// wait until not busy
	while (SPI.SR.reg.BSY);

	// disable interrupts to prevent timing issue
	DISABLE_INTERRUPTS;


	// configure PORT_C5(=SCK) as output
	PORT_C.ODR.bit.b5 = 1;   // init outputs to low, except for DAC update 
	PORT_C.DDR.bit.b5 = 1;   // input(=0) or output(=1)
	PORT_C.CR1.bit.b5 = 1;   // input: 0=float, 1=pull-up; output: 0=open-drain, 1=push-pull
	PORT_C.CR2.bit.b5 = 1;   // input: 0=no exint, 1=exint; output: 0=2MHz slope, 1=10MHz slope


	// clear MISO buffer
	i = SPI.DR.byte;
	i = 0;
	// loop over numTx bytes
	while (i != numTx) {

		// fill MOSI buffer when available
		if ((SPI.SR.reg.TXE) && (i<numTx)) {
			SPI.DR.byte = buf[i];
			_NOP_;
		}

		// read MISO buffer when available
		if (SPI.SR.reg.RXNE) {
			buf[i] = SPI.DR.byte;
			_NOP_;
		}
		i++;

	} // send/receive loop

	// re-enable interrupts
	ENABLE_INTERRUPTS;

	// wait until not busy
	while (SPI.SR.reg.BSY);

} // spi_send_receive



	/*-----------------------------------------------------------------------------
	  END OF MODULE
	  -----------------------------------------------------------------------------*/
