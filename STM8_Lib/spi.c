/**
  \file spi.c
   
  \author G. Icking-Konert
  \date 2013-11-22
  \version 0.1
   
  \brief implementation of SPI functions/macros
   
  implementation of functions for SPI communication
  For SPI bus, see https://en.wikipedia.org/wiki/Serial_Peripheral_Interface_Bus
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
  \fn void spi_config(uint8_t type, uint16_t timeout, uint8_t plex, uint8_t pre, uint8_t order, uint8_t phase, uint8_t idle)
   
  \brief configure SPI interface in master mode

  \param[in]  type      SPI master(=1) or slave(=0)
  \param[in]  timeout   frame timeout [ms] for slave mode (or none if 0)
  \param[in]  plex      SPI type: 4-wire full-duplex(=0) or 3-wire half duplex(=1)
  \param[in]  pre       baudrate prescaler (BR=16MHz/2^(pre+1))
  \param[in]  order     bit order: MSB(=0) or LSB(=1) first
  \param[in]  phase     clock phase (uC samples on falling(=0) or rising(=1) edge)
  \param[in]  idle      idle clock polarity low(=0) or high(=1)

  configure SPI interface in master mode for 3- or 4-wire communication
  
*/
void spi_config(uint8_t type, uint16_t timeout, uint8_t plex, uint8_t pre, uint8_t order, uint8_t phase, uint8_t idle) {

  // disable SPI module
  SPI_CR1.reg.SPE = 0;

  // SPI master mode
  if (type==PRM_SPI_MASTER) {
    
    // configure SPI for master mode
    SPI_CR2.reg.SSM  = 1;
    SPI_CR2.reg.SSI  = 1;
    SPI_CR1.reg.MSTR = 1;
    
    // configure PC5(=SCK) as output
    PC_ODR.bit.b5 = 1;   // default = high
    PC_DDR.bit.b5 = 1;   // input(=0) or output(=1)
    PC_CR1.bit.b5 = 1;   // input: 0=float, 1=pull-up; output: 0=open-drain, 1=push-pull
    PC_CR2.bit.b5 = 1;   // input: 0=no exint, 1=exint; output: 0=2MHz slope, 1=10MHz slope  
    
  } // SPI master
  
  // SPI slave mode
  else {
    
    // configure SPI for slave mode
    SPI_CR2.reg.SSM  = 1;
    SPI_CR2.reg.SSI  = 0;
    SPI_CR1.reg.MSTR = 0;
    
    // configure PC5(=SCK) as input
    PC_CR2.bit.b5 = 0;   // input: 0=no exint, 1=exint; output: 0=2MHz slope, 1=10MHz slope
    PC_DDR.bit.b5 = 0;   // input(=0) or output(=1)
    PC_CR1.bit.b5 = 1;   // input: 0=float, 1=pull-up; output: 0=open-drain, 1=push-pull

  } // SPI slave
  

  // set SPI type: 4-wire full-duplex(=0) or 3-wire half duplex(=1)
  SPI_CR2.reg.BDM = plex;
 
  // for half-duplex set CR_2 of MOSI to 0 to disable interrupts in input mode (see ref. manual)
  if (SPI_CR2.reg.BDM == PRM_SPI_HALF_DUPLEX)
    PC_CR2.bit.b6 = 0; 
  else
    PC_CR2.bit.b6 = 1; 
  
  // set baudrate (=16MHz/2^(pre+1))
  SPI_CR1.reg.BR = (uint8_t) pre;
 
  // bit ordering (0=MSB first, 1=LSB first)
  SPI_CR1.reg.LSBFIRST = order; 

  // set idle SCK polarity (0=low / 1=high)
  SPI_CR1.reg.CPOL = idle;

  // uC samples on falling (phase=0) or rising (phase=1) edge
  // -> STM8 convention: data CAPTURE edge (0=first clock transistion / 1=second clock transition)
  if (idle) {           // idle clock high
    if (phase)            // rising edge
      SPI_CR1.reg.CPHA = 1;
    else                  // falling edge
      SPI_CR1.reg.CPHA = 0;
  }
  else {                // idle clock low
    if (phase)            // rising edge
      SPI_CR1.reg.CPHA = 0;
    else                  // falling edge
      SPI_CR1.reg.CPHA = 1;
  }
  
  // Select SPI master mode
  SPI_CR1.reg.MSTR = 1;  

  // configure NSS control to SW-Mode
  SPI_CR2.reg.SSM = 1;
  SPI_CR2.reg.SSI = 1;
  
  // enable SPI module
  SPI_CR1.reg.SPE = 1;
  
} // spi_config


/**
  \fn void spi_send_receive(uint8_t csn, uint8_t numTx, uint8_t *MOSI, uint8_t numRx, uint8_t *MISO)
   
  \brief send/receive via SPI in master mode

  \param[in]  csn        GPIO used as CSN (1..16=io_x; else none)
  \param[in]  numTx      number of bytes to send per frame
  \param[in]  MOSI       array of MOSI bytes [0..MAX_WIDTH_SPI-1]
  \param[in]  numRx      number of bytes to receive per frame (=numTx for full-duplex)
  \param[out] MISO       array of MISO bytes [0..MAX_WIDTH_SPI-1]

  send/receive via SPI in master mode i.e. CSN control is by muBoard
  
*/
void spi_send_receive(uint8_t csn, uint8_t numTx, uint8_t *MOSI, uint8_t numRx, uint8_t *MISO) {
  
  uint8_t		i, j;
    
  // wait until not busy
  while (SPI_SR.reg.BSY);
  
  // disable interrupts to prevent timing issue
  DISABLE_INTERRUPTS;
       
       
  // configure PC5(=SCK) as output
  PC_ODR.bit.b5 = 1;   // init outputs to low, except for DAC update 
  PC_DDR.bit.b5 = 1;   // input(=0) or output(=1)
  PC_CR1.bit.b5 = 1;   // input: 0=float, 1=pull-up; output: 0=open-drain, 1=push-pull
  PC_CR2.bit.b5 = 1;   // input: 0=no exint, 1=exint; output: 0=2MHz slope, 1=10MHz slope


  // full-duplex mode (= 4-wire)
  if (SPI_CR2.reg.BDM == PRM_SPI_FULL_DUPLEX) {
  
    // send/receive MSB first
    if (SPI_CR1.reg.LSBFIRST == PRM_SPI_MSB_FIRST) {                    
      
      // clear MISO buffer
      i = SPI_DR.byte;
      
      // init array indices
      i = j = 0;
      
      // pull CSN low
      if ((csn>1) && (csn<=16))
        clear_gpio(csn);
      
      // loop over numTx bytes
      while (j != numTx) {
        
        // fill MOSI buffer when available
        if ((SPI_SR.reg.TXE) && (i<numTx)) {
          SPI_DR.byte = MOSI[i++];
          _NOP_;
        }
        
        // read MISO buffer when available
        if (SPI_SR.reg.RxNE) {
          MISO[j++] = SPI_DR.byte;
          _NOP_;
        }

      } // send/receive loop
      
      // pull CSN high (255 -> do nothing)
      if ((csn>1) && (csn<=16))
        set_gpio(csn);
            
    } // MSB first
    
    // send/receive LSB first
    else {                    
      
      // clear MISO buffer
      i = SPI_DR.byte;
      
      // init array indices
      i = j = numTx;
    
      // pull CSN low (255 -> do nothing)
      if ((csn>1) && (csn<=16))
        clear_gpio(csn);
      
      // loop over numTx bytes
      while (j > 0) {
        
        // fill MOSI buffer when available
        if ((SPI_SR.reg.TXE) && (i>0)) {
          SPI_DR.byte = MOSI[i-1];
					i--;
					_NOP_;
				}
          
        // read MISO buffer when available
        if (SPI_SR.reg.RxNE) {
          MISO[j-1] = SPI_DR.byte;
					j--;
					_NOP_;
				}
      
      } // send/receive loop
            
      // pull CSN high (255 -> do nothing)
      if ((csn>1) && (csn<=16))
        set_gpio(csn);

    } // LSB first
    
  } // full-duplex mode
  
  
  // half duplex mode (= 3-wire)
  else {
  
    // send/receive MSB first
    if (SPI_CR1.reg.LSBFIRST == PRM_SPI_MSB_FIRST) {                    
      
      // pull CSN low (255 -> do nothing)
      if ((csn>1) && (csn<=16))
        clear_gpio(csn);
      
      // switch MOSI to output
      while (SPI_SR.reg.BSY);
      SPI_CR2.reg.BDOE = 1;   

      // send data
      for (i=0; i<numTx; i++) {
        while (!SPI_SR.reg.TXE);                  // wait for transmit buffer empty
        SPI_DR.byte = MOSI[i];
      }      
      
      // receive (if specified)
      if (numRx > 0) {
      
        // after sending done, switch MOSI to input (with pull-up)
        while (SPI_SR.reg.BSY);
        PC_DDR.bit.b6   = 0;   // input(=0) or output(=1)
				PC_CR1.bit.b6 = 1;   	// input: 0=float, 1=pull-up
				PC_CR2.bit.b6 = 0;   	// input: 0=no exint, 1=exint
				SPI_CR2.reg.BDOE = 0;   // switch SPI direction of Data pin
    
        // receive data
        for (i=0; i<numRx; i++) {
          while (!SPI_SR.reg.RxNE);                 // wait for receive buffer not empty
          MISO[i] = SPI_DR.byte;
        }
        
      } // numRx > 0
      
      // pull CSN high (255 -> do nothing)
      while (SPI_SR.reg.BSY);
      if ((csn>1) && (csn<=16))
        clear_gpio(csn);

    } // half-duplex, MSB first
    
    
    // send/receive LSB first
    else {                    

      // pull CSN low (255 -> do nothing)
      if ((csn>1) && (csn<=16))
        clear_gpio(csn);

      // switch MOSI to output
      while (SPI_SR.reg.BSY);
      SPI_CR2.reg.BDOE = 1;   

      // send data
      for (i=numTx; i>0; i--) {
        while (!SPI_SR.reg.TXE);                  // wait for transmit buffer empty
        SPI_DR.byte = MOSI[i-1];
      }      
      
      // receive (if specified)
      if (numRx > 0) {
      
        // when done, switch MOSI to input (with pull-up)
        while (SPI_SR.reg.BSY);
        PC_DDR.bit.b6   = 0;   // input(=0) or output(=1)
				PC_CR1.bit.b6   = 1;   // input: 0=float, 1=pull-up
				PC_CR2.bit.b6   = 0;   // input: 0=no exint, 1=exint
        SPI_CR2.reg.BDOE = 0;   
    
        // wait a bit to allow for command decoding in slave
        //sleep_us(1);
        
        // receive data
        for (i=numRx; i>0; i--) {
          while (!SPI_SR.reg.RxNE);                 // wait for receive buffer not empty
          MISO[i-1] = SPI_DR.byte;
        }
        
      } // numRx > 0
      
      // pull CSN high (255 -> do nothing)
      if ((csn>1) && (csn<=16))
        set_gpio(csn);

    } // half-duplex, LSB first
     
    // when done, switch MOSI back to output
    while (SPI_SR.reg.BSY);
    SPI_CR2.reg.BDOE = 1;   
    PC_DDR.bit.b6   = 1;   // input(=0) or output(=1)
		PC_CR1.bit.b6   = 1;   // output: 0=open-drain, 1=push-pull
		PC_CR2.bit.b6   = 1;   // output: 0=2MHz slope, 1=10MHz slope
 
  } // half duplex mode

  // re-enable interrupts
  ENABLE_INTERRUPTS;
  
  // wait until not busy
  while (SPI_SR.reg.BSY);
  
  // wait a bit to guarantee a min. CSN=1 time
  sleep_us(10);
      
} // spi_send_receive

 

/**
  \fn void spi_ISR(void)
   
  \brief ISR for SPI_ISR bus (required for slave operation)
  
  Interrupt service routine for SPI_ISR bus (required for operation as slave).
  
*/
#if defined(__CSMC__)
  @near @interrupt void spi_ISR(void) {
#elif defined(__SDCC)
  void spi_ISR() __interrupt(__SPI_ISR_VECTOR__) {
#endif
  
  // dummy
  // check different SPI status bits and react accordingly

  return;

} // spi_ISR


/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/
