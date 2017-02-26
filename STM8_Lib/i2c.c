/**
  \file i2c.h
   
  \author G. Icking-Konert
  \date 2013-11-22
  \version 0.1
   
  \brief implementation of I2C functions/macros
   
  implementation of functions for I2C bus communication
  For I2C bus, see http://en.wikipedia.org/wiki/I2C
*/

/*-----------------------------------------------------------------------------
    INCLUDE FILES
-----------------------------------------------------------------------------*/
#include <stdint.h>
#include "stm8as.h"
#include "gpio.h"
#include "i2c.h"
#include "timer3.h"
#include "error_codes.h"


/*----------------------------------------------------------
    FUNCTIONS
----------------------------------------------------------*/

/**
  \fn void i2c_init(void)
   
  \brief configure I2C bus

  configure I2C bus as master in standard mode, BR=100kHz, 7bit address, 250ns rise time
  
*/
void i2c_init() {

  // configure I2C pins PE1(=SCL) and PE2(=SDA)
  gpio_init(&PORT_E, PIN_1 | PIN_2, OUTPUT_OPENDRAIN_FAST);

  // init I2C bus
  I2C.CR1.byte          = 0x00;     // disable I2C, no clock stretching
  I2C.CR2.byte          = 0x00;     // reset I2C bus
  I2C.CR2.reg.ACK       = 1;        // enable ACK on address match
  I2C.FREQR.reg.FREQ    = 16;       // peripheral clock = 16MHz (f=val*1MHz)
  I2C.OARH.reg.ADDCONF  = 1;        // set 7b addressing mode
  I2C.OARL.reg.ADD      = 0x04;     // set own 7b addr to 0x04
  I2C.SR1.byte          = 0x00;     // clear status registers
  I2C.SR2.byte          = 0x00;
  I2C.SR3.byte          = 0x00;
  I2C.CCRL.reg.CCR      = 0x50;     // BR = 100kBaud (t_low = t_high = 80/16MHz)
  I2C.CCRH.byte         = 0x00;     // I2C standard mode
  I2C.TRISER.reg.TRISE  = 0x04;     // t_rise<250ns (<300ns for display driver)
  I2C.CR1.reg.PE        = 1;        // enable I2C module with broadcast receive disabled

} // i2c_init



/**
  \fn uint8_t i2c_start(void)
   
  \brief generate I2C start condition

  \return error code

  generate I2C start condition with 1ms timeout
  
*/
uint8_t i2c_start() {

  // start overall timeout
  start_timeout_ms(1);                                        

  // generate start condition
  I2C.CR2.reg.START = 1;
  while ((!I2C.SR1.reg.SB) && (!check_timeout()));       // wait for start condition generated or timeout
 
  // on I2C timeout set error flag
  if (check_timeout()) {
    stop_timeout_ms();
    return(ERROR_TIMOUT);
  }

  // reset timeout timer
  stop_timeout_ms(); 

  return(SUCCESS);

} // i2c_start



/**
  \fn uint8_t i2c_stop(void)
   
  \brief generate I2C stop condition

  \return operation successful?

  generate I2C stop condition with 1ms timeout
  
*/
uint8_t i2c_stop() {

  // start overall timeout
  start_timeout_ms(1);                                        

  // generate stop condition
  I2C.CR2.reg.STOP = 1;
  while ((I2C.SR3.reg.MSL) && (!check_timeout()));       // wait for stop condition generated or timeout
 
  // on I2C timeout set error flag
  if (check_timeout()) {
    stop_timeout_ms();
    return(ERROR_TIMOUT);
  }

  // reset timeout timer
  stop_timeout_ms(); 
  
  return(SUCCESS);

} // i2c_stop



/**
  \fn uint8_t i2c_send(uint8_t addr, uint8_t numTx, uint8_t *bufTx)
   
  \brief write data via I2C

  \param[in]  addr        7b address [6:0] of I2C slave
  \param[in]  numTx       number of bytes to send
  \param[in]  bufTx       send buffer

  \return operation successful?

  write data via I2C with 5ms frame timeout. Note that no start or 
  stop condition is generated.
  
*/
uint8_t i2c_send(uint8_t addr, uint8_t numTx, uint8_t *bufTx) {

  uint8_t    i;
  
  // start overall timeout
  start_timeout_ms(5);                                        

  // send 7b slave adress [7:1] + write flag (LSB=0)
  I2C.DR.byte = (uint8_t) ((addr << 1) & ~0x01);                 // shift left and set LSB (write=0, read=1)
  while ((!I2C.SR1.reg.ADDR) && (!check_timeout()));     // wait until address sent or timeout
  while ((!I2C.SR3.reg.BUSY) && (!check_timeout()));     // seems to be required...???
  
  // send data
  if (!check_timeout()) {
    
    for (i=0; i<numTx; i++) {
      I2C.DR.byte = bufTx[i];
      while ((!I2C.SR1.reg.TXE) && (!check_timeout()));    // wait until DR buffer empty or timeout
    }
    
  }
 
  // on I2C timeout set error flag
  if (check_timeout()) {
    stop_timeout_ms();
    return(ERROR_TIMOUT);
  }

  // reset timeout timer
  stop_timeout_ms(); 

  return(SUCCESS);

} // i2c_send



/**
  \fn uint8_t i2c_request(uint8_t slaveAddr, uint8_t numRx, uint8_t *bufRx)
   
  \brief request data via I2C as master

  \param[in]  slaveAddr   7b address [6:0] of I2C slave
  \param[in]  numRx       number of bytes to receive
  \param[out] bufRx       receive buffer

  \return operation successful?

  request data from I2C slave with a 5ms frame timeout. Note that no start or stop condition is generated.
  
*/
uint8_t i2c_request(uint8_t slaveAddr, uint8_t numRx, uint8_t *bufRx) {

  uint8_t      i;
  
  // init receive buffer
  for (i=0; i<numRx; i++)
    bufRx[i] = 0;
    
  // start overall timeout
  start_timeout_ms(5);                                        

  // send 7b slave adress [7:1] + read flag (LSB=1)
  I2C.DR.byte = (uint8_t) ((slaveAddr << 1) | 0x01);        // shift left and set LSB (write=0, read=1)
  while ((!I2C.SR1.reg.ADDR) && (!check_timeout()));     // wait until adress sent or timeout
  while ((!I2C.SR3.reg.BUSY) && (!check_timeout()));     // seems to be required...???

  // receive data from slave
  for (i=0; i<numRx; i++) {
    while ((!I2C.SR1.reg.RXNE) && (!check_timeout()));   // wait until DR buffer not empty or timeout
    bufRx[i] = I2C.DR.byte;
  }
 
  // on I2C timeout set error flag
  if (check_timeout()) {
    stop_timeout_ms();
    return(ERROR_TIMOUT);
  }

  // reset timeout timer
  stop_timeout_ms(); 

  return(SUCCESS);
 
} // i2c_request
 
 

/**
  \fn void i2c_ISR(void)
   
  \brief ISR for I2C bus (required for slave operation)
  
  Interrupt service routine for I2C bus (required for operation as slave).
  
*/
#if defined(__CSMC__)
  @near @interrupt void i2c_ISR(void) {
#elif defined(__SDCC)
  void i2c_ISR() __interrupt(__I2C_VECTOR__) {
#endif
  
  // dummy
  // check different I2C status bits and react accordingly

  return;

} // i2c_ISR


/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/
