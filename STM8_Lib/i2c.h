/**
  \file i2c.h
   
  \author G. Icking-Konert
  \date 2013-11-22
  \version 0.1
   
  \brief declaration of I2C functions/macros
   
  declaration of functions for I2C bus communication
  For I2C bus, see http://en.wikipedia.org/wiki/I2C
*/

/*-----------------------------------------------------------------------------
    MODULE DEFINITION FOR MULTIPLE INCLUSION
-----------------------------------------------------------------------------*/
#ifndef _I2C_H_
#define _I2C_H_

#include <stdint.h>
#include "stm8as.h"


/*-----------------------------------------------------------------------------
    DEFINITION OF GLOBAL MACROS/#DEFINES
-----------------------------------------------------------------------------*/

/// check I2C busy flag
#define I2C_BUSY  (I2C.SR3.reg.BUSY)


/*-----------------------------------------------------------------------------
    DECLARATION OF GLOBAL FUNCTIONS
-----------------------------------------------------------------------------*/

/// configure I2C bus as master in standard mode
void      i2c_init(void);

/// generate I2C start condition
uint8_t   i2c_start(void);

/// generate I2C stop condition
uint8_t   i2c_stop(void);

/// write data via I2C
uint8_t   i2c_send(uint8_t addr, uint8_t numTx, uint8_t *Tx);

/// request data via I2C as master
uint8_t   i2c_request(uint8_t addr, uint8_t numRx, uint8_t *Rx);

/// ISR for I2C bus (required for slave operation)
#if defined(__CSMC__)
  @near @interrupt void i2c_ISR(void);
#elif defined(__SDCC)
  void i2c_ISR(void) __interrupt(__I2C_VECTOR__);
#endif


/*-----------------------------------------------------------------------------
    DEFINITION OF GLOBAL MACROS/#DEFINES
-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
    END OF MODULE DEFINITION FOR MULTIPLE INLUSION
-----------------------------------------------------------------------------*/
#endif // _I2C_H_
