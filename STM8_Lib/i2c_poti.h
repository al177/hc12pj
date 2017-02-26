/**
  \file i2c_poti.h
   
  \author G. Icking-Konert
  \date 2013-11-22
  \version 0.1
   
  \brief declaration of I2C digital potentiometer functions/macros
   
  declaration of functions for controlling a 20kR / 8-bit digital 
  potentiometer AD5280BRUZ20 (Farnell 1438441).
  Connect poti I2C bus to STM8 SCL/SDA
*/

/*-----------------------------------------------------------------------------
    MODULE DEFINITION FOR MULTIPLE INCLUSION
-----------------------------------------------------------------------------*/
#ifndef _I2C_POTI_H_
#define _I2C_POTI_H_


/*-----------------------------------------------------------------------------
    DEFINITION OF GLOBAL MACROS/#DEFINES
-----------------------------------------------------------------------------*/

#include <stdint.h>

// I2C addresses of digital potentiometer
#define ADDR_I2C_POTI 46


/*-----------------------------------------------------------------------------
    DECLARATION OF GLOBAL FUNCTIONS
-----------------------------------------------------------------------------*/

/// set resistance of potentiometer
uint8_t     set_dig_poti(uint8_t res);


/*-----------------------------------------------------------------------------
    END OF MODULE DEFINITION FOR MULTIPLE INLUSION
-----------------------------------------------------------------------------*/
#endif // _I2C_POTI_H_
