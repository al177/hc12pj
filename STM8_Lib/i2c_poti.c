/**
  \file i2c_poti.c
   
  \author G. Icking-Konert
  \date 2013-11-22
  \version 0.1
   
  \brief implementation of I2C digital potentiometer functions/macros
   
  implementation of functions for controlling a 20kR / 8-bit digital 
  potentiometer AD5280BRUZ20 (Farnell 1438441).
  Connect poti I2C bus to STM8 SCL/SDA
*/

/*-----------------------------------------------------------------------------
    INCLUDE FILES
-----------------------------------------------------------------------------*/
#include <stdlib.h>
#include <stdint.h>

#include "stm8as.h"
#include "timer3.h"
#include "i2c.h"
#include "i2c_poti.h"
#include "error_codes.h"


/*----------------------------------------------------------
    FUNCTIONS
----------------------------------------------------------*/

/**
  \fn uint8_t set_dig_poti(uint8_t res)
   
  \brief set resistance of potentiometer
  
  \param[in] res  new resistor value in [78.4R/INC] 
  
  \return  operation succeeded?
   
  set resistor btw terminal A and washer to 20k/255*res (res=[0;255]).
  Resistance btw terminal B and washer to 20k/255*(255-res)
*/
uint8_t set_dig_poti(uint8_t res) {

  // I2C Tx buffer
  uint8_t  s[2];

  // invert value to have right R(A;washer) behaviour
  res = (uint8_t)(255-res);
  
  // wait until bus is free (with timeout)
  start_timeout_ms(100);
  while ((I2C_BUSY) && (!check_timeout()));
  stop_timeout_ms(); 
    
  // generate start condition
  i2c_start();

  // send data (with timeout)
  s[0] = 0x00;
  s[1] = res;
  i2c_send(ADDR_I2C_POTI, 2, s);

  // generate stop condition
  i2c_stop();
  
  // wait until bus is free (with timeout)
  start_timeout_ms(100);
  while ((I2C_BUSY) && (!check_timeout()));
  stop_timeout_ms(); 

  // on I2C timeout return error code
  if (check_timeout())
    return(ERROR_TIMOUT);  
    
  // success
  return(SUCCESS);
  
} // set_dig_poti


/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/
