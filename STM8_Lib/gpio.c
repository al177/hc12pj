/**
  \file gpio.c
   
  \author G. Icking-Konert
  \date 2013-11-22
  \version 0.1
   
  \brief implementation of functions/macros for GPIO control
   
  implementation of functions/macros for port pin control
  For speed use macros instead of functions where reasonable
*/

/*-----------------------------------------------------------------------------
    INCLUDE FILES
-----------------------------------------------------------------------------*/
#include <stdint.h>
#include "stm8as.h"
#include "gpio.h"


/*----------------------------------------------------------
    FUNCTIONS
----------------------------------------------------------*/

/**
  \fn void gpio_init(GPIO_TypeDef *addrPort, uint8_t pins, uint8_t config)
   
  \brief configure GPIO pin
  
  \param[in]  addrPort  address of port to configure, e.g. PORT_A (see stm8as.h)
  \param[in]  pins      bitmask of one or more port pins to configure, e.g. PIN_1 (see gpio.h)
  \param[in]  config    select pin configuration, e.g. INPUT_PULLUP_NOEXINT (see gpio.h)

  configure GPIO port pins as input or output with different features activated.
  For supported configurations see gpio.h. Multiple port pins can be configured
  with same configuration
*/
void gpio_init(GPIO_TypeDef *addrPort, uint8_t pins, uint8_t config) {

  // set corresponding port config bits
  switch (config) {
    
    case INPUT_FLOAT_NOEXINT:
      addrPort->DDR.byte &= (uint8_t) (~pins);    // input(=0) or output(=1)
      addrPort->CR1.byte &= (uint8_t) (~pins);    // input: 0=float, 1=pull-up; output: 0=open-drain, 1=push-pull
      addrPort->CR2.byte &= (uint8_t) (~pins);    // input: 0=no exint, 1=exint; output: 0=2MHz slope, 1=10MHz slope
      break;
    
    case INPUT_FLOAT_EXINT:
      addrPort->DDR.byte &= (uint8_t) (~pins);    // input(=0) or output(=1)
      addrPort->CR1.byte &= (uint8_t) (~pins);    // input: 0=float, 1=pull-up; output: 0=open-drain, 1=push-pull
      addrPort->CR2.byte |= (uint8_t) pins;       // input: 0=no exint, 1=exint; output: 0=2MHz slope, 1=10MHz slope
      break;
    
    case INPUT_PULLUP_NOEXINT:
      addrPort->DDR.byte &= (uint8_t) (~pins);    // input(=0) or output(=1)
      addrPort->CR1.byte |= (uint8_t) pins;       // input: 0=float, 1=pull-up; output: 0=open-drain, 1=push-pull
      addrPort->CR2.byte &= (uint8_t) (~pins);    // input: 0=no exint, 1=exint; output: 0=2MHz slope, 1=10MHz slope
      break;
    
    case INPUT_PULLUP_EXINT:
      addrPort->DDR.byte &= (uint8_t) (~pins);    // input(=0) or output(=1)
      addrPort->CR1.byte |= (uint8_t) pins;       // input: 0=float, 1=pull-up; output: 0=open-drain, 1=push-pull
      addrPort->CR2.byte |= (uint8_t) pins;       // input: 0=no exint, 1=exint; output: 0=2MHz slope, 1=10MHz slope
      break;
    
    case OUTPUT_OPENDRAIN_SLOW:
      addrPort->DDR.byte |= (uint8_t) pins;       // input(=0) or output(=1)
      addrPort->CR1.byte &= (uint8_t) (~pins);    // input: 0=float, 1=pull-up; output: 0=open-drain, 1=push-pull
      addrPort->CR2.byte &= (uint8_t) (~pins);    // input: 0=no exint, 1=exint; output: 0=2MHz slope, 1=10MHz slope
      break;
    
    case OUTPUT_OPENDRAIN_FAST:
      addrPort->DDR.byte |= (uint8_t) pins;       // input(=0) or output(=1)
      addrPort->CR1.byte &= (uint8_t) (~pins);    // input: 0=float, 1=pull-up; output: 0=open-drain, 1=push-pull
      addrPort->CR2.byte |= (uint8_t) pins;       // input: 0=no exint, 1=exint; output: 0=2MHz slope, 1=10MHz slope
      break;
    
    case OUTPUT_PUSHPULL_SLOW:
      addrPort->DDR.byte |= (uint8_t) pins;       // input(=0) or output(=1)
      addrPort->CR1.byte |= (uint8_t) pins;       // input: 0=float, 1=pull-up; output: 0=open-drain, 1=push-pull
      addrPort->CR2.byte &= (uint8_t) (~pins);    // input: 0=no exint, 1=exint; output: 0=2MHz slope, 1=10MHz slope
      break;
    
    case OUTPUT_PUSHPULL_FAST:
      addrPort->DDR.byte |=  (uint8_t) pins;       // input(=0) or output(=1)
      addrPort->CR1.byte |=  (uint8_t) pins;       // input: 0=float, 1=pull-up; output: 0=open-drain, 1=push-pull
      addrPort->CR2.byte |=  (uint8_t) pins;       // input: 0=no exint, 1=exint; output: 0=2MHz slope, 1=10MHz slope
      break;
    
  } // switch (config) 
    
} // gpio_init


/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/
