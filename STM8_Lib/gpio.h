/**
  \file gpio.h
   
  \author G. Icking-Konert
  \date 2013-11-22
  \version 0.1
   
  \brief declaration of functions/macros for GPIO control
   
  declaration of functions/macros for port pin control
  For speed use macros instead of functions where reasonable
*/

/*-----------------------------------------------------------------------------
    MODULE DEFINITION FOR MULTIPLE INCLUSION
-----------------------------------------------------------------------------*/
#ifndef _GPIO_H_
#define _GPIO_H_

/*-----------------------------------------------------------------------------
    INCLUDE FILES
-----------------------------------------------------------------------------*/
 
#include <stdint.h>
#include "stm8as.h"


/*-----------------------------------------------------------------------------
    DECLARATION OF GLOBAL MACROS
-----------------------------------------------------------------------------*/

// port control structs (PORT_x) see stm8as.h

// bitmasks for port pins
#define PIN_0    0x01                 ///< bitmask for port pin 0
#define PIN_1    0x02                 ///< bitmask for port pin 1
#define PIN_2    0x04                 ///< bitmask for port pin 2
#define PIN_3    0x08                 ///< bitmask for port pin 3
#define PIN_4    0x10                 ///< bitmask for port pin 4
#define PIN_5    0x20                 ///< bitmask for port pin 5
#define PIN_6    0x40                 ///< bitmask for port pin 6
#define PIN_7    0x80                 ///< bitmask for port pin 7

// define states for GPIOs for config_gpio()
#define INPUT_FLOAT_NOEXINT     0     ///< configure pin as: input, float, no exint
#define INPUT_FLOAT_EXINT       1     ///< configure pin as: input, float, with exint
#define INPUT_PULLUP_NOEXINT    2     ///< configure pin as: input, pull-up, no exint
#define INPUT_PULLUP_EXINT      3     ///< configure pin as: input, pull-up, with exint
#define OUTPUT_OPENDRAIN_SLOW   4     ///< configure pin as: output, open-drain, slow (2MHz)
#define OUTPUT_OPENDRAIN_FAST   5     ///< configure pin as: output, open-drain, fast (10MHz)
#define OUTPUT_PUSHPULL_SLOW    6     ///< configure pin as: output, push-pull, slow (2MHz)
#define OUTPUT_PUSHPULL_FAST    7     ///< configure pin as: output, push-pull, fast (10MHz)

// useful macros for accessing port pin (alternatively access bit Py.ODR.bit.by or Py.IDR.bit.by directly, see stm8as.h)
#define GPIO_HIGH(port,pins)      (port.ODR.byte |= (uint8_t)  pins)   ///< set port pin(s) to high (alternative: Py.ODR.bit.by=1, see stm8as.h)
#define GPIO_LOW(port,pins)       (port.ODR.byte &= (uint8_t) (~pins)) ///< set port pin(s) to low (alternative: Py.ODR.bit.by=0, see stm8as.h)
#define GPIO_TOGGLE(port,pins)    (port.ODR.byte ^= pins)              ///< toggle port pin(s) (alternative: Py.ODR.bit.by^=1, see stm8as.h)
#define GPIO_SET(port,pins,state) ( state ? GPIO_HIGH(port,pins) : GPIO_LOW(port,pins))   ///< set port pin(s) to state (alternative: Py.ODR.bit.by=state, see stm8as.h)
#define GPIO_READ(port,pins)      (uint8_t)(port.IDR.byte & pins)      ///< read state of port pin(s); is =1 if any masked pin is high (alternative: a=Py.IDR.bit.by, see stm8as.h)


/*-----------------------------------------------------------------------------
    DECLARATION OF GLOBAL FUNCTIONS
-----------------------------------------------------------------------------*/

/// configure GPIO pin
void gpio_init(GPIO_TypeDef *addrPort, uint8_t pins, uint8_t config);


/*-----------------------------------------------------------------------------
    END OF MODULE DEFINITION FOR MULTIPLE INLUSION
-----------------------------------------------------------------------------*/
#endif // _GPIO_H_
