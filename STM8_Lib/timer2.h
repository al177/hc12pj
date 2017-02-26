/**
  \file timer2.h
   
  \author G. Icking-Konert
  \date 2013-11-22
  \version 0.1
   
  \brief declaration of TIM2 functions/macros (PWM channel) 
   
  declaration of timer TIM2 functions for generating and measuring 
  PWM signals, receiving SENT protocol etc.
*/

/*-----------------------------------------------------------------------------
    MODULE DEFINITION FOR MULTIPLE INCLUSION
-----------------------------------------------------------------------------*/
#ifndef _TIMER2_H_
#define _TIMER2_H_


/*-----------------------------------------------------------------------------
    INCLUDE FILES
-----------------------------------------------------------------------------*/

#include <stdint.h>
#include "stm8as.h"


/*-----------------------------------------------------------------------------
    DECLARATION OF GLOBAL FUNCTIONS
-----------------------------------------------------------------------------*/

/// init timer 2 (PWM channel)
void tim2_init(void);

/// generate PWM signal on TIM2_CC2 (=PD3)
void tim2_set_pwm(uint32_t centHz, uint16_t deciPrc);

/// measure PWM signal on TIM2_CC2 (=PD3)
void tim2_get_pwm(uint32_t *centHz, uint16_t *deciPrc);

/// receive data via SENT protocol on TIM2_CC2 (=PD3)
uint8_t tim2_receive_SENT(uint8_t *status, uint16_t *data_1, uint16_t *data_2, uint16_t timeout);


/*-----------------------------------------------------------------------------
    END OF MODULE DEFINITION FOR MULTIPLE INLUSION
-----------------------------------------------------------------------------*/
#endif // _TIMER2_H_
