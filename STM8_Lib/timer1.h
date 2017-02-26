/**
  \file timer1.h
   
  \author G. Icking-Konert
  \date 2013-11-22
  \version 0.1
   
  \brief declaration of TIM1 functions/macros (PWM channel) 
   
  declaration of timer TIM1 functions for generating and measuring 
  PWM signals, receiving SENT protocol etc.
*/

/*-----------------------------------------------------------------------------
    MODULE DEFINITION FOR MULTIPLE INCLUSION
-----------------------------------------------------------------------------*/
#ifndef _TIMER1_H_
#define _TIMER1_H_


/*-----------------------------------------------------------------------------
    INCLUDE FILES
-----------------------------------------------------------------------------*/

#include <stdint.h>
#include "stm8as.h"


/*-----------------------------------------------------------------------------
    DECLARATION OF GLOBAL FUNCTIONS
-----------------------------------------------------------------------------*/

/// init timer 1 (PWM channel)
void tim1_init(void);

/// generate PWM signal on TIM1_CC1 (=PC1)
void tim1_set_pwm(uint32_t centHz, uint16_t deciPrc);

// generate complementary PWM signal on TIM1_CC1(=PC1) and TIM1_NCC1(=PB0)
void tim1_set_pwm_complement(uint32_t centHz, uint16_t deciPrc);

/// measure PWM signal on TIM1_CC1 (=PC1)
void tim1_get_pwm(uint32_t *centHz, uint16_t *deciPrc);

/// receive data via SENT protocol on TIM1_CC1 (=PC1)
uint8_t tim1_receive_SENT(uint8_t *status, uint16_t *data_1, uint16_t *data_2, uint16_t timeout);


/*-----------------------------------------------------------------------------
    END OF MODULE DEFINITION FOR MULTIPLE INLUSION
-----------------------------------------------------------------------------*/
#endif // _TIMER1_H_
