/**
  \file timer3.h
   
  \author G. Icking-Konert
  \date 2013-11-22
  \version 0.1
   
  \brief declaration of timer TIM3 functions/macros for sleep_x and timeout
   
  declaration of timer TIM3 functions for sleep_x and timeout
  via polling
*/

/*-----------------------------------------------------------------------------
    MODULE DEFINITION FOR MULTIPLE INCLUSION
-----------------------------------------------------------------------------*/
#ifndef _TIMER3_H_
#define _TIMER3_H_


/*-----------------------------------------------------------------------------
    INCLUDE FILES
-----------------------------------------------------------------------------*/

#include <stdint.h>
#include "stm8as.h"


/*-----------------------------------------------------------------------------
    DEFINITION OF GLOBAL MACROS/#DEFINES
-----------------------------------------------------------------------------*/

/// check for TIM3 timeout flag (see start_timeout_ms())
#define check_timeout()  (TIM3.SR1.reg.CC1IF)


/*-----------------------------------------------------------------------------
    DECLARATION OF GLOBAL FUNCTIONS
-----------------------------------------------------------------------------*/

/// init timer 3 (sleep/timeout)
void tim3_init(void);

/// halt code execution for dt*62.5ns
void sleep_ns(uint16_t dt);

/// halt code execution for  dt microseconds
void sleep_us(uint16_t dt);

/// halt code execution for dt milliseconds
void sleep_ms(uint16_t dt);

/// start timeout timer
void start_timeout_ms(uint16_t dt);

/// start timeout timer
void stop_timeout_ms(void);


/*-----------------------------------------------------------------------------
    END OF MODULE DEFINITION FOR MULTIPLE INLUSION
-----------------------------------------------------------------------------*/
#endif // _TIMER3_H_
