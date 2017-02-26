/**
  \file timer4.h
   
  \author G. Icking-Konert
  \date 2013-11-22
  \version 0.1
   
  \brief declaration of timer TIM4 (1ms clock) functions/macros
   
  declaration of timer TIM4 functions as 1ms master clock
*/

/*-----------------------------------------------------------------------------
    MODULE DEFINITION FOR MULTIPLE INCLUSION
-----------------------------------------------------------------------------*/
#ifndef _TIMER4_H_
#define _TIMER4_H_


/*-----------------------------------------------------------------------------
    INCLUDE FILES
-----------------------------------------------------------------------------*/

#include <stdint.h>
#include "stm8as.h"


/*-----------------------------------------------------------------------------
    DECLARATION OF GLOBAL VARIABLES
-----------------------------------------------------------------------------*/

global uint8_t            g_flagClock;        ///< flag for master clock interrupt. Set in TIM4 ISR
global uint32_t	          g_clock;            ///< counter for master clock interrupt. Increased in TIM4 ISR


/*-----------------------------------------------------------------------------
    DECLARATION OF GLOBAL FUNCTIONS
-----------------------------------------------------------------------------*/

/// init timer 4 (1ms master clock)
void tim4_init(void);

/// ISR for timer 4 (1ms master clock)
#if defined(__CSMC__)
  @near @interrupt void tim4_ISR(void);
#elif defined(__SDCC)
  void tim4_ISR(void) __interrupt(__TIM4UPD_VECTOR__);
#endif


/*-----------------------------------------------------------------------------
    END OF MODULE DEFINITION FOR MULTIPLE INLUSION
-----------------------------------------------------------------------------*/
#endif // _TIMER4_H_
