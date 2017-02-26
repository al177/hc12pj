/**
  \file timer4.c
   
  \author G. Icking-Konert
  \date 2013-11-22
  \version 0.1
   
  \brief implementation of timer TIM4 (1ms clock) functions/macros
   
  implementation of timer TIM4 functions as 1ms master clock
*/

/*----------------------------------------------------------
    INCLUDE FILES
----------------------------------------------------------*/
#include <stdint.h>
#include "stm8as.h"
#include "timer4.h"


/*----------------------------------------------------------
    FUNCTIONS
----------------------------------------------------------*/

/**
  \fn void tim4_init(void)
   
  \brief init timer 4 (master clock with 1ms)
   
  init timer TIM4 with 1ms tick. Is used for SW master clock.
*/
void tim4_init(void) {

  // initialize global TIM4 variables
  g_flagClock = 0;
  g_clock     = 0;

  // auto-reload value buffered
  TIM4.CR1.reg.ARPE = 1;

  // clear pending events
  TIM4.EGR.byte  = 0x00;

  // set clock to 16Mhz/128 -> 8us
  TIM4.PSCR.reg.PSC = 7;

  // set autoreload to 125 (125*8us = 1ms)
  TIM4.ARR.byte  = 125;          

  // enable timer 4 interrupt
  TIM4.IER.reg.UIE = 1;
  
  // start the timer
  TIM4.CR1.reg.CEN = 1;
  
} // tim4_init



/**
  \fn void tim4_ISR(void)
   
  \brief ISR for timer 4 (1ms master clock)
   
  interrupt service routine for timer TIM4.
  Used for 1ms master clock
*/
#if defined(__CSMC__)
  @near @interrupt void tim4_ISR(void)
#elif defined(__SDCC)
  void tim4_ISR() __interrupt(__TIM4UPD_VECTOR__)
#endif
{
  // clear timer 4 interrupt flag
  TIM4.SR1.reg.UIF = 0;

  // set global variables
  g_flagClock = 1;
  g_clock++;
  
  // service independent timeout watchdog
  //IWDG.KR.byte = 0xAAu;
 
  return;

} // tim4_ISR


/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/
