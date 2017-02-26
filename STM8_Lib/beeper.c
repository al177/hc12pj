/**
  \file beeper.c
   
  \author G. Icking-Konert
  \date 2013-11-22
  \version 0.1
   
  \brief implementation of beeper control
   
  implementation of functions for beeper control
*/

/*-----------------------------------------------------------------------------
    INCLUDE FILES
-----------------------------------------------------------------------------*/
#include "stm8as.h"
#include "beeper.h"
#include "timer3.h"   // for sleep_ms()


/*----------------------------------------------------------
    FUNCTIONS
----------------------------------------------------------*/

/**
  \fn void beep(uint8_t freq, uint16_t duration)
   
  \brief beeper control
   
  \param[in] freq       beeper frequency/state (1=1kHz, 2=2kHz, 4=4kHz, other=off)
  \param[in] duration   duration [ms] of signal (0 = start beep and continue code execution)
  
  control the on-board beeper with frequency/state and duration
*/
void beep(uint8_t freq, uint16_t duration) {

  // set to default calibration
  BEEP.CSR.byte = 0x0B;
  
  // switch beeper frequency ond state (assume stable LS clock 128kHz)
  switch (freq) {
  
    case 1:
      BEEP.CSR.reg.BEEPSEL = 0;  // set 1KHz
      BEEP.CSR.reg.BEEPEN  = 1;  // enable beeper
      break;
  
    case 2:
      BEEP.CSR.reg.BEEPSEL = 1;  // set 2KHz
      BEEP.CSR.reg.BEEPEN  = 1;  // enable beeper
      break;
  
    case 4:
      BEEP.CSR.reg.BEEPSEL = 2;  // set 4KHz
      BEEP.CSR.reg.BEEPEN  = 1;  // enable beeper
      break;
      
    default:
      BEEP.CSR.reg.BEEPEN  = 0;  // disable beeper

  } // switch (state) 
      
  
  // if duration >0 -> wait time [ms], then disable beeper
  if (duration > 0) {
    sleep_ms(duration);
    BEEP.CSR.reg.BEEPEN  = 0;    // disable beeper
  }

} // beep


/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/
