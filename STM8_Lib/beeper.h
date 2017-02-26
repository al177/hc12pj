/**
  \file beeper.h
   
  \author G. Icking-Konert
  \date 2013-11-22
  \version 0.1
   
  \brief declaration of beeper control
   
  declaration of functions for beeper control
*/

/*-----------------------------------------------------------------------------
    MODULE DEFINITION FOR MULTIPLE INCLUSION
-----------------------------------------------------------------------------*/
#ifndef _BEEPER_H_
#define _BEEPER_H_

#include <stdint.h>


/*-----------------------------------------------------------------------------
    DECLARATION OF GLOBAL FUNCTIONS
-----------------------------------------------------------------------------*/

/// beeper control
void beep(uint8_t freq, uint16_t duration);


/*-----------------------------------------------------------------------------
    DEFINITION OF GLOBAL MACROS/#DEFINES
-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
    END OF MODULE DEFINITION FOR MULTIPLE INLUSION
-----------------------------------------------------------------------------*/
#endif // _BEEPER_H_
