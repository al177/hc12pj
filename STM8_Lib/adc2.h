/**
  \file adc2.h
   
  \author G. Icking-Konert
  \date 2013-11-22
  \version 0.1
   
  \brief declaration of ADC2 functions/macros
   
  declaration of functions for ADC2 measurements in single-shot mode
*/

/*-----------------------------------------------------------------------------
    MODULE DEFINITION FOR MULTIPLE INCLUSION
-----------------------------------------------------------------------------*/
#ifndef _ADC2_H_
#define _ADC2_H_


/*-----------------------------------------------------------------------------
    DEFINITION OF GLOBAL MACROS/#DEFINES
-----------------------------------------------------------------------------*/

// includes
#include <stdint.h>


/*-----------------------------------------------------------------------------
    DECLARATION OF GLOBAL FUNCTIONS
-----------------------------------------------------------------------------*/

/// initialize ADC2 for single shot mode
void      adc2_init(void);

/// measure ADC2 channel
uint16_t  adc2_measure(uint8_t ch);


/*-----------------------------------------------------------------------------
    END OF MODULE DEFINITION FOR MULTIPLE INLUSION
-----------------------------------------------------------------------------*/
#endif  // _ADC_H_
