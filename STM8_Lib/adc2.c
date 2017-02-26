/**
  \file adc2.c
   
  \author G. Icking-Konert
  \date 2013-11-22
  \version 0.1
   
  \brief implementation of ADC2 functions/macros
   
  implementation of functions for ADC2 measurements in single-shot mode
*/

/*-----------------------------------------------------------------------------
    INCLUDE FILES
-----------------------------------------------------------------------------*/
#include <stdint.h>
#include "stm8as.h"
#include "adc2.h"


/*----------------------------------------------------------
    FUNCTIONS
----------------------------------------------------------*/

/**
  \fn void adc2_init(void)
   
  \brief initialize ADC
   
  initialize ADC2 for single-shot measurements. Default to channel AIN0
*/
void adc2_init() {

  // reset channel selection [3:0], no ADC interrupt [5]
  ADC2.CSR.byte = 0x00;
  
  // set single shot mode
  ADC2.CR1.reg.CONT = 0;
  
  // set ADC clock 1/12*fMaster (<2MHz)
  ADC2.CR1.reg.SPSEL = 6;
  
  // right alignment (read DRL, then DRH), no external trigger
  ADC2.CR2.reg.ALIGN = 1;
  
  // disable Schmitt trigger only for measurement channels
  //ADC2.TDR = 0xFFFF;
  
  // ADC module on
  ADC2.CR1.reg.ADON = 1; 
  
} // adc2_init



/**
  \fn uint16_t adc2_measure(uint8_t ch)
   
  \brief measure ADC2 channel
   
  \param ch         ADC2 channel to query
  \return           ADC2 value in INC
  
  measure ADC2 channel in single shot mode.
*/
uint16_t adc2_measure(uint8_t ch) {
 
  uint16_t result;
 
  // switch to ADC channel if required. Else skip to avoid AMUX charge injection
  if (ADC2.CSR.reg.CH != ch)
    ADC2.CSR.reg.CH = ch;
  
  // clear conversion ready flag, start conversion, wait until conversion done
  ADC2.CSR.reg.EOC  = 0;
  ADC2.CR1.reg.ADON = 1;             // start conversion 
  while(!ADC2.CSR.reg.EOC);          // wait for "conversion ready" flag
  
  // get ADC result (read low byte first for right alignment!)
  result  = (uint16_t) (ADC2.DR.byteL);
  result += ((uint16_t)(ADC2.DR.byteH)) << 8;
  
  // don't switch to default channel to avoid AMUX charge injection
  
  // return result
  return(result);

} // adc2_measure


/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/
