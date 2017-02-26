/**
  \file exint.h
   
  \author G. Icking-Konert
  \date 2013-11-22
  \version 0.1
   
  \brief declaration of functions/macros for external interrupts
   
  declaration of functions/macros for external interrupts on port pins
*/

/*-----------------------------------------------------------------------------
    MODULE DEFINITION FOR MULTIPLE INCLUSION
-----------------------------------------------------------------------------*/
#ifndef _EXINT_H_
#define _EXINT_H_

/*-----------------------------------------------------------------------------
    INCLUDE FILES
-----------------------------------------------------------------------------*/
 
#include <stdint.h>
#include "stm8as.h"


/*-----------------------------------------------------------------------------
    DECLARATION OF GLOBAL VARIABLES
-----------------------------------------------------------------------------*/

// EXINT event counters (increased in respective ISR)
global uint32_t     g_countExintA;      ///< PA external interrupts
global uint32_t	    g_countExintB;      ///< PB external interrupts
global uint32_t	    g_countExintC;      ///< PC external interrupts
global uint32_t	    g_countExintD;      ///< PD external interrupts
global uint32_t	    g_countExintE;      ///< PE external interrupts



/*-----------------------------------------------------------------------------
    DECLARATION OF GLOBAL MACROS
-----------------------------------------------------------------------------*/

// edge control for external interrupts (0=low, 1=rising, 2=falling, 3=both)
#define EXINT_LOW_LEVEL         0     ///< external interrupt on low level --> careful!
#define EXINT_RISE_EDGE         1     ///< external interrupt on rising edge
#define EXINT_FALL_EDGE         2     ///< external interrupt on falling edge
#define EXINT_BOTH_EDGE         3     ///< external interrupt on both edges


/*-----------------------------------------------------------------------------
    DECLARATION OF GLOBAL FUNCTIONS
-----------------------------------------------------------------------------*/

/// edge control for EXINTs
void exint_init(GPIO_TypeDef *addrPort, uint8_t edge);

/// ISR for external interrupt pins
#if defined(__CSMC__)
  @near @interrupt void EXINT_PA_ISR(void);  // port a
  @near @interrupt void EXINT_PB_ISR(void);  // port b
  @near @interrupt void EXINT_PC_ISR(void);  // port c
  @near @interrupt void EXINT_PD_ISR(void);  // port d
  @near @interrupt void EXINT_PE_ISR(void);  // port e
#elif defined(__SDCC)
  void EXINT_PA_ISR(void) __interrupt(__EXTI0_VECTOR__);  // port a
  void EXINT_PB_ISR(void) __interrupt(__EXTI1_VECTOR__);  // port b
  void EXINT_PC_ISR(void) __interrupt(__EXTI2_VECTOR__);  // port c
  void EXINT_PD_ISR(void) __interrupt(__EXTI3_VECTOR__);  // port d
  void EXINT_PE_ISR(void) __interrupt(__EXTI4_VECTOR__);  // port e
#endif


/*-----------------------------------------------------------------------------
    END OF MODULE DEFINITION FOR MULTIPLE INLUSION
-----------------------------------------------------------------------------*/
#endif // _EXINT_H_
