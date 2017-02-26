/**
  \file exint.c
   
  \author G. Icking-Konert
  \date 2013-11-22
  \version 0.1
   
  \brief implementation of functions/macros for external interrupts
   
  implementation of functions/macros for external interrupts on port pins
*/

/*-----------------------------------------------------------------------------
    INCLUDE FILES
-----------------------------------------------------------------------------*/
#include <stdint.h>
#include "stm8as.h"
#include "gpio.h"
#include "exint.h"


/*----------------------------------------------------------
    FUNCTIONS
----------------------------------------------------------*/

/**
  \fn void exint_init(GPIO_TypeDef *addrPort, uint8_t edge)
   
  \brief edge control for EXINTs
  
  \param[in]  addrPort  address of port to configure, e.g. PORT_A (see gpio.h)
  \param[in]  edge      select edge for exint, e.g. EXINT_RISE_EDGE (see gpio.h)

  configure edge control for GPIO external interrupts. Can only be done for complete
  port. For pins with EXINT functionality see exint.h.
  
  Note that all interrupts have to be disabled when calling this routine!
*/
void exint_init(GPIO_TypeDef *addrPort, uint8_t edge) {

  // select correct port
  if (addrPort == &PORT_A) {
    EXTI.CR1.reg.PAIS = edge;   // set edge control
    g_countExintA     = 0;      // reset counter
  }
  else if (addrPort == &PORT_B) {
    EXTI.CR1.reg.PBIS = edge;   // set edge control
    g_countExintB     = 0;      // reset counter
  }
  else if (addrPort == &PORT_C) {
    EXTI.CR1.reg.PCIS = edge;   // set edge control
    g_countExintC     = 0;      // reset counter
  }
  else if (addrPort == &PORT_D) {
    EXTI.CR1.reg.PDIS = edge;   // set edge control
    g_countExintD     = 0;      // reset counter
  }
  else if (addrPort == &PORT_E) {
    EXTI.CR2.reg.PEIS = edge;   // set edge control
    g_countExintE     = 0;      // reset counter
  }

} // exint_init



/**
  \fn void EXINT_PA_ISR(void)
   
  \brief ISR for external interrupt on port A
   
  interrupt service routine for external interrupt on a port A pin.
  Here only a global event counter is increased, see exint.h.
*/
#if defined(__CSMC__)
  @near @interrupt void EXINT_PA_ISR(void)
#elif defined(__SDCC)
  void EXINT_PA_ISR() __interrupt(__EXTI0_VECTOR__)
#endif
{
  // increase global exint counter (port specific)
  g_countExintA++;
  
  return;
  
} // EXINT_PA_ISR



/**
  \fn void EXINT_PB_ISR(void)
   
  \brief ISR for external interrupt on port B
   
  interrupt service routine for external interrupt on a port B pin.
  Here only a global event counter is increased, see exint.h.
*/
#if defined(__CSMC__)
  @near @interrupt void EXINT_PB_ISR(void)
#elif defined(__SDCC)
  void EXINT_PB_ISR() __interrupt(__EXTI1_VECTOR__)
#endif
{
  // increase global exint counter (port specific)
  g_countExintB++;
  
  return;
  
} // EXINT_PB_ISR



/**
  \fn void EXINT_PC_ISR(void)
   
  \brief ISR for external interrupt on port C
   
  interrupt service routine for external interrupt on a port C pin.
  Here only a global event counter is increased, see exint.h.
*/
#if defined(__CSMC__)
  @near @interrupt void EXINT_PC_ISR(void)
#elif defined(__SDCC)
  void EXINT_PC_ISR() __interrupt(__EXTI2_VECTOR__)
#endif
{
  // increase global exint counter (port specific)
  g_countExintC++;
  
  return;

} // EXINT_PC_ISR



/**
  \fn void EXINT_PD_ISR(void)
   
  \brief ISR for external interrupt on port D
   
  interrupt service routine for external interrupt on a port D pin.
  Here only a global event counter is increased, see exint.h.
*/
#if defined(__CSMC__)
  @near @interrupt void EXINT_PD_ISR(void)
#elif defined(__SDCC)
  void EXINT_PD_ISR() __interrupt(__EXTI3_VECTOR__)
#endif
{
  // increase global exint counter (port specific)
  g_countExintD++;
  
  return;

} // EXINT_PD_ISR



/**
  \fn void EXINT_PE_ISR(void)
   
  \brief ISR for external interrupt on port E
   
  interrupt service routine for external interrupt on a port E pin.
  Here only a global event counter is increased, see exint.h.
*/
#if defined(__CSMC__)
  @near @interrupt void EXINT_PE_ISR(void)
#elif defined(__SDCC)
  void EXINT_PE_ISR() __interrupt(__EXTI4_VECTOR__)
#endif
{

  // increase global exint counter (port specific)
  g_countExintE++;
  
  return;

} // EXINT_PE_ISR


/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/
