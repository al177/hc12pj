/**
  \file debug.h
   
  \author G. Icking-Konert
  \date 2013-11-22
  \version 0.1
   
  \brief declaration of global debug variables
   
  declaration of global variables with fixed addresses for debugging,
  e.g. via UART or STMStudio (via SWIM).
  For clarity all global variables start with "g_".
*/

/*----------------------------------------------------------
    MODULE DEFINITION FOR MULTIPLE INCLUSION
----------------------------------------------------------*/
#ifndef _DEBUG_H_
#define _DEBUG_H_

#include <stdint.h>
#include "stm8as.h"


/*-----------------------------------------------------------------------------
    DECLARATION OF GLOBAL VARIABLES
-----------------------------------------------------------------------------*/

// declare global variables with fixed addresses
#ifdef _MAIN_
  
  // misc variables for debugging via STM Studio (needs fixed address)
  reg(0x00F0, uint8_t,        g_valu8);             ///< global uint8 variable for debug via STM Studio
  reg(0x00F1, uint16_t,       g_valu16);            ///< global uint16 variable for debug via STM Studio
  reg(0x00F3, uint32_t,       g_valu32);            ///< global uint32 variable for debug via STM Studio
  
// refer to global variables
#else // _MAIN_
  extern uint8_t              g_valu8;
  extern uint16_t	            g_valu16;
  extern uint32_t	            g_valu32;
#endif // _MAIN_


/*-----------------------------------------------------------------------------
    END OF MODULE DEFINITION FOR MULTIPLE INLUSION
-----------------------------------------------------------------------------*/
#endif // _DEBUG_H_

