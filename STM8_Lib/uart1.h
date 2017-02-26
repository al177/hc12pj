/**
  \file uart1.h
   
  \author G. Icking-Konert
  \date 2013-11-22
  \version 0.1
   
  \brief declaration of UART1 / USART functions & macros
   
  declaration of UART1 / USART functions and macros. 
  Optional functionality via #define:
    - UART1_HALF_DUPLEX:  communication via 1-wire interface, e.g. LIN -> ignore echo after send (default: 2-wire)
    - UART1_FIFO:         send/receive is in background via interrupts and SW FIFO (default: blocking w/o FIFO)
*/

/*-----------------------------------------------------------------------------
    MODULE DEFINITION FOR MULTIPLE INCLUSION
-----------------------------------------------------------------------------*/
#ifndef _UART1_H_
#define _UART1_H_

#include <stdint.h>
#include "stm8as.h"


/*-----------------------------------------------------------------------------
    DECLARATION OF GLOBAL FUNCTIONS
-----------------------------------------------------------------------------*/

/// init UART1 for communication
void  uart1_init(uint32_t BR);

/// send byte via UART1
void  uart1_send_byte(uint8_t data);

/// send arry of bytes via UART1
void  uart1_send_buf(uint16_t num, uint8_t *buf);

/// UART1 check if data was received
uint8_t uart1_check_Rx(void);

/// UART1 data receive function (if FIFO is used, remove data from it)
uint8_t uart1_receive(void);

// SW fifo uses interrupts
#ifdef UART1_FIFO

  /// UART1 data peek function (only with FIFO, keep data in FIFO)
  uint8_t uart1_peek(void);

  // for Cosmic compiler
  #if defined(__CSMC__)
    
    /// with FIFO: UART1 transmit ISR; w/o FIFO: dummy
    @near @interrupt void uart1_TXE_ISR(void);

    /// with FIFO: UART1 receive ISR; w/o FIFO: dummy
    @near @interrupt void uart1_RXF_ISR(void);

  // for SDCC compiler
  #elif defined(__SDCC)
  
    /// with FIFO: UART1 transmit ISR; w/o FIFO: dummy
    void uart1_TXE_ISR(void) __interrupt(__UART1_TX_CMPL_VECTOR__);
  
    /// with FIFO: UART1 receive ISR; w/o FIFO: dummy
    void uart1_RXF_ISR(void) __interrupt(__UART1_RX_FULL_VECTOR__);  
  
  #endif // __CSMC__
  
#endif // UART1_FIFO

/*-----------------------------------------------------------------------------
    END OF MODULE DEFINITION FOR MULTIPLE INLUSION
-----------------------------------------------------------------------------*/
#endif  // _UART1_H_
