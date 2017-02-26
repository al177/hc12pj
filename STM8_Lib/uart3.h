/**
  \file uart3.h
   
  \author G. Icking-Konert
  \date 2013-11-22
  \version 0.1
   
  \brief declaration of UART3 / LINUART functions & macros
   
  declaration of UART3 / LINUART functions and macros. 
  Optional functionality via #define:
    - UART3_HALF_DUPLEX:  communication via 1-wire interface, e.g. LIN -> ignore echo after send (default: 2-wire)
    - UART3_FIFO:         send/receive is in background via interrupts and SW FIFO (default: blocking w/o FIFO)
*/

/*-----------------------------------------------------------------------------
    MODULE DEFINITION FOR MULTIPLE INCLUSION
-----------------------------------------------------------------------------*/
#ifndef _UART3_H_
#define _UART3_H_

#include <stdint.h>
#include "stm8as.h"


/*-----------------------------------------------------------------------------
    DECLARATION OF GLOBAL FUNCTIONS
-----------------------------------------------------------------------------*/

/// init UART3 for communication
void  uart3_init(uint32_t BR);

/// send byte via UART3
void  uart3_send_byte(uint8_t data);

/// send arry of bytes via UART3
void  uart3_send_buf(uint16_t num, uint8_t *buf);

/// UART3 check if data was received
uint8_t uart3_check_Rx(void);

/// UART3 data receive function (if FIFO is used, remove data from it)
uint8_t uart3_receive(void);


// SW fifo uses interrupts
#ifdef UART3_FIFO

  /// UART3 data peek function (only with FIFO, keep data in FIFO)
  uint8_t uart3_peek(void);

  // for Cosmic compiler
  #if defined(__CSMC__)
    
    /// with FIFO: UART3 transmit ISR; w/o FIFO: dummy
    @near @interrupt void uart3_TXE_ISR(void);

    /// with FIFO: UART3 receive ISR; w/o FIFO: dummy
    @near @interrupt void uart3_RXF_ISR(void);

  // for SDCC compiler
  #elif defined(__SDCC)
  
    /// with FIFO: UART3 transmit ISR; w/o FIFO: dummy
    void uart3_TXE_ISR(void) __interrupt(__UART2_3_4_TX_CMPL_VECTOR__);
  
    /// with FIFO: UART3 receive ISR; w/o FIFO: dummy
    void uart3_RXF_ISR(void) __interrupt(__UART2_3_4_RX_FULL_VECTOR__);  
  
  #endif // __CSMC__
  
#endif // UART3_FIFO

/*-----------------------------------------------------------------------------
    END OF MODULE DEFINITION FOR MULTIPLE INLUSION
-----------------------------------------------------------------------------*/
#endif  // _UART3_H_
