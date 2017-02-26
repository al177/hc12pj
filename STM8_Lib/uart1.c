/**
  \file uart1.c
   
  \author G. Icking-Konert
  \date 2013-11-22
  \version 0.1
   
  \brief implementation of UART1 / USART functions/macros
   
  implementation of UART1 / USART functions and macros. 
  Optional functionality via #define:
    - UART1_HALF_DUPLEX:  communication via 1-wire interface, e.g. LIN -> ignore echo after send (default: 2-wire)
    - UART1_FIFO:         send/receive is in background via interrupts and SW FIFO (default: blocking w/o FIFO)
    - UART1_RST:          trigger SW reset on reception of "Re5eT!"
*/

/*-----------------------------------------------------------------------------
    INCLUDE FILES
-----------------------------------------------------------------------------*/
#include <stdio.h>
#include <stdint.h>
#include "stm8as.h"
#include "uart1.h"

// configure SW FIFO (see sw_fifo.h)
#ifdef UART1_FIFO
  #define FIFO_BUFFER_SIZE  128   ///< set buffer size for Rx and Tx (have to be same)
  #define FIFO_OPTIMIZE_RAM 0     ///< internal management. 1: save 2B RAM/FIFO; 0: save ~25B flash/FIFO and gain some speed
  #include "sw_fifo.h"            //   FIFO declaration and inline fuctions (for speed)
#endif // UART1_FIFO


/*-----------------------------------------------------------------------------
    MODULE VARIABLES (for clarity module internal variables start with "m_")
-----------------------------------------------------------------------------*/

// if FIFO is used, reserve buffers for Rx and Tx
#ifdef UART1_FIFO

  /// UART1 receive FIFO buffer
  fifo_t  m_UART1_Rx_Fifo = { {0}, 0, 0, 0, 0 };

  /// UART1 transmit FIFO buffer
  fifo_t  m_UART1_Tx_Fifo = { {0}, 0, 0, 0, 0 };

#endif // UART1_FIFO


// if UART reset command is used
#ifdef UART1_RST
  
  /// reset command string
  uint8_t  m_UART1_cmdReset[] = "Re5eT!";
  
  /// length of command string
  uint8_t  m_UART1_lenReset = 6;
  
  /// index for state machine
  uint8_t  m_UART1_idxReset = 0;
  
#endif // UART1_RST


/*----------------------------------------------------------
    FUNCTIONS
----------------------------------------------------------*/

/**
  \fn void uart1_init(uint32_t BR)
   
  \brief initialize UART1 for communication 
  
  \param[in]  BR    baudrate [Baud]

  initialize UART1 for communication with specified baudrate.
  Use 1 start, 8 data and 1 stop bit; no parity or flow control.
  
  If FIFO is used, initialize Rx and Tx FIFOs and activate Rx interrupt.
*/
void uart1_init(uint32_t BR) {

  volatile uint16_t  val16;
  
  // set UART1 behaviour
  UART1.CR1.byte = 0x00;  // enable UART1, 8 data bits, no parity control
  UART1.CR2.byte = 0x00;  // no interrupts, disable sender/receiver 
  UART1.CR3.byte = 0x00;  // no LIN support, 1 stop bit, no clock output(?)

  // set baudrate (note: BRR2 must be written before BRR1!)
  val16 = (uint16_t) (((uint32_t) 16000000L)/BR);
  UART1.BRR2.byte = (uint8_t) (((val16 & 0xF000) >> 8) | (val16 & 0x000F));
  UART1.BRR1.byte = (uint8_t) ((val16 & 0x0FF0) >> 4);
  
  // enable transmission
  UART1.CR2.reg.REN = 1; // enable receiver
  UART1.CR2.reg.TEN = 1; // enable sender
  
  #ifdef UART1_FIFO

    // init FIFOs for receive and transmit
    fifo_init(&m_UART1_Rx_Fifo);
    fifo_init(&m_UART1_Tx_Fifo);
    
    // enable Rx interrupt. Tx interrupt is enabled in uart1_send()
    UART1.CR2.reg.RIEN = 1;

  #endif // UART1_FIFO

} // uart1_init


 
/**
  \fn void  uart1_send_byte(uint8_t data)
   
  \brief send byte via UART1
  
  \param[in]  byte   data to send

  send byte via UART1. 
  
  For direct UART1 access:
   - send byte via UART1 directly 
  
  If FIFO is used:
   - stores data into the Tx FIFO software buffer
   - enable the UART1 "Tx buffer empty" interrupt
   - actual transmission is handled by TXE interrupt routine 
*/
void  uart1_send_byte(uint8_t data) {

  // send data in background using FIFO
  #ifdef UART1_FIFO
    
    // wait until FIFO has free space 
    while(FIFO_FULL(m_UART1_Tx_Fifo));
        
    // disable UART1 "Tx empty interrupt" while manipulating Tx FIFO
    UART1.CR2.reg.TIEN = 0;
   
    // store byte in software FIFO
    fifo_enqueue(&m_UART1_Tx_Fifo, data);
    
    // enable UART1 "Tx empty interrupt" to handle sending data
    UART1.CR2.reg.TIEN = 1;


  // send data blocking without FIFO
  #else // UART1_FIFO

    // wait until TX buffer is available
    while (!(UART1.SR.reg.TXE));

    // for 1-wire I/F (e.g. LIN) disable receiver 
    #ifdef UART1_HALF_DUPLEX
      UART1.CR2.reg.REN = 0;
    #endif // UART1_HALF_DUPLEX

    // send byte
    UART1.DR.byte = data;

    // for 1-wire I/F (e.g. LIN) wait until byte is sent, then re-enable receiver 
    #ifdef UART1_HALF_DUPLEX
      while (!(UART1.SR.reg.TXE));
      UART1.CR2.reg.REN = 1;
    #endif // UART1_HALF_DUPLEX

  #endif // UART1_FIFO

} // uart1_send_byte


 
/**
  \fn void  uart1_send_buf(uint16_t num, uint8_t *data)
   
  \brief send arry of bytes via UART1
  
  \param[in]  num    buf size in bytes
  \param[in]  data   bytes to send

  send array of bytes via UART1. 
  
  For direct UART1 access:
   - send bytes via UART1 directly 
  
  If FIFO is used:
   - stores data into the Tx FIFO software buffer
   - enable the UART1 "Tx buffer empty" interrupt
   - actual transmission is handled by TXE interrupt routine 
*/
void  uart1_send_buf(uint16_t num, uint8_t *data) {

  uint16_t i;
  
  // send data in background using FIFO
  #ifdef UART1_FIFO
    
    // wait until FIFO has enough free space 
    while((FIFO_BUFFER_SIZE - m_UART1_Tx_Fifo.numBytes) < num);
    
    // disable UART1 "Tx empty interrupt" while manipulating Tx FIFO
    UART1.CR2.reg.TIEN = 0;
   
    // store bytes in software FIFO
    for (i=0; i<num; i++)
      fifo_enqueue(&m_UART1_Tx_Fifo, data[i]);
    
    // enable UART1 "Tx empty interrupt" to handle sending data
    UART1.CR2.reg.TIEN = 1;


  // send data blocking without FIFO
  #else // UART1_FIFO

    // wait until TX buffer is available
    while (!(UART1.SR.reg.TXE));

    // for 1-wire I/F (e.g. LIN) disable receiver 
    #ifdef UART1_HALF_DUPLEX
      UART1.CR2.reg.REN = 0;
    #endif // UART1_HALF_DUPLEX

    // send bytes
    for (i=0; i<num; i++)
      UART1.DR.byte = data[i];

    // for 1-wire I/F (e.g. LIN) wait until byte is sent, then re-enable receiver 
    #ifdef UART1_HALF_DUPLEX
      while (!(UART1.SR.reg.TXE));
      UART1.CR2.reg.REN = 1;
    #endif // UART1_HALF_DUPLEX

  #endif // UART1_FIFO

} // uart1_send_buf


 
/**
  \fn uint8_t uart1_check_Rx(void)
   
  \brief check if data was received via UART1 
  
  \return  1 = data in FIFO resp. in UART1 buffer

  check if data was received via UART1.
  
  For direct UART1 access:
   - check if UART1 Rx data register contains data
  
  If FIFO is used:
   - check if FIFO contains data 
*/
uint8_t uart1_check_Rx(void) {
  
  uint8_t   result;
  
  // check Rx FIFO status
  #ifdef UART1_FIFO
    
    // check if FIFO contains data
    result = FIFO_NOT_EMPTY(m_UART1_Rx_Fifo);
    
    
  // check UART1 register
  #else // UART1_FIFO
    
    // check if UART1 Rx register contains data
    result = UART1.SR.reg.RXNE;

  #endif // UART1_FIFO

  // return FIFO status
  return(result);
  
} // uart1_check_Rx



/**
  \fn uint8_t uart1_receive(void)
   
  \brief UART1 data receive function
  
  \return  oldest, not treated data
  
  For direct UART1 access:
   - return content of UART1 Rx data register
  
  If FIFO is used:
   - checks if data exists in the Rx FIFO software buffer
   - if data exists, return oldest FIFO element
   - remove oldest FIFO element from FIFO
*/
uint8_t uart1_receive(void) {

  uint8_t   data;
  
  // get Rx data from FIFO
  #ifdef UART1_FIFO

    // disable UART1 "Rx full interrupt" while manipulating Rx FIFO
    UART1.CR2.reg.RIEN = 0;
     
    // get oldest FIFO element (or -128 if FIFO is empty)
    data = fifo_dequeue(&m_UART1_Rx_Fifo);
    
    // re-enable UART1 "Rx full interrupt"
    UART1.CR2.reg.RIEN = 1;
    
  // get data from UART1 register
  #else // UART1_FIFO
    
    // get content of UART1 Rx register
    data = UART1.DR.byte;

    // optional check for UART reset command
    #ifdef UART1_RST

      // check next byte vs. keyword
      if (data == m_UART1_cmdReset[m_UART1_idxReset])
        m_UART1_idxReset++;
      else
        m_UART1_idxReset = 0;

      // trigger SW reset if full keyword received
      if (m_UART1_idxReset >= m_UART1_lenReset)
        SW_RESET;
      
    #endif // UART1_RST

  #endif // UART1_FIFO
    
  // return the FIFO data
  return(data);

} // uart1_receive


 
/**
  \fn uint8_t uart1_peek(void)
   
  \brief UART1 data peek function (only with FIFO)
  
  \return  oldest, not treated data
  
  Required for FIFO operation:
   - checks if data exists in the Rx FIFO software buffer
   - if data exists, return oldest FIFO element
   - Rx FIFO is not altered
*/
#ifdef UART1_FIFO
  uint8_t uart1_peek(void) {

    uint8_t   data=0;
    
    // disable UART1 "Rx full interrupt" while manipulating Rx FIFO
    UART1.CR2.reg.RIEN = 0;
     
    // get oldest FIFO element (or -128 if FIFO is empty)
    data = fifo_peek(&m_UART1_Rx_Fifo);
    
    // re-enable UART1 "Rx full interrupt"
    UART1.CR2.reg.RIEN = 1;

    // return the FIFO data
    return(data);

  } // uart1_peek
#endif // UART1_FIFO



/**
  \fn void uart1_RXF_ISR(void)
   
  \brief UART1 receive interrupt service routine
  
  Only for FIFO operation:
    - called if data received via UART1
    - copy data from HW buffer to FIFO
*/
#ifdef UART1_FIFO
  #if defined(__CSMC__)
    @near @interrupt void uart1_RXF_ISR(void) {
  #elif defined(__SDCC)
    void uart1_RXF_ISR() __interrupt(__UART1_RX_FULL_VECTOR__) {
  #endif

    uint8_t   data;
  
    // clearing of ISR flag not required for STM8
    //UART1.SR.reg.RXNE
   
    // read byte from UART1 buffer
    data = UART1.DR.byte;
  
    // add a new byte to the FIFO buffer
    fifo_enqueue(&m_UART1_Rx_Fifo, data);

    // check for optional UART reset command
    #ifdef UART1_RST

      // check next byte vs. keyword
      if (data == m_UART1_cmdReset[m_UART1_idxReset])
        m_UART1_idxReset++;
      else
        m_UART1_idxReset = 0;

      // trigger SW reset if full keyword received
      if (m_UART1_idxReset >= m_UART1_lenReset)
        SW_RESET;

    #endif // UART1_RST

    return;

  } // uart1_RXF_ISR
#endif // UART1_FIFO
 
 

/**
  \fn void uart1_TXE_ISR(void)
   
  \brief UART1 transmit interrupt service routine
  
  Only for FIFO operation:
    - called if Tx HW buffer is empty
    - checks if FIFO constains data
    - if yes, move oldest element from FIFO to Tx buffer
    - if FIFO is empty, disable this interrupt
*/
#ifdef UART1_FIFO
  #if defined(__CSMC__)
    @near @interrupt void uart1_TXE_ISR(void) {
  #elif defined(__SDCC)
    void uart1_TXE_ISR() __interrupt(__UART1_TX_CMPL_VECTOR__) {
  #endif
  
    uint8_t   data;
     
    // clearing of ISR flag not required for STM8
    //UART1.SR.reg.TXE
  
    // if Tx FIFO contains data, get oldest element and send it
    if (FIFO_NOT_EMPTY(m_UART1_Tx_Fifo)) {
      
      // get Tx byte from FIFO
      data = fifo_dequeue(&m_UART1_Tx_Fifo);

      // for 1-wire I/F (e.g. LIN) disable receiver 
      #ifdef UART1_HALF_DUPLEX
        UART1.CR2.reg.REN = 0;
      #endif // UART1_HALF_DUPLEX

      // send byte
      UART1.DR.byte = data;

      // for 1-wire I/F (e.g. LIN) wait until byte is sent, then re-enable receiver 
      #ifdef UART1_HALF_DUPLEX
        while (!(UART1.SR.reg.TXE));
        UART1.CR2.reg.REN = 1;
      #endif // UART1_HALF_DUPLEX

    } // Tx FIFO not empty
  
    // check again. If FIFO is empty now, deactivate this interrupt
    if (!(FIFO_NOT_EMPTY(m_UART1_Tx_Fifo)))
      UART1.CR2.reg.TIEN = 0;

    return;

  } // uart1_TXE_ISR
#endif // UART1_FIFO


/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/
