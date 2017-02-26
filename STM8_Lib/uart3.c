/**
  \file uart3.c
   
  \author G. Icking-Konert
  \date 2013-11-22
  \version 0.1
   
  \brief implementation of UART3 / LINUART functions/macros
   
  implementation of UART3 / LINUART functions and macros. 
  Optional functionality via #define:
    - UART3_HALF_DUPLEX:  communication via 1-wire interface, e.g. LIN -> ignore echo after send (default: 2-wire)
    - UART3_FIFO:         send/receive is in background via interrupts and SW FIFO (default: blocking w/o FIFO)
    - UART3_RST:          trigger SW reset on reception of "Re5eT!"
*/

/*-----------------------------------------------------------------------------
    INCLUDE FILES
-----------------------------------------------------------------------------*/
#include <stdio.h>
#include <stdint.h>
#include "stm8as.h"
#include "uart3.h"

// configure SW FIFO (see sw_fifo.h)
#ifdef UART3_FIFO
  #define FIFO_BUFFER_SIZE  64    ///< set buffer size for Rx and Tx (have to be same)
  #define FIFO_OPTIMIZE_RAM 0     ///< internal management. 1: save 2B RAM/FIFO; 0: save ~25B flash/FIFO and gain some speed
  #include "sw_fifo.h"            //   FIFO declaration and inline fuctions (for speed)
#endif // UART3_FIFO


/*-----------------------------------------------------------------------------
    MODULE VARIABLES (for clarity module internal variables start with "m_")
-----------------------------------------------------------------------------*/

// if FIFO is used, reserve buffers for Rx and Tx
#ifdef UART3_FIFO

  /// UART3 receive FIFO buffer
  fifo_t  m_UART3_Rx_Fifo = { {0}, 0, 0, 0, 0 };

  /// UART3 transmit FIFO buffer
  fifo_t  m_UART3_Tx_Fifo = { {0}, 0, 0, 0, 0 };

#endif // UART3_FIFO


// if UART reset command is used
#ifdef UART3_RST
  
  /// reset command string
  uint8_t  m_UART3_cmdReset[] = "Re5eT!";
  
  /// length of command string
  uint8_t  m_UART3_lenReset = 6;
  
  /// index for state machine
  uint8_t  m_UART3_idxReset = 0;
  
#endif // UART3_RST


/*----------------------------------------------------------
    FUNCTIONS
----------------------------------------------------------*/

/**
  \fn void uart3_init(uint32_t BR)
   
  \brief initialize UART3 for communication 
  
  \param[in]  BR    baudrate [Baud]

  initialize UART3 for communication with specified baudrate.
  Use 1 start, 8 data and 1 stop bit; no parity or flow control
  
  If FIFO is used, initialize Rx and Tx FIFOs and activate Rx interrupt.
*/
void uart3_init(uint32_t BR) {

  volatile uint16_t  val16;
  
  // set UART3 behaviour
  UART3.CR1.byte = 0x00;  // enable UART3, 8 data bits, no parity control
  UART3.CR2.byte = 0x00;  // no interrupts, disable sender/receiver 
  UART3.CR3.byte = 0x00;  // no LIN support, 1 stop bit, no clock output(?)

  // set baudrate (note: BRR2 must be written before BRR1!)
  val16 = (uint16_t) (((uint32_t) 16000000L)/BR);
  UART3.BRR2.byte = (uint8_t) (((val16 & 0xF000) >> 8) | (val16 & 0x000F));
  UART3.BRR1.byte = (uint8_t) ((val16 & 0x0FF0) >> 4);
  
  // enable transmission
  UART3.CR2.reg.REN = 1; // enable receiver
  UART3.CR2.reg.TEN = 1; // enable sender
  
  #ifdef UART3_FIFO

    // init FIFOs for receive and transmit
    fifo_init(&m_UART3_Rx_Fifo);
    fifo_init(&m_UART3_Tx_Fifo);
    
    // enable Rx interrupt. Tx interrupt is enabled in uart3_send()
    UART3.CR2.reg.RIEN = 1;

  #endif // UART3_FIFO

} // uart3_init


 
/**
  \fn void  uart3_send_byte(uint8_t data)
   
  \brief send byte via UART3
  
  \param[in]  byte   data to send

  send byte via UART3. 
  
  For direct UART3 access:
   - send byte via UART3 directly 
  
  If FIFO is used:
   - stores data into the Tx FIFO software buffer
   - enable the UART3 "Tx buffer empty" interrupt
   - actual transmission is handled by TXE interrupt routine 
*/
void  uart3_send_byte(uint8_t data) {

  // send data in background using FIFO
  #ifdef UART3_FIFO
    
    // wait until FIFO has free space 
    while(FIFO_FULL(m_UART3_Tx_Fifo));
    
    // disable UART3 "Tx empty interrupt" while manipulating Tx FIFO
    UART3.CR2.reg.TIEN = 0;
   
    // store byte in software FIFO
    fifo_enqueue(&m_UART3_Tx_Fifo, data);
    
    // enable UART3 "Tx empty interrupt" to handle sending data
    UART3.CR2.reg.TIEN = 1;


  // send data blocking without FIFO
  #else // UART3_FIFO

    // wait until TX buffer is available
    while (!(UART3.SR.reg.TXE));

    // for 1-wire I/F (e.g. LIN) disable receiver 
    #ifdef UART3_HALF_DUPLEX
      UART3.CR2.reg.REN = 0;
    #endif // UART3_HALF_DUPLEX

    // send byte
    UART3.DR.byte = data;

    // for 1-wire I/F (e.g. LIN) wait until byte is sent, then re-enable receiver 
    #ifdef UART3_HALF_DUPLEX
      while (!(UART3.SR.reg.TXE));
      UART3.CR2.reg.REN = 1;
    #endif // UART3_HALF_DUPLEX

  #endif // UART3_FIFO

} // uart3_send_byte


 
/**
  \fn void  uart3_send_byte(uint8_t data)
   
  \brief send byte via UART3
  
  \param[in]  byte   data to send

  send byte via UART3. 
  
  For direct UART3 access:
   - send byte via UART3 directly 
  
  If FIFO is used:
   - stores data into the Tx FIFO software buffer
   - enable the UART1 "Tx buffer empty" interrupt
   - actual transmission is handled by TXE interrupt routine 
*/
void  uart3_send_buf(uint16_t num, uint8_t *data) {

  uint16_t i;
  
  // send data in background using FIFO
  #ifdef UART3_FIFO
    
    // wait until FIFO has enough free space 
    while((FIFO_BUFFER_SIZE - m_UART3_Tx_Fifo.numBytes) < num);
    
    // disable UART3 "Tx empty interrupt" while manipulating Tx FIFO
    UART3.CR2.reg.TIEN = 0;
   
    // store bytes in software FIFO
    for (i=0; i<num; i++)
      fifo_enqueue(&m_UART3_Tx_Fifo, data[i]);
    
    // enable UART3 "Tx empty interrupt" to handle sending data
    UART3.CR2.reg.TIEN = 1;


  // send data blocking without FIFO
  #else // UART3_FIFO

    // wait until TX buffer is available
    while (!(UART3.SR.reg.TXE));

    // for 1-wire I/F (e.g. LIN) disable receiver 
    #ifdef UART3_HALF_DUPLEX
      UART3.CR2.reg.REN = 0;
    #endif // UART3_HALF_DUPLEX

    // send bytes
    for (i=0; i<num; i++)
      UART3.DR.byte = data[i];

    // for 1-wire I/F (e.g. LIN) wait until byte is sent, then re-enable receiver 
    #ifdef UART3_HALF_DUPLEX
      while (!(UART3.SR.reg.TXE));
      UART3.CR2.reg.REN = 1;
    #endif // UART3_HALF_DUPLEX

  #endif // UART3_FIFO

} // uart3_send_buf


 
/**
  \fn uint8_t uart3_check_Rx(void)
   
  \brief check if data was received via UART3 
  
  \return  1 = data in FIFO resp. in UART3 buffer

  check if data was received via UART3.
  
  For direct UART3 access:
   - check if UART3 Rx data register contains data
  
  If FIFO is used:
   - check if FIFO contains data 
*/
uint8_t uart3_check_Rx(void) {
  
  uint8_t   result;
  
  // check Rx FIFO status
  #ifdef UART3_FIFO
    
    // check if FIFO contains data
    result = FIFO_NOT_EMPTY(m_UART3_Rx_Fifo);
    
    
  // check UART3 register
  #else // UART3_FIFO
    
    // check if UART3 Rx register contains data
    result = UART3.SR.reg.RXNE;

  #endif // UART3_FIFO

  // return FIFO status
  return(result);
  
} // uart3_check_Rx



/**
  \fn uint8_t uart3_receive(void)
   
  \brief UART3 data receive function
  
  \return  oldest, not treated data
  
  For direct UART3 access:
   - return content of UART3 Rx data register
  
  If FIFO is used:
   - checks if data exists in the Rx FIFO software buffer
   - if data exists, return oldest FIFO element
   - remove oldest FIFO element from FIFO
*/
uint8_t uart3_receive(void) {

  uint8_t   data;
  
  // get Rx data from FIFO
  #ifdef UART3_FIFO

    // disable UART3 "Rx full interrupt" while manipulating Rx FIFO
    UART3.CR2.reg.RIEN = 0;
     
    // get oldest FIFO element (or -128 if FIFO is empty)
    data = fifo_dequeue(&m_UART3_Rx_Fifo);
    
    // re-enable UART3 "Rx full interrupt"
    UART3.CR2.reg.RIEN = 1;
    
  // get data from UART3 register
  #else // UART3_FIFO
    
    // get content of UART3 Rx register
    data = UART3.DR.byte;
    
    // optional check for UART reset command
    #ifdef UART3_RST
      
      // check next byte vs. keyword
      if (data == m_UART3_cmdReset[m_UART3_idxReset])
        m_UART3_idxReset++;
      else
        m_UART3_idxReset = 0;

      // trigger SW reset if full keyword received
      if (m_UART3_idxReset >= m_UART3_lenReset)
        SW_RESET;
      
    #endif // UART3_RST

  #endif // UART3_FIFO

  // return the FIFO data
  return(data);

} // uart3_receive


 
/**
  \fn uint8_t uart3_peek(void)
   
  \brief UART3 data peek function (only with FIFO)
  
  \return  oldest, not treated data
  
  Required for FIFO operation:
   - checks if data exists in the Rx FIFO software buffer
   - if data exists, return oldest FIFO element
   - Rx FIFO is not altered
*/
#ifdef UART3_FIFO
  uint8_t uart3_peek(void) {

    uint8_t   data=0;
    
    // disable UART3 "Rx full interrupt" while manipulating Rx FIFO
    UART3.CR2.reg.RIEN = 0;
     
    // get oldest FIFO element (or -128 if FIFO is empty)
    data = fifo_peek(&m_UART3_Rx_Fifo);
    
    // re-enable UART3 "Rx full interrupt"
    UART3.CR2.reg.RIEN = 1;

    // return the FIFO data
    return(data);

  } // uart3_peek
#endif // UART3_FIFO



/**
  \fn void uart3_RXF_ISR(void)
   
  \brief UART3 receive interrupt service routine
  
  Only for FIFO operation:
    - called if data received via UART3
    - copy data from HW buffer to FIFO
*/
#ifdef UART3_FIFO
  #if defined(__CSMC__)
    @near @interrupt void uart3_RXF_ISR(void) {
  #elif defined(__SDCC)
    void uart3_RXF_ISR() __interrupt(__UART2_3_4_RX_FULL_VECTOR__) {
  #endif

    uint8_t   data;
  
    // clearing of ISR flag not required for STM8
    //UART3.SR.reg.RXNE
   
    // read byte from UART3 buffer
    data = UART3.DR.byte;
  
    // add a new byte to the FIFO buffer
    fifo_enqueue(&m_UART3_Rx_Fifo, data);

    // check for optional UART reset command
    #ifdef UART3_RST

      // check next byte vs. keyword
      if (data == m_UART3_cmdReset[m_UART3_idxReset])
        m_UART3_idxReset++;
      else
        m_UART3_idxReset = 0;

      // trigger SW reset if full keyword received
      if (m_UART3_idxReset >= m_UART3_lenReset)
        SW_RESET;

    #endif // UART3_RST

    return;

  } // uart3_RXF_ISR
#endif // UART3_FIFO
 
 

/**
  \fn void uart3_TXE_ISR(void)
   
  \brief UART3 transmit interrupt service routine
  
  Only for FIFO operation:
    - called if Tx HW buffer is empty
    - checks if FIFO constains data
    - if yes, move oldest element from FIFO to Tx buffer
    - if FIFO is empty, disable this interrupt
*/
#ifdef UART3_FIFO
  #if defined(__CSMC__)
    @near @interrupt void uart3_TXE_ISR(void) {
  #elif defined(__SDCC)
    void uart3_TXE_ISR() __interrupt(__UART2_3_4_TX_CMPL_VECTOR__) {
  #endif
  
    uint8_t   data;
     
    // clearing of ISR flag not required for STM8
    //UART3.SR.reg.TXE
  
    // if Tx FIFO contains data, get oldest element and send it
    if (FIFO_NOT_EMPTY(m_UART3_Tx_Fifo)) {
      
      // get Tx byte from FIFO
      data = fifo_dequeue(&m_UART3_Tx_Fifo);

      // for 1-wire I/F (e.g. LIN) disable receiver 
      #ifdef UART3_HALF_DUPLEX
        UART3.CR2.reg.REN = 0;
      #endif // UART3_HALF_DUPLEX

      // send byte
      UART3.DR.byte = data;

      // for 1-wire I/F (e.g. LIN) wait until byte is sent, then re-enable receiver 
      #ifdef UART3_HALF_DUPLEX
        while (!(UART3.SR.reg.TXE));
        UART3.CR2.reg.REN = 1;
      #endif // UART3_HALF_DUPLEX

    } // Tx FIFO not empty
  
    // check again. If FIFO is empty now, deactivate this interrupt
    if (!(FIFO_NOT_EMPTY(m_UART3_Tx_Fifo)))
      UART3.CR2.reg.TIEN = 0;

    return;

  } // uart3_TXE_ISR
#endif // UART3_FIFO


/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/
