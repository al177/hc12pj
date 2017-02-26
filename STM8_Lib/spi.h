/**
  \file spi.h
   
  \author G. Icking-Konert
  \date 2013-11-22
  \version 0.1
   
  \brief declaration of SPI functions/macros
   
  declaration of functions for SPI communication
  For SPI bus, see https://en.wikipedia.org/wiki/Serial_Peripheral_Interface_Bus
  
  2017-02-25 al177 - hack and slash to make SPI master MSB only
*/

/*-----------------------------------------------------------------------------
    MODULE DEFINITION FOR MULTIPLE INCLUSION
-----------------------------------------------------------------------------*/
#ifndef _SPI_H_
#define _SPI_H_

#include <stdint.h>


/*-----------------------------------------------------------------------------
    DECLARATION OF GLOBAL FUNCTIONS
-----------------------------------------------------------------------------*/

/// configure SPI interface in master mode
void      spi_config(uint8_t pre, uint8_t phase, uint8_t idle);

/// send and receive data via SPI (master mode)
void      spi_send_receive(uint8_t numTx, uint8_t *buf);


/*-----------------------------------------------------------------------------
    DEFINITION OF GLOBAL MACROS/#DEFINES
-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
    END OF MODULE DEFINITION FOR MULTIPLE INLUSION
-----------------------------------------------------------------------------*/
#endif // _SPI_H_
