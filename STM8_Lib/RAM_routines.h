/**
  \file RAM_routines.h
   
  \author G. Icking-Konert
  \date 2013-11-22
  \version 0.1
   
  \brief declaration of RAM routines for flash block w/e
   
  declaration of RAM routines which are required for flash write/erase
*/

/*-----------------------------------------------------------------------------
    MODULE DEFINITION FOR MULTIPLE INCLUSION
-----------------------------------------------------------------------------*/
#ifndef _RAM_ROUTINES_H_
#define _RAM_ROUTINES_H_

/*-----------------------------------------------------------------------------
    INCLUDE FILES
-----------------------------------------------------------------------------*/
#include <stdint.h>
#include "stm8as.h"


/*-----------------------------------------------------------------------------
    DEFINITION OF GLOBAL FUNCTIONS
-----------------------------------------------------------------------------*/

/// RAM routine to erase 128B flash block
void    RAM_erase_block(MEM_POINTER_T addr);

/// RAM routine to copy 128B buffer to flash block
void    RAM_write_block(MEM_POINTER_T addr, uint8_t buf[]);

/*-----------------------------------------------------------------------------
    END OF MODULE DEFINITION FOR MULTIPLE INLUSION
-----------------------------------------------------------------------------*/
#endif // _FLASH_H_
