/**
  \file RAM_routines.c
   
  \author G. Icking-Konert
  \date 2013-11-22
  \version 0.1
   
  \brief implementation of RAM routines for flash block w/e
   
  implementation of RAM routines which are required for flash write/erase
*/

/*----------------------------------------------------------
    INCLUDE FILES
----------------------------------------------------------*/
#include <stdint.h>
#include "stm8as.h"
#include "flash.h"
#include "memory_access.h"
#include "RAM_routines.h"


// keyword for Cosmic compiler place code in RAM
#if defined(__CSMC__)  
  #pragma section (RAM_CODE)
#endif // __CSMC__


/**
  \fn void RAM_erase_block(void)
  
  \brief RAM routine to erase 128B flash block
  
  erase 128B flash block which contains g_addr. 
  Actual block erase needs to be executed from RAM.
  Use global address to minimize valuable RAM space
*/
void RAM_erase_block(MEM_POINTER_T addr) {

  // enable block erase
  FLASH.CR2.reg.ERASE  = 1;
  FLASH.NCR2.reg.ERASE = 0;      // complementary register
  
  // clear 4B word in sector
  write_4B(addr, 0x00000000);
  
  // wait until erase finished
  while (!FLASH.IAPSR.reg.EOP);

} // RAM_erase_block



/**
  \fn void RAM_write_block(MEM_POINTER_T addr, uint8_t buf[])
  
  \brief RAM routine to copy 128B buffer to flash block
  
  \param[in] addr   starting address of block to write
  \param[in] buf    128B buffer to write

  write 128B flash block starting at addr. 
  Actual block write needs to be executed from RAM
*/
void RAM_write_block(MEM_POINTER_T addr, uint8_t buf[]) {

  uint8_t       i;
    
  // enable fast block programming mode
  FLASH.CR2.reg.FPRG   = 1;
  FLASH.NCR2.reg.FPRG  = 0;
  
  // copy content to latch
  for (i=0; i<128; i++)
    write_1B(addr++, buf[i]);
  
  // wait until erase finished
  while (!FLASH.IAPSR.reg.EOP);
      
} // RAM_write_block


// end of RAM code for Cosmic compiler
#if defined(__CSMC__)  
  #pragma section ()
#endif // __CSMC__

/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/
