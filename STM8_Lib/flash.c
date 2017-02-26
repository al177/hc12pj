/**
  \file flash.c
   
  \author G. Icking-Konert
  \date 2013-11-22
  \version 0.1
   
  \brief implementation of flash read/write routines
   
  implementation of flash read/write routines.
  RAM routines for block operations are in RAM_routines.c
*/

/*----------------------------------------------------------
    INCLUDE FILES
----------------------------------------------------------*/
#include <stdint.h>
#include "stm8as.h"
#include "flash.h"
#include "memory_access.h"

/**
  \fn void flash_OPT_default(void)
  
  \brief assert default option byte setting
  
  assert that all option bytes have their default setting (see below).
	On change trigger a reset.
*/
void flash_OPT_default() {
	
	uint8_t	flagWD = 0;

  // reset alternate GPIO mapping (=OPT2/NOPT2)
  if ((*((uint8_t*) OPT2) != 0x00) || (*((uint8_t*) NOPT2)  != 0xFF)) {
    flash_write_option_byte(OPT2,  0x00);
    flash_write_option_byte(NOPT2, 0xFF);
    flagWD = 1;
  }
  
  // deactivate watchdog (=OPT3/NOPT3)
  if ((*((uint8_t*) OPT3) != 0x00) || (*((uint8_t*) NOPT3)  != 0xFF)) {
    flash_write_option_byte(OPT3,  0x00);
    flash_write_option_byte(NOPT3, 0xFF);
    flagWD = 1;
  }
  
  // reset clock options to default (=OPT4/NOPT4)
  if ((*((uint8_t*) OPT4) != 0x00) || (*((uint8_t*) NOPT4)  != 0xFF)) {
    flash_write_option_byte(OPT4,  0x00);
    flash_write_option_byte(NOPT4, 0xFF);
    flagWD = 1;
  }
   
  // max. HCE clock startup time (=OPT5/NOPT5)
  if ((*((uint8_t*) OPT5) != 0x00) || (*((uint8_t*) NOPT5)  != 0xFF)) {
    flash_write_option_byte(OPT5,  0x00);
    flash_write_option_byte(NOPT5, 0xFF);
    flagWD = 1;
  }
   
  // OPT6 is reserved/undocumented
   
  // no flash wait state (required for >16MHz) (=OPT7/NOPT7)
  if ((*((uint8_t*) OPT7) != 0x00) || (*((uint8_t*) NOPT7)  != 0xFF)) {
    flash_write_option_byte(OPT7,  0x00);
    flash_write_option_byte(NOPT7, 0xFF);
    flagWD = 1;
  }
   
  // OPT8-16 contain temporary memory unprotection key (TMU) -> rather don't touch 

  // activate ROM-bootloader (=OPT17/NOPT17)  
  if ((*((uint8_t*) OPT17) != 0x55) || (*((uint8_t*) NOPT17)  != 0xAA)) {
    flash_write_option_byte(OPT17,  0x55);
    flash_write_option_byte(NOPT17, 0xAA);
    flagWD = 1;
  }
 
  // if any option byte was changed trigger SW reset
  if (flagWD != 0)
    SW_RESET;

} // flash_OPT_default()



/**
  \fn uint8_t flash_write_option_byte(uint16_t addr, uint8_t byte)
  
  \brief write option byte
  
  \param[in] addr   16b address of option byte to write
  \param[in] byte   byte to program
  
  \return byte changed (=1) or unchanged (=0)

  write single option byte to given address (16b sufficient for all devices).
  Only write if value needs to be changed.
*/
uint8_t flash_write_option_byte(uint16_t addr, uint8_t byte) {

  uint8_t       countTimeout=0;   // use counter rather than timer for flash timeout to avoid conflicts

  // check address
  if ((addr < 0x4800) || (addr > 0x48FF))
    return(0);
  
  // skip write if value is already correct
  if ((*((uint8_t*) addr)) == byte)
    return(0);
  
  // disable interrupts
  DISABLE_INTERRUPTS;

  // unlock w/e access to EEPROM & option bytes
  FLASH.DUKR.byte = 0xAE;
  FLASH.DUKR.byte = 0x56;
  
  // additionally required for option bytes
  FLASH.CR2.byte  |= 0x80;
  FLASH.NCR2.byte &= 0x7F;
  
  // wait until access granted
  while(!FLASH.IAPSR.reg.DUL);

  // write option byte to p-flash
  *((uint8_t*) addr) = byte;
  
  // wait until done or timeout (normal flash write measured with 0 --> 100 is more than sufficient)
  while ((!FLASH.IAPSR.reg.EOP) && ((++countTimeout) < 100));
    
  // lock EEPROM again against accidental erase/write
  FLASH.IAPSR.reg.DUL = 0;
  
  // additional lock
  FLASH.CR2.byte  &= 0x7F;
  FLASH.NCR2.byte |= 0x80;
  
  
  // enable interrupts
  ENABLE_INTERRUPTS;

  // option byte changed -> return 1
  return(1);

} // flash_write_option_byte



/**
  \fn void flash_write_byte(MEM_POINTER_T addr, uint8_t data)
  
  \brief write single byte to flash
  
  \param[in] addr   address to write to
  \param[in] data   byte to program

  write single byte to address in P-flash or EEPROM. 
  For address width see file stm8as.h
  
  Warning: for simplicity no safeguard is used to protect against
  accidental data loss or even overwriting the application -> use with care!
*/
void flash_write_byte(MEM_POINTER_T addr, uint8_t data) {

  uint8_t       countTimeout=0;   // use counter for timeout to avoid timer conflicts
  
  // disable interrupts
  DISABLE_INTERRUPTS;
  
  // unlock w/e access to P-flash
  FLASH.PUKR.byte = 0x56;
  FLASH.PUKR.byte = 0xAE;
    
  // unlock w/e access to EEPROM
  FLASH.DUKR.byte = 0xAE;
  FLASH.DUKR.byte = 0x56;
    
  // wait until access granted
  while(!FLASH.IAPSR.reg.PUL);
  while(!FLASH.IAPSR.reg.DUL);
  
  // write byte using 16-bit or 32-bit macro/function (see flash.h)
  write_1B(addr, data);

  // wait until done or timeout (normal flash write measured with 0 --> 100 is more than sufficient)
  while ((!FLASH.IAPSR.reg.EOP) && ((++countTimeout) < 100));
    
  // lock P-flash and EEPROM again against accidental erase/write
  FLASH.IAPSR.reg.PUL = 0;
  FLASH.IAPSR.reg.DUL = 0;
    
  // enable interrupts
  ENABLE_INTERRUPTS;
  
} // flash_write_byte



// only required for flash block write/erase operations (need to be executed from RAM!)
#if defined(FLASH_BLOCK_OPS)

  // start RAM code section (Cosmic compiler)
  #if defined(__CSMC__)
    #pragma section (RAM_CODE)      // Cosmic
  #endif // __CSMC__
  
  /**
    \fn void flash_erase_block(MEM_POINTER_T addr)
    
    \brief erase 128B block in P-flash (must be executed from RAM)
    
    \param[in] addr   address of block to erase
  
    erase 128B flash block which contains addr. 
    Actual code for block erase needs to be executed from RAM
    For address width see file stm8as.h
    
    Warning: for simplicity no safeguard is used to protect against
    accidental data loss or even overwriting the application -> use with care!
  */
  void flash_erase_block(MEM_POINTER_T addr) {
    
    // disable interrupts
    DISABLE_INTERRUPTS;
    
    // unlock w/e access to P-flash
    FLASH.PUKR.byte = 0x56;
    FLASH.PUKR.byte = 0xAE;
  
    // unlock w/e access to EEPROM
    FLASH.DUKR.byte = 0xAE;
    FLASH.DUKR.byte = 0x56;
  
  
    /////////////// start min. code section in RAM... /////////////////
  
    // enable p-flash erase
    FLASH.CR2.reg.ERASE   = 1;
    FLASH.NCR2.reg.ERASE  = 0;      // complementary register
    while(!FLASH.IAPSR.reg.PUL);
  
    // init erwase by writing 0x00 to 4B word inside page
    write_4B(addr, 0x00000000);
  
    /////////////// ...until here /////////////////
  
  
    // lock P-flash and EEPROM again against accidental erase/write
    FLASH.IAPSR.reg.PUL = 0;
    FLASH.IAPSR.reg.DUL = 0;
      
    // enable interrupts
    ENABLE_INTERRUPTS;
    
  } // flash_erase_block
  
  
  
  /**
    \fn void flash_write_block(MEM_POINTER_T addr, uint8_t ch[])
    
    \brief write 128B block to flash (must be executed from RAM)
    
    \param[in] addr   address to write to (for width see stm8as.h)
    \param[in] buf    128B buffer to write
  
    Fast write 128B block to flash (adress must be divisable by 128)
    Actual code for block write needs to be executed from RAM
    For address width see file stm8as.h
    
    Warning: for simplicity no safeguard is used to protect against
    accidental data loss or even overwriting the application -> use with care!
  */
  void flash_write_block(MEM_POINTER_T addr, uint8_t buf[]) {
    
    uint8_t   i;
    
    // disable interrupts
    DISABLE_INTERRUPTS;
    
    // unlock w/e access to P-flash
    FLASH.PUKR.byte = 0x56;
    FLASH.PUKR.byte = 0xAE;
    
    // unlock w/e access to EEPROM
    FLASH.DUKR.byte = 0xAE;
    FLASH.DUKR.byte = 0x56;
      
    // wait until access granted
    while(!FLASH.IAPSR.reg.PUL);
    while(!FLASH.IAPSR.reg.DUL);
  
  
    /////////////// start min. code section in RAM... /////////////////
  
    // set fast block programming mode
    FLASH.CR2.reg.FPRG  = 1;
    FLASH.NCR2.reg.FPRG = 0;
  
    // copy 128B to latch
    for (i=0; i<128; i++)
      write_1B(addr+i, buf[i]);
  
    /////////////// ...until here /////////////////
  
  
    // lock P-flash and EEPROM again against accidental erase/write
    FLASH.IAPSR.reg.PUL = 0;
    FLASH.IAPSR.reg.DUL = 0;
    
    // enable interrupts
    ENABLE_INTERRUPTS;
    
  } // flash_write_block
  
  
  // end RAM code section (Cosmic compiler)
  #if defined(__CSMC__)
    #pragma section ()
  #endif // __CSMC__

#endif // FLASH_BLOCK_OPS

/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/
