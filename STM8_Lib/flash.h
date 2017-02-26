/**
  \file flash.h
   
  \author G. Icking-Konert
  \date 2013-11-22
  \version 0.1
   
  \brief declaration of flash read/write routines
   
  declaration of flash read/write routines and macros.
  RAM routines for block operations are in RAM_routines.c
*/

/*-----------------------------------------------------------------------------
    MODULE DEFINITION FOR MULTIPLE INCLUSION
-----------------------------------------------------------------------------*/
#ifndef _FLASH_H_
#define _FLASH_H_

/*-----------------------------------------------------------------------------
    INCLUDE FILES
-----------------------------------------------------------------------------*/
#include <stdint.h>
#include "stm8as.h"
#include "memory_access.h"


/*-----------------------------------------------------------------------------
    DEFINITION OF GLOBAL MACROS/#DEFINES
-----------------------------------------------------------------------------*/

//////
// addresses of option bytes
//////
#define OPT0   (OPT_BaseAddress+0x00)  //!< Option byte 0: Read-out protection (not accessible in IAP mode)
#define OPT1   (OPT_BaseAddress+0x01)  //!< Option byte 1: User boot code */
#define NOPT1  (OPT_BaseAddress+0x02)  //!< Complementary Option byte 1 */
#define OPT2   (OPT_BaseAddress+0x03)  //!< Option byte 2: Alternate function remapping */
#define NOPT2  (OPT_BaseAddress+0x04)  //!< Complementary Option byte 2 */
#define OPT3   (OPT_BaseAddress+0x05)  //!< Option byte 3: Watchdog option */
#define NOPT3  (OPT_BaseAddress+0x06)  //!< Complementary Option byte 3 */
#define OPT4   (OPT_BaseAddress+0x07)  //!< Option byte 4: Clock option */
#define NOPT4  (OPT_BaseAddress+0x08)  //!< Complementary Option byte 4 */
#define OPT5   (OPT_BaseAddress+0x09)  //!< Option byte 5: HSE clock startup */
#define NOPT5  (OPT_BaseAddress+0x0A)  //!< Complementary Option byte 5 */
#define RES1   (OPT_BaseAddress+0x0B)  //!< Reserved Option byte*/
#define RES2   (OPT_BaseAddress+0x0C)  //!< Reserved Option byte*/
#define OPT7   (OPT_BaseAddress+0x0D)  //!< Option byte 7: flash wait states */
#define NOPT7  (OPT_BaseAddress+0x0E)  //!< Complementary Option byte 7 */

#define OPT8   (OPT_BaseAddress+0x10)  //!< Option byte 8:  TMU key 1 */
#define OPT9   (OPT_BaseAddress+0x11)  //!< Option byte 9:  TMU key 2 */
#define OPT10  (OPT_BaseAddress+0x12)  //!< Option byte 10: TMU key 3 */
#define OPT11  (OPT_BaseAddress+0x13)  //!< Option byte 11: TMU key 4 */
#define OPT12  (OPT_BaseAddress+0x14)  //!< Option byte 12: TMU key 5 */
#define OPT13  (OPT_BaseAddress+0x15)  //!< Option byte 13: TMU key 6 */
#define OPT14  (OPT_BaseAddress+0x16)  //!< Option byte 14: TMU key 7 */
#define OPT15  (OPT_BaseAddress+0x17)  //!< Option byte 15: TMU key 8 */
#define OPT16  (OPT_BaseAddress+0x18)  //!< Option byte 16: TMU access failure counter */

#define OPT17  (OPT_BaseAddress+0x7E)  //!< Option byte 17: BSL activation */
#define NOPT17 (OPT_BaseAddress+0x7F)  //!< Complementary Option byte 17 */


/*-----------------------------------------------------------------------------
    DEFINITION OF GLOBAL FUNCTIONS
-----------------------------------------------------------------------------*/

// flash write/erase routines
void 			flash_OPT_default(void);
uint8_t   flash_write_option_byte(uint16_t addr, uint8_t data);   ///< write option byte (all in 16-bit range)
void      flash_write_byte(MEM_POINTER_T addr, uint8_t data);     ///< write 1B to P-flash or EEPROM
void      flash_erase_block(MEM_POINTER_T addr);                  ///< erase 128B block in flash (must be executed from RAM)
void      flash_write_block(MEM_POINTER_T addr, uint8_t buf[]);   ///< write 128B block to flash (must be executed from RAM)

/*-----------------------------------------------------------------------------
    END OF MODULE DEFINITION FOR MULTIPLE INLUSION
-----------------------------------------------------------------------------*/
#endif // _FLASH_H_
