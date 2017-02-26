/**
  \file misc.h
   
  \author G. Icking-Konert
  \date 2008-11-02
  \version 0.1
   
  \brief declaration of misc macros & routines
   
  declaration of macros & routines not really fitting anywhere else

*/

// for including file only once
#ifndef _MISC_H_
#define _MISC_H_


/*-----------------------------------------------------------------------------
    GLOBAL MACROS
-----------------------------------------------------------------------------*/

/// skip warnings for unused parameters, see http://stackoverflow.com/questions/3599160/unused-parameter-warnings-in-c-code
#define UNUSED(x) (void)(x)

/*---
 macros for bitwise r/w access
---*/
#define read_bit(byte, bit)			      (byte & (1 << bit), >> bit)                 ///< read single bit from data
#define set_bit(byte, bit)            byte |= (1 << bit)                          ///< set single bit in data to '1'
#define clear_bit(byte, bit)		      byte &= ~(1 << bit)                         ///< clear single bit in data to '0'
#define write_bit(byte, bit, state)		(state?(byte|=(1<<bit)):(byte&=~(1<<bit)))  ///< set single bit state in data to specified value
#define toggle_bit(byte, bit)		      byte ^= (1 << bit)                          ///< toggle single bit state in data


/*---
 some useful general macros
---*/
#define MIN(a,b)  ((a < b) ?  (a) : (b))
#define MAX(a,b)  ((a > b) ?  (a) : (b))
#define ROUND(a)  ((uint32_t)((float) a + 0.5001))
#define CEIL(a)   ((uint32_t)((float) a + 0.9))


/*-----------------------------------------------------------------------------
    GLOBAL FUNCTIONS
-----------------------------------------------------------------------------*/

/// concatenate 2*8b buffer -> uint16_t (MSB first)
uint16_t    concat_u16(uint8_t *buf);

/// concatenate 2*8b args -> uint16_t (MSB first)
uint16_t    concat2_u16(uint8_t hb, uint8_t lb);

/// concatenate 2*8b buffer -> int16_t (MSB first)
int16_t     concat_s16(uint8_t *buf);

/// concatenate 4*8b buffer -> uint32_t (MSB first)
uint32_t    concat_u32(uint8_t *buf);

/// concatenate 4*8b buffer -> uint32_t (MSB first)
int32_t     concat_s32(uint8_t *buf);

/// get byte idx out of u16b argument (0=LSB)
uint8_t     get_byte_u16(uint16_t value, uint8_t idx);

/// get byte idx out of s16b argument (0=LSB)
uint8_t     get_byte_s16(int16_t value, uint8_t idx);

/// get byte idx out of u32b argument (0=LSB)
uint8_t     get_byte_u32(uint32_t value, uint8_t idx);

/// get byte idx out of s32b argument (0=LSB)
uint8_t     get_byte_s32(int32_t value, uint8_t idx);

/// simple calculation of log2(x); missing in Cosmic & SDCC math lib
uint16_t    log2(float arg);


/*-----------------------------------------------------------------------------
    END OF MODULE DEFINITION FOR MULTIPLE INLUSION
-----------------------------------------------------------------------------*/
#endif // _MISC_H_
