/**
  \file misc.c
   
  \author G. Icking-Konert
  \date 2008-11-02
  \version 0.1
   
  \brief implementation of misc macros & routines
   
  implementation of macros & routines not really fitting anywhere else

*/

/////////////
// headers
/////////////

#include <stdint.h>
#include <math.h>
#include "misc.h"


/**
  \fn uint16_t concat_u16(uint8_t *buf)
  
  \brief concatenate 2*8b buffer -> uint16_t
  
  \param[in]  buf   buffer containing data (Rx[0]=MSB; Rx[1]=LSB)

  \return resulting unsigned short value

  concatenate 2 unsigned chars from buffer to one value of type uint16_t
*/
uint16_t concat_u16(uint8_t *buf) {
  
  uint16_t  value;

  // use endian independent (manual) conversion
  value = (((uint16_t) buf[0]) << 8) | ((uint16_t) buf[1]);

  return(value);

} // concat_u16

  
  
/**
  \fn uint16_t concat2_u16(uint8_t hb, uint8_t lb)
  
  \brief concatenate 2*8b args -> uint16_t
  
  \param[in]  hb   high byte
  \param[in]  lb   low byte 

  \return resulting unsigned short value

  concatenate 2 unsigned chars from arguments to one value of type uint16_t
*/
uint16_t concat2_u16(uint8_t hb, uint8_t lb) {
  
  volatile uint16_t     result;
  
  result = (((uint16_t)hb) << 8) | (((uint16_t) lb) & 0x00FF);
  return(result);

} // concat2_u16

  
  
/**
  \fn int16_t concat_s16(uint8_t *buf)
  
  \brief concatenate 2*8b buffer -> int16_t
  
  \param[in]  buf   buffer containing data (Rx[0]=MSB; Rx[1]=LSB)

  \return resulting signed short value

  concatenate 2 unsigned chars from buffer to one value of type int16_t
*/
int16_t concat_s16(uint8_t *buf) {
  
  int16_t  value;

  // use endian independent (manual) conversion
  value = (((int16_t) buf[0]) << 8) | ((int16_t) buf[1]);

  return(value);

} // concat_u16

  
  
/**
  \fn uint32_t concat_u32(uint8_t *buf)
  
  \brief concatenate 4*B -> uint32_t
  
  \param[in]  buf   buffer containing data (Rx[0]=MSB; Rx[3]=LSB)

  \return resulting unsigned 32b int value

  concatenate 4 unsigned chars to one value of type uint32_t
*/
uint32_t concat_u32(uint8_t *buf) {
  
  uint32_t   value;

  // use endian independent (manual) conversion
  value = (((uint32_t) buf[0]) << 24) | (((uint32_t) buf[1]) << 16) | (((uint32_t) buf[2]) << 8) | ((uint32_t) buf[3]);

  return(value);

} // concat_u32


  
  
/**
  \fn int32_t concat_s32(uint8_t *buf)
  
  \brief concatenate 4*B -> int32_t
  
  \param[in]  buf   buffer containing data (Rx[0]=MSB; Rx[3]=LSB)

  \return resulting signed 32b int value

  concatenate 4 unsigned chars to one value of type int32_t
*/
int32_t concat_s32(uint8_t *buf) {
  
  int32_t   value;

  // use endian independent (manual) conversion
  value = (((int32_t) buf[0]) << 24) | (((int32_t) buf[1]) << 16) | (((int32_t) buf[2]) << 8) | ((int32_t) buf[3]);

  return(value);

} // concat_s32
  
  
  
/**
  \fn uint8_t get_byte_u16(uint16_t value, uint8_t idx)
  
  \brief get byte idx out of u16b argument (0->LSB)
  
  \param[in]  value   input value 
  \param[in]  idx     index [1..size] of byte to get

  \return byte idx of input value
  
  get byte idx out of u16b argument (0->LSB)
*/
uint8_t get_byte_u16(uint16_t value, uint8_t idx) {
  
  // use endian independent (manual) conversion
  return((uint8_t) (value >> (8 * idx)));

} // get_byte_u16

  
  
/**
  \fn uint8_t get_byte_s16(int16_t value, uint8_t idx)
  
  \brief get byte idx out of s16b argument (0->LSB)
  
  \param[in]  value   input value 
  \param[in]  idx     index [1..size] of byte to get

  \return byte idx of input value
  
  get byte idx out of s16b argument (0->LSB)
*/
uint8_t get_byte_s16(int16_t value, uint8_t idx) {
  
  // use endian independent (manual) conversion
  return((uint8_t) (value >> (8 * idx)));

} // get_byte_s16

  
  
/**
  \fn uint8_t get_byte_u32(uint32_t value, uint8_t idx)
  
  \brief get byte idx out of 32b argument (0->LSB)
  
  \param[in]  value   input value 
  \param[in]  idx     index [1..size] of byte to get

  \return byte idx of input value
  
  get byte idx out of 32b argument (0->LSB)
*/
uint8_t get_byte_u32(uint32_t value, uint8_t idx) {
  
  // use endian independent (manual) conversion
  return((uint8_t) (value >> (8 * idx)));

} // get_byte_u32

  
  
/**
  \fn uint8_t get_byte_s32(int32_t value, uint8_t idx)
  
  \brief get byte idx out of 32b argument (0->LSB)
  
  \param[in]  value   input value 
  \param[in]  idx     index [1..size] of byte to get

  \return byte idx of input value
  
  get byte idx out of 32b argument (0->LSB)
*/
uint8_t get_byte_s32(int32_t value, uint8_t idx) {
  
  // use endian independent (manual) conversion
  return((uint8_t) (value >> (8 * idx)));

} // get_byte_s32

  
  
/**
  \fn uint16_t log2(float arg)
  
  \brief simple calculation of log2(x)
  
  \param[in]  arg   log2 argument. Has to be positive.

  \return result of log2(arg)
  
  simple calculation of log2(x) in O(log(N)). Basically
  compare argument vs. increasing 1<<n
*/
uint16_t log2(float arg) {

  uint16_t result = 0;
  
  while ((float)(1L<<result) < arg)
    result++;
    
  return(result);

} // log2

// end of file
