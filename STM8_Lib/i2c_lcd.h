/**
  \file i2c_lcd.h
   
  \author G. Icking-Konert
  \date 2013-11-22
  \version 0.1
   
  \brief declaration of 2x16 LCD output functions/macros
   
  declaration of functions for printing strings via I2C to 2x16 LCD
  Batron BTHQ21605V-COG-FSRE-I2C (Farnell 1220409).
  Connect LCD I2C bus to STM8 SCL/SDA, and LCD reset pin to STM8 pin PE3
*/

/*-----------------------------------------------------------------------------
    MODULE DEFINITION FOR MULTIPLE INCLUSION
-----------------------------------------------------------------------------*/
#ifndef _I2C_LCD_H_
#define _I2C_LCD_H_


/*-----------------------------------------------------------------------------
    DEFINITION OF GLOBAL MACROS/#DEFINES
-----------------------------------------------------------------------------*/

#include <stdint.h>

// I2C addresses of LCD display
#define LCD_ADDR_I2C  59


/*-----------------------------------------------------------------------------
    DECLARATION OF GLOBAL FUNCTIONS
-----------------------------------------------------------------------------*/

/// initialize I2C, and reset LCD
uint8_t   lcd_init(void);

/// clear LCD
void      lcd_clear(void);

/// print string to LCD diplay
void      lcd_print(uint8_t line, uint8_t col, char *s);


/*-----------------------------------------------------------------------------
    END OF MODULE DEFINITION FOR MULTIPLE INLUSION
-----------------------------------------------------------------------------*/
#endif // _I2C_LCD_H_
