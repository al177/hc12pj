/**
  \file timer3.c
   
  \author G. Icking-Konert
  \date 2013-11-22
  \version 0.1
   
  \brief implementation of timer TIM3 functions/macros for sleep_x and timeout
   
  implementation of timer TIM3 functions for sleep_x and timeout
  via polling
*/

/*----------------------------------------------------------
    INCLUDE FILES
----------------------------------------------------------*/
#include <stdint.h>
#include "stm8as.h"
#include "timer3.h"


/*----------------------------------------------------------
    FUNCTIONS
----------------------------------------------------------*/

/**
  \fn void tim3_init(void)
   
  \brief init timer 3 (sleep_x/timeout)
   
  init timer TIM3 (used for sleep_x and timeout).
*/
void tim3_init(void) {

  // stop the timer
  TIM3.CR1.reg.CEN = 0;

  // disable single-shot mode (causes SW stalls)
  TIM3.CR1.reg.OPM = 0;
  
  // set prescaler to fclk/2^4 -> 1MHz clock -> 1us resolution
  TIM3.PSCR.reg.PSC = 4;
  
  // init to 1ms timeout (write high byte first)
  TIM3.CC1R.byteH = 0x03;
  TIM3.CC1R.byteL = 0xE8;

  // clear status registers
  TIM3.SR1.byte = 0x00;
  TIM3.SR2.byte = 0x00;
  
  // reset counter register (write high byte first)
  TIM3.CNTR.byteH = 0;
  TIM3.CNTR.byteL = 0;
      
  // request register update
  TIM3.EGR.reg.UG = 1;
  
  // disable T3 interrupts
  TIM3.IER.byte = 0x00;

} // tim3_init



/**
  \fn void sleep_ns(uint16_t dt)

  \brief halt code execution for specified 62.5ns units
   
  \param dt halt duration in 62.5ns units
   
  code execution is halted for specified number of 62.5ns units. 
*/
void sleep_ns(uint16_t dt) {
  
  // convert 250ns-->62.5ns
  dt *= 4;

  // stop timer
  TIM3.CR1.reg.CEN = 0;

  // set prescaler to fclk -> 16MHz clock -> 62.5ns resolution
  TIM3.PSCR.reg.PSC = 0;
  
  // set timout to dt*250ns (freq_Hz=fclk/(prescaler*ARR)) (write high byte first)
  TIM3.CC1R.byteH = (uint8_t) (dt >> 8);
  TIM3.CC1R.byteL = (uint8_t) dt;
  
  // reset counter register (write high byte first)
  TIM3.CNTR.byteH = (uint8_t) 0;
  TIM3.CNTR.byteL = (uint8_t) 0;
      
  // request register update
  TIM3.EGR.reg.UG = 1;
  
  // clear status registers
  TIM3.SR1.byte = 0x00;
  
  // start the timer
  TIM3.CR1.reg.CEN = 1;
  
  // wait for overflow (no interrupts!)
  while (!TIM3.SR1.reg.CC1IF);

} // sleep_ns



/**
  \fn void sleep_us(uint16_t dt)

  \brief halt code execution for specified microseconds
   
  \param dt halt duration in us
   
  code execution is halted for specified number of microseconds. 
*/
void sleep_us(uint16_t dt) {
  
  // stop timer
  TIM3.CR1.reg.CEN = 0;

  // set prescaler to fclk/2^4 -> 1MHz clock -> 1us resolution
  TIM3.PSCR.reg.PSC = 4;
  
  // set timout to dt us (freq_Hz=fclk/(prescaler*ARR)) (write high byte first)
  TIM3.CC1R.byteH = (uint8_t) (dt >> 8);
  TIM3.CC1R.byteL = (uint8_t) dt;
  
  // reset counter register (write high byte first)
  TIM3.CNTR.byteH = (uint8_t) 0;
  TIM3.CNTR.byteL = (uint8_t) 0;
      
  // request register update
  TIM3.EGR.reg.UG = 1;
  
  // clear status registers
  TIM3.SR1.byte = 0x00;
  
  // start the timer
  TIM3.CR1.reg.CEN = 1;
  
  // wait for overflow (no interrupts!)
  while (!TIM3.SR1.reg.CC1IF);
    
} // sleep_us



/**
  \fn void sleep_ms(uint16_t dt)

  \brief halt code execution for specified milliseconds
   
  \param dt halt duration in ms
   
  code execution is halted for specified number of milliseconds. 
*/
void sleep_ms(uint16_t dt) {

  // stop timer
  TIM3.CR1.reg.CEN = 0;

  // set prescaler to fclk/2^14 -> ~1kHz clock 
  TIM3.PSCR.reg.PSC = 14;
  
  // set timout to dt ms (freq_Hz=fclk/(prescaler*ARR)) (write high byte first)
  TIM3.CC1R.byteH = (uint8_t) (dt >> 8);
  TIM3.CC1R.byteL = (uint8_t) dt;
  
  // reset counter register (write high byte first)
  TIM3.CNTR.byteH = (uint8_t) 0;
  TIM3.CNTR.byteL = (uint8_t) 0;
  
  // clear status registers
  TIM3.SR1.byte = 0x00;
      
  // request register update
  TIM3.EGR.reg.UG = 1;
  
  // start the timer
  TIM3.CR1.reg.CEN = 1;
  
  // wait for overflow (no interrupts!)
  while (!TIM3.SR1.reg.CC1IF);

} // sleep_ms



/**
  \fn void start_timeout_ms(uint16_t dt)

  \brief start timeout with specified number of ms
   
  \param dt timeout duration in ms (0=forever)
   
  start timeout with specified number of milliseconds. To check for
  timeout query respective timer overflow flag.
*/
void start_timeout_ms(uint16_t dt) {

  // stop timer
  TIM3.CR1.reg.CEN = 0;

  // prescaler to fclk/2^14 -> ~1kHz clock 
  TIM3.PSCR.reg.PSC = 14;
  
  // set auto-reload value to n us (freq_Hz=fclk/(prescaler*ARR)) (write high byte first)
  TIM3.CC1R.byteH = (uint8_t) (dt >> 8);
  TIM3.CC1R.byteL = (uint8_t) dt;
  
  // reset counter register (write high byte first)
  TIM3.CNTR.byteH = 0;
  TIM3.CNTR.byteL = 0;
      
  // request register update
  TIM3.EGR.reg.UG = 1;
  
  // clear status registers
  TIM3.SR1.byte = 0x00;
  
  // reset timeout flag
  TIM3.SR1.reg.CC1IF = 0;
  
  // start the timer only if dt!=0
  if (dt != 0)
    TIM3.CR1.reg.CEN = 1;
  else
    TIM3.CR1.reg.CEN = 0;
  
} // start_timeout_ms



/**
  \fn void stop_timeout_ms(void)

  \brief top timeout timer
   
  stop the timeout timer to avoid conflicts e.g. with sleep_x.
*/
void stop_timeout_ms() {

  // stop timer
  TIM3.CR1.reg.CEN = 0;

  // set prescaler to fclk/2^4 -> 1MHz clock -> 1us resolution
  TIM3.PSCR.reg.PSC = 4;
  
} // stop_timeout_ms

/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/
