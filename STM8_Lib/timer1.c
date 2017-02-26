/**
  \file timer1.c
   
  \author G. Icking-Konert
  \date 2013-11-22
  \version 0.1
   
  \brief implementation of TIM1 functions/macros (PWM channel) 
   
  implementation of timer TIM1 functions for generating and measuring 
  PWM signals, receiving SENT protocol etc.
*/

/*----------------------------------------------------------
    INCLUDE FILES
----------------------------------------------------------*/
#include <stdint.h>
#include <math.h>
#include "stm8as.h"
#include "misc.h"
#include "timer1.h"
#include "gpio.h"
#include "timer3.h"
#include "error_codes.h"


/*----------------------------------------------------------
    FUNCTIONS
----------------------------------------------------------*/

/**
  \fn void tim1_init(void)
   
  \brief init timer 1 (PWM channel)
   
  init timer TIM1 to defaut values (used for PWM)
*/
void tim1_init(void) {

  // configure pin TIM1_CC1 (=PC1) as input float. Deactivate interrupt first, just in case...
  PORT_C.CR1.bit.b1 = 0;       // input: 0=no exint, 1=exint; output: 0=2MHz slope, 1=10MHz slope
  PORT_C.CR2.bit.b1 = 0;       // input: 0=float, 1=pull-up; output: 0=open-drain, 1=push-pull
  PORT_C.DDR.bit.b1 = 0;       // input(=0) or output(=1)
  
  // reset all registers to reset value
  TIM1.CR1.byte   = TIM1_CR1_RESET_VALUE;
  TIM1.CR2.byte   = TIM1_CR2_RESET_VALUE;
  TIM1.SMCR.byte  = TIM1_SMCR_RESET_VALUE;
  TIM1.ETR.byte   = TIM1_ETR_RESET_VALUE;
  TIM1.IER.byte   = TIM1_IER_RESET_VALUE;
  TIM1.SR1.byte   = TIM1_SR1_RESET_VALUE;
  TIM1.SR2.byte   = TIM1_SR2_RESET_VALUE;
  TIM1.EGR.byte   = TIM1_EGR_RESET_VALUE;
  TIM1.CCMR1.byte = TIM1_CCMR1_RESET_VALUE;
  TIM1.CCMR2.byte = TIM1_CCMR2_RESET_VALUE;
  TIM1.CCMR3.byte = TIM1_CCMR3_RESET_VALUE;
  TIM1.CCMR4.byte = TIM1_CCMR4_RESET_VALUE;
  TIM1.CCER1.byte = TIM1_CCER1_RESET_VALUE;
  TIM1.CCER2.byte = TIM1_CCER2_RESET_VALUE;
  TIM1.CNTR.byteH = TIM1_CNTRH_RESET_VALUE;
  TIM1.CNTR.byteL = TIM1_CNTRL_RESET_VALUE;
  TIM1.PSCR.byteH = TIM1_PSCRH_RESET_VALUE;
  TIM1.PSCR.byteL = TIM1_PSCRL_RESET_VALUE;
  TIM1.ARR.byteH  = TIM1_ARRH_RESET_VALUE;
  TIM1.ARR.byteL  = TIM1_ARRL_RESET_VALUE;
  TIM1.RCR.byte   = TIM1_RCR_RESET_VALUE;
  TIM1.CC1R.byteH = TIM1_CCR1H_RESET_VALUE;
  TIM1.CC1R.byteL = TIM1_CCR1L_RESET_VALUE;
  TIM1.CC2R.byteH = TIM1_CCR2H_RESET_VALUE;
  TIM1.CC2R.byteL = TIM1_CCR2L_RESET_VALUE;
  TIM1.CC3R.byteH = TIM1_CCR3H_RESET_VALUE;
  TIM1.CC3R.byteL = TIM1_CCR3L_RESET_VALUE;
  TIM1.CC4R.byteH = TIM1_CCR4H_RESET_VALUE;
  TIM1.CC4R.byteL = TIM1_CCR4L_RESET_VALUE;
  TIM1.BKR.byte   = TIM1_BKR_RESET_VALUE;
  TIM1.DTR.byte   = TIM1_DTR_RESET_VALUE;
  TIM1.OISR.byte  = TIM1_OISR_RESET_VALUE;
  
  // force register update
  TIM1.EGR.reg.UG = 1;
  
} // tim1_init



/**
  \fn void tim1_set_pwm(uint32_t centHz, uint16_t deciPrc)
   
  \brief generate PWM signal on TIM1_CC1 (=PC1)
   
  \param centHz   frequency in 0.01Hz
  \param deciPrc  duty cycle in 0.1% (0..1000)
  
  generate a PWM signal of given frequency and duty cycle (with high polarity).
  A frequency of 0 deactivates the respective output.

*/
void tim1_set_pwm(uint32_t centHz, uint16_t deciPrc) {

  uint16_t    pre;    // 16b timer prescaler
  uint16_t    ARR;    // 16b reload value
  uint16_t    CCR;    // 16b compare value -> duty cycle
  
  
  //////////////
  // calculate timer parameter
  //////////////

  // calculate max. prescaler to maximize ARR resolution. Condition: centHz > 100*(16e6/((pre+1)*(2^16-1)))
  pre = (uint16_t) CEIL(24414.4350347143/(float)centHz - 1.0);
  pre = (uint16_t) MAX(pre, 0);
  pre = (uint16_t) MIN(pre, 65535);
  
  // set freq to spec. value (centHz = 100*fCPU/((pre+1)*(ARR+1))
  ARR = (uint16_t) ROUND(16e8/((float)(pre+1)*(float)(centHz)) - 1.0);
  ARR = (uint16_t) MAX(ARR, 0);
  ARR = (uint16_t) MIN(ARR, 65535);
  
  // calculate compare value (CCR = ARR*(deciPrc/1000))
  if (deciPrc >= 1000)
    CCR = ARR + 1;
  else
    CCR = (uint16_t) ROUND((float) ARR * (float) deciPrc / 1000.0);
  CCR = (uint16_t) MAX(CCR, 0);
  CCR = (uint16_t) MIN(CCR, 65535);

  
  //////////////
  // generate PWM
  //////////////

  // reset timer registers (just to make sure)
  tim1_init();
  
  // config TIM1 pin (=TIM1CC1 =PC1) as output
  PORT_C.DDR.bit.b1 = 1;       // input(=0) or output(=1)
  PORT_C.CR1.bit.b1 = 1;       // input: 0=float, 1=pull-up; output: 0=open-drain, 1=push-pull
  PORT_C.CR2.bit.b1 = 1;       // input: 0=no exint, 1=exint; output: 0=2MHz slope, 1=10MHz slope
  
  // exit on zero frequency specified. Above Init deactivates timer 1
  if (centHz == 0)
    return;


  // set TIM1 prescaler f = fcpu/(pre+1) with pre in [0..2^16-1]
  TIM1.PSCR.byteH = (uint8_t) (pre >> 8);
  TIM1.PSCR.byteL = (uint8_t) pre;

  // set reload value (fPWM = fCPU/((pre+1)*(ARR+1))
  TIM1.ARR.byteH = (uint8_t) (ARR >> 8);
  TIM1.ARR.byteL = (uint8_t) ARR;

  // set capture/compare value for duty cycle (DC=CCR/ARR)
  TIM1.CC1R.byteH = (uint8_t) (CCR >> 8);
  TIM1.CC1R.byteL = (uint8_t) CCR;

  // set active polarity to high (=0) (1=low polarity)
  TIM1.CCER1.reg.CC1P = 0;

  // main output enable
  TIM1.BKR.reg.MOE = 1;

  // enable output
  TIM1.CCER1.reg.CC1E = 1;

  // set PWM mode 1
  TIM1.CCMR1.byte = 0b01101000;

  // request register update
  TIM1.EGR.reg.UG = 1;
  
  // activate timer
  TIM1.CR1.reg.CEN = 1;       // start the timer

} // tim1_set_pwm
   


/**
  \fn void tim1_set_pwm_complement(uint32_t centHz, uint16_t deciPrc)
   
  \brief generate complementary PWM signal on TIM1_CC1(=PC1) and TIM1_NCC1(=PB0)
   
  \param centHz   frequency in 0.01Hz
  \param deciPrc  duty cycle in 0.1% (0..1000)
  
  generate a complementary PWM signal of given frequency and duty cycle (with high polarity).
  A frequency of 0 deactivates both outputs.

*/
void tim1_set_pwm_complement(uint32_t centHz, uint16_t deciPrc) {

  uint16_t    pre;    // 16b timer prescaler
  uint16_t    ARR;    // 16b reload value
  uint16_t    CCR;    // 16b compare value -> duty cycle
  

  //////////////
  // calculate timer parameter
  //////////////

  // calculate max. prescaler to maximize ARR resolution. Condition: centHz > 100*(16e6/((pre+1)*(2^16-1)))
  pre = (uint16_t) CEIL(24414.4350347143/(float)centHz - 1.0);
  pre = (uint16_t) MAX(pre, 0);
  pre = (uint16_t) MIN(pre, 65535);
  
  // set freq to spec. value (centHz = 100*fCPU/((pre+1)*(ARR+1))
  ARR = (uint16_t) ROUND(16e8/((float)(pre+1)*(float)(centHz)) - 1.0);
  ARR = (uint16_t) MAX(ARR, 0);
  ARR = (uint16_t) MIN(ARR, 65535);
  
  // calculate compare value (CCR = ARR*(deciPrc/1000))
  if (deciPrc >= 1000)
    CCR = ARR + 1;
  else
    CCR = (uint16_t) ROUND((float) ARR * (float) deciPrc / 1000.0);
  CCR = (uint16_t) MAX(CCR, 0);
  CCR = (uint16_t) MIN(CCR, 65535);

  
  //////////////
  // generate PWM
  //////////////

  // reset timer registers (just to make sure)
  tim1_init();
  
  // config TIM1 pin (=TIM1CC1 =PC1) as output
  PORT_C.DDR.bit.b1 = 1;       // input(=0) or output(=1)
  PORT_C.CR1.bit.b1 = 1;       // input: 0=float, 1=pull-up; output: 0=open-drain, 1=push-pull
  PORT_C.CR2.bit.b1 = 1;       // input: 0=no exint, 1=exint; output: 0=2MHz slope, 1=10MHz slope
  
  // config TIM1 pin (=TIM1NCC1 =PB0) as output
  PORT_B.DDR.bit.b0 = 1;       // input(=0) or output(=1)
  PORT_B.CR1.bit.b0 = 1;       // input: 0=float, 1=pull-up; output: 0=open-drain, 1=push-pull
  PORT_B.CR2.bit.b0 = 1;       // input: 0=no exint, 1=exint; output: 0=2MHz slope, 1=10MHz slope
  
  // exit on zero frequency specified. Above Init deactivates timer 1
  if (centHz == 0)
    return;
  
  
  // set TIM1 prescaler f = fcpu/(pre+1) with pre in [0..2^16-1]
  TIM1.PSCR.byteH = (uint8_t) (pre >> 8);
  TIM1.PSCR.byteL = (uint8_t) pre;

  // set reload value (fPWM = fCPU/((pre+1)*(ARR+1))
  TIM1.ARR.byteH = (uint8_t) (ARR >> 8);
  TIM1.ARR.byteL = (uint8_t) ARR;

  // set capture/compare value for duty cycle (DC=CCR/ARR)
  TIM1.CC1R.byteH = (uint8_t) (CCR >> 8);
  TIM1.CC1R.byteL = (uint8_t) CCR;

  // set active polarity to high (=0) (1=low polarity)
  TIM1.CCER1.reg.CC1NP = 1;
  TIM1.CCER1.reg.CC1P  = 0;

  // main output enable
  TIM1.BKR.reg.MOE = 1;
  
  // enable output
  TIM1.CCER1.reg.CC1NE = 1;
  TIM1.CCER1.reg.CC1E  = 1;

  // set PWM mode 1
  TIM1.CCMR1.byte = 0b01101000;

  // request register update
  TIM1.EGR.reg.UG = 1;
  
  // activate timer
  TIM1.CR1.reg.CEN = 1;       // start the timer

} // tim1_set_pwm_complement



/**
  \fn void tim1_get_pwm(uint32_t *centHz, uint16_t *deciPrc)
   
  \brief measure PWM signal on TIM1_CC1 (=PC1)
   
  \param[out] centHz   frequency in 0.01Hz (100..tbd)
  \param[out] decPrc   duty cycle in 0.1% (0..1000)
  
  measure a PWM input signal on TIM1_CC1 (=PC1) (frequency & duty cycle). 
  Perform automatic range adaptation (=prescaler).
*/
void tim1_get_pwm(uint32_t *centHz, uint16_t *deciPrc) {

  uint16_t    pre, CC1, CC2;
  uint8_t     flag, h=0, l=0;
  
  // reset timer registers and reset GPIO to input
  tim1_init();
        
  // configure timer behaviour
  TIM1.CR1.reg.UDIS = 0;
  TIM1.CR1.reg.URS  = 1;        // set SR1_UIF bit only on overflow

  // select input and set filter=0
  TIM1.CCMR1.regIn.CC1S = 1;      // TIM1.CC1 -> PC1
  TIM1.CCMR2.regIn.CC2S = 2;      // TIM1.CC2 -> PC1
  
  // capture CC1 on rising (=0),and CC2 on falling (=1) edge
  TIM1.CCER1.reg.CC1P = 0;
  TIM1.CCER1.reg.CC2P = 1;
  
  // select slave mode & input trigger (reset CNTR on trigger event)
  TIM1.SMCR.byte = 0x54;

  // enable capcom for CC1 & CC2
  TIM1.CCER1.reg.CC1E = 1;
  TIM1.CCER1.reg.CC2E = 1;

  // adapt clock prescaler
  flag  = 0;
  pre   = 0;
  do {

    // set prescaler f = fcpu/(pre+1) with pre in [0..2^16-1]
    TIM1.PSCR.byteH = (uint8_t) (pre >> 8);             // set clock prescaler
    TIM1.PSCR.byteL = (uint8_t) pre;

    // wait for 2nd rising edge or timer overflow; 1st edge automatically resets counter
    TIM1.SR1.byte     = 0x00;                           // reset status registers
    TIM1.SR2.byte     = 0x00;
    TIM1.EGR.reg.UG   = 1;                              // request register update
    TIM1.CR1.byte     = 0x05;                           // start the timer (keep URS=1)
    while (!(TIM1.SR1.byte&0x03));                      // wait for 1st rising edge or timeout
    TIM1.CC1R.byteH   = 0x00;                           // reset CC1IF bit
    TIM1.CC1R.byteL   = 0x00;
    while ((!(TIM1.SR2.byte&0x02)) && (!(TIM1.SR1.byte&0x01)));  // wait for 2nd rising edge or timeout
    TIM1.CR1.byte     = 0x04;                           // stop the timer (keep URS=1)
    
    // on timer overflow reduce clock or exit with static level
    if (TIM1.SR1.reg.UIF) {
        
      // max prescaler for 1Hz reached --> exit
      if (pre >= 245)
        flag = 1;
 
      // half timer frequency (f = fcpu / (pre+1) with pre in [0..2^16-1])
      else
        pre = (pre + 1) * 2 - 1;
     
    } // timer overflow

    // no overflow --> done
    else
      flag = 2;
      
  } while (!flag);
           

  // capture succeeded --> get final prescaler and both capture values
  if (flag == 2) {
    CC1  = (uint16_t) (TIM1.CC1R.byteH << 8);
    CC1 += (uint16_t) (TIM1.CC1R.byteL);
    CC2  = (uint16_t) (TIM1.CC2R.byteH << 8);
    CC2 += (uint16_t) (TIM1.CC2R.byteL);
    
    // correct for SW latency (empirically)
    if (pre == 0){
      CC1 += 1;      
      CC2 += 1;
    }
    
    // convert to frequency [0.01Hz] and duty cycle [0.1%]
    *centHz  = (uint32_t) ROUND(16e8/(float)(pre+1) / (float) (CC1 + 1));
    *deciPrc = (uint16_t) ROUND((float) CC2 / (float) CC1 * 1000.0);
      
  } // if capture succeeded
  
  // timer overflow (flag==1) --> static signal
  else {
    *centHz = 100;                  // set arbitrary frequency to 1Hz
    if (GPIO_READ(PORT_C, PIN_1))   // set duty cycle according to GPIO level
      *deciPrc = 1000;
    else
      *deciPrc = 0;
  }
  
  // just to be sure re-init timer & GPIO
  tim1_init();

} // tim1_get_pwm



/**
  \fn uint8_t tim1_receive_SENT(uint8_t *status, uint16_t *data_1, uint16_t *data_2, uint16_t timeout)
   
  \brief receive data via SENT protocol on TIM1_CC1 (=PC1)

  \param[out] status  status nibble
  \param[out] data_1  12b data 1
  \param[out] data_2  12b data 2
  \param[in]  timeout communication timeout [ms]
  
  \return data received (=0), timeout (=1) 

  read data via SENT protocol with a timeout of 5ms
  For a description of SENT, see e.g. 
    http://www.hanser-automotive.de/fileadmin/heftarchiv/2004/28135.pdf  or
    http://www.elektroniknet.de/automotive/technik-know-how/bauelemente/article/75190/0/Nutzung_eines_Hall-Sensors_in_Verbindung_mit_einem_8-bit-Controller/
  
*/
uint8_t tim1_receive_SENT(uint8_t *status, uint16_t *data_1, uint16_t *data_2, uint16_t timeout) {

  uint8_t    nibble[8];                    // status, 2x nibbles 1..3, crc 
  uint8_t    checksum, i; 
  uint8_t    nibbleCount=0;                // nibble counter (0..7)
  uint8_t    durationSync=0;               // duration of sync interval
  uint8_t    duration;                     // duration of current interval
  uint8_t    oldCapture = 0;               // last capture value
  const uint8_t CRCLookup[16] = {0, 13, 7, 10, 14, 3, 9, 4, 1, 12, 6, 11, 15, 2, 8, 5}; 
  const uint8_t TLookup[] = {0,0,0, 1,1,1, 2,2,2, 3,3,3, 4,4,4, 5,5,5, 6,6,6, 7,7,7, 8,8,8, \
                             9,9,9, 10,10,10, 11,11,11, 12,12,12, 13,13,13, 14,14,14, 15,15,15}; 
  
  // init data
  for (i=0; i<8; i++)
    nibble[i] = 0; 
  *status = 0;
  *data_1 = 0; 
  *data_2 = 0; 
  
  // config TIM1 pin (=TIM1CC1 =PC1) as input pull-up
  PORT_C.DDR.bit.b1 = 0;       // input(=0) or output(=1)
  PORT_C.CR1.bit.b1 = 1;       // input: 0=float, 1=pull-up; output: 0=open-drain, 1=push-pull
  PORT_C.CR2.bit.b1 = 0;       // input: 0=no exint, 1=exint; output: 0=2MHz slope, 1=10MHz slope

  // stop timer
  TIM1.CR1.reg.CEN = 0;

  // set prescaler for 1us resolution (fClk=16MHz/(n+1) -> 1us)
  TIM1.PSCR.byteH = 0;
  TIM1.PSCR.byteL = 15;

  // disable capcom for CC1 (reset CCE)
  TIM1.CCER1.reg.CC1E = 0;
  
  // select input and set filter=0
  TIM1.CCMR1.regIn.CC1S = 1;      // TIM1.CC1 -> PC1, no filter

  // capture CC1 on falling (=1) edge and enable
  TIM1.CCER1.reg.CC1P = 1;
  TIM1.CCER1.reg.CC1E = 1;

  // request register update
  TIM1.EGR.reg.UG = 1;

  // start the timer
  TIM1.CR1.reg.CEN = 1;


  ////////////
  // loop until frame received or timeout
  ////////////
  start_timeout_ms(timeout);                                        
  do {
      
    // wait for falling edge or timeout
    while ((!TIM1.SR1.reg.CC1IF) && (!check_timeout()));

    // get duration [us]
    duration = (uint8_t) (TIM1.CC1R.byteL - oldCapture);
 
    // get TIM1CC1 low byte, clears interrupt flag 
    oldCapture = TIM1.CC1R.byteL; 
    
    // if received pulse is 168us(+/-25%) -> sync nibble 
    if ((duration > 126) && (duration < 210)) {
      
      durationSync = duration;      // store sync duration
      nibbleCount = 0;              // reset nibble counter         
  
    } // sync nibble
      
      
    // not a sync nibble -> store
    else if (durationSync != 0) {
        
      // scale measured duration and subtract 35us offset
      duration = (uint8_t) (((uint16_t)(168*duration + (durationSync/2))) / ((uint16_t)durationSync) - 35);
     
      // valid nibble
      if (duration <= 47) 
        nibble[nibbleCount++] = *(TLookup+duration); 
  
      // not a valid nibble -> abort frame
      else {
        durationSync = 0;
        nibbleCount  = 0; 
      }
  
    } // data nibble
  
  // until frame done or timeout
  } while ((nibbleCount != 8) && (!check_timeout()));
  
  
  // on SENT timeout set error flag
  if (check_timeout()) {
    stop_timeout_ms(); 
    return(ERROR_TIMOUT);
  }

  // reset timeout timer
  stop_timeout_ms(); 

  
  // calculate CRC
  checksum = 5; 
  for (i=0; i<7; i++) { 
    checksum = (uint8_t) (checksum ^ nibble[i]); 
    checksum = CRCLookup[checksum]; 
  } 
   
  // checksum error
  if (checksum != nibble[7]) {
    return(ERROR_CHK);
  }

  // extract data
  *status = nibble[0];
  *data_1 = (((uint16_t) nibble[1]) << 8 | (uint16_t) (nibble[2] << 4 | nibble[3])); 
  *data_2 = (((uint16_t) nibble[4]) << 8 | (uint16_t) (nibble[5] << 4 | nibble[6])); 
  
  return(SUCCESS);

} // receive_SENT


/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/
