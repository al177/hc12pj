/**
  \file timer2.c
   
  \author G. Icking-Konert
  \date 2013-11-22
  \version 0.1
   
  \brief implementation of TIM2 functions/macros (PWM channel) 
   
  implementation of timer TIM2 functions for generating and measuring 
  PWM signals, receiving SENT protocol etc.
*/

/*----------------------------------------------------------
    INCLUDE FILES
----------------------------------------------------------*/
#include <stdint.h>
#include <math.h>
#include "stm8as.h"
#include "misc.h"
#include "timer2.h"
#include "gpio.h"
#include "timer3.h"
#include "error_codes.h"


/*----------------------------------------------------------
    FUNCTIONS
----------------------------------------------------------*/

/**
  \fn void tim2_init(void)
   
  \brief init timer 2 (PWM channel)
   
  init timer TIM2 to defaut values (used for PWM)
*/
void tim2_init(void) {

  // configure pin TIM2_CC2 (=PD3) as input float. Deactivate interrupt first, just in case...
  PORT_D.CR1.bit.b3 = 0;       // input: 0=no exint, 1=exint; output: 0=2MHz slope, 1=10MHz slope
  PORT_D.CR2.bit.b3 = 0;       // input: 0=float, 1=pull-up; output: 0=open-drain, 1=push-pull
  PORT_D.DDR.bit.b3 = 0;       // input(=0) or output(=1)
  
  // reset all registers to reset value
  TIM2.CR1.byte   = TIM2_CR1_RESET_VALUE;
  TIM2.IER.byte   = TIM2_IER_RESET_VALUE;
  TIM2.SR1.byte   = TIM2_SR1_RESET_VALUE;
  TIM2.SR2.byte   = TIM2_SR2_RESET_VALUE;
  TIM2.EGR.byte   = TIM2_EGR_RESET_VALUE;
  TIM2.CCMR1.byte = TIM2_CCMR1_RESET_VALUE;
  TIM2.CCMR2.byte = TIM2_CCMR2_RESET_VALUE;
  TIM2.CCMR3.byte = TIM2_CCMR3_RESET_VALUE;
  TIM2.CCER1.byte = TIM2_CCER1_RESET_VALUE;
  TIM2.CCER2.byte = TIM2_CCER2_RESET_VALUE;
  TIM2.CNTR.byteH = TIM2_CNTRH_RESET_VALUE;
  TIM2.CNTR.byteL = TIM2_CNTRL_RESET_VALUE;
  TIM2.PSCR.byte  = TIM2_PSCR_RESET_VALUE;
  TIM2.ARR.byteH  = TIM2_ARRH_RESET_VALUE;
  TIM2.ARR.byteL  = TIM2_ARRL_RESET_VALUE;
  TIM2.CC1R.byteH = TIM2_CCR1H_RESET_VALUE;
  TIM2.CC1R.byteL = TIM2_CCR1L_RESET_VALUE;
  TIM2.CC2R.byteH = TIM2_CCR2H_RESET_VALUE;
  TIM2.CC2R.byteL = TIM2_CCR2L_RESET_VALUE;
  TIM2.CC3R.byteH = TIM2_CCR3H_RESET_VALUE;
  TIM2.CC3R.byteL = TIM2_CCR3L_RESET_VALUE;
  
  // force register update
  TIM2.EGR.reg.UG = 1;
  
} // tim2_init



/**
  \fn void tim2_set_pwm(uint32_t centHz, uint16_t deciPrc)
   
  \brief generate PWM signal on TIM2_CC2 (=PD3)
   
  \param centHz   frequency in 0.01Hz
  \param deciPrc  duty cycle in 0.1% (0..1000)
  
  generate a PWM signal of given frequency and duty cycle (with high polarity).
  A frequency of 0 deactivates the respective output.

*/
void tim2_set_pwm(uint32_t centHz, uint16_t deciPrc) {

  uint16_t    pre;    // 16b timer prescaler
  uint16_t    ARR;    // 16b reload value
  uint16_t    CCR;    // 16b compare value -> duty cycle
  
  
  //////////////
  // calculate timer parameter
  //////////////

  // calculate max. prescaler to maximize ARR resolution. Condition: centHz > 100*(16e6/(2^pre*(2^16-1)))
  pre = (uint8_t) log2(24414.4350347143/(float)centHz);         // checked via Excel
  pre = (uint16_t) MAX(pre, 0);
  pre = (uint16_t) MIN(pre, 15);
  
  // set freq to spec. value (centHz = 100*fCPU/((pre+1)*(ARR+1))
  ARR = (uint16_t) ROUND(16e8/((float)(1L<<pre)*(float)(centHz)) - 1.0);
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
    
  // reset timer registers
  tim2_init();
  
  // config TIM2 pin (=TIM2CC2 =PD3) as output
  PORT_D.CR1.bit.b3 = 1;       // input: 0=no exint, 1=exint; output: 0=2MHz slope, 1=10MHz slope
  PORT_D.CR2.bit.b3 = 1;       // input: 0=float, 1=pull-up; output: 0=open-drain, 1=push-pull
  PORT_D.DDR.bit.b3 = 1;       // input(=0) or output(=1)

  // set TIM2 prescaler f=fCPU/2^pre with pre in [0..15]
  TIM2.PSCR.reg.PSC = (uint8_t) pre;
  
  // set reload value (fPWM = fCPU/(2^pre*(ARR+1)))
  TIM2.ARR.byteH = (uint8_t) (ARR >> 8);
  TIM2.ARR.byteL = (uint8_t) ARR;
  
  // set capture/compare value for duty cycle (DC=CCR/ARR)
  TIM2.CC2R.byteH = (uint8_t) (CCR >> 8);
  TIM2.CC2R.byteL = (uint8_t) CCR;
  
  // set active polarity to high (=0) (1=low polarity)
  TIM2.CCER1.reg.CC2P = 0;

  // enable output
  TIM2.CCER1.reg.CC2E = 1;

  // set PWM mode 1
  TIM2.CCMR2.byte = 0b01101000;

  // request register update
  TIM2.EGR.reg.UG = 1;
  
  // start the timer
  TIM2.CR1.reg.CEN = 1;

} // tim2_set_pwm




/**
  \fn void tim2_get_pwm(uint32_t *centHz, uint16_t *deciPrc)
   
  \brief measure PWM signal on TIM2_CC2 (=PD3)
   
  \param[out] centHz   frequency in 0.01Hz (100..tbd)
  \param[out] decPrc   duty cycle in 0.1% (0..1000)
  
  measure a PWM input signal on TIM2_CC2 (=PD3) (frequency & duty cycle). 
  Perform automatic range adaptation (=prescaler).
*/
void tim2_get_pwm(uint32_t *centHz, uint16_t *deciPrc) {

  uint16_t    pre, CC1, CC2;
  uint8_t     flag, h=0, l=0;
  
  // reset timer registers and reset GPIO to input
  tim2_init();
    
  // configure timer behaviour
  TIM2.CR1.reg.UDIS = 0;
  TIM2.CR1.reg.URS  = 1;  // set SR1_UIF bit only on overflow

  // select input and set filter=0
  TIM2.CCMR1.regIn.CC1S = 2;      // TIM2.CC1 -> PD3
  TIM2.CCMR2.regIn.CC2S = 1;      // TIM2.CC2 -> PD3

  // capture CC1 on rising (=0),and CC2 on falling (=1) edge
  TIM2.CCER1.reg.CC1P = 0;
  TIM2.CCER1.reg.CC2P = 1;

  // enable capcom for CC1 & CC2
  TIM2.CCER1.reg.CC1E = 1;
  TIM2.CCER1.reg.CC2E = 1;

  // adapt clock prescaler
  flag  = 0;
  pre   = 0;
  do {

    // set prescaler f = fcpu/2^pre with pre in [0..15]
    TIM2.PSCR.reg.PSC = (uint8_t) pre;

    // wait for 2nd rising edge or timer overflow. TIM2 has no slave mode and needs some more effort
    TIM2.SR1.byte     = 0x00;                           // reset status registers
    TIM2.SR2.byte     = 0x00;
    l  = TIM2.CC1R.byteL;                               // clear CC1 capture flag
    TIM2.CNTR.byteH   = 0x00;
    TIM2.CNTR.byteL   = 0x00;
    TIM2.EGR.reg.UG   = 1;                              // request register update
    TIM2.CR1.byte     = 0x05;                           // start the timer (keep URS=1)
    while (!(TIM2.SR1.byte&0x03));                      // wait for 2nd rising edge or timeout
    h  = TIM2.CNTR.byteH;                               // store capture value
    l  = TIM2.CNTR.byteL;   
    while ((!(TIM2.SR2.byte&0x02)) && (!(TIM2.SR1.byte&0x01)));  // wait for 2nd rising edge or timeout
    TIM2.CR1.byte     = 0x04;                           // stop the timer (keep URS=1)
    
    // on timer overflow or if duty cycle>100% reduce clock or exit with static level
    if (TIM2.SR1.reg.UIF) {
        
      // max prescaler for 1Hz reached (need 2x period since counter is not reset) --> exit
      if (pre >= 9)
        flag = 1;
 
      // half timer frequency (f = fcpu/2^pre with pre in [0..15])
      else
        pre++;

    } // timer overflow

    // no overflow --> done
    else
      flag = 2;
      
  } while (!flag);
           

  // capture succeeded --> get final prescaler and both capture values
  if (flag == 2) {

    // get capture values
    CC1  = (uint16_t) TIM2.CC1R.byteH << 8;
    CC1 += (uint16_t) TIM2.CC1R.byteL;
    CC1 -= (uint16_t) l + ((uint16_t) h << 8);
    CC2  = (uint16_t) TIM2.CC2R.byteH << 8;
    CC2 += (uint16_t) TIM2.CC2R.byteL;
    CC2 -= (uint16_t) l + ((uint16_t) h << 8);
    
    // correct for SW latency (empirically)
    if (pre == 0){
      CC1 += 5;      
      CC2 += 5;
    }
    if (pre == 1){
      CC1 += 4;      
      CC2 += 4;
    }
          
    // convert to frequency [0.01Hz] and duty cycle [0.1%]
    *centHz  = (uint32_t) ROUND(16e8/(float)(1L<<pre) / (float)(CC1 + 1.0));
    *deciPrc = (uint16_t) ROUND((float) CC2 / (float) CC1 * 1000.0);
    
            
  } // if capture succeeded
  
  // timer overflow (flag==1) --> static signal
  else {
    *centHz = 100;              // set arbitrary frequency to 1Hz
    if (GPIO_READ(PORT_D, PIN_3))   // set duty cycle according to GPIO level
      *deciPrc = 1000;
    else
      *deciPrc = 0;
  }
  
  // just to be sure re-init timer & GPIO
  tim2_init();

} // tim2_get_pwm



/**
  \fn uint8_t tim2_receive_SENT(uint8_t *status, uint16_t *data_1, uint16_t *data_2, uint16_t timeout)
   
  \brief receive data via SENT protocol on TIM2_CC2 (=PD3)

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
uint8_t tim2_receive_SENT(uint8_t *status, uint16_t *data_1, uint16_t *data_2, uint16_t timeout) {

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
  
  // config TIM2 pin (TIM2_CC2 = PD3) as input pull-up
  PORT_D.DDR.bit.b3 = 0;       // input(=0) or output(=1)
  PORT_D.CR1.bit.b3 = 1;       // input: 0=float, 1=pull-up; output: 0=open-drain, 1=push-pull
  PORT_D.CR2.bit.b3 = 0;       // input: 0=no exint, 1=exint; output: 0=2MHz slope, 1=10MHz slope

  // stop timer
  TIM2.CR1.reg.CEN = 0;
  
  // set prescaler for 1us resolution
  TIM2.PSCR.reg.PSC = 4;        // fClk=16MHz/2^n -> 1us 

  // disable capcom for CC2 (reset CCE)
  TIM2.CCER1.reg.CC2E = 0;

  // select input and set filter=0
  TIM2.CCMR2.regIn.CC2S = 1;      // TIM2.CC2 -> PD3, no filter

  // capture CC2 on falling (=1) edge and enable
  TIM2.CCER1.reg.CC2P = 1;
  TIM2.CCER1.reg.CC2E = 1;

  // request register update
  TIM2.EGR.reg.UG = 1;

  // start the timer
  TIM2.CR1.reg.CEN = 1;


  ////////////
  // loop until frame received or timeout
  ////////////
  start_timeout_ms(timeout);                                        
  do {
      
    // wait for falling edge or timeout
    while ((!TIM2.SR1.reg.CC2IF) && (!check_timeout()));
    
    // get duration [us]
    duration = (uint8_t) (TIM2.CC2R.byteL - oldCapture);
 
    // get TIM2_CC2 low byte, clears interrupt flag 
    oldCapture = TIM2.CC2R.byteL; 
    
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
