/**
  \file stm8as.h
   
  \author G. Icking-Konert
  \date 2015-09-08
  \version 0.2
   
  \brief definition of STM8 peripherals etc.
   
  definition of STM8 peripheral registers, interrupt
  vector table, and some useful macros
*/

/*-----------------------------------------------------------------------------
    MODULE DEFINITION FOR MULTIPLE INCLUSION
-----------------------------------------------------------------------------*/
#ifndef _STM8AS_H
#define _STM8AS_H

#include <stdint.h>


/* Supported STM8 devices. Define respective device in Makefile
 STM8S103     standard line low density device
 STM8S903     standard line low density device
 STM8S105     standard line medium density device
 STM8S208     standard line high density device with CAN
 STM8S207     standard line high density device without CAN
 STM8AF622x   automotive low density devices without CAN
 STM8AF626x   automotive medium density devices without CAN
 STM8AF52ax   automotive high density devices with CAN
 STM8AF62ax   automotive high density devices without CAN
 STM8S003     value line low density device
 STM8S005     value line medium density device
 STM8S007     value line high density device
*/


/*-----------------------------------------------------------------------------
  memory range <=64kB (->16b pointer) or >=64kB (->32b pointer)
-----------------------------------------------------------------------------*/
#if defined(STM8S208) || defined(STM8S207) || defined(STM8AF52Ax) || defined(STM8AF62Ax) || defined(STM8S007)
  #define ADDR_WIDTH      32
  #define MEM_POINTER_T   uint32_t
#elif defined(STM8S103) || defined(STM8S903) || defined(STM8S105) || defined(STM8AF622x) || defined(STM8AF626x) || defined(STM8S003) || defined(STM8S005)
  #define ADDR_WIDTH 16
  #define MEM_POINTER_T   uint16_t
#else 
 #error "unsupported STM8 device, check Makefile"
#endif


/*-----------------------------------------------------------------------------
    set peripherals base addresses. From datasheets it seems like they are
    identical across all devices.
-----------------------------------------------------------------------------*/
#define OPT_BaseAddress         0x4800
#define GPIOA_BaseAddress       0x5000
#define GPIOB_BaseAddress       0x5005
#define GPIOC_BaseAddress       0x500A
#define GPIOD_BaseAddress       0x500F
#define GPIOE_BaseAddress       0x5014
#define GPIOF_BaseAddress       0x5019
#define GPIOG_BaseAddress       0x501E
#define GPIOH_BaseAddress       0x5023
#define GPIOI_BaseAddress       0x5028
#define FLASH_BaseAddress       0x505A
#define EXTI_BaseAddress        0x50A0
#define RST_BaseAddress         0x50B3
#define CLK_BaseAddress         0x50C0
#define WWDG_BaseAddress        0x50D1
#define IWDG_BaseAddress        0x50E0
#define AWU_BaseAddress         0x50F0
#define BEEP_BaseAddress        0x50F3
#define SPI_BaseAddress         0x5200
#define I2C_BaseAddress         0x5210
#define UART1_BaseAddress       0x5230
#define UART2_BaseAddress       0x5240
#define UART3_BaseAddress       0x5240
#define UART4_BaseAddress       0x5230
#define TIM1_BaseAddress        0x5250
#define TIM2_BaseAddress        0x5300
#define TIM3_BaseAddress        0x5320
#define TIM4_BaseAddress        0x5340
#define TIM5_BaseAddress        0x5300
#define TIM6_BaseAddress        0x5340
#define ADC1_BaseAddress        0x53E0
#define ADC2_BaseAddress        0x5400
#define CAN_BaseAddress         0x5420
#define CFG_BaseAddress         0x7F60
#define ITC_BaseAddress         0x7F70
#define DM_BaseAddress          0x7F90


/*-----------------------------------------------------------------------------
    GLOBAL MACROS
-----------------------------------------------------------------------------*/

// declare or reference to global variables, depending on '_MAIN_'
#ifdef _MAIN_
  #define global volatile
  //#define global
#else // _MAIN_
  #define global extern volatile
  //#define global extern
#endif // _MAIN_


/*-----------------------------------------------------------------------------
    COMPILER SPECIFIC SETTINGS
-----------------------------------------------------------------------------*/

// Cosmic compiler
#if defined(__CSMC__)
  
  #define reg(addr,type,name)    extern volatile type name @addr    ///< syntax for variables at absolute addresses
  #define ASM(mnem)    _asm(mnem)                                   ///< single line inline assembler
  #define ASM_START    #asm                                         ///< start multi-line inline assembler
  #define ASM_END      #endasm                                      ///< end multi-line inline assembler

  #define FAR  @far
  #define NEAR @near
  #define TINY @tiny
  #define EEPROM @eeprom
  #define CONST  const


// SDCC compiler
#elif defined(__SDCC)

  #define reg(addr,type,name)    volatile __at(addr) type name      ///< syntax for variables at absolute addresses
  #define ASM(mnem)    __asm__(mnem)                                ///< single line inline assembler
  #define ASM_START    __asm                                        ///< start multi-line inline assembler
  #define ASM_END      __endasm;                                    ///< end multi-line inline assembler

  #define FAR
  #define NEAR
  #define TINY
  #define EEPROM
  #define CONST  const

// compiler unknown
#else
  #error in 'stm8as.h': compiler not supported
#endif


/*-----------------------------------------------------------------------------
    DEFINITION OF GLOBAL MACROS/#DEFINES
-----------------------------------------------------------------------------*/

/*---
 misc macros
---*/

// common assembler instructions
#define _NOP_                ASM("nop")           ///< perform a nop() operation (=minimum delay)
#define DISABLE_INTERRUPTS   ASM("sim")           ///< disable interrupt handling
#define ENABLE_INTERRUPTS    ASM("rim")           ///< enable interrupt handling
#define TRIGGER_TRAP         ASM("trap")          ///< trigger a trap (=soft interrupt) e.g. for EMC robustness (see AN1015)
#define WAIT_FOR_INTERRUPT   ASM("wfi")           ///< stop code execution and wait for interrupt
#define ENTER_HALT           ASM("halt")          ///< put controller to HALT mode

// generic
#define SW_RESET             WWDG.CR.byte=0xBF    ///< reset controller via WWGD module


/*---
 STM8 interrupt vector for SDCC (for Cosmic see 'stm8_interrupt_vector.c')
---*/
#if defined(__SDCC)
  
  #define __TLI_VECTOR__                0    ///< irq0 - External Top Level interrupt (TLI)
  #define __AWU_VECTOR__                1    ///< irq1 - Auto Wake Up from Halt interrupt
  #define __CLK_VECTOR__                2    ///< irq2 - Clock Controller interrupt
  #define __EXTI0_VECTOR__              3    ///< irq3 - External interrupt 0 (GPIOA)
  #define __EXTI1_VECTOR__              4    ///< irq4 - External interrupt 1 (GPIOB)
  #define __EXTI2_VECTOR__              5    ///< irq5 - External interrupt 2 (GPIOC)
  #define __EXTI3_VECTOR__              6    ///< irq6 - External interrupt 3 (GPIOD)
  #define __EXTI4_VECTOR__              7    ///< irq7 - External interrupt 4 (GPIOE)
  #define __CAN_RX_VECTOR__             8    ///< irq8 - CAN receive interrupt (device dependent)
  #define __CAN_TX_VECTOR__             9    ///< irq9 - CAN transmit interrupt (device dependent)
  #define __SPI_VECTOR__               10    ///< irq10 - SPI End of transfer interrupt
  #define __TIM1UPD_VECTOR__           11    ///< irq11 - TIM1 Update/Overflow/Trigger/Break interrupt
  #define __TIM1CAP_VECTOR__           12    ///< irq12 - TIM1 Capture/Compare interrupt
  #define __TIM2UPD_VECTOR__           13    ///< irq13 - TIM2 Update/Overflow/Break interrupt
  #define __TIM2CAP_VECTOR__           14    ///< irq14 - TIM2 Capture/Compare interrupt
  #define __TIM3UPD_VECTOR__           15    ///< irq15 - Reserved
  #define __TIM3CAP_VECTOR__           16    ///< irq16 - Reserved
  #define __UART1_TX_CMPL_VECTOR__     17    ///< irq17 - USART1, Tx complete interrupt
  #define __UART1_RX_FULL_VECTOR__     18    ///< irq18 - USART1, Rx interrupt
  #define __I2C_VECTOR__               19    ///< irq19 - I2C interrupt
  #define __UART2_3_4_TX_CMPL_VECTOR__ 20    ///< irq20 - UART2, 3, or 4 Tx interrupt
  #define __UART2_3_4_RX_FULL_VECTOR__ 21    ///< irq21 - UART2, 3, or 4 Rx interrupt
  #define __ADC_VECTOR__               22    ///< irq22 - ADC end of conversion/Analog watchdog interrupts
  #define __TIM4UPD_VECTOR__           23    ///< irq23 - Timer 4 interrupt
  #define __FLASH_VECTOR__             24    ///< irq24 - FLASH interrupt

#endif // __SDCC


/*-----------------------------------------------------------------------------
    DECLARATION OF GLOBAL TYPEDEFS
-----------------------------------------------------------------------------*/

/** union for bit- or bytewise r/w-access to 8-bit data (byte_t) */
typedef union {

  /// for byte access
  uint8_t  byte;
  
  /// for bitwise access
  struct {
    uint8_t b0 : 1;     ///< bit 0 in byte
    uint8_t b1 : 1;     ///< bit 1 in byte
    uint8_t b2 : 1;     ///< bit 2 in byte
    uint8_t b3 : 1;     ///< bit 3 in byte
    uint8_t b4 : 1;     ///< bit 4 in byte
    uint8_t b5 : 1;     ///< bit 5 in byte
    uint8_t b6 : 1;     ///< bit 6 in byte
    uint8_t b7 : 1;     ///< bit 7 in byte
  } bit;
  
} byte_t;


/** struct for bytewise r/w access to 16bit data (word_t)
 \note
   order of r/w access is important and cannot be guaranteed in C --> don't use union like for byte_t
     - write: HB+LB ok / LB+HB and word fails! \n
     - read: LB+HB and word ok / HB+LB fails! \n
*/
typedef struct {
  uint8_t byteH;        ///< high byte in 16b word
  uint8_t byteL;        ///< low byte in 16b word
} word_t;


/*-----------------------------------------------------------------------------
    DEFINITION OF STM8 PERIPHERAL REGISTERS
-----------------------------------------------------------------------------*/

//------------------------
// GPIO ports (implemented on all devices) --> done
//------------------------
#if (1)   // dummy for filtering out in editor

  /** structure for controlling/monitoring pins in GPIO mode (GPIO_TypeDef) */
  typedef struct {
    byte_t ODR;     ///< Port x output data register (Px_ODR)
    byte_t IDR;     ///< Port x pin input register (Px_IDR)
    byte_t DDR;     ///< Port x data direction register (Px_DDR)
    byte_t CR1;     ///< Port x control register 1 (Px_CR1)
    byte_t CR2;     ///< Port x control register 2 (Px_CR2)
  } GPIO_TypeDef;

  // GPIO port A..F implemented on all devices
  reg(GPIOA_BaseAddress, GPIO_TypeDef, PORT_A);   ///< registers for GPIO port A access
  reg(GPIOB_BaseAddress, GPIO_TypeDef, PORT_B);   ///< registers for GPIO port B access
  reg(GPIOC_BaseAddress, GPIO_TypeDef, PORT_C);   ///< registers for GPIO port C access
  reg(GPIOD_BaseAddress, GPIO_TypeDef, PORT_D);   ///< registers for GPIO port D access
  reg(GPIOE_BaseAddress, GPIO_TypeDef, PORT_E);   ///< registers for GPIO port E access
  reg(GPIOF_BaseAddress, GPIO_TypeDef, PORT_F);   ///< registers for GPIO port F access

  // GPIO port G implemented on selected devices
  #if defined(STM8S207) || defined (STM8S007) || defined(STM8S208) || defined(STM8S105) || \
      defined(STM8S005) || defined (STM8AF52Ax) || defined (STM8AF62Ax) || defined (STM8AF626x)
    reg(GPIOG_BaseAddress, GPIO_TypeDef, PORT_G);   ///< registers for GPIO port G access
  #endif /* (STM8S208) ||(STM8S207)  || (STM8S105) || (STM8AF52Ax) || (STM8AF62Ax) || (STM8AF626x) */

  // GPIO port H+I implemented on selected devices
  #if defined(STM8S207) || defined (STM8S007) || defined(STM8S208) || defined (STM8AF52Ax) || \
      defined (STM8AF62Ax)
    reg(GPIOH_BaseAddress, GPIO_TypeDef, PORT_H);   ///< registers for GPIO port H access
    reg(GPIOI_BaseAddress, GPIO_TypeDef, PORT_I);   ///< registers for GPIO port I access
  #endif /* (STM8S208) ||(STM8S207) || (STM8AF62Ax) || (STM8AF52Ax) */

  /* GPIO Module Reset Values (all ports) */
  #define GPIO_ODR_RESET_VALUE ((uint8_t)0x00)
  #define GPIO_DDR_RESET_VALUE ((uint8_t)0x00)
  #define GPIO_CR1_RESET_VALUE ((uint8_t)0x00)
  #define GPIO_CR2_RESET_VALUE ((uint8_t)0x00)

#endif // (1)



//------------------------
// Clock module CLK (implemented on all devices) --> done
//------------------------
#if (1)   // dummy for filtering out in editor

  /** struct containing clock module registers (CLK) */
  typedef struct {

    /** Internal clock register (CLK_ICKR) */
    union {
    
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t HSIEN   : 1;    ///< High speed internal RC oscillator enable
        uint8_t HSIRDY  : 1;    ///< High speed internal oscillator ready flag
        uint8_t FHW     : 1;    ///< Fast wakeup from Halt/Active-halt modes enable
        uint8_t LSIEN   : 1;    ///< Low speed internal RC oscillator enable
        uint8_t LSIRDY  : 1;    ///< Low speed internal oscillator ready flag
        uint8_t REGAH   : 1;    ///< Regulator power off in Active-halt mode enable
        uint8_t res     : 2;    ///< Reserved, must be kept cleared
      } reg;
      
    } ICKR;


    /** External clock register (CLK_ECKR) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t HSEEN   : 1;    ///< High speed external crystal oscillator enable
        uint8_t HSERDY  : 1;    ///< High speed external crystal oscillator ready
        uint8_t res     : 6;    ///< Reserved, must be kept cleared
      } reg;
      
    } ECKR;


    /** Reserved register (1B) */
    uint8_t         res[1]; 


    /** Clock master status register (CLK_CMSR) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t CKM     : 8;    ///< Clock master status bits
      } reg;
      
    } CMSR;


    /** Clock master switch register (CLK_SWR) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t SWI     : 8;    ///< Clock master selection bits
      } reg;
      
    } SWR;


    /** Switch control register (CLK_SWCR) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t SWBSY   : 1;    ///< Switch busy flag
        uint8_t SWEN    : 1;    ///< Switch start/stop enable
        uint8_t SWIEN   : 1;    ///< Clock switch interrupt enable
        uint8_t SWIF    : 1;    ///< Clock switch interrupt flag
        uint8_t res     : 4;    ///< Reserved
      } reg;

    } SWCR;


    /** Clock divider register (CLK_CKDIVR) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t CPUDIV  : 3;    ///< CPU clock prescaler
        uint8_t HSIDIV  : 2;    ///< High speed internal clock prescaler
        uint8_t res     : 3;    ///< Reserved, must be kept cleared.
      } reg;

    } CKDIVR;


    /** Peripheral clock gating register 1 (CLK_PCKENR1) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t PCKEN_I2C       : 1;    ///< clock enable I2C
        uint8_t PCKEN_SPI       : 1;    ///< clock enable SPI
        uint8_t PCKEN_UART1     : 1;    ///< clock enable UART1
        uint8_t PCKEN_UART2     : 1;    ///< clock enable UART2
        uint8_t PCKEN_TIM4_TIM6 : 1;    ///< clock enable TIM4/TIM6
        uint8_t PCKEN_TIM2_TIM5 : 1;    ///< clock enable TIM4/TIM6
        uint8_t PCKEN_TIM3      : 1;    ///< clock enable TIM3
        uint8_t PCKEN_TIM1      : 1;    ///< clock enable TIM1
      } reg;
      
    } PCKENR1;


    /** Clock security system register (CLK_CSSR) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t CSSEN   : 1;    ///< Clock security system enable
        uint8_t AUX     : 1;    ///< Auxiliary oscillator connected to master clock
        uint8_t CSSDIE  : 1;    ///< Clock security system detection interrupt enable
        uint8_t CSSD    : 1;    ///< Clock security system detection
        uint8_t res     : 4;    ///< Reserved, must be kept cleared.
      } reg;
      
    } CSSR;


    /** Configurable clock output register (CLK_CCOR) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t CCOEN   : 1;    ///< Configurable clock output enable
        uint8_t CCOSEL  : 4;    ///< Configurable clock output selection.
        uint8_t CCORDY  : 1;    ///< Configurable clock output ready
        uint8_t CCOBSY  : 1;    ///< Configurable clock output busy
        uint8_t res     : 1;    ///< Reserved, must be kept cleared.
      } reg;

    } CCOR;


    /** Peripheral clock gating register 2 (CLK_PCKENR2) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t res         : 2;    ///< Reserved
        uint8_t PCKEN_AWU   : 1;    ///< clock enable AWU
        uint8_t PCKEN_ADC   : 1;    ///< clock enable ADC
        uint8_t res2        : 3;    ///< Reserved
        uint8_t PCKEN_CAN   : 1;    ///< clock enable CAN
      } reg;

    } PCKENR2;


    /// Reserved register (1B). Was CAN clock control (obsolete as of STM8 UM rev 7)
    uint8_t         res2[1]; 
    /*
    union {
      uint8_t  byte;
      struct {
        uint8_t CANDIV  : 3;
        uint8_t res     : 5;
      } reg;
    } CANCCR;
    */
    

    /** HSI clock calibration trimming register (CLK_HSITRIMR) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t HSITRIM : 4;    ///< HSI trimming value (some devices only support 3 bits, see DS!)
        uint8_t res     : 4;    ///< Reserved, must be kept cleared.
      } reg;
      
    } HSITRIMR;


    /** SWIM clock control register (CLK_SWIMCCR) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t SWIMCLK : 1;    ///< SWIM clock divider
        uint8_t res     : 7;    ///< Reserved.
      } reg;
      
    } SWIMCCR;

  } CLK_TypeDef;

  /// pointer to all CLK registers (all devices)
  reg(CLK_BaseAddress, CLK_TypeDef, CLK);

  /* CLK Module Reset Values */
  #define CLK_ICKR_RESET_VALUE     ((uint8_t)0x01)
  #define CLK_ECKR_RESET_VALUE     ((uint8_t)0x00)
  #define CLK_CMSR_RESET_VALUE     ((uint8_t)0xE1)
  #define CLK_SWR_RESET_VALUE      ((uint8_t)0xE1)
  #define CLK_SWCR_RESET_VALUE     ((uint8_t)0x00)
  #define CLK_CKDIVR_RESET_VALUE   ((uint8_t)0x18)
  #define CLK_PCKENR1_RESET_VALUE  ((uint8_t)0xFF)
  #define CLK_PCKENR2_RESET_VALUE  ((uint8_t)0xFF)
  #define CLK_CSSR_RESET_VALUE     ((uint8_t)0x00)
  #define CLK_CCOR_RESET_VALUE     ((uint8_t)0x00)
  #define CLK_HSITRIMR_RESET_VALUE ((uint8_t)0x00)
  #define CLK_SWIMCCR_RESET_VALUE  ((uint8_t)0x00)

#endif // (1)



//------------------------
// Window Watchdog WWDG (implemented on all devices) --> done
//------------------------
#if (1)   // dummy for filtering out in editor

  /** struct containing Window Watchdog registers (WWDG) */
  typedef struct {

    /** WWDG Control register (WWDG_CR) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t T       : 7;    ///< 7-bit WWDG counter (MSB to LSB)
        uint8_t WDGA    : 1;    ///< WWDG activation bit (not used if WWDG is enabled by option byte)
      } reg;
      
    } CR;


    /** WWDR Window register (WWDG_WR) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t W       : 7;    ///< 7-bit window value
        uint8_t res     : 1;    ///< Reserved
      } reg;
      
    } WR;

  } WWDG_TypeDef;

  /// pointer to all WWDG Window Watchdog registers (all devices)
  reg(WWDG_BaseAddress, WWDG_TypeDef, WWDG);

  /* WWDG Module Reset Values */
  #define WWDG_CR_RESET_VALUE ((uint8_t)0x7F)
  #define WWDG_WR_RESET_VALUE ((uint8_t)0x7F)

#endif // (1)



//------------------------
// Independent Timeout Watchdog IWDG (implemented on all devices) --> done
//------------------------
#if (1)   // dummy for filtering out in editor

  /** struct containing IWDG independent watchdog registers */
  typedef struct {

    /** IWDG Key register (IWDG_KR) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t KEY     : 8;    ///< IWDG key value
      } reg;
      
    } KR;


    /** IWDG Prescaler register (IWDG_PR) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t PR      : 3;    ///< Prescaler divider
        uint8_t res     : 5;    ///< Reserved
      } reg;
      
    } PR;


    /** IWDG Reload register (IWDG_RLR) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t RL      : 8;    ///< Watchdog counter reload value
      } reg;
      
    } RLR;

  } IWDG_TypeDef;

  /// pointer to all IWDG independent timeout watchdog registers (all devices)
  reg(IWDG_BaseAddress, IWDG_TypeDef, IWDG);

  /* IWDG Module Reset Values */
  #define IWDG_PR_RESET_VALUE  ((uint8_t)0x00)
  #define IWDG_RLR_RESET_VALUE ((uint8_t)0xFF)

#endif // (1)



//------------------------
// NVM Module FLASH (all devices, but differet sizes) --> done
//------------------------
#if (1)   // dummy for filtering out in editor

  /** struct containing FLASH memory registers */
  typedef struct {

    /** Flash control register 1 (FLASH_CR1) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t FIX     : 1;    ///< Fixed Byte programming time
        uint8_t IE      : 1;    ///< Flash Interrupt enable
        uint8_t AHALT   : 1;    ///< Power-down in Active-halt mode
        uint8_t HALT    : 1;    ///< Power-down in Halt mode
        uint8_t res     : 4;    ///< Reserved
      } reg;
    } CR1;


    /** Flash control register 2 (FLASH_CR2) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t PRG     : 1;    ///< Standard block programming
        uint8_t res     : 3;    ///< Reserved
        uint8_t FPRG    : 1;    ///< Fast block programming
        uint8_t ERASE   : 1;    ///< Block erasing
        uint8_t WPRG    : 1;    ///< Word programming
        uint8_t OPT     : 1;    ///< Write option bytes
      } reg;

    } CR2;


    /** complementary Flash control register 2 (FLASH_CR2) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t PRG     : 1;    ///< Standard block programming
        uint8_t res     : 3;    ///< Reserved
        uint8_t FPRG    : 1;    ///< Fast block programming
        uint8_t ERASE   : 1;    ///< Block erasing
        uint8_t WPRG    : 1;    ///< Word programming
        uint8_t OPT     : 1;    ///< Write option bytes
      } reg;

    } NCR2;


    /** Flash protection register (FLASH_FPR) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t WPB     : 6;    ///< User boot code area protection bits
        uint8_t res     : 2;    ///< Reserved
      } reg;

    } FPR;


    /** complementary Flash protection register (FLASH_FPR) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t WPB     : 6;    ///< User boot code area protection bits
        uint8_t res     : 2;    ///< Reserved
      } reg;

    } NFPR;


    /** Flash status register (FLASH_IAPSR) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t WR_PG_DIS : 1;    ///< Write attempted to protected page flag
        uint8_t PUL       : 1;    ///< Flash Program memory unlocked flag
        uint8_t EOP       : 1;    ///< End of programming (write or erase operation) flag
        uint8_t DUL       : 1;    ///< Data EEPROM area unlocked flag
        uint8_t res       : 2;    ///< Reserved, forced by hardware to 0
        uint8_t HVOFF     : 1;    ///< End of high voltage flag
        uint8_t res2      : 1;    ///< Reserved
      } reg;

    } IAPSR;


    /** Reserved registers (2B) */
    uint8_t         res[2]; 


    /** Flash program memory unprotecting key register (FLASH_PUKR) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t PUK     : 8;    ///< Main program memory unlock keys
      } reg;
      
    } PUKR;


    /** Reserved register (1B) */
    uint8_t         res2[1]; 


    /** Data EEPROM unprotection key register (FLASH_DUKR) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t DUK     : 8;    ///< Data EEPROM write unlock keys
      } reg;
      
    } DUKR;

  } FLASH_TypeDef;

  /// pointer to all Flash registers (all devices, but differet sizes)
  reg(FLASH_BaseAddress, FLASH_TypeDef, FLASH);

  /* FLASH Module Reset Values */
  #define FLASH_CR1_RESET_VALUE   ((uint8_t)0x00)
  #define FLASH_CR2_RESET_VALUE   ((uint8_t)0x00)
  #define FLASH_NCR2_RESET_VALUE  ((uint8_t)0xFF)
  #define FLASH_IAPSR_RESET_VALUE ((uint8_t)0x40)
  #define FLASH_PUKR_RESET_VALUE  ((uint8_t)0x00)
  #define FLASH_DUKR_RESET_VALUE  ((uint8_t)0x00)

#endif // (1)



//------------------------
// Auto Wake-Up Module AWU (implemented on all devices) --> done
//------------------------
#if (1)   // dummy for filtering out in editor

  /** struct containing AWU module registers */
  typedef struct {

    /** AWU Control/status register (AWU_CSR) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t MSR     : 1;    ///< LSI measurement enable
        uint8_t res     : 3;    ///< Reserved
        uint8_t AWUEN   : 1;    ///< Auto-wakeup enable
        uint8_t AWUF    : 1;    ///< Auto-wakeup flag
        uint8_t res2    : 2;    ///< Reserved
      } reg;

    } CSR;


    /** AWU Asynchronous prescaler register (AWU_APR) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t APR     : 6;    ///< Asynchronous prescaler divider
        uint8_t res     : 2;    ///< Reserved
      } reg;
      
    } APR;


    /** AWU Timebase selection register (AWU_TBR) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t AWUTB   : 4;    ///< Auto-wakeup timebase selection
        uint8_t res     : 4;    ///< Reserved
      } reg;

    } TBR;

  } AWU_TypeDef;

  /// pointer to all AWU registers (all devices)
  reg(AWU_BaseAddress, AWU_TypeDef, AWU);

  /* AWU Module Reset Values */
  #define AWU_CSR_RESET_VALUE ((uint8_t)0x00)
  #define AWU_APR_RESET_VALUE ((uint8_t)0x3F)
  #define AWU_TBR_RESET_VALUE ((uint8_t)0x00)

#endif // (1)



//------------------------
// Beeper module BEEP (all devices, may require option byte change) --> done
//------------------------
#if (1)   // dummy for filtering out in editor

  /** struct containing BEEP registers */
  typedef struct {

    /** Beeper control/status register (BEEP_CSR) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t BEEPDIV : 5;    ///< Beep clock prescaler divider
        uint8_t BEEPEN  : 1;    ///< Beep enable
        uint8_t BEEPSEL : 2;    ///< Beeper frequency selection
      } reg;
      
    } CSR;

  } BEEP_TypeDef;

  /// register for beeper control (all devices)
  reg(BEEP_BaseAddress, BEEP_TypeDef, BEEP);

  /* BEEP Module Reset Values */
  #define BEEP_CSR_RESET_VALUE ((uint8_t)0x1F)

#endif // (1)



//------------------------
// Reset Module RST (all devices) --> done
//------------------------
#if (1)   // dummy for filtering out in editor

  /** struct containing RST registers */
  typedef struct {

    /** Reset status register (RST_SR) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t WWDGF   : 1;    ///< Window Watchdog reset flag
        uint8_t IWDGF   : 1;    ///< Independent Watchdog reset flag
        uint8_t ILLOPF  : 1;    ///< Illegal opcode reset flag
        uint8_t SWIMF   : 1;    ///< SWIM reset flag
        uint8_t EMCF    : 1;    ///< EMC reset flag
        uint8_t res     : 3;    ///< Reserved
      } reg;

    } SR;

  } RST_TypeDef;

  /// register for reset status module (all devices)
  reg(RST_BaseAddress, RST_TypeDef, RST);

#endif // (1)



//------------------------
// Global Configuration CFG (all devices) --> done
//------------------------
#if (1)   // dummy for filtering out in editor

  /** struct containing CFG registers */
  typedef struct {

    /** Global configuration register (CFG_GCR) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t SWD     : 1;    ///< SWIM disable
        uint8_t AL      : 1;    ///< Activation level
        uint8_t res     : 6;    ///< Reserved
      } reg;
      
    } GCR;

  } CFG_TypeDef;

  /// register for CFG module (all devices)
  reg(CFG_BaseAddress, CFG_TypeDef, CFG);

  /* CFG Module Reset Values */
  #define CFG_GCR_RESET_VALUE ((uint8_t)0x00)

#endif // (1)



//------------------------
// External Interrupt Control EXTI (all devices) --> done
//------------------------
#if (1)   // dummy for filtering out in editor

  /** struct containing EXTI registers */
  typedef struct {

    /** External interrupt control register 1 (EXTI_CR1) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t PAIS    : 2;    ///< Port A external interrupt sensitivity bits
        uint8_t PBIS    : 2;    ///< Port B external interrupt sensitivity bits
        uint8_t PCIS    : 2;    ///< Port C external interrupt sensitivity bits
        uint8_t PDIS    : 2;    ///< Port D external interrupt sensitivity bits
      } reg;

    } CR1;


    /** External interrupt control register 2 (EXTI_CR2) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t PEIS    : 2;    ///< Port E external interrupt sensitivity bits
        uint8_t TLIS    : 1;    ///< Top level interrupt sensitivity
        uint8_t res     : 5;    ///< Reserved
      } reg;

    } CR2;

  } EXTI_TypeDef;

  /// pointer to all EXTI registers (all devices)
  reg(EXTI_BaseAddress, EXTI_TypeDef, EXTI);

  /* EXTI Module Reset Values */
  #define EXTI_CR1_RESET_VALUE ((uint8_t)0x00)
  #define EXTI_CR2_RESET_VALUE ((uint8_t)0x00)

#endif // (1)



//------------------------
// Interrupt Priority Module ITC (all devices) --> done
//------------------------
#if (1)   // dummy for filtering out in editor

  /** struct containing ITC registers */
  typedef struct {

    /** Software priority register 1/8 (ITC_SPR1) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t   res       : 2;    ///< Reserved
        uint8_t   AWU       : 2;    ///< AWU (=irq1) interrupt priority 
        uint8_t   CLK       : 2;    ///< CLK (=irq2) interrupt priority 
        uint8_t   EXTI_A    : 2;    ///< EXINT on port A (=irq3) interrupt priority
      } reg;

    } SPR1;


    /** Software priority register 2/8 (ITC_SPR2) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t   EXTI_B    : 2;    ///< EXINT on port B (=irq4) interrupt priority
        uint8_t   EXTI_C    : 2;    ///< EXINT on port C (=irq5) interrupt priority
        uint8_t   EXTI_D    : 2;    ///< EXINT on port D (=irq6) interrupt priority
        uint8_t   EXTI_E    : 2;    ///< EXINT on port E (=irq7) interrupt priority
      } reg;

    } SPR2;


    /** Software priority register 3/8 (ITC_SPR3) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t   CAN_RX    : 2;    ///< CAN Rx (=irq8) interrupt priority 
        uint8_t   CAN_TX    : 2;    ///< CAN Tx (=irq9) interrupt priority 
        uint8_t   SPI       : 2;    ///< SPI (=irq10) interrupt priority 
        uint8_t   TIM1_UPD  : 2;    ///< TIM1 Update (=irq11) interrupt priority 
      } reg;

    } SPR3;


    /** Software priority register 4/8 (ITC_SPR4) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t   TIM1_CAP  : 2;    ///< TIM1 Capture/Compare (=irq12) interrupt priority 
        uint8_t   TIM2_UPD  : 2;    ///< TIM2 Update (=irq13) interrupt priority 
        uint8_t   TIM2_CAP  : 2;    ///< TIM2 Capture/Compare (=irq14) interrupt priority 
        uint8_t   TIM3_UPD  : 2;    ///< TIM3 Update (=irq15) interrupt priority (Reserved)
      } reg;

    } SPR4;


    /** Software priority register 5/8 (ITC_SPR5) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t   TIM3_CAP  : 2;    ///< TIM3 Capture/Compare (=irq16) interrupt priority (Reserved)
        uint8_t   USART1_TX : 2;    ///< UART1 Tx complete (=irq17) interrupt priority 
        uint8_t   USART1_RX : 2;    ///< UART1 Rx full (=irq18) interrupt priority 
        uint8_t   I2C       : 2;    ///< I2C (=irq19) interrupt priority 
      } reg;

    } SPR5;


    /** Software priority register 6/8 (ITC_SPR6) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t   UART3_TX  : 2;    ///< LINUART3 Tx complete (=irq20) interrupt priority 
        uint8_t   UART3_RX  : 2;    ///< LINUART3 Rx full (=irq21) interrupt priority 
        uint8_t   ADC       : 2;    ///< ADC (=irq22) interrupt priority 
        uint8_t   TIM4_UPD  : 2;    ///< TIM4 Update (=irq23) interrupt priority (Reserved)
      } reg;

    } SPR6;


    /** Software priority register 7/8 (ITC_SPR7) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t   FLASH     : 2;    ///< flash (=irq24) interrupt priority 
        uint8_t   VECT25    : 2;    ///< irq25 interrupt priority (not documented)
        uint8_t   VECT26    : 2;    ///< irq26 interrupt priority (not documented)
        uint8_t   VECT27    : 2;    ///< irq27 interrupt priority (not documented)
      } reg;

    } SPR7;


    /** Software priority register 8/8 (ITC_SPR8) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t   VECT28    : 2;    ///< irq28 interrupt priority (not documented)
        uint8_t   res       : 6;    ///< Reserved
      } reg;

    } SPR8;

  } ITC_TypeDef;

  /// register for ITC control (all devices)
  reg(ITC_BaseAddress, ITC_TypeDef, ITC);

  /* ITC Module Reset Values (all registers) */
  #define ITC_SPRX_RESET_VALUE ((uint8_t)0xFF)

#endif // (1)



//------------------------
// Serial Peripheral Interface SPI (all devices) --> done
//------------------------
#if (1)   // dummy for filtering out in editor

  /** struct containing SPI registers */
  typedef struct {

    /** SPI control register 1 (SPI_CR1) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t CPHA     : 1;    ///< Clock phase
        uint8_t CPOL     : 1;    ///< Clock polarity
        uint8_t MSTR     : 1;    ///< Master/slave selection
        uint8_t BR       : 3;    ///< Baudrate control
        uint8_t SPE      : 1;    ///< SPI enable
        uint8_t LSBFIRST : 1;    ///< Frame format
      } reg;

    } CR1;


    /** SPI control register 2 (SPI_CR2) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t SSI     : 1;    ///< Internal slave select
        uint8_t SSM     : 1;    ///< Software slave management
        uint8_t RXONLY  : 1;    ///< Receive only
        uint8_t res     : 1;    ///< Reserved
        uint8_t CRCNEXT : 1;    ///< Transmit CRC next
        uint8_t CRCEN   : 1;    ///< Hardware CRC calculation enable
        uint8_t BDOE    : 1;    ///< Input/Output enable in bidirectional mode
        uint8_t BDM     : 1;    ///< Bidirectional data mode enable
      } reg;

    } CR2;


    /** SPI interrupt control register (SPI_ICR) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t res     : 4;    ///< Reserved
        uint8_t WKIE    : 1;    ///< Wakeup interrupt enable
        uint8_t ERRIE   : 1;    ///< Error interrupt enable
        uint8_t RXIE    : 1;    ///< Rx buffer not empty interrupt enable
        uint8_t TXIE    : 1;    ///< Tx buffer empty interrupt enable
      } reg;

    } ICR;


    /** SPI status register (SPI_SR) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t RXNE    : 1;    ///< Receive buffer not empty
        uint8_t TXE     : 1;    ///< Transmit buffer empty
        uint8_t res     : 1;    ///< Reserved
        uint8_t WKUP    : 1;    ///< Wakeup flag
        uint8_t CRCERR  : 1;    ///< CRC error flag
        uint8_t MODF    : 1;    ///< Mode fault
        uint8_t OVR     : 1;    ///< Overrun flag
        uint8_t BSY     : 1;    ///< Busy flag
      } reg;

    } SR;


    /** SPI data register (SPI_DR) */
    byte_t      DR; 
    
    
    /** SPI CRC polynomial register (SPI_CRCPR) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t CRCPOLY : 8;    ///< CRC polynomial register
      } reg;
      
    } CRCPR;


    /** SPI Rx CRC register (SPI_RXCRCR) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t RxCRC   : 8;    ///< Rx CRC Register
      } reg;
      
    } RXCRCR;


    /** SPI Tx CRC register (SPI_TXCRCR) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t TxCRC   : 8;    ///< Tx CRC register
      } reg;
      
    } TXCRCR;

  } SPI_TypeDef;

  /// register for SPI control
  reg(SPI_BaseAddress, SPI_TypeDef, SPI);

  /* SPI Module Reset Values */
  #define SPI_CR1_RESET_VALUE    ((uint8_t)0x00) /*!< Control Register 1 reset value */
  #define SPI_CR2_RESET_VALUE    ((uint8_t)0x00) /*!< Control Register 2 reset value */
  #define SPI_ICR_RESET_VALUE    ((uint8_t)0x00) /*!< Interrupt Control Register reset value */
  #define SPI_SR_RESET_VALUE     ((uint8_t)0x02) /*!< Status Register reset value */
  #define SPI_DR_RESET_VALUE     ((uint8_t)0x00) /*!< Data Register reset value */
  #define SPI_CRCPR_RESET_VALUE  ((uint8_t)0x07) /*!< Polynomial Register reset value */
  #define SPI_RXCRCR_RESET_VALUE ((uint8_t)0x00) /*!< RX CRC Register reset value */
  #define SPI_TXCRCR_RESET_VALUE ((uint8_t)0x00) /*!< TX CRC Register reset value */

#endif // (1)



//------------------------
// I2C Bus Interface (all devices) --> done
//------------------------
#if (1)   // dummy for filtering out in editor

  /** struct containing I2C registers */
  typedef struct {

    /** I2C Control register 1 (I2C_CR1) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t PE        : 1;    ///< Peripheral enable
        uint8_t res       : 5;    ///< Reserved
        uint8_t ENGC      : 1;    ///< General call enable
        uint8_t NOSTRETCH : 1;    ///< Clock stretching disable (Slave mode)
      } reg;

    } CR1;


    /** I2C Control register 2 (I2C_CR2) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t START     : 1;    ///< Start generation
        uint8_t STOP      : 1;    ///< Stop generation
        uint8_t ACK       : 1;    ///< Acknowledge enable
        uint8_t POS       : 1;    ///< Acknowledge position (for data reception)
        uint8_t res       : 3;    ///< Reserved
        uint8_t SWRST     : 1;    ///< Software reset
      } reg;

    } CR2;


    /** I2C Frequency register (I2C_FREQR) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t FREQ      : 6;    ///< Peripheral clock frequency
        uint8_t res       : 2;    ///< Reserved
      } reg;

    } FREQR;


    /** I2C Own address register LSB (I2C_OARL) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t ADD0      : 1;    ///< Interface address [10] (in 10-bit address mode)
        uint8_t ADD       : 7;    ///< Interface address [7:1]
      } reg;

    } OARL;


    /** I2C own address high (I2C_OARH) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t res       : 1;    ///< Reserved
        uint8_t ADD       : 2;    ///< Interface address [9:8] (in 10-bit address mode)
        uint8_t res2      : 3;    ///< Reserved
        uint8_t ADDCONF   : 1;    ///< Address mode configuration (must always be written as ‘1’)
        uint8_t ADDMODE   : 1;    ///< 7-/10-bit addressing mode (Slave mode)
      } reg;
      
    } OARH;


    /** Reserved register (1B) */
    uint8_t         res[1]; 


    /** I2C data register (I2C_DR) */
    byte_t          DR; 


    /** I2C Status register 1 (I2C_SR1) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t SB        : 1;    ///< Start bit (Mastermode)
        uint8_t ADDR      : 1;    ///< Address sent (master mode) / matched (slave mode)
        uint8_t BTF       : 1;    ///< Byte transfer finished
        uint8_t ADD10     : 1;    ///< 10-bit header sent (Master mode)
        uint8_t STOPF     : 1;    ///< Stop detection (Slave mode)
        uint8_t res       : 1;    ///< Reserved
        uint8_t RXNE      : 1;    ///< Data register not empty (receivers)
        uint8_t TXE       : 1;    ///< Data register empty (transmitters)
      } reg;
      
    } SR1;


    /** I2C Status register 2 (I2C_SR2) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t BERR      : 1;    ///< Bus error
        uint8_t ARLO      : 1;    ///< Arbitration lost (master mode)
        uint8_t AF        : 1;    ///< Acknowledge failure
        uint8_t OVR       : 1;    ///< Overrun/underrun
        uint8_t res       : 1;    ///< Reserved
        uint8_t WUFH      : 1;    ///< Wakeup from Halt
        uint8_t res2      : 2;    ///< Reserved
      } reg;
      
    } SR2;


    /** I2C Status register 3 (I2C_SR3) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t MSL       : 1;    ///< Master/Slave
        uint8_t BUSY      : 1;    ///< Bus busy
        uint8_t TRA       : 1;    ///< Transmitter/Receiver
        uint8_t res       : 1;    ///< Reserved
        uint8_t GENCALL   : 1;    ///< General call header (Slavemode)
        uint8_t res2      : 3;    ///< Reserved
      } reg;
      
    } SR3;


    /** I2C Interrupt register (I2C_ITR) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t ITERREN   : 1;    ///< Error interrupt enable
        uint8_t ITEVTEN   : 1;    ///< Event interrupt enable
        uint8_t ITBUFEN   : 1;    ///< Buffer interrupt enable
        uint8_t res       : 5;    ///< Reserved
      } reg;
      
    } ITR;


    /** I2C Clock control register low (I2C_CCRL) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t CCR       : 8;    ///< Clock control register (Master mode)
      } reg;
      
    } CCRL;


    /** I2C Clock control register high (I2C_CCRH) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t CCR       : 4;    ///< Clock control register in Fast/Standard mode (Master mode)
        uint8_t res       : 2;    ///< Reserved
        uint8_t DUTY      : 1;    ///< Fast mode duty cycle
        uint8_t FS        : 1;    ///< I2C master mode selection
      } reg;
      
    } CCRH;


    /** I2C TRISE register (I2C_TRISER) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t TRISE     : 6;    ///< Maximum rise time in Fast/Standard mode (Master mode)
        uint8_t res       : 2;    ///< Reserved
      } reg;
      
    } TRISER;


    /** Reserved register (1B). Was I2C packet error checking (undocumented in STM8 UM rev 9) */
    uint8_t         res2[1]; 
    /*
    union {
      uint8_t  byte;
      struct {
        uint8_t res       : 8;
      } reg;
    } PECR;
    */

  } I2C_TypeDef;

  /// pointer to all I2C registers (all devices)
  reg(I2C_BaseAddress, I2C_TypeDef, I2C);

  /* I2C Module Reset Values */
  #define I2C_CR1_RESET_VALUE    ((uint8_t)0x00)
  #define I2C_CR2_RESET_VALUE    ((uint8_t)0x00)
  #define I2C_FREQR_RESET_VALUE  ((uint8_t)0x00)
  #define I2C_OARL_RESET_VALUE   ((uint8_t)0x00)
  #define I2C_OARH_RESET_VALUE   ((uint8_t)0x00)
  #define I2C_DR_RESET_VALUE     ((uint8_t)0x00)
  #define I2C_SR1_RESET_VALUE    ((uint8_t)0x00)
  #define I2C_SR2_RESET_VALUE    ((uint8_t)0x00)
  #define I2C_SR3_RESET_VALUE    ((uint8_t)0x00)
  #define I2C_ITR_RESET_VALUE    ((uint8_t)0x00)
  #define I2C_CCRL_RESET_VALUE   ((uint8_t)0x00)
  #define I2C_CCRH_RESET_VALUE   ((uint8_t)0x00)
  #define I2C_TRISER_RESET_VALUE ((uint8_t)0x02)

#endif // (1)


//------------------------
// Controller Area Network CAN Module (selected devices)
//------------------------
#if defined (STM8S208) || defined (STM8AF52Ax)

  /** struct containing CAN registers (selected devices) */
  typedef struct {

    /** CAN master control register (CAN.MCR) */
    union {
  
      /// bytewise access to register
      uint8_t  byte;
  
      /// bitwise access to register
      struct {
        uint8_t INRQ      : 1;    ///< Initialization Request
        uint8_t SLEEP     : 1;    ///< Sleep Mode Request
        uint8_t TXFP      : 1;    ///< Transmit FIFO Priority
        uint8_t RFLM      : 1;    ///< Receive FIFO Locked Mode
        uint8_t NART      : 1;    ///< No Automatic Retransmission
        uint8_t AWUM      : 1;    ///< Automatic Wakeup Mode
        uint8_t ABOM      : 1;    ///< Automatic Bus-Off Management
        uint8_t TTCM      : 1;    ///< Time Triggered Communication Mode
      } reg;
  
    } MCR;


    /** CAN master status register (CAN.MSR) */
    union {
  
      /// bytewise access to register
      uint8_t  byte;
  
      /// bitwise access to register
      struct {
        uint8_t INAK      : 1;    ///< Initialization Acknowledge
        uint8_t SLAK      : 1;    ///< Sleep Acknowledge
        uint8_t ERRI      : 1;    ///< Error Interrupt
        uint8_t WKUI      : 1;    ///< Wakeup Interrupt
        uint8_t TX        : 1;    ///< Transmit
        uint8_t RX        : 1;    ///< Receive
        uint8_t res       : 2;    ///< Reserved
      } reg;
  
    } MSR;


    /** CAN transmit status register (CAN.TSR) */
    union {
  
      /// bytewise access to register
      uint8_t  byte;
  
      /// bitwise access to register
      struct {
        uint8_t RQCP0     : 1;    ///< Request Completed for Mailbox 0 
        uint8_t RQCP1     : 1;    ///< Request Completed for Mailbox 1
        uint8_t RQCP2     : 1;    ///< Request Completed for Mailbox 2
        uint8_t res       : 1;    ///< Reserved
        uint8_t TXOK0     : 1;    ///< Transmission ok for Mailbox 0
        uint8_t TXOK1     : 1;    ///< Transmission ok for Mailbox 1
        uint8_t TXOK2     : 1;    ///< Transmission ok for Mailbox 2
        uint8_t res2      : 1;    ///< Reserved
      } reg;
  
    } TSR;


    /** CAN transmit priority register (CAN.TPR) */
    union {
  
      /// bytewise access to register
      uint8_t  byte;
  
      /// bitwise access to register
      struct {
        uint8_t CODE      : 2;    ///< Mailbox Code
        uint8_t TME0      : 1;    ///< Transmit Mailbox 0 Empty
        uint8_t TME1      : 1;    ///< Transmit Mailbox 1 Empty
        uint8_t TME2      : 1;    ///< Transmit Mailbox 2 Empty
        uint8_t LOW0      : 1;    ///< Lowest Priority Flag for Mailbox 0
        uint8_t LOW1      : 1;    ///< Lowest Priority Flag for Mailbox 1
        uint8_t LOW2      : 1;    ///< Lowest Priority Flag for Mailbox 2
      } reg;
  
    } TPR;


    /** CAN receive FIFO register (CAN.RFR) */
    union {
  
      /// bytewise access to register
      uint8_t  byte;
  
      /// bitwise access to register
      struct {
        uint8_t FMP       : 2;    ///< FIFO Message Pending
        uint8_t res       : 1;    ///< Reserved
        uint8_t FULL      : 1;    ///< FIFO Full
        uint8_t FOVR      : 1;    ///< FIFO Overrun
        uint8_t RFOM      : 1;    ///< Release FIFO Output Mailbox
        uint8_t res2      : 2;    ///< Reserved    
      } reg;

    } RFR;


    /** CAN interrupt enable register (CAN.IER) */
    union {
  
      /// bytewise access to register
      uint8_t  byte;
  
      /// bitwise access to register
      struct {
        uint8_t TMEIE     : 1;    ///< Transmit Mailbox Empty Interrupt Enable
        uint8_t FMPIE     : 1;    ///< FIFO Message Pending Interrupt Enable
        uint8_t FFIE      : 1;    ///< FIFO Full Interrupt Enable
        uint8_t FOVIE     : 1;    ///< FIFO Overrun Interrupt Enable
        uint8_t res       : 3;    ///< Reserved
        uint8_t WKUIE     : 1;    ///< Wakeup Interrupt Enable
      } reg;

    } IER;


    /** CAN diagnosis register (CAN.DGR) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t LBKM      : 1;    ///< Loop back mode
        uint8_t SILM      : 1;    ///< Silent mode
        uint8_t SAMP      : 1;    ///< Last sample point
        uint8_t RX        : 1;    ///< CAN Rx Signal
        uint8_t TXM2E     : 1;    ///< TX Mailbox 2 enable
        uint8_t res       : 3;    ///< Reserved
      } reg;
      
    } DGR;


    /** CAN page selection register for below paged registers (CAN.PSR) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t PS        : 3;    ///< Page select
        uint8_t res       : 5;    ///< Reserved
      } reg;
      
    } PSR;



    /** paged CAN registers (selection via CAN.PSR) */
    union {
    
      /** CAN page 0,1,5: Tx Mailbox 0,1,2 (CAN.Page.TxMailbox) */
      struct {

        /** CAN message control/status register (CAN.Page.TxMailbox.MCSR) */
        union {
          
          /// bytewise access to register
          uint8_t  byte;
          
          /// bitwise access to register
          struct {
            uint8_t TXRQ      : 1;    ///< Transmission mailbox request
            uint8_t ABRQ      : 1;    ///< Abort request for mailbox
            uint8_t RQCP      : 1;    ///< Request completed
            uint8_t TXOK      : 1;    ///< Transmission OK
            uint8_t ALST      : 1;    ///< Arbitration lost
            uint8_t TERR      : 1;    ///< Transmission error
            uint8_t res       : 2;    ///< Reserved
          } reg;
          
        } MCSR;


        /** CAN mailbox data length control register (CAN.Page.TxMailbox.MDLCR) */
        union {
          
          /// bytewise access to register
          uint8_t  byte;
          
          /// bitwise access to register
          struct {
            uint8_t DLC       : 4;    ///< Data length code
            uint8_t res       : 3;    ///< Reserved
            uint8_t TGT       : 1;    ///< Transmit global time
          } reg;
          
        } MDLCR;


        /** CAN mailbox identifier register 1 (CAN.Page.TxMailbox.MIDR1)*/
        union {
          
          /// bytewise access to register
          uint8_t  byte;
          
          /// bitwise access to register
          struct {
            uint8_t ID        : 5;    ///< STID[10:6] or EXID[28:24]
            uint8_t RTR       : 1;    ///< Remote transmission request
            uint8_t IDE       : 1;    ///< Extended identifier
            uint8_t res       : 1;    ///< Reserved
          } reg;
          
        } MIDR1;


        /** CAN mailbox identifier register 2 (CAN.Page.TxMailbox.MIDR2) */
        union {
          
          /// bytewise access to register
          uint8_t  byte;
          
          /// bitwise access to register
          struct {
            uint8_t EXID      : 2;    ///< EXID[17:16]
            uint8_t ID        : 6;    ///< STID[5:0] or EXID[23:18]
          } reg;
        } MIDR2;


        /** CAN mailbox identifier register 3 (CAN.Page.TxMailbox.MIDR3) */
        union {
          
          /// bytewise access to register
          uint8_t  byte;
          
          /// bitwise access to register
          struct {
            uint8_t EXID      : 8;    ///< EXID[15:8]
          } reg;
          
        } MIDR3;


        /** CAN mailbox identifier register 4 (CAN.Page.TxMailbox.MIDR4) */
        union {
          
          /// bytewise access to register
          uint8_t  byte;
          
          /// bitwise access to register
          struct {
            uint8_t EXID      : 8;    ///< EXID[7:0]
          } reg;
          
        } MIDR4;


        /** CAN mailbox data register 1 (CAN.Page.TxMailbox.MDAR1) */
        byte_t    MDAR1;


        /** CAN mailbox data register 2 (CAN.Page.TxMailbox.MDAR1) */
        byte_t    MDAR2;


        /** CAN mailbox data register 3 (CAN.Page.TxMailbox.MDAR1) */
        byte_t    MDAR3;


        /** CAN mailbox data register 4 (CAN.Page.TxMailbox.MDAR1) */
        byte_t    MDAR4;


        /** CAN mailbox data register 5 (CAN.Page.TxMailbox.MDAR1) */
        byte_t    MDAR5;


        /** CAN mailbox data register 6 (CAN.Page.TxMailbox.MDAR1) */
        byte_t    MDAR6;


        /** CAN mailbox data register 7 (CAN.Page.TxMailbox.MDAR1) */
        byte_t    MDAR7;


        /** CAN mailbox data register 8 (CAN.Page.TxMailbox.MDAR1) */
        byte_t    MDAR8;
        
        
        /** CAN mailbox time stamp register (CAN.Page.TxMailbox.MTSR) */
        word_t    MTSR;

      } TxMailbox;



      /** CAN page 2: Acceptance Filter 0:1 (CAN.Page.Filter01) */
      struct {

        // filter 0
        byte_t    FR01;
        byte_t    FR02;
        byte_t    FR03;
        byte_t    FR04;
        byte_t    FR05;
        byte_t    FR06;
        byte_t    FR07;
        byte_t    FR08;
      
        // filter 1
        byte_t    FR11;
        byte_t    FR12;
        byte_t    FR13;
        byte_t    FR14;
        byte_t    FR15;
        byte_t    FR16;
        byte_t    FR17;
        byte_t    FR18;

      } Filter01;



      /** CAN page 3: Acceptance Filter 2:3 (CAN.Page.Filter23) */
      struct {

        // filter 2
        byte_t    FR21;
        byte_t    FR22;
        byte_t    FR23;
        byte_t    FR24;
        byte_t    FR25;
        byte_t    FR26;
        byte_t    FR27;
        byte_t    FR28;
      
        // filter 3
        byte_t    FR31;
        byte_t    FR32;
        byte_t    FR33;
        byte_t    FR34;
        byte_t    FR35;
        byte_t    FR36;
        byte_t    FR37;
        byte_t    FR38;

      } Filter23;



      /** CAN page 4: Acceptance Filter 4:5 (CAN.Page.Filter45) */
      struct {

        // filter 4
        byte_t    FR41;
        byte_t    FR42;
        byte_t    FR43;
        byte_t    FR44;
        byte_t    FR45;
        byte_t    FR46;
        byte_t    FR47;
        byte_t    FR48;
      
        // filter 5
        byte_t    FR51;
        byte_t    FR52;
        byte_t    FR53;
        byte_t    FR54;
        byte_t    FR55;
        byte_t    FR56;
        byte_t    FR57;
        byte_t    FR58;

      } Filter45;



      /** CAN page 6: Configuration/Diagnostics (CAN.Page.Config) */
      struct {

        /** CAN error status register (CAN.Page.Config.ESR) */
        union {
          
          /// bytewise access to register
          uint8_t  byte;
          
          /// bitwise access to register
          struct {
            uint8_t EWGF      : 1;    ///< Error warning flag
            uint8_t EPVF      : 1;    ///< Error passive flag
            uint8_t BOFF      : 1;    ///< Bus off flag
            uint8_t res       : 1;    ///< Reserved
            uint8_t LEC       : 3;    ///< Last error code
            uint8_t res2      : 1;    ///< Reserved
          } reg;
          
        } ESR;


        /** CAN error interrupt enable register (CAN.Page.Config.EIER) */
        union {
          
          /// bytewise access to register
          uint8_t  byte;
          
          /// bitwise access to register
          struct {
            uint8_t EWGIE     : 1;    ///< Error warning interrupt enable
            uint8_t EPVIE     : 1;    ///< Error passive  interrupt enable
            uint8_t BOFIE     : 1;    ///< Bus-Off  interrupt enable
            uint8_t res       : 1;    ///< Reserved
            uint8_t LECIE     : 1;    ///< Last error code interrupt enable
            uint8_t res2      : 2;    ///< Reserved
            uint8_t ERRIE     : 1;    ///< Error interrupt enable
          } reg;
          
        } EIER;


        /** CAN transmit error counter register (CAN.Page.Config.TECR) */
        union {
          
          /// bytewise access to register
          uint8_t  byte;
          
          /// bitwise access to register
          struct {
            uint8_t TEC       : 8;    ///< Transmit error counter
          } reg;
          
        } TECR;


        /** CAN receive error counter register (CAN.Page.Config.RECR) */
        union {
          
          /// bytewise access to register
          uint8_t  byte;
          
          /// bitwise access to register
          struct {
            uint8_t REC       : 8;    ///< Receive error counter
          } reg;
          
        } RECR;


        /** CAN bit timing register 1 (CAN.Page.Config.BTR1) */
        union {
          
          /// bytewise access to register
          uint8_t  byte;
          
          /// bitwise access to register
          struct {
            uint8_t BRP       : 6;    ///< Baud rate prescaler
            uint8_t SJW       : 2;    ///< Resynchronization jump width
          } reg;
          
        } BTR1;


        /** CAN bit timing register 2 (CAN.Page.Config.BTR2) */
        union {
          
          /// bytewise access to register
          uint8_t  byte;
          
          /// bitwise access to register
          struct {
            uint8_t BS1       : 4;    ///< Bit segment 1
            uint8_t BS2       : 3;    ///< Bit segment 2
            uint8_t res       : 1;    ///< Reserved, must be kept cleared
          } reg;
          
        } BTR2;


        /** Reserved registers (2B) */
        uint8_t       res[2];
        
        
        /** CAN filter mode register 1 (CAN.Page.Config.FMR1) */
        union {
          
          /// bytewise access to register
          uint8_t  byte;
          
          /// bitwise access to register
          struct {
            uint8_t FML0      : 1;    ///< Filter 0 mode low
            uint8_t FMH0      : 1;    ///< Filter 0 mode high
            uint8_t FML1      : 1;    ///< Filter 1 mode low
            uint8_t FMH1      : 1;    ///< Filter 1 mode high
            uint8_t FML2      : 1;    ///< Filter 2 mode low
            uint8_t FMH2      : 1;    ///< Filter 2 mode high
            uint8_t FML3      : 1;    ///< Filter 3 mode low
            uint8_t FMH3      : 1;    ///< Filter 3 mode high
          } reg;
          
        } FMR1;


        /** CAN filter mode register 2 (CAN.Page.Config.FMR2) */
        union {
          
          /// bytewise access to register
          uint8_t  byte;
          
          /// bitwise access to register
          struct {
            uint8_t FML4      : 1;    ///< Filter 4 mode low
            uint8_t FMH4      : 1;    ///< Filter 4 mode high
            uint8_t FML5      : 1;    ///< Filter 5 mode low
            uint8_t FMH5      : 1;    ///< Filter 5 mode high
            uint8_t res       : 4;    ///< Reserved
          } reg;
          
        } FMR2;


        /** CAN filter configuration register 1 (CAN.Page.Config.FCR1) */
        union {
          
          /// bytewise access to register
          uint8_t  byte;
          
          /// bitwise access to register
          struct {
            uint8_t FACT0     : 1;    ///< Filter 0 active
            uint8_t FSC0      : 2;    ///< Filter 0 scale configuration
            uint8_t res       : 1;    ///< Reserved
            uint8_t FACT1     : 1;    ///< Filter 1 active
            uint8_t FSC1      : 2;    ///< Filter 1 scale configuration
            uint8_t res2      : 1;    ///< Reserved
          } reg;
          
        } FCR1;


        /** CAN filter configuration register 2 (CAN.Page.Config.FCR2) */
        union {
          
          /// bytewise access to register
          uint8_t  byte;
          
          /// bitwise access to register
          struct {
            uint8_t FACT2     : 1;    ///< Filter 2 active
            uint8_t FSC2      : 2;    ///< Filter 2 scale configuration
            uint8_t res       : 1;    ///< Reserved
            uint8_t FACT3     : 1;    ///< Filter 3 active
            uint8_t FSC3      : 2;    ///< Filter 3 scale configuration
            uint8_t res2      : 1;    ///< Reserve
          } reg;
          
        } FCR2;




        /** CAN filter configuration register 3 (CAN.Page.Config.FCR3) */
        union {
          
          /// bytewise access to register
          uint8_t  byte;
          
          /// bitwise access to register
          struct {
            uint8_t FACT4     : 1;    ///< Filter 4 active
            uint8_t FSC4      : 2;    ///< Filter 4 scale configuration
            uint8_t res       : 1;    ///< Reserved
            uint8_t FACT5     : 1;    ///< Filter 5 active
            uint8_t FSC5      : 2;    ///< Filter 5 scale configuration
            uint8_t res2      : 1;    ///< Reserve
          } reg;
          
        } FCR3;


        /** Reserved registers (3B) */
        uint8_t       res2[3];

      } Config;



      /** CAN page 7: Receive FIFO (CAN.Page.RxFIFO) */
      struct {

        /** CAN mailbox filter match index register (CAN.Page.RxFIFO.MFMIR) */
        union {
          
          /// bytewise access to register
          uint8_t  byte;
          
          /// bitwise access to register
          struct {
            uint8_t FMI     : 8;    ///< Filter match index  
          } reg;
          
        } MFMIR;


        /** CAN mailbox data length control register (CAN.Page.RxFIFO.MDLCR) */
        union {
          
          /// bytewise access to register
          uint8_t  byte;
          
          /// bitwise access to register
          struct {
            uint8_t DLC       : 4;    ///< Data length code
            uint8_t res       : 3;    ///< Reserved
            uint8_t TGT       : 1;    ///< Transmit global time
          } reg;
          
        } MDLCR;


        /** CAN mailbox identifier register 1 (CAN.Page.RxFIFO.MIDR1)*/
        union {
          
          /// bytewise access to register
          uint8_t  byte;
          
          /// bitwise access to register
          struct {
            uint8_t ID        : 5;    ///< STID[10:6] or EXID[28:24]
            uint8_t RTR       : 1;    ///< Remote transmission request
            uint8_t IDE       : 1;    ///< Extended identifier
            uint8_t res       : 1;    ///< Reserved
          } reg;
          
        } MIDR1;


        /** CAN mailbox identifier register 2 (CAN.Page.RxFIFO.MIDR2) */
        union {
          
          /// bytewise access to register
          uint8_t  byte;
          
          /// bitwise access to register
          struct {
            uint8_t EXID      : 2;    ///< EXID[17:16]
            uint8_t ID        : 6;    ///< STID[5:0] or EXID[23:18]
          } reg;
        } MIDR2;


        /** CAN mailbox identifier register 3 (CAN.Page.RxFIFO.MIDR3) */
        union {
          
          /// bytewise access to register
          uint8_t  byte;
          
          /// bitwise access to register
          struct {
            uint8_t EXID      : 8;    ///< EXID[15:8]
          } reg;
          
        } MIDR3;


        /** CAN mailbox identifier register 4 (CAN.Page.RxFIFO.MIDR4) */
        union {
          
          /// bytewise access to register
          uint8_t  byte;
          
          /// bitwise access to register
          struct {
            uint8_t EXID      : 8;    ///< EXID[7:0]
          } reg;
          
        } MIDR4;


        /** CAN mailbox data register 1 (CAN.Page.RxFIFO.MDAR1) */
        byte_t    MDAR1;


        /** CAN mailbox data register 2 (CAN.Page.RxFIFO.MDAR1) */
        byte_t    MDAR2;


        /** CAN mailbox data register 3 (CAN.Page.RxFIFO.MDAR1) */
        byte_t    MDAR3;


        /** CAN mailbox data register 4 (CAN.Page.RxFIFO.MDAR1) */
        byte_t    MDAR4;


        /** CAN mailbox data register 5 (CAN.Page.RxFIFO.MDAR1) */
        byte_t    MDAR5;


        /** CAN mailbox data register 6 (CAN.Page.RxFIFO.MDAR1) */
        byte_t    MDAR6;


        /** CAN mailbox data register 7 (CAN.Page.RxFIFO.MDAR1) */
        byte_t    MDAR7;


        /** CAN mailbox data register 8 (CAN.Page.RxFIFO.MDAR1) */
        byte_t    MDAR8;
        
        
        /** CAN mailbox time stamp register (CAN.Page.RxFIFO.MTSR) */
        word_t    MTSR;

      } RxFIFO;

    } Page; 

  } CAN_TypeDef;

  /// pointer to all CAN registers (all devices)
  reg(CAN_BaseAddress, CAN_TypeDef, CAN);

  /* CAN Module Reset Values */
  #define  	CAN_MCR_RESET_VALUE			((uint8_t)0x02)
  #define  	CAN_MSR_RESET_VALUE			((uint8_t)0x02)
  #define  	CAN_TSR_RESET_VALUE			((uint8_t)0x00)
  #define  	CAN_TPR_RESET_VALUE			((uint8_t)0x0C)
  #define  	CAN_RFR_RESET_VALUE			((uint8_t)0x00)
  #define  	CAN_IER_RESET_VALUE			((uint8_t)0x00)
  #define  	CAN_DGR_RESET_VALUE			((uint8_t)0x0C)
  #define  	CAN_PSR_RESET_VALUE			((uint8_t)0x00)

  #define  	CAN_ESR_RESET_VALUE			((uint8_t)0x00)
  #define  	CAN_EIER_RESET_VALUE		((uint8_t)0x00)
  #define  	CAN_TECR_RESET_VALUE		((uint8_t)0x00)
  #define  	CAN_RECR_RESET_VALUE		((uint8_t)0x00)
  #define  	CAN_BTR1_RESET_VALUE		((uint8_t)0x40)
  #define  	CAN_BTR2_RESET_VALUE		((uint8_t)0x23)
  #define  	CAN_FMR1_RESET_VALUE		((uint8_t)0x00)
  #define  	CAN_FMR2_RESET_VALUE		((uint8_t)0x00)
  #define  	CAN_FCR_RESET_VALUE			((uint8_t)0x00)

  #define  	CAN_MFMI_RESET_VALUE		((uint8_t)0x00)
  #define  	CAN_MDLC_RESET_VALUE		((uint8_t)0x00)
  #define  	CAN_MCSR_RESET_VALUE		((uint8_t)0x00)

#endif /* (STM8S208) || (STM8AF52Ax) */



//------------------------
// UART1 Module (selected devices) with --> done
//    asyncronous & synchronous mode
//    multiprocessor communication
//    SmartCard & IrDA mode
//    1-wire, half-duplex
//    LIN master mode
//------------------------
#if defined(STM8S208) ||defined(STM8S207) || defined (STM8S007) || defined(STM8S103) || \
    defined(STM8S003) ||defined(STM8S903) || defined (STM8AF52Ax) || defined (STM8AF62Ax)

  /** struct containing UART1 registers (selected devices) */
  typedef struct {

    /** UART1 Status register (UART1_SR) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t PE      : 1;    ///< Parity error
        uint8_t FE      : 1;    ///< Framing error
        uint8_t NF      : 1;    ///< Noise flag
        uint8_t OR_LHE  : 1;    ///< LIN Header Error (LIN slave mode) / Overrun error
        uint8_t IDLE    : 1;    ///< IDLE line detected
        uint8_t RXNE    : 1;    ///< Read data register not empty
        uint8_t TC      : 1;    ///< Transmission complete
        uint8_t TXE     : 1;    ///< Transmit data register empty
      } reg;
      
    } SR;


    /** UART1 data register (UART1_DR) */
    byte_t          DR; 


    /** UART1 Baud rate register 1 (UART1_BRR1) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t UART_DIV_4_11 : 8;    ///< UART_DIV bits [11:4]
      } reg;
      
    } BRR1;


    /** UART1 Baud rate register 2 (UART1_BRR2) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t UART_DIV_0_3   : 4;    ///< UART_DIV bits [3:0]
        uint8_t UART_DIV_12_15 : 4;    ///< UART_DIV bits [15:12]
      } reg;
      
    } BRR2;


    /** UART1 Control register 1 (UART1_CR1) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t PIEN    : 1;    ///< Parity interrupt enable
        uint8_t PS      : 1;    ///< Parity selection
        uint8_t PCEN    : 1;    ///< Parity control enable
        uint8_t WAKE    : 1;    ///< Wakeup method
        uint8_t M       : 1;    ///< word length
        uint8_t UARTD   : 1;    ///< UART Disable (for low power consumption)
        uint8_t T8      : 1;    ///< Transmit Data bit 8 (in 9-bit mode)
        uint8_t R8      : 1;    ///< Receive Data bit 8 (in 9-bit mode)
      } reg;
      
    } CR1;


    /** UART1 Control register 2 (UART1_CR2) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t SBK     : 1;    ///< Send break
        uint8_t RWU     : 1;    ///< Receiver wakeup
        uint8_t REN     : 1;    ///< Receiver enable
        uint8_t TEN     : 1;    ///< Transmitter enable
        uint8_t ILIEN   : 1;    ///< IDLE Line interrupt enable
        uint8_t RIEN    : 1;    ///< Receiver interrupt enable
        uint8_t TCIEN   : 1;    ///< Transmission complete interrupt enable
        uint8_t TIEN    : 1;    ///< Transmitter interrupt enable
      } reg;
      
    } CR2;


    /** UART1 Control register 3 (UART1_CR3) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t LBCL    : 1;    ///< Last bit clock pulse
        uint8_t CPHA    : 1;    ///< Clock phase
        uint8_t CPOL    : 1;    ///< Clock polarity
        uint8_t CKEN    : 1;    ///< Clock enable
        uint8_t STOP    : 2;    ///< STOP bits
        uint8_t LINEN   : 1;    ///< LIN mode enable
        uint8_t res     : 1;    ///< Reserved, must be kept cleared
      } reg;
      
    } CR3;


    /** UART1 Control register 4 (UART1_CR4) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t ADD     : 4;    ///< Address of the UART node
        uint8_t LBDF    : 1;    ///< LIN Break Detection Flag
        uint8_t LBDL    : 1;    ///< LIN Break Detection Length
        uint8_t LBDIEN  : 1;    ///< LIN Break Detection Interrupt Enable
        uint8_t res     : 1;    ///< Reserved, must be kept cleared
      } reg;
      
    } CR4;


    /** UART1 Control register 5 (UART1_CR5) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t res     : 1;    ///< Reserved, must be kept cleared
        uint8_t IREN    : 1;    ///< IrDA mode Enable
        uint8_t IRLP    : 1;    ///< IrDA Low Power
        uint8_t HDSEL   : 1;    ///< Half-Duplex Selection
        uint8_t NACK    : 1;    ///< Smartcard NACK enable
        uint8_t SCEN    : 1;    ///< Smartcard mode enable
        uint8_t res2    : 2;    ///< Reserved, must be kept cleared
      } reg;
      
    } CR5;


    /** UART1 guard time register register (UART1_GTR) */
    byte_t          GTR;


    /** UART1 prescaler register register register (UART1_PSCR) */
    byte_t          PSCR;

  } UART1_TypeDef;
    
  /// pointer to UART1 registers
  reg(UART1_BaseAddress, UART1_TypeDef, UART1);

  /* UART1 Module Reset Values */
  #define UART1_SR_RESET_VALUE   ((uint8_t)0xC0)
  #define UART1_BRR1_RESET_VALUE ((uint8_t)0x00)
  #define UART1_BRR2_RESET_VALUE ((uint8_t)0x00)
  #define UART1_CR1_RESET_VALUE  ((uint8_t)0x00)
  #define UART1_CR2_RESET_VALUE  ((uint8_t)0x00)
  #define UART1_CR3_RESET_VALUE  ((uint8_t)0x00)
  #define UART1_CR4_RESET_VALUE  ((uint8_t)0x00)
  #define UART1_CR5_RESET_VALUE  ((uint8_t)0x00)
  #define UART1_GTR_RESET_VALUE  ((uint8_t)0x00)
  #define UART1_PSCR_RESET_VALUE ((uint8_t)0x00)

#endif /* (STM8S208) ||(STM8S207)  || (STM8S103) || (STM8S903) || (STM8AF52Ax) || (STM8AF62Ax) */



//------------------------
// UART2 Module (selected devices) with --> done
//    asyncronous & synchronous mode
//    multiprocessor communication
//    SmartCard & IrDA mode
//    LIN master & slave mode
//------------------------
#if defined (STM8S105) || defined (STM8S005) || defined (STM8AF626x)

  /** struct containing UART2 registers (selected devices) */
  typedef struct   {

    /** UART2 Status register (UART2_SR) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t PE      : 1;    ///< Parity error
        uint8_t FE      : 1;    ///< Framing error
        uint8_t NF      : 1;    ///< Noise flag
        uint8_t OR_LHE  : 1;    ///< LIN Header Error (LIN slave mode) / Overrun error
        uint8_t IDLE    : 1;    ///< IDLE line detected
        uint8_t RXNE    : 1;    ///< Read data register not empty
        uint8_t TC      : 1;    ///< Transmission complete
        uint8_t TXE     : 1;    ///< Transmit data register empty
      } reg;
      
    } SR;


    /** UART2 data register (UART2_DR) */
    byte_t          DR; 


    /** UART2 Baud rate register 1 (UART2_BRR1) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t UART_DIV_4_11 : 8;    ///< UART_DIV bits [11:4]
      } reg;
      
    } BRR1;


    /** UART2 Baud rate register 2 (UART2_BRR2) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t UART_DIV_0_3   : 4;    ///< UART_DIV bits [3:0]
        uint8_t UART_DIV_12_15 : 4;    ///< UART_DIV bits [15:12]
      } reg;
      
    } BRR2;


    /** UART2 Control register 1 (UART2_CR1) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t PIEN    : 1;    ///< Parity interrupt enable
        uint8_t PS      : 1;    ///< Parity selection
        uint8_t PCEN    : 1;    ///< Parity control enable
        uint8_t WAKE    : 1;    ///< Wakeup method
        uint8_t M       : 1;    ///< word length
        uint8_t UARTD   : 1;    ///< UART Disable (for low power consumption)
        uint8_t T8      : 1;    ///< Transmit Data bit 8 (in 9-bit mode)
        uint8_t R8      : 1;    ///< Receive Data bit 8 (in 9-bit mode)
      } reg;
      
    } CR1;


    /** UART2 Control register 2 (UART2_CR2) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t SBK     : 1;    ///< Send break
        uint8_t RWU     : 1;    ///< Receiver wakeup
        uint8_t REN     : 1;    ///< Receiver enable
        uint8_t TEN     : 1;    ///< Transmitter enable
        uint8_t ILIEN   : 1;    ///< IDLE Line interrupt enable
        uint8_t RIEN    : 1;    ///< Receiver interrupt enable
        uint8_t TCIEN   : 1;    ///< Transmission complete interrupt enable
        uint8_t TIEN    : 1;    ///< Transmitter interrupt enable
      } reg;
      
    } CR2;


    /** UART2 Control register 3 (UART2_CR3) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t LBCL    : 1;    ///< Last bit clock pulse
        uint8_t CPHA    : 1;    ///< Clock phase
        uint8_t CPOL    : 1;    ///< Clock polarity
        uint8_t CKEN    : 1;    ///< Clock enable
        uint8_t STOP    : 2;    ///< STOP bits
        uint8_t LINEN   : 1;    ///< LIN mode enable
        uint8_t res     : 1;    ///< Reserved, must be kept cleared
      } reg;
      
    } CR3;


    /** UART2 Control register 4 (UART2_CR4) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t ADD     : 4;    ///< Address of the UART node
        uint8_t LBDF    : 1;    ///< LIN Break Detection Flag
        uint8_t LBDL    : 1;    ///< LIN Break Detection Length
        uint8_t LBDIEN  : 1;    ///< LIN Break Detection Interrupt Enable
        uint8_t res     : 1;    ///< Reserved, must be kept cleared
      } reg;
      
    } CR4;


    /** UART2 Control register 5 (UART2_CR5) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t res     : 1;    ///< Reserved, must be kept cleared
        uint8_t IREN    : 1;    ///< IrDA mode Enable
        uint8_t IRLP    : 1;    ///< IrDA Low Power
        uint8_t res2    : 1;    ///< Reserved, must be kept cleared
        uint8_t NACK    : 1;    ///< Smartcard NACK enable
        uint8_t SCEN    : 1;    ///< Smartcard mode enable
        uint8_t res3    : 2;    ///< Reserved, must be kept cleared
      } reg;
      
    } CR5;


    /** UART2 Control register 6 (UART2_CR6) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t LSF     : 1;    ///< LIN Sync Field
        uint8_t LHDF    : 1;    ///< LIN Header Detection Flag
        uint8_t LHDIEN  : 1;    ///< LIN Header Detection Interrupt Enable
        uint8_t res     : 1;    ///< Reserved
        uint8_t LASE    : 1;    ///< LIN automatic resynchronisation enable
        uint8_t LSLV    : 1;    ///< LIN Slave Enable
        uint8_t res2    : 1;    ///< Reserved
        uint8_t LDUM    : 1;    ///< LIN Divider Update Method
      } reg;
      
    } CR6;


    /** UART2 guard time register register (UART2_GTR) */
    byte_t          GTR;


    /** UART2 prescaler register register register (UART2_PSCR) */
    byte_t          PSCR;

  } UART2_TypeDef;

  /// pointer to UART2 registers
  reg(UART2_BaseAddress, UART2_TypeDef, UART2);

  /* UART2 Module Reset Values */
  #define UART2_SR_RESET_VALUE   ((uint8_t)0xC0)
  #define UART2_BRR1_RESET_VALUE ((uint8_t)0x00)
  #define UART2_BRR2_RESET_VALUE ((uint8_t)0x00)
  #define UART2_CR1_RESET_VALUE  ((uint8_t)0x00)
  #define UART2_CR2_RESET_VALUE  ((uint8_t)0x00)
  #define UART2_CR3_RESET_VALUE  ((uint8_t)0x00)
  #define UART2_CR4_RESET_VALUE  ((uint8_t)0x00)
  #define UART2_CR5_RESET_VALUE  ((uint8_t)0x00)
  #define UART2_CR6_RESET_VALUE  ((uint8_t)0x00)
  #define UART2_GTR_RESET_VALUE  ((uint8_t)0x00)
  #define UART2_PSCR_RESET_VALUE ((uint8_t)0x00)

#endif /* STM8S105 || STM8S005 || STM8AF626x */



//------------------------
// UART3 Module (selected devices) with --> done
//    asyncronous mode
//    multiprocessor communication
//    LIN master & slave mode
//------------------------
#if defined(STM8S208) ||defined(STM8S207) || defined (STM8S007) || defined (STM8AF52Ax) || \
    defined (STM8AF62Ax)

  /** struct containing UART3 registers (selected devices) */
  typedef struct {

    /** UART3 Status register (UART3_SR) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t PE      : 1;    ///< Parity error
        uint8_t FE      : 1;    ///< Framing error
        uint8_t NF      : 1;    ///< Noise flag
        uint8_t OR_LHE  : 1;    ///< LIN Header Error (LIN slave mode) / Overrun error
        uint8_t IDLE    : 1;    ///< IDLE line detected
        uint8_t RXNE    : 1;    ///< Read data register not empty
        uint8_t TC      : 1;    ///< Transmission complete
        uint8_t TXE     : 1;    ///< Transmit data register empty
      } reg;
      
    } SR;


    /** UART3 data register (UART3_DR) */
    byte_t          DR; 


    /** UART3 Baud rate register 1 (UART3_BRR1) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t UART_DIV_4_11 : 8;    ///< UART_DIV bits [11:4]
      } reg;
      
    } BRR1;


    /** UART3 Baud rate register 2 (UART3_BRR2) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t UART_DIV_0_3   : 4;    ///< UART_DIV bits [3:0]
        uint8_t UART_DIV_12_15 : 4;    ///< UART_DIV bits [15:12]
      } reg;
      
    } BRR2;


    /** UART3 Control register 1 (UART3_CR1) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t PIEN    : 1;    ///< Parity interrupt enable
        uint8_t PS      : 1;    ///< Parity selection
        uint8_t PCEN    : 1;    ///< Parity control enable
        uint8_t WAKE    : 1;    ///< Wakeup method
        uint8_t M       : 1;    ///< word length
        uint8_t UARTD   : 1;    ///< UART Disable (for low power consumption)
        uint8_t T8      : 1;    ///< Transmit Data bit 8 (in 9-bit mode)
        uint8_t R8      : 1;    ///< Receive Data bit 8 (in 9-bit mode)
      } reg;
      
    } CR1;


    /** UART3 Control register 2 (UART3_CR2) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t SBK     : 1;    ///< Send break
        uint8_t RWU     : 1;    ///< Receiver wakeup
        uint8_t REN     : 1;    ///< Receiver enable
        uint8_t TEN     : 1;    ///< Transmitter enable
        uint8_t ILIEN   : 1;    ///< IDLE Line interrupt enable
        uint8_t RIEN    : 1;    ///< Receiver interrupt enable
        uint8_t TCIEN   : 1;    ///< Transmission complete interrupt enable
        uint8_t TIEN    : 1;    ///< Transmitter interrupt enable
      } reg;
      
    } CR2;


    /** UART3 Control register 3 (UART3_CR3) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t res     : 4;    ///< Reserved, must be kept cleared
        uint8_t STOP    : 2;    ///< STOP bits
        uint8_t LINEN   : 1;    ///< LIN mode enable
        uint8_t res2    : 1;    ///< Reserved, must be kept cleared
      } reg;
      
    } CR3;


    /** UART3 Control register 4 (UART3_CR4) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t ADD     : 4;    ///< Address of the UART node
        uint8_t LBDF    : 1;    ///< LIN Break Detection Flag
        uint8_t LBDL    : 1;    ///< LIN Break Detection Length
        uint8_t LBDIEN  : 1;    ///< LIN Break Detection Interrupt Enable
        uint8_t res     : 1;    ///< Reserved, must be kept cleared
      } reg;
      
    } CR4;


    /** Reserved register (1B) */
    uint8_t         res2[1]; 


    /** UART3 Control register 6 (UART3_CR6) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t LSF     : 1;    ///< LIN Sync Field
        uint8_t LHDF    : 1;    ///< LIN Header Detection Flag
        uint8_t LHDIEN  : 1;    ///< LIN Header Detection Interrupt Enable
        uint8_t res     : 1;    ///< Reserved
        uint8_t LASE    : 1;    ///< LIN automatic resynchronisation enable
        uint8_t LSLV    : 1;    ///< LIN Slave Enable
        uint8_t res2    : 1;    ///< Reserved
        uint8_t LDUM    : 1;    ///< LIN Divider Update Method
      } reg;
      
    } CR6;

  } UART3_TypeDef;

  /// pointer to UART3 registers
  reg(UART3_BaseAddress, UART3_TypeDef, UART3);

  /* UART3 Module Reset Values */
  #define UART3_SR_RESET_VALUE   ((uint8_t)0xC0)
  #define UART3_BRR1_RESET_VALUE ((uint8_t)0x00)
  #define UART3_BRR2_RESET_VALUE ((uint8_t)0x00)
  #define UART3_CR1_RESET_VALUE  ((uint8_t)0x00)
  #define UART3_CR2_RESET_VALUE  ((uint8_t)0x00)
  #define UART3_CR3_RESET_VALUE  ((uint8_t)0x00)
  #define UART3_CR4_RESET_VALUE  ((uint8_t)0x00)
  #define UART3_CR6_RESET_VALUE  ((uint8_t)0x00)

#endif /* (STM8S208) ||(STM8S207) || (STM8AF62Ax) || (STM8AF52Ax) */



//------------------------
// UART4 Module (selected devices) with --> done
//    asyncronous & synchronous mode
//    multiprocessor communication
//    SmartCard & IrDA mode
//    1-wire, half-duplex
//    LIN master & slave mode
//------------------------
#if defined(STM8AF622x)

  /** struct containing UART4 registers (selected devices) */
  typedef struct {

    /** UART4 Status register (UART4_SR) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t PE      : 1;    ///< Parity error
        uint8_t FE      : 1;    ///< Framing error
        uint8_t NF      : 1;    ///< Noise flag
        uint8_t OR_LHE  : 1;    ///< LIN Header Error (LIN slave mode) / Overrun error
        uint8_t IDLE    : 1;    ///< IDLE line detected
        uint8_t RXNE    : 1;    ///< Read data register not empty
        uint8_t TC      : 1;    ///< Transmission complete
        uint8_t TXE     : 1;    ///< Transmit data register empty
      } reg;
      
    } SR;


    /** UART4 data register (UART4_DR) */
    byte_t          DR; 


    /** UART4 Baud rate register 1 (UART4_BRR1) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t UART_DIV_4_11 : 8;    ///< UART_DIV bits [11:4]
      } reg;
      
    } BRR1;


    /** UART4 Baud rate register 2 (UART4_BRR2) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t UART_DIV_0_3   : 4;    ///< UART_DIV bits [3:0]
        uint8_t UART_DIV_12_15 : 4;    ///< UART_DIV bits [15:12]
      } reg;
      
    } BRR2;


    /** UART4 Control register 1 (UART4_CR1) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t PIEN    : 1;    ///< Parity interrupt enable
        uint8_t PS      : 1;    ///< Parity selection
        uint8_t PCEN    : 1;    ///< Parity control enable
        uint8_t WAKE    : 1;    ///< Wakeup method
        uint8_t M       : 1;    ///< word length
        uint8_t UARTD   : 1;    ///< UART Disable (for low power consumption)
        uint8_t T8      : 1;    ///< Transmit Data bit 8 (in 9-bit mode)
        uint8_t R8      : 1;    ///< Receive Data bit 8 (in 9-bit mode)
      } reg;
      
    } CR1;


    /** UART4 Control register 2 (UART4_CR2) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t SBK     : 1;    ///< Send break
        uint8_t RWU     : 1;    ///< Receiver wakeup
        uint8_t REN     : 1;    ///< Receiver enable
        uint8_t TEN     : 1;    ///< Transmitter enable
        uint8_t ILIEN   : 1;    ///< IDLE Line interrupt enable
        uint8_t RIEN    : 1;    ///< Receiver interrupt enable
        uint8_t TCIEN   : 1;    ///< Transmission complete interrupt enable
        uint8_t TIEN    : 1;    ///< Transmitter interrupt enable
      } reg;
      
    } CR2;


    /** UART4 Control register 3 (UART4_CR3) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t LBCL    : 1;    ///< Last bit clock pulse
        uint8_t CPHA    : 1;    ///< Clock phase
        uint8_t CPOL    : 1;    ///< Clock polarity
        uint8_t CKEN    : 1;    ///< Clock enable
        uint8_t STOP    : 2;    ///< STOP bits
        uint8_t LINEN   : 1;    ///< LIN mode enable
        uint8_t res     : 1;    ///< Reserved, must be kept cleared
      } reg;
      
    } CR3;


    /** UART4 Control register 4 (UART4_CR4) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t ADD     : 4;    ///< Address of the UART node
        uint8_t LBDF    : 1;    ///< LIN Break Detection Flag
        uint8_t LBDL    : 1;    ///< LIN Break Detection Length
        uint8_t LBDIEN  : 1;    ///< LIN Break Detection Interrupt Enable
        uint8_t res     : 1;    ///< Reserved, must be kept cleared
      } reg;
      
    } CR4;


    /** UART4 Control register 5 (UART4_CR5) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t res     : 1;    ///< Reserved, must be kept cleared
        uint8_t IREN    : 1;    ///< IrDA mode Enable
        uint8_t IRLP    : 1;    ///< IrDA Low Power
        uint8_t HDSEL   : 1;    ///< Half-Duplex Selection
        uint8_t NACK    : 1;    ///< Smartcard NACK enable
        uint8_t SCEN    : 1;    ///< Smartcard mode enable
        uint8_t res2    : 2;    ///< Reserved, must be kept cleared
      } reg;
      
    } CR5;


    /** UART4 Control register 6 (UART4_CR6) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t LSF     : 1;    ///< LIN Sync Field
        uint8_t LHDF    : 1;    ///< LIN Header Detection Flag
        uint8_t LHDIEN  : 1;    ///< LIN Header Detection Interrupt Enable
        uint8_t res     : 1;    ///< Reserved
        uint8_t LASE    : 1;    ///< LIN automatic resynchronisation enable
        uint8_t LSLV    : 1;    ///< LIN Slave Enable
        uint8_t res2    : 1;    ///< Reserved
        uint8_t LDUM    : 1;    ///< LIN Divider Update Method
      } reg;
      
    } CR6;


    /** UART4 guard time register register (UART4_GTR) */
    byte_t          GTR;


    /** UART4 prescaler register register register (UART4_PSCR) */
    byte_t          PSCR;

  } UART4_TypeDef;
    
  /// pointer to UART4 registers
  reg(UART4_BaseAddress, UART4_TypeDef, UART4);

  /* UART4 Module Reset Values */
  #define UART4_SR_RESET_VALUE   ((uint8_t)0xC0)
  #define UART4_BRR1_RESET_VALUE ((uint8_t)0x00)
  #define UART4_BRR2_RESET_VALUE ((uint8_t)0x00)
  #define UART4_CR1_RESET_VALUE  ((uint8_t)0x00)
  #define UART4_CR2_RESET_VALUE  ((uint8_t)0x00)
  #define UART4_CR3_RESET_VALUE  ((uint8_t)0x00)
  #define UART4_CR4_RESET_VALUE  ((uint8_t)0x00)
  #define UART4_CR5_RESET_VALUE  ((uint8_t)0x00)
  #define UART4_CR6_RESET_VALUE  ((uint8_t)0x00)
  #define UART4_GTR_RESET_VALUE  ((uint8_t)0x00)
  #define UART4_PSCR_RESET_VALUE ((uint8_t)0x00)
 
#endif /* (STM8AF622x) */



//------------------------
// Analog Digital Converter ADC1 (selected devices) with result buffer & analog watchdog etc --> done
//------------------------
#if defined(STM8S105) || defined(STM8S005) || defined(STM8S103) || defined(STM8S003) || \
    defined(STM8S903) || defined(STM8AF626x) || defined(STM8AF622x)

  /** struct containing ADC1 registers (selected devices) */
  typedef struct {

    /** ADC1 10-bit Data Buffer Register 0 (ADC1_DB0R) */
    word_t        DB0R;                    
                                         
    /** ADC1 10-bit Data Buffer Register 1 (ADC1_DB1R) */
    word_t        DB1R;                    
                                         
    /** ADC1 10-bit Data Buffer Register 2 (ADC1_DB2R) */
    word_t        DB2R;                    
                                         
    /** ADC1 10-bit Data Buffer Register 3 (ADC1_DB3R) */
    word_t        DB3R;                    
                                         
    /** ADC1 10-bit Data Buffer Register 4 (ADC1_DB4R) */
    word_t        DB4R;                    
                                         
    /** ADC1 10-bit Data Buffer Register 5 (ADC1_DB5R) */
    word_t        DB5R;                    
                                         
    /** ADC1 10-bit Data Buffer Register 6 (ADC1_DB6R) */
    word_t        DB6R;                    
                                         
    /** ADC1 10-bit Data Buffer Register 7 (ADC1_DB7R) */
    word_t        DB7R;                    
                                         
    /** ADC1 10-bit Data Buffer Register 8 (ADC1_DB8R) */
    word_t        DB8R;                    
                                         
    /** ADC1 10-bit Data Buffer Register 9 (ADC1_DB9R) */
    word_t        DB9R;


    /** Reserved register (12B) */
    uint8_t       res[12]; 


    /** ADC1 control/status register (ADC1_CSR) */
    union {

      /// bytewise access to register
      uint8_t	 byte;

      /// bitwise access to register
      struct {
        uint8_t CH			: 4;		///< Channel selection bits
        uint8_t AWDIE		: 1;		///< Analog watchdog interrupt enable
        uint8_t EOCIE		: 1;		///< Interrupt enable for EOC
        uint8_t AWD			: 1;		///< Analog Watchdog flag
        uint8_t EOC			: 1;		///< End of conversion
      } reg;

    } CSR;


    /** ADC1 Configuration Register 1 (ADC1_CR1) */
    union {

      /// bytewise access to register
      uint8_t	 byte;

      /// bitwise access to register
      struct {
        uint8_t ADON		: 1;		///< A/D Converter on/off
        uint8_t CONT		: 1;		///< Continuous conversion
        uint8_t res			: 2;		///< Reserved, always read as 0
        uint8_t SPSEL		: 3;		///< Clock prescaler selection
        uint8_t res2		: 1;		///< Reserved, always read as 0
      } reg;

    } CR1;


    /** ADC1 Configuration Register 2 (ADC1_CR2) */
    union {

      /// bytewise access to register
      uint8_t	 byte;

      /// bitwise access to register
      struct {
        uint8_t res			: 1;		///< Reserved, must be kept cleared
        uint8_t SCAN		: 1;		///< Scan mode enable
        uint8_t res2		: 1;		///< Reserved, must be kept cleared
        uint8_t ALIGN		: 1;		///< Data alignment
        uint8_t EXTSEL	: 2;		///< External event selection
        uint8_t EXTTRIG : 1;		///< External trigger enable
        uint8_t res3		: 1;		///< Reserved, must be kept cleared
      } reg;

    } CR2;


    /** ADC1 Configuration Register 3 (ADC1_CR3) */
    union {

      /// bytewise access to register
      uint8_t	 byte;

      /// bitwise access to register
      struct {
        uint8_t res			: 6;		///< Reserved, must be kept cleared
        uint8_t OVR			: 1;		///< Overrun flag
        uint8_t DBUF		: 1;		///< Data buffer enable
      } reg;

    } CR3;


    /** ADC1 (unbuffered) 10-bit measurement result (ADC1_DR) */
    word_t        DR;

    /** ADC1 Schmitt trigger disable register (ADC1_TDR) */
    word_t        TDR;

    /** ADC1 watchdog high threshold register (ADC1_HTR) */
    word_t        HTR;

    /** ADC1 watchdog low threshold register (ADC1_LTR) */
    word_t        LTR;

    /** ADC1 watchdog status register (ADC1_AWSR) */
    word_t        AWSR;

    /** ADC1 watchdog control register (ADC1_AWCR) */
    word_t        AWCR;

  } ADC1_TypeDef;
    
  /// pointer to ADC1 registers
  reg(ADC1_BaseAddress, ADC1_TypeDef, ADC1);

  /* ADC1 Module Reset Values */
  #define  ADC1_CSR_RESET_VALUE    ((uint8_t)0x00)
  #define  ADC1_CR1_RESET_VALUE    ((uint8_t)0x00)
  #define  ADC1_CR2_RESET_VALUE    ((uint8_t)0x00)
  #define  ADC1_CR3_RESET_VALUE    ((uint8_t)0x00)
  #define  ADC1_TDRL_RESET_VALUE   ((uint8_t)0x00)
  #define  ADC1_TDRH_RESET_VALUE   ((uint8_t)0x00)
  #define  ADC1_HTRL_RESET_VALUE   ((uint8_t)0x03)
  #define  ADC1_HTRH_RESET_VALUE   ((uint8_t)0xFF)
  #define  ADC1_LTRH_RESET_VALUE   ((uint8_t)0x00)
  #define  ADC1_LTRL_RESET_VALUE   ((uint8_t)0x00)
  #define  ADC1_AWCRH_RESET_VALUE  ((uint8_t)0x00)
  #define  ADC1_AWCRL_RESET_VALUE  ((uint8_t)0x00)

#endif // ADC1 on STM8S105 || STM8S103 || STM8S005 || STM8S003 || STM8S903 || STM8AF626x || STM8AF622x



//------------------------
// Analog Digital Converter ADC2 (selected devices) without result buffer, analog watchdog etc. --> done
//------------------------
#if defined(STM8S208) || defined(STM8S207) || defined (STM8S007) || defined (STM8AF52Ax) || defined (STM8AF62Ax)

  /** struct containing ADC2 registers (selected devices) */
  typedef struct {

    /** ADC2 control/status register (ADC2_CSR) */
    union {

      /// bytewise access to register
      uint8_t	 byte;

      /// bitwise access to register
      struct {
        uint8_t CH			: 4;		///< Channel selection bits
        uint8_t AWDIE		: 1;		///< Analog watchdog interrupt enable
        uint8_t EOCIE		: 1;		///< Interrupt enable for EOC
        uint8_t AWD			: 1;		///< Analog Watchdog flag
        uint8_t EOC			: 1;		///< End of conversion
      } reg;

    } CSR;


    /** ADC2 Configuration Register 1 (ADC2_CR1) */
    union {

      /// bytewise access to register
      uint8_t	 byte;

      /// bitwise access to register
      struct {
        uint8_t ADON		: 1;		///< A/D Converter on/off
        uint8_t CONT		: 1;		///< Continuous conversion
        uint8_t res			: 2;		///< Reserved, always read as 0
        uint8_t SPSEL		: 3;		///< Clock prescaler selection
        uint8_t res2		: 1;		///< Reserved, always read as 0
      } reg;

    } CR1;


    /** ADC2 Configuration Register 2 (ADC2_CR2) */
    union {

      /// bytewise access to register
      uint8_t	 byte;

      /// bitwise access to register
      struct {
        uint8_t res			: 1;		///< Reserved, must be kept cleared
        uint8_t SCAN		: 1;		///< Scan mode enable
        uint8_t res2		: 1;		///< Reserved, must be kept cleared
        uint8_t ALIGN		: 1;		///< Data alignment
        uint8_t EXTSEL	: 2;		///< External event selection
        uint8_t EXTTRIG : 1;		///< External trigger enable
        uint8_t res3		: 1;		///< Reserved, must be kept cleared
      } reg;

    } CR2;


    /** Reserved register (1B) */
    uint8_t         res[1]; 


    /** ADC2 (unbuffered) 10-bit measurement result (ADC2_DR) */
    word_t          DR;

    /** ADC2 Schmitt trigger disable register (ADC2_TDR) */
    word_t          TDR;

  } ADC2_TypeDef;
    
  /// pointer to all ADC2 registers
  reg(ADC2_BaseAddress, ADC2_TypeDef, ADC2);

  /* ADC2 Module Reset Values */
  #define  ADC2_CSR_RESET_VALUE  ((uint8_t)0x00)
  #define  ADC2_CR1_RESET_VALUE  ((uint8_t)0x00)
  #define  ADC2_CR2_RESET_VALUE  ((uint8_t)0x00)
  #define  ADC2_TDRL_RESET_VALUE ((uint8_t)0x00)
  #define  ADC2_TDRH_RESET_VALUE ((uint8_t)0x00)

#endif // ADC2 on STM8S208 || STM8S207 || STM8S007 || STM8AF52Ax || STM8AF62Ax



//------------------------
// 16-Bit Timer TIM1 (all devices)
//------------------------
#if (1)   // dummy for filtering out in editor

  /** struct containing TIM1 registers (all devices) */
  typedef struct {

    /** TIM1 Control register 1 (TIM1_CR1) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t CEN     : 1;    ///< Counter enable
        uint8_t UDIS    : 1;    ///< Update disable
        uint8_t URS     : 1;    ///< Update request source
        uint8_t OPM     : 1;    ///< One-pulse mode
        uint8_t DIR     : 1;    ///< Direction
        uint8_t CMS     : 2;    ///< Center-aligned mode selection
        uint8_t ARPE    : 1;    ///< Auto-reload preload enable
      } reg;
      
    } CR1;


    /** TIM1 Control register 2 (TIM1_CR2) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t CCPC    : 1;    ///< Capture/compare preloaded control
        uint8_t res     : 1;    ///< Reserved, forced by hardware to 0
        uint8_t COMS    : 1;    ///< Capture/compare control update selection
        uint8_t res2    : 1;    ///< Reserved, must be kept cleared
        uint8_t MMS     : 3;    ///< Master mode selection
        uint8_t res3    : 1;    ///< Reserved
      } reg;
      
    } CR2;


    /** Slave mode control register (TIM1_SMCR) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t SMS     : 3;    ///< Clock/trigger/slave mode selection
        uint8_t res     : 1;    ///< Reserved
        uint8_t TS      : 3;    ///< Trigger selection
        uint8_t MSM     : 1;    ///< Master/slave mode
      } reg;
      
    } SMCR;


    /** TIM1 External trigger register (TIM1_ETR) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t ETF     : 4;    ///< External trigger filter
        uint8_t ETPS    : 2;    ///< External trigger prescaler
        uint8_t ECE     : 1;    ///< External clock enable
        uint8_t ETP     : 1;    ///< External trigger polarity
      } reg;
      
    } ETR;


    /** TIM1 Interrupt enable register (TIM1_IER) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t UIE     : 1;    ///< Update interrupt enable
        uint8_t CC1IE   : 1;    ///< Capture/compare 1 interrupt enable
        uint8_t CC2IE   : 1;    ///< Capture/compare 2 interrupt enable
        uint8_t CC3IE   : 1;    ///< Capture/compare 3 interrupt enable
        uint8_t CC4IE   : 1;    ///< Capture/compare 4 interrupt enable
        uint8_t COMIE   : 1;    ///< Commutation interrupt enable
        uint8_t TIE     : 1;    ///< Trigger interrupt enable
        uint8_t BIE     : 1;    ///< Break interrupt enable
      } reg;
      
    } IER;


    /** TIM1 Status register 1 (TIM1_SR1) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t UIF     : 1;    ///< Update interrupt flag
        uint8_t CC1IF   : 1;    ///< Capture/compare 1 interrupt flag
        uint8_t CC2IF   : 1;    ///< Capture/compare 2 interrupt flag
        uint8_t CC3IF   : 1;    ///< Capture/compare 3 interrupt flag
        uint8_t CC4IF   : 1;    ///< Capture/compare 4 interrupt flag
        uint8_t COMIF   : 1;    ///< Commutation interrupt flag
        uint8_t TIF     : 1;    ///< Trigger interrupt flag
        uint8_t BIF     : 1;    ///< Break interrupt flag
      } reg;
      
    } SR1;


    /** TIM1 Status register 2 (TIM1_SR2) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t res     : 1;    ///< Reserved, must be kept cleared
        uint8_t CC1OF   : 1;    ///< Capture/compare 1 overcapture flag
        uint8_t CC2OF   : 1;    ///< Capture/compare 2 overcapture flag
        uint8_t CC3OF   : 1;    ///< Capture/compare 3 overcapture flag
        uint8_t CC4OF   : 1;    ///< Capture/compare 4 overcapture flag
        uint8_t res2    : 3;    ///< Reserved, must be kept cleared
      } reg;
      
    } SR2;


    /** TIM1 Event generation register (TIM1_EGR) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t UG      : 1;    ///< Update generation
        uint8_t CC1G    : 1;    ///< Capture/compare 1 generation
        uint8_t CC2G    : 1;    ///< Capture/compare 2 generation
        uint8_t CC3G    : 1;    ///< Capture/compare 3 generation
        uint8_t CC4G    : 1;    ///< Capture/compare 4 generation
        uint8_t COMG    : 1;    ///< Capture/compare control update generation
        uint8_t TG      : 1;    ///< Trigger generation
        uint8_t BG      : 1;    ///< Break generation
      } reg;
      
    } EGR;


    /** TIM1 Capture/compare mode register 1 (TIM1_CCMR1) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register (output mode)
      struct {
        uint8_t CC1S    : 2;    ///< Capture/compare 1 selection
        uint8_t OC1FE   : 1;    ///< Output compare 1 fast enable
        uint8_t OC1PE   : 1;    ///< Output compare 1 preload enable
        uint8_t OC1M    : 3;    ///< Output compare 1 mode
        uint8_t OC1CE   : 1;    ///< Output compare 1 clear enable
      } regOut;
      
      /// bitwise access to register (input mode)
      struct {
        uint8_t CC1S    : 2;    ///< Capture/compare 1 selection
        uint8_t IC1PSC  : 2;    ///< Input capture 1 prescaler
        uint8_t IC1F    : 4;    ///< Input capture 1 filter
      } regIn;
      
    } CCMR1;


    /** TIM1 Capture/compare mode register 2 (TIM1_CCMR2) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register (output mode)
      struct {
        uint8_t CC2S    : 2;    ///< Capture/compare 2 selection
        uint8_t OC2FE   : 1;    ///< Output compare 2 fast enable
        uint8_t OC2PE   : 1;    ///< Output compare 2 preload enable
        uint8_t OC2M    : 3;    ///< Output compare 2 mode
        uint8_t OC2CE   : 1;    ///< Output compare 2 clear enable
      } regOut;
      
      /// bitwise access to register (input mode)
      struct {
        uint8_t CC2S    : 2;    ///< Capture/compare 2 selection
        uint8_t IC2PSC  : 2;    ///< Input capture 2 prescaler
        uint8_t IC2F    : 4;    ///< Input capture 2 filter
      } regIn;
      
    } CCMR2;


    /** TIM1 Capture/compare mode register 3 (TIM1_CCMR3) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register (output mode)
      struct {
        uint8_t CC3S    : 2;    ///< Capture/compare 3 selection
        uint8_t OC3FE   : 1;    ///< Output compare 3 fast enable
        uint8_t OC3PE   : 1;    ///< Output compare 3 preload enable
        uint8_t OC3M    : 3;    ///< Output compare 3 mode
        uint8_t OC3CE   : 1;    ///< Output compare 3 clear enable
      } regOut;
        
      /// bitwise access to register (input mode)
      struct {
        uint8_t CC3S    : 2;    ///< Capture/compare 3 selection
        uint8_t IC3PSC  : 2;    ///< Input capture 3 prescaler
        uint8_t IC3F    : 4;    ///< Input capture 3 filter
      } regIn;

    } CCMR3;


    /** TIM1 Capture/compare mode register 4 (TIM1_CCMR4) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register (output mode)
      struct {
        uint8_t CC4S    : 2;    ///< Capture/compare 4 selection
        uint8_t OC4FE   : 1;    ///< Output compare 4 fast enable
        uint8_t OC4PE   : 1;    ///< Output compare 4 preload enable
        uint8_t OC4M    : 3;    ///< Output compare 4 mode
        uint8_t OC4CE   : 1;    ///< Output compare 4 clear enable
      } regOut;
        
      /// bitwise access to register (input mode)
      struct {
        uint8_t CC4S    : 2;    ///< Capture/compare 4 selection
        uint8_t IC4PSC  : 2;    ///< Input capture 4 prescaler
        uint8_t IC4F    : 4;    ///< Input capture 4 filter
      } regIn;
      
    } CCMR4;


    /** TIM1 Capture/compare enable register 1 (TIM1_CCER1) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t CC1E    : 1;    ///< Capture/compare 1 output enable
        uint8_t CC1P    : 1;    ///< Capture/compare 1 output polarity
        uint8_t CC1NE   : 1;    ///< Capture/compare 1 complementary output enable
        uint8_t CC1NP   : 1;    ///< Capture/compare 1 complementary output polarity
        uint8_t CC2E    : 1;    ///< Capture/compare 2 output enable
        uint8_t CC2P    : 1;    ///< Capture/compare 2 output polarity
        uint8_t CC2NE   : 1;    ///< Capture/compare 2 complementary output enable
        uint8_t CC2NP   : 1;    ///< Capture/compare 2 complementary output polarity
      } reg;
      
    } CCER1;


    /** TIM1 Capture/compare enable register 2 (TIM1_CCER2) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t CC3E    : 1;    ///< Capture/compare 3 output enable
        uint8_t CC3P    : 1;    ///< Capture/compare 3 output polarity
        uint8_t CC3NE   : 1;    ///< Capture/compare 3 complementary output enable
        uint8_t CC3NP   : 1;    ///< Capture/compare 3 complementary output polarity
        uint8_t CC4E    : 1;    ///< Capture/compare 4 output enable
        uint8_t CC4P    : 1;    ///< Capture/compare 4 output polarity
        uint8_t res     : 2;    ///< Reserved
      } reg;
      
    } CCER2;


    /** TIM1 16-bit counter (TIM1_CNTR) */
    word_t        CNTR;
    
    
    /** TIM1 16-bit prescaler (TIM1_PSCR) */
    word_t        PSCR;
    
    
    /** TIM1 16-bit re-load value (TIM1_ARR) */
    word_t        ARR;
    
    
    /** TIM1 Repetition counter (TIM1_RCR) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t REP     : 8;    ///< Repetition counter value
      } reg;
      
    } RCR;


    /** TIM1 16-bit capture/compare value 1 (TIM1_CC1R) */
    word_t        CC1R;


    /** TIM1 16-bit capture/compare value 2 (TIM1_CC2R) */
    word_t        CC2R;


    /** TIM1 16-bit capture/compare value 3 (TIM1_CC3R) */
    word_t        CC3R;


    /** TIM1 16-bit capture/compare value 4 (TIM1_CC4R) */
    word_t        CC4R;
    
    
    /** TIM1 Break register (TIM1_BKR) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t LOCK    : 2;    ///< Lock configuration
        uint8_t OSSI    : 1;    ///< Off state selection for idle mode
        uint8_t OSSR    : 1;    ///< Off state selection for Run mode
        uint8_t BKE     : 1;    ///< Break enable
        uint8_t BKP     : 1;    ///< Break polarity
        uint8_t AOE     : 1;    ///< Automatic output enable
        uint8_t MOE     : 1;    ///< Main output enable
      } reg;
      
    } BKR;


    /** TIM1 Dead-time register (TIM1_DTR) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t DTG     : 8;    ///< Deadtime generator set-up
      } reg;
      
    } DTR;


    /** TIM1 Output idle state register (TIM1_OISR) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t OIS1    : 1;    ///< Output idle state 1 (OC1 output)
        uint8_t OIS1N   : 1;    ///< Output idle state 1 (OC1N output)
        uint8_t OIS2    : 1;    ///< Output idle state 2 (OC2 output)
        uint8_t OIS2N   : 1;    ///< Output idle state 2 (OC2N output)
        uint8_t OIS3    : 1;    ///< Output idle state 3 (OC3 output)
        uint8_t OIS3N   : 1;    ///< Output idle state 3 (OC3N output)
        uint8_t OIS4    : 1;    ///< Output idle state 4 (OC4 output)
        uint8_t res     : 1;    ///< Reserved, forced by hardware to 0
      } reg;
      
    } OISR;

  } TIM1_TypeDef;

  /// pointer to all TIM1 registers
  reg(TIM1_BaseAddress, TIM1_TypeDef, TIM1);

  /* TIM1 Module Reset Values */
  #define TIM1_CR1_RESET_VALUE   ((uint8_t)0x00)
  #define TIM1_CR2_RESET_VALUE   ((uint8_t)0x00)
  #define TIM1_SMCR_RESET_VALUE  ((uint8_t)0x00)
  #define TIM1_ETR_RESET_VALUE   ((uint8_t)0x00)
  #define TIM1_IER_RESET_VALUE   ((uint8_t)0x00)
  #define TIM1_SR1_RESET_VALUE   ((uint8_t)0x00)
  #define TIM1_SR2_RESET_VALUE   ((uint8_t)0x00)
  #define TIM1_EGR_RESET_VALUE   ((uint8_t)0x00)
  #define TIM1_CCMR1_RESET_VALUE ((uint8_t)0x00)
  #define TIM1_CCMR2_RESET_VALUE ((uint8_t)0x00)
  #define TIM1_CCMR3_RESET_VALUE ((uint8_t)0x00)
  #define TIM1_CCMR4_RESET_VALUE ((uint8_t)0x00)
  #define TIM1_CCER1_RESET_VALUE ((uint8_t)0x00)
  #define TIM1_CCER2_RESET_VALUE ((uint8_t)0x00)
  #define TIM1_CNTRH_RESET_VALUE ((uint8_t)0x00)
  #define TIM1_CNTRL_RESET_VALUE ((uint8_t)0x00)
  #define TIM1_PSCRH_RESET_VALUE ((uint8_t)0x00)
  #define TIM1_PSCRL_RESET_VALUE ((uint8_t)0x00)
  #define TIM1_ARRH_RESET_VALUE  ((uint8_t)0xFF)
  #define TIM1_ARRL_RESET_VALUE  ((uint8_t)0xFF)
  #define TIM1_RCR_RESET_VALUE   ((uint8_t)0x00)
  #define TIM1_CCR1H_RESET_VALUE ((uint8_t)0x00)
  #define TIM1_CCR1L_RESET_VALUE ((uint8_t)0x00)
  #define TIM1_CCR2H_RESET_VALUE ((uint8_t)0x00)
  #define TIM1_CCR2L_RESET_VALUE ((uint8_t)0x00)
  #define TIM1_CCR3H_RESET_VALUE ((uint8_t)0x00)
  #define TIM1_CCR3L_RESET_VALUE ((uint8_t)0x00)
  #define TIM1_CCR4H_RESET_VALUE ((uint8_t)0x00)
  #define TIM1_CCR4L_RESET_VALUE ((uint8_t)0x00)
  #define TIM1_BKR_RESET_VALUE   ((uint8_t)0x00)
  #define TIM1_DTR_RESET_VALUE   ((uint8_t)0x00)
  #define TIM1_OISR_RESET_VALUE  ((uint8_t)0x00)

#endif // (1)



//------------------------
// 16-Bit Timer TIM2 (selected devices)
//------------------------
#if defined(STM8S208) || defined(STM8S207) || defined (STM8S007) || defined(STM8S103) || \
    defined(STM8S003) || defined(STM8S105) || defined(STM8S005) || defined (STM8AF52Ax) || \
    defined (STM8AF62Ax) || defined (STM8AF626x)

  /** struct containing TIM2 registers */
  typedef struct {

    /** TIM2 Control register (TIM2_CR1) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t CEN     : 1;    ///< Counter enable
        uint8_t UDIS    : 1;    ///< Update disable
        uint8_t URS     : 1;    ///< Update request source
        uint8_t OPM     : 1;    ///< One-pulse mode
        uint8_t res     : 3;    ///< Reserved
        uint8_t ARPE    : 1;    ///< Auto-reload preload enable
      } reg;
      
    } CR1;

    
    /** Reserved registers on selected devices (2B) */
    #if defined(STM8S103) || defined(STM8S003)
      uint8_t         res[2];
    #endif
      

    /** TIM2 Interrupt enable (TIM2_IER) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t UIE     : 1;    ///< Update interrupt enable
        uint8_t CC1IE   : 1;    ///< Capture/compare 1 interrupt enable
        uint8_t CC2IE   : 1;    ///< Capture/compare 2 interrupt enable
        uint8_t CC3IE   : 1;    ///< Capture/compare 3 interrupt enable
        uint8_t res     : 4;    ///< Reserved 
      } reg;
      
    } IER;


    /** TIM2 Status register 1 (TIM2_SR1) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t UIF     : 1;    ///< Update interrupt flag
        uint8_t CC1IF   : 1;    ///< Capture/compare 1 interrupt flag
        uint8_t CC2IF   : 1;    ///< Capture/compare 2 interrupt flag
        uint8_t CC3IF   : 1;    ///< Capture/compare 3 interrupt flag
        uint8_t res     : 4;    ///< Reserved
      } reg;
      
    } SR1;


    /** TIM2 Status register 2 (TIM2_SR2) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t res     : 1;    ///< Reserved
        uint8_t CC1OF   : 1;    ///< Capture/compare 1 overcapture flag
        uint8_t CC2OF   : 1;    ///< Capture/compare 2 overcapture flag
        uint8_t CC3OF   : 1;    ///< Capture/compare 3 overcapture flag
        uint8_t res2    : 4;    ///< Reserved
      } reg;
      
    } SR2;


    /** TIM2 Event Generation (TIM2_EGR) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t UG      : 1;    ///< Update generation
        uint8_t CC1G    : 1;    ///< Capture/compare 1 generation
        uint8_t CC2G    : 1;    ///< Capture/compare 2 generation
        uint8_t CC3G    : 1;    ///< Capture/compare 3 generation
        uint8_t res     : 4;    ///< Reserved
      } reg;
      
    } EGR;


    /** TIM2 Capture/compare mode register 1 (TIM2_CCMR1) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register (output mode)
      struct {
        uint8_t CC1S    : 2;    ///< Capture/compare 1 selection
        uint8_t res     : 1;    ///< Reserved
        uint8_t OC1PE   : 1;    ///< Output compare 1 preload enable
        uint8_t OC1M    : 3;    ///< Output compare 1 mode
        uint8_t res2    : 1;    ///< Reserved
      } regOut;
      
      /// bitwise access to register (input mode)
      struct {
        uint8_t CC1S    : 2;    ///< Capture/compare 1 selection
        uint8_t IC1PSC  : 2;    ///< Input capture 1 prescaler
        uint8_t IC1F    : 4;    ///< Input capture 1 filter
      } regIn;
      
    } CCMR1;


    /** TIM2 Capture/compare mode register 2 (TIM2_CCMR2) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register (output mode)
      struct {
        uint8_t CC2S    : 2;    ///< Capture/compare 2 selection
        uint8_t res     : 1;    ///< Reserved
        uint8_t OC2PE   : 1;    ///< Output compare 2 preload enable
        uint8_t OC2M    : 3;    ///< Output compare 2 mode
        uint8_t res2    : 1;    ///< Reserved
      } regOut;
      
      /// bitwise access to register (input mode)
      struct {
        uint8_t CC2S    : 2;    ///< Capture/compare 2 selection
        uint8_t IC2PSC  : 2;    ///< Input capture 2 prescaler
        uint8_t IC2F    : 4;    ///< Input capture 2 filter
      } regIn;
      
    } CCMR2;


    /** TIM2 Capture/compare mode register 3 (TIM2_CCMR3) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register (output mode)
      struct {
        uint8_t CC3S    : 2;    ///< Capture/compare 3 selection
        uint8_t res     : 1;    ///< Reserved
        uint8_t OC3PE   : 1;    ///< Output compare 3 preload enable
        uint8_t OC3M    : 3;    ///< Output compare 3 mode
        uint8_t OC3CE   : 1;    ///< Reserved
      } regOut;
        
      /// bitwise access to register (input mode)
      struct {
        uint8_t CC3S    : 2;    ///< Capture/compare 3 selection
        uint8_t IC3PSC  : 2;    ///< Input capture 3 prescaler
        uint8_t IC3F    : 4;    ///< Input capture 3 filter
      } regIn;

    } CCMR3;


    /** TIM2 Capture/compare enable register 1 (TIM2_CCER1) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t CC1E    : 1;    ///< Capture/compare 1 output enable
        uint8_t CC1P    : 1;    ///< Capture/compare 1 output polarity
        uint8_t res     : 2;    ///< Reserved
        uint8_t CC2E    : 1;    ///< Capture/compare 2 output enable
        uint8_t CC2P    : 1;    ///< Capture/compare 2 output polarity
        uint8_t res2    : 2;    ///< Reserved
      } reg;
      
    } CCER1;


    /** TIM2 Capture/compare enable register 2 (TIM2_CCER2) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t CC3E    : 1;    ///< Capture/compare 3 output enable
        uint8_t CC3P    : 1;    ///< Capture/compare 3 output polarity
        uint8_t res     : 6;    ///< Reserved
      } reg;
      
    } CCER2;


    /** TIM2 16-bit counter (TIM2_CNTR) */
    word_t        CNTR;


    /** TIM2 clock prescaler (TIM2_PSCR) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t PSC     : 4;    ///< clock prescaler
        uint8_t res     : 4;    ///< Reserved
      } reg;
      
    } PSCR;
    
    
    /** TIM2 16-bit re-load value (TIM2_ARR) */
    word_t        ARR;


    /** TIM2 16-bit capture/compare value 1 (TIM2_CC1R) */
    word_t        CC1R;


    /** TIM2 16-bit capture/compare value 2 (TIM2_CC2R) */
    word_t        CC2R;


    /** TIM2 16-bit capture/compare value 3 (TIM2_CC3R) */
    word_t        CC3R;

  } TIM2_TypeDef;

  /// pointer to all TIM2 registers (selected devices)
  reg(TIM2_BaseAddress, TIM2_TypeDef, TIM2);

  /* TIM2 Module Reset Values */
  #define TIM2_CR1_RESET_VALUE   ((uint8_t)0x00)
  #define TIM2_IER_RESET_VALUE   ((uint8_t)0x00)
  #define TIM2_SR1_RESET_VALUE   ((uint8_t)0x00)
  #define TIM2_SR2_RESET_VALUE   ((uint8_t)0x00)
  #define TIM2_EGR_RESET_VALUE   ((uint8_t)0x00)
  #define TIM2_CCMR1_RESET_VALUE ((uint8_t)0x00)
  #define TIM2_CCMR2_RESET_VALUE ((uint8_t)0x00)
  #define TIM2_CCMR3_RESET_VALUE ((uint8_t)0x00)
  #define TIM2_CCER1_RESET_VALUE ((uint8_t)0x00)
  #define TIM2_CCER2_RESET_VALUE ((uint8_t)0x00)
  #define TIM2_CNTRH_RESET_VALUE ((uint8_t)0x00)
  #define TIM2_CNTRL_RESET_VALUE ((uint8_t)0x00)
  #define TIM2_PSCR_RESET_VALUE  ((uint8_t)0x00)
  #define TIM2_ARRH_RESET_VALUE  ((uint8_t)0xFF)
  #define TIM2_ARRL_RESET_VALUE  ((uint8_t)0xFF)
  #define TIM2_CCR1H_RESET_VALUE ((uint8_t)0x00)
  #define TIM2_CCR1L_RESET_VALUE ((uint8_t)0x00)
  #define TIM2_CCR2H_RESET_VALUE ((uint8_t)0x00)
  #define TIM2_CCR2L_RESET_VALUE ((uint8_t)0x00)
  #define TIM2_CCR3H_RESET_VALUE ((uint8_t)0x00)
  #define TIM2_CCR3L_RESET_VALUE ((uint8_t)0x00)

#endif /* (STM8S208) ||(STM8S207)  || (STM8S103) || (STM8S105) || (STM8AF52Ax) || (STM8AF62Ax) || (STM8AF626x)*/



//------------------------
// 16-Bit Timer TIM3 (selected devices)
//------------------------
#if defined(STM8S208) || defined(STM8S207) || defined (STM8S007) || defined(STM8S105) || \
    defined(STM8S005) || defined (STM8AF52Ax) || defined (STM8AF62Ax) || defined (STM8AF626x)

  /** struct containing TIM3 registers */
  typedef struct {

    /** TIM3 Control register (TIM3_CR1) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t CEN     : 1;    ///< Counter enable
        uint8_t UDIS    : 1;    ///< Update disable
        uint8_t URS     : 1;    ///< Update request source
        uint8_t OPM     : 1;    ///< One-pulse mode
        uint8_t res     : 3;    ///< Reserved
        uint8_t ARPE    : 1;    ///< Auto-reload preload enable
      } reg;
      
    } CR1;


    /** TIM3 Interrupt enable (TIM3_IER) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t UIE     : 1;    ///< Update interrupt enable
        uint8_t CC1IE   : 1;    ///< Capture/compare 1 interrupt enable
        uint8_t CC2IE   : 1;    ///< Capture/compare 2 interrupt enable
        uint8_t res     : 5;    ///< Reserved 
      } reg;
      
    } IER;


    /** TIM3 Status register 1 (TIM3_SR1) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t UIF     : 1;    ///< Update interrupt flag
        uint8_t CC1IF   : 1;    ///< Capture/compare 1 interrupt flag
        uint8_t CC2IF   : 1;    ///< Capture/compare 2 interrupt flag
        uint8_t res     : 5;    ///< Reserved
      } reg;
      
    } SR1;


    /** TIM3 Status register 2 (TIM3_SR2) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t res     : 1;    ///< Reserved
        uint8_t CC1OF   : 1;    ///< Capture/compare 1 overcapture flag
        uint8_t CC2OF   : 1;    ///< Capture/compare 2 overcapture flag
        uint8_t res2    : 5;    ///< Reserved
      } reg;
      
    } SR2;


    /** TIM3 Event Generation (TIM3_EGR) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t UG      : 1;    ///< Update generation
        uint8_t CC1G    : 1;    ///< Capture/compare 1 generation
        uint8_t CC2G    : 1;    ///< Capture/compare 2 generation
        uint8_t res     : 5;    ///< Reserved
      } reg;
      
    } EGR;


    /** TIM3 Capture/compare mode register 1 (TIM3_CCMR1) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register (output mode)
      struct {
        uint8_t CC1S    : 2;    ///< Capture/compare 1 selection
        uint8_t res     : 1;    ///< Reserved
        uint8_t OC1PE   : 1;    ///< Output compare 1 preload enable
        uint8_t OC1M    : 3;    ///< Output compare 1 mode
        uint8_t res2    : 1;    ///< Reserved
      } regOut;
      
      /// bitwise access to register (input mode)
      struct {
        uint8_t CC1S    : 2;    ///< Capture/compare 1 selection
        uint8_t IC1PSC  : 2;    ///< Input capture 1 prescaler
        uint8_t IC1F    : 4;    ///< Input capture 1 filter
      } regIn;
      
    } CCMR1;


    /** TIM3 Capture/compare mode register 2 (TIM3_CCMR2) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register (output mode)
      struct {
        uint8_t CC2S    : 2;    ///< Capture/compare 2 selection
        uint8_t res     : 1;    ///< Reserved
        uint8_t OC2PE   : 1;    ///< Output compare 2 preload enable
        uint8_t OC2M    : 3;    ///< Output compare 2 mode
        uint8_t res2    : 1;    ///< Reserved
      } regOut;
      
      /// bitwise access to register (input mode)
      struct {
        uint8_t CC2S    : 2;    ///< Capture/compare 2 selection
        uint8_t IC2PSC  : 2;    ///< Input capture 2 prescaler
        uint8_t IC2F    : 4;    ///< Input capture 2 filter
      } regIn;
      
    } CCMR2;


    /** TIM3 Capture/compare enable register 1 (TIM3_CCER1) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t CC1E    : 1;    ///< Capture/compare 1 output enable
        uint8_t CC1P    : 1;    ///< Capture/compare 1 output polarity
        uint8_t res     : 2;    ///< Reserved
        uint8_t CC2E    : 1;    ///< Capture/compare 2 output enable
        uint8_t CC2P    : 1;    ///< Capture/compare 2 output polarity
        uint8_t res2    : 2;    ///< Reserved
      } reg;
      
    } CCER1;


    /** TIM3 16-bit counter (TIM3_CNTR) */
    word_t        CNTR;


    /** TIM3 clock prescaler (TIM3_PSCR) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t PSC     : 4;    ///< clock prescaler
        uint8_t res     : 4;    ///< Reserved
      } reg;
      
    } PSCR;
    
    
    /** TIM3 16-bit re-load value (TIM3_ARR) */
    word_t        ARR;


    /** TIM3 16-bit capture/compare value 1 (TIM3_CC1R) */
    word_t        CC1R;


    /** TIM3 16-bit capture/compare value 2 (TIM3_CC2R) */
    word_t        CC2R;

  } TIM3_TypeDef;

  /// pointer to all TIM3 registers (selected devices)
  reg(TIM3_BaseAddress, TIM3_TypeDef, TIM3);

  /* TIM3 Module Reset Values */
  #define TIM3_CR1_RESET_VALUE   ((uint8_t)0x00)
  #define TIM3_IER_RESET_VALUE   ((uint8_t)0x00)
  #define TIM3_SR1_RESET_VALUE   ((uint8_t)0x00)
  #define TIM3_SR2_RESET_VALUE   ((uint8_t)0x00)
  #define TIM3_EGR_RESET_VALUE   ((uint8_t)0x00)
  #define TIM3_CCMR1_RESET_VALUE ((uint8_t)0x00)
  #define TIM3_CCMR2_RESET_VALUE ((uint8_t)0x00)
  #define TIM3_CCER1_RESET_VALUE ((uint8_t)0x00)
  #define TIM3_CNTRH_RESET_VALUE ((uint8_t)0x00)
  #define TIM3_CNTRL_RESET_VALUE ((uint8_t)0x00)
  #define TIM3_PSCR_RESET_VALUE  ((uint8_t)0x00)
  #define TIM3_ARRH_RESET_VALUE  ((uint8_t)0xFF)
  #define TIM3_ARRL_RESET_VALUE  ((uint8_t)0xFF)
  #define TIM3_CCR1H_RESET_VALUE ((uint8_t)0x00)
  #define TIM3_CCR1L_RESET_VALUE ((uint8_t)0x00)
  #define TIM3_CCR2H_RESET_VALUE ((uint8_t)0x00)
  #define TIM3_CCR2L_RESET_VALUE ((uint8_t)0x00)

#endif /* (STM8S208) ||(STM8S207)  || (STM8S105) || (STM8AF62Ax) || (STM8AF52Ax) || (STM8AF626x)*/



//------------------------
// 8-Bit Timer TIM4 (selected devices)
//------------------------
/// TIM4 only implemented on selected devices
#if defined(STM8S208) ||defined(STM8S207) || defined (STM8S007) || defined(STM8S103) || \
    defined(STM8S003) || defined(STM8S105) || defined(STM8S005) || defined (STM8AF52Ax) || \
    defined (STM8AF62Ax) || defined (STM8AF626x)

  /** struct containing TIM4 registers (selected devices) */
  typedef struct {
  
    /** TIM4 Control register (TIM4_CR1) */
    union {
    
      /// bytewise access to register
      uint8_t  byte;  
    
      /// bitwise access to register
      struct {
        uint8_t CEN     : 1;    ///< Counter enable
        uint8_t UDIS    : 1;    ///< Update disable
        uint8_t URS     : 1;    ///< Update request source
        uint8_t OPM     : 1;    ///< One-pulse mode
        uint8_t res     : 3;    ///< Reserved
        uint8_t ARPE    : 1;    ///< Auto-reload preload enable
      } reg;
    
    } CR1;
  
  
    /** Reserved registers on selected devices (2B) */
    #if defined(STM8S103) || defined(STM8S003)
      uint8_t         res[2]; 
    #endif


    /** TIM4 Interrupt enable (TIM4_IER) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t UIE     : 1;    ///< Update interrupt enable
        uint8_t res     : 7;    ///< Reserved
      } reg;
      
    } IER;


    /** TIM4 Status register (TIM4_SR1) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t UIF     : 1;    ///< Update interrupt flag
        uint8_t res     : 7;    ///< Reserved
      } reg;
      
    } SR1;
  
  
    /** TIM4 Event Generation (TIM4_EGR) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t UG      : 1;    ///< Update generation
        uint8_t res     : 7;    ///< Reserved
      } reg;
      
    } EGR;
  
  
    /** TIM4 8-bit counter register (TIM4_CNTR) */
    byte_t            CNTR;
  
  
    /** TIM4 clock prescaler (TIM4_PSCR) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t PSC     : 3;    ///< clock prescaler
        uint8_t res     : 5;    ///< Reserved
      } reg;
      
    } PSCR;
  
  
    /** TIM4 8-bit auto-reload register (TIM4_ARR) */
    byte_t            ARR;
  
  } TIM4_TypeDef;
  
  /// pointer to TIM4 registers
  reg(TIM4_BaseAddress, TIM4_TypeDef, TIM4);

  /* TIM4 Module Reset Values */
  #define TIM4_CR1_RESET_VALUE  ((uint8_t)0x00)
  #define TIM4_IER_RESET_VALUE  ((uint8_t)0x00)
  #define TIM4_SR1_RESET_VALUE  ((uint8_t)0x00)
  #define TIM4_EGR_RESET_VALUE  ((uint8_t)0x00)
  #define TIM4_CNTR_RESET_VALUE ((uint8_t)0x00)
  #define TIM4_PSCR_RESET_VALUE ((uint8_t)0x00)
  #define TIM4_ARR_RESET_VALUE  ((uint8_t)0xFF)

#endif /* (STM8S208) ||(STM8S207)  || (STM8S103) || (STM8S105) || (STM8AF52Ax) || (STM8AF62Ax) || (STM8AF626x)*/



//------------------------
// 16-Bit Timer TIM5 (selected devices)
//------------------------
#if defined (STM8S903) || defined (STM8AF622x)

  /** struct containing TIM5 registers */
  typedef struct {

    /** TIM5 Control register (TIM5_CR1) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t CEN     : 1;    ///< Counter enable
        uint8_t UDIS    : 1;    ///< Update disable
        uint8_t URS     : 1;    ///< Update request source
        uint8_t OPM     : 1;    ///< One-pulse mode
        uint8_t res     : 3;    ///< Reserved
        uint8_t ARPE    : 1;    ///< Auto-reload preload enable
      } reg;
      
    } CR1;


    /** TIM5 Control register 2 (TIM5_CR2) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t CCPC    : 1;    ///< Capture/compare preloaded control
        uint8_t res     : 1;    ///< Reserved, forced by hardware to 0
        uint8_t COMS    : 1;    ///< Capture/compare control update selection
        uint8_t res2    : 1;    ///< Reserved, must be kept cleared
        uint8_t MMS     : 3;    ///< Master mode selection
        uint8_t res3    : 1;    ///< Reserved
      } reg;
      
    } CR2;


    /** TIM5 Slave mode control register (TIM5_SMCR) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t SMS     : 3;    ///< Clock/trigger/slave mode selection
        uint8_t res     : 1;    ///< Reserved
        uint8_t TS      : 3;    ///< Trigger selection
        uint8_t MSM     : 1;    ///< Master/slave mode
      } reg;
      
    } SMCR;


    /** TIM5 Interrupt enable (TIM5_IER) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t UIE     : 1;    ///< Update interrupt enable
        uint8_t CC1IE   : 1;    ///< Capture/compare 1 interrupt enable
        uint8_t CC2IE   : 1;    ///< Capture/compare 2 interrupt enable
        uint8_t CC3IE   : 1;    ///< Capture/compare 3 interrupt enable
        uint8_t res     : 2;    ///< Reserved 
        uint8_t TIE     : 1;    ///< Trigger interrupt enable
        uint8_t res2    : 1;    ///< Reserved
      } reg;
      
    } IER;


    /** TIM5 Status register 1 (TIM5_SR1) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t UIF     : 1;    ///< Update interrupt flag
        uint8_t CC1IF   : 1;    ///< Capture/compare 1 interrupt flag
        uint8_t CC2IF   : 1;    ///< Capture/compare 2 interrupt flag
        uint8_t CC3IF   : 1;    ///< Capture/compare 3 interrupt flag
        uint8_t res     : 2;    ///< Reserved
        uint8_t TIF     : 1;    ///< Trigger interrupt flag
        uint8_t res2    : 1;    ///< Reserved
      } reg;
      
    } SR1;


    /** TIM5 Status register 2 (TIM5_SR2) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t res     : 1;    ///< Reserved
        uint8_t CC1OF   : 1;    ///< Capture/compare 1 overcapture flag
        uint8_t CC2OF   : 1;    ///< Capture/compare 2 overcapture flag
        uint8_t CC3OF   : 1;    ///< Capture/compare 3 overcapture flag
        uint8_t res2    : 4;    ///< Reserved
      } reg;
      
    } SR2;


    /** TIM5 Event Generation (TIM5_EGR) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t UG      : 1;    ///< Update generation
        uint8_t CC1G    : 1;    ///< Capture/compare 1 generation
        uint8_t CC2G    : 1;    ///< Capture/compare 2 generation
        uint8_t CC3G    : 1;    ///< Capture/compare 3 generation
        uint8_t res     : 2;    ///< Reserved
        uint8_t TG      : 1;    ///< Trigger generation
        uint8_t res2    : 1;    ///< Reserved
      } reg;
      
    } EGR;


    /** TIM5 Capture/compare mode register 1 (TIM5_CCMR1) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register (output mode)
      struct {
        uint8_t CC1S    : 2;    ///< Capture/compare 1 selection
        uint8_t res     : 1;    ///< Reserved
        uint8_t OC1PE   : 1;    ///< Output compare 1 preload enable
        uint8_t OC1M    : 3;    ///< Output compare 1 mode
        uint8_t res2    : 1;    ///< Reserved
      } regOut;
      
      /// bitwise access to register (input mode)
      struct {
        uint8_t CC1S    : 2;    ///< Capture/compare 1 selection
        uint8_t IC1PSC  : 2;    ///< Input capture 1 prescaler
        uint8_t IC1F    : 4;    ///< Input capture 1 filter
      } regIn;
      
    } CCMR1;


    /** TIM5 Capture/compare mode register 2 (TIM5_CCMR2) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register (output mode)
      struct {
        uint8_t CC2S    : 2;    ///< Capture/compare 2 selection
        uint8_t res     : 1;    ///< Reserved
        uint8_t OC2PE   : 1;    ///< Output compare 2 preload enable
        uint8_t OC2M    : 3;    ///< Output compare 2 mode
        uint8_t res2    : 1;    ///< Reserved
      } regOut;
      
      /// bitwise access to register (input mode)
      struct {
        uint8_t CC2S    : 2;    ///< Capture/compare 2 selection
        uint8_t IC2PSC  : 2;    ///< Input capture 2 prescaler
        uint8_t IC2F    : 4;    ///< Input capture 2 filter
      } regIn;
      
    } CCMR2;


    /** TIM5 Capture/compare mode register 3 (TIM5_CCMR3) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register (output mode)
      struct {
        uint8_t CC3S    : 2;    ///< Capture/compare 3 selection
        uint8_t res     : 1;    ///< Reserved
        uint8_t OC3PE   : 1;    ///< Output compare 3 preload enable
        uint8_t OC3M    : 3;    ///< Output compare 3 mode
        uint8_t OC3CE   : 1;    ///< Reserved
      } regOut;
        
      /// bitwise access to register (input mode)
      struct {
        uint8_t CC3S    : 2;    ///< Capture/compare 3 selection
        uint8_t IC3PSC  : 2;    ///< Input capture 3 prescaler
        uint8_t IC3F    : 4;    ///< Input capture 3 filter
      } regIn;

    } CCMR3;


    /** TIM5 Capture/compare enable register 1 (TIM5_CCER1) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t CC1E    : 1;    ///< Capture/compare 1 output enable
        uint8_t CC1P    : 1;    ///< Capture/compare 1 output polarity
        uint8_t res     : 2;    ///< Reserved
        uint8_t CC2E    : 1;    ///< Capture/compare 2 output enable
        uint8_t CC2P    : 1;    ///< Capture/compare 2 output polarity
        uint8_t res2    : 2;    ///< Reserved
      } reg;
      
    } CCER1;


    /** TIM5 Capture/compare enable register 2 (TIM5_CCER2) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t CC3E    : 1;    ///< Capture/compare 3 output enable
        uint8_t CC3P    : 1;    ///< Capture/compare 3 output polarity
        uint8_t res     : 6;    ///< Reserved
      } reg;
      
    } CCER2;


    /** TIM5 16-bit counter (TIM5_CNTR) */
    word_t        CNTR;


    /** TIM5 clock prescaler (TIM5_PSCR) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t PSC     : 4;    ///< clock prescaler
        uint8_t res     : 4;    ///< Reserved
      } reg;
      
    } PSCR;


    /** TIM5 16-bit re-load value (TIM5_ARR) */
    word_t        ARR;


    /** TIM5 16-bit capture/compare value 1 (TIM5_CC1R) */
    word_t        CC1R;


    /** TIM5 16-bit capture/compare value 2 (TIM5_CC2R) */
    word_t        CC2R;


    /** TIM5 16-bit capture/compare value 3 (TIM5_CC3R) */
    word_t        CC3R;

  } TIM5_TypeDef;

  /// pointer to all TIM5 registers (selected devices)
  reg(TIM5_BaseAddress, TIM5_TypeDef, TIM5);

  /* TIM5 Module Reset Values */
  #define TIM5_CR1_RESET_VALUE   ((uint8_t)0x00)
  #define TIM5_CR2_RESET_VALUE 	 ((uint8_t)0x00)
  #define TIM5_SMCR_RESET_VALUE	 ((uint8_t)0x00)
  #define TIM5_IER_RESET_VALUE   ((uint8_t)0x00)
  #define TIM5_SR1_RESET_VALUE   ((uint8_t)0x00)
  #define TIM5_SR2_RESET_VALUE   ((uint8_t)0x00)
  #define TIM5_EGR_RESET_VALUE   ((uint8_t)0x00)
  #define TIM5_CCMR1_RESET_VALUE ((uint8_t)0x00)
  #define TIM5_CCMR2_RESET_VALUE ((uint8_t)0x00)
  #define TIM5_CCMR3_RESET_VALUE ((uint8_t)0x00)
  #define TIM5_CCER1_RESET_VALUE ((uint8_t)0x00)
  #define TIM5_CCER2_RESET_VALUE ((uint8_t)0x00)
  #define TIM5_CNTRH_RESET_VALUE ((uint8_t)0x00)
  #define TIM5_CNTRL_RESET_VALUE ((uint8_t)0x00)
  #define TIM5_PSCR_RESET_VALUE  ((uint8_t)0x00)
  #define TIM5_ARRH_RESET_VALUE  ((uint8_t)0xFF)
  #define TIM5_ARRL_RESET_VALUE  ((uint8_t)0xFF)
  #define TIM5_CCR1H_RESET_VALUE ((uint8_t)0x00)
  #define TIM5_CCR1L_RESET_VALUE ((uint8_t)0x00)
  #define TIM5_CCR2H_RESET_VALUE ((uint8_t)0x00)
  #define TIM5_CCR2L_RESET_VALUE ((uint8_t)0x00)
  #define TIM5_CCR3H_RESET_VALUE ((uint8_t)0x00)
  #define TIM5_CCR3L_RESET_VALUE ((uint8_t)0x00)

#endif /* (STM8S903) || (STM8AF622x) */ 



//------------------------
// 8-Bit Timer TIM6 (selected devices)
//------------------------
#if defined (STM8S903) || defined (STM8AF622x)

  /** struct containing TIM6 registers (selected devices) */
  typedef struct {

    /** TIM6 Control register (TIM6_CR1) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t CEN     : 1;    ///< Counter enable
        uint8_t UDIS    : 1;    ///< Update disable
        uint8_t URS     : 1;    ///< Update request source
        uint8_t OPM     : 1;    ///< One-pulse mode
        uint8_t res     : 3;    ///< Reserved
        uint8_t ARPE    : 1;    ///< Auto-reload preload enable
      } reg;
      
    } CR1;


    /** TIM6 Control register 2 (TIM6_CR2) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t res     : 4;    ///< Reserved
        uint8_t MMS     : 3;    ///< Master mode selection
        uint8_t res3    : 1;    ///< Reserved
      } reg;
      
    } CR2;


    /// Slave mode control register (TIM6_SMCR) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t SMS     : 3;    ///< Clock/trigger/slave mode selection
        uint8_t res     : 1;    ///< Reserved
        uint8_t TS      : 3;    ///< Trigger selection
        uint8_t MSM     : 1;    ///< Master/slave mode
      } reg;
      
    } SMCR;


    /** TIM6 Interrupt enable (TIM6_IER) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t UIE     : 1;    ///< Update interrupt enable
        uint8_t res     : 7;    ///< Reserved
      } reg;
      
    } IER;


    /** TIM6 Status register (TIM6_SR1) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t UIF     : 1;    ///< Update interrupt flag
        uint8_t res     : 7;    ///< Reserved
      } reg;
      
    } SR1;


    /** TIM6 Event Generation (TIM6_EGR) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t UG      : 1;    ///< Update generation
        uint8_t res     : 7;    ///< Reserved
      } reg;
      
    } EGR;


    /** TIM6 8-bit counter (TIM6_CNTR) */
    byte_t        CNTR;


    /** TIM6 clock prescaler (TIM6_PSCR) */
    union {
      
      /// bytewise access to register
      uint8_t  byte;
      
      /// bitwise access to register
      struct {
        uint8_t PSC     : 3;    ///< clock prescaler
        uint8_t res     : 5;    ///< Reserved
      } reg;
      
    } PSCR;


    /** TIM6 8-bit auto-reload register (TIM6_ARR) */
    byte_t        ARR;

  } TIM6_TypeDef;

  /// pointer to all TIM6 registers (selected devices)
  reg(TIM6_BaseAddress, TIM6_TypeDef, TIM6);
  
  /* TIM6 Module Reset Values */
  #define TIM6_CR1_RESET_VALUE    ((uint8_t)0x00)
  #define TIM6_CR2_RESET_VALUE    ((uint8_t)0x00)
  #define TIM6_SMCR_RESET_VALUE   ((uint8_t)0x00)
  #define TIM6_IER_RESET_VALUE    ((uint8_t)0x00)
  #define TIM6_SR1_RESET_VALUE    ((uint8_t)0x00)
  #define TIM6_EGR_RESET_VALUE    ((uint8_t)0x00)
  #define TIM6_CNTR_RESET_VALUE   ((uint8_t)0x00)
  #define TIM6_PSCR_RESET_VALUE   ((uint8_t)0x00)
  #define TIM6_ARR_RESET_VALUE    ((uint8_t)0xFF)

#endif /* (STM8S903) || (STM8AF622x) */ 


/*-----------------------------------------------------------------------------
    END OF MODULE DEFINITION FOR MULTIPLE INLUSION
-----------------------------------------------------------------------------*/
#endif // _STM8AS_H
