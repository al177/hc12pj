/**
  \file error_codes.h
   
  \author G. Icking-Konert
  \date 2013-11-22
  \version 0.1
   
  \brief declaration of error codes
   
  declaration of codes for error tracking
*/

/*-----------------------------------------------------------------------------
    MODULE DEFINITION FOR MULTIPLE INCLUSION
-----------------------------------------------------------------------------*/
#ifndef _ERROR_CODES_H_
#define _ERROR_CODES_H_


/*-----------------------------------------------------------------------------
    DEFINITION OF GLOBAL MACROS/#DEFINES
-----------------------------------------------------------------------------*/

// error codes
#define SUCCESS           0       // no error
#define ERROR_TIMOUT      1       // timeout
#define ERROR_COMM        2       // communication error
#define ERROR_RANGE       3       // out of range error
#define ERROR_ILLPRM      5       // illegal parameter error
#define ERROR_CHK         6       // checksum error
#define ERROR_FLASH       7       // flash error (e.g. during write)
#define ERROR_MISC      255       // misc error and muCom indicator for error (precedes actual error code)


/*-----------------------------------------------------------------------------
    END OF MODULE DEFINITION FOR MULTIPLE INLUSION
-----------------------------------------------------------------------------*/
#endif // _ERROR_CODES_H_
