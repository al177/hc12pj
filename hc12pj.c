#include <stdint.h>
#define _MAIN_
#include "stm8as.h"
#include "uart1.h"
#include "gpio.h"
#include "spi.h"

#undef _MAIN_

#include "si446x.h"

inline void setup_platform(void) {
	DISABLE_INTERRUPTS;

	CLK.CKDIVR.byte = 0x00;  
	spi_config(4, 0, 0);
	uart1_init(115200L);
	ENABLE_INTERRUPTS;
}

void main (void) {

	setup_platform();
	si446x_init();	

	while(1);

}
