/*
 * Pin# | Function          | board pin/signal
 * 1    | PD4               | Si4463 p1 - SDN
 * 2    | PD5/UART1_TX      | header TXD pad via level shifter
 * 3    | PD6/UART1_RX      | header RXD pad via level shifter
 * 4    | nRST              | reset test point nearest header RXD pad
 * 11   | PB5               | header SET pad via level shifter
 * 12   | PB4               | Si4463 p9  - GPIO0
 * 13   | PC3               | Si4463 p10 - GPIO1
 * 14   | PC4               | Si4463 p11 - nIRQ
 * 15   | PC5/SPI_SCK       | Si4463 p12 - SCLK
 * 16   | PC6/SPI_MOSI      | Si4463 p14 - SDI
 * 17   | PC7/SPI_MISO      | Si4463 p13 - SDO
 * 18   | PD1/SWIM          | SWIM test point nearest header TXD pad
 * 19   | PD2               | Si4463 p15 - nSEL
 */

#include <stdint.h>
#include <stdio.h>
#define _MAIN_
#include "stm8as.h"
#include "uart1.h"
#include "gpio.h"

#undef _MAIN_

#include "si446x.h"
#include "ax25.h"

inline void setup_platform(void) {
	DISABLE_INTERRUPTS;

	CLK.CKDIVR.byte = 0x00;  
    gpio_init(&PORT_D, PIN_2, OUTPUT_PUSHPULL_FAST); /* nSEL */
	gpio_init(&PORT_D, PIN_4, OUTPUT_PUSHPULL_FAST); /* SDN */
	GPIO_SET(PORT_D, PIN_4, 0); /* SDN low by default */

	gpio_init(&PORT_B, PIN_4, INPUT_PULLUP_NOEXINT); /* GPIO0 input */
	gpio_init(&PORT_C, PIN_3, INPUT_PULLUP_NOEXINT); /* GPIO1 input */

	gpio_init(&PORT_D, PIN_5, OUTPUT_PUSHPULL_FAST); /* UART1 TxD */
	gpio_init(&PORT_D, PIN_6, INPUT_PULLUP_NOEXINT); /* UART1 RxD */

	
	uart1_init(115200L);
	si446x_init();	
	ENABLE_INTERRUPTS;
}

void main (void) {

	setup_platform();
	printf("Hello!\n");
	si446x_dummy();
	while(1);

}
