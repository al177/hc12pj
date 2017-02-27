#include "si446x.h"
#include <stdint.h>
#include <stdio.h>
#include "gpio.h"

inline void si446x_busy_cs(void) {
	uint32_t f=100000;
	while(f>0) {
		f--;
	}
}
	
void si446x_spi_xfer(uint8_t len, char* buf) {

	uint8_t i;
	/* assert nSEL */
	GPIO_SET(PORT_D, PIN_2, 0);
	si446x_busy_cs();
	
	// disable interrupts to prevent timing issue
	DISABLE_INTERRUPTS;

	// clear MISO buffer
	i = SPI.DR.byte;
	i = 0;
	// loop over numTx bytes
	while (i < len) {

		SPI.DR.byte = buf[i];
		
		while(!SPI.SR.reg.TXE) {
			_NOP_;
		}
		while(!SPI.SR.reg.RXNE) {
			_NOP_;
		}
		buf[i] = SPI.DR.byte;

		i++;
	} // send/receive loop

	// re-enable interrupts
	ENABLE_INTERRUPTS;

	// wait until not busy
	while (SPI.SR.reg.BSY);
	
	/* deassert nSEL */
	GPIO_SET(PORT_D, PIN_2, 1);
	si446x_busy_cs();

}


void si446x_init(void) {
	printf("si446x_init()\n");


	/* dirty SPI init */
	SPI.CR1.byte=0x14; /* SPI off, master mode, CLK/8, CPOL=CPHA=0, MSB first */
	SPI.CR2.byte=0x01; /* no NSS, full duplex, no CRC */
	SPI.CR1.byte=0x54; /* enable SPI */
	
	/* set up SPI GPIOs */
	gpio_init(&PORT_C, PIN_5, OUTPUT_PUSHPULL_FAST); /* SCK */
	gpio_init(&PORT_C, PIN_6, OUTPUT_PUSHPULL_FAST); /* MOSI */
	gpio_init(&PORT_C, PIN_7, INPUT_PULLUP_NOEXINT); /* MISO */

	GPIO_SET(PORT_D, PIN_2, 1);

	/* toggle shutdown to reset Si446x */
	GPIO_SET(PORT_D, PIN_4, 1);
	si446x_busy_cs();
	GPIO_SET(PORT_D, PIN_4, 0);
	si446x_busy_cs();
}

void si446x_dummy(void) {
	char buf[64];
	int f;
	buf[0]=0x1;
	si446x_spi_xfer(1, buf);
	
	buf[0]=0x44;
	si446x_spi_xfer(10, buf);
	printf("\n");
	for(f=0;f<10;f++) {
		printf("%02x ", buf[f]);
	}
	printf("\n");
	return;
}
