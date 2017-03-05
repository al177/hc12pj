#include "si446x.h"
#include <stdint.h>
#include <stdio.h>
#include "gpio.h"


const uint8_t config_array[] = RADIO_CONFIGURATION_DATA_ARRAY;

void si446x_busy_cs(void) {
	uint32_t f=5000;
	while(f>0) {
		f--;
	}
}

void si446x_spi_frame_start(void) {
		register uint8_t i;
		/* assert nSEL */
		GPIO_SET(PORT_D, PIN_2, 0);
		si446x_busy_cs();

		// disable interrupts to prevent timing issue
		DISABLE_INTERRUPTS;

		// clear MISO buffer
		i = SPI.DR.byte;
}

void si446x_spi_frame_end(void) {
		// re-enable interrupts
		ENABLE_INTERRUPTS;

		// wait until not busy
		while (SPI.SR.reg.BSY);

		/* deassert nSEL */
		GPIO_SET(PORT_D, PIN_2, 1);
		si446x_busy_cs();
}

uint8_t si446x_spi_shift_byte(uint8_t tx) {
			while(!SPI.SR.reg.TXE) {
				_NOP_;
			}

			SPI.DR.byte = tx;

			while(!SPI.SR.reg.RXNE) {
				_NOP_;
			}
			return SPI.DR.byte;
}

uint8_t si446x_spi_cmd_resp(char* buf, uint8_t size, uint16_t timeout) {

	uint8_t i;

	while(timeout > 0) {
		si446x_spi_frame_start();
		i = 0;

		// loop over numTx bytes
		while (i < (size + 2)) {

			if(i == 0) {
				buf[0] = si446x_spi_shift_byte(0x44);
			} else if (i == 1) {
				if(si446x_spi_shift_byte(0xFF) != 0xFF) {
					timeout--;
					break;
				}
			} else {
				buf[i-2] = si446x_spi_shift_byte(0xFF);
			}
			i++;
		} // send/receive loop

		si446x_spi_frame_end();
		if (i > 1) {
			return 1; /* data was read */
		}
	}
	return 0; /* timed out waiting for CTS */
}

void si446x_spi_cmd_send(char* buf, uint8_t size) {

	uint8_t i;
	uint8_t dummy;
	si446x_spi_frame_start();
	i = 0;
	// loop over numTx bytes
	while (i < size) {

		si446x_spi_shift_byte(buf[i]);

		i++;
	} // send/receive loop

	si446x_spi_frame_end();
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

/* stolen from https://github.com/alexander-sholohov/si4463-beacon.git */
uint8_t si446x_play_cmds(const uint8_t *cmds) {
	uint8_t buf[16];
	uint8_t size, bufidx;

	while(*cmds != 0) {
		size = *cmds;
		if( size > 16) {
			return 0;
		}
		cmds++;
		for(bufidx=0; bufidx<size; bufidx++) {
			buf[bufidx]=cmds[bufidx];
		}
		si446x_spi_cmd_send(buf, size);
	}
	return 1;	
}

void si446x_dummy(void) {
	char buf[64];
	int f;
	si446x_play_cmds(config_array);
	buf[0]=0x1;
	si446x_spi_cmd_send(buf, 10);

	si446x_spi_cmd_resp(buf,10, 100);
	for(f=0;f<10;f++) {
		printf("%02x ", buf[f]);
	}
	buf[0]=0x22;
	buf[1]=0;
	si446x_spi_cmd_send(buf, 10);
	si446x_spi_cmd_resp(buf,10, 100);
	for(f=0;f<10;f++) {
		printf("%02x ", buf[f]);
	}
	printf("\n");
	return;
}

