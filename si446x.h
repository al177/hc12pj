#ifndef __SI446X_H_
#define __SI446X_H_

#include "radio_config_Si4463.h"

void si446x_init();
void si446x_dummy();
void si446x_spi_frame_start(void);
void si446x_spi_frame_end(void);
#endif /*__SI446X_H_*/
