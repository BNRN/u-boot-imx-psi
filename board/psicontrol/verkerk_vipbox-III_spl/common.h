/*
 * Copyright (C) 2014, Compulab Ltd - http://compulab.co.il/
 *
 * Author: Nikita Kiryanov <nikita@compulab.co.il>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <asm/arch/mx6-pins.h>
#include <asm/arch/clock.h>

#define UART_PAD_CTRL  (PAD_CTL_PUS_100K_UP |	\
			PAD_CTL_SPEED_LOW | PAD_CTL_DSE_120ohm |	\
			PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define WEAK_PULLUP	(PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS |	\
	PAD_CTL_SRE_SLOW)

#define WEAK_PULLDOWN	(PAD_CTL_PUS_100K_DOWN |		\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_HYS | PAD_CTL_SRE_SLOW)


int board_spi_cs_gpio(unsigned bus, unsigned cs);

int board_mmc_init(bd_t *bis);
int board_mmc_getcd(struct mmc *mmc);

void enable_ddr_led(unsigned number, unsigned on_off);