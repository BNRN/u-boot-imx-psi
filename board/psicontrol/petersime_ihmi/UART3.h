/*
 * (c) 2007 Sascha Hauer <s.hauer@pengutronix.de>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>

void uart3_setbrg(void);

int uart3_getc(void);

void uart3_putc(const char c);

/*
 * Test whether a character is in the RX buffer
 */
int uart3_tstc(void);

/*
 * Initialise the serial port with the given baudrate. The settings
 * are always 8 data bits, no parity, 1 stop bit, no start bits.
 *
 */
int uart3_init(void);
