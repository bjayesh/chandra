/*
 *  arch/arm/mach-lm2/include/mach/uncompress.h
 *
 *  Copyright (c) 2014 Wind River Systems, Inc
 *  Copyright (C) 2003 ARM Limited
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#define AMBA_UART_DR(base)	(*(volatile unsigned char *)((base) + 0x10))
#define AMBA_UART_LSR(base)	(*(volatile unsigned char *)((base) + 0x15))

#define UART_BASE	0x10009000
#define	UART_BASE_0	0x040b0000
#define	UART_BASE_1	0x04160000
#define	UART_BASE_2	0x052c0000
#define	UART_BASE_3	0x052d0000
#define	UART_BASE_4	0x052e0000
#define	UART_BASE_5	0x052f0000

#define UART_BASE_RS1	0x1c090000

static unsigned long get_uart_base(void)
{
	return	UART_BASE_1;
}

/*
 * This does not append a newline
 */
static inline void putc(int c)
{
	unsigned long base = get_uart_base();

	while ((AMBA_UART_LSR(base) & 0x40) == 0)
		barrier();

	AMBA_UART_DR(base) = c;
}

static inline void flush(void)
{
	unsigned long base = get_uart_base();

	while ((AMBA_UART_LSR(base) & 0x40) == 0)
		barrier();
}

static	inline	int getc(void)
{
	unsigned long base = get_uart_base();

	while(( AMBA_UART_LSR(base) & 0x01) == 0)
		barrier();
	return	AMBA_UART_DR(base);
}
/*
 * nothing to do
 */
#define arch_decomp_setup()
#define arch_decomp_wdog()
