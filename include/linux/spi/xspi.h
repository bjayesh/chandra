/*
 * xspi.h - XSPI Kernel driver interface
 *
 * Copyright (C) 2014 Wind River Systems Inc.
 *
 */
/*
 * change type 
 */
#ifndef	XSPI_H_
#define	XSPI_H_

typedef	struct _SPILIB_PARAM {
	unsigned int	unit;	/* Unit # */
	unsigned int	offset;	/* write/read start address */
	void		*buf;	/* write/read buffer address */
	size_t		size;	/* write/read byte size */ 
}SPILIB_PARAM, *PSPILIB_PARAM;

#define	SPI_UNIT1	0	/* CPLD */
#define	SPI_UNIT2	1	/* SRAM	*/
#define	SPI_UNIT3	2	/* MRAM/FRAM */

extern	int	xspi_read(SPILIB_PARAM *spiParam);
extern	int	xspi_write(SPILIB_PARAM *spiParam);

#endif	/* XSPI_ */
