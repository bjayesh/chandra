/*
 * xspi.h - XSPI Kernel driver interface
 *
 * Copyright (C) 2014 Wind River Systems Inc.
 *
 */

#ifndef	XSPI_H_
#define	XSPI_H_

typedef	struct _XSPILIB_PARAM {
	int		unit;
	unsigned char	offset;
	void		*buf;
	unsigned long	size;
}XSPILIB_PARAM, *PXSPILIB_PARAM;

#define	XSPI_UNIT1	0;
#define	XSPI_UNIT2	1;
#define	XSPI_UNIT3	2;

extern	int	xspi_read(XSPILIB_PARAM *spiParam);
extern	int	xspi_write(XSPILIB_PARAM *spiParam);

#endif	/* XSPI_ */
