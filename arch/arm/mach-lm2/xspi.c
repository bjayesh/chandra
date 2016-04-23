/*
 * xspi.c - xspi driver register stub
 * Copyright (C) Wind River Systems, Inc.
 *
 */
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/module.h>

#include "mach/irqs.h"
#include "mach/motherboard.h"

static	struct resource	lm2_xspi_resource[] = {
	{
		.start	= LM2_XSPI_0_BASE,
		.end	= LM2_XSPI_0_BASE + 0x100,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= LM2_IRQ_SPI_0,
		.end	= LM2_IRQ_SPI_0,
		.flags	= IORESOURCE_IRQ,
	},
};

static	struct platform_device	lm2_xspi_device = {
	.name	= "xspi",
	.id	= -1,
	.num_resources	= ARRAY_SIZE( lm2_xspi_resource ),
	.resource	= lm2_xspi_resource,
};

static	struct platform_device lm2_spidev_device = {
	.name	= "spidev",
	.id	= -1,
};

int	__init lm2_xspi_register(void)
{
	int	result;

	result = platform_device_register( &lm2_xspi_device );
	result = platform_device_register( &lm2_spidev_device );
	return	result;
}

