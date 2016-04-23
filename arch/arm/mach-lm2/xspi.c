/*
 * xspi.c - xspi driver register stub
 * Copyright (C) Wind River Systems, Inc.
 *
 */
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/spi/spi.h>

#include "mach/irqs.h"
#include "mach/motherboard.h"

static	struct spi_board_info	lm2_spi_devices[] __initdata = {
	{
		.modalias	= "spidev",
		.bus_num	= 0,
		.chip_select	= 0,
		.max_speed_hz	= 580000,
		.mode		= SPI_MODE_0,
	},
	{
		.modalias	= "spidev",
		.bus_num	= 0,
		.chip_select	= 1,
		.max_speed_hz	= 580000,
		.mode		= SPI_MODE_0,
	}
};

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
	.name	= "mmio-xspi",
	.id	= 0,
	.num_resources	= ARRAY_SIZE( lm2_xspi_resource ),
	.resource	= lm2_xspi_resource,
};

static	struct platform_device	lm2_spidev_device = {
	.name	= "spidev",
	.id	= -1,
};

int	__init lm2_xspi_register(void)
{
	int	result;

	spi_register_board_info(lm2_spi_devices,ARRAY_SIZE(lm2_spi_devices));
	result = platform_device_register( &lm2_spidev_device );
	result = platform_device_register( &lm2_xspi_device );
	return	result;
}

