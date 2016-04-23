/*
 * xspi.c - xspi driver register stub
 * Copyright (C) Wind River Systems, Inc.
 *
 */
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/mtd/partitions.h>
#include <linux/spi/flash.h>

#include "mach/irqs.h"
#include "mach/motherboard.h"

static	struct	mtd_partition	lm2_flash_parts[] = {
	{
		.name	= "bootloader",
		.offset	= 0x00000000,
		.size	= 0x00600000,
	},
	{	
		.name	="rootfs",
		.offset	= 0x00600000,
		.size	= 0x00100000,
	},
};

static	struct flash_platform_data lm2_flash_data ={
	.name		= "fcspi-flash",
	.parts		= lm2_flash_parts,
	.nr_parts	= ARRAY_SIZE( lm2_flash_parts ),
};

static	struct resource	lm2_fcspi_resource[] = {
	{
		.start	= 0x04110000,
		.end	= 0x04110900,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= 53,
		.end	= 53,
		.flags	= IORESOURCE_IRQ,
	},
};
static	struct	platform_device	lm2_fcspi_device = {
	.name		= "fcspi",
	.id		= 0,
	.resource	= lm2_fcspi_resource,
	.num_resources	= ARRAY_SIZE( lm2_fcspi_resource),
	.dev		= {
		.platform_data = &lm2_flash_data,
	},
};

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
	},
	{
		.modalias	= "spidev",
		.bus_num	= 0,
		.chip_select	= 2,
		.max_speed_hz	= 580000,
		.mode		= SPI_MODE_0,
	},
};

static	struct resource	lm2_xspi_resource[] = {
	{
		.start	= LM2_XSPI_0_BASE,
		.end	= LM2_XSPI_0_BASE + 0x100,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= LM2_IRQ_XSPI_0_CMP,
		.end	= LM2_IRQ_XSPI_0_CMP,
		.flags	= IORESOURCE_IRQ,
	},
};

static	struct platform_device	lm2_xspi_device = {
	.name	= "mmio-xspi",
	.id	= 0,
	.num_resources	= ARRAY_SIZE( lm2_xspi_resource ),
	.resource	= lm2_xspi_resource,
};
#if 0
static	struct platform_device	lm2_spidev_device = {
	.name	= "spidev",
	.id	= -1,
};
#endif
int	__init lm2_xspi_register(void)
{
	int	result;

	spi_register_board_info(lm2_spi_devices,ARRAY_SIZE(lm2_spi_devices));
//	result = platform_device_register( &lm2_spidev_device );
	result = platform_device_register( &lm2_xspi_device );
	result = platform_device_register( &lm2_fcspi_device );
	return	result;
}

