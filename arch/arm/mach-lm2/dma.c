/*
 * dma.c - dma driver register stub
 * Copyright (C) Wind River Systems, Inc.
 *
 */
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/platform_data/uio_dmem_genirq.h>
#include <linux/module.h>

#include "mach/irqs.h"
#include "mach/motherboard.h"

static	struct resource	lm2_dma_resource[] = {
	{
		.start	= LM2_GPDMA_0_BASE,
		.end	= LM2_GPDMA_0_BASE + 0x100,
		.flags	= IORESOURCE_MEM,
	},
#ifdef CONFIG_UIO_DMEM_GENIRQ
#else
	{
		.start	= 59,
		.end	= 59,
		.flags	= IORESOURCE_IRQ,
	},
#endif
};

static	struct platform_device	lm2_dma_device = {
	.name	= "quatro-gpdma",
	.id	= 0,
	.num_resources	= ARRAY_SIZE( lm2_dma_resource ),
	.resource	= lm2_dma_resource,
};


static	struct resource	lm2_uiodma_resource[] = {
	{
		.start	= LM2_GPDMA_0_BASE,
		.end	= LM2_GPDMA_0_BASE + 0x100,
		.flags	= IORESOURCE_MEM,
	},
};

static unsigned int region_sizes[] = {
	0x4000,
};

static struct uio_dmem_genirq_pdata lm2_platform_data = {
	.uioinfo = {
		.name = "quatro-gpdma",
		.version = "0",
#ifdef CONFIG_UIO_DMEM_GENIRQ
		.irq = 59,
#else
		.irq = -1,
#endif
	},
	.dynamic_region_sizes = region_sizes,
	.num_dynamic_regions = ARRAY_SIZE(region_sizes),
};


static	struct platform_device	lm2_uiodma_device = {
	.name	= "uio_dmem_genirq",
	.id	= 0,
	.dev = {
		.platform_data = &lm2_platform_data,
		},
	.num_resources	= ARRAY_SIZE( lm2_uiodma_resource ),
	.resource	= lm2_uiodma_resource,
};


int	__init lm2_dma_register(void)
{
	int	result;
	
	result = platform_device_register( &lm2_dma_device );
	result = platform_device_register( &lm2_uiodma_device );
	return	result;
}

