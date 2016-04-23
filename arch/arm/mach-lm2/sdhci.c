/*
 * sdhci.c - LM2-Waikiki board USB Interface platform
 * Copyright 2014 Wind River Systems,Inc.
 * j.w.
 */
/*
 * revision:
 * 0.0	initial base on SDHCI driver
 */
/* general include */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/dma-contiguous.h>
#include <linux/platform_device.h>
#include <linux/of_address.h>
#include <linux/of_fdt.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>

#include <mach/irqs.h>
#include <mach/motherboard.h>

/*
 * 
 */

static u64 sdio_dmamask = DMA_BIT_MASK(64);
static struct resource lm2_sdhci0_resources[] = {
	[0] = {
		.start  = LM2_SDIO0_BASE + 0x100,       /* slot0 */
		.end    = LM2_SDIO0_BASE + 0x200,
		.flags  = IORESOURCE_MEM,
	},
	[1] = {
		.start  = LM2_IRQ_SDIO_0_STAT,
		.end    = LM2_IRQ_SDIO_0_STAT,
		.flags  = IORESOURCE_IRQ,
	},
};
static struct platform_device lm2_sdhci0_device = {
	.name           = "sdhci-lm2",
	.id             = 0,
	.dev		= {
				.dma_mask = &sdio_dmamask,
				.coherent_dma_mask = DMA_BIT_MASK(64),
			  },
	.num_resources  = ARRAY_SIZE(lm2_sdhci0_resources),
	.resource       = lm2_sdhci0_resources,
};
static struct resource lm2_sdhci1_resources[] = {
	[0] = {
		.start  = LM2_SDIO1_BASE + 0x200,       /* slot1 */
		.end    = LM2_SDIO1_BASE + 0x300,
		.flags  = IORESOURCE_MEM,
	},
	[1] = {
		.start  = LM2_IRQ_SDIO_1_STAT,
		.end    = LM2_IRQ_SDIO_1_STAT,
		.flags  = IORESOURCE_IRQ,
	},
};
static struct platform_device lm2_sdhci1_device = {
	.name           = "sdhci-lm2",
	.id             = 1,
	.num_resources  = ARRAY_SIZE(lm2_sdhci1_resources),
	.resource       = lm2_sdhci1_resources,
};

void __init lm2_sdhci_init(void)
{
	platform_device_register(&lm2_sdhci0_device);
//	platform_device_register(&lm2_sdhci1_device);
}
