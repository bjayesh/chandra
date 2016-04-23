/*
 * sata.c	-	LM2-Waikiki board SATA Interface platform
 *
 * Copyright Wind River Systems, Inc.
 *
 */
/*
 * revison:
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/ahci_platform.h>
#include <linux/dma-mapping.h>
#include <linux/dma-contiguous.h>

#include <asm/io.h>

#include <mach/irqs.h>
#include <mach/motherboard.h>
#include <mach/hardware.h>

#define	LM2_SATA_BASE		0x04a30000
#define	LM2_SATA_GPR_BASE	0x04a72000

/* 
 * Local functions
 */
static	int	lm2_sata_initial(struct device *dev, void __iomem *addr)
{
	struct	lm2_sata3_gpr	*gpr;
	void __iomem	*vptr;


	vptr = ioremap(LM2_SATA_GPR_BASE,0x200);
	gpr = (struct lm2_sata3_gpr *)vptr;
#if 0
	dev_info(dev,"BSP Initial function GPR = %x\n",gpr);
	dev_info(dev,"tx_preemph_gen1 = %x\n",gpr->tx_preemph_gen1);
#endif
	/* ToDo:adjustment for board parameters */

	return	0;
}

/*
 * Platform parameters
 */
static	struct resource	lm2_sata_resource[] = {
	{
		.start	= LM2_SATA_BASE,
		.end	= LM2_SATA_BASE + 0x200,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= LM2_IRQ_SATA_CTRL,
		.flags	= IORESOURCE_IRQ,
	},
};

static	u64	lm2_sata_dmamask = DMA_BIT_MASK(64);

static	struct ahci_platform_data	lm2_sata_pdata = {
	.init	= lm2_sata_initial,
};

static	struct platform_device lm2_sata_device = {
	.name	= "ahci",
	.id	= -1,
	.dev	= {
		.platform_data	= &lm2_sata_pdata,
		.dma_mask	= &lm2_sata_dmamask,
		.coherent_dma_mask	= DMA_BIT_MASK(64),
	},
	.num_resources	= ARRAY_SIZE(lm2_sata_resource),
	.resource	= lm2_sata_resource,
};


int	__init	lm2_sata_register(void)
{
	return	platform_device_register(&lm2_sata_device);
}
