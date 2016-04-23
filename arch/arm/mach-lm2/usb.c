/*
 * usb.c - LM2-Waikiki board USB Interface platform
 * Copyright 2014 Wind River Systems,Inc.
 * j.w.
 */
/*
 * revision:
 * 0.0	initial base on EHCI driver
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
#include <linux/usb/phy.h>

#include <mach/irqs.h>
#include <mach/motherboard.h>

/*
 * resource 1 : USB Host
 * resource 2 : USB Device
 */
static	struct resource lm2_usb2_phy_resource[] ={
	{
		.start	= LM2_USB2_PHY,
		.end	= LM2_USB2_PHY + 0x500,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= LM2_USB3_PHY,
		.end	= LM2_USB3_PHY + 0x500,
		.flags	= IORESOURCE_MEM,
	},
};

static	struct platform_device lm2_usb2_phy_device = {
	.name	= "keystone-usbphy",
	.id	= -1,
	.num_resources	= ARRAY_SIZE(lm2_usb2_phy_resource),
	.resource	= lm2_usb2_phy_resource,
};

static	struct resource	lm2_usb2_resource[] = {
	{
		.name	= "dwc3",
		.start	= LM2_USB2,
		.end	= LM2_USB2 + 0x10000,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "host irq",
		.start	= LM2_IRQ_USB_HOST,
		.end	= LM2_IRQ_USB_HOST,
		.flags	= IORESOURCE_IRQ,
	},
};

static  u64     lm2_usb_dmamask = DMA_BIT_MASK(64);

static	struct platform_device lm2_usb_host_device = {
	.name	= "dwc3",
	.id	= -1,
	.num_resources	= ARRAY_SIZE(lm2_usb2_resource),
	.resource	= lm2_usb2_resource,
	.dev	= {
                .dma_mask       = &lm2_usb_dmamask,
                .coherent_dma_mask      = DMA_BIT_MASK(64),
	},
};


int	__init	lm2_usb_register(void)
{
	int	result;
#if 0
	usb_bind_phy("dwc3",0,"usb-phy");
#endif
	result = platform_device_register(&lm2_usb2_phy_device);
	result = platform_device_register(&lm2_usb_host_device);
#if 0
	if(result != 0)
		result = platform_device_register(&lm2_usb_devs_device);
#endif
	return	result;
}
