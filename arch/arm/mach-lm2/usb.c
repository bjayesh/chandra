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
#include <linux/printk.h>
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

#include <linux/usb/ch9.h>
#include <linux/usb/otg.h>
/*
 * resource 1 : USB Host
 * resource 2 : USB Device
 */
static	struct resource lm2_usbh_phy_resource[] ={
	{
		.start	= LM2_USB2_PHY,
		.end	= LM2_USB2_PHY + 0x500,
		.flags	= IORESOURCE_MEM,
	},
};

static	struct platform_device lm2_usbh_phy_device = {
	.name	= "usb-phy",
	.id	= -1,
	.num_resources	= ARRAY_SIZE(lm2_usbh_phy_resource),
	.resource	= lm2_usbh_phy_resource,
};

static	struct resource lm2_usbd_phy_resource[] ={
	{
		.start	= LM2_USB3_PHY,
		.end	= LM2_USB3_PHY + 0x500,
		.flags	= IORESOURCE_MEM,
	},
};

static	struct platform_device lm2_usbd_phy_device = {
	.name	= "usb-phy",
	.id	= -1,
	.num_resources	= ARRAY_SIZE(lm2_usbd_phy_resource),
	.resource	= lm2_usbd_phy_resource,
};


static	struct resource	lm2_usbh_resource[] = {
	{
		.start	= LM2_USB2,
		.end	= LM2_USB2 + 0x10000,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= LM2_IRQ_USB_HOST,
		.end	= LM2_IRQ_USB_HOST,
		.flags	= IORESOURCE_IRQ,
	},
};

static  u64     lm2_usbh_dmamask = DMA_BIT_MASK(64);

static	struct platform_device lm2_usb_host_device = {
	.name	= "xhci-hcd",
	.id	= -1,
	.num_resources	= ARRAY_SIZE(lm2_usbh_resource),
	.resource	= lm2_usbh_resource,
	.dev	= {
                .dma_mask       = &lm2_usbh_dmamask,
                .coherent_dma_mask      = DMA_BIT_MASK(64),
	},
};

static	struct resource	lm2_usbd_resource[] = {
	{
		.start	= LM2_USB3,
		.end	= LM2_USB3 + 0x10000,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= LM2_IRQ_USB3_DEV,
		.end	= LM2_IRQ_USB3_DEV,
		.flags	= IORESOURCE_IRQ,
	},
};

struct dwc3_platform_data {
        enum usb_device_speed maximum_speed;
        enum usb_dr_mode dr_mode;
        bool tx_fifo_resize;
};

static	struct dwc3_platform_data	dwc3_plat_data = {
	.maximum_speed	= USB_SPEED_SUPER,
	.dr_mode	= USB_DR_MODE_PERIPHERAL,
//	.tx_fifo_resize	= 1,	/* original */
	.tx_fifo_resize	= 0,	/* original */
};

static  u64     lm2_usbd_dmamask = DMA_BIT_MASK(64);

static	struct platform_device lm2_usb_devs_device = {
	.name	= "dwc3",
	.id	= -1,
	.num_resources	= ARRAY_SIZE(lm2_usbd_resource),
	.resource	= lm2_usbd_resource,
	.dev	= {
                .dma_mask       = &lm2_usbd_dmamask,
                .coherent_dma_mask      = DMA_BIT_MASK(64),
		.platform_data = &dwc3_plat_data,
	},
};

int	__init	lm2_usb_register(void)
{
	int	result;
	void __iomem	*ptr;

	ptr= ioremap(LM2_USB2_PHY,0x500);
	printk(KERN_WARNING "USB Register Device %x PHY address",ptr);

	result = platform_device_register(&lm2_usb_host_device);
	result = platform_device_register(&lm2_usb_devs_device);
	return	result;
}
