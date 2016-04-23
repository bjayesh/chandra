/*
 * xhci-plat.c - xHCI host controller driver platform Bus Glue.
 *
 * Copyright (C) 2012 Texas Instruments Incorporated - http://www.ti.com
 * Author: Sebastian Andrzej Siewior <bigeasy@linutronix.de>
 *
 * A lot of code borrowed from the Linux xHCI driver.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 */

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/slab.h>

#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/proc_fs.h>
#include <linux/fcntl.h>
#include <linux/seq_file.h>
#include <linux/cdev.h>

#include "xhci.h"

#include <linux/fxmodule/xchi_ioctl.h>

static int xchi_cdev_open(struct inode *inode, struct file *filp);
static int xchi_cdev_release(struct inode *inode, struct file *filp);
static long xchi_cdev_ioctl(struct file *filp,unsigned int cmd, unsigned long arg);
static int xhci_plat_create_dev(struct platform_device *pdev);


static void xhci_plat_quirks(struct device *dev, struct xhci_hcd *xhci)
{
	/*
	 * As of now platform drivers don't provide MSI support so we ensure
	 * here that the generic code does not try to make a pci_dev from our
	 * dev struct in order to setup MSI
	 */
	xhci->quirks |= XHCI_PLAT;
}

/* called during probe() after chip reset completes */
static int xhci_plat_setup(struct usb_hcd *hcd)
{
	return xhci_gen_setup(hcd, xhci_plat_quirks);
}

static const struct hc_driver xhci_plat_xhci_driver = {
	.description =		"xhci-hcd",
	.product_desc =		"xHCI Host Controller",
	.hcd_priv_size =	sizeof(struct xhci_hcd *),

	/*
	 * generic hardware linkage
	 */
	.irq =			xhci_irq,
	.flags =		HCD_MEMORY | HCD_USB3 | HCD_SHARED,

	/*
	 * basic lifecycle operations
	 */
	.reset =		xhci_plat_setup,
	.start =		xhci_run,
	.stop =			xhci_stop,
	.shutdown =		xhci_shutdown,

	/*
	 * managing i/o requests and associated device resources
	 */
	.urb_enqueue =		xhci_urb_enqueue,
	.urb_dequeue =		xhci_urb_dequeue,
	.alloc_dev =		xhci_alloc_dev,
	.free_dev =		xhci_free_dev,
	.alloc_streams =	xhci_alloc_streams,
	.free_streams =		xhci_free_streams,
	.add_endpoint =		xhci_add_endpoint,
	.drop_endpoint =	xhci_drop_endpoint,
	.endpoint_reset =	xhci_endpoint_reset,
	.check_bandwidth =	xhci_check_bandwidth,
	.reset_bandwidth =	xhci_reset_bandwidth,
	.address_device =	xhci_address_device,
	.update_hub_device =	xhci_update_hub_device,
	.reset_device =		xhci_discover_or_reset_device,

	/*
	 * scheduling support
	 */
	.get_frame_number =	xhci_get_frame,

	/* Root hub support */
	.hub_control =		xhci_hub_control,
	.hub_status_data =	xhci_hub_status_data,
	.bus_suspend =		xhci_bus_suspend,
	.bus_resume =		xhci_bus_resume,
};

struct xchi_cdevinfo{
	struct platform_device *pdev;
	struct device*			dev;	
	struct cdev				cdev;
	struct class*			pclass;				/* the class for this device */
	dev_t devno;

};



struct file_operations xhci_cdev_fops = {
	.owner =    THIS_MODULE,
	.unlocked_ioctl =    xchi_cdev_ioctl,
	.open =     xchi_cdev_open,
	.release =  xchi_cdev_release,
};

static struct xchi_cdevinfo g_xchi_cdev;
static int xhci_plat_create_dev(struct platform_device *pdev)
{
	int status;
	struct cdev* p_cdev;

	memset(&g_xchi_cdev,0,sizeof(g_xchi_cdev));

	g_xchi_cdev.pclass=class_create(THIS_MODULE, "xhci_cdev");
	if(IS_ERR(g_xchi_cdev.pclass)){
		status = PTR_ERR(g_xchi_cdev.pclass);
		pr_err("unable to create xhcid class %d\n", status);
		return status;
	}
	status = alloc_chrdev_region(&g_xchi_cdev.devno, 0, 1,"xchi cdev");
	if (status) {
		pr_err("alloc_chrdev_region %d\n", status);
		class_destroy(g_xchi_cdev.pclass);
		return status;
	}
	p_cdev=&g_xchi_cdev.cdev;

	cdev_init(p_cdev, &xhci_cdev_fops);
	p_cdev->owner = THIS_MODULE;

	status = cdev_add(p_cdev, g_xchi_cdev.devno, 1);
	if (status) {
		pr_err("Failed to open char device\n");
		class_destroy(g_xchi_cdev.pclass);
		return status;
	}

	g_xchi_cdev.dev = device_create(g_xchi_cdev.pclass, NULL, g_xchi_cdev.devno,NULL, XHCI_CDEV_DEVICE);
	if (IS_ERR(g_xchi_cdev.dev)) {
		pr_err("Failed to open char device\n");
		class_destroy(g_xchi_cdev.pclass);
		return(-ENODEV);
	}

	g_xchi_cdev.pdev=pdev;
	return 0;
}

static int xhci_plat_probe(struct platform_device *pdev)
{
	const struct hc_driver	*driver;
	struct xhci_hcd		*xhci;
	struct resource         *res;
	struct usb_hcd		*hcd;
	int			ret;
	int			irq;

	if (usb_disabled())
		return -ENODEV;

	driver = &xhci_plat_xhci_driver;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return -ENODEV;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	hcd = usb_create_hcd(driver, &pdev->dev, dev_name(&pdev->dev));
	if (!hcd)
		return -ENOMEM;

	hcd->rsrc_start = res->start;
	hcd->rsrc_len = resource_size(res);

	if (!request_mem_region(hcd->rsrc_start, hcd->rsrc_len,
				driver->description)) {
		dev_dbg(&pdev->dev, "controller already in use\n");
		ret = -EBUSY;
		goto put_hcd;
	}

	hcd->regs = ioremap_nocache(hcd->rsrc_start, hcd->rsrc_len);
	if (!hcd->regs) {
		dev_dbg(&pdev->dev, "error mapping memory\n");
		ret = -EFAULT;
		goto release_mem_region;
	}

//dev_info(&pdev->dev, "usb_add_hcd reg virt = %x\n",hcd->regs);
//dev_info(&pdev->dev, "usb_add_hcd\n");
	ret = usb_add_hcd(hcd, irq, IRQF_SHARED);
	if (ret)
		goto unmap_registers;

	/* USB 2.0 roothub is stored in the platform_device now. */
	hcd = dev_get_drvdata(&pdev->dev);
	xhci = hcd_to_xhci(hcd);
	xhci->shared_hcd = usb_create_shared_hcd(driver, &pdev->dev,
			dev_name(&pdev->dev), hcd);
	if (!xhci->shared_hcd) {
		ret = -ENOMEM;
		goto dealloc_usb2_hcd;
	}

	/*
	 * Set the xHCI pointer before xhci_plat_setup() (aka hcd_driver.reset)
	 * is called by usb_add_hcd().
	 */
	*((struct xhci_hcd **) xhci->shared_hcd->hcd_priv) = xhci;

// dev_info(&pdev->dev, "usb_add_hcd\n");
	ret = usb_add_hcd(xhci->shared_hcd, irq, IRQF_SHARED);
	if (ret)
		goto put_usb3_hcd;


	xhci_plat_create_dev(pdev);

//dev_info(&pdev->dev, "==================== USB Normal End =========================\n");
	return 0;

put_usb3_hcd:
	usb_put_hcd(xhci->shared_hcd);

dealloc_usb2_hcd:
	usb_remove_hcd(hcd);

unmap_registers:
	iounmap(hcd->regs);

release_mem_region:
	release_mem_region(hcd->rsrc_start, hcd->rsrc_len);

put_hcd:
	usb_put_hcd(hcd);

	return ret;
}

static int xhci_plat_remove(struct platform_device *dev)
{
	struct usb_hcd	*hcd = platform_get_drvdata(dev);
	struct xhci_hcd	*xhci = hcd_to_xhci(hcd);

	if(g_xchi_cdev.pclass){
		unregister_chrdev_region(g_xchi_cdev.devno, 1);
		class_destroy(g_xchi_cdev.pclass);
	}

	usb_remove_hcd(xhci->shared_hcd);
	usb_put_hcd(xhci->shared_hcd);

	usb_remove_hcd(hcd);
	iounmap(hcd->regs);
	release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
	usb_put_hcd(hcd);
	kfree(xhci);

	return 0;
}

#ifdef	CONFIG_ARCH_LM2
#define LM2_REGBAK_SIZE 160
static unsigned int     reg_bak[LM2_REGBAK_SIZE];
static unsigned int     reg_bak_chksum;
extern unsigned int     chksum_info;

void xhci_reg_save(void __iomem *base, int *bak_adr, int offset, int size)
{
        int i;
        int adr = *bak_adr;

        for(i=adr; i<(adr+size); i++ ) {
                reg_bak[i] = readl(base + offset);
                offset +=4;
        }
        *bak_adr = i;
}

void xhci_reg_load(void __iomem *base, int *bak_adr, int offset, int size)
{
        int i;
        int adr = *bak_adr;

        for(i=adr; i<(adr+size); i++ ) {
                writel( reg_bak[i], base + offset);
                wmb();
                offset +=4;
        }
        *bak_adr = i;
}

static int xhci_host_suspend(struct platform_device *pdev)
{
	struct usb_hcd  *hcd = platform_get_drvdata(pdev);
	struct xhci_hcd *xhci = hcd_to_xhci(hcd);
	int i=0;
	void __iomem *base;
	base = ioremap_nocache(0x04600000, 0x500);
	xhci_reg_save(base, &i, 0x020, 15);
	xhci_reg_save(base, &i, 0x420,  4);
	iounmap(base);

	base = ioremap_nocache(0x0460c000, 0x700);
	xhci_reg_save(base, &i, 0x100, 36);
	xhci_reg_save(base, &i, 0x200,  2);
	xhci_reg_save(base, &i, 0x2c0,  1);
	xhci_reg_save(base, &i, 0x300,  5);
	xhci_reg_save(base, &i, 0x380,  4);
	xhci_reg_save(base, &i, 0x400,  4);
	xhci_reg_save(base, &i, 0x618,  4);
	xhci_reg_save(base, &i, 0x630,  1);
	iounmap(base);

	/* usb2.0 phy */
	base = ioremap_nocache(0x04408000, 0x500);
	xhci_reg_save(base, &i, 0x000, 17);
	xhci_reg_save(base, &i, 0x100,  8);
	xhci_reg_save(base, &i, 0x130, 15);
	xhci_reg_save(base, &i, 0x170,  7);
	xhci_reg_save(base, &i, 0x1a0, 15);
	xhci_reg_save(base, &i, 0x200,  5);
	xhci_reg_save(base, &i, 0x280, 10);
	xhci_reg_save(base, &i, 0x400,  1);
	iounmap(base);

        /* chksum gen */
        reg_bak_chksum=0;
        for(i=0; i<LM2_REGBAK_SIZE; i++)
                reg_bak_chksum += reg_bak[i];

	i = xhci_suspend(xhci);
        return i;
}

static int xhci_host_resume(struct platform_device *pdev)
{
	struct usb_hcd  *hcd = platform_get_drvdata(pdev);
	struct xhci_hcd *xhci = hcd_to_xhci(hcd);
	int i;
	void __iomem *base;
	unsigned int    tmp;
        /* chksum chk */
        tmp=0;
        for(i=0; i<LM2_REGBAK_SIZE; i++)
                tmp += reg_bak[i];
        if ( tmp != reg_bak_chksum ){
                chksum_info |= 0x100;
        }
	i=0;
	base = ioremap_nocache(0x04600000, 0x500);
	xhci_reg_load(base, &i, 0x020, 15);
	xhci_reg_load(base, &i, 0x420,  4);
	iounmap(base);

	base = ioremap_nocache(0x0460c000, 0x700);
	xhci_reg_load(base, &i, 0x100, 36);
	xhci_reg_load(base, &i, 0x200,  2);
	xhci_reg_load(base, &i, 0x2c0,  1);
	xhci_reg_load(base, &i, 0x300,  5);
	xhci_reg_load(base, &i, 0x380,  4);
	xhci_reg_load(base, &i, 0x400,  4);
	xhci_reg_load(base, &i, 0x618,  4);
	xhci_reg_load(base, &i, 0x630,  1);
	iounmap(base);

	/* usb2.0 phy */
	base = ioremap_nocache(0x04408000, 0x500);
	xhci_reg_load(base, &i, 0x000, 17);
	xhci_reg_load(base, &i, 0x100,  8);
	xhci_reg_load(base, &i, 0x130, 15);
	xhci_reg_load(base, &i, 0x170,  7);
	xhci_reg_load(base, &i, 0x1a0, 15);
	xhci_reg_load(base, &i, 0x200,  5);
	xhci_reg_load(base, &i, 0x280, 10);
	xhci_reg_load(base, &i, 0x400,  1);
	iounmap(base);

	i = xhci_resume(xhci, false);
        return i;
}

static int xchi_cdev_open(struct inode *inode, struct file *filp)
{
	struct xchi_cdevinfo* p_cdevinfo;
	p_cdevinfo = container_of(inode->i_cdev, struct xchi_cdevinfo, cdev);
	filp->private_data = p_cdevinfo;
	return 0;
}

static int xchi_cdev_release(struct inode *inode, struct file *filp)
{
	return 0;
}

static long xchi_cdev_ioctl(struct file *filp,unsigned int cmd, unsigned long arg)
{
	struct xchi_cdevinfo* p_cdevinfo;
	struct platform_device *pdev;
	struct usb_hcd *hcd;
	int retval = 0;

	p_cdevinfo=(struct xchi_cdevinfo*)(filp->private_data);
	pdev=p_cdevinfo->pdev;
	hcd = platform_get_drvdata(pdev);

	switch(cmd) {
		case XHCI_SLEEPING:
			//turn off port power of roothub
			xhci_hub_control(hcd, ClearPortFeature,USB_PORT_FEAT_POWER,1, 0, 0);
			xhci_hub_control(hcd, ClearPortFeature,USB_PORT_FEAT_POWER,2, 0, 0);
			break;

		case XHCI_WAKEUP:
			xhci_hub_control(hcd, SetPortFeature,USB_PORT_FEAT_POWER,1, 0, 0);
			xhci_hub_control(hcd, SetPortFeature,USB_PORT_FEAT_POWER,2, 0, 0);
			break;
		default:
			return -EINVAL;
	}
	return retval;
}


#endif	/* CONFIG_ARCH_LM2 */

static struct platform_driver usb_xhci_driver = {
	.probe	= xhci_plat_probe,
	.remove	= xhci_plat_remove,
	.driver	= {
		.name = "xhci-hcd",
	},
#ifdef	CONFIG_ARCH_LM2
	.suspend        = xhci_host_suspend,
	.resume         = xhci_host_resume,
#endif	/* CONFIG_ARCH_LM2 */
};
MODULE_ALIAS("platform:xhci-hcd");

int xhci_register_plat(void)
{
	return platform_driver_register(&usb_xhci_driver);
}

void xhci_unregister_plat(void)
{
	platform_driver_unregister(&usb_xhci_driver);
}
