/*
 * PCIe host controller driver for xxxx SoCs
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#if 0
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/of_pci.h>
#endif
#include <linux/pci.h>
#include <linux/pci_regs.h>
#include <linux/platform_device.h>
#include <linux/resource.h>
#include <linux/signal.h>
#include <linux/slab.h>
#include <linux/types.h>
#include "synopsys_pcie.h"

#undef	DEBUG_TRACE
#undef	DEBUG_RW

#define	PCIE_PORT1	1
#define	PCIE_PORT2	2
#define	PCIE_PORT3	3

#define	PCIE_LANE_PHY_CONF_1	1	/* link1:l4:cont1:p0 */
#define	PCIE_LANE_PHY_CONF_2	2
#define	PCIE_LANE_PHY_CONF_3	3	/* l1:p1:cont1:p0 default ! */

struct pcie_port_info {
	u32		io_size;
	u32		mem_size;
//	phys_addr_t	mem_bus_addr;
	struct resource		cfg;			/* ohkuma */
	void __iomem		*va_cfg;
	struct resource		io;			/* ohkuma */
	struct resource		mem;			/* ohkuma */
};

struct pcie_port {
	struct device		*dev;
	u8			controller;
	u8			root_bus_nr;
	void __iomem		*resetgen_base;
	void __iomem		*pciewrap_base;		/* ohkuma */
	void __iomem		*pciegen3_base1;	/* ohkuma */
	void __iomem		*pciegen3_base2;	/* ohkuma */
	void __iomem		*pciegen3_base3;	/* ohkuma */
	struct resource		resetgen;		/* ohkuma */
	struct resource		pciewrap;		/* ohkuma */
	struct resource		pciegen3[3];		/* ohkuma */
	spinlock_t		conf_lock;
	void __iomem		*va_cfg;
	struct resource		io;
	void __iomem		*va_io;
	struct resource		mem;
	void __iomem		*va_mem;
	struct pcie_port_info	config[2];
	struct clk		*clk;
	struct clk		*bus_clk;
	int			irq;
//	int			reset_gpio;
};

int	rc_num = 1;
int	ep_num = 2;
int	nu_num = 3;
int	bifur_num = 2;

#define IRQ_V2M_PCIE            (32 + 17)
/*
 * Exynos PCIe IP consists of Synopsys specific part and Exynos
 * specific part. Only core block is a Synopsys designware part;
 * other parts are Exynos specific.
 */

static struct hw_pci synopsys_pci;


#ifdef	DEBUG_RW1
static u32 inline synopsys_readl(void __iomem *offset)
{
	u32	val;
	val = readl(offset);
	printk(KERN_ERR "%s : Addr=0x%8.8x Data=0x%8.8x\n", __FUNCTION__, offset, val);
	return val;
}
static u32 inline synopsys_readw(void __iomem *offset)
{
	u32	val;
	val = readw(offset);
	printk(KERN_ERR "%s : Addr=0x%8.8x Data=0x%4.4x\n", __FUNCTION__, offset, val);
	return (val & 0x0000ffff);
}
static u32 inline synopsys_readb(void __iomem *offset)
{
	u32	val;
	val = readb(offset);
	printk(KERN_ERR "%s : Addr=0x%8.8x Data=0x%2.2x\n", __FUNCTION__, offset, val);
	return (val & 0x000000ff);
}

static void inline synopsys_writel(void __iomem *offset, u32 value)
{
	printk(KERN_ERR "%s : Addr=0x%8.8x Data=0x%8.8x\n", __FUNCTION__, offset, value);
	writel(value, offset);
}
static void inline synopsys_writew(void __iomem *offset, u32 value)
{
	printk(KERN_ERR "%s : Addr=0x%8.8x Data=0x%4.4x\n", __FUNCTION__, offset, value);
	writew(value, offset);
}
static void inline synopsys_writeb(void __iomem *offset, u32 value)
{
	printk(KERN_ERR "%s : Addr=0x%8.8x Data=0x%2.2x\n", __FUNCTION__, offset, value);
	writeb(value, offset);
}

#else

#define	synopsys_readl(offset)			readl(offset)
#define	synopsys_readw(offset)			readw(offset)
#define	synopsys_readb(offset)			readb(offset)
#define	synopsys_writel(offset,value)	writel(value, offset)
#define	synopsys_writew(offset,value)	writew(value, offset)
#define	synopsys_writeb(offset,value)	writeb(value, offset)

#endif



static inline struct pcie_port *sys_to_pcie(struct pci_sys_data *sys)
{
	return sys->private_data;
}


//static unsigned long global_io_offset;

static int synopsys_pcie_setup(int nr, struct pci_sys_data *sys)
{
	struct pcie_port *pp;

	pp = sys_to_pcie(sys);

	if (!pp) {
#ifdef	DEBUG_TRACE
		dev_err(pp->dev, "synopsys_pcie_setup: Error End\n");
#endif
		return 0;
	}
#ifdef	DEBUG_TRACE
	dev_err(pp->dev, "synopsys_pcie_setup: Start\n");
#endif


//	sys->mem_offset = pp->mem.start - pp->config.mem_bus_addr;
	pci_add_resource_offset(&sys->resources, &pp->mem, sys->mem_offset);

#ifdef	DEBUG_TRACE
	dev_err(pp->dev, "synopsys_pcie_setup: End\n");
#endif
	return 1;
}

static int synopsys_pcie_link_up(struct pcie_port *pp)
{
	u32 val;
	
	val = synopsys_readl(pp->pciewrap_base + PCIE1_MISC_STAT);
//	switch (pp->controller) {
//		case 0: val = synopsys_readl(pp->pciewrap_base + PCIE1_MISC_STAT);
//			   break;
//		case 1: val = synopsys_readl(pp->pciewrap_base + PCIE2_MISC_STAT);
//			   break;
//		case 2: val = synopsys_readl(pp->pciewrap_base + PCIE3_MISC_STAT);
//			   break;
//		default: dev_err(pp->dev, "error controller\n");
//			   break;
//	}
	if (val == PCIE1_MISC_STAT__GDA_PAB_DL_UP__MASK)
		return 1;

	return 0;
}

/*
 * PCI Express Configuration Register Access primitive
 * synopsys_pcie_rd_conf - read a register in configuration space
 * bus   : device data
 * devfn : function number
 * where : register offset(config space)
 * size  : byte 1,2,4
 * val   : read data
 */
static int synopsys_pcie_rd_conf(struct pci_bus *bus, u32 devfn, int where, int size, u32 *val)
{
	struct pcie_port	*pp = sys_to_pcie(bus->sysdata);
	void __iomem		*pcieconf_base;		/* Windows 0 cfg space */
	u32			bdf_adr;

#ifdef	DEBUG_TRACE1
//	dev_err(pp->dev, "%s entry bus=%x DevFn=%x where=%x size=%d\n",__FUNCTION__, bus->number, devfn, where, size);
#endif
	if (!pp) {
		BUG();
		return -EINVAL;
	}

	bdf_adr =(((bus->number << 24) & 0x0f000000) | ((devfn << 16) & 0x00ff0000));
	if (bus->number > 2) {
		*val = 0xffffffff;
		return PCIBIOS_DEVICE_NOT_FOUND;
	}
	if(bus->number != pp->root_bus_nr){
//		dev_err(pp->dev, "endpoint read access\n");
		pcieconf_base = (void __iomem *)((u32)pp->va_cfg + bdf_adr + where);
	}else{
		if(devfn != 0)	return	PCIBIOS_DEVICE_NOT_FOUND;
//		dev_err(pp->dev, "root read access");
		pcieconf_base = (void __iomem *)((u32)pp->pciegen3_base1 + where);
	}
	if (size == 1)
		*val = synopsys_readb(pcieconf_base);
	else if (size == 2)
		*val = synopsys_readw(pcieconf_base);
	else
		*val = synopsys_readl(pcieconf_base);
//	dev_err(pp->dev, " 0x%8.8lx : 0x%8.8lx\n", pcieconf_base, val);
out:	
#ifdef	DEBUG_TRACE1
	dev_err(pp->dev, "%s exit\n", __FUNCTION__);
#endif
	return PCIBIOS_SUCCESSFUL;
}

/*
 * PCI Express Configuration Register Access primitive
 * synopsys_pcie_wr_conf - write a register in configuration space
 * bus   : device data
 * devfn : device/function number
 * where : register offset (config sapce)
 * size  : byte 1,2,4
 * val   : write data
 */
//			return PCIBIOS_BAD_REGISTER_NUMBER;
static int synopsys_pcie_wr_conf(struct pci_bus *bus, u32 devfn,
			int where, int size, u32 val)
{
	struct pcie_port *pp = sys_to_pcie(bus->sysdata);
	void __iomem	*pcieconf_base;
	u32		bdf_adr;

#ifdef	DEBUG_TRACE1
//	dev_err(pp->dev, "write devfn = %x where = %x size = %x val = %x\n", devfn, where, size, val);
#endif
	if (!pp) {
		BUG();
		return -EINVAL;
	}

	bdf_adr =(((bus->number << 24) & 0x0f000000) | ((devfn << 16) & 0x00ff0000));
	if(bus->number > 2)
		return PCIBIOS_DEVICE_NOT_FOUND;

	if(bus->number != pp->root_bus_nr){
//		dev_err(pp->dev, "endpoint write access\n");
		pcieconf_base = (void __iomem *)((u32)pp->va_cfg + bdf_adr + where);
	}else{
		if(devfn != 0)	return	PCIBIOS_DEVICE_NOT_FOUND;
//		dev_err(pp->dev, "Root write access");
		pcieconf_base = (void __iomem *)((u32)pp->pciegen3_base1 + where);
	}
//	dev_err(pp->dev, " 0x%8.8lx : 0x%8.8x\n", pcieconf_base, val);
	if(size == 1)
		synopsys_writeb(pcieconf_base, val);
	else if(size == 2)
		synopsys_writew(pcieconf_base, val);
	else
		synopsys_writel(pcieconf_base, val);
out:
#ifdef	DEBUG_TRACE1
//	dev_err(pp->dev, "%s exit\n",__FUNCTION__);
#endif
	return PCIBIOS_SUCCESSFUL;
}

static struct pci_ops synopsys_pcie_ops = {
	.read  = synopsys_pcie_rd_conf,
	.write = synopsys_pcie_wr_conf,
};

static struct pci_bus *synopsys_pcie_scan_bus(int nr, struct pci_sys_data *sys)
{
	struct pci_bus *bus;
	struct pcie_port *pp = sys_to_pcie(sys);

#ifdef	DEBUG_TRACE
	dev_err(pp->dev, "%s: nr=%d bus=%d Start\n", __FUNCTION__, nr, sys->busnr);
#endif

	if (pp) {
		pp->root_bus_nr = sys->busnr;
		bus = pci_scan_root_bus(NULL, sys->busnr, &synopsys_pcie_ops, sys, &sys->resources);
	} else {
		bus = NULL;
		BUG();
	}
#ifdef	DEBUG_TRACE
	dev_err(pp->dev, "synopsys_pcie_scan_bus: End\n");
#endif
	return bus;
}

static int synopsys_pcie_map_irq(const struct pci_dev *dev, u8 slot, u8 pin)
{
	struct pcie_port *pp = sys_to_pcie(dev->bus->sysdata);

#ifdef	DEBUG_TRACE
	dev_err(pp->dev, "%s entry\n",__FUNCTION__);
#endif
	return pp->irq;
}

static struct hw_pci synopsys_pci = {
	.setup		= synopsys_pcie_setup,
	.scan		= synopsys_pcie_scan_bus,
	.map_irq	= synopsys_pcie_map_irq,
};


static void synopsys_pcie_assert_phy_reset(struct pcie_port *pp)
{
	u32 regVal;
	void __iomem *pciewrap_base = pp->pciewrap_base;
	
	regVal  = synopsys_readl(pciewrap_base + PCIE_PHY_RST_CTRL);
	regVal |= PCIE_PHY_RST_CTRL__PHY_RESET__MASK;
	synopsys_writel(pciewrap_base + PCIE_PHY_RST_CTRL, regVal);
}

static void synopsys_pcie_assert_pipe_reset(struct pcie_port *pp)
{
	u32 regVal;
	void __iomem *pciewrap_base = pp->pciewrap_base;
	
	regVal  = synopsys_readl(pciewrap_base + PCIE_PHY_RST_CTRL);
	regVal &= PCIE_PHY_RST_CTRL__PIPE0_RESET_N__INV_MASK;
	regVal &= PCIE_PHY_RST_CTRL__PIPE1_RESET_N__INV_MASK;
	regVal &= PCIE_PHY_RST_CTRL__PIPE2_RESET_N__INV_MASK;
	synopsys_writel(pciewrap_base + PCIE_PHY_RST_CTRL, regVal);
}

static void synopsys_pcie_assert_gpex_reset(struct pcie_port *pp)
{
	u32 regVal;
	void __iomem *pciewrap_base = pp->pciewrap_base;
	
	if( rc_num == 1 || ep_num == 1 || nu_num == 1 ) {
		regVal  = synopsys_readl(pciewrap_base + PCIE1_SW_RST);
		regVal &=(PCIE1_SW_RST__PAB_N__INV_MASK  &
			      PCIE1_SW_RST__AMBA_N__INV_MASK &
				  PCIE1_SW_RST__PBUS_N__INV_MASK &
			      PCIE1_SW_RST__LINK_N__INV_MASK);
		synopsys_writel(pciewrap_base + PCIE1_SW_RST, regVal);
	}
	if( rc_num == 2 || ep_num == 2 || nu_num == 2 ) {
		regVal  = synopsys_readl(pciewrap_base + PCIE2_SW_RST);
		regVal &=(PCIE2_SW_RST__PAB_N__INV_MASK  &
			      PCIE2_SW_RST__AMBA_N__INV_MASK &
				  PCIE2_SW_RST__PBUS_N__INV_MASK &
			      PCIE2_SW_RST__LINK_N__INV_MASK);
		synopsys_writel(pciewrap_base + PCIE2_SW_RST, regVal);
	}
	if( rc_num == 3 || ep_num == 3 || nu_num == 3 ) {
		regVal  = synopsys_readl(pciewrap_base + PCIE3_SW_RST);
		regVal &=(PCIE3_SW_RST__PAB_N__INV_MASK  &
			      PCIE3_SW_RST__AMBA_N__INV_MASK &
				  PCIE3_SW_RST__PBUS_N__INV_MASK &
			      PCIE3_SW_RST__LINK_N__INV_MASK);
		synopsys_writel(pciewrap_base + PCIE3_SW_RST, regVal);
	}
}

static void synopsys_pcie_set_bootstrap(struct pcie_port *pp, int which, int ep_rc)
{
	u32 regVal;
	void __iomem *pciewrap_base = pp->pciewrap_base;
	
	regVal = synopsys_readl(pciewrap_base + PCIE_SW_BOOTSTRAP);
	switch (which)
	{
		case 1 :
			if (ep_rc == 1)
				regVal |= PCIE_SW_BOOTSTRAP__PCIE1_EP_RC_SEL__MASK;
			else
				regVal &= PCIE_SW_BOOTSTRAP__PCIE1_EP_RC_SEL__INV_MASK;
			break;
		case 2 :
			if (ep_rc == 1)
				regVal |= PCIE_SW_BOOTSTRAP__PCIE2_EP_RC_SEL__MASK;
			else
				regVal &= PCIE_SW_BOOTSTRAP__PCIE2_EP_RC_SEL__INV_MASK;
			break;
		case 3 :
			if (ep_rc == 1)
				regVal |= PCIE_SW_BOOTSTRAP__PCIE3_EP_RC_SEL__MASK;
			else
				regVal &= PCIE_SW_BOOTSTRAP__PCIE3_EP_RC_SEL__INV_MASK;
			break;
		default :
			dev_err(pp->dev, "synopsys_pcie_set_bootstrap: which is %d Error\n", which);
				break;
	}
	synopsys_writel(pciewrap_base + PCIE_SW_BOOTSTRAP, regVal);
	
}

static void synopsys_pcie_pcie_bifur(struct pcie_port *pp, int port_sel)
{
	u32 regVal;
	void __iomem *pciewrap_base = pp->pciewrap_base;
	

	/* Set EP_RC_SEL */
	regVal = synopsys_readl(pciewrap_base + PCIE_SW_BOOTSTRAP);
	regVal &= PCIE_SW_BOOTSTRAP__PIPE_PORT_SEL__INV_MASK;
	regVal |= (port_sel << PCIE_SW_BOOTSTRAP__PIPE_PORT_SEL__SHIFT);
	synopsys_writel(pciewrap_base +PCIE_SW_BOOTSTRAP, regVal);

}

static void synopsys_pcie_deassert_phy_reset(struct pcie_port *pp)
{
	u32 regVal;
	void __iomem *pciewrap_base = pp->pciewrap_base;
	
	regVal  = synopsys_readl(pciewrap_base + PCIE_PHY_RST_CTRL);
	regVal &= PCIE_PHY_RST_CTRL__PHY_RESET__INV_MASK;
	synopsys_writel(pciewrap_base + PCIE_PHY_RST_CTRL, regVal);
}

static void synopsys_pcie_deassert_pipe_reset(struct pcie_port *pp, int which, int assert)
{
	u32 regVal;
	void __iomem *pciewrap_base = pp->pciewrap_base;
	
	regVal  = synopsys_readl(pciewrap_base + PCIE_PHY_RST_CTRL);
	switch(which) {
		case 1: 
			if (assert == 0)
				regVal |= PCIE_PHY_RST_CTRL__PIPE0_RESET_N__MASK;
			else
				regVal &= PCIE_PHY_RST_CTRL__PIPE0_RESET_N__INV_MASK;
			break;
		case 2: 
			if (assert == 0)
				regVal |= PCIE_PHY_RST_CTRL__PIPE1_RESET_N__MASK;
			else
				regVal &= PCIE_PHY_RST_CTRL__PIPE1_RESET_N__INV_MASK;
				break;
		case 3: 
			if (assert == 0)
				regVal |= PCIE_PHY_RST_CTRL__PIPE2_RESET_N__MASK;
			else
				regVal &= PCIE_PHY_RST_CTRL__PIPE2_RESET_N__INV_MASK;
			break;
		default:
			dev_err(pp->dev, "synopsys_pcie_deassert_pipe_reset: which is %d Error\n",which);
			break;
	}
	synopsys_writel(pciewrap_base + PCIE_PHY_RST_CTRL, regVal);
}

static void synopsys_pcie_pipe_ok(struct pcie_port *pp, int which)
{
	u32 regVal;
	void __iomem *pciewrap_base = pp->pciewrap_base;
	
	regVal  = synopsys_readl(pciewrap_base + PCIE_PHY_PIPE_STAT);
	switch(which) {
		case 1: while ((regVal &= PCIE_PHY_PIPE_STAT__PIPE0_PHYSTATUS__MASK) != 0)
		        {
				regVal = synopsys_readl(pciewrap_base + PCIE_PHY_PIPE_STAT);
		        }
		        break;
		case 2: while ((regVal &= PCIE_PHY_PIPE_STAT__PIPE1_PHYSTATUS__MASK) != 0)
		        {
				regVal = synopsys_readl(pciewrap_base + PCIE_PHY_PIPE_STAT);
		        }
		        break;
		case 3: while ((regVal &= PCIE_PHY_PIPE_STAT__PIPE2_PHYSTATUS__MASK) != 0)
		        {
				regVal = synopsys_readl(pciewrap_base + PCIE_PHY_PIPE_STAT);
		        }
		default:
			dev_err(pp->dev, "synopsys_pcie_pipe_ok: which is %d Error\n",which);
			break;
	}
}

static void synopsys_pcie_deassert_gpex_reset(struct pcie_port *pp, int which, int assert)
{
	u32 regVal;
	void __iomem *pciewrap_base = pp->pciewrap_base;
	
	switch(which) {
		case 1:
			regVal  = synopsys_readl(pciewrap_base + PCIE1_SW_RST);
			if (assert == 0) {
				regVal |=(PCIE1_SW_RST__PAB_N__MASK  |
					      PCIE1_SW_RST__AMBA_N__MASK |
						  PCIE1_SW_RST__PBUS_N__MASK |
						      PCIE1_SW_RST__LINK_N__MASK);
			} else {
				regVal &=(PCIE1_SW_RST__PAB_N__MASK  &
						      PCIE1_SW_RST__AMBA_N__MASK &
							  PCIE1_SW_RST__PBUS_N__MASK &
						      PCIE1_SW_RST__LINK_N__MASK);
			}
//			dev_err(pp->dev, "synopsys_pcie_deassert_gpex_reset:  %d \n",regVal);
			synopsys_writel(pciewrap_base + PCIE1_SW_RST, regVal);
		        break;
		case 2:
			regVal  = synopsys_readl(pciewrap_base + PCIE2_SW_RST);
			if (assert == 0) {
				regVal |=(PCIE2_SW_RST__PAB_N__MASK  |
					      PCIE2_SW_RST__AMBA_N__MASK |
						  PCIE2_SW_RST__PBUS_N__MASK |
					      PCIE2_SW_RST__LINK_N__MASK);
			} else {
				regVal &=(PCIE2_SW_RST__PAB_N__MASK  &
					      PCIE2_SW_RST__AMBA_N__MASK &
						  PCIE2_SW_RST__PBUS_N__MASK &
					      PCIE2_SW_RST__LINK_N__MASK);
			}
			synopsys_writel(pciewrap_base + PCIE2_SW_RST, regVal);
		        break;
		case 3: 
			regVal  = synopsys_readl(pciewrap_base + PCIE3_SW_RST);
			if (assert == 0) {
				regVal |=(PCIE3_SW_RST__PAB_N__MASK  |
					      PCIE3_SW_RST__AMBA_N__MASK |
						  PCIE3_SW_RST__PBUS_N__MASK |
						      PCIE3_SW_RST__LINK_N__MASK);
			} else {
				regVal &=(PCIE3_SW_RST__PAB_N__MASK  &
					      PCIE3_SW_RST__AMBA_N__MASK &
						  PCIE3_SW_RST__PBUS_N__MASK &
					      PCIE3_SW_RST__LINK_N__MASK);
			}
			synopsys_writel(pciewrap_base + PCIE3_SW_RST, regVal);
		        break;
		default:
			dev_err(pp->dev, "synopsys_pcie_deassert_gpex_reset: which is %d Error\n",which);
			break;
	}
}

static void synopsys_pcie_gpexd_core_clk_ratio(struct pcie_port *pp, int which)
{
	u32 regVal=0;
	void __iomem *pciewrap_base = pp->pciewrap_base;
	void __iomem *pciegen3_base1 = pp->pciegen3_base1;
	void __iomem *pciegen3_base2 = pp->pciegen3_base2;
	void __iomem *pciegen3_base3 = pp->pciegen3_base3;
	
	switch(which)
	{
		case 1: synopsys_writel(pciegen3_base1 + PCIE_GPEXD_CORE_CLK_RATIO, 0x10);
			while (regVal != PCIE1_MISC_STAT__GDA_PAB_DL_UP__MASK)
			{
				regVal = synopsys_readl(pciewrap_base + PCIE1_MISC_STAT);
				regVal &= PCIE1_MISC_STAT__GDA_PAB_DL_UP__MASK;
			}
		        break;
		case 2: synopsys_writel(pciegen3_base2 + PCIE_GPEXD_CORE_CLK_RATIO, 0x10);
			while (regVal != PCIE2_MISC_STAT__GDA_PAB_DL_UP__MASK)
			{
				regVal = synopsys_readl(pciewrap_base + PCIE2_MISC_STAT);
				regVal &= PCIE2_MISC_STAT__GDA_PAB_DL_UP__MASK;
			}
		        break;
		case 3: synopsys_writel(pciegen3_base3 + PCIE_GPEXD_CORE_CLK_RATIO, 0x10);
			while (regVal != PCIE3_MISC_STAT__GDA_PAB_DL_UP__MASK)
			{
				regVal = synopsys_readl(pciewrap_base + PCIE3_MISC_STAT);
				regVal &= PCIE3_MISC_STAT__GDA_PAB_DL_UP__MASK;
			}
		        break;
		default:
			dev_err(pp->dev, "synopsys_pcie_gpexd_core_clk_ratio: which is %d Error\n",which);
			break;
	}
}

static void synopsys_pcie_AxiToPexInit(struct pcie_port *pp, int which)
{
	void __iomem *pciegen3_base1 = pp->pciegen3_base1;
	
	switch (which)	{
		case 1:
		// PCIE1 has 512 MB AXI target space
		synopsys_writel(pciegen3_base1 + PCIE_PAB_AXI_PIO_CTRL0,
			ENABLE_ALL_ACCESS_TYPE|ENABLE_PORT);
			
		// - window 0
		//   - CFG access
		//   - 64 MB
		synopsys_writel(pciegen3_base1 + PCIE_PAB_AXI_AMAP_CTRL0,
		      PAB_AXI_AMAP_CTRL_64_CFG);
		synopsys_writel(pciegen3_base1 + PCIE_PAB_AXI_AMAP_AXI_BASE0,  0x00000000);
		synopsys_writel(pciegen3_base1 + PCIE_PAB_AXI_AMAP_AXI_BASE0X, 0x00000004);
		synopsys_writel(pciegen3_base1 + PCIE_PAB_AXI_AMAP_PEX_BASEL0, 0x00000000);
		synopsys_writel(pciegen3_base1 + PCIE_PAB_AXI_AMAP_PEX_BASEH0, 0x00000000);

		// - window 1
		//   - MEM access
		//   - 3 MB
		//   - axi side is not where dependent
		//   - pex side is where dependent
		synopsys_writel(pciegen3_base1 + PCIE_PAB_AXI_AMAP_CTRL1,      0x00030005);
		synopsys_writel(pciegen3_base1 + PCIE_PAB_AXI_AMAP_AXI_BASE1,  0x04000000);
		synopsys_writel(pciegen3_base1 + PCIE_PAB_AXI_AMAP_AXI_BASE1X, 0x00000004);
		synopsys_writel(pciegen3_base1 + PCIE_PAB_AXI_AMAP_PEX_BASEL1, 0x10000000);
		synopsys_writel(pciegen3_base1 + PCIE_PAB_AXI_AMAP_PEX_BASEH1, 0x00000000);
		// - window2
		//   - MEM access
		//   - 64 KB
		//   - axi side is not where dependent
		//   - pex side is where dependent
		synopsys_writel(pciegen3_base1 + PCIE_PAB_AXI_AMAP_CTRL2,      0x00010003);
		synopsys_writel(pciegen3_base1 + PCIE_PAB_AXI_AMAP_AXI_BASE2,  0x10000000);
		synopsys_writel(pciegen3_base1 + PCIE_PAB_AXI_AMAP_AXI_BASE2X, 0x00000004); 
		synopsys_writel(pciegen3_base1 + PCIE_PAB_AXI_AMAP_PEX_BASEL2, 0x10100000);
		synopsys_writel(pciegen3_base1 + PCIE_PAB_AXI_AMAP_PEX_BASEH2, 0x00000000);
		// - window 3
		//   - axi side is not where dependent
		//   - pex side is where dependent
		//   - MEM access
		//   - 64 MB
		break;
    	default: 
		dev_err(pp->dev, "synopsys_pcie_AxiToPexInit: which is %d Error\n",which);
		break;
	}
}

/*
 * RAM Address setting 0x8_0500_0000 - 0x8_bfff_ffff
 */
static void synopsys_pcie_PexToAxiInitRc(struct pcie_port *pp, int which)
{
//	u32 regVal=0;
//	void __iomem *pciewrap_base  = pp->pciewrap_base;
	void __iomem *pciegen3_base1 = pp->pciegen3_base1;
	void __iomem *pciegen3_base2 = pp->pciegen3_base2;
	void __iomem *pciegen3_base3 = pp->pciegen3_base3;
	
	switch (which)
	{
		case 1: 
			synopsys_writel(pciegen3_base1 + PCIE_PAB_PEX_PIO_CTRL0, ENABLE_PORT);
			// window 0
			synopsys_writel(pciegen3_base1 + PCIE_PAB_PEX_AMAP_CTRL0,      PAB_PEX_AMAP_CTRL0);
//			synopsys_writel(pciegen3_base1 + PCIE_PAB_PEX_AMAP_AXI_BASE0,  AXI_ADDR_L_DDR);
			synopsys_writel(pciegen3_base1 + PCIE_PAB_PEX_AMAP_AXI_BASE0,  0x05000000);
//			synopsys_writel(pciegen3_base1 + PCIE_PAB_PEX_AMAP_AXI_BASE0X, AXI_ADDR_H_DDR);
			synopsys_writel(pciegen3_base1 + PCIE_PAB_PEX_AMAP_AXI_BASE0X, 0x00000008);
			synopsys_writel(pciegen3_base1 + PCIE_PAB_PEX_AMAP_PEX_BASEL0, PEX_ADDR_L_PCIE1_DDR);
			synopsys_writel(pciegen3_base1 + PCIE_PAB_PEX_AMAP_PEX_BASEH0, PEX_ADDR_H_PCIE1_DDR);
#if 0
			// window 1
			synopsys_writel(pciegen3_base1 + PCIE_PAB_PEX_AMAP_CTRL1,      PAB_PEX_AMAP_CTRL1);
			synopsys_writel(pciegen3_base1 + PCIE_PAB_PEX_AMAP_AXI_BASE1,  AXI_ADDR_SP);
			synopsys_writel(pciegen3_base1 + PCIE_PAB_PEX_AMAP_PEX_BASEL1, PEX_ADDR_L_PCIE1_SP);
			synopsys_writel(pciegen3_base1 + PCIE_PAB_PEX_AMAP_PEX_BASEH1, PEX_ADDR_H_PCIE1_SP);
			// window 2
			synopsys_writel(pciegen3_base1 + PCIE_PAB_PEX_AMAP_CTRL2,      PAB_PEX_AMAP_CTRL2);
			synopsys_writel(pciegen3_base1 + PCIE_PAB_PEX_AMAP_AXI_BASE2,  AXI_ADDR_XYZ);
			synopsys_writel(pciegen3_base1 + PCIE_PAB_PEX_AMAP_PEX_BASEL2, PEX_ADDR_PCIE1_XYZ);
			synopsys_writel(pciegen3_base1 + PCIE_PAB_PEX_AMAP_PEX_BASEH2, 0x00000000);
			// window 3
			synopsys_writel(pciegen3_base1 + PCIE_PAB_PEX_AMAP_CTRL3,      PAB_PEX_AMAP_CTRL3);
			synopsys_writel(pciegen3_base1 + PCIE_PAB_PEX_AMAP_AXI_BASE3,  0x00000000); // not applicable
			synopsys_writel(pciegen3_base1 + PCIE_PAB_PEX_AMAP_PEX_BASEL3, PEX_ADDR_L_PCIE1_MSI);
			synopsys_writel(pciegen3_base1 + PCIE_PAB_PEX_AMAP_PEX_BASEH3, PEX_ADDR_H_PCIE1_MSI);
#endif 	/* not used */
			break;
		case 2: 
			synopsys_writel(pciegen3_base2 + PCIE_PAB_PEX_PIO_CTRL0, ENABLE_PORT);
			// window 0
			synopsys_writel(pciegen3_base2 + PCIE_PAB_PEX_AMAP_CTRL0,      PAB_PEX_AMAP_CTRL0);
			synopsys_writel(pciegen3_base2 + PCIE_PAB_PEX_AMAP_AXI_BASE0,  AXI_ADDR_L_DDR);
			synopsys_writel(pciegen3_base2 + PCIE_PAB_PEX_AMAP_AXI_BASE0X,  AXI_ADDR_H_DDR);
			synopsys_writel(pciegen3_base2 + PCIE_PAB_PEX_AMAP_PEX_BASEL0, PEX_ADDR_L_PCIE2_DDR_RC); // NOTE special _RC suffix
			synopsys_writel(pciegen3_base2 + PCIE_PAB_PEX_AMAP_PEX_BASEH0, PEX_ADDR_H_PCIE2_DDR_RC); // NOTE special _RC suffix
#if 0
			// window 1
			synopsys_writel(pciegen3_base2 + PCIE_PAB_PEX_AMAP_CTRL1,      PAB_PEX_AMAP_CTRL1);
			synopsys_writel(pciegen3_base2 + PCIE_PAB_PEX_AMAP_AXI_BASE1,  AXI_ADDR_SP);
			synopsys_writel(pciegen3_base2 + PCIE_PAB_PEX_AMAP_PEX_BASEL1, PEX_ADDR_L_PCIE2_SP_RC);  // NOTE special _RC suffix
			synopsys_writel(pciegen3_base2 + PCIE_PAB_PEX_AMAP_PEX_BASEH1, PEX_ADDR_H_PCIE2_SP_RC);  // NOTE special _RC suffix
			// window 2
			synopsys_writel(pciegen3_base2 + PCIE_PAB_PEX_AMAP_CTRL2,      PAB_PEX_AMAP_CTRL2);
			synopsys_writel(pciegen3_base2 + PCIE_PAB_PEX_AMAP_AXI_BASE2,  AXI_ADDR_XYZ);
			synopsys_writel(pciegen3_base2 + PCIE_PAB_PEX_AMAP_PEX_BASEL2, PEX_ADDR_PCIE2_XYZ_RC);   // NOTE special _RC suffix
			synopsys_writel(pciegen3_base2 + PCIE_PAB_PEX_AMAP_PEX_BASEH2, 0x00000000);
			// window 3
			synopsys_writel(pciegen3_base2 + PCIE_PAB_PEX_AMAP_CTRL3,      PAB_PEX_AMAP_CTRL3);
			synopsys_writel(pciegen3_base2 + PCIE_PAB_PEX_AMAP_AXI_BASE3,  0x00000000); // not applicable
			synopsys_writel(pciegen3_base2 + PCIE_PAB_PEX_AMAP_PEX_BASEL3, PEX_ADDR_L_PCIE2_MSI_RC); // NOTE special _RC suffix
			synopsys_writel(pciegen3_base2 + PCIE_PAB_PEX_AMAP_PEX_BASEH3, PEX_ADDR_H_PCIE2_MSI_RC); // NOTE special _RC suffix
#endif
			break;
		case 3: 
			synopsys_writel(pciegen3_base3 + PCIE_PAB_PEX_PIO_CTRL0, ENABLE_PORT);
			// window 0
			synopsys_writel(pciegen3_base3 + PCIE_PAB_PEX_AMAP_CTRL0,      PAB_PEX_AMAP_CTRL0);
			synopsys_writel(pciegen3_base3 + PCIE_PAB_PEX_AMAP_AXI_BASE0,  AXI_ADDR_L_DDR);
			synopsys_writel(pciegen3_base3 + PCIE_PAB_PEX_AMAP_AXI_BASE0X,  AXI_ADDR_H_DDR);
			synopsys_writel(pciegen3_base3 + PCIE_PAB_PEX_AMAP_PEX_BASEL0, PEX_ADDR_L_PCIE3_DDR);
			synopsys_writel(pciegen3_base3 + PCIE_PAB_PEX_AMAP_PEX_BASEH0, PEX_ADDR_H_PCIE3_DDR);
#if 0
			// window 1
			synopsys_writel(pciegen3_base3 + PCIE_PAB_PEX_AMAP_CTRL1,      PAB_PEX_AMAP_CTRL1);
			synopsys_writel(pciegen3_base3 + PCIE_PAB_PEX_AMAP_AXI_BASE1,  AXI_ADDR_SP);
			synopsys_writel(pciegen3_base3 + PCIE_PAB_PEX_AMAP_PEX_BASEL1, PEX_ADDR_L_PCIE3_SP);
			synopsys_writel(pciegen3_base3 + PCIE_PAB_PEX_AMAP_PEX_BASEH1, PEX_ADDR_H_PCIE3_SP);
			// window 2
			synopsys_writel(pciegen3_base3 + PCIE_PAB_PEX_AMAP_CTRL2,      PAB_PEX_AMAP_CTRL2);
			synopsys_writel(pciegen3_base3 + PCIE_PAB_PEX_AMAP_AXI_BASE2,  AXI_ADDR_XYZ);
			synopsys_writel(pciegen3_base3 + PCIE_PAB_PEX_AMAP_PEX_BASEL2, PEX_ADDR_PCIE3_XYZ);
			synopsys_writel(pciegen3_base3 + PCIE_PAB_PEX_AMAP_PEX_BASEH2, 0x00000000);
			// window 3
			synopsys_writel(pciegen3_base3 + PCIE_PAB_PEX_AMAP_CTRL3,      PAB_PEX_AMAP_CTRL3);
			synopsys_writel(pciegen3_base3 + PCIE_PAB_PEX_AMAP_AXI_BASE3,  0x00000000); // not applicable
			synopsys_writel(pciegen3_base3 + PCIE_PAB_PEX_AMAP_PEX_BASEL3, PEX_ADDR_L_PCIE3_MSI);
			synopsys_writel(pciegen3_base3 + PCIE_PAB_PEX_AMAP_PEX_BASEH3, PEX_ADDR_H_PCIE3_MSI);
#endif
			break;
		default:
			dev_err(pp->dev, "synopsys_pcie_PexToAxiInitRc: which is %d Error\n",which);
			break;
	}
}

static void synopsys_pcie_PexToAxiInitEp(struct pcie_port *pp, int which)
{
//	u32 regVal=0;
//	void __iomem *pciewrap_base  = pp->pciewrap_base;
	void __iomem *pciegen3_base1 = pp->pciegen3_base1;
	void __iomem *pciegen3_base2 = pp->pciegen3_base2;
	void __iomem *pciegen3_base3 = pp->pciegen3_base3;

	switch (which)
	{
		case 1: 
			synopsys_writel(pciegen3_base1 + PCIE_PAB_PEX_PIO_CTRL0, ENABLE_PORT);
			synopsys_writel(pciegen3_base1 + PCIE_PAB_PEX_AMAP_BAR0_F0, AXI_ADDR_DDR | ENABLE_BIT);
			synopsys_writel(pciegen3_base1 + PCIE_PAB_PEX_AMAP_BAR1_F0, AXI_ADDR_SP  | ENABLE_BIT);
			synopsys_writel(pciegen3_base1 + PCIE_PAB_PEX_AMAP_BAR2_F0, AXI_ADDR_XYZ | ENABLE_BIT);
			break;
		case 2: 
			synopsys_writel(pciegen3_base2 + PCIE_PAB_PEX_PIO_CTRL0, ENABLE_PORT);
			synopsys_writel(pciegen3_base2 + PCIE_PAB_PEX_AMAP_BAR0_F0, AXI_ADDR_DDR | ENABLE_BIT);
			synopsys_writel(pciegen3_base2 + PCIE_PAB_PEX_AMAP_BAR1_F0, AXI_ADDR_SP  | ENABLE_BIT);
			synopsys_writel(pciegen3_base2 + PCIE_PAB_PEX_AMAP_BAR2_F0, AXI_ADDR_XYZ | ENABLE_BIT);
			break;
		case 3: 
			synopsys_writel(pciegen3_base3 + PCIE_PAB_PEX_PIO_CTRL0, ENABLE_PORT);
			synopsys_writel(pciegen3_base3 + PCIE_PAB_PEX_AMAP_BAR0_F0, AXI_ADDR_DDR | ENABLE_BIT);
			synopsys_writel(pciegen3_base3 + PCIE_PAB_PEX_AMAP_BAR1_F0, AXI_ADDR_SP  | ENABLE_BIT);
			synopsys_writel(pciegen3_base3 + PCIE_PAB_PEX_AMAP_BAR2_F0, AXI_ADDR_XYZ | ENABLE_BIT);
			break;
		default:
			dev_err(pp->dev, "synopsys_pcie_PexToAxiInitEp: which is %d Error\n",which);
      		break;
	}
}



static int synopsys_pcie_establish_link(struct pcie_port *pp)
{
	u32 val;
//	int count = 0;
	void __iomem *pciewrap_base  = pp->pciewrap_base;
	void __iomem *pciegen3_base1 = pp->pciegen3_base1;
	void __iomem *pciegen3_base2 = pp->pciegen3_base2;
//	void __iomem *pciegen3_base3 = pp->pciegen3_base3;
#ifdef	DEBUG_TRACE
	dev_err(pp->dev, "synopsys_pcie_establish_link: Start(rc_num=%d ep_num=%d nu_num=%d)\n",rc_num,ep_num,nu_num);
#endif
	if (synopsys_pcie_link_up(pp)) {
		dev_err(pp->dev, "synopsys_pcie_establish_link: Link already up\n");
		return 0;
	}
	/* assert PHY Reset */
	synopsys_pcie_assert_phy_reset(pp);

	/* assert PIPE Reset */
	synopsys_pcie_assert_pipe_reset(pp);
	
	/* assert GPEX Reset */
	synopsys_pcie_assert_gpex_reset(pp);
	
	/* Set sw_bootstrap Root Complex*/
	synopsys_pcie_set_bootstrap(pp, PCIE_PORT1, 1);
	synopsys_pcie_set_bootstrap(pp, PCIE_PORT2, 1);
	synopsys_pcie_set_bootstrap(pp, PCIE_PORT3, 1);
	/* Connetc lane */
	synopsys_pcie_pcie_bifur(pp, PCIE_LANE_PHY_CONF_3);
	
	// - NEW FOR 55xx PHY
	//   - Specify ref_clk for PHY
	//   - use external ref_clk for this test so set phy_ref_use_pad control bit
	val  = synopsys_readl(pciewrap_base + PCIE_PHY_CLK_CTRL);
	val |= PCIE_PHY_CLK_CTRL__PHY_REF_USE_PAD__MASK;
	synopsys_writel(pciewrap_base + PCIE_PHY_CLK_CTRL, val);

	// - NEW FOR 55xx PHY
	//   - assert macP_pclkreq_n inputs to PHY for each PIPE being used
	//   - if you do not do this then PHY will kill mpll_dword_clk output
	//     and pipe#_phystatus will not fall to indicate pipe is ready
	val = synopsys_readl(pciewrap_base + PCIE_PHY_PIPE_CTRL);
	val &= PCIE_PHY_PIPE_CTRL__MAC0_PCLKREQ_N__INV_MASK; // MAC0 = PCIE1
	val &= PCIE_PHY_PIPE_CTRL__MAC1_PCLKREQ_N__INV_MASK; // MAC1 = PCIE2
	synopsys_writel(pciewrap_base + PCIE_PHY_PIPE_CTRL, val);
	
	/* de-assert PHY Reset */
	synopsys_pcie_deassert_phy_reset(pp);

	/* de-assert PIPE Reset */
	synopsys_pcie_deassert_pipe_reset(pp, PCIE_PORT1, 0);
//	synopsys_pcie_deassert_pipe_reset(pp, PCIE_PORT2, 0);
//	synopsys_pcie_deassert_pipe_reset(pp, PCIE_PORT3, 0);
	
	/*  PIPE Status Check */
	synopsys_pcie_pipe_ok(pp, PCIE_PORT1);
//	synopsys_pcie_pipe_ok(pp, PCIE_PORT2);
//	synopsys_pcie_pipe_ok(pp, PCIE_PORT3);
	
	/* de-assert GPEX Reset */
	synopsys_pcie_deassert_gpex_reset(pp, PCIE_PORT1, 0);
//	synopsys_pcie_deassert_gpex_reset(pp, PCIE_PORT2, 0);
//	synopsys_pcie_deassert_gpex_reset(pp, PCIE_PORT3, 0);
	
	/* Set GPEXD_CORE_CLK_RATIO  */
//	synopsys_pcie_gpexd_core_clk_ratio(pp, rc_num);	/* yamano */
//	synopsys_pcie_gpexd_core_clk_ratio(pp, ep_num);
	//-------------------------------------------------------
	// - INITIALIZE AXI and PEX WINDOWS
	//-------------------------------------------------------
	synopsys_writel(pciegen3_base1 + PCIE_PAB_CTRL, 0x00000a2f);	/* yamano */
//	synopsys_writel(pciegen3_base2 + PCIE_PAB_CTRL, 0x00000a2f);
	/* initialize AXI to PEX windows for RC to EP accesses */
	synopsys_pcie_AxiToPexInit(pp, rc_num);
	
	/* initialize AXI to PEX windows for EP to RC accesses */
//	synopsys_pcie_AxiToPexInit(pp, ep_num, rc_num, rc_num);	/* yamano */
	
	/* initialize root complex registers */
	synopsys_pcie_PexToAxiInitRc(pp, PCIE_PORT1);
	
	/* initialize endpoint registers */
//	synopsys_pcie_PexToAxiInitEp(pp, ep_num);	/* yamano debug */
	
//	/* check if the link is up or not */
//	while (!synopsys_pcie_link_up(pp)) {
//		mdelay(100);
//		count++;
//		if (count == 10) {
//			dev_err(pp->dev, "PCIe Link Fail\n");
//			return -EINVAL;
//		}
//	}
//	
//	dev_info(pp->dev, "Link up\n");
out:
#ifdef	DEBUG_TRACE
	dev_err(pp->dev, "synopsys_pcie_establish_link: End\n");
#endif
	return 0;
}

static void exynos_pcie_clear_irq_pulse(struct pcie_port *pp)
{
	void __iomem *resetgen_base  = pp->resetgen_base;
	
	synopsys_writel(resetgen_base + PCIE1_INT_CLR, PCIE1_INT_CLR__PERST_N_PIN__MASK | PCIE1_INT_CLR__GDA_PAB__MASK);
	synopsys_writel(resetgen_base + PCIE1_INT_CLR, PCIE2_INT_CLR__PERST_N_PIN__MASK | PCIE2_INT_CLR__GDA_PAB__MASK);
	synopsys_writel(resetgen_base + PCIE1_INT_CLR, PCIE3_INT_CLR__PERST_N_PIN__MASK | PCIE3_INT_CLR__GDA_PAB__MASK);
	return;
}

static void synopsys_pcie_enable_irq_pulse(struct pcie_port *pp)
{
//	u32 val;
	void __iomem *resetgen_base  = pp->resetgen_base;

	/* enable INTX interrupt */
	synopsys_writel(resetgen_base + PCIE1_INT_EN, PCIE1_INT_EN__PERST_N_PIN__MASK | PCIE1_INT_EN__GDA_PAB__MASK);
	synopsys_writel(resetgen_base + PCIE2_INT_EN, PCIE2_INT_EN__PERST_N_PIN__MASK | PCIE2_INT_EN__GDA_PAB__MASK);
	synopsys_writel(resetgen_base + PCIE3_INT_EN, PCIE3_INT_EN__PERST_N_PIN__MASK | PCIE3_INT_EN__GDA_PAB__MASK);
	return;
}

static irqreturn_t exynos_pcie_irq_handler(int irq, void *arg)
{
	struct pcie_port *pp = arg;

	exynos_pcie_clear_irq_pulse(pp);
	return IRQ_HANDLED;
}

static void synopsys_pcie_enable_interrupts(struct pcie_port *pp)
{
	synopsys_pcie_enable_irq_pulse(pp);
	return;
}

static int  synopsys_pcie_host_init(struct pcie_port *pp)
{
	struct pcie_port_info *config = &pp->config;
	u32 val,result,wait_loop;
	u64	adr_base;
	void __iomem *resetgen_base  = pp->resetgen_base;
	void __iomem *pciewrap_base  = pp->pciewrap_base;
	void __iomem *pciegen3_base1 = pp->pciegen3_base1;
	void __iomem *pciegen3_base2 = pp->pciegen3_base2;
//	void __iomem *pciegen3_base3 = pp->pciegen3_base3;
	void __iomem *conFig;

#ifdef	DEBUG_TRACE
	dev_err(pp->dev, "synopsys_pcie_host_init: Start\n");
#endif

	/* PCIE core resets from RSTGEN default to asserted, deassert them now */
	val = synopsys_readl(resetgen_base + RSTGENSWRSTSTATIC10);
	val &= 0xff0fffff;
	synopsys_writel(resetgen_base + RSTGENSWRSTSTATIC10, val);

	/* enable link */
	synopsys_pcie_establish_link(pp);

	/* PCIE1 (RC) */
	val = synopsys_readl(pciegen3_base1 + PCIE_GPEXP_CFG_VENDORID);
	if ( val != 0x000811de ) {
		dev_err(pp->dev, "synopsys_pcie_host_init: PCIE_GPEXP_CFG_VENDORID error rc(0x%x != 0x000811de)\n",val);
	}
	val = synopsys_readl(pciegen3_base1 + PCIE_GPEXD_ID);
	if ( val != 0x000811de ) {
		dev_err(pp->dev, "synopsys_pcie_host_init: PCIE_GPEXD_ID error rc(0x%x != 0x000811de)\n",val);
	}
	synopsys_writel(pciegen3_base1 + PCIE_GPEXD_ID, 0x55001135);
	val = synopsys_readl(pciegen3_base1 + PCIE_GPEXP_CFG_VENDORID);
	if ( val != 0x55001135 ) {
		dev_err(pp->dev, "synopsys_pcie_host_init: PCIE_GPEXP_CFG_VENDORID error rc(0x%x != 0x123411de)\n",val);
	}
	val = synopsys_readl(pciegen3_base1 + PCIE_GPEXD_ID);
	if ( val != 0x55001135 ) {
		dev_err(pp->dev, "synopsys_pcie_host_init: PCIE_GPEXD_ID error rc(0x%x != 0x123411de)\n",val);
	}
	synopsys_writel(pciegen3_base1 + PCIE_GPEXD_CLASSCODE, 0xffffffff);
	val = synopsys_readl(pciegen3_base1 + PCIE_GPEXD_CLASSCODE);
//	dev_err(pp->dev, "PCIE_GPEXD_CLASSCODE 0x%x\n",val);

        synopsys_writel(pciegen3_base1 + PCIE_GPEXP_CFG_BASE2_PRIBUS, 0x00010100);
        val = synopsys_readl(pciegen3_base1 + PCIE_GPEXP_CFG_BASE2_PRIBUS);
//      dev_err(pp->dev, "PCIE_GPEXP_CFG_BASE2_PRIBUS 0x%x\n",val);

	/* SET GPEXD_CFG_RDY bit */
	val = synopsys_readl(pciegen3_base1 + PCIE_GPEXD_CFG_RDY);
	if ( (val & 0x1) != 0x0 ) {
		dev_err(pp->dev, "synopsys_pcie_host_init: PCIE_GPEXD_CFG_RDY error\n");
	}
/*
	val = synopsys_readl(pciegen3_base2 + PCIE_GPEXD_CFG_RDY);
	if ( (val & 0x1) != 0x0 ) {
		dev_err(pp->dev, "synopsys_pcie_host_init: PCIE_GPEXD_CFG_RDY error\n");
	}
*/	
	val = synopsys_readl(pciegen3_base1 + PCIE_GPEXD_CFG_RDY);
	val |= 0x1;
	synopsys_writel(pciegen3_base1 + PCIE_GPEXD_CFG_RDY, val);
/*	
	val = synopsys_readl(pciegen3_base2 + PCIE_GPEXD_CFG_RDY);
	val |= 0x1;
	synopsys_writel(pciegen3_base2 + PCIE_GPEXD_CFG_RDY, val);
*/	
	/* locally initialize more PCIE1 RC CFG regs (45xx legacy code) */
	val = synopsys_readl(pciegen3_base1 + PCIE_GPEXP_CFG_CACHE);
	val = synopsys_readl(pciegen3_base1 + PCIE_GPEXP_CFG_BASE3_IOBASE);
	synopsys_writel(pciegen3_base1 + PCIE_GPEXP_CFG_BASE4_MEMBASE, PCIE1_MEM_LIMIT_BASE);
	val = synopsys_readl(pciegen3_base1 + PCIE_GPEXP_CFG_BASE4_MEMBASE);
	synopsys_writel(pciegen3_base1 + PCIE_GPEXP_CFG_BASE5_PMEMBASE, PCIE1_PMEM_LIMIT_BASE_L);
	val = synopsys_readl(pciegen3_base1 + PCIE_GPEXP_CFG_BASE5_PMEMBASE);
	synopsys_writel(pciegen3_base1 + PCIE_GPEXP_CFG_X_PBASEUDW, PCIE1_PMEM_BASE_U);
	synopsys_writel(pciegen3_base1 + PCIE_GPEXP_CFG_SUBVENID_PLIMITUDW, PCIE1_PMEM_LIMIT_U);
	
	/* SET BITS in PCIE1 PCIE Command Register */
	val  = synopsys_readl(pciegen3_base1 + PCIE_GPEXP_CFG_COMMAND);
	val |= 0x6;
	synopsys_writel((pciegen3_base1 + PCIE_GPEXP_CFG_COMMAND), val);
	
	/* CHANGE PIOS_CONV_SEL to 01 */
//	val  = synopsys_readl(resetgen_base + PCIE1_MISC_CTRL);
//	val &= PCIE1_MISC_CTRL__PIOS_CONV_SEL__INV_MASK;
//	val |= (0x1 << PCIE1_MISC_CTRL__PIOS_CONV_SEL__SHIFT);
//	synopsys_writel(resetgen_base + PCIE1_MISC_CTRL, val);
	result = 0;
	wait_loop=0;
	val = synopsys_readl(pciewrap_base + PCIE1_MISC_STAT);
	while(val != PCIE1_MISC_STAT__GDA_PAB_DL_UP__MASK){
		if(wait_loop > 1000){
			result = 1;
			break;
		}
		msleep(10);
		wait_loop++;
		val = synopsys_readl(pciewrap_base + PCIE1_MISC_STAT);
		smp_wmb();
	};
	if(result != 0){
		dev_err(pp->dev, "PCIe can't Data link Up\n");
		return	-1;
	}
//
//	adr_base = 0x400000000ULL;
//	conFig = ioremap(adr_base, 0x10000000);

//	dev_err(pp->dev, "addr = %llx vadr = %lx\n", adr_base,(u32)conFig);
//	dev_err(pp->dev, "reg = %x\n",*((int *)(conFig+0x01000000)));
//	val = synopsys_readl(conFig);
//	dev_err(pp->dev, "reg = %x\n",val);
//	iounmap(conFig);
//	synopsys_pcie_enable_interrupts(pp);
out:
#ifdef	DEBUG_TRACE
	dev_err(pp->dev, "synopsys_pcie_host_init:%x End\n",pciegen3_base1);
#endif
	return	0;
}

static int add_pcie_port(struct pcie_port *pp, struct platform_device *pdev)
{
	struct resource *tmp;
	int ret;
#ifdef	DEBUG_TRACE
	dev_err(pp->dev, "add_pcie_port: Start\n");
#endif
	tmp = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!tmp) {
		dev_err(pp->dev, "add_pcie_port: couldn't get resetgen base resource\n");
		return -EINVAL;
	}
	pp->resetgen_base = devm_ioremap_resource(&pdev->dev, tmp);
	if (IS_ERR(pp->resetgen_base))
		return PTR_ERR(pp->resetgen_base);
	
	
	tmp = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!tmp) {
		dev_err(pp->dev, "add_pcie_port: couldn't get pciewrap base resource\n");
		return -EINVAL;
	}
	pp->pciewrap_base = devm_ioremap_resource(&pdev->dev, tmp);
	if (IS_ERR(pp->pciewrap_base))
		return PTR_ERR(pp->pciewrap_base);
	
	
	tmp = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	if (!tmp) {
		dev_err(pp->dev, "add_pcie_port: couldn't get pciegen3_1 base resource\n");
		return -EINVAL;
	}
	pp->pciegen3_base1 = devm_ioremap_resource(&pdev->dev, tmp);
	if (IS_ERR(pp->pciegen3_base1))
		return PTR_ERR(pp->pciegen3_base1);
	
	
	tmp = platform_get_resource(pdev, IORESOURCE_MEM, 3);
	if (!tmp) {
		dev_err(pp->dev, "add_pcie_port: couldn't get pciegen3_2 base resource\n");
		return -EINVAL;
	}
	pp->pciegen3_base2 = devm_ioremap_resource(&pdev->dev, tmp);
	if (IS_ERR(pp->pciegen3_base2))
		return PTR_ERR(pp->pciegen3_base2);
	
	
	tmp = platform_get_resource(pdev, IORESOURCE_MEM, 4);
	if (!tmp) {
		dev_err(pp->dev, "add_pcie_port: couldn't get pciegen3_3 base resource\n");
		return -EINVAL;
	}
	pp->pciegen3_base3 = devm_ioremap_resource(&pdev->dev, tmp);
	if (IS_ERR(pp->pciegen3_base3))
		return PTR_ERR(pp->pciegen3_base3);
	
	pp->irq = IRQ_V2M_PCIE;
//	pp->irq = platform_get_irq(pdev, 1);
	if (!pp->irq) {
		dev_err(pp->dev, "add_pcie_port: failed to get irq\n");
		return -ENODEV;
	}
//	ret = devm_request_irq(&pdev->dev, pp->irq, exynos_pcie_irq_handler, IRQF_SHARED, "synopsys-pcie", pp);
//	if (ret) {
//		dev_err(pp->dev, "add_pcie_port: failed to request irq\n");
//		return ret;
//	}
	
	
	pp->root_bus_nr = 0;

	spin_lock_init(&pp->conf_lock);
	ret = synopsys_pcie_host_init(pp);
	if(ret != 0){
		return	-ENODEV;
	}
#ifdef	DEBUG_TRACE
	dev_err(pp->dev, "add_pcie_port: End\n");
#endif
	return 0;
}

/*
 * PCI-Express Driver probe
 */
static int __init synopsys_pcie_probe(struct platform_device *pdev)
{
	struct pcie_port *pp;
#if 1
#ifdef	DEBUG_TRACE
	dev_info(&pdev->dev, "synopsys_pcie_probe :Start\n");
#endif
#else
	struct device_node *np = pdev->dev.of_node;
	struct of_pci_range range;
	struct of_pci_range_parser parser;
#endif
	int ret;
	pp = devm_kzalloc(&pdev->dev, sizeof(*pp), GFP_KERNEL);
	if (!pp) {
		dev_err(pp->dev, "synopsys_pcie_probe: no memory for pcie port\n");
		return -ENOMEM;
	}

	pp->dev = &pdev->dev;
#if 0
	if (of_pci_range_parser_init(&parser, np)) {
		dev_err(pp->dev, "synopsys_pcie_probe: missing ranges property\n");
		return -EINVAL;
	}

	/* Get the I/O and memory ranges from DT */
	for_each_of_pci_range(&parser, &range) {
		unsigned long restype = range.flags & IORESOURCE_TYPE_BITS;
		if (restype == IORESOURCE_IO) {
			of_pci_range_to_resource(&range, np, &pp->io);
			pp->io.name = "I/O";
			pp->io.start = max_t(resource_size_t,
					     PCIBIOS_MIN_IO,
					     range.pci_addr + global_io_offset);
			pp->io.end = min_t(resource_size_t,
					   IO_SPACE_LIMIT,
					   range.pci_addr + range.size
					   + global_io_offset);
			pp->config.io_size = resource_size(&pp->io);
			pp->config.io_bus_addr = range.pci_addr;
		}
		if (restype == IORESOURCE_MEM) {
			of_pci_range_to_resource(&range, np, &pp->mem);
			pp->mem.name = "MEM";
			pp->config.mem_size = resource_size(&pp->mem);
			pp->config.mem_bus_addr = range.pci_addr;
		}
		if (restype == 0) {
			of_pci_range_to_resource(&range, np, &pp->cfg);
			pp->config.cfg0_size = resource_size(&pp->cfg)/2;
			pp->config.cfg1_size = resource_size(&pp->cfg)/2;
		}
	}
#endif

	pp->io.name	= "Multiport";
	pp->io.start	= 0x410000000ULL;
	pp->io.end	= 0x41000ffffULL;
	pp->io.flags	= IORESOURCE_MEM;
	pp->va_io = ioremap(0x410000000ULL,SZ_64K);
	pp->config[0].io_size = resource_size(&pp->io);
//	pp->config[0].io_bus_addr	= 0x410000000ULL;

	pp->mem.name	= "Memory";
	pp->mem.start	= 0x400000000ULL;
	pp->mem.end	= 0x40fffffffULL;
	pp->va_cfg = ioremap(0x400000000ULL,SZ_64M);
	pp->va_mem = ioremap(0x404000000ULL,SZ_128+SZ_64);
	pp->config[0].mem_size = resource_size(&pp->mem);
//	pp->config[0].mem_bus_addr	= 0x400000000ULL;

/*
	pp->clk = devm_clk_get(&pdev->dev, "pcie");
	if (IS_ERR(pp->clk)) {
		dev_err(pp->dev, "synopsys_pcie_probe: Failed to get pcie rc clock\n");
		return PTR_ERR(pp->clk);
	}
	ret = clk_prepare_enable(pp->clk);
	if (ret)
		return ret;

	pp->bus_clk = devm_clk_get(&pdev->dev, "pcie_bus");
	if (IS_ERR(pp->bus_clk)) {
		dev_err(pp->dev, "synopsys_pcie_probe: Failed to get pcie bus clock\n");
		ret = PTR_ERR(pp->bus_clk);
		goto fail_clk;
	}
	ret = clk_prepare_enable(pp->bus_clk);
	if (ret)
		goto fail_clk;
*/
	ret = add_pcie_port(pp, pdev);
	if (ret < 0)
		goto fail_bus_clk;

	synopsys_pci.nr_controllers = 1;
	synopsys_pci.private_data = (void **)&pp;
	pp->controller = synopsys_pci.nr_controllers;

	pci_common_init(&synopsys_pci);
	pci_assign_unassigned_resources();
#ifdef CONFIG_PCI_DOMAINS
	synopsys_pci.domain++;
#endif

	platform_set_drvdata(pdev, pp);
#ifdef	DEBUG_TRACE
	dev_err(pp->dev, "synopsys_pcie_probe: End\n");
#endif
	return 0;

fail_bus_clk:
	clk_disable_unprepare(pp->bus_clk);
fail_clk:
	clk_disable_unprepare(pp->clk);
	return ret;
}

static int __exit synopsys_pcie_remove(struct platform_device *pdev)
{
	struct pcie_port *pp = platform_get_drvdata(pdev);
	clk_disable_unprepare(pp->bus_clk);
	clk_disable_unprepare(pp->clk);
	return 0;
}

static const struct of_device_id synopsys_pcie_of_match[] = {
	{ .compatible = "synopsys-pcie", },
	{},
};
//MODULE_DEVICE_TABLE(of, synopsys_pcie_of_match);

static	const struct platform_device_id pcie_id_table[]={
	{	"synopsys-pcie",	},
	{},
};

MODULE_DEVICE_TABLE( platform, pcie_id_table);

static struct platform_driver synopsys_pcie_driver = {
	.remove		= __exit_p(synopsys_pcie_remove),
	.probe		= synopsys_pcie_probe,
	.id_table	= pcie_id_table,
	.driver = {
		.name	= "synopsys-pcie",
		.owner	= THIS_MODULE,
//		.of_match_table = of_match_ptr(synopsys_pcie_of_match),
	},
};

static int synopsys_pcie_abort(unsigned long addr, unsigned int fsr, struct pt_regs *regs)
{
//	unsigned long pc = instruction_pointer(regs);
//	unsigned long instr = *(unsigned long *)pc;
#ifdef	DEBUG_TRACE
	printk(KERN_ERR "synopsys_pcie_abort: Start\n");
#endif
	WARN_ONCE(1, "pcie abort\n");

	/*
	 * If the instruction being executed was a read,
	 * make it look like it read all-ones.
	 */
#if 0	/* ToDo */
	if ((instr & 0x0c100000) == 0x04100000) {
		int reg = (instr >> 12) & 15;
		unsigned long val;

		if (instr & 0x00400000)
			val = 255;
		else
			val = -1;

		regs->uregs[reg] = val;
		regs->ARM_pc += 4;
		return 0;
	}

	if ((instr & 0x0e100090) == 0x00100090) {
		int reg = (instr >> 12) & 15;

		regs->uregs[reg] = -1;
		regs->ARM_pc += 4;
		return 0;
	}
#endif
#ifdef	DEBUG_TRACE
	printk(KERN_ERR "synopsys_pcie_abort: End\n");
#endif
	return 1;
}

/* Synopsys PCIe driver does not allow module unload */

static int __init pcie_init(void)
{
#ifdef	DEBUG_TRACE
	printk(KERN_ERR "pcie_init: Start\n");
#endif
	hook_fault_code(16 + 6, synopsys_pcie_abort, SIGBUS, 0, "imprecise external abort");

//	platform_driver_probe(&synopsys_pcie_driver, synopsys_pcie_probe);
	platform_driver_register(&synopsys_pcie_driver);
#ifdef	DEBUG_TRACE
	printk(KERN_ERR "pcie_init: End\n");
#endif
	return 0;
}
subsys_initcall(pcie_init);

MODULE_AUTHOR("M.Ohkuma <jg1.han@samsung.com>");
MODULE_DESCRIPTION("Synopsys PCIe host controller driver");
MODULE_LICENSE("GPL v2");
