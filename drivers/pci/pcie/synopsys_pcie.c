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
#include <linux/pci.h>
#include <linux/pci_regs.h>
#include <linux/platform_device.h>
#include <linux/resource.h>
#include <linux/signal.h>
#include <linux/slab.h>
#include <linux/types.h>
#include "synopsys_pcie.h"

#undef	DEBUG_TRACE
#undef	DEBUG_TRACE_IRQ
#undef	DEBUG_TRACE_CFG
#undef	DEBUG_RW1
#undef	PCIE_IRQ_USE
#undef	DMA_USE
#define MEM_SIZE_1M
#undef	PORT_ALL_RESET

#define	PCIE_PORT1	1
#define	PCIE_PORT2	2
#define	PCIE_PORT3	3

#define	PCIE_LANE_PHY_CONF_1	1	/* link1:l4:cont1:p0 */
#define	PCIE_LANE_PHY_CONF_2	2
#define	PCIE_LANE_PHY_CONF_3	3	/* l1:p1:cont1:p0 default ! */

static unsigned char   pcie_probe_end=0;

struct pcie_port_info {
	u32			io_size;
	u32			mem_size;
	struct resource		cfg;
	void __iomem		*va_cfg;
	int			irq;
	struct resource		io;
	struct resource		mem;
};

struct pcie_port {
	struct device		*dev;
	u8			controller;
	u8			root_bus_nr;
	void __iomem		*resetgen_base;
	void __iomem		*pciewrap_base;
	void __iomem		*pciegen3_base1;
	void __iomem		*pciegen3_base2;
	void __iomem		*pciegen3_base3;
	struct resource		resetgen;
	struct resource		pciewrap;
	struct resource		pciegen3[3];
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
#ifdef	PCIE_IRQ_USE
	int			irq_pciebus;
#endif
};

int	rc_num = 1;
int	ep_num = 2;
int	nu_num = 3;
int	bifur_num = 2;
static	void __iomem	*pcie_1_reg;
static	void __iomem	*pcie_wrap;

/*
 * CSR PCIe IP consists of Synopsys specific part and CSR
 * specific part. Only core block is a Synopsys designware part;
 * other parts are CSR specific.
 */

static struct hw_pci synopsys_pci;


#ifdef	DEBUG_RW1
static u32 inline synopsys_readl(void __iomem *offset)
{
	u32	val;
	val = readl(offset);
	printk(KERN_ERR "%s  : Addr=0x%8.8x Data=0x%8.8x\n", __FUNCTION__, offset, val);
	return val;
}
static u32 inline synopsys_readw(void __iomem *offset)
{
	u32	val;
	val = readw(offset);
	printk(KERN_ERR "%s  : Addr=0x%8.8x Data=0x%4.4x\n", __FUNCTION__, offset, val);
	return (val & 0x0000ffff);
}
static u32 inline synopsys_readb(void __iomem *offset)
{
	u32	val;
	val = readb(offset);
	printk(KERN_ERR "%s  : Addr=0x%8.8x Data=0x%2.2x\n", __FUNCTION__, offset, val);
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

#ifdef	DEBUG_TRACE
	dev_err(pp->dev, "%s : Start\n",__FUNCTION__);
#endif

	if (!pp) {
		dev_err(pp->dev, "%s : Error End\n",__FUNCTION__);
#ifdef	DEBUG_TRACE
		dev_err(pp->dev, "%s : Error End\n",__FUNCTION__);
#endif
		return 0;
	}
/* yamano resource debug */
//	sys->mem_offset = pp->mem.start - pp->config.mem_bus_addr;
//	pci_add_resource_offset(&sys->resources, &pp->mem, sys->mem_offset);
//	pci_add_resource_offset(&sys->resources, &pp->config[1].mem, sys->mem_offset);
	sys->mem_offset = 0x404000000ULL - 0x10000000ULL;
	sys->io_offset  = 0x410000000ULL - 0x10100000ULL;
	if(request_resource(&iomem_resource,&pp->config[0].io)){
		printk(KERN_ERR " iomem io resource reqest error \n");
	}
	pci_add_resource_offset(&sys->resources, &pp->config[0].io, sys->io_offset);
	if(request_resource(&iomem_resource,&pp->config[0].mem)){
		printk(KERN_ERR " iomem memory resource reqest error \n");
	}
	pci_add_resource_offset(&sys->resources, &pp->config[0].mem, sys->mem_offset);
	
#ifdef	DEBUG_TRACE
	dev_err(pp->dev, "%s : End\n",__FUNCTION__);
#endif
	return 1;
}

static int synopsys_pcie_link_up(struct pcie_port *pp)
{
	u32 val;
	int rtn = 0;
#ifdef	DEBUG_TRACE
	dev_err(pp->dev, "%s : Start\n",__FUNCTION__);
#endif
	
	val = synopsys_readl(pp->pciewrap_base + PCIE1_MISC_STAT);
	if (val == PCIE1_MISC_STAT__GDA_PAB_DL_UP__MASK) 
		rtn = 1;

#ifdef	DEBUG_TRACE
	dev_err(pp->dev, "%s : End(rtn=%d)\n",__FUNCTION__, rtn);
#endif
	return rtn;
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
	int	debugFlag = 0;
#ifdef	DEBUG_TRACE_CFG
	printk(KERN_ERR "####%s entry bus=%x DevFn=%x where=%x size=%d\n",__FUNCTION__, bus->number, devfn, where, size);
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
		debugFlag = 1;
	}
	if (size == 1)
		*val = synopsys_readb(pcieconf_base);
	else if (size == 2)
		*val = synopsys_readw(pcieconf_base);
	else
		*val = synopsys_readl(pcieconf_base);
//	dev_err(pp->dev, " 0x%8.8lx : 0x%8.8lx\n", pcieconf_base, val);
//out:
	if(debugFlag == 1){
//		dev_err(pp->dev, "root read access %x : %x\n",where, *val);
		if(where == 8)
			*val = 0x06040000;
	}
#ifdef	DEBUG_TRACE_CFG
	printk(KERN_ERR "####%s val=%x\n",__FUNCTION__, *val);
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

#ifdef	DEBUG_TRACE_CFG
	printk(KERN_ERR "####%s entry bus=%x DevFn=%x where=%x size=%d val=%x\n",__FUNCTION__, bus->number, devfn, where, size,val);
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
//		dev_err(pp->dev, "Root write access %x : %x",where,val);
	}
//	dev_err(pp->dev, " 0x%8.8lx : 0x%8.8x\n", pcieconf_base, val);
	if(size == 1)
		synopsys_writeb(pcieconf_base, val);
	else if(size == 2)
		synopsys_writew(pcieconf_base, val);
	else
		synopsys_writel(pcieconf_base, val);
//out:
#ifdef	DEBUG_TRACE_CFG
	printk(KERN_ERR "####%s End\n",__FUNCTION__);
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
	dev_err(pp->dev, "%s : Entry nr=%d bus=%d Start\n", __FUNCTION__, nr, sys->busnr);
#endif
	if (pp) {
		pp->root_bus_nr = sys->busnr;
		bus = pci_scan_root_bus(NULL, sys->busnr, &synopsys_pcie_ops, sys, &sys->resources);
	} else {
		bus = NULL;
		BUG();
	}
#ifdef	DEBUG_TRACE
	dev_err(pp->dev, "%s : Exit\n",__FUNCTION__);
#endif
	return bus;
}

static int synopsys_pcie_map_irq(const struct pci_dev *dev, u8 slot, u8 pin)
{
	struct pcie_port *pp = sys_to_pcie(dev->bus->sysdata);
#ifdef	DEBUG_TRACE
	dev_err(pp->dev, "%s : Entry(irq=%d)\n",__FUNCTION__,pp->irq);
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

#ifdef	DEBUG_TRACE
	dev_err(pp->dev, "%s : Entry\n",__FUNCTION__);
#endif
	
	regVal  = synopsys_readl(pciewrap_base + PCIE_PHY_RST_CTRL);
	regVal |= PCIE_PHY_RST_CTRL__PHY_RESET__MASK;
	synopsys_writel(pciewrap_base + PCIE_PHY_RST_CTRL, regVal);

#ifdef	DEBUG_TRACE
	dev_err(pp->dev, "%s : Exit\n",__FUNCTION__);
#endif
}

static void synopsys_pcie_assert_pipe_reset(struct pcie_port *pp)
{
	u32 regVal;
	void __iomem *pciewrap_base = pp->pciewrap_base;

#ifdef	DEBUG_TRACE
	dev_err(pp->dev, "%s : Entry\n",__FUNCTION__);
#endif
	regVal  = synopsys_readl(pciewrap_base + PCIE_PHY_RST_CTRL);
	regVal &= PCIE_PHY_RST_CTRL__PIPE0_RESET_N__INV_MASK;
#ifdef	PORT_ALL_RESET
	regVal &= PCIE_PHY_RST_CTRL__PIPE1_RESET_N__INV_MASK;
	regVal &= PCIE_PHY_RST_CTRL__PIPE2_RESET_N__INV_MASK;
#endif
	synopsys_writel(pciewrap_base + PCIE_PHY_RST_CTRL, regVal);

#ifdef	DEBUG_TRACE
	dev_err(pp->dev, "%s : Exit\n",__FUNCTION__);
#endif
}

static void synopsys_pcie_assert_gpex_reset(struct pcie_port *pp)
{
	u32 regVal;
	void __iomem *pciewrap_base = pp->pciewrap_base;
	
#ifdef	DEBUG_TRACE
	dev_err(pp->dev, "%s : Entry\n",__FUNCTION__);
#endif
	regVal  = synopsys_readl(pciewrap_base + PCIE1_SW_RST);
	regVal |= PCIE1_SW_RST__PERST_N__MASK;
	regVal &=(PCIE1_SW_RST__PAB_N__INV_MASK  &
		  PCIE1_SW_RST__AMBA_N__INV_MASK &
		  PCIE1_SW_RST__PBUS_N__INV_MASK &
		  PCIE1_SW_RST__LINK_N__INV_MASK);
	synopsys_writel(pciewrap_base + PCIE1_SW_RST, regVal);

#ifdef	PORT_ALL_RESET
	regVal  = synopsys_readl(pciewrap_base + PCIE2_SW_RST);
	regVal &=(PCIE2_SW_RST__PAB_N__INV_MASK  &
		  PCIE2_SW_RST__AMBA_N__INV_MASK &
		  PCIE2_SW_RST__PBUS_N__INV_MASK &
		  PCIE2_SW_RST__LINK_N__INV_MASK);
	synopsys_writel(pciewrap_base + PCIE2_SW_RST, regVal);

	regVal  = synopsys_readl(pciewrap_base + PCIE3_SW_RST);
	regVal &=(PCIE3_SW_RST__PAB_N__INV_MASK  &
		  PCIE3_SW_RST__AMBA_N__INV_MASK &
		  PCIE3_SW_RST__PBUS_N__INV_MASK &
		  PCIE3_SW_RST__LINK_N__INV_MASK);
	synopsys_writel(pciewrap_base + PCIE3_SW_RST, regVal);
#endif
#ifdef	DEBUG_TRACE
	dev_err(pp->dev, "%s : Exit\n",__FUNCTION__);
#endif
}

#if 1	/* ADD */
static void synopsys_pcie_ovlctl12(struct pcie_port *pp)
{
	u32 regVal;
	void __iomem *addr;
#ifdef	DEBUG_TRACE
	dev_err(pp->dev, "%s : Entry\n",__FUNCTION__);
#endif
	addr = ioremap_nocache(0x0409003c,4);
	regVal = readl(addr);
	regVal &= ~0x00000070;
	writel(regVal, addr);
	iounmap(addr);

	addr = ioremap_nocache(0x040800e0,4);	//PCIE_PIORESA
	writel((1<<12), addr);
	iounmap(addr);

	addr = ioremap_nocache(0x04080040,4);	//PCIE_PIODIRA
	regVal = readl(addr);
	regVal &= ~(1<<12);
	writel(regVal, addr);
	iounmap(addr);

#ifdef	DEBUG_TRACE
	dev_err(pp->dev, "%s : Exit\n",__FUNCTION__);
#endif
}
#endif

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
	regVal |= PCIE_PHY_RST_CTRL__PIPE0_RESET_N__MASK;
	synopsys_writel(pciewrap_base + PCIE_PHY_RST_CTRL, regVal);
}

static	int	synopsys_pcie_pipe_ok(struct pcie_port *pp, int which)
{
	u32	regVal;
	void __iomem *pciewrap_base = pp->pciewrap_base;
	int	timeout = 0;
	
	regVal  = synopsys_readl(pciewrap_base + PCIE_PHY_PIPE_STAT);
	switch(which) {
		case 1: while ((regVal & PCIE_PHY_PIPE_STAT__PIPE0_PHYSTATUS__MASK) == 0)
		        {
				if(timeout > 50)	
					return -1;
				timeout++;
				msleep(10);
				regVal = synopsys_readl(pciewrap_base + PCIE_PHY_PIPE_STAT);
		        }
		        break;
		case 2: while ((regVal &= PCIE_PHY_PIPE_STAT__PIPE1_PHYSTATUS__MASK) != 0)
		        {
				if(timeout > 50)	
					return -1;
				timeout++;
				msleep(10);
				regVal = synopsys_readl(pciewrap_base + PCIE_PHY_PIPE_STAT);
		        }
		        break;
		case 3: while ((regVal &= PCIE_PHY_PIPE_STAT__PIPE2_PHYSTATUS__MASK) != 0)
		        {
				if(timeout > 50)	
					return -1;
				timeout++;
				msleep(10);
				regVal = synopsys_readl(pciewrap_base + PCIE_PHY_PIPE_STAT);
		        }
		default:
			dev_err(pp->dev, "synopsys_pcie_pipe_ok: which is %d Error\n",which);
			return	-1;
			break;
	}
	return	0;
}

static void synopsys_pcie_deassert_gpex_reset(struct pcie_port *pp, int which, int assert)
{
	u32 regVal;
	void __iomem *pciewrap_base = pp->pciewrap_base;
	
	regVal  = synopsys_readl(pciewrap_base + PCIE1_SW_RST);
	regVal &= ~0x00000010;
	regVal |= (0x00000001|0x00000002|0x00000004|0x00000008);
	synopsys_writel(pciewrap_base + PCIE1_SW_RST, regVal);
}

static void synopsys_pcie_AxiToPexInit(struct pcie_port *pp)
{
	void __iomem *pciegen3_base1 = pp->pciegen3_base1;
	
	// PCIE1 has 512 MB AXI target space
#ifdef	DMA_USE
#else
	synopsys_writel(pciegen3_base1 + PCIE_PAB_AXI_PIO_CTRL0, ENABLE_ALL_ACCESS_TYPE|ENABLE_PORT);
#endif
			
	// - window 0
	//   - CFG access
	//   - 64 MB
	synopsys_writel(pciegen3_base1 + PCIE_PAB_AXI_AMAP_CTRL0,      PAB_AXI_AMAP_CTRL_64_CFG);
	synopsys_writel(pciegen3_base1 + PCIE_PAB_AXI_AMAP_AXI_BASE0,  0x00000000);
	synopsys_writel(pciegen3_base1 + PCIE_PAB_AXI_AMAP_AXI_BASE0X, 0x00000004);
	synopsys_writel(pciegen3_base1 + PCIE_PAB_AXI_AMAP_PEX_BASEL0, 0x00000000);
	synopsys_writel(pciegen3_base1 + PCIE_PAB_AXI_AMAP_PEX_BASEH0, 0x00000000);

	// - window 1
	//   - MEM access
	//   - 1 MB
	//   - axi side is not where dependent
	//   - pex side is where dependent
#ifdef	MEM_SIZE_1M /* (VxWorks:192k Linux:1MB) */
	synopsys_writel(pciegen3_base1 + PCIE_PAB_AXI_AMAP_CTRL1,      0x00100005);
#else
	synopsys_writel(pciegen3_base1 + PCIE_PAB_AXI_AMAP_CTRL1,      0x00030005);
#endif
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
}

/*
 * RAM Address setting 0x8_0500_0000 - 0x8_bfff_ffff
 */
static void synopsys_pcie_PexToAxiInitRc(struct pcie_port *pp)
{
	void __iomem *pciegen3_base1 = pp->pciegen3_base1;
	unsigned long	kernel_addr_l = 0x05000000;	// Vx:0x08000000 Li:0x05000000
	unsigned long	kernel_addr_u = 0x00000008;	
	unsigned long	pex_addr_u    = 0x05000000;	// Vx:0x08000000 Li:0x05000000
	unsigned long	pex_addr_l    = 0x00000008;	
	unsigned long	in_size       = 0x0ffffc00;	/* 255MB */
	unsigned long	wind_cmd;

	wind_cmd = in_size | 0x05;

#ifdef	DMA_USE
#else
	synopsys_writel(pciegen3_base1 + PCIE_PAB_PEX_PIO_CTRL0, ENABLE_PORT);
#endif

	// window 0
	synopsys_writel(pciegen3_base1 + PCIE_PAB_PEX_AMAP_CTRL0,      wind_cmd);
	synopsys_writel(pciegen3_base1 + PCIE_PAB_PEX_AMAP_AXI_BASE0,  kernel_addr_l);
	synopsys_writel(pciegen3_base1 + PCIE_PAB_PEX_AMAP_AXI_BASE0X, kernel_addr_u);
	synopsys_writel(pciegen3_base1 + PCIE_PAB_PEX_AMAP_PEX_BASEL0, pex_addr_u);
	synopsys_writel(pciegen3_base1 + PCIE_PAB_PEX_AMAP_PEX_BASEH0, pex_addr_l);
	kernel_addr_l += in_size;
	pex_addr_u    += in_size;

	// Windows 1
	synopsys_writel(pciegen3_base1 + PCIE_PAB_PEX_AMAP_CTRL1,      wind_cmd);
	synopsys_writel(pciegen3_base1 + PCIE_PAB_PEX_AMAP_AXI_BASE1,  kernel_addr_l);
	synopsys_writel(pciegen3_base1 + PCIE_PAB_PEX_AMAP_AXI_BASE1X, kernel_addr_u);
	synopsys_writel(pciegen3_base1 + PCIE_PAB_PEX_AMAP_PEX_BASEL1, pex_addr_u);
	synopsys_writel(pciegen3_base1 + PCIE_PAB_PEX_AMAP_PEX_BASEH1, pex_addr_l);
	kernel_addr_l += in_size;
	pex_addr_u    += in_size;

	// Window 2
	synopsys_writel(pciegen3_base1 + PCIE_PAB_PEX_AMAP_CTRL2,      wind_cmd);
	synopsys_writel(pciegen3_base1 + PCIE_PAB_PEX_AMAP_AXI_BASE2,  kernel_addr_l);
	synopsys_writel(pciegen3_base1 + PCIE_PAB_PEX_AMAP_AXI_BASE2X, kernel_addr_u);
	synopsys_writel(pciegen3_base1 + PCIE_PAB_PEX_AMAP_PEX_BASEL2, pex_addr_u);
	synopsys_writel(pciegen3_base1 + PCIE_PAB_PEX_AMAP_PEX_BASEH2, pex_addr_l);
	kernel_addr_l += in_size;
	pex_addr_u    += in_size;

	// Window 3
	synopsys_writel(pciegen3_base1 + PCIE_PAB_PEX_AMAP_CTRL3,      wind_cmd);
	synopsys_writel(pciegen3_base1 + PCIE_PAB_PEX_AMAP_AXI_BASE3,  kernel_addr_l);
	synopsys_writel(pciegen3_base1 + PCIE_PAB_PEX_AMAP_AXI_BASE3X, kernel_addr_u);
	synopsys_writel(pciegen3_base1 + PCIE_PAB_PEX_AMAP_PEX_BASEL3, pex_addr_u);
	synopsys_writel(pciegen3_base1 + PCIE_PAB_PEX_AMAP_PEX_BASEH3, pex_addr_l);
#ifdef	DEBUG_TRACE
{
	unsigned long	val_u;
	unsigned long	val_l;
	unsigned long	set_size;
	val_u = synopsys_readl(pciegen3_base1 + PCIE_PAB_PEX_AMAP_AXI_BASE0X);
	val_l = synopsys_readl(pciegen3_base1 + PCIE_PAB_PEX_AMAP_AXI_BASE0);
	set_size = synopsys_readl(pciegen3_base1 + PCIE_PAB_PEX_AMAP_CTRL0);
	set_size &= 0xfffffc00;
	printk( KERN_ERR "%s:PCIe Inbound window 0:0x%x_%08x - 0x%x_%08x\n", __func__, val_u, val_l, val_u, (val_l+set_size));
	val_u = synopsys_readl(pciegen3_base1 + PCIE_PAB_PEX_AMAP_AXI_BASE1X);
	val_l = synopsys_readl(pciegen3_base1 + PCIE_PAB_PEX_AMAP_AXI_BASE1);
	set_size = synopsys_readl(pciegen3_base1 + PCIE_PAB_PEX_AMAP_CTRL1);
	set_size &= 0xfffffc00;
	printk( KERN_ERR "%s:PCIe Inbound window 1:0x%x_%08x - 0x%x_%08x\n", __func__, val_u, val_l, val_u, (val_l+set_size));
	val_u = synopsys_readl(pciegen3_base1 + PCIE_PAB_PEX_AMAP_AXI_BASE2X);
	val_l = synopsys_readl(pciegen3_base1 + PCIE_PAB_PEX_AMAP_AXI_BASE2);
	set_size = synopsys_readl(pciegen3_base1 + PCIE_PAB_PEX_AMAP_CTRL2);
	set_size &= 0xfffffc00;
	printk( KERN_ERR "%s:PCIe Inbound window 2:0x%x_%08x - 0x%x_%08x\n", __func__, val_u, val_l, val_u, (val_l+set_size));
	val_u = synopsys_readl(pciegen3_base1 + PCIE_PAB_PEX_AMAP_AXI_BASE3X);
	val_l = synopsys_readl(pciegen3_base1 + PCIE_PAB_PEX_AMAP_AXI_BASE3);
	set_size = synopsys_readl(pciegen3_base1 + PCIE_PAB_PEX_AMAP_CTRL3);
	set_size &= 0xfffffc00;
	printk( KERN_ERR "%s:PCIe Inbound window 3:0x%x_%08x - 0x%x_%08x\n", __func__, val_u, val_l, val_u, (val_l+set_size));
}
#endif	/* DEBUG_TRACE */
}


static int synopsys_pcie_establish_link(struct pcie_port *pp)
{
	u32 val;
	void __iomem *pciewrap_base  = pp->pciewrap_base;
	void __iomem *pciegen3_base1 = pp->pciegen3_base1;
#ifdef	DEBUG_TRACE
	dev_err(pp->dev, "synopsys_pcie_establish_link: Start(rc_num=%d ep_num=%d nu_num=%d)\n",rc_num,ep_num,nu_num);
#endif
	if (synopsys_pcie_link_up(pp)) {
		dev_err(pp->dev, "synopsys_pcie_establish_link: Link already up\n");
		return 0;
	}
	/* Assert PCIe PHY Reset */
	synopsys_pcie_assert_phy_reset(pp);

	/* Assert PCIe Pipe Resets */
	synopsys_pcie_assert_pipe_reset(pp);
	
	/* Reset PAB, AMBA, PBUS, Link */
	synopsys_pcie_assert_gpex_reset(pp);
	
#if 1   /* ADD */
	/* Note: PERST_N control is currently broken, so use GPO12 to drive it */
	synopsys_pcie_ovlctl12(pp);
#endif

#if 1	/* Change */
	/* PCIe Endpoint 1 is Root Complex and using lanes 3:0 */
	synopsys_writel(pciewrap_base + PCIE_SW_BOOTSTRAP, 0x00000011);
#else
	/* Set sw_bootstrap Root Complex*/
	synopsys_pcie_set_bootstrap(pp, PCIE_PORT1, 1);
	synopsys_pcie_set_bootstrap(pp, PCIE_PORT2, 1);
	synopsys_pcie_set_bootstrap(pp, PCIE_PORT3, 1);
	/* Connetc lane */
	synopsys_pcie_pcie_bifur(pp, PCIE_LANE_PHY_CONF_3);
#endif
	
	/* Use on-chip reference clock (PHY_REF_ALT_CLK) */
	val  = synopsys_readl(pciewrap_base + PCIE_PHY_CLK_CTRL);
#if 1	/* A0/B0 Change */
	val |= PCIE_PHY_CLK_CTRL__PHY_REF_USE_PAD__MASK;
#else
	val &= ~PCIE_PHY_CLK_CTRL__PHY_REF_USE_PAD__MASK;
#endif
	synopsys_writel(pciewrap_base + PCIE_PHY_CLK_CTRL, val);

	/* Enable clock for Endpoint 1, but not the unused endpoints */
	val = synopsys_readl(pciewrap_base + PCIE_PHY_PIPE_CTRL);
	val &= PCIE_PHY_PIPE_CTRL__MAC0_PCLKREQ_N__INV_MASK; // MAC0 = PCIE1
	val |= (PCIE_PHY_PIPE_CTRL__MAC1_PCLKREQ | PCIE_PHY_PIPE_CTRL__MAC2_PCLKREQ);
	synopsys_writel(pciewrap_base + PCIE_PHY_PIPE_CTRL, val);
	
	/* De-assert PCIE PHY Resets */
	synopsys_pcie_deassert_phy_reset(pp);

	/* de-assert PIPE Reset */
	synopsys_pcie_deassert_pipe_reset(pp, PCIE_PORT1, 0);
	
	/* wait until pipe setup is complete */
	if(synopsys_pcie_pipe_ok(pp, PCIE_PORT1) < 0){
		printk( KERN_ERR "%s:PCIe Status error\n", __func__);
		return	-1;
	}
	
	/* Deassert GPEX Resets for RC endpoint 1 */
	synopsys_pcie_deassert_gpex_reset(pp, PCIE_PORT1, 0);
	
#if 1	/* ADD */
	{
        void __iomem *addr;
        addr = ioremap_nocache(0x040800c0,4);
        writel((1<<12), addr);
        iounmap(addr);
	}
	/* Generate 300MHz PCI clock from core clock */
#define PCIE_FREQ		(266*1000*1000)
#define PCIE_CLK_266MHZ		(PCIE_FREQ/(266*1000*1000/16))
#define PCIE_CORE_RESET_WAIT	200			/* 200 ms */
        synopsys_writel(pciegen3_base1+PCIE_GPEXD_CORE_CLK_RATIO, PCIE_CLK_266MHZ );
	
	//msleep( PCIE_CORE_RESET_WAIT );
	msleep( PCIE_CORE_RESET_WAIT * 10 );
#endif

#ifdef	DEBUG_TRACE
	dev_err(pp->dev, "synopsys_pcie_establish_link: End\n");
#endif
	return 0;
}

static void exynos_pcie_clear_irq_pulse(int irq_no)		// Check OK
{
//	u32     val;
#ifdef	DEBUG_TRACE_IRQ
	printk(KERN_ERR "PCIe:%s: Start\n",__func__);
#endif
//	val = synopsys_readl(pcie_1_reg + PCIE_PAB_AXI_INT_MISC_STAT);	
//#if 1
//	if ( val != 0x20 ) { 		// INTA
//		printk(KERN_ERR, "PCIe IRQ Status=0x%08x(see 0xc00)\n", val);
//	}
//#endif
//	if ( irq_no == 0 ){
//		/* clear interrupt at PAB (this register is w1c) */
//		if ( val ) {
//			synopsys_writel(pcie_1_reg + PCIE_PAB_AXI_INT_MISC_STAT, val);
//		}
//	}
//	if ( irq_no == 96 ) {
//		synopsys_writel(pcie_1_reg + PCIE_PAB_AXI_INT_MISC_STAT, 0x20);	//INTA
//	}
//
	/* clear interrupt at PCIe General (this register is w1c) */
	synopsys_writel(pcie_wrap + PCIE1_INT_CLR, PCIE1_INT_CLR__PERST_N_PIN | PCIE1_INT_CLR__GDA_PAB);
#ifdef	DEBUG_TRACE_IRQ
	printk(KERN_ERR "PCIe:%s: End\n", __func__);
#endif
}

#ifdef	PCIE_IRQ_USE
static irqreturn_t pcie_irq_handler(int irq, void *arg)		// Check OK
{
	printk(KERN_ERR "PCIe:%s: Call\n", __func__);
	synopsys_writel(pcie_wrap + PCIE_INT_CLR, PCIE_INT_CLR__PCIETP_WRAP);
	return IRQ_HANDLED;
}
#endif	/* PCIE_IRQ_USE */

static irqreturn_t exynos_pcie_irq_handler(int irq, void *arg)		// Check OK
{
//	struct pcie_port *pp = arg;

#ifdef	DEBUG_TRACE_IRQ
	printk(KERN_ERR "PCIe:%s: Start\n", __func__);
#endif
#ifdef	DEBUG_TRACE
	{
		u32     val;
		// 0x848
		val = synopsys_readl(pcie_1_reg + PCIE1_PAB_AXI_PIO_STAT0);	
		if ( val )
			printk(KERN_ERR "AXI PIO               Status Register=0x%x\n", val);

		val = synopsys_readl(pcie_1_reg + 0x84c);	
		if ( val )
			printk(KERN_ERR "AMBA PIO Slave        Status Register=0x%x\n", val);

		// 0x8e8
		val = synopsys_readl(pcie_1_reg + 0x8e8);	
		if ( val )
			printk(KERN_ERR "PEX PIO               Status Register=0x%x\n", val);
		synopsys_writel(pcie_1_reg + 0x8e8, val);

		val = synopsys_readl(pcie_1_reg + 0x8ec);	
		if ( val )
			printk(KERN_ERR "PEX PIO Master        Status Register=0x%x\n", val);
		val = synopsys_readl(pcie_1_reg + 0x984);	
		if ( val )
			printk(KERN_ERR "CSR PIO Slave         Status Register=0x%x\n", val);

		// 0xba8
		val = synopsys_readl(pcie_1_reg + PCIE1_PAB_PEX_INT_STAT);	
		if ( (val & 0xfffffffa) != 0 )
			printk(KERN_ERR "PEX Interrupt         Status Register=0x%x\n", val);
		synopsys_writel(pcie_1_reg + PCIE1_PAB_PEX_INT_STAT, val);
		
		// 0xbf4
		val = synopsys_readl(pcie_1_reg + 0xbf4);	
		if ( (val & 0xfffefffe) != 0 )
			printk(KERN_ERR "AMBA Interrupt PIO    Status Register=0x%x\n", val);
		synopsys_writel(pcie_1_reg + 0xbf4, val);


		val = synopsys_readl(pcie_1_reg + 0xbf8);	
		if ( val )
			printk(KERN_ERR "AMBA Interrupt WDMA   Status Register=0x%x\n", val);
		val = synopsys_readl(pcie_1_reg + 0xbfc);	
		if ( val )
			printk(KERN_ERR "AMBA Interrupt RDMA   Status Register=0x%x\n", val);
		val = synopsys_readl(pcie_1_reg + 0xc00);	
		if ( (val != 0x00) && (val != 0x20) )
			printk(KERN_ERR "AMBA Interrupt Misc   Status Register=0x%x\n", val);
		val = synopsys_readl(pcie_1_reg + 0xc64);	
		if ( val )
			printk(KERN_ERR "PEX Inbound MSI       Status Register=0x%x\n", val);
		val = synopsys_readl(pcie_1_reg + 0x990);	
		if ( val )
			printk(KERN_ERR "DMA Descriptor Master Status Register=0x%x\n", val);
		val = synopsys_readl(pcie_1_reg + 0x9a8);	
		if ( val )
			printk(KERN_ERR "WDMA Engine           Status Register=0x%x\n", val);
		val = synopsys_readl(pcie_1_reg + 0x9ac);	
		if ( val )
			printk(KERN_ERR "WDMA Engine Master    Status Register=0x%x\n", val);
		val = synopsys_readl(pcie_1_reg + 0xa48);	
		if ( val )
			printk(KERN_ERR "RDMA Engine           Status Register=0x%x\n", val);
		val = synopsys_readl(pcie_1_reg + 0xa4c);	
		if ( val )
			printk(KERN_ERR "RDMA Engine Master    Status Register=0x%x\n", val);
	}
#endif	/* DEBUG_TRACE */
	exynos_pcie_clear_irq_pulse(0);
#ifdef	DEBUG_TRACE_IRQ
	printk(KERN_ERR, "PCIe:%s: End\n");
#endif
	return IRQ_HANDLED;
}

/* PCIe Card Driver Interrupt helper function */
//int	synopsys_pcie_interrupt_clear(unsigned int irq_no)
//{
//#ifdef	DEBUG_TRACE_IRQ
//	printk(KERN_ERR, "PCIe:%s: Start(irq_no=0x%x)\n", irq_no);
//#endif
//	synopsys_writel(pcie_1_reg + PCIE_PAB_AXI_INT_MISC_STAT, irq_no);
//	synopsys_writel(pcie_wrap + PCIE_PCIE1_INT_CLR, PCIE_INT_GDA_PAB);
//	exynos_pcie_clear_irq_pulse(irq_no);
//#ifdef	DEBUG_TRACE_IRQ
//	printk(KERN_ERR, "PCIe:%s: End\n", irq_no);
//#endif
//	return	0;
//}
//EXPORT_SYMBOL(synopsys_pcie_interrupt_clear);

static int  synopsys_pcie_host_init(struct pcie_port *pp)
{
	u32 val;
	void __iomem *resetgen_base  = pp->resetgen_base;
	void __iomem *pciewrap_base  = pp->pciewrap_base;
	void __iomem *pciegen3_base1 = pp->pciegen3_base1;

#ifdef	DEBUG_TRACE
	dev_err(pp->dev, "synopsys_pcie_host_init: Start\n");
#endif

	/* PCIE core resets from RSTGEN default to asserted, deassert them now */
	val = synopsys_readl(resetgen_base + RSTGENSWRSTSTATIC10);
	val &= 0xff0fffff;
	synopsys_writel(resetgen_base + RSTGENSWRSTSTATIC10, val);

	/* enable link */
	if(synopsys_pcie_establish_link(pp) < 0 ){
		printk( KERN_ERR "%s:PCIe Link Establish Error\n", __func__);
		return	-1;
	}

#if 1	/* ADD */ 
	/* wait until link established */
	{
		int	i;
		u32 	val;
		for( i = 0; i < 10; i++ ){
			val = synopsys_readl(pciewrap_base + PCIE1_MISC_STAT);
			smp_wmb();
			if( (val & PCIE1_MISC_STAT__GDA_PAB_DL_UP__MASK) != 0 ){
				printk( KERN_ERR "PCIe Link 1 Establish\n");
				break;
			}
			msleep(10);
		}
		if ( i >= 10 ) {
			dev_err(pp->dev, "PCIe can't Data link Up\n");
			return  -1;
		}
	}
#endif

	/*-- setup PAB --*/
//	synopsys_writel(pciegen3_base1 + PCIE_PAB_CTRL, 0x0000022f);
	synopsys_writel(pciegen3_base1 + PCIE_PAB_CTRL, 0x0000023f);	// Maximum Burst Length is 16
	synopsys_pcie_AxiToPexInit(pp);
	synopsys_pcie_PexToAxiInitRc(pp);

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
	/* set config ready */
	val = synopsys_readl(pciegen3_base1 + PCIE_GPEXD_CFG_RDY);
	val |= 0x1;
	synopsys_writel(pciegen3_base1 + PCIE_GPEXD_CFG_RDY, val);

	/* locally initialize more PCIE1 RC CFG regs (45xx legacy code) */
	synopsys_writel(pciegen3_base1 + 0x10, 0x04000000);		//
	synopsys_writel(pciegen3_base1 + 0x14, 0x00000004);		//
	synopsys_writel(pciegen3_base1 + PCIE_GPEXP_CFG_BASE5_PMEMBASE, PCIE1_PMEM_LIMIT_BASE_L);
	synopsys_writel(pciegen3_base1 + PCIE_GPEXP_CFG_BASE5_PMEMBASE, 0x04100400);	// 0x0024 -> 0xFFFF0000
	synopsys_writel(pciegen3_base1 + PCIE_GPEXP_CFG_X_PBASEUDW, 0x4);		// 0x0028 -> 0x00002000 =>0x4
	synopsys_writel(pciegen3_base1 + PCIE_GPEXP_CFG_SUBVENID_PLIMITUDW, 0x4);	// 0x002C -> 0x000032FF =>0x4
	/* set bus number: Primory = 0x00, Secondary = 0x01, Subordinate = 0x01 */
	synopsys_writel(pciegen3_base1 + PCIE_GPEXP_CFG_BASE2_PRIBUS, 0x10100);		// 0x0018 -> 0x1i0100
	/* set snoop enable */
	val = synopsys_readl(pciegen3_base1 + 0x0054);
	val &= ~0x00000800;
	synopsys_writel(pciegen3_base1 + 0x0054, val);
	
	/* SET BITS in PCIE1 PCIE Command Register */
	val  = synopsys_readl(pciegen3_base1 + PCIE_GPEXP_CFG_COMMAND);
	val |= 0x6;
	synopsys_writel((pciegen3_base1 + PCIE_GPEXP_CFG_COMMAND), val);
	
	/* CHANGE PIOS_CONV_SEL to 01 */
//	val  = synopsys_readl(resetgen_base + PCIE1_MISC_CTRL);
//	val &= PCIE1_MISC_CTRL__PIOS_CONV_SEL__INV_MASK;
//	val |= (0x1 << PCIE1_MISC_CTRL__PIOS_CONV_SEL__SHIFT);
//	synopsys_writel(resetgen_base + PCIE1_MISC_CTRL, val);
//
	/* Interrupt clear reg base */
	pcie_wrap  = pp->pciewrap_base;
	pcie_1_reg = pp->pciegen3_base1;

	/* host bridge interrupt routing enable */
	val = synopsys_readl(pciewrap_base +PCIE1_INT_EN);
	val |= PCIE1_INT_EN__GDA_PAB;
	synopsys_writel(pciewrap_base + PCIE1_INT_EN, val);
#ifdef	PCIE_IRQ_USE
	val = synopsys_readl(pciewrap_base + PCIE_INT_EN);
	val |= PCIE_INT_EN__PCIETP_WRAP_rerror;
	synopsys_writel(pciewrap_base + PCIE1_INT_EN, val);
//	val = synopsys_readl(pciewrap_base + 0xbe0);
//	val |= 0x01100100;
//	synopsys_writel(pciewrap_base + 0xbe0, val);
#endif

	val = synopsys_readl(pciegen3_base1 + PCIE_PAB_AXI_INT_MISC_EN);
#ifdef	PCIE_IRQ_USE
	val |= PCIE_AXI_INT_INTA | 0x200003f0;
#else
	val |= PCIE_AXI_INT_INTA;
#endif
	synopsys_writel(pciegen3_base1 + PCIE_PAB_AXI_INT_MISC_EN, val);

#ifdef  PCIE_IRQ_USE
	val = synopsys_readl(pciegen3_base1 + PCIE1_PAB_PEX_INT_EN);
	val |= 0x3fa;
	synopsys_writel(pciegen3_base1 + PCIE1_PAB_PEX_INT_EN, val);
#endif	/* PCIE_IRQ_USE */

#ifdef	DEBUG_TRACE
	dev_err(pp->dev, "synopsys_pcie_host_init: End(pciegen3_base1=0x%x)\n",pciegen3_base1);
#endif
	return	0;
}

static int synopsys_add_pcie_port(struct pcie_port *pp, struct platform_device *pdev)
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
#ifdef	DEBUG_TRACE
	dev_err(pp->dev, "reset gen: 0x04010000->(v)0x%x\n",pp->resetgen_base);
#endif
	
	
	tmp = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!tmp) {
		dev_err(pp->dev, "add_pcie_port: couldn't get pciewrap base resource\n");
		return -EINVAL;
	}
	pp->pciewrap_base = devm_ioremap_resource(&pdev->dev, tmp);
	if (IS_ERR(pp->pciewrap_base))
		return PTR_ERR(pp->pciewrap_base);
#ifdef	DEBUG_TRACE
	dev_err(pp->dev, "warp     : 0x04a70000->(v)0x%x\n",pp->pciewrap_base);
#endif
	
	
	tmp = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	if (!tmp) {
		dev_err(pp->dev, "add_pcie_port: couldn't get pciegen3_1 base resource\n");
		return -EINVAL;
	}
	pp->pciegen3_base1 = devm_ioremap_resource(&pdev->dev, tmp);
	if (IS_ERR(pp->pciegen3_base1))
		return PTR_ERR(pp->pciegen3_base1);
#ifdef	DEBUG_TRACE
	dev_err(pp->dev, "port1    : 0x04a40000->(v)0x%x\n",pp->pciegen3_base1);
#endif
	
	pp->irq = platform_get_irq(pdev, 0);
	if (!pp->irq) {
		dev_err(pp->dev, "add_pcie_port: failed to get irq\n");
		return -ENODEV;
	}
	ret = devm_request_irq(&pdev->dev, pp->irq, exynos_pcie_irq_handler, IRQF_SHARED, "synopsys-pcie", pp);
	if (ret) {
		dev_err(pp->dev, "add_pcie_port: failed to request irq(PCIE)\n");
		return ret;
	}
#ifdef	PCIE_IRQ_USE
	pp->irq_pciebus = 95;
	ret = devm_request_irq(&pdev->dev, pp->irq_pciebus, pcie_irq_handler, IRQF_SHARED, "pcie_bus", pp);
	if (ret) {
		dev_err(pp->dev, "add_pcie_port: failed to request irq(PCIE_PORT1)\n");
		return ret;
	}
#endif
	
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
#ifdef	DEBUG_TRACE
	dev_info(&pdev->dev, "synopsys_pcie_probe :Start\n");
#endif
	int ret;

	pp = devm_kzalloc(&pdev->dev, sizeof(*pp), GFP_KERNEL);
	if (!pp) {
		dev_err(pp->dev, "synopsys_pcie_probe: no memory for pcie port\n");
		return -ENOMEM;
	}

	pp->dev = &pdev->dev;

	/* Configureation resource */
	pp->io.name	= "Multiport";
	pp->io.start	= 0x410000000ULL;
	pp->io.end	= 0x41000ffffULL;
	pp->io.flags	= IORESOURCE_IO;

	pp->config[0].io.name = "Port 0 IO space";
	pp->config[0].io.start = 0x410000000ULL;
	pp->config[0].io.end   = 0x41000FFFFULL;
	pp->config[0].io.flags = IORESOURCE_IO;
	pp->config[0].io_size = resource_size(&pp->config[0].io);

	pp->mem.name	= "Memory";
	pp->mem.start	= 0x404000000ULL;
#ifdef	MEM_SIZE_1M
	pp->mem.end	= 0x4040fffffULL;
#else
	pp->mem.end	= 0x40402ffffULL;
#endif
	pp->mem.flags	= IORESOURCE_MEM;
	pp->va_cfg = ioremap(0x400000000ULL,SZ_32M);

	pp->config[0].mem.name = "Port 0 Memory";
	pp->config[0].mem.start = 0x404000000ULL;
#ifdef	MEM_SIZE_1M
	pp->config[0].mem.end  	= 0x4040fffffULL;
#else
	pp->config[0].mem.end  	= 0x40402ffffULL;
#endif
	pp->config[0].mem.flags = IORESOURCE_MEM;
	pp->config[0].mem_size = resource_size(&pp->config[0].mem);

	pp->config[0].irq = LM2_IRQ_PCIE1;	/* device interrupt by port */

	ret = synopsys_add_pcie_port(pp, pdev);
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
	pcie_probe_end=1;
#ifdef	DEBUG_TRACE
	dev_err(pp->dev, "synopsys_pcie_probe: End\n");
#endif
	return 0;

fail_bus_clk:
	clk_disable_unprepare(pp->bus_clk);
//fail_clk:
	clk_disable_unprepare(pp->clk);
	return ret;
}

static int __exit synopsys_pcie_remove(struct platform_device *pdev)
{
	struct pcie_port *pp = platform_get_drvdata(pdev);
	clk_disable_unprepare(pp->bus_clk);
	clk_disable_unprepare(pp->clk);
	pcie_probe_end=0;
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

#ifdef	CONFIG_ARCH_LM2
#define LM2_REGBAK_SIZE 280
static unsigned int	reg_bak[LM2_REGBAK_SIZE];
static unsigned int	reg_bak_chksum;
extern unsigned int	chksum_info;
void pcie_reg_save(void __iomem *base, int *bak_adr, int offset, int size)
{
	int i;
	int adr = *bak_adr;

	for(i=adr; i<(adr+size); i++ ) {
		reg_bak[i] = readl(base + offset);
		offset +=4;
	}
	*bak_adr = i;
}

void pcie_reg_load(void __iomem *base, int *bak_adr, int offset, int size)
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

void lm2_pcie_suspend(void)
{
	int i=0;
	void __iomem *base;
	/* reset gen (  1)*/
	if ( pcie_probe_end == 1 ) {
	base = ioremap_nocache(0x04010000, 0x120);
	pcie_reg_save(base, &i, 0x104,  1);
	iounmap(base);

	/* warp ( 31)*/
	base = ioremap_nocache(0x04a70000, 0x140);
	pcie_reg_save(base, &i, 0x00c,  8);
	pcie_reg_save(base, &i, 0x03c,  2);
	pcie_reg_save(base, &i, 0x05c,  8);
	pcie_reg_save(base, &i, 0x100, 13);
	iounmap(base);

	/* port 1 (219)*/
	base = ioremap_nocache(0x04a40000, 0x1000);
	pcie_reg_save(base, &i, 0x000, 15);
	pcie_reg_save(base, &i, 0x044, 11);
	pcie_reg_save(base, &i, 0x07c,  1);
	pcie_reg_save(base, &i, 0x088,  4);
	pcie_reg_save(base, &i, 0x100, 21);
	pcie_reg_save(base, &i, 0x400, 10);
	pcie_reg_save(base, &i, 0x434,  4);
	pcie_reg_save(base, &i, 0x450,  3);
	pcie_reg_save(base, &i, 0x468,  6);
	pcie_reg_save(base, &i, 0x488,  1);
	pcie_reg_save(base, &i, 0x490,  1);
	pcie_reg_save(base, &i, 0x4ac,  2);
	pcie_reg_save(base, &i, 0x4b8,  2);
	pcie_reg_save(base, &i, 0x4c8, 17);
	pcie_reg_save(base, &i, 0x510,  3);
	pcie_reg_save(base, &i, 0x53c,  1);
	pcie_reg_save(base, &i, 0x544,  1);
	pcie_reg_save(base, &i, 0x54c,  1);
	pcie_reg_save(base, &i, 0x55c,  1);
	pcie_reg_save(base, &i, 0x594,  2);
	pcie_reg_save(base, &i, 0x5f0,  2);
	pcie_reg_save(base, &i, 0x620,  5);
	pcie_reg_save(base, &i, 0x640,  1);
	pcie_reg_save(base, &i, 0x6fc,  1);
	pcie_reg_save(base, &i, 0x800,  4);
	pcie_reg_save(base, &i, 0x814,  1);
	pcie_reg_save(base, &i, 0x81c,  1);
	pcie_reg_save(base, &i, 0x820,  1);
	pcie_reg_save(base, &i, 0x844,  2);
	pcie_reg_save(base, &i, 0x8e4,  3);
	pcie_reg_save(base, &i, 0x984,  1);
	pcie_reg_save(base, &i, 0x990,  1);
	pcie_reg_save(base, &i, 0x9a0,  4);
	pcie_reg_save(base, &i, 0xa40,  4);
	pcie_reg_save(base, &i, 0xae0,  8);
	pcie_reg_save(base, &i, 0xb24,  2);
	pcie_reg_save(base, &i, 0xb64,  2);
	pcie_reg_save(base, &i, 0xba4,  3);
	pcie_reg_save(base, &i, 0xbc4,  1);
	pcie_reg_save(base, &i, 0xbcc,  1);
	pcie_reg_save(base, &i, 0xbd8,  6);
	pcie_reg_save(base, &i, 0xbf4,  4);
	pcie_reg_save(base, &i, 0xc64,  6);
	pcie_reg_save(base, &i, 0xc84,  3);
	pcie_reg_save(base, &i, 0xca4, 19);
	pcie_reg_save(base, &i, 0xea0, 16);
	pcie_reg_save(base, &i, 0xf00,  4);
	pcie_reg_save(base, &i, 0xf40,  3);
	pcie_reg_save(base, &i, 0xf80,  3);
	iounmap(base);

	/* chksum gen */
	reg_bak_chksum=0;
	for(i=0; i<LM2_REGBAK_SIZE; i++)
		reg_bak_chksum += reg_bak[i];
	}

}
EXPORT_SYMBOL(lm2_pcie_suspend);

void lm2_pcie_resume(struct device *dev)
{
	int i=0;
	void __iomem *base;
	unsigned int    tmp;

	if ( pcie_probe_end == 1) {
	/* chksum chk */
	tmp=0;
	for(i=0; i<LM2_REGBAK_SIZE; i++)
		tmp += reg_bak[i];
	if ( tmp != reg_bak_chksum ){
		chksum_info |= 0x80;
	}

	i=0;
	/* reset gen (  1)*/
	base = ioremap_nocache(0x04010000, 0x120);
	pcie_reg_load(base, &i, 0x104,  1);
	iounmap(base);

	/* warp ( 31)*/
	base = ioremap_nocache(0x04a70000, 0x140);
	pcie_reg_load(base, &i, 0x00c,  8);
	pcie_reg_load(base, &i, 0x03c,  2);
	pcie_reg_load(base, &i, 0x05c,  8);
	pcie_reg_load(base, &i, 0x100, 13);
	iounmap(base);

	/* port 1 (219)*/
	base = ioremap_nocache(0x04a40000, 0x1000);
	pcie_reg_load(base, &i, 0x000, 15);
	pcie_reg_load(base, &i, 0x044, 11);
	pcie_reg_load(base, &i, 0x07c,  1);
	pcie_reg_load(base, &i, 0x088,  4);
	pcie_reg_load(base, &i, 0x100, 21);
	pcie_reg_load(base, &i, 0x400, 10);
	pcie_reg_load(base, &i, 0x434,  4);
	pcie_reg_load(base, &i, 0x450,  3);
	pcie_reg_load(base, &i, 0x468,  6);
	pcie_reg_load(base, &i, 0x488,  1);
	pcie_reg_load(base, &i, 0x490,  1);
	pcie_reg_load(base, &i, 0x4ac,  2);
	pcie_reg_load(base, &i, 0x4b8,  2);
	pcie_reg_load(base, &i, 0x4c8, 17);
	pcie_reg_load(base, &i, 0x510,  3);
	pcie_reg_load(base, &i, 0x53c,  1);
	pcie_reg_load(base, &i, 0x544,  1);
	pcie_reg_load(base, &i, 0x54c,  1);
	pcie_reg_load(base, &i, 0x55c,  1);
	pcie_reg_load(base, &i, 0x594,  2);
	pcie_reg_load(base, &i, 0x5f0,  2);
	pcie_reg_load(base, &i, 0x620,  5);
	pcie_reg_load(base, &i, 0x640,  1);
	pcie_reg_load(base, &i, 0x6fc,  1);
	pcie_reg_load(base, &i, 0x800,  4);
	pcie_reg_load(base, &i, 0x814,  1);
	pcie_reg_load(base, &i, 0x81c,  1);
	pcie_reg_load(base, &i, 0x820,  1);
	pcie_reg_load(base, &i, 0x844,  2);
	pcie_reg_load(base, &i, 0x8e4,  3);
	pcie_reg_load(base, &i, 0x984,  1);
	pcie_reg_load(base, &i, 0x990,  1);
	pcie_reg_load(base, &i, 0x9a0,  4);
	pcie_reg_load(base, &i, 0xa40,  4);
	pcie_reg_load(base, &i, 0xae0,  8);
	pcie_reg_load(base, &i, 0xb24,  2);
	pcie_reg_load(base, &i, 0xb64,  2);
	pcie_reg_load(base, &i, 0xba4,  3);
	pcie_reg_load(base, &i, 0xbc4,  1);
	pcie_reg_load(base, &i, 0xbcc,  1);
	pcie_reg_load(base, &i, 0xbd8,  6);
	pcie_reg_load(base, &i, 0xbf4,  4);
	pcie_reg_load(base, &i, 0xc64,  6);
	pcie_reg_load(base, &i, 0xc84,  3);
	pcie_reg_load(base, &i, 0xca4, 19);
	pcie_reg_load(base, &i, 0xea0, 16);
	pcie_reg_load(base, &i, 0xf00,  4);
	pcie_reg_load(base, &i, 0xf40,  3);
	pcie_reg_load(base, &i, 0xf80,  3);
	iounmap(base);

	}
}
EXPORT_SYMBOL(lm2_pcie_resume);

#endif	/* CONFIG_ARCH_LM2 */

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
