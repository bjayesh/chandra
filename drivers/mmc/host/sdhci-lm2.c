/*
 * SDHCI support for FujiXerox LM2 Waikiki Motherboard Support
 * Copyright (c) 2013-2014 Wind River Systems, Inc
 * Koki Yamano < koki.yamano@windriver.com >
 * This file is released under the GPLv2
 *
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/mmc/host.h>
#include <linux/module.h>
#include "sdhci-pltfm.h"

#define LM2_GPFSYS_BASE         0x04050000
#define LM2_OVLSYS_BASE         0x04090000
#define LM2_SDIO0_BASE          0x04440000
#define LM2_SDIO1_BASE          0x04450000

static unsigned int sdhci_lm2_get_max_clk(struct sdhci_host *host)
{
	return 177780000/2;	/* 177.78MHz/2 */
}

static void sdhci_lm2_set_clock(struct sdhci_host *host, unsigned int clock)
{
	struct device *dev = mmc_dev(host->mmc);
	int div = 1;
	u16 clk=0;
	unsigned long timeout;

	if (clock == host->clock)
		return;

	sdhci_writew(host, 0, SDHCI_CLOCK_CONTROL);

	if (clock == 0)
		return;

	host->clock = clock;

	dev_dbg(dev, "baseClk: %dHz, targetClk: %dHz.\n",host->max_clk, host->clock);
	sdhci_writew(host, 0x000e, SDHCI_TIMEOUT_CONTROL);
	if (host->clock >= host->max_clk) {
		div = 0x00;
	} else {
		div = host->max_clk / ( 2 * host->clock);
		if ( host->max_clk % ( 2 * host->clock )) {
			++div;
		}
	}
	
	clk  = (div&0x00ff)<<8 | (div&0x0300)>>2;
	clk |= SDHCI_CLOCK_INT_EN;		/* 0x01 */
	sdhci_writew(host, clk, SDHCI_CLOCK_CONTROL);

	timeout = 120;
	while (!((clk = sdhci_readw(host, SDHCI_CLOCK_CONTROL))
			& SDHCI_CLOCK_INT_STABLE)) {
		if (timeout == 0) {
			dev_warn(dev, "clock is unstable");
			break;
		}
		timeout--;
		mdelay(1);
	}

	clk |= SDHCI_CLOCK_CARD_EN;		/* 0x04 */
	sdhci_writew(host, clk, SDHCI_CLOCK_CONTROL);
	dev_dbg(dev, "SD clock: %dHz, div: 0x%x 0x%02x->%04x write.\n",clock, div, SDHCI_CLOCK_CONTROL,clk);

	mdelay(2);
}

static int sdhci_lm2_buswidth(struct sdhci_host *host, int bus_width)
{
	struct device *dev = mmc_dev(host->mmc);
        u32 ctrl;

        ctrl = sdhci_readb(host, SDHCI_HOST_CONTROL);
	if ((host->mmc->caps & MMC_CAP_8_BIT_DATA) &&
	    (bus_width == MMC_BUS_WIDTH_8)) {
                ctrl &= ~SDHCI_CTRL_4BITBUS;
                ctrl |= SDHCI_CTRL_8BITBUS;
		dev_dbg(dev, "buswidth: 8bit\n");
	} else {
		ctrl &= ~SDHCI_CTRL_8BITBUS;
		if (bus_width == MMC_BUS_WIDTH_4) {
			ctrl |= SDHCI_CTRL_4BITBUS;
			dev_dbg(dev, "buswidth: 4bit\n");
		} else {
			ctrl &= ~SDHCI_CTRL_4BITBUS;
			dev_dbg(dev, "buswidth: 1bit\n");
		}
	}
        sdhci_writeb(host, ctrl, SDHCI_HOST_CONTROL);
        return 0;
}
static void sdhci_lm2_platform_init(struct sdhci_host *host)
{
	void __iomem *virt_addr;
	u32     val;

	/***************/
	/* SDIO 0 init */
	/***************/

	/* GPF-SYS(0x288:SDIOPWRCTRL) */
	virt_addr = ioremap(LM2_GPFSYS_BASE + 0x280 ,0x10);
	val  = readl(virt_addr + 0x8);
	val  = (val & 0xffffff8e);
	val |= 0x0071;				/* SDIO0S0_PWR_EN */
	writel(val, virt_addr + 0x8);		/* SDIO Low Dropout Regulator Output : */
	iounmap(virt_addr);

	/*  OVL-SYS(0x24:SYS_OVLCTL6) */
	virt_addr = ioremap(LM2_OVLSYS_BASE, 0x30);
	val  = readl(virt_addr + 0x24);
	val  = (val & 0x88ffffff);
	val |= 0x11000000;			/* Input is "SD0S0CMDI"; output is "SD0S0CMDO" */
						/* Output is "SD0S0CLK" */
	writel(val, virt_addr + 0x24);		/* SDIO Low Dropout Regulator Output : */

	/*  OVL-SYS(0x28:SYS_OVLCTL7) */
	val  = 0x00441111;
	writel(val, virt_addr + 0x28);		/* SDIO Low Dropout Regulator Output : */
	iounmap(virt_addr);

	/* SDIO0 HRS2 */
	virt_addr = ioremap(LM2_SDIO0_BASE + 0x08,0x4);
	writel(0x00000004, virt_addr);		/* SDIO0_HRS2  DMA Burst=4  */
	iounmap(virt_addr);

	/* GPF-SYS(0x280:SDIO0_EXTCTL) */
	virt_addr = ioremap(LM2_GPFSYS_BASE + 0x280 ,0x10);
	val = readl(virt_addr + 0x0);
	val = (val & 0xfffffff3) | 0x4;
	writel(val, virt_addr + 0x0);
	iounmap(virt_addr);

	/* GPF-SYS(0x1B4:SDIO0ADBCTL) */
	virt_addr = ioremap(LM2_GPFSYS_BASE + 0x1b4 ,0x10);
	writel(0x00000081, virt_addr + 0x0);	/* SDIO0ADB Control register */
	iounmap(virt_addr);

	/***************/
	/* SDIO 1 init */
	/***************/

	/* GPF-SYS(0x288) */
	virt_addr = ioremap(LM2_GPFSYS_BASE + 0x280 ,0x10);
	val  = readl(virt_addr + 0x8);
	val  = (val & 0xffff8eff);
	val |= 0x3100;				/* SDIO1S0_TUNE1P8=3 SDIO1S0_PWR_EN=1 */
	writel(val, virt_addr + 0x8);		/* SDIO Low Dropout Regulator Output : */
	iounmap(virt_addr);

	/*  OVL-SYS(0x24:SYS_OVLCTL8) */
	virt_addr = ioremap(LM2_OVLSYS_BASE, 0x30);
	val  = readl(virt_addr + 0x2c);
	val  = (val & 0x88888888);
	val |= 0x00000022;			/* Input is "SD0S1CMDI"; output is "SD0S1CMDO" */
						/* Output is "SD0S1CLK" */
	val |= 0x11222200;
	writel(val, virt_addr + 0x2c);		/* SDIO Low Dropout Regulator Output : */


	/* SDIO1 HRS2 */
	virt_addr = ioremap(LM2_SDIO1_BASE + 0x08,0x4);
	writel(0x00000004, virt_addr);		/* SDIO1_HRS2  DMA Burst=4  */
	pr_err("SDIO1_HRS2 0x8: 0x%08x\n",readl(virt_addr));
	iounmap(virt_addr);

	/* GPF-SYS(0x284) */
	virt_addr = ioremap(LM2_GPFSYS_BASE + 0x280 ,0x10);
	val = readl(virt_addr + 0x4);
	val = (val & 0xfffffff3) | 0x4;
	writel(val, virt_addr + 0x4);
	iounmap(virt_addr);

	/* GPF-SYS(0x1B8:SDIO0ADBCTL) */
	virt_addr = ioremap(LM2_GPFSYS_BASE + 0x1b8 ,0x10);
	writel(0x00000081, virt_addr + 0x4);	/* SDIO0ADB Control register */
	iounmap(virt_addr);


}
static u8 sdhci_lm2_read_b(struct sdhci_host *host, int reg)
{
	u16 res;
	u8  rtn;
	volatile u16 *ptr;
	if ( (reg&0x01) == 0 ) {
		ptr = host->ioaddr + reg;
		res = readw(ptr);
		rtn = (u8)(res&0x00ff);
	} else {
		ptr = host->ioaddr + (reg - 1);
		res = readw(ptr);
		rtn = (u8)((res&0xff00)>>8);
	}
	return rtn;
}
static u16 sdhci_lm2_read_w(struct sdhci_host *host, int reg)
{
	volatile u16 *ptr = host->ioaddr + reg;
	return readw(ptr);
}
static u32 sdhci_lm2_read_l(struct sdhci_host *host, int reg)
{
	volatile u32 *ptr = host->ioaddr + reg;
	return readl(ptr);
}
static void sdhci_lm2_write_b(struct sdhci_host *host, u8 val, int reg)
{
	u16 res;
	u16 wdt;
	volatile u16 *ptr;
	if ( (reg&0x01) == 0 ) {
		ptr = host->ioaddr + reg;
		res = readw(ptr);
		wdt = (res&0xff00) | val;
	} else {
		ptr = host->ioaddr + (reg - 1);
		res = readw(ptr);
		wdt = (res&0x00ff) | (val<<8);
	}
	writew(wdt, ptr);
	barrier();
}
static void sdhci_lm2_write_w(struct sdhci_host *host, u16 val, int reg)
{
	volatile u16 *ptr = host->ioaddr + reg;
	writew(val, ptr);
	barrier();
}
static void sdhci_lm2_write_l(struct sdhci_host *host, u32 val, int reg)
{
	volatile u32 *ptr = host->ioaddr + reg;
	writel(val, ptr);
	barrier();
}

static const struct sdhci_ops sdhci_lm2_ops = {
	.get_max_clock	    = sdhci_lm2_get_max_clk,
	.set_clock	    = sdhci_lm2_set_clock,
	.platform_bus_width = sdhci_lm2_buswidth,
	.platform_init      = sdhci_lm2_platform_init,
	.read_b             = sdhci_lm2_read_b,
	.read_w             = sdhci_lm2_read_w,
	.read_l             = sdhci_lm2_read_l,
	.write_b            = sdhci_lm2_write_b,
	.write_w            = sdhci_lm2_write_w,
	.write_l            = sdhci_lm2_write_l,
};

static const struct sdhci_pltfm_data sdhci_lm2_pdata = {
	.ops = &sdhci_lm2_ops,
	.quirks = SDHCI_QUIRK_CAP_CLOCK_BASE_BROKEN |
		  SDHCI_QUIRK_NONSTANDARD_CLOCK |
		  SDHCI_QUIRK_DATA_TIMEOUT_USES_SDCLK,
};

static int sdhci_lm2_probe(struct platform_device *pdev)
{
	return sdhci_pltfm_register(pdev, &sdhci_lm2_pdata);
}

static int sdhci_lm2_remove(struct platform_device *pdev)
{
	return sdhci_pltfm_unregister(pdev);
}

static struct platform_driver sdhci_lm2_driver = {
	.driver		= {
		.name	= "sdhci-lm2",
		.owner	= THIS_MODULE,
		.pm	= SDHCI_PLTFM_PMOPS,
	},
	.probe		= sdhci_lm2_probe,
	.remove		= sdhci_lm2_remove,
};

module_platform_driver(sdhci_lm2_driver);

MODULE_DESCRIPTION("SDHCI driver for LM2");
MODULE_AUTHOR("Koki Yamano < koki.yamano@windriver.com >");
MODULE_LICENSE("GPLv2");
