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

static unsigned int sdhci_lm2_get_max_clk(struct sdhci_host *host)
{
	return 89000000;	/* SDMCLK=89MHz */
}

static void sdhci_lm2_set_clock(struct sdhci_host *host, unsigned int clock)
{
	struct device *dev = mmc_dev(host->mmc);
	int div = 1;
	u16 clk=0;
	unsigned long timeout;
	unsigned int get_clk;

	if (clock == host->clock)
		return;

	sdhci_writew(host, 0, SDHCI_CLOCK_CONTROL);

	if (clock == 0)
		return;

	host->clock = clock;

#if 0
	if (host->max_clk / div > clock) {
		div = 0x01;
	} else {
		div = 0x00;
	}
	dev_dbg(dev, "desired SD clock: %d, actual: %d\n",clock, host->max_clk / div);
	
	clk  = div<<SDHCI_DIVIDER_SHIFT;	/* SDCLKFS=0x001 89MHz/2=45MHz*/
#endif
	clk  = SDHCI_PROG_CLOCK_MODE;
	clk |= SDHCI_CLOCK_INT_EN;		/* 0x01 */
	sdhci_writew(host, clk, SDHCI_CLOCK_CONTROL);

	timeout = 20;
	while (!((clk = sdhci_readw(host, SDHCI_CLOCK_CONTROL))
			& SDHCI_CLOCK_INT_STABLE)) {
		if (timeout == 0) {
			dev_warn(dev, "clock is unstable");
			break;
		}
		timeout--;
		mdelay(1);
	}

	clk |= SDHCI_CLOCK_CARD_EN;	/* 0x04 */
	sdhci_writew(host, clk, SDHCI_CLOCK_CONTROL);
}

static const struct sdhci_ops sdhci_lm2_ops = {
	.get_max_clock	= sdhci_lm2_get_max_clk,
	.set_clock	= sdhci_lm2_set_clock,
};

static const struct sdhci_pltfm_data sdhci_lm2_pdata = {
	.ops = &sdhci_lm2_ops,
	.quirks = SDHCI_QUIRK_CAP_CLOCK_BASE_BROKEN |
		  SDHCI_QUIRK_NONSTANDARD_CLOCK,
//	.quirks = SDHCI_QUIRK_BROKEN_DMA |
//		  SDHCI_QUIRK_DATA_TIMEOUT_USES_SDCLK |
//		  SDHCI_QUIRK_INVERTED_WRITE_PROTECT |
//		  SDHCI_QUIRK_CAP_CLOCK_BASE_BROKEN,
//		  SDHCI_QUIRK_BROKEN_TIMEOUT_VAL |
//		  SDHCI_QUIRK_NONSTANDARD_CLOCK,
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
