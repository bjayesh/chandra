/*
 * clock.c - LM2-Waikiki board Clock Interface platform
 * Copyright 2014 Wind River Systems,Inc.
 * 
 */
#include <linux/clkdev.h>

static struct clk dmac_clk = {
	.rate	= 125000000,	// 125MHz
};
static struct clk xspi_clk = {
	.rate	= 300000000,	// 300MHz
};
static struct clk sata_clk = {
	.rate	= 100000000,	// 100MHz
};

static struct clk_lookup lookups[] = {
		    /* dev_id     con_id       clk */
	CLKDEV_INIT("stmmaceth",  "stmmaceth", &dmac_clk),	/* GMAC */
	CLKDEV_INIT("mmio-xspi.0",NULL,        &xspi_clk),	/* XSPI */
	CLKDEV_INIT("ahci",       NULL,        &sata_clk),	/* SATA */
};


void __init lm2_init_clock(void)
{
	clkdev_add_table(lookups, ARRAY_SIZE(lookups));
}
