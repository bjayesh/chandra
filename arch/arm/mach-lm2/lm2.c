/*
 * FujiXerox LM2 Waikiki Motherboard Support
 * Copyright (c) 2013-2014 Wind River Systems, Inc 
 * Koki Yamano < koki.yamano@windriver.com >
 * This file is released under the GPLv2
 *
 */

#include <linux/device.h>
#include <linux/io.h>
#include <linux/init.h>
#include <linux/of_address.h>
#include <linux/of_fdt.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/irqchip.h>
#include <linux/ata_platform.h>
#include <linux/memblock.h>
#include <linux/spinlock.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/clkdev.h>
#include <linux/mtd/physmap.h>
#include <linux/serial_8250.h>
#include <linux/stmmac.h>
#include <linux/console.h>
#include <asm/arch_timer.h>
#include <asm/mach-types.h>
#include <asm/setup.h>
#include <asm/sizes.h>
#include <asm/smp_twd.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/time.h>
#include <asm/hardware/cache-l2x0.h>
#include <linux/irqchip/arm-gic.h>
/*
 * BSPs include files 
 */
#include <mach/motherboard.h>
#include <mach/irqs.h>
#include <mach/eth.h>
#include <plat/sched_clock.h>

#include "core.h"

extern	void	lm2_clocksource_init(void __iomem *gpt);
extern	void	lm2_clockevent_init(int irq, void __iomem *gpt);
/*
 * LM2 early_debug
 */
#define UART_DATA(base) (*(volatile unsigned char *)((base) + 0x10))
#define UART_STAT(base) (*(volatile unsigned char *)((base) + 0x15))

#ifdef	CONFIG_LM2_GPDMA
extern	lm2_dma_register(void);
#endif	/* CONFIG_LM2_GPDMA */

static	void	lm2_putchar(unsigned long base, char c)
{
#if 0
        while((UART_STAT(base) & 0x40) == 0)
                barrier();
        UART_DATA(base) = c;
#endif
        return;
}

static  void	lm2_flush(unsigned long base)
{
#if 0
        while((UART_STAT(base) & 0x40) == 0)
                barrier();
#endif
}

void	lm2_printk(unsigned long base, const char *ptr)
{
#if 0
        char    c;

        while((c = *ptr++) != '\0'){
                if(c == '\n')
                        lm2_putchar(base,'\r');
                lm2_putchar(base,c);
        }
	lm2_flush(base);
#endif
}

/*
 * Static I/O map
 * serial ,GIC, timer, ethernet, SPI etc.
 */
static struct map_desc lm2_io_desc[] __initdata = {
#if 0
	{
		.virtual	= 0xffc00000,
		.pfn		= __phys_to_pfn(0x0000000004060000ULL),
		.length		= SZ_64K,
		.type		= MT_DEVICE,
	},
	{
/*		.virtual	= LM2_DEBUG_SERIAL_VIRT,	*/
		.virtual	= 0xffc10000,
		.pfn		= __phys_to_pfn(0x04160000ULL),
		.length		= SZ_4K,
		.type		= MT_DEVICE,
	},
#endif
};

/*
 * system timer initial
 */
static void __init lm2_timer_init(void)
{
	char	buf[128];
	void	__iomem	*clksrc_timer;
	void	__iomem	*clkevt_timer;

/*	clksrc_timer = ioremap(LM2_TIMER_BASE + 0x38, 0x0c);	*/
	clksrc_timer = ioremap(LM2_TIMER_BASE + 0x10, 0x10);
	clkevt_timer = ioremap(LM2_TIMER_BASE + 0x5c, 0x0c);
	lm2_clocksource_init(clksrc_timer);
	lm2_clockevent_init(LM2_IRQ_TIMER_4,clkevt_timer);
}

/*
 * UART
 */
static	struct	plat_serial8250_port	lm2_serial_resource[]={
	{
//		.membase	= LM2_UART_1_BASE,
		.mapbase	= LM2_UART_1_BASE,
		.irq		= LM2_IRQ_UART_1,
		.uartclk	= LM2_UART1_CLK,
		.regshift	= 0,
		.iotype		= UPIO_MEM,
		.flags		= UPF_SKIP_TEST,
	},
	{
//		.membase	= LM2_UART_0_BASE,
		.mapbase	= LM2_UART_0_BASE,
		.irq		= LM2_IRQ_UART_0,	/* change. 0->1 */
		.uartclk	= LM2_UART0_CLK,	/* change. 0->1 */
		.regshift	= 0,
		.iotype		= UPIO_MEM,
		.flags		= UPF_SKIP_TEST,
	},
	{},
};
static struct platform_device lm2_serial_device = {
	.name = "serial8250",
	.id	= PLAT8250_DEV_PLATFORM,
	.dev	= {
		.platform_data = lm2_serial_resource,
	},
};

#ifdef	CONFIG_SERIAL_8250_CONSOLE
static int	__init lm2_console_init(void)
{
#if 0
	return add_preferred_console("ttyS",0,"38400");
#endif
}
#endif	/* CONFIG_SERIAL_8250_CONSOLE */

/*
 * I2C
 */

/*
 * Ethernet Controller
 */
static struct resource lm2_eth_resources[] = {
        {
                .start  = LM2_GMAC_BASE,
                .end    = LM2_GMAC_BASE + SZ_64K - 1,
                .flags  = IORESOURCE_MEM,
        },
	{
		.name	="macirq",
		.start	= LM2_IRQ_GMACK_STAT,
		.end	= LM2_IRQ_GMACK_STAT,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name	= "eth_wake_irq",
		.start	= LM2_IRQ_GMACK_WAKE,
		.end	= LM2_IRQ_GMACK_WAKE,
		.flags	= IORESOURCE_IRQ,
	},
};

static	struct stmmac_mdio_bus_data phy_private_data = {
	.phy_mask	= 1,
//	.irqs		= 15,
	.irqs		= 0,	/* poll */
};

static	struct	plat_stmmacenet_data	lm2_eth_config = {
	.bus_id		= 0,
	.phy_addr	= -1,
	.mdio_bus_data	= &phy_private_data,
	.has_gmac	= 1,
	.clk_csr	= 0,
	.enh_desc       = 1,	/* ohkuma add */
};

static struct platform_device lm2_eth_device = {
	.name		= "stmmaceth",
	.id		= -1,
	.resource	= lm2_eth_resources,
	.num_resources	= ARRAY_SIZE(lm2_eth_resources),
	.dev.platform_data = &lm2_eth_config,
	.dev.coherent_dma_mask	= DMA_BIT_MASK(64),
};

/*
 * GPIO
 */
static	struct resource	lm2_gpio_resource[]={
	{
		.start	= 0x04080000,
		.end	= 0x04080100,
		.flags	= IORESOURCE_MEM,
	},
};

static	struct platform_device	lm2_gpio_device = {
	.name		= "mmio-gpio",
	.id		= -1,
	.resource	= lm2_gpio_resource,
	.num_resources	= ARRAY_SIZE(lm2_gpio_resource),
};

/*
 * RTC
 */
static  struct resource lm2_rtc_resource[] ={
        {
                .start  = 0x04030000,
                .end    = 0x04030100,
                .flags  = IORESOURCE_MEM,
        },
	{
		.start	= 34,
		.end	= 34,
		.flags	= IORESOURCE_IRQ,
	},
};

static	struct platform_device lm2_rtc_device = {
	.name		= "lm2-rtc",
	.id		= -1,
	.resource	= lm2_rtc_resource,
	.num_resources	= ARRAY_SIZE(lm2_rtc_resource),
};

/*
 * Watch Dog
 */
static struct resource lm2_wdt_resource[]={
       {
               .start  = 0x04040000,
               .end    = 0x04040100,
               .flags  = IORESOURCE_MEM,
       },
};

static struct platform_device lm2_wdt_device = {
       .name           = "lm2-wdt",
       .id             = -1,
       .resource       = lm2_wdt_resource,
       .num_resources  = ARRAY_SIZE(lm2_wdt_resource),
};


static	struct resource	lm2_i2c_resource[] = {
	{
		.start	= 0x041F0100,
		.end	= 0x041F01FF,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= 0x041F0000,
		.end	= 0x041F00FF,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= LM2_IRQ_CIPUI,
		.end	= LM2_IRQ_CIPUI,
		.flags	= IORESOURCE_IRQ,
	},
};

static	struct platform_device lm2_i2c_device = {
	.name		= "lm2-eeprom",
	.id		= -1,
	.resource	= lm2_i2c_resource,
	.num_resources	= ARRAY_SIZE(lm2_i2c_resource),
};

static struct resource lm2_pcie_resource[]={
       {	/* reset gen */
               .start  = 0x04010000,
               .end    = 0x040101ff,
               .flags  = IORESOURCE_MEM,
       },
       {	/* warp */
               .start  = 0x04a70000,
               .end    = 0x04a701ff,
               .flags  = IORESOURCE_MEM,
       },
       {	/* port 1 */
               .start  = 0x04a40000,
               .end    = 0x04a4ffff,
               .flags  = IORESOURCE_MEM,
       },
       {	/* port 2 */
               .start  = 0x04a50000,
               .end    = 0x04a5ffff,
               .flags  = IORESOURCE_MEM,
       },
       {	/* port 3 */
               .start  = 0x04a60000,
               .end    = 0x04a6ffff,
               .flags  = IORESOURCE_MEM,
       },
};

static	struct platform_device lm2_pcie_device = {
	.name		= "synopsys-pcie",
	.id		=-1,
	.resource	= lm2_pcie_resource,
	.num_resources	= ARRAY_SIZE(lm2_pcie_resource),
};

static void __init lm2_init_early(void)
{
//	lm2_printk(0xfc000000,"lm2_init_early\n");
}

static void lm2_power_off(void)
{
	printk(KERN_EMERG "Unable to shutdown\n");
}

static void lm2_restart(char str, const char *cmd)
{
	printk(KERN_EMERG "Unable to reboot\n");
}

static void __init lm2_map_io(void)
{
	iotable_init(lm2_io_desc, ARRAY_SIZE(lm2_io_desc));
#ifdef CONFIG_SERIAL_8250_CONSOLE
//	lm2_early_serial_setup();
#endif /* CONFIG_SERIAL_8250_CONSOLE */
}

/*
 * General Interrupt Controller Initialize without DTB mode
 */
static void __init lm2_init_irq(void)
{
	char	buf[128];
	void __iomem	*virt_dist;
	void __iomem	*virt_cpui;

	virt_dist = ioremap(LM2_GIC_DIST,SZ_4K);
	virt_cpui = ioremap(LM2_GIC_CPU,SZ_4K);

//	lm2_printk(0xfc000000,"lm2_init_irq\n");
	gic_init_bases(0,29,ioremap(LM2_GIC_DIST,SZ_4K),ioremap(LM2_GIC_CPU,SZ_4K),0,NULL);
//	lm2_printk(0xfc000000,"lm2_init_irq end\n");
}

/*
 * lowmem limit 0x8_bfff_ffff
 * tags:from u-boot ATAG info
 * from:from u-boot Command line parameter
 * meminfo: from ATAG mem size parameter
 */
static	void __init lm2_fixup_mem(struct tag *tags, char **form, struct meminfo *meminfo)
{
#if 1	/* yamano debug */
	meminfo->bank[0].start = PHYS_OFFSET;
	meminfo->bank[0].size = 760*1024*1024;
	meminfo->nr_banks = 1;
#endif /* yamano debug */
	return;
}
/*
 * Machine initialize routine without DTB mode
 */
static void __init lm2_init(void)
{
	void __iomem *virt_addr;

	virt_addr = ioremap(LM2_UART_1_BASE,0x32);
	lm2_serial_resource[0].membase = virt_addr;
	virt_addr = ioremap(LM2_UART_0_BASE,0x32);
	lm2_serial_resource[1].membase = virt_addr;
	platform_device_register(&lm2_serial_device);
	platform_device_register(&lm2_eth_device);
#ifdef	CONFIG_SATA_AHCI_PLATFORM
	lm2_sata_register();
#endif	/* CONFIG_SATA_AHCI_PLATFORM */
	lm2_usb_register();
#ifdef	CONFIG_MMC_SDHCI_PLTFM
        lm2_sdhci_init();
#endif
#ifdef	CONFIG_LM2_GPDMA
	lm2_dma_register();
#endif	/* CONFIG_LM2_GPDMA */

	platform_device_register(&lm2_gpio_device);
	platform_device_register(&lm2_rtc_device);
	platform_device_register(&lm2_i2c_device);
	platform_device_register(&lm2_wdt_device);
#ifdef	CONFIG_SPI_XSPI
	lm2_xspi_register();
#endif	/* CONFIG_SPI_XSPI */
	platform_device_register(&lm2_pcie_device);
}

MACHINE_START(LM2, "FujiXerox Waikiki")
	.atag_offset	= 0x100,
#ifdef	CONFIG_SMP
	.smp		= smp_ops(lm2_smp_ops),
#endif
	.fixup		= lm2_fixup_mem,
	.map_io		= lm2_map_io,
	.init_early	= lm2_init_early,
	.init_irq	= lm2_init_irq,
	.init_time	= lm2_timer_init,
	.init_machine	= lm2_init,
	.restart	= lm2_restart,
MACHINE_END

/*
 * Use Flatten Device Tree
 */
#if defined(CONFIG_ARCH_LM2_DT)

/*
 * I/O mapping
 */

void __init lm2_dt_map_io(void)
{

	iotable_init(lm2_io_desc, ARRAY_SIZE(lm2_io_desc));

}

//static u32 osc;

/*
 * Device Tree Brob Initializer
 */
void __init lm2_dt_init_early(void)
{
	return;
}
/*
 * GIC find data 
 */
#if 0
static  struct of_device_id lm2_irq_match[] __initdata = {
	{ .compatible = "arm,cortex-a15-gic", .data = gic_of_init, },
	{}
};

/*
 * General Interrupt Controller Initialize by DTB
 */
static void __init lm2_dt_init_irq(void)
{
	of_irq_init(lm2_irq_match);
}
#endif

/*
 * Kernel timer initialize routine by DTB
 */
static void __init lm2_dt_timer_init(void)
{
	struct device_node *node;
	const char *path;
	int err;

	/* aliase check to unique node */ 
	err = of_property_read_string(of_aliases, "arm,lm2_timer", &path);
	if (WARN_ON(err))
		return;

	/* get DTB node */
	node = of_find_node_by_path(path);

	/* timer device add device node */
//	regbase = of_iomap(node,0);
//	timer_irq = irq_of_parse_and_map(node,0);
//	clocksource_register_hz(&clocksource,LM2_TIMER_HZ);
//	clocksource_register_hz(&clocksource,LM2_TIM32_CLK);
}

/*
 * init_machine by DTB
 */
static const struct of_device_id lm2_dt_bus_match[] __initconst = {
        { .compatible = "simple-bus", },
        { .compatible = "arm,amba-bus", },
        { .compatible = "arm,vexpress,config-bus", },
        {}
};

static void __init lm2_dt_init(void)
{
        void __iomem *virt_addr;

	/* Serial DTB ok */
	virt_addr = ioremap(LM2_UART_1_BASE,0x32);
	lm2_serial_resource[0].membase = virt_addr;
	virt_addr = ioremap(LM2_UART_0_BASE,0x32);
	lm2_serial_resource[1].membase = virt_addr;
	platform_device_register(&lm2_serial_device);
	platform_device_register(&lm2_eth_device);
#ifdef	CONFIG_SATA_AHCI_PLATFORM
	lm2_sata_register();
#endif	/* CONFIG_SATA_AHCI_PLATFORM */
	lm2_usb_register();
#ifdef	CONFIG_MMC_SDHCI_PLTFM
        lm2_sdhci_init();
#endif
#ifdef	CONFIG_LM2_GPDMA
	lm2_dma_register();
#endif	/* CONFIG_LM2_GPDMA */
	platform_device_register(&lm2_gpio_device);
	platform_device_register(&lm2_rtc_device);
	platform_device_register(&lm2_i2c_device);
	platform_device_register(&lm2_wdt_device);
#ifdef	CONFIG_SPI_XSPI
	lm2_xspi_register();
#endif	/* CONFIG_SPI_XSPI */
	platform_device_register(&lm2_pcie_device);

//	l2x0_of_init(0x00400000, 0xfe0fffff);
	of_platform_populate(NULL, lm2_dt_bus_match, NULL, NULL);
}

/*
 * match string for dtb
 */
static const char * const lm2_dt_match[] __initconst = {
	"FujiXerox,waikiki",
	NULL,
};

DT_MACHINE_START(LM2_DT, "FujiXerox Waikiki")
	.dt_compat	= lm2_dt_match,
#ifdef	CONFIG_SMP
	.smp		= smp_ops(lm2_smp_ops),
#endif
	.map_io		= lm2_dt_map_io,
	.init_early	= lm2_dt_init_early,
	.init_irq	= lm2_init_irq,
//	.init_irq	= irqchip_init,
	.init_time	= lm2_timer_init,
	.init_machine	= lm2_dt_init,
MACHINE_END

#endif	/* CONFIG_ARCH_LM2_DT */
