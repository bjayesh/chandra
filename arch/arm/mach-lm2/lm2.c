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
#include <linux/dma-mapping.h>
#include <linux/ata_platform.h>
#include <linux/memblock.h>
#include <linux/spinlock.h>
#include <linux/device.h>
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
#include <asm/hardware/arm_timer.h>
#include <asm/hardware/cache-l2x0.h>
#include <linux/stmmac.h>
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

static	void	lm2_putchar(unsigned long base, char c)
{
        while((UART_STAT(base) & 0x40) == 0)
                barrier();
        UART_DATA(base) = c;
        return;
}

static  void	lm2_flush(unsigned long base)
{
        while((UART_STAT(base) & 0x40) == 0)
                barrier();
}

void	lm2_printk(unsigned long base, const char *ptr)
{
        char    c;

        while((c = *ptr++) != '\0'){
                if(c == '\n')
                        lm2_putchar(base,'\r');
                lm2_putchar(base,c);
        }
	lm2_flush(base);
}

/*
 * Static I/O map
 * serial ,GIC, timer, ethernet, SPI etc.
 */
static struct map_desc lm2_io_desc[] __initdata = {
	{
/*		.virtual	= LM2_DEBUG_SERIAL_VIRT,	*/
		.virtual	= 0xfc000000,
		.pfn		= __phys_to_pfn(0x04160000ULL),
		.length		= SZ_4K,
		.type		= MT_DEVICE,
	},
#if 0
	{
		.virtual	= LM2_CCI_VIRT,
		.pfn		= __phys_to_pfn(0x04060000ULL),
		.length		= SZ_64K,
		.type		= MT_DEVICE,
	},
	{
		.virtual	= LM2_GPIO_VIRT,
		.pfn		= __phys_to_pfn(0x04080000ULL),
		.length		= SZ_64K,
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

lm2_printk(0xfc000000,"lm2_timer_init\n");
/*	clksrc_timer = ioremap(LM2_TIMER_BASE + 0x38, 0x0c);	*/
	clksrc_timer = ioremap(LM2_TIMER_BASE+0x10, 0x10);
sprintf(buf,"Timer1=%x\n",clksrc_timer);
lm2_printk(0xfc000000,buf);
	clkevt_timer = ioremap(LM2_TIMER_BASE+0x2c,0x0c);
sprintf(buf,"Timer0=%x\n",clkevt_timer);
lm2_printk(0xfc000000,buf);
	lm2_clocksource_init(clksrc_timer);
	lm2_clockevent_init(LM2_IRQ_TIMER_0,clkevt_timer);
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
//static	struct	plat_stmmacenet_data	lm2_gmac_resource ={
//	.pmt = 1,
//	.has_gmac = 1,
//};
//
//static struct	platform_device lm2_gmac_device ={
//	.name = "dwmac",
//	.dev.platform_data = &lm2_gmac_resource,
//};

static struct resource lm2_eth_resources[] = {
	{
		.start	= FM3_MAC_BASE,
		.end	= FM3_MAC_BASE + SZ_64K - 1,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= IRQ_V2M_LAN9118,
		.end	= IRQ_V2M_LAN9118,
		.flags	= IORESOURCE_IRQ,
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
 * USB Host Device
 */
static struct resource lm2_usb_resources[] = {
	{
		.start	= LM2_USB2,
		.end	= LM2_USB2 + SZ_128K - 1,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= LM2_IRQ_USB_HOST,
		.end	= LM2_IRQ_USB_HOST,
		.flags	= IORESOURCE_IRQ,
	},
};

/*
 * non bus master device
 */
/*
static AMBA_APB_DEVICE(wdt,   "mb:wdt",   0, V2M_WDT, IRQ_V2M_WDT, NULL);
static AMBA_APB_DEVICE(rtc,   "mb:rtc",   0, V2M_RTC, IRQ_V2M_RTC, NULL);

static struct amba_device *v2m_amba_devs[] __initdata = {
	&wdt_device,
	&rtc_device,
};

*/
static void __init lm2_init_early(void)
{
	lm2_printk(0xfc000000,"lm2_init_early\n");
}

static void lm2_power_off(void)
{
	printk(KERN_EMERG "Unable to shutdown\n");
}

static void lm2_restart(char str, const char *cmd)
{
	printk(KERN_EMERG "Unable to reboot\n");
}

//static	void	__init	lm2_early_serial_setup(void)
//{
//#ifdef	CONFIG_SERIAL_8250_CONSOLE
//	static struct uart_port	lm2_console_port = {
//		.irq 		= LM2_IRQ_UART_1,
//		.iotype		= UPIO_MEM,
//		.flags		= UPF_SKIP_TEST,
//		.regshift	= 0,
//		.uartclk	= LM2_UART1_CLK,
//		.line		= 0,
//	};
//	lm2_console_port.membase = ioremap(LM2_UART_1_BASE,0x32);
//
//	early_serial_setup(&lm2_console_port);
//#endif	/* CONFIG_SERIAL_8250_CONSOLE */
//}

static void __init lm2_map_io(void)
{
	lm2_printk(0xfc160000,"lm2_map_io\n");
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
sprintf(buf,"GIC:Dist=%x\n",virt_dist);
lm2_printk(0xfc000000,buf);
	virt_cpui = ioremap(LM2_GIC_CPU,SZ_4K);
sprintf(buf,"GIC:CPU=%x\n",virt_cpui);
lm2_printk(0xfc000000,buf);

	lm2_printk(0xfc000000,"lm2_init_irq\n");
	gic_init_bases(0,29,ioremap(LM2_GIC_DIST,SZ_4K),ioremap(LM2_GIC_CPU,SZ_4K),0,NULL);
//	lm2_early_serial_setup();
	lm2_printk(0xfc000000,"lm2_init_irq end\n");
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
	meminfo->bank[0].start = 0x890000000ULL;
	meminfo->bank[0].size = SZ_512M;
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

	lm2_printk(0xfc000000,"lm2_init\n");
	virt_addr = ioremap(LM2_UART_1_BASE,0x32);
	lm2_serial_resource[0].membase = virt_addr;
	virt_addr = ioremap(LM2_UART_0_BASE,0x32);
	lm2_serial_resource[1].membase = virt_addr;
	platform_device_register(&lm2_serial_device);
	platform_device_register(&lm2_eth_device);
	lm2_printk(0xfc000000,"lm2_init:platform_register end\n");
#ifdef	CONFIG_SATA_AHCI_PLATFORM
	lm2_sata_register();
#endif	/* CONFIG_SATA_AHCI_PLATFORM */
	lm2_usb_register();
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

#if defined(CONFIG_SMP)
	vexpress_dt_smp_map_io();
#endif
}

static u32 osc;

/*
 * Device Tree Brob Initializer
 */
void __init lm2_dt_init_early(void)
{
	struct device_node *node;
	u32 dt_hbi;

	node = of_find_compatible_node(NULL, NULL, "arm,vexpress-sysreg");
}
/*
 * GIC find data 
 */
static  struct of_device_id lm2_irq_match[] __initdata = {
	{ .compatible = "arm,cortex-a9-gic", .data = gic_of_init, },
	{}
};
i
/*
 * General Interrupt Controller Initialize by DTB
 */
static void __init lm2_dt_init_irq(void)
{
	of_irq_init(lm2_irq_match);
}

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
	regbase = of_iomap(node,0)
	timer_irq = irq_of_parse_and_map(node,0);
	clocksource_register_hz(&clocksource,LM2_TIMER_HZ);
}

/*
 * init_machine by DTB
 */
static void __init lm2_dt_init(void)
{

	l2x0_of_init(0x00400000, 0xfe0fffff);
}

/*
 * match string for dtb
 */
const static char *lm2_dt_match[] __initconst = {
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
	.init_irq	= lm2_dt_init_irq,
	.init_time	= &lm2_dt_timer,
	.init_machine	= lm2_dt_init,
	.restart	= v2m_restart,
MACHINE_END

#endif	/* CONFIG_ARCH_LM2_DT */
