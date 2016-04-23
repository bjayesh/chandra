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
#include <linux/phy.h>

#include "core.h"

/* #ifdef CONFIG_TCG_ST33_SPI_OPTION */ 
#include <linux/spi/tpm_stm_st33_spi.h> 
#include <linux/spi/spi.h> 
/* #endif */ /* CONFIG_TCG_ST33_SPI_OPTION */ 

#if 1	/* WR Change */
#define OS_PRODUCT_ID_BOMBORA    0x33
#define OS_PRODUCT_ID_KAIMANA    0x34
#define OS_PRODUCT_ID_TOMBOLO    0x3b
#define OS_PRODUCT_ID_MARIS      0x3c
int kernelGetProductId(void){
        int* productid;
        struct device_node *node;

        node = of_find_node_by_path("/bootinfo");
        if(node == NULL){
                printk(KERN_ERR "[kernelGetProductId]of_find_node_by_path failed\n");
                return ERROR;
        }

        productid = (int *)of_get_property(node, "productid", NULL);
        if(productid == NULL){
                printk(KERN_ERR "[kernelGetProductId]of_get_property failed\n");
                return ERROR;
        }

        return *productid;
}
#else
#include <linux/fxmodule/kernelOsddi.h>
#endif

extern	void	lm2_clocksource_init(void __iomem *gpt);
extern	void	lm2_clockevent_init(int irq, void __iomem *gpt);
extern void    lm2_init_clock(void);
extern void    lm2_cipui_tim_init(void);
#define        NEW_PANBUG
/*
 * LM2 early_debug
 */
#define UART_DATA(base) (*(volatile unsigned char *)((base) + 0x10))
#define UART_STAT(base) (*(volatile unsigned char *)((base) + 0x15))

#ifdef	CONFIG_LM2_GPDMA
extern	lm2_dma_register(void);
#endif	/* CONFIG_LM2_GPDMA */
/*
 * Static I/O map
 * serial ,GIC, timer, ethernet, SPI etc.
 */
static struct map_desc lm2_io_desc[] __initdata = {
};

/*
 * Use Irq
 */
const unsigned char lm2_use_irq[] = {
34,    // RTC
40,    // Timer
44,    // CPLD
45,    // Ethernet PHY Link up
#ifndef        NEW_PANBUG
50,    // UART0
#endif
53,    // fcspi
56,    // xspi
59,    // quatro-gpdma
61,    // UART1
64,    // lm2-timalm.0, lm2-i2c
85,    // ahci
93,    // dwc3
95,    // pci
96,    // pci
97,    // pci
98,    // pci
125,   // GMAC (WakeUp)
126,   // GMAC
129,   // USB
130,   // USB
131,   // mmc0
133,   // mmc1
};
const unsigned int lm2_use_irq_size = sizeof(lm2_use_irq);

static int lm2_board_version = 0;

static void lm2_get_board_version(void)
{
	void __iomem *virt_addr;
	u32     val;
	virt_addr = ioremap(0x04010024,0x4);
	val  = readl(virt_addr);
	iounmap(virt_addr);
	lm2_board_version = val;
}

int lm2_board_is_A0(void)
{
	return (lm2_board_version == 0);
}

EXPORT_SYMBOL_GPL(lm2_board_is_A0);

/*
 * system timer initial
 */
static void __init lm2_timer_init(void)
{
	void	__iomem	*clksrc_timer;
	void	__iomem	*clkevt_timer;

	lm2_get_board_version();

	clksrc_timer = ioremap(LM2_TIMER_BASE + 0x20, 0x10); /* HRT1 */
	clkevt_timer = ioremap(LM2_TIMER_BASE + 0x5c, 0x0c);
	lm2_clocksource_init(clksrc_timer);
	lm2_clockevent_init(LM2_IRQ_TIMER_4,clkevt_timer);
}

/*
 * UART
 */
static	struct	plat_serial8250_port	lm2_serial_resource[]={
	{
		.mapbase	= LM2_UART_1_BASE,
		.irq		= LM2_IRQ_UART_1,
		.uartclk	= LM2_UART1_CLK,
		.regshift	= 0,
		.iotype		= UPIO_MEM,
		.flags		= UPF_SKIP_TEST,
		.fxpwsave_enable	= 0,
	},
	{
		.mapbase	= LM2_UART_3_BASE,
		.irq		= LM2_IRQ_UART_3,
		.uartclk	= LM2_UART3_CLK,
		.regshift	= 0,
		.iotype		= UPIO_MEM,
		.flags		= UPF_SKIP_TEST,
		.fxpwsave_enable	= 1,
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
	.irqs		= 0,	/* poll */
};

static	struct	plat_stmmacenet_data	lm2_eth_config = {
	.bus_id		= 0,
	.phy_addr	= -1,
	.mdio_bus_data	= &phy_private_data,
	.has_gmac	= 1,
	.clk_csr	= 0,
	.enh_desc       = 1,	/* add */
	.interface	= PHY_INTERFACE_MODE_GMII,
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
               .end    = 0x04a40fff,
               .flags  = IORESOURCE_MEM,
       },
       {	/* host bridge interrput */
               .flags  = IORESOURCE_IRQ,
               .start  = LM2_IRQ_PCIE1,
               .end    = LM2_IRQ_PCIE1,
       },
};

static	struct platform_device lm2_pcie_device = {
	.name		= "synopsys-pcie",
	.id		=-1,
	.resource	= lm2_pcie_resource,
	.num_resources	= ARRAY_SIZE(lm2_pcie_resource),
};

/* #ifdef CONFIG_TCG_ST33_SPI_OPTION */ 
static struct st33zp24_platform_data tpm_data = {
    .io_serirq = 52,
};

static struct spi_board_info tpm_st33_spi_board_info[] = {
        {
                .modalias = TPM_ST33_SPI,
                .max_speed_hz = 10000000,
                .bus_num = 0,
                .chip_select = 2,
                .mode = SPI_MODE_0,
                .platform_data = &tpm_data,
        },
};
/* #endif*/ /* CONFIG_TCG_ST33_SPI_OPTION */

static void __init lm2_init_early(void)
{
}

static void lm2_restart(char str, const char *cmd)
{
	printk(KERN_EMERG "Unable to reboot\n");
}

static void __init lm2_map_io(void)
{
	iotable_init(lm2_io_desc, ARRAY_SIZE(lm2_io_desc));
}

/*
 * General Interrupt Controller Initialize without DTB mode
 */
static void __init lm2_init_irq(void)
{
	void __iomem	*virt_dist;
	void __iomem	*virt_cpui;

	virt_dist = ioremap(LM2_GIC_DIST,SZ_4K);
	virt_cpui = ioremap(LM2_GIC_CPU,SZ_4K);

	gic_init_bases(0,29,ioremap(LM2_GIC_DIST,SZ_4K),ioremap(LM2_GIC_CPU,SZ_4K),0,NULL);
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

	lm2_init_clock();

	virt_addr = ioremap(LM2_UART_1_BASE,0x32);
	lm2_serial_resource[0].membase = virt_addr;
#ifndef NEW_PANBUG
	virt_addr = ioremap(LM2_UART_0_BASE,0x32);
	lm2_serial_resource[1].membase = virt_addr;
#endif /* NEW_PANBUG */
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
	lm2_cipui_tim_init();
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
 * init_machine by DTB
 */
static const struct of_device_id lm2_dt_bus_match[] __initconst = {
        { .compatible = "simple-bus", },
        { .compatible = "arm,amba-bus", },
        { .compatible = "arm,vexpress,config-bus", },
        {}
};

static const struct of_device_id lm2_pcie_match[] = {
        {
                .compatible = "synopsys-pcie",
        },
};

static const struct of_device_id lm2_eth_match[] = {
        {
                .compatible = "snps,dwmac",
        },
};

static void __init lm2_dt_init(void)
{
        void __iomem *virt_addr;
	struct device_node	*node;
	const struct	of_device_id *of_id;
	int productId;

	lm2_init_clock();
	/* Serial DTB ok */
	virt_addr = ioremap(LM2_UART_1_BASE,0x32);
	lm2_serial_resource[0].membase = virt_addr;
	if (lm2_board_is_A0()) lm2_serial_resource[0].uartclk = (300*1000*1000);
#ifndef NEW_PANBUG
	virt_addr = ioremap(LM2_UART_0_BASE,0x32);
	lm2_serial_resource[1].membase = virt_addr;
	if (lm2_board_is_A0()) lm2_serial_resource[1].uartclk = (300*1000*1000);
#endif /* NEW_PANBUG */

	productId = kernelGetProductId();
	if (productId < 0) {
		printk(KERN_ERR "lm2.c UART : %s : <ERROR> kernelGetProductId failed (ret=%d). Assign default port.\n", __func__, productId);
	}
	else {
		printk(KERN_INFO "lm2.c UART : %s : productId = 0x%x\n", __func__, productId);
	}

	switch (productId) {
	case OS_PRODUCT_ID_BOMBORA:
	case OS_PRODUCT_ID_KAIMANA:
		printk(KERN_INFO "lm2.c UART : %s : assign port 2\n", __func__);
		lm2_serial_resource[1].membase	= ioremap(LM2_UART_2_BASE,0x32);
		lm2_serial_resource[1].irq		= LM2_IRQ_UART_2;
		lm2_serial_resource[1].uartclk	= LM2_UART2_CLK;
		break;
	case OS_PRODUCT_ID_TOMBOLO:
	case OS_PRODUCT_ID_MARIS:
		printk(KERN_INFO "lm2.c UART : %s : assign port 3\n", __func__);
		lm2_serial_resource[1].membase	= ioremap(LM2_UART_3_BASE,0x32);
		lm2_serial_resource[1].irq		= LM2_IRQ_UART_3;
		lm2_serial_resource[1].uartclk	= LM2_UART3_CLK;
		break;
	default:
		printk(KERN_INFO "lm2.c UART : %s : assign port 5\n", __func__);
		lm2_serial_resource[1].membase	= ioremap(LM2_UART_5_BASE,0x32);
		lm2_serial_resource[1].irq		= LM2_IRQ_UART_5;
		lm2_serial_resource[1].uartclk	= LM2_UART5_CLK;
	}

	if (lm2_board_is_A0()) lm2_serial_resource[1].uartclk = (300*1000*1000);

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
	
/* #ifdef CONFIG_TCG_ST33_SPI_OPTION */
        spi_register_board_info(tpm_st33_spi_board_info, ARRAY_SIZE(tpm_st33_spi_board_info));
/* #endif */ /* CONFIG_TCG_ST33_SPI_OPTION */

	of_platform_populate(NULL, lm2_dt_bus_match, NULL, NULL);
	node = of_find_matching_node(NULL,lm2_pcie_match);
	if(node){
		of_id = of_match_node(lm2_pcie_match,node);
		if(of_id)
			platform_device_register(&lm2_pcie_device);
		of_node_put(node);
	}
	lm2_cipui_tim_init();

	if (lm2_board_is_A0()){
		printk("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
		printk("@             This is A0 board              @\n");
		printk("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	} else {
		printk("*********************************************\n");
		printk("*             This is B0 board               \n");
		printk("*********************************************\n");
	}
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
	.init_time	= lm2_timer_init,
	.init_machine	= lm2_dt_init,
MACHINE_END

#endif	/* CONFIG_ARCH_LM2_DT */
