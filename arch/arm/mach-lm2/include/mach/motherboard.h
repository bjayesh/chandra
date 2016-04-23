/*
 * Fuji Xerox Co., Ltd. Waikiki LM2 board
 * Copyright 2014-2015 Wind River Systems Inc.
 * koki.yamano@windriver.com
 * JPN-FXCL-12490
 */

#ifndef __MACH_MOTHERBOARD_H
#define __MACH_MOTHERBOARD_H

#define	LM2_A15_CPUS	2
/*
 * System clocking
 */
#define	LM2_A15_CLK	(800*1000*1000)	/* 800MHz */
#define	LM2_A7_CLK	(400*1000*1000)	/* 400MHz */
#define	LM2_TIM32_CLK	(300*1000*1000)	/* 300MHz */
#define	LM2_TIM64_CLK	(300*1000*1000)	/* 300MHz */
#define	LM2_UART0_CLK	(300*1000*1000)	/* 300MHz */
#define	LM2_UART1_CLK	(300*1000*1000)	/* 300MHz */
#define	LM2_UART2_CLK	(343*1000*1000)	/* 300MHz */
#define	LM2_UART3_CLK	(343*1000*1000)	/* 300MHz */
#define	LM2_UART4_CLK	(343*1000*1000)	/* 300MHz */
#define	LM2_UART5_CLK	(343*1000*1000)	/* 300MHz */

#define	LM2_UART0_BPS	38400

/* Physical addresses, Sub system 0 */
#define	LM2_REGS_0	0x04000000
#define	LM2_PERIPHERAL	LM2_REGS_0

/* Physical addresses, Sub system 1 */
#define	LM2_REGS_1	0x05000000
#define	LM2_REGS_2	0x05400000
#define	LM2_REGS_3	0x05600000

/*
 * Physical addresses, offset from V2M_PA_CS0-3
 */

/*
 * Physical address, I/O 
 */
#define	LM2_RTC_BASE		(LM2_REGS_0 + 0x00030000)
#define	LM2_TIMER_BASE		(LM2_REGS_0 + 0x00040000)
#define	LM2_CCI_BASE		(LM2_REGS_0 + 0x00060000)
#define	LM2_GPIO_BASE		(LM2_REGS_0 + 0x00080000)
#define	LM2_FCSPI_0_BASE	(LM2_REGS_0 + 0x00110000)
#define	LM2_XSPI_0_BASE		(LM2_REGS_0 + 0x00130000)
#define	LM2_GPDMA_0_BASE	(LM2_REGS_0 + 0x00150000)
#define	LM2_UART_0_BASE		(LM2_REGS_0 + 0x000b0010)
#define	LM2_UART_1_BASE		(LM2_REGS_0 + 0x00160010)
#define	LM2_CIP_UI_BASE		(LM2_REGS_0 + 0x001f0000)
#define	LM2_GIC_BASE		(LM2_REGS_0 + 0x00300000)
#define	LM2_USB3_PHY		(LM2_REGS_0 + 0x00400000)
#define	LM2_USB2_PHY		(LM2_REGS_0 + 0x00408000)
#define	LM2_GMAC_BASE		(LM2_REGS_0 + 0x00410000)
#define	LM2_USB3		(LM2_REGS_0 + 0x00500000)
#define	LM2_USB2		(LM2_REGS_0 + 0x00600000)
#define LM2_SDIO0_BASE          (LM2_REGS_0 + 0x00440200)       /* SD0 slot1 */
#define LM2_SDIO1_BASE          (LM2_REGS_0 + 0x00450100)       /* SD1 slot0 */
#define LM2_GPFSYS_BASE         (LM2_REGS_0 + 0x00050000)

/*
 * Physical addresses, offset from V2M_PA_CS7
 */


/*
 * Interrupts.  Those in {} are for AMBA devices
 */
#define IRQ_V2M_WDT		{ (32 + 0) }
#define IRQ_V2M_TIMER0		(32 + 2)
#define IRQ_V2M_TIMER1		(32 + 2)
#define IRQ_V2M_TIMER2		(32 + 3)
#define IRQ_V2M_TIMER3		(32 + 3)
#define IRQ_V2M_RTC		{ (32 + 4) }
#define IRQ_V2M_UART0		{ (32 + 5) }
#define IRQ_V2M_UART1		{ (32 + 6) }
#define IRQ_V2M_UART2		{ (32 + 7) }
#define IRQ_V2M_UART3		{ (32 + 8) }
#define IRQ_V2M_MMCI		{ (32 + 9), (32 + 10) }
#define IRQ_V2M_AACI		{ (32 + 11) }
#define IRQ_V2M_KMI0		{ (32 + 12) }
#define IRQ_V2M_KMI1		{ (32 + 13) }
#define IRQ_V2M_CLCD		{ (32 + 14) }
#define IRQ_V2M_LAN9118		(32 + 15)
#define IRQ_V2M_ISP1761		(32 + 16)
#define IRQ_V2M_PCIE		(32 + 17)

/*
 * Miscellaneous
 */
#define SYS_MISC_MASTERSITE	(1 << 14)
#define SYS_PROCIDx_HBI_MASK	0xfff

/*
 * Core tile IDs
 */
struct ct_desc {
	u32			id;
	const char		*name;
	void			(*map_io)(void);
	void			(*init_early)(void);
	void			(*init_irq)(void);
	void			(*init_tile)(void);
#ifdef CONFIG_SMP
	void			(*init_cpu_map)(void);
	void			(*smp_enable)(unsigned int);
#endif
};

extern struct ct_desc *ct_desc;

#endif
