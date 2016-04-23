/*
 * FujiXerox LM2 Waikiki Motherboard Support
 * Copyright (c) 2013-2014 Wind River Systems, Inc
 * Koki Yamano < koki.yamano@windriver.com >
 * This file is released under the GPLv2
 *
 * arch/arm/mach-lm2/lm2_pm.c - LM2 power management support
 */
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/spinlock.h>
#include <linux/errno.h>
#include <linux/irqchip/arm-gic.h>

#include <linux/cpu_pm.h>
#include <linux/suspend.h>
#include <linux/cpu.h>
#include <asm/proc-fns.h>
#include <asm/cacheflush.h>
#include <asm/cputype.h>
#include <asm/cp15.h>
#include <linux/spi/xspi.h>
#include <linux/debugfs.h>

#include "core.h"

#define LM2_PM_DEBUG
//#define PM_TEST

/* Suspend End flag -> memory write */
#define LM2_SUSPEND_END_ADDRESS		0x804f00000
#define LM2_SUSPEND_END_DATA 		0x13
/* chksum */
extern unsigned long	chksum_info;

extern void deepsleep_up(void);
extern void irq_to_a7(unsigned int);
extern void lm2_wfi0(void);
extern void lm2_save_a15core(void);

/*
 * We can't use regular spinlocks. In the switcher case, it is possible
 * for an outbound CPU to call power_down() after its inbound counterpart
 * is already live using the same logical CPU number which trips lockdep
 * debugging.
 */

#define LM2_CLUSTERS		2
#define QUATRO55XX_MAX_CPUS_PER_CLUSTER	2

static unsigned int lm2_nr_cpus[LM2_CLUSTERS];

/* Keep per-cpu usage count to cope with unordered up/down requests */
static int lm2_pm_use_count[QUATRO55XX_MAX_CPUS_PER_CLUSTER][LM2_CLUSTERS];

#define lm2_cluster_unused(cluster) \
	(!lm2_pm_use_count[0][cluster] && \
	 !lm2_pm_use_count[1][cluster])

/* lm2_deeepsleepup_addr_set */
static void lm2_deeepsleepup_addr_set(void)
{
	SPILIB_PARAM	param;
	int		ret;
	phys_addr_t	start_adr = virt_to_phys(&deepsleep_up);
	u32		buf;

	buf = (start_adr&0xffffffff);
	param.unit   = SPI_UNIT3;
	param.offset = 0x4c;
	param.buf    = &buf;
	param.size   = 4;
#ifdef	LM2_PM_DEBUG
printk(KERN_ERR "LM2_PM: NVM addr=0x%x write_data=0x%08x\n", param.offset, buf);
#endif
	ret = xspi_write(&param);
	if ( ret != 0 )
		printk(KERN_ERR "== write: NVM Error.\n");
}

/* lm2_suspend_reg_set */
static void lm2_suspend_reg_set(phys_addr_t addr, u32 set_data)
{
	void    __iomem	*set_address;

	set_address = ioremap(addr, 0x4);
#ifdef	LM2_PM_DEBUG
printk(KERN_ERR "LM2_PM: addr=0x%llx write_data=0x%x\n", addr, set_data);
#endif
	writel(set_data, set_address);
	iounmap(set_address);
}

static int lm2_pm_begin(suspend_state_t state)
{
	lm2_deeepsleepup_addr_set();

	cpu_idle_poll_ctrl(true);
        return 0;
}

static void lm2_pm_end(void)
{
#ifdef  LM2_PM_DEBUG
        void    __iomem *get_address;
        u32             data;
        SPILIB_PARAM    param;
        u32             buf;
#endif

        cpu_idle_poll_ctrl(false);

#ifdef  LM2_PM_DEBUG
	/* Get */
        get_address = ioremap(LM2_SUSPEND_END_ADDRESS, 0x4);
        data = readl(get_address);
        printk(KERN_ERR "LM2_PM: BOOT adr=0x%llx r_data=0x%08x\n", LM2_SUSPEND_END_ADDRESS, data);
        iounmap(get_address);

        param.unit   = SPI_UNIT3;
        param.offset = 0x4c;
        param.buf    = &buf;
        param.size   = 4;
        if ( xspi_read(&param) != 0 )
                printk(KERN_ERR "== read: NVM Error.\n");
        printk(KERN_ERR "LM2_PM: NVM  addr=0x%x r_data=0x%08x\n", param.offset, buf);

        printk(KERN_ERR "LM2_PM: CHKF adr=0x%x(0x%llx) r_data=0x%08x\n",&chksum_info, virt_to_phys(&chksum_info), chksum_info);
#endif  /* LM2_PM_DEBUG */
}


static void lm2_pm_suspend(void)
{
	unsigned int mpidr, cpu, cluster;

	mpidr = read_cpuid_mpidr();
	cpu = MPIDR_AFFINITY_LEVEL(mpidr, 0);
	cluster = MPIDR_AFFINITY_LEVEL(mpidr, 1);

	pr_debug("%s: cpu %u cluster %u\n", __func__, cpu, cluster);
	BUG_ON(cluster >= LM2_CLUSTERS || cpu >= QUATRO55XX_MAX_CPUS_PER_CLUSTER);

#ifdef	PM_TEST
	printk(KERN_ERR "=> %s: irq=%03d TagetReg not Change\n",__func__,LM2_IRQ_CIPUI);
#else	/* PM_TEST */
	irq_to_a7(LM2_IRQ_CIPUI);	//  64
#endif	/* PM_TEST */
	irq_to_a7(LM2_IRQ_GMACK_STAT);	// 126
	irq_to_a7(LM2_IRQ_SPI_0);	//  44
	irq_to_a7(LM2_IRQ_SPI_2);	//  46

	lm2_suspend_reg_set(LM2_SUSPEND_END_ADDRESS, LM2_SUSPEND_END_DATA);

	lm2_save_a15core();		// Save A15Core C2/C13/SP
	lm2_wfi0();
	/************* Resume Start *************/
}

extern void dw3_reg_save(void);
extern void dw3_reg_load(void);
extern void lm2_pcie_suspend(void);
extern void lm2_pcie_resume(void);
extern void stmac_reg_save(void);
extern void stmac_reg_load(void);

static int lm2_pm_enter(suspend_state_t suspend_state)
{
	int ret=0;
	switch (suspend_state) {
		case PM_SUSPEND_STANDBY:
		case PM_SUSPEND_MEM:
#ifdef  LM2_PM_DEBUG
printk(KERN_ERR "LM2_PM: Register Save\n");
#endif
			stmac_reg_save();
			lm2_pcie_suspend();
			dw3_reg_save();
			lm2_pm_suspend();
#ifdef  LM2_PM_DEBUG
printk(KERN_ERR "LM2_PM: Register Load\n");
#endif
			dw3_reg_load();
			lm2_pcie_resume();
			stmac_reg_load();
			break;
		default:
			ret = -EINVAL;
	}
	return ret;
}

static int lm2_pm_finish(void)
{
	return 0;
}

#ifdef  LM2_PM_DEBUG
extern unsigned int	__deepsleep_mode;
static u8	run_flag[32];
static ssize_t
lm2_pm_status_read(struct file *filp, char __user *buf, size_t len, loff_t *ppos)
{
	if ( __deepsleep_mode ) {
		strcpy(&run_flag, "warm boot\n");
	} else {
		strcpy(&run_flag, "cold boot\n");
	}
        return simple_read_from_buffer(buf, len, ppos, &run_flag, sizeof(run_flag));
}

static struct file_operations lm2_pm_fops = {
        .owner = THIS_MODULE,
        .read  = lm2_pm_status_read,
};
#endif
static const struct platform_suspend_ops lm2_pm_ops = {
	.begin		= lm2_pm_begin,			// Suspend 1
	.end		= lm2_pm_end,
	.enter		= lm2_pm_enter,			// Suspend 2 ->lm2_pm_suspend()
	.finish		= lm2_pm_finish,
	.valid		= suspend_valid_only_mem,	//
};
	
static bool __init lm2_pm_usage_count_init(void)
{
	unsigned int mpidr, cpu, cluster;

	mpidr = read_cpuid_mpidr();
	cpu = MPIDR_AFFINITY_LEVEL(mpidr, 0);
	cluster = MPIDR_AFFINITY_LEVEL(mpidr, 1);

#ifdef  LM2_PM_DEBUG
printk(KERN_ERR "== %s: cpu %u cluster %u\n",__func__,cpu, cluster);
#endif
	if (cluster >= LM2_CLUSTERS || cpu >= lm2_nr_cpus[cluster]) {
		pr_err("%s: boot CPU is out of bound!\n", __func__);
		return false;
	}
	lm2_pm_use_count[cpu][cluster] = 1;
	return true;
}

static int __init lm2_pm_init(void)
{
	int ret;
#ifdef  LM2_PM_DEBUG
	struct dentry *d;
#endif

	lm2_nr_cpus[0] = 1;
	lm2_nr_cpus[1] = 2;

	if (!lm2_pm_usage_count_init()) {
		return -EINVAL;
	}

	suspend_set_ops(&lm2_pm_ops);

#ifdef  LM2_PM_DEBUG
	/* /sys/kernel/debug/lm2_pm */
	d = debugfs_create_file("lm2_pm", 0444, NULL, &run_flag, &lm2_pm_fops);
	if (!d)
		ret = -ENOMEM;
	strcpy(&run_flag, "cold boot\n");
#endif
	return ret;
}

early_initcall(lm2_pm_init);
