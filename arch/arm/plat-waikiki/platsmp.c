/*
 *  linux/arch/arm/plat-versatile/platsmp.c
 *
 *  Copyright (C) 2002 ARM Ltd.
 *  All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/memory.h>
#include <linux/io.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/smp.h>

#include <asm/cacheflush.h>
#include <asm/smp_plat.h>
//#include <asm/hardware/gic.h>

/*
 * control for which core is the next to come out of the secondary
 * boot "holding pen"

volatile int __cpuinitdata pen_release = -1;
*/
/*
 * Write pen_release in a way that is guaranteed to be visible to all
 * observers, irrespective of whether they're taking part in coherency
 * or not.  This is necessary for the hotplug code to work reliably.
 */
static void __cpuinit write_pen_release(int val)
{
	pen_release = val;
	smp_wmb();
#if 1
	sync_cache_w(&pen_release);
#else
	__cpuc_flush_dcache_area((void *)&pen_release, sizeof(pen_release));
	outer_clean_range(__pa(&pen_release), __pa(&pen_release + 1));
#endif
}

static DEFINE_SPINLOCK(boot_lock);

void __cpuinit waikiki_secondary_init(unsigned int cpu)
{
#if 1	/* debug */
	printk("#### %s\n", __func__); //FX
#endif
	/*
	 * if any interrupts are already enabled for the primary
	 * core (e.g. timer irq), then they will not have been enabled
	 * for us: do so
	 * gic_secondary_init(0); comment out
	 */
	/*
	 * let the primary processor know we're out of the
	 * pen, then head off into the C entry point
	 */
	write_pen_release(-1);

	/*
	 * Synchronise with the boot thread.
	 */
	spin_lock(&boot_lock);
	spin_unlock(&boot_lock);
}

int __cpuinit waikiki_boot_secondary(unsigned int cpu, struct task_struct *idle)
{
	unsigned long timeout;
#if 1	// FX
	unsigned char *cpu1_addr;
	volatile int a = 1;
	printk("#### %s(cpu=%x)\n", __func__, cpu); // FX
#endif

	/*
	 * Set synchronisation state between this boot processor
	 * and the secondary one
	 */
	spin_lock(&boot_lock);

	/*
	 * This is really belt and braces; we hold unintended secondary
	 * CPUs in the holding pen until we're ready for them.  However,
	 * since we haven't sent them a soft interrupt, they shouldn't
	 * be there.
	 */
	write_pen_release(cpu_logical_map(cpu));

	/*
	 * Send the secondary CPU a soft interrupt, thereby causing
	 * the boot monitor to read the system wide flags register,
	 * and branch to the address found there.
	 */
#if 1
	write_pen_release(cpu);
// FX
#if 1
	cpu1_addr = ioremap(0x043B0000,0x32);
	writel(0x3ff, cpu1_addr + 0x24);
	iounmap(cpu1_addr);
//	while (a);
#endif
#else
	gic_raise_softirq(cpumask_of(cpu), 1);
#endif

	timeout = jiffies + (6 * HZ);
	while (time_before(jiffies, timeout)) {
		smp_rmb();
		if (pen_release == -1)
			break;

		udelay(10);
	}

	/*
	 * now the secondary core is starting up let it run its
	 * calibrations, then wait for it to finish
	 */
	spin_unlock(&boot_lock);

	return pen_release != -1 ? -ENOSYS : 0;
}
