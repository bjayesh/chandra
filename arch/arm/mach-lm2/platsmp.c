/*
 *  linux/arch/arm/mach-lm2/platsmp.c
 *
 * Copyright (C) 2014 Wind River Systems, Inc.
 *  Copyright (C) 2002 ARM Ltd.
 *  All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/smp.h>
#include <linux/io.h>
#include <linux/of_fdt.h>

#include <asm/smp_scu.h>
#include <asm/mach/map.h>

#include <mach/motherboard.h>

#include "core.h"

extern	void	lm2_secondary_startup(void);
static	void	__iomem *misc_base;
extern	int	waikiki_boot_secondary(void);
extern	int	waikiki_secondary_init(unsigned int cpus);
extern	void	waikiki_cpu_die(unsigned int cpu);

#if defined(CONFIG_OF)

void __init lm2_dt_smp_map_io(void)
{
}
#if 0
static int __init lm2_dt_cpus_num(unsigned long node, const char *uname,
		int depth, void *data)
{
	static int prev_depth = -1;
	static int nr_cpus = -1;

	if (prev_depth > depth && nr_cpus > 0)
		return nr_cpus;

	if (nr_cpus < 0 && strcmp(uname, "cpus") == 0)
		nr_cpus = 0;

	if (nr_cpus >= 0) {
		const char *device_type = of_get_flat_dt_prop(node,
				"device_type", NULL);

		if (device_type && strcmp(device_type, "cpu") == 0)
			nr_cpus++;
	}

	prev_depth = depth;

	return 0;
}

static void __init lm2_dt_smp_prepare_cpus(unsigned int max_cpus)
{
}
#endif
#else

static void __init lm2_dt_smp_init_cpus(void)
{
}

void __init lm2_dt_smp_prepare_cpus(unsigned int max_cpus)
{
}

#endif /* CONFIG_OF */

/*
 * Initialise the CPU possible map early - this describes the CPUs
 * which may be present or become present in the system.
 */
void __init lm2_smp_init_cpus(void)
{
#ifdef	CONFIG_ARCH_LM2_DT
/*
	lm2_dt_smp_init_cpus();
*/
#else
	int	i,ncore;

	ncore = LM2_A15_CPUS;
	if(ncore > nr_cpu_ids)	/* Kernel config check */
		ncore = nr_cpu_ids;

	for(i = 0 ; i < ncore ; i++)
		set_cpu_possible(i,true);

#if 0	/* drivers/irqchip/irq-gic.c set */
	set_smp_cross_call(gic_raise_softirq);
#endif
#endif	/* CONFIG_ARCH_LM2_DT */
}

void __init lm2_smp_prepare_cpus(unsigned int max_cpus)
{
//	void __iomem		*cpu1_addr;
	unsigned long long	adr;
	/*
	 * Initialise the present map, which describes the set of CPUs
	 * actually populated at the present time.
	 */
/*
	if (ct_desc)
		ct_desc->smp_enable(max_cpus);
	else
		vexpress_dt_smp_prepare_cpus(max_cpus);
*/
	/*
	 * Write the address of secondary startup into the
	 * system-wide flags register. The boot monitor waits
	 * until it receives a soft interrupt, and then the
	 * secondary CPU branches to this address.
	 */
/*
	v2m_flags_set(virt_to_phys(versatile_secondary_startup));
*/
	misc_base = ioremap(0x05400018, 0x4);
	adr = virt_to_phys(lm2_secondary_startup);
	__raw_writel((adr&0xffffffff), misc_base);
	iounmap(misc_base);
}

struct	smp_operations	__initdata	lm2_smp_ops = {
	.smp_init_cpus	= lm2_smp_init_cpus,
	.smp_prepare_cpus	= lm2_smp_prepare_cpus,
	.smp_secondary_init	= waikiki_secondary_init,
	.smp_boot_secondary	= waikiki_boot_secondary,
#ifdef CONFIG_HOTPLUG_CPU
	.cpu_die		= waikiki_cpu_die,
#endif
};

