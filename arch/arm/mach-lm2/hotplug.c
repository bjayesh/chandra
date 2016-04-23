/*
 * FujiXerox LM2 Waikiki Motherboard Support
 * Copyright (c) 2013-2014 Wind River Systems, Inc
 * Koki Yamano < koki.yamano@windriver.com >
 * This file is released under the GPLv2
 *
 * arch/arm/mach-realview/hotplug.c
 *  Copyright (C) 2002 ARM Ltd.
 *  All Rights Reserved
 *
 */
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/smp.h>

#include <asm/cacheflush.h>
#include <asm/smp_plat.h>
#include <asm/cp15.h>

//#define	LM2_PM_DEBUG

extern void lm2_wfi1(void);
extern void lm2_save_a15core(void);

static inline void cpu_enter_lowpower(void)
{
	unsigned int v;
#ifdef	LM2_PM_DEBUG
printk(KERN_ERR "== cpu_enter_lowpower:\n");
#endif
	asm volatile(
	"mrc    p15, 0, %0, c1, c0, 0\n"
	"       orr     %0, %0, %1\n"
	"       mcr     p15, 0, %0, c1, c0, 0\n"
	"       mrc     p15, 0, %0, c1, c0, 1\n"
	"       orr     %0, %0, %2\n"
	"       mcr     p15, 0, %0, c1, c0, 1\n"
	  : "=&r" (v)
	  : "Ir" (CR_C), "Ir" (0x40)
	  : "cc");
}

static inline void cpu_leave_lowpower(void)
{
	unsigned int v;
#ifdef	LM2_PM_DEBUG
printk(KERN_ERR "== cpu_leave_lowpower:\n");
#endif

	asm volatile(
		"mrc	p15, 0, %0, c1, c0, 0\n"
	"	orr	%0, %0, %1\n"
	"	mcr	p15, 0, %0, c1, c0, 0\n"
	"	mrc	p15, 0, %0, c1, c0, 1\n"
	"	orr	%0, %0, %2\n"
	"	mcr	p15, 0, %0, c1, c0, 1\n"
	  : "=&r" (v)
	  : "Ir" (CR_C), "Ir" (0x40)
	  : "cc");
}

static inline void platform_do_lowpower(unsigned int cpu, int *spurious)
{
#ifdef	LM2_PM_DEBUG
printk(KERN_ERR "==%d platform_do_lowpower: wfi call\n",read_cpuid_mpidr()&0xff);
#endif
	/*
	 * there is no power-control hardware on this platform, so all
	 * we can do is put the core into WFI; this is safe as the calling
	 * code will have already disabled interrupts
	 */
	lm2_save_a15core();
	for (;;) {
		lm2_wfi1();

		if (pen_release == cpu_logical_map(cpu)) {
			/*
			 * OK, proper wakeup, we're done
			 */
			break;
		}

		/*
		 * Getting here, means that we have come out of WFI without
		 * having been woken up - this shouldn't happen
		 *
		 * Just note it happening - when we're woken, we can report
		 * its occurrence.
		 */
		(*spurious)++;
	}
#ifdef	LM2_PM_DEBUG
printk(KERN_ERR "==%d platform_do_lowpower: wfi end(pen_release=%d)\n",read_cpuid_mpidr()&0xff,pen_release);
#endif
}

/*
 * platform-specific code to shutdown a CPU
 *
 * Called with IRQs disabled
 */
void __ref waikiki_cpu_die(unsigned int cpu)
{
	int spurious = 0;
#ifdef	LM2_PM_DEBUG
printk(KERN_ERR "==%d %s: cpu=%d\n",read_cpuid_mpidr()&0xff, __func__, cpu);
#endif

	/*
	 * we're ready for shutdown now, so do it
	 */
	cpu_enter_lowpower();
	platform_do_lowpower(cpu, &spurious);

	/*
	 * bring this CPU back into the world of cache
	 * coherency, and then restore interrupts
	 */
	cpu_leave_lowpower();

	if (spurious)
		pr_warn("CPU%u: %u spurious wakeup calls\n", cpu, spurious);
}
