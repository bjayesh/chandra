/*
 * Copyright (C) 2013, Broadcom Corporation. All Rights Reserved.
 * 
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION
 * OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */
/* Based on arch/arm/mach-realview/headsmp.S */
/*
 *  linux/arch/arm/mach-realview/headsmp.S
 *
 *  Copyright (c) 2003 ARM Limited
 *  All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/linkage.h>
#include <linux/init.h>
#include <mach/iproc_regs.h>
#include <asm/cp15.h>

/*
 * This is called from head.S to fix the u-boot not initializing
 * MMU and cache the way Linux likes it to be. I have talked to
 * u-boot developer to fix it in u-boot. This will remain as a
 * safety. Don't use r1 and r2 as u-boot/CFE may pass some parameters
 * for the kernel.
 * When this code completes execution, we expect the following:
 * MMU = OFF
 * D-Cache = OFF
 * I-Cache = Don't Care
 * regs r0 = 0 r1 = machine id and r2 = atags pointer or 0
 */
	__HEAD
ENTRY(__iproc_head_fixup)
	mov	r12, lr			@ Save the return address
	mrc	p15, 0, r8,  c1, c0, 0	@ Read SCTLR
	mrc	p15, 0, r9,  c1, c0, 1	@ Read ACTLR
	mrc	p15, 0, r10, c2, c0, 0	@ Read TTBR0
	mrc	p15, 0, r11, c2, c0, 1	@ Read TTBR1
	mrc	p15, 0, r7,  c2, c0, 2	@ Read TTBCR
	mov	r0, r8
	bic	r0, #CR_C||CR_M
	mcr	p15, 0, r0, c1, c0, 0	@ Write the control register
	nop
	mrc	p15, 0, r0, c1, c0, 0	@ Read the control register
	bic	r0, #CR_C|CR_A|CR_M|CR_W
	bic	r0, #CR_I|CR_Z
	mcr	p15, 0, r0, c1, c0, 0	@ Write the control register
	nop
	mov	r0, #0
	@ L2 cache controller control register
	ldr	r3, =IPROC_L2CC_REG_BASE
	str	r0, [r3, #0x100]	@ Disable L2 cache
	bl	__v7_invalidate_dcache_all

	mov	r0, #0
	mov	pc, r12			@ Return for regular boot
	nop
ENDPROC(__iproc_head_fixup)

/*
 * iProc specific entry point for secondary CPUs.  This provides
 * a "holding pen" into which all secondary cores are held until we're
 * ready for them to initialise.
 */
#ifdef CONFIG_SMP
ENTRY(iproc_secondary_startup)
	bl	v7_invalidate_l1
	bl	v7_flush_dcache_all
	mrc     p15, 0, r0, c0, c0, 5
	and     r0, r0, #15
	adr     r4, 1f
	ldmia   r4, {r5, r6}
	sub     r4, r4, r5
        add     r6, r6, r4
pen:	ldr     r7, [r6]
	cmp     r7, r0
	bne     pen

	/*
	 * we've been released from the holding pen: secondary_stack
	 * should now contain the SVC stack for this core
	 */
	b    secondary_startup

1:	.long   .
	.long   pen_release

ENDPROC(iproc_secondary_startup)
#endif
