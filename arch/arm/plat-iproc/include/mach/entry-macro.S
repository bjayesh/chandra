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
#include <mach/io_map.h>

	.macro	disable_fiq
	.endm

	/*
	 * This is the interrupt handling part of the GIC code -
	 * the base_va exists in a variable, but here is defined
	 * at compile time for effeciency (?)
	 */

	.macro  get_irqnr_preamble, base, tmp
	ldr	\base, =IPROC_GICCPU_VA
	.endm

	/*
	 * Interrupts 0-15 are IPI
	 * 16-31 are local
	 * 32-1020 are global
	 * 1021-1022 are reserved
	 * 1023 is "spurious" (no interrupt)
	 *
	 * Spurious interrupt must be ignored in all events.
	 * When in SMP mode, then IPI interrupts must be ignored here,
	 * amd picked up later with the test_for_ipi macro.
	 * When in SMP mode and local timers are enabled,
	 * the private timer/watchdog interrupt must be ignored here
	 * so it can be handled later in test_for_ltirq routine.
	 *
	 * A simple read from the controller will tell us the number of the
	 * highest priority enabled interrupt.  We then just need to check
	 * whether it is in the range that must be handled.
	 *
	 * Upon return, Z=1 tells to ignore this interrupt
	 */

	.macro	get_irqnr_and_base, irqnr, irqstat, base, tmp

	/* bits 12-10 = src CPU, 9-0 = int # */
	ldr     \irqstat, [\base, #GIC_CPU_INTACK]
	ldr	\tmp, =1021

	bic     \irqnr, \irqstat, #0x1c00

	/* Private timers to call do_local_timer() */
	cmp	\irqnr, #29
	beq	29f
    
	/* SPI to call asm_do_IRQ(); IPI to call do_IPI() */
	cmp	\irqnr, #15
	cmpcc	\irqnr, \irqnr
	cmpne	\irqnr, \tmp
	cmpcs	\irqnr, \irqnr

	/* SPI if NE; IPI (0-15) or private timer (29) if EQ */
29:
	.endm

	@code taken from realview/entry-macro.S
	/* We assume that irqstat (the raw value of the IRQ acknowledge
	 * register) is preserved from the macro above.
	 * If there is an IPI, we immediately signal end of interrupt on the
	 * controller, since this requires the original irqstat value which
	 * we won't easily be able to recreate later.
	 */
	.macro	test_for_ipi, irqnr, irqstat, base, tmp
	bic	\irqnr, \irqstat, #0x1c00
	cmp	\irqnr, #16
	strcc	\irqstat, [\base, #GIC_CPU_EOI]
	cmpcs	\irqnr, \irqnr
	.endm

	.macro	test_for_ltirq, irqnr, irqstat, base,tmp
	bic	\irqnr, \irqstat, #0x1c00
	mov	\tmp, #0
	cmp	\irqnr, #29
	moveq	\tmp, #1
	streq	\irqstat, [\base, #GIC_CPU_EOI]
	cmp	\tmp, #0
	.endm

	.macro  arch_ret_to_user, tmp1, tmp2
	.endm
