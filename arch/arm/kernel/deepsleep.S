/*
 * FujiXerox LM2 Waikiki Motherboard Support
 * Copyright (c) 2013-2014 Wind River Systems, Inc
 * Koki Yamano < koki.yamano@windriver.com >
 * This file is released under the GPLv2
 *
 * linux/arch/arm/kernel/head.S
 */

#define LM2_CHKSUM_ADDR chksum_info

	__HEAD
ENTRY(deepsleep_up)
	/*** set __deepsleep_mode ***/
	ldr	r0, =(__deepsleep_mode)
	/* __virt_to_phys */
	sub	r0, r0, #-1073741824	@ -PAGE_OFFSET
	add	r0, r0, #83886080	@ +PHYS_OFFSET
	/* __virt_to_phys */
	ldr	r1, =(deepsleep_up)
	str	r1, [r0]
	/*** set __deepsleep_mode ***/
	ldr	r2, =(deepsleep_init)
	/* __virt_to_phys */
	sub	r2, r2, #-1073741824	@ -PAGE_OFFSET
	add	r2, r2, #83886080	@ +PHYS_OFFSET
	/* __virt_to_phys */
	mov	pc, r2			@ r2 = deepsleep_init
ENDPROC(deepsleep_up)
	.globl	__deepsleep_mode	@ use arch/arm/mm/proc-v7.S
__deepsleep_mode:
	.long	0x0

	__CPUINIT

ENTRY(deepsleep_init)
	b	chk_chksum
chk_chksum_end:
#ifdef CONFIG_ARM_VIRT_EXT
	bl      __hyp_stub_install_secondary
#endif
	safe_svcmode_maskall r9

	mrc     p15, 0, r9, c0, c0              @ get processor id
	bl      __lookup_processor_type		@ r5=procinfo r9=cpuid
	movs    r10, r5				@ invalid processor (r5=0)?
 THUMB( it      eq )				@ force fixup-able long branch encoding
	beq     __error_p

	ldr	r13, =(deepsleep_switch)
	nop
	adr     lr, BSYM(__enable_mmu)
 ARM(   add     pc, r10, #PROCINFO_INITFUNC     ) @ initialise processor
 THUMB( add     r12, r10, #PROCINFO_INITFUNC    )
 THUMB( mov     pc, r12                         )
ENDPROC(deepsleep_init)

ENTRY(chk_chksum)
	mrc     p15, 0, r1, c0, c0, 5	@ read MPIDR
	and     r1, r1, #0xf
	cmp	r1, #0
	beq     ccc0
	b	chk_chksum_end		@ cpu1 skip
ccc0:
	/** resume_core0_info chksum */
	ldr	r0, =(resume_core0_info)
        /* __virt_to_phys */
        sub     r0, r0, #-1073741824    @ -PAGE_OFFSET
        add     r0, r0, #83886080       @ +PHYS_OFFSET
        /* __virt_to_phys */
	mov	r1, #0x0		@ offset
	mov	r2, #37			@ loop num
	mov	r3, #0x0		@ chksum
chk_loop0:
	sub	r2, #1
	ldr	r4, [r0, r1]
	add	r3,  r4
	add	r1, #4
	cmp	r2, #0
	bne	chk_loop0
	ldr	r2,  [r0, r1]
	cmp	r2, r3
	beq     cc_ok0
	mov	r0, #1
	b	cc_str0
cc_ok0:
	mov	r0, #0
cc_str0:
	ldr     r1, =(LM2_CHKSUM_ADDR)
	str	r0,  [r1]
	/** resume_core1_info chksum */
	ldr	r0, =(resume_core1_info)
        /* __virt_to_phys */
        sub     r0, r0, #-1073741824    @ -PAGE_OFFSET
        add     r0, r0, #83886080       @ +PHYS_OFFSET
        /* __virt_to_phys */
	mov	r1, #0x0		@ offset
	mov	r2, #37			@ loop num
	mov	r3, #0x0		@ chksum
chk_loop1:
	sub	r2, #1
	ldr	r4, [r0, r1]
	add	r3,  r4
	add	r1, #4
	cmp	r2, #0
	bne	chk_loop1
	ldr	r2,  [r0, r1]
	cmp	r2, r3
	beq     cc_ok1
	mov	r0, #2
	b	cc_str1
cc_ok1:
	mov	r0, #0
cc_str1:
	ldr     r1, =(LM2_CHKSUM_ADDR)
	ldr	r2,  [r1]
	add	r0, r2
	str	r0,  [r1]
	b	chk_chksum_end
ENDPROC(chk_chksum)

/*********** phy  ***********************/
/*********** virt ***********************/

ENTRY(deepsleep_switch)
	nop
	mcr	p15, 0, r0, c8, c6, 0	@ DTLBIALL
	nop
	/* select CPU */
	mrc     p15, 0, r0, c0, c0, 5	@ read MPIDR
	and	r0, r0, #0xf
	cmp	r0, #0
	beq     ds0
ds1:
        /* core1 */
	ldr	r0, =(resume_core1_info)
	b	dse
ds0:
	/* core0 */
	ldr	r0, =(resume_core0_info)
dse:
	/* CPSR Resume  */
	mov	r2, #0xd2		@ IRQ_MODE(0x12) | PSR_I_BIT(0x80) | PSR_F_BIT(0x40)
	msr	cpsr_c, r2
        ldr     sp, [r0, #0x34]		@ Load (IRQ_MODE) SP
        ldr     lr, [r0, #0x38]		@ Load (IRQ_MODE) LR
	mov	r2, #0xd7		@ ABT_MODE(0x17) | PSR_I_BIT(0x80) | PSR_F_BIT(0x40)
	msr	cpsr_c, r2
        ldr     sp, [r0, #0x3c]		@ Load (ABT_MODE) SP
        ldr     lr, [r0, #0x40]		@ Load (ABT_MODE) LR
	mov	r2, #0xdb		@ UND_MODE(0x1b) | PSR_I_BIT(0x80) | PSR_F_BIT(0x40)
	msr	cpsr_c, r2
        ldr     sp, [r0, #0x44]		@ Load (UND_MODE) SP
        ldr     lr, [r0, #0x48]		@ Load (UND_MODE) LR
#if 1
	mov	r2, #0xc0		@ USR_MODE(0x10) | PSR_I_BIT(0x80) | PSR_F_BIT(0x40)
	msr	cpsr_c, r2
        ldr     sp, [r0, #0x54]		@ Load (USR_MODE) SP
        ldr     lr, [r0, #0x58]		@ Load (USR_MODE) LR
        ldr     r8, [r0, #0x5c]		@ Load (USR_MODE) R8
        ldr     r9, [r0, #0x60]		@ Load (USR_MODE) R9
        ldr     r10, [r0, #0x64]	@ Load (USR_MODE) R10
        ldr     r11, [r0, #0x68]	@ Load (USR_MODE) R11
        ldr     r12, [r0, #0x6c]	@ Load (USR_MODE) R12
#endif
	mov	r2, #0xd1		@ FIQ_MODE(0x11) | PSR_I_BIT(0x80) | PSR_F_BIT(0x40)
	msr	cpsr_c, r2
        ldr     sp, [r0, #0x70]		@ Load (FIQ_MODE) SP
        ldr     lr, [r0, #0x74]		@ Load (FIQ_MODE) LR
        ldr     r8, [r0, #0x78]		@ Load (FIQ_MODE) R8
        ldr     r9, [r0, #0x7c]		@ Load (FIQ_MODE) R9
        ldr     r10, [r0, #0x80]	@ Load (FIQ_MODE) R10
        ldr     r11, [r0, #0x84]	@ Load (FIQ_MODE) R11
        ldr     r12, [r0, #0x88]	@ Load (FIQ_MODE) R12
	mov	r2, #0xda		@ HYP_MODE(0x1a) | PSR_I_BIT(0x80) | PSR_F_BIT(0x40)
	msr	cpsr_c, r2
        ldr     sp, [r0, #0x8c]		@ Load (HYP_MODE) SP
        ldr     lr, [r0, #0x90]		@ Load (HYP_MODE) LR

	mov	r2, #0xd3		@ SVC_MODE(0x13) | PSR_I_BIT(0x80) | PSR_F_BIT(0x40)
	msr	cpsr_c, r2
        ldr     sp, [r0, #0x4c]		@ Load (SVC_MODE) SP
        ldr     lr, [r0, #0x50]		@ Load (SVC_MODE) LR
	/* C13  Resume */
	ldr	r1, [r0, #0x20]
	mcr	p15, 0, r1, c13, c0, 4	@ TPIDRPRW
	ldr	r1, [r0, #0x24]
	mcr	p15, 0, r1, c13, c0, 0	@ FCSEIDR
	ldr	r1, [r0, #0x28]
	mcr	p15, 0, r1, c13, c0, 1	@ CONTEXTIDR
	ldr	r1, [r0, #0x2c]
	mcr	p15, 0, r1, c13, c0, 2	@ TPIDRURW
	ldr	r1, [r0, #0x30]
	mcr	p15, 0, r1, c13, c0, 3	@ TPIDRURO
	/* Resume CPSR,SP */
	ldr	r1, [r0, #0x08]		@ CPSR
	msr	cpsr, r1
	ldr	sp, [r0, #0x04]		@ SP
	/*****************/
 	ldr	pc, [r0, #0]		@ PC (Jump to __core0_start or __core1_start)
ENDPROC(deepsleep_switch)

	.align

ENTRY(lm2_wfi0)
	stmfd	sp!, {r0-r12, lr}
	ldr	r0, =(resume_core0_info)
	ldr	r1, =(__core0_start)
	str	r1, [r0, #0x00]
	str	sp, [r0, #0x04]
	mrs	r1, cpsr
	str	r1, [r0, #0x08]
	bl	set_chksum
	bl	v7_flush_kern_cache_all
	wfi
__core0_start:
	ldmfd   sp!, {r0-r12, pc}
ENDPROC(lm2_wfi0)

ENTRY(lm2_wfi1)
	stmfd	sp!, {r0-r12, lr}
	ldr	r0, =(resume_core1_info)
	ldr	r1, =(__core1_start)
	str	r1, [r0, #0x00]
	str     sp, [r0, #0x04]
	mrs	r1, cpsr
	str	r1, [r0, #0x08]
	bl	set_chksum
	bl	v7_flush_kern_cache_all
__core1_start:
	wfi
	ldmfd	sp!, {r0-r12, pc}
ENDPROC(lm2_wfi1)


ENTRY(lm2_save_a15core)
	stmfd   sp!, {r0-r3, lr}
	mrc     p15, 0, r3, c0, c0, 5	@ read MPIDR
	and     r3, r3, #0xf
	cmp	r3, #0
	beq     lm2_save_core0
	ldr	r0, =(resume_core1_info)
lm2_save_core1:
	b	lm2_save_end
lm2_save_core0:
	ldr	r0, =(resume_core0_info)
lm2_save_end:
	/* Save C2 */
	mrrc	p15, 0, r1, r2, c2	@ read TTBR0
	str	r2,  [r0, #0x10]
	str	r1,  [r0, #0x14]
	mrrc	p15, 1, r1, r2, c2	@ read TTBR1
	str	r2,  [r0, #0x18]
	str	r1,  [r0, #0x1c]
	mrc	p15, 0, r1, c2, c0, 2	@ read TTBCR
	str	r1,  [r0, #0x0c]
	/* Now Backup CPSR */
        mrs     r1, cpsr
	/* Save SP */
	mov	r2, #0xd2		@ IRQ_MODE(0x12) | PSR_I_BIT(0x80) | PSR_F_BIT(0x40)
	msr	cpsr_c, r2
        str     sp, [r0, #0x34]		@ Save (IRQ_MODE) SP
        str     lr, [r0, #0x38]		@ Save (IRQ_MODE) LR
	mov	r2, #0xd7		@ ABT_MODE(0x17) | PSR_I_BIT(0x80) | PSR_F_BIT(0x40)
	msr	cpsr_c, r2
        str     sp, [r0, #0x3c]		@ Save (ABT_MODE) SP
        str     lr, [r0, #0x40]		@ Save (ABT_MODE) LR
	mov	r2, #0xdb		@ UND_MODE(0x1b) | PSR_I_BIT(0x80) | PSR_F_BIT(0x40)
	msr	cpsr_c, r2
        str     sp, [r0, #0x44]		@ Save (UND_MODE) SP
        str     lr, [r0, #0x48]		@ Save (UND_MODE) LR
#if 1
	mov	r2, #0xc0		@ USR_MODE(0x10) | PSR_I_BIT(0x80) | PSR_F_BIT(0x40)
	msr	cpsr_c, r2
        str     sp, [r0, #0x54]		@ Save (USR_MODE) SP
        str     lr, [r0, #0x58]		@ Save (USR_MODE) LR
        str     r8, [r0, #0x5c]		@ Save (USR_MODE) R8
        str     r9, [r0, #0x60]		@ Save (USR_MODE) R9
        str     r10, [r0, #0x64]	@ Save (USR_MODE) R10
        str     r11, [r0, #0x68]	@ Save (USR_MODE) R11
        str     r12, [r0, #0x6c]	@ Save (USR_MODE) R12
#endif
	mov	r2, #0xd1		@ FIQ_MODE(0x11) | PSR_I_BIT(0x80) | PSR_F_BIT(0x40)
	msr	cpsr_c, r2
        str     sp, [r0, #0x70]		@ Save (FIQ_MODE) SP
        str     lr, [r0, #0x74]		@ Save (FIQ_MODE) LR
        str     r8, [r0, #0x78]		@ Save (FIQ_MODE) R8
        str     r9, [r0, #0x7c]		@ Save (FIQ_MODE) R9
        str     r10, [r0, #0x80]	@ Save (FIQ_MODE) R10
        str     r11, [r0, #0x84]	@ Save (FIQ_MODE) R11
        str     r12, [r0, #0x88]	@ Save (FIQ_MODE) R12
	mov	r2, #0xda		@ HYP_MODE(0x1a) | PSR_I_BIT(0x80) | PSR_F_BIT(0x40)
	msr	cpsr_c, r2
        str     sp, [r0, #0x8c]		@ Save (HYP_MODE) SP
        str     lr, [r0, #0x90]		@ Save (HYP_MODE) LR

	mov	r2, #0xd3		@ SVC_MODE(0x13) | PSR_I_BIT(0x80) | PSR_F_BIT(0x40)
	msr	cpsr_c, r2
        str     sp, [r0, #0x4c]		@ Save (SVC_MODE) SP
        str     lr, [r0, #0x50]		@ Save (SVC_MODE) LR
	/* Save C13 */
	mrc     p15, 0, r3, c13, c0, 4	@ read SVC_MODE TPIDRPRW
        str     r3, [r0, #0x20]		@ Save          TPIDRPRW
	mrc     p15, 0, r3, c13, c0, 0	@ read FCSEIDR
        str     r3, [r0, #0x24]		@ Save FCSEIDR
	mrc     p15, 0, r3, c13, c0, 1	@ read CONTEXTIDR
        str     r3, [r0, #0x28]		@ Save CONTEXTIDR
	mrc     p15, 0, r3, c13, c0, 2	@ read TPIDRURW
        str     r3, [r0, #0x2c]		@ Save TPIDRPRW
	mrc     p15, 0, r3, c13, c0, 3	@ read TPIDRURO
        str     r3, [r0, #0x30]		@ Save TPIDRURO
	/* Resutore CPSR */
	msr	cpsr, r1
	ldmfd   sp!, {r0-r3, pc}
ENDPROC(lm2_save_a15core)

ENTRY(set_chksum)
	stmfd   sp!, {r0-r5, lr}
	mrc     p15, 0, r1, c0, c0, 5	@ read MPIDR
	and     r1, r1, #0xf
	cmp	r1, #0
	beq     set_chksum_core0
	ldr	r0, =(resume_core1_info)
set_chksum_core1:
#if 0	/* Err chk in core1 */
	mov	r5, #1	
#else
	mov	r5, #0	
#endif
	b	set_chksum_end
set_chksum_core0:
	ldr	r0, =(resume_core0_info)
#if 0	/* Err chk in core0 */
	mov	r5, #1	
#else
	mov	r5, #0	
#endif
set_chksum_end:
	mov	r1, #0x0		@ offset
	mov	r2, #37			@ loop num
	mov	r3, #0x0		@ chksum
set_loop:
	sub	r2, #1
	ldr	r4, [r0, r1]
	add	r3,  r4
	add	r1, #4
	cmp	r2, #0
	bne	set_loop
	add	r3, r5
	str	r3,  [r0, r1]
	ldmfd   sp!, {r0-r5, pc}
ENDPROC(set_chksum)


        .globl chksum_info
chksum_info:
        .long   0x0				@ chksum_info

        .globl resume_core0_info
resume_core0_info:
        .long   0x0				@ 0x00 Save PC
        .long   0x0				@ 0x04 Save SP
        .long   0x0				@ 0x08 Save CPSR
        .long   0x0				@ 0x0c Save C2  TTBCR
        .long   0x0				@ 0x10 Save C2  TTBR0
        .long   0x0				@ 0x14 Save C2  TTBR0
        .long   0x0				@ 0x18 Save C2  TTBR1
        .long   0x0				@ 0x1c Save C2  TTBR1
        .long   0x0				@ 0x20 Save C13 TPIDRPRW 
        .long   0x0				@ 0x24 Save C13 FCSEIDR
        .long   0x0				@ 0x28 Save C13 CONTEXTIDR
        .long   0x0				@ 0x2c Save C13 TPIDRURW
        .long   0x0				@ 0x30 Save C13 TPIDRURO
        .long   0x0				@ 0x34 Save (IRQ_MODE) sp
        .long   0x0				@ 0x38 Save (IRQ_MODE) lr
        .long   0x0				@ 0x3c Save (ABT_MODE) sp
        .long   0x0				@ 0x40 Save (ABT_MODE) lr
        .long   0x0				@ 0x44 Save (UND_MODE) sp
        .long   0x0				@ 0x48 Save (UND_MODE) lr
        .long   0x0				@ 0x4c Save (SVC_MODE) sp
        .long   0x0				@ 0x50 Save (SVC_MODE) lr
        .long   0x0				@ 0x54 Save (USR_MODE) sp
        .long   0x0				@ 0x58 Save (USR_MODE) lr
        .long   0x0				@ 0x5c Save (USR_MODE) r8
        .long   0x0				@ 0x60 Save (USR_MODE) r9
        .long   0x0				@ 0x64 Save (USR_MODE) r10
        .long   0x0				@ 0x68 Save (USR_MODE) r11
        .long   0x0				@ 0x6c Save (USR_MODE) r12
        .long   0x0				@ 0x70 Save (FIQ_MODE) sp
        .long   0x0				@ 0x74 Save (FIQ_MODE) lr
        .long   0x0				@ 0x78 Save (FIQ_MODE) r8
        .long   0x0				@ 0x7c Save (FIQ_MODE) r9
        .long   0x0				@ 0x80 Save (FIQ_MODE) r10
        .long   0x0				@ 0x84 Save (FIQ_MODE) r11
        .long   0x0				@ 0x88 Save (FIQ_MODE) r12
        .long   0x0				@ 0x8c Save (HYP_MODE) sp
        .long   0x0				@ 0x90 Save (HYP_MODE) lr
        .long   0x0				@ 0x94 chksum

        .globl resume_core1_info
resume_core1_info:
        .long   0x0				@ 0x00 save PC
        .long   0x0				@ 0x04 save SP
        .long   0x0				@ 0x08 save CPSR
        .long   0x0				@ 0x0c save C2  TTBCR
        .long   0x0				@ 0x10 save C2  TTBR0
        .long   0x0				@ 0x14 save C2  TTBR0
        .long   0x0				@ 0x18 save C2  TTBR1
        .long   0x0				@ 0x1c save C2  TTBR1
        .long   0x0				@ 0x20 save C13 TPIDRPRW
        .long   0x0				@ 0x24 save C13 FCSEIDR
        .long   0x0				@ 0x28 save C13 CONTEXTIDR
        .long   0x0				@ 0x2c save C13 TPIDRURW
        .long   0x0				@ 0x30 save C13 TPIDRURO
        .long   0x0				@ 0x34 save (IRQ_MODE) sp
        .long   0x0				@ 0x38 save (IRQ_MODE) lr
        .long   0x0				@ 0x3c save (ABT_MODE) sp
        .long   0x0				@ 0x40 save (ABT_MODE) lr
        .long   0x0				@ 0x44 save (UND_MODE) sp
        .long   0x0				@ 0x48 save (UND_MODE) lr
        .long   0x0				@ 0x4c save (SVC_MODE) sp
        .long   0x0				@ 0x50 save (SVC_MODE) lr
        .long   0x0				@ 0x54 Save (USR_MODE) sp
        .long   0x0				@ 0x58 Save (USR_MODE) lr
        .long   0x0				@ 0x5c Save (USR_MODE) r8
        .long   0x0				@ 0x60 Save (USR_MODE) r9
        .long   0x0				@ 0x64 Save (USR_MODE) r10
        .long   0x0				@ 0x68 Save (USR_MODE) r11
        .long   0x0				@ 0x6c Save (USR_MODE) r12
        .long   0x0				@ 0x70 Save (FIQ_MODE) sp
        .long   0x0				@ 0x74 Save (FIQ_MODE) lr
        .long   0x0				@ 0x78 Save (FIQ_MODE) r8
        .long   0x0				@ 0x7c Save (FIQ_MODE) r9
        .long   0x0				@ 0x80 Save (FIQ_MODE) r10
        .long   0x0				@ 0x84 Save (FIQ_MODE) r11
        .long   0x0				@ 0x88 Save (FIQ_MODE) r12
        .long   0x0				@ 0x8c Save (HYP_MODE) sp
        .long   0x0				@ 0x90 Save (HYP_MODE) lr
        .long   0x0				@ 0x94 chksum

