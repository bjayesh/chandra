/*
 * FujiXerox Co., Ltd Waikiki LM2 processor board
 * 	Copyright 2014-2015 Wind River Systems Inc.
 * 	koki.yamano@windriver.com
 * 	This source file is released under the GPLv2
 */
#ifndef	LM2_LOCAL_MEMORY_H
#define	LM2_LOCAL_MEMORY_H

/*
 * BSP original macro

#define	LM2_PHYS_MEM_START	0x0000000800000000ULL
#define	LM2_PHYS_MEM_END	0x0000000bffffffffULL

#define	KERNEL_PHYS_OFFSET	0x0000000880000000ULL
*/

/*
 * Linux Standard macro
 */
#if 0	/* yamano debug */
#define	PLAT_PHYS_OFFSET	(0x0000000890000000ULL)
#endif
#endif	/* LM2_LOCAL_MEMORY_H */
