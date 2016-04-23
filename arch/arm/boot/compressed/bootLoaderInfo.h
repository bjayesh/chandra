/* bootLoaderInfo.h 0.1 -- Header for OSs boot information from bootLoader */
/* Copyright(C)2015 by Fuji Xerox Co., Ltd. All rights reserved. */
/*
DESCRIPTION
*/
/*
Modification History
--------------------
0000, 
*/

#ifndef _BOOT_BOOTINFO_H_
#define _BOOT_BOOTINFO_H_

#define BOOT_BOOTINFO_MAGIC			(0x2CB1)

/* To Linux Kernel */
/* PWBAs 4GB, 2GB, 1GB Memory */
#define BOOT_LINUX_ENTRY			(0x10000000)    /* Entry Point for Linux kernel */
#define BOOT_LINUX_INITRD_ENTRY		(0x30000000)    /* initrd load location */
#define BOOT_LINUX_BOOTPARAM_SIZE	(512)			/* Must be a multiple of 512bytes */
#define BOOT_LINUX_BOOTINFO_ADDR	(BOOT_LINUX_ENTRY-4096)

/* To osIPoC */
#define BOOT_OSIPOC_BOOTINFO_ADDR	(0x00810000-4096) /* Temp: CA7 Entry-4096 */ /*(0x04E00000)*/

/* for TOKIWA */
#define LINUX_KERNEL_LOAD_ADDRESS_TOKIWA	( BOOT_LINUX_ENTRY )
#define LINUX_INITRD_LOAD_ADDRESS_TOKIWA	( BOOT_LINUX_INITRD_ENTRY )
#define TOKIWA_BOOTPARAM_NVRAM		( 0x200 )
#define TOKIWA_MEMCONFIG_SZ_IN_BYTES	(512)

/* Info from BootLoader. */
typedef struct {
    unsigned int  magic;
    unsigned char macaddr[8];
    unsigned int  reserved;
    unsigned long long ramsize;							/* 4GB/2GB/1GB */
    unsigned char bootparam[BOOT_LINUX_BOOTPARAM_SIZE];
    unsigned long initrd_addr;							/* initrdを置くアドレス */
	unsigned long initrd_size;							/* initrdのサイズ[byte] */
	unsigned long firm_addr;							/* OSIPoC のstartアドレス */
	unsigned long firm_size;							/* OSIPoC のサイズ[byte] */
	unsigned long psaver_mode;							/* Cold Boot - 0 / SWOFF復帰 - 1 / CPUOFF復帰 - 2 */
	unsigned long pwba_rev;								/* PWBA Revision */
} boot_infoBootLoader_t;

/* boot_infoMemConfig_t : Copy from SD/EMMC
 */
typedef struct {
    unsigned int  magic;
    unsigned char data[TOKIWA_MEMCONFIG_SZ_IN_BYTES];	/* copied from SD/EMMC Data Poitedd in Compatiblity ID MEMC */
} boot_infoMemConfig_t;

/* boot_infoInSeepRom_t : Copy from SEEPROM
 *   Unit7
 *   Unit4
 */
typedef struct {
    unsigned int  magic;
    unsigned char Unit4[256];	/* copied from SEEPROM Unit4 */
    unsigned char Unit7[256];	/* copied from SEEPROM Unit7 */
} boot_infoInSeepRom_t;

/* boot_infoInNVMem_t : Copy from NVMem
 *   OS Area - 0x00 - 0xFF
 */
typedef struct {
    unsigned int  magic;
    unsigned char OSArea[256];	/* copied from NVMem 0x00-0xFF */
} boot_infoInNVMem_t;

/* Unified Boot Information struct
 */
typedef struct {
	/* Information Generated by bootLoader */
	boot_infoBootLoader_t	bootInfoBootLoader;
	char reserve01[1024-sizeof(boot_infoBootLoader_t)];

	/* Memory Configuration Information on SD/EMMC */
	boot_infoMemConfig_t	bootInfoMemConfig;
	char reserve02[1024-sizeof(boot_infoMemConfig_t)];

	/* Information on SEEPROM */
	boot_infoInSeepRom_t	bootInfoInSeepRom;
	char reserve03[1024-sizeof(boot_infoInSeepRom_t)];

	/* Information on NVMem */
	boot_infoInNVMem_t		bootInfoInNVMem;
	char reserve04[ 512-sizeof(boot_infoInNVMem_t)];

} boot_bootinfo_t, *pboot_bootinfo_t;


#endif /* _BOOT_BOOTINFO_H_ */
