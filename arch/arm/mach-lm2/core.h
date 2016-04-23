/*
 * FujiXerox Co., Ltd Waikiki LM2 board
 *	Copyright 2014-2015 Wind River Systems Inc.
 *	koki.yamano@windriver.com
 */
#ifndef	CORE_H
#define	CORE_H

/* 2MB large area for motherboard's peripherals static mapping */
#define V2M_PERIPH 0xf8000000

/* Tile's peripherals static mappings should start here */
#define V2T_PERIPH 0xf8200000

#define	LM2_PERIPHERAL_VIRT	0xff000000
#define	LM2_DEBUG_SERIAL_VIRT	(LM2_PERIPHERAL_VIRT)
#define	LM2_CCI_VIRT		(LM2_PERIPHERAL_VIRT + 0x10000)
#define	LM2_GPIO_VIRT		(LM2_PERIPHERAL_VIRT + 0x20000)

extern	struct	smp_operations	lm2_smp_ops;

void	lm2_dt_smp_map_io(void);

void vexpress_dt_smp_map_io(void);

/*
 * Deivces
 */
extern	int	lm2_sata_register(void);
extern	int	lm2_usb_register(void);
extern	int	lm2_sdhci_init(void);
#ifdef	CONFIG_SPI_XSPI
extern	int	lm2_xspi_register(void);
#endif	/* CONFIG_SPI_XSPI */
#ifdef	CONFIG_LM2_GPDMA
extern	int	lm2_dma_register(void);
#endif
#endif
