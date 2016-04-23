/*
 * hardware.h - local specific hardware detail
 *
 * Copyright 2014 Wind River Systems, Inc.
 */

#ifndef	HARDWARE_H
#define	HARDWARE_H

/* I2C - EEPROM Driver */
struct	lm2_i2c_platdata {
	unsigned long speed;	/* KHz */
};

/*
 * SATA General Purpose Registers
 */
struct	lm2_sata3_gpr {
	unsigned long	tx_preemph_gen1;	/* 0x000 */
	unsigned long	tx_preemph_gen2;
	unsigned long	tx_preemph_gen3;
	unsigned long	los_level;
	unsigned long	tx_amplitude_gen1;	/* 0x010 */
	unsigned long	tx_amplitude_gen2;
	unsigned long	tx_amplitude_gen3;
	unsigned long	los_bias;
	unsigned long	mpll_multiplier;	/* 0x020 */
	unsigned long	clkdiv2;
	unsigned long	ref_ssp_en;
	unsigned long	ssc_en;
	unsigned long	ssc_range;		/* 0x030 */
	unsigned long	ssc_ref_clk_sel;
	unsigned long	ref_use_pad;
	unsigned long	reset;
	unsigned long	test_burnin;		/* 0x040 */
	unsigned long	test_bypass;
	unsigned long	test_powerdown;		/* 0x04c */
	unsigned long	reserve1[36];
	unsigned long	phy_spdmode;		/* 0x070 */
	unsigned long	phy_farafelb;
	unsigned long	phy_nearafelb;
	unsigned long	phy_devslp;
	unsigned long	phy_partial;		/* 0x080 */
	unsigned long	phy_reset;
	unsigned long	phy_rx_err;
	unsigned long	lab_lb_pin;		/* 0x08c */
	unsigned long	reserve2[4];
	unsigned long	ted_testbus;		/* 0x0a0 */
	unsigned long	ted_test_clk_mux;
	unsigned long	reserve3[80];
	unsigned long	a2p_ctrl_reg;		/* 0x100 */
	unsigned long	a2p_fifo_stat_reg;	
};
	
	
/* empty */
#endif	/* HARDWARE_H */
