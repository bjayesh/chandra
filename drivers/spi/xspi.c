/*
 * xspi.c - lm2 xspi driver
 *
 * Copyright (C) Wind River Systems, Inc.
 *
 * base on sram.c driver
 *
 * Generic on-chip SRAM allocation driver
 *
 * Copyright (C) 2012 Philipp Zabel, Pengutronix
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 */

#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/completion.h>
#include <linux/interrupt.h>
#include <linux/of_irq.h>
#include <linux/of_device.h>
#include <linux/spi/spi.h>
#include <linux/module.h>

/* Register offset Address */
/* configuration and clock */
#define	SPI_ERR		0x00
#define	SPI_CLK0	0x10
#define	SPI_CLK1	0x14
#define	SPI_XCFG	0x18	/* not used */
#define	SPI_CST		0x1c

/* operation register */
#define	SPI_CFG		0x20
#define	SPI_STAT	0x30
#define	SPI_CMD		0x40	/* not used */
#define	SPI_LIT		0x50
#define	SPI_ROM		0x60	/* not used */
#define	SPI_PROG	0x70	/* not used */

/* DMA control */
#define	SPI_DCTL	0x80
#define	SPI_DCMD	0x84
#define	SPI_DADDR	0x88
#define	SPI_RADDR	0x8c
#define	SPI_FIFO	0x90	/* direct mode used */

/* Interrupt */
#define	SPI_SPIINT	0xa0	/* Interupt */
#define	SPI_SPIINT_DONE	0x02
#define	SPI_SPIINT_ERR	0x01

/* Boot device */
#define	SPI_ROMB	0xe0	/* not used */

/* target memory type */
#define	TYPE_SRAM	0
#define	TYPE_FRAM	1
#define	TYPE_MRAM	2

struct xspi_dev {
	struct mutex		lock;
	struct completion	done;
	void __iomem		*reg_base;
	int			irq;

	int			mem_type;
	int			len;
	u8			*tx_buf;
	u8			*rx_buf;
	int			stat;
};

static	inline	u32 xspi_rd(struct xspi_dev *dev, unsigned int reg)
{
	return	readl(dev->reg_base + reg);
}

static	inline	void	xspi_wr(struct xspi_dev * dev, unsigned int reg, u32 val)
{
	writel(val, dev->reg_base + reg);
}

static	irqreturn_t	xspi_interrupt(int irq, void *dev_id)
{
	struct spi_master	*master = dev_id;
	struct xspi_dev		*dev = spi_master_get_devdata(master);
	u32	stat;

	stat = xspi_rd(dev, SPI_SPIINT);
	if(stat & SPI_SPIINT_DONE){
		dev->stat = 1;
	}else{
		dev->stat = 0;
	}
	complete(&dev->done);
	return	IRQ_HANDLED;
}

#define	CMD_START	0x80000000
#define	CMD_LEN		0x07000000
#define	CMD_RD		(0x03<<16)
#define	CMD_WR		(0x02<<16)

static	int	xspi_start_transfer(struct spi_device *spi,struct spi_transfer *tfr)
{
	struct xspi_dev	*xspi = spi_master_get_devdata(spi->master);
	u32	cmd = CMD_START;

	if(spi->chip_select != 0){	/* CPLD */
		if(tfr->tx_buf == NULL)	/* read */
			cmd = cmd | CMD_RD | CMD_LEN;
		else
			cmd = cmd | CMD_WR |*((char *)tfr->tx_buf);

		xspi_wr(xspi->reg_base, SPI_LIT+4, cmd);
	}else{	/* SRAM */
	/*	dev_info("SRAM Access\n");*/
	}
	return	0;
}

static	int	xspi_transfer_one(struct spi_master *master, struct spi_message *msg)
{
	struct xspi_dev *xspi = spi_master_get_devdata(master);
	struct spi_transfer	*tfr;
	struct spi_device	*spi = msg->spi;
	int	err;

	list_for_each_entry(tfr, &msg->transfers, transfer_list){
		err = xspi_start_transfer(spi, tfr);
		if(err)
			goto out;
		wait_for_completion(&xspi->done);
	}
	spi_finalize_current_message(master);
out:
	return	0;
}

static int xspi_probe(struct platform_device *pdev)
{
	struct xspi_dev	*xspi;
	struct resource	*res;
	int		err;
	struct spi_master	*master;

	dev_info(&pdev->dev, "XSPI Driver loading...\n");

	master = spi_alloc_master(&pdev->dev,sizeof(struct xspi_dev));
	if(!master){
		dev_err(&pdev->dev, "spi_alloc_master failed\n");
		return	-ENOMEM;
	}

//	master->dev.of_node = pdev->dev.of_node;
	master->mode_bits = SPI_MODE_0;
//	master->bits_per_word_mask = BIT(8 - 1);
	master->bus_num = pdev->id;
	master->num_chipselect = 2;
	master->transfer_one_message = xspi_transfer_one;
	platform_set_drvdata(pdev, master);

	xspi= spi_master_get_devdata(master);

	init_completion(&xspi->done);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res){
		dev_err(&pdev->dev, "could not io resource\n");
		err = -EINVAL;
		goto out_master_put;
	}

	xspi->reg_base = devm_request_and_ioremap(&pdev->dev, res);
	if (!xspi->reg_base){
		dev_err(&pdev->dev, "could not ioremap\n");
		err = -EINVAL;
		goto out_master_put;
	}

#if 0
	/* Waikiki clock 300MHz */
	xspi->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(xspi->clk))
		xspi->clk = NULL;
	else
		clk_prepare_enable(xspi->clk);
#endif
#if 1
	xspi->irq = platform_get_irq(pdev, 0);
#else
	xspi->irq = irq_of_parse_and_map(pdev->dev.of_node,0);
#endif
	if(xspi->irq <= 0){
		dev_err(&pdev->dev, "could no get IRQ\n");
		goto out_master_put;
	}

#if 0	/* yamano debug */
	clk_prepare_enable(xspi->clk);
#endif	/* yamano debug */

	err = request_irq(xspi->irq, xspi_interrupt, 0, "xspi", master);
	if(err){
		dev_err(&pdev->dev, "could not register IRQ\n");
		goto out_master_put;
	}

	dev_info(&pdev->dev, "XSPI I/O %x IRQ %d \n",xspi->reg_base,xspi->irq);
	/* initialize hardware set up */
	/* CS0 :SRAM,FRAM,MRAM */
	xspi_wr(xspi, SPI_CLK0, 0x00002020); /* SPI mode 0 300MHz/32 */
	xspi_wr(xspi, SPI_CFG, 0x80000000);

	/* CS1 : CPLD */
	xspi_wr(xspi, SPI_CLK1, 0x40002020); /* SPI mode 0 300MHz/64 */
	xspi_wr(xspi, SPI_CFG + 4, 0xc00000e0);

//	err = devm_spi_register_master(&pdev->dev, master);
	err = spi_register_master(master);
	if(err){
		dev_err(&pdev->dev, "could not register SPI Master Driver \n");
		goto out_free_irq;
	}
	dev_info(&pdev->dev, "XSPI set up completed Virt %x ",xspi->reg_base);

	return 0;

	/* error exit */
out_free_irq:
	free_irq(xspi->irq,master);

out_master_put:
	spi_master_put(master);
	return	err;
}

static int xspi_remove(struct platform_device *pdev)
{
	struct xspi_dev *xspi = platform_get_drvdata(pdev);

	dev_info(&pdev->dev, "XSPI device remove\n");
#if 0
	if (gen_pool_avail(sram->pool) < gen_pool_size(sram->pool))
		dev_dbg(&pdev->dev, "removed while SRAM allocated\n");

	gen_pool_destroy(sram->pool);

	if (sram->clk)
		clk_disable_unprepare(sram->clk);
#endif
	
	return 0;
}

#ifdef CONFIG_OF
static struct of_device_id xspi_dt_ids[] = {
	{ .compatible = "mmio-xspi" },
	{}
};
MODULE_DEVICE_TABLE(of, xspi_dt_ids);
#endif

static struct platform_driver xspi_driver = {
	.driver = {
		.name	= "mmio-xspi",
		.owner	= THIS_MODULE,
#ifdef	CONFIG_OF
		.of_match_table = of_match_ptr(xspi_dt_ids),
#endif
	},
	.probe	= xspi_probe,
	.remove	= xspi_remove,
};

static int __init xspi_init(void)
{
	return platform_driver_register(&xspi_driver);
}

module_init(xspi_init);

MODULE_DESCRIPTION("LM2 XSPI driver");
MODULE_LICENSE("GPL");
