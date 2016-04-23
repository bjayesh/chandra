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

#include <linux/spi/xspi.h>

/* Register offset Address */
/* configuration and clock */
#define	SPI_ERR		0x000
#define	SPI_CLK0	0x010
#define	SPI_CLK1	0x014
#define	SPI_XCFG	0x018	/* not used */

#define	SPI_CST		0x01c

/* operation register */
#define	SPI_CFG		0x020	/* CS0=0x20 CS1=0x24 CS2= 0x28 */

#define	SPI_STAT	0x030
#define	SPI_CMD		0x040	/*  */
#define	SPI_LIT		0x050	/* CS0=0x50 CS1=0x54 CS2=0x58 */
#define	SPI_ROM		0x060	/* not used */
#define	SPI_PROG	0x070	/* not used */

/* DMA control */
#define	SPI_DCTL	0x080
#define	SPI_DCMD	0x084
#define	SPI_DADDR	0x088
#define	SPI_RADDR	0x08c
#define	SPI_FIFO	0x090	/* direct mode used */

/* Interrupt */
#define	SPI_SPIINT	0x0a0	/* Interupt */

#define	SPI_SPIINT_DONE	0x02
#define	SPI_SPIINT_ERR	0x01

/* Boot device */
#define	SPI_ROMB	0x0e0	/* not used */

#define	SPI_WRSR_CMD	1	/* Write Status Register */
#define	SPI_WR_CMD	2	/* Write memory */
#define	SPI_RD_CMD	3	/* Read memory */
#define	SPI_WRDI_CMD	4	/* Write Disable */
#define	SPI_RDST_CMD	5	/* read status */
#define	SPI_WREN_CMD	6	/* Write enable */

#define	SPI_CMD_START	0x80000000
#define	CMD_ARG		0x00000000
#define	CMD_ARG_WD	0x20000000
#define	CMD_RD		0x40000000
#define	CMD_ARG_RD	0x60000000
#define	CMD_LEN		0x07000000

/* target memory type */
#define	TYPE_SRAM	0
#define	TYPE_FRAM	1
#define	TYPE_MRAM	2

struct xspi_dev {
	struct mutex		lock;
	struct completion	done;
	volatile void __iomem	*reg_base;
	int			irq;

	int			mem_type;
	int			len;
	u8			*tx_buf;
	u8			*rx_buf;
	int			stat;
	struct clk		*clk;
};

static struct xspi_dev	*xspi_api_lib = NULL;

static	inline	u32 xspi_rd(struct xspi_dev *dev, unsigned int reg)
{
	return	readl(dev->reg_base + reg);
}

static	inline	void	xspi_wr(struct xspi_dev *dev, unsigned int reg, u32 val)
{
//	*((unsigned int *)(dev->reg_base + reg)) = val;
//	writel(val, dev->reg_base + reg);
	__raw_writel(val, dev->reg_base + reg);
	barrier();
}

static	void	xspi_write_disable(struct xspi_dev *dev, int cs)
{
	u32	reg;

	reg = xspi_rd(dev, (SPI_CFG + cs ));
	reg = reg & 0xfffffff0;
	reg = reg | 0x00000007;
	xspi_wr(dev, (SPI_CFG + cs), reg);

//	init_completion(&dev->done);

	reg = SPI_CMD_START | (SPI_WRDI_CMD << 16);	/* 0x80040000 */
	xspi_wr(dev, (SPI_LIT + cs), reg);	

	while(xspi_rd(dev , SPI_CST) & 0xc000){};
//	wait_for_completion(&dev->done);
	
	return;
}

static	void	xspi_write_enable(struct xspi_dev *dev, int cs)
{
	u32	reg;

	reg = xspi_rd(dev, (SPI_CFG + cs));
	reg = reg & 0xfffffff0;
	reg = reg | 0x00000007;
	xspi_wr(dev, (SPI_CFG + cs), reg);

//	init_completion(&dev->done);

	reg = SPI_CMD_START | (SPI_WREN_CMD << 16);	/* 0x80060000 */
	xspi_wr(dev, (SPI_LIT + cs), reg);	
	
	while(xspi_rd(dev , SPI_CST) & 0xc000){};
//	wait_for_completion(&dev->done);

	return;
}

static	void	xspi_clear_fifo(struct xspi_dev *dev)
{
	int	cnt;

	for(cnt = 0 ; cnt < 0x20 ; cnt++ ){
		xspi_wr(dev, SPI_FIFO, 0x00000000);
	}
	xspi_wr(dev, SPI_DCTL, 0x00000180);
}

static	irqreturn_t	xspi_interrupt(int irq, void *dev_id)
{
	struct spi_master	*master = dev_id;
	struct xspi_dev		*dev = spi_master_get_devdata(master);
	u32	stat;

#ifdef	CONFIG_ARCH_LM2 
	stat = 0xffffffff;
#else	/* CONFIG_ARCH_LM2 */
	stat = xspi_rd(dev, SPI_SPIINT);
#endif	/* CONFIG_ARCH_LM2 */
	if(stat & SPI_SPIINT_DONE){
		dev->stat = 0;
	}else{
		dev->stat = stat;
	}
	xspi_wr(dev, SPI_SPIINT, stat);
	complete(&dev->done);
	return	IRQ_HANDLED;
}

static	int	xspi_read_trans(struct xspi_dev *xspi, int adr, int offset)
{
	unsigned int	reg;
	unsigned int	cmd;

	xspi_clear_fifo(xspi);  /* fifo clear */
	/* err */
	xspi_wr(xspi, SPI_ERR, 0x00);

	/* Endian */
	reg = xspi_rd(xspi, SPI_CFG + offset);
	reg |= 0xca000007;
	xspi_wr(xspi, SPI_CFG + offset, reg);

	/* DCTL */
	reg = 0x100;
	xspi_wr(xspi, SPI_DCTL, reg);	/* DMA Off */

	/* DCTL */
	reg = xspi_rd(xspi, SPI_DCTL);
	reg |= 0x80;
	xspi_wr(xspi, SPI_DCTL, reg);	/* FIFO Clear */

	/* SPI Command */
	cmd = (SPI_RD_CMD << 24) | (adr << 16);
	xspi_wr(xspi, SPI_FIFO, cmd);

	/* Operation */
	cmd = 0xe00fc007;
	xspi_wr(xspi,SPI_CMD + offset,cmd);	/* operation start */

	while(xspi_rd(xspi, SPI_CST) & 0xc000){};

	return	(xspi_rd(xspi, SPI_FIFO) >> 24);
}

static	int	xspi_read_trans24(struct xspi_dev *xspi, int adr, int offset)
{
	unsigned int	reg;
	unsigned int	cmd;

	xspi_clear_fifo(xspi);  /* fifo clear */
	/* err */
	xspi_wr(xspi, SPI_ERR, 0x00);

	/* Endian */
	reg = xspi_rd(xspi, SPI_CFG + offset);
	reg |= 0xca000007;
	xspi_wr(xspi, SPI_CFG + offset, reg);

	/* DCTL */
	reg = 0x100;
	xspi_wr(xspi, SPI_DCTL, reg);	/* DMA Off */

	/* DCTL */
	reg = xspi_rd(xspi, SPI_DCTL);
	reg |= 0x80;
	xspi_wr(xspi, SPI_DCTL, reg);	/* FIFO Clear */

	/* SPI Command */
	cmd = (SPI_RD_CMD << 24) | (adr & 0x00ffffff);
	xspi_wr(xspi, SPI_FIFO, cmd);

	/* Operation */
	cmd = 0xe01fc007;
	xspi_wr(xspi, SPI_CMD + offset, cmd);	/* operation start */

	while(xspi_rd(xspi, SPI_CST) & 0xc000){};

	return	(xspi_rd(xspi, SPI_FIFO) >> 24);
}

static	void	xspi_write_trans(struct xspi_dev *xspi, int adr, char dat, int offset)
{
	unsigned int	reg;
	unsigned int	cmd;

	xspi_clear_fifo(xspi);  /* fifo clear */
	/* err */
	xspi_wr(xspi, SPI_ERR, 0x00);

	/* Endian */
	reg = xspi_rd(xspi, SPI_CFG + offset);
	reg |= 0xca000007;
	xspi_wr(xspi, SPI_CFG + offset, reg);

	/* DCTL */
	reg = 0x100;
	xspi_wr(xspi, SPI_DCTL, reg);	/* DMA Off */

	/* DCTL */
	reg = xspi_rd(xspi, SPI_DCTL);
	reg |= 0x80;
	xspi_wr(xspi, SPI_DCTL, reg);	/* FIFO Clear */

	/* SPI Command */
	cmd = (SPI_WR_CMD << 24) | (adr << 16) | dat << 8;
//	dev_info(&spi->dev, "command = %x %x\n", cmd,adr);
	xspi_wr(xspi, SPI_FIFO, cmd);

	/* Operation */
	cmd = 0xa017c000;	/* 3byte cmd adr dat no read */
	xspi_wr(xspi,SPI_CMD + offset,cmd);	/* operation start */

	while(xspi_rd(xspi, SPI_CST) & 0xc000){};

	return;
}

static	void	xspi_write_trans24(struct xspi_dev *xspi, int adr, char dat, int offset)
{
	unsigned int	reg;
	unsigned int	cmd;

	xspi_clear_fifo(xspi);  /* fifo clear */
	/* err */
	xspi_wr(xspi, SPI_ERR, 0x00);

	/* Endian */
	reg = xspi_rd(xspi, SPI_CFG + offset);
	reg |= 0xca000007;
	xspi_wr(xspi, SPI_CFG + offset, reg);

	/* DCTL */
	reg = 0x100;
	xspi_wr(xspi, SPI_DCTL, reg);	/* DMA Off */

	/* DCTL */
	reg = xspi_rd(xspi, SPI_DCTL);
	reg |= 0x80;
	xspi_wr(xspi, SPI_DCTL, reg);	/* FIFO Clear */

	/* SPI Command */
	cmd = (SPI_WR_CMD << 24) | (adr &0x00ffffff);
	xspi_wr(xspi, SPI_FIFO, cmd);
	cmd = dat << 24;
	xspi_wr(xspi, SPI_FIFO, cmd);

	/* Operation */
	cmd = 0xa027c000;	/* 5byte cmd adr dat no read */
	xspi_wr(xspi,SPI_CMD + offset,cmd);	/* operation start */

	while(xspi_rd(xspi, SPI_CST) & 0xc000){};

	return;
}

#define	DIR_BUSY	0x00000000
#define	DIR_WRITE	0x20000000
#define	DIR_READ	0x40000000
#define	DIR_WR_RD	0x60000000
 
static	int	xspi_start_transfer(struct spi_device *spi,struct spi_transfer *tfr, int addr)
{
	struct xspi_dev	*xspi = spi_master_get_devdata(spi->master);
	char	*buf;
	int	cnt;
	u32	reg;

	dev_info(&spi->dev, "xspi_start_transfer\n");
	dev_info(&spi->dev, "tfr->len %d\n",tfr->len);
	dev_info(&spi->dev, "tfr->cs_change %d\n",tfr->cs_change);
	dev_info(&spi->dev, "tfr->bit_per_word %d\n",tfr->bits_per_word);
	dev_info(&spi->dev, "tfr->delay_usec %d\n",tfr->delay_usecs);
	dev_info(&spi->dev, "tfr->speed_hz %d\n",tfr->speed_hz);

	xspi_wr(xspi, SPI_ERR, 0x00000000);	/* error clear */
	xspi_clear_fifo(xspi);
	xspi_wr(xspi, SPI_DCTL, 0x00000100);	/* No DMA mode */

	if(tfr->rx_buf == NULL){	/* write */
		buf = (char *)tfr->tx_buf;
		for( cnt = 0 ; cnt < tfr->len ; cnt++ ){
			if(spi->chip_select == 1){
				dev_info(&spi->dev, "### Write CPLD ### %d : %x\n",addr,*buf);
				xspi_write_trans(xspi, addr, *buf, 4);
			}else{
				dev_info(&spi->dev, "### Write MEMORY ### %d : %x\n",addr,*buf);
				xspi_write_trans24(xspi, addr, *buf, 0);
			}
			addr++;
			buf++;
		}
	}else{				/* read */
		buf = (char *)tfr->rx_buf;
		for(cnt = 0 ; cnt < tfr->len ;cnt++){
			if(spi->chip_select == 1){
				*buf = (char)xspi_read_trans(xspi, addr, 4);
				dev_info(&spi->dev, "### read CPLD ### %d : %x\n",addr,*buf);
			}else{
				*buf = (char)xspi_read_trans24(xspi, addr, 0);
				dev_info(&spi->dev, "### read MEMORY ### %d : %x\n",addr,*buf);
			}
			addr++;
			buf++;
		}
	}
	xspi->len = tfr->len;	
	return	0;
}

static	int	xspi_transfer_one(struct spi_master *master, struct spi_message *msg)
{
	struct xspi_dev *xspi = spi_master_get_devdata(master);
	struct spi_transfer	*tfr;
	struct spi_device	*spi = msg->spi;
	int	err,dat;

	dev_info(&spi->dev,"xspi_transfer_one call\n");
	dev_info(&spi->dev,"actual_length=%d\n",msg->actual_length);
	dev_info(&spi->dev,"chip_select=%d\n",spi->chip_select);
	dev_info(&spi->dev,"mode=%d\n",spi->mode);
	dev_info(&spi->dev,"bus_num=%d\n",spi->master->bus_num);

	list_for_each_entry(tfr, &msg->transfers, transfer_list){
		do{
			err = xspi_start_transfer(spi, tfr, msg->actual_length);
			if(err)
				goto out;

//			wait_for_completion(&xspi->done);
//
//			if(xspi->stat){
//				msg->status = -1;
//				goto	out;
//			}
//			if(spi->chip_select != 0){	/* CPLD */
//				err = xspi_rd(xspi, SPI_STAT + 4);
//				dev_info(&spi->dev,"data = %x\n",err);
//				dat = xspi_rd(xspi, SPI_FIFO); 	
//				dev_info(&spi->dev,"data = %x\n",dat);
//			}else{
//
//			}
			msg->actual_length += xspi->len;
		}while(msg->actual_length < tfr->len);
	}
	msg->status = 0;
out:
	spi_finalize_current_message(master);
	return	0;
}

/*
 * driver probing
 */
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

	/* Waikiki clock 300MHz */
	xspi->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(xspi->clk))
		xspi->clk = NULL;
	else
		clk_prepare_enable(xspi->clk);
#if 1
	xspi->irq = platform_get_irq(pdev, 0);
#else
	xspi->irq = irq_of_parse_and_map(pdev->dev.of_node,0);
#endif
	if(xspi->irq <= 0){
		dev_err(&pdev->dev, "could no get IRQ\n");
		goto out_master_put;
	}

	clk_prepare_enable(xspi->clk);

	err = request_irq(xspi->irq, xspi_interrupt, IRQF_DISABLED, "xspi", master);
	if(err){
		dev_err(&pdev->dev, "could not register IRQ\n");
		goto out_master_put;
	}

	dev_info(&pdev->dev, "XSPI I/O %x IRQ %d \n",xspi->reg_base,xspi->irq);
	/* initialize hardware set up */
//	dev_info(&pdev->dev, "SPI_CLK0 %x\n", xspi_rd(xspi, SPI_CLK0));
//	dev_info(&pdev->dev, "SPI_CLK1 %x\n", xspi_rd(xspi, SPI_CLK1));
//	dev_info(&pdev->dev, "SPI0CFG0 %x\n", xspi_rd(xspi, SPI_CFG));
//	dev_info(&pdev->dev, "SPI0CFG1 %x\n", xspi_rd(xspi, SPI_CFG+4));
	/* CS0 :SRAM,FRAM,MRAM */
//	xspi_wr(xspi, SPI_CLK0, 0x00002020); /* SPI mode 0 300MHz/32 */
//	xspi_wr(xspi, SPI_CFG, 0x80000000);

	/* CS1 : CPLD */
//	xspi_wr(xspi, SPI_CLK1, 0x40002020); /* SPI mode 0 300MHz/64 */
//	xspi_wr(xspi, SPI_CFG + 4, 0xc00000e0);

//	err = devm_spi_register_master(&pdev->dev, master);
	err = spi_register_master(master);
	if(err){
		dev_err(&pdev->dev, "could not register SPI Master Driver \n");
		goto out_free_irq;
	}
	dev_info(&pdev->dev, "XSPI set up completed Virt %x \n",xspi->reg_base);

	xspi_api_lib = xspi;	/* SPILIB Static data */

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

/*
 * Driver Function xspi_read
 */
int	xspi_read(SPILIB_PARAM *spiParam)
{
	int	ret = 0;
	int	cs;
	int	count;
	char	*buf;
	u32	adr;

	if(xspi_api_lib == NULL)	return	-1;

	if(spiParam->unit == SPI_UNIT1)	cs = 4;
	else				cs = 0;
	adr = spiParam->offset;
	buf = (char *)spiParam->buf;

	for(count = 0 ; count < spiParam->size ; count++){
		if(spiParam->unit == SPI_UNIT1)
			*buf =xspi_read_trans(xspi_api_lib, adr, cs);
		else
			*buf =xspi_read_trans24(xspi_api_lib, adr, cs);
		buf++;
		adr++;
	}
	return	ret;
}
EXPORT_SYNBOL(xspi_read);

/*
 * Driver Function xspi_write
 */
int	xspi_write(SPILIB_PARAM *spiParam)
{
	int	ret = 0;
	int	cs;
	int	count;
	char	*buf;
	u32	adr;

	if(xspi_api_lib == NULL)	return	-1;

	if(spiParam->unit == SPI_UNIT1)	cs = 4;
	else				cs = 0;

	adr = spiParam->offset;
	buf = (char *)spiParam->buf;

	if(spiParam->unit == SPI_UNIT3){
		/* Write enable */
		xspi_write_enable(xspi_api_lib, cs);
	}
	for(count = 0 ; count < spiParam->size ; count++){
		if(spiParam->unit == SPI_UNIT1)
			xspi_write_trans(xspi_api_lib, adr , *buf, cs);
		else
			xspi_write_trans24(xspi_api_lib, adr , *buf, cs);
		buf++;
		adr++;
	}
	if(spiParam->unit == SPI_UNIT3){
		/* Write disable */
		xspi_write_disable(xspi_api_lib, cs);
	}

	return	ret;
}
EXPORT_SYMBOL(xspi_write);

#ifdef CONFIG_OF
static struct of_device_id xspi_dt_ids[] = {
	{ .compatible = "mmio-xspi" },
	{}
};
MODULE_DEVICE_TABLE(of, xspi_dt_ids);
#endif

#ifdef	CONFIG_CPU_PM
static int xspi_suspend(struct device *dev)
{
        return 0;
}

static int xspi_resume(struct device *dev)
{
        return 0;
}
#endif	/* CONFIG_CPU_PM */

static const struct dev_pm_ops xspi_pm_ops = {
	.suspend        = xspi_suspend,
	.resume         = xspi_resume,
};

static struct platform_driver xspi_driver = {
	.driver = {
		.name	= "mmio-xspi",
		.owner	= THIS_MODULE,
#ifdef	CONFIG_OF
		.of_match_table = of_match_ptr(xspi_dt_ids),
#endif
#ifdef	CONFIG_CPU_PM
		.pm     = &xspi_pm_ops,
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
