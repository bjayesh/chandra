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
#include <linux/delay.h>

#include <linux/spi/xspi.h>

#undef	XSPI_DEBUG_FUNC
#undef	XSPI_DEBUG_REG
#undef	XSPI_DEBUG_OPE
#undef	XSPI_DEBUG_INT

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

/* offset */
#define PIOSETB		0x000000C4 // 2016.01.20
#define PIORESB		0x000000E4 // 2016.01.20

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

volatile void __iomem   *treg_base; // 2016.01.20

static struct xspi_dev	*xspi_api_lib = NULL;

static	inline	u32 xspi_rd(struct xspi_dev *dev, unsigned int reg)
{
#ifdef	XSPI_DEBUG_REG
u32	data;

data = readl(dev->reg_base + reg);

printk(KERN_ERR "# %s # reg[0x%02x] = '0x%8.8x'\n", __func__, reg, data);

return	data;
#else
return	readl(dev->reg_base + reg);
#endif
}

static	inline	void	xspi_wr(struct xspi_dev *dev, unsigned int reg, u32 val)
{
#ifdef	XSPI_DEBUG_REG
printk(KERN_ERR "# %s # reg[0x%02x] = '0x%8.8x'\n", __func__, reg, val);

writel(val, dev->reg_base + reg);
#else
__raw_writel(val, dev->reg_base + reg);
#endif
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
#if 0
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
#endif
static	int	xspi_read_trans(struct xspi_dev *xspi, int adr, int offset)
{
unsigned int	reg;
unsigned int	cmd;

xspi_clear_fifo(xspi);  /* fifo clear */
/* err */
xspi_wr(xspi, SPI_ERR, 0x00);

/* Endian */
reg = xspi_rd(xspi, SPI_CFG + offset);
//	reg |= 0xca000007;
reg |= 0x08000000;
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
//	reg |= 0xca000007;
reg |= 0x08000000;
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
//	reg |= 0xca000007;
reg |= 0x08000000;
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
//	reg |= 0xca000007;
reg |= 0x08000000;
xspi_wr(xspi, SPI_CFG + offset, reg);

/* DCTL */
reg = 0x100;
xspi_wr(xspi, SPI_DCTL, reg);	/* DMA Off */

/* DCTL */
reg = xspi_rd(xspi, SPI_DCTL);
reg |= 0x80;
xspi_wr(xspi, SPI_DCTL, reg);	/* FIFO Clear */

/* SPI Command */
cmd = (SPI_WR_CMD << 24) | (adr & 0x00ffffff);
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

static	int	xspi_read_prim(struct xspi_dev *xspi,int cs,int width,unsigned int offset,unsigned int length,char *buf, char *rbuf);
static	int	xspi_write_prim(struct xspi_dev *xspi,int cs,int width,unsigned int offset,unsigned int length,char *buf);

static	int	xspi_start_transfer(struct spi_device *spi,struct spi_transfer *tfr, int addr)
{
	struct xspi_dev	*xspi = spi_master_get_devdata(spi->master);
	unsigned char   *buf;

mutex_lock(&xspi->lock);

	if(tfr->rx_buf != NULL)
		xspi_read_prim(xspi, spi->chip_select, tfr->bits_per_word, 0, tfr->len, tfr->tx_buf, tfr->rx_buf);
	else
		xspi_write_prim(xspi, spi->chip_select, tfr->bits_per_word, 0, tfr->len, tfr->tx_buf);

mutex_unlock(&xspi->lock);
return	0;
}

static  int     xspi_start_transfer_tpm(struct spi_device *spi,struct spi_transfer *tfr, int addr)
{
struct xspi_dev *xspi = spi_master_get_devdata(spi->master);
int     cnt;
u32     reg;
unsigned int cmd;
char    wf;
int	i,j,k;	     // 2016.01.20
int	total_wcnt;  // 2016.01.20
int	total_rcnt;  // 2016.01.20
unsigned int data;   // 2016.01.20
const char *tbuf;    // 2016.01.20
char *rbuf;          // 2016.01.20
j=0;                 // 2016.01.20
k=0;                 // 2016.01.20
tbuf = tfr->tx_buf;  // 2016.01.20
rbuf = tfr->rx_buf;  // 2016.01.20

/*
dev_info(&spi->dev, "xspi_start_transfer(TPM)\n");
dev_info(&spi->dev, "tfr->len %d\n",tfr->len);
dev_info(&spi->dev, "tfr->cs_change %d\n",tfr->cs_change);
dev_info(&spi->dev, "tfr->bit_per_word %d\n",tfr->bits_per_word);
dev_info(&spi->dev, "tfr->delay_usec %d\n",tfr->delay_usecs);
dev_info(&spi->dev, "tfr->speed_hz %d\n",tfr->speed_hz);
*/

	reg = 1<<(42-32);                                  // 2016.01.20
	writel(reg, treg_base + PIOSETB);                  // 2016.01.20

        xspi_clear_fifo(xspi);  /* fifo clear */
        /* err */
        xspi_wr(xspi, SPI_ERR, 0x00);
        /* Endian */
        reg = xspi_rd(xspi, SPI_CFG + 8);
        reg |= 0xca000007;
        xspi_wr(xspi, SPI_CFG + 8, reg);

        /* DCTL */
        reg = 0x100;
        xspi_wr(xspi, SPI_DCTL, reg);   /* DMA Off */
        /* DCTL */
        reg = xspi_rd(xspi, SPI_DCTL);
        reg |= 0x80;
        xspi_wr(xspi, SPI_DCTL, reg);   /* FIFO Clear */

	memcpy(&wf, tfr->tx_buf, 1);

	/* Total count */ // 2016.01.20
	if(wf & 0x80) {	
	   total_wcnt = (tfr->len -1);
	   total_rcnt = 1;
	} else {
	   total_wcnt = 4;
	   total_rcnt = (tfr->len -4);
	}

	reg = 1<<(42-32);                 // 2016.01.20
	writel(reg, treg_base + PIORESB); // 2016.01.20

continue_fifo_ope:
 
        /* write */
	xspi_wr(xspi, SPI_DCTL, 0x00000180);

	if(total_wcnt < 65)
	   cnt = total_wcnt;
	else
	   cnt = 64;

	if(total_wcnt !=  0) {
	   for(i = 0;i < cnt; i++) {
		if((i % 4) == 0)
		   data = 0;
		data |= (tbuf[j] << (8 * (3 - (i % 4))));
		if((i % 4) == 3 || j == ((tfr->len) - 2)) {
//		   dev_info(&spi->dev, "### WRITE TPM ### %08x\n",data);
		   xspi_wr(xspi, SPI_FIFO, data);
		}
		j++;
	  }
	} 

        /* Create Start CMD */
	if(wf & 0x80) {
	   if(total_wcnt < 65)
	      cmd = 0xe000c000 | (((total_wcnt * 8)-1) << 16) | ((total_rcnt * 8) - 1); // Write + Status
	   else
	      cmd = 0xa1ffc000; // Write Only
	} else {
	   if(total_wcnt == 0)
	      cmd = 0xc000c000;
	   else if(total_wcnt < 65)
	      cmd = 0xe000c000 | (((total_wcnt*8)-1)<<16);
	   if(total_rcnt < 65)
	      cmd = cmd | ((total_rcnt*8)-1);
	   else
	      cmd = cmd | 0x1ff;
	}
	
	/* Operation start */
        xspi_wr(xspi,SPI_CMD + 8,cmd); 
	while(xspi_rd(xspi, SPI_STAT + 8) & 0x80000000){};	// Y.M

        /* read */
	if(total_rcnt < 65)
	   cnt = total_rcnt;
	else
	   cnt = 64;

	if(wf & 0x80) {
	   if(total_wcnt < 65) {
	      for(i = 0;i < cnt ;i++) {
		 if((i % 4) == 0) {
		   data = xspi_rd(xspi, SPI_FIFO);
//		   dev_info(&spi->dev, "### READ TPM ### %08x\n",data);
		 }
		 rbuf[k] = (data >> ((8 * (3 - (i % 4)))) & 0xff);
		 k++;
	      }
	      total_wcnt = 0;
	      total_rcnt = 0;
	   } else {
	      total_wcnt = total_wcnt - 64;
	   }
	} else {
	   for(i = 0;i < cnt ;i++) {
	      if((i % 4) == 0) {
		 data = xspi_rd(xspi, SPI_FIFO);
//		 dev_info(&spi->dev, "### READ TPM ### %08x\n",data);
	      }	
	      rbuf[k] = (data >> ((8 * (3 - (i % 4)))) & 0xff);
	      k++;
	   }
	   total_wcnt = 0;
	   total_rcnt = total_rcnt - cnt;
	}

	if((total_wcnt + total_rcnt) != 0)
	   goto continue_fifo_ope;

	reg = 1<<(42-32);                                                 // 2016.01.20
	writel(reg, treg_base + PIOSETB);                                 // 2016.01.20

        xspi->len = tfr->len;
        return  0;
}

static	int	xspi_transfer_one(struct spi_master *master, struct spi_message *msg)
{
//	struct xspi_dev *xspi = spi_master_get_devdata(master);
	struct spi_transfer	*tfr;
	struct spi_device	*spi = msg->spi;
	int	err;

#ifdef	XPSI_DEBUG_FUNC
	printk(KERN_ERR "# %s entry\n",__func__);
#endif
//	dev_info(&spi->dev,"xspi_transfer_one call\n");
//	dev_info(&spi->dev,"actual_length=%d\n",msg->actual_length);
//	dev_info(&spi->dev,"chip_select=%d\n",spi->chip_select);
//	dev_info(&spi->dev,"mode=%d\n",spi->mode);
//	dev_info(&spi->dev,"bus_num=%d\n",spi->master->bus_num);

	list_for_each_entry(tfr, &msg->transfers, transfer_list){
		if(spi->chip_select == 2) {
			err = xspi_start_transfer_tpm(spi, tfr, msg->actual_length);
		} else {
			err = xspi_start_transfer(spi, tfr, msg->actual_length);
		}
		if(err)
			goto out;

		msg->actual_length += tfr->len;
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

//	dev_info(&pdev->dev, "XSPI Driver loading...\n");

	master = spi_alloc_master(&pdev->dev,sizeof(struct xspi_dev));
	if(!master){
		dev_err(&pdev->dev, "spi_alloc_master failed\n");
		return	-ENOMEM;
	}
	platform_set_drvdata(pdev, master);

//	master->dev.of_node = pdev->dev.of_node;
	master->mode_bits = SPI_MODE_0;
//	master->bits_per_word_mask = BIT(8 - 1);
	master->bits_per_word_mask = 0;
	master->bus_num = pdev->id;
	master->num_chipselect = 3;
	master->transfer_one_message = xspi_transfer_one;

	xspi= spi_master_get_devdata(master);

	init_completion(&xspi->done);
	mutex_init(&xspi->lock);

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
//	xspi->irq = platform_get_irq(pdev, 0);
#else
//	xspi->irq = irq_of_parse_and_map(pdev->dev.of_node,0);
#endif
//	if(xspi->irq <= 0){
//		dev_err(&pdev->dev, "could no get IRQ\n");
//		goto out_master_put;
//	}

	clk_prepare_enable(xspi->clk);

//	err = request_irq(xspi->irq, xspi_interrupt, IRQF_DISABLED, "xspi", master);
//	if(err){
//		dev_err(&pdev->dev, "could not register IRQ\n");
//		goto out_master_put;
//	}

//	dev_info(&pdev->dev, "XSPI I/O %x IRQ %d \n",xspi->reg_base,xspi->irq);
	/* initialize hardware set up */
	dev_info(&pdev->dev, "SPI_CLK0 %x\n", xspi_rd(xspi, SPI_CLK0));
	dev_info(&pdev->dev, "SPI_CLK1 %x\n", xspi_rd(xspi, SPI_CLK1));
	dev_info(&pdev->dev, "SPI0CFG0 %x\n", xspi_rd(xspi, SPI_CFG));
	dev_info(&pdev->dev, "SPI0CFG1 %x\n", xspi_rd(xspi, SPI_CFG+4));
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
//	dev_info(&pdev->dev, "XSPI set up completed Virt %x \n",xspi->reg_base);

	xspi_api_lib = xspi;	/* SPILIB Static data */

	treg_base = ioremap(0x04080000, 0x100); // 2016.02.19
	if (!treg_base){
                dev_err(&pdev->dev, "could not ioremap : LM2 register\n");
                err = -EINVAL;
                goto out_master_put;
        }

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
//	struct xspi_dev *xspi = platform_get_drvdata(pdev);

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
 * @ cs chip select 0,1,2
 * @ width 1:24bit 0:8bit
 * @ offset device address offset
 * @ length length of read data
 * @ buf pointer of stored read data
 */ 
static	int	xspi_read_prim(
	struct xspi_dev *xspi,
	int cs,
	int width, 
	unsigned int offset,
	unsigned int length,
	char *buf,
	char *rbuf
)
{
	u32	ope,cmd;
	u32	rt_cnt,trans;
	u32	data;
	u32	read_bit,write_bit;
	u32	read_cnt,cur_pos;
	u32	reg_off;
	u32	reg;
	int	shift;
	u32	cnt=0;

#ifdef	XSPI_DEBUG_FUNC
	printk(KERN_ERR "# %s entry cs[%d] Width[%d] length[%d]\n",__func__, cs, width, length);
#endif
	reg_off = cs * 4;	/* cs0 : 0 cs1 : 4 cs2 : 8 */	

	/* SPI Space Enable */
	reg |= 0x80000000;
	if ( cs == 0)
		reg &= 0xbfffffff;
	else
		reg |= 0x40000000;
	/* Chip Select Setup/Hold Time */
	reg &= 0xcfffffff;
	reg |= 0 << 28;
	/* Endian */
	reg |= 0x08000000;
	xspi_wr(xspi, SPI_CFG + reg_off, reg);

	/* SPI DMA Control */
	reg = 0x180;
	xspi_wr(xspi, SPI_DCTL, reg);   /* DMA Off */

	/* transrate mode setting */
	if(width != 8){
		cur_pos = buf[0]<<16 | buf[1]<<8 | buf[2];
		read_cnt = length - 3;
	} else {
		cur_pos = buf[0];
		read_cnt = length - 1;
	}

#ifdef	XSPI_DEBUG_FUNC
	printk(KERN_ERR "# %s read_cnt %d cur_pos 0x%x\n",__func__, read_cnt, cur_pos);
#endif
	/* transaction start */
	while(read_cnt > 0){

		xspi_clear_fifo(xspi);  	/* fifo clear */
		xspi_wr(xspi, SPI_ERR, 0);	/* error clear */

		/* calcurate frame and read cnt at frame
		 * Maximam 64 ( 4 byte * 16 fifos) byte read per frame transcation 
		 */ 
		if(read_cnt > 64) trans = 64;
		else		trans = read_cnt;

		/* command start SPIx Direct Mode command register */
		if(width != 8){
			write_bit = 31;
			ope = (SPI_RD_CMD << 24) | (cur_pos & 0x00ffffff);
		}else{
			write_bit = 15;
			ope = (SPI_RD_CMD << 24) | (cur_pos & 0xff) << 16;
		}
		xspi_wr(xspi, SPI_FIFO, ope);

		/* SPIx Direct Mode Command */
		read_bit = 7;

		/* command register */
		cmd = 0xe000c000 | (write_bit << 16) | read_bit;
		xspi_wr(xspi, SPI_CMD + reg_off, cmd);

		msleep(10);

		/* one read transaction ( a frame read )*/
		data = xspi_rd(xspi, SPI_FIFO);
		rbuf[cnt] = (u8)(data >> (3 * 8));
#ifdef	XSPI_DEBUG_FUNC
		printk(KERN_ERR "# %s  rbuf[%d]=0x%02x \n",__func__,cnt, rbuf[cnt]);
#endif
		cnt++;
#ifdef	XSPI_DEBUG_FUNC
		printk(KERN_ERR "# %s  rt_cnt %d shift %d \n",__func__,rt_cnt,shift);
#endif
		read_cnt--;
		cur_pos++;
	}
	xspi_wr(xspi,SPI_ERR, 0);	/* error clear */
	return	length;
}

static  int     xspi_write_prim(
	struct xspi_dev *xspi,
	int cs,
	int width,
	unsigned int offset,
	unsigned int length,
	char *buf
)
{
        u32     ope,cmd;
        u32     data;
        u32     read_bit,write_bit;
        int     write_cnt,cur_pos;
	u32	reg_off,reg;
	u32	trans,dat_cnt;
	u32	wt_cnt;
	int	shift;

#ifdef	XSPI_DEBUG_FUNC
	printk(KERN_ERR "# %s entry cs[%d] Width[%d] length[%d]\n",__func__, cs, width, length);
	printk(KERN_ERR "# %s buf[%02x %02x %02x] \n",__func__, buf[0], buf[1], buf[2]);
#endif
	reg_off = cs * 4;	/* cs0 : 0 cs1 : 4 cs2 : 8 */	
        
	/* SPI Space Enable */
	reg |= 0x80000000;
	if ( cs == 0)
		reg &= 0xbfffffff;
	else
		reg |= 0x40000000;
	/* Chip Select Setup/Hold Time */
	reg &= 0xcfffffff;
	reg |= 0 << 28;
	/* Endian */
	reg |= 0x08000000;
	xspi_wr(xspi, SPI_CFG + reg_off, reg);

	/* SPI DMA Control */
	reg = 0x180;
	xspi_wr(xspi, SPI_DCTL, reg);   /* DMA Off */

        /* transrate mode setting */
        if(width != 8){
                cur_pos = buf[0]<<16 | buf[1]<<8 | buf[2];
		data    = buf[3];
        } else {
                cur_pos = buf[0];
		data    = buf[1];
	}
       	write_cnt = 1;

	read_bit = 0;

#ifdef	XSPI_DEBUG_FUNC
	printk(KERN_ERR "# %s cur_pos %d data 0x%x\n",__func__, cur_pos, data);
#endif
        /* transaction start */
        while(write_cnt > 0){
                
		xspi_clear_fifo(xspi);  	/* fifo clear */
		xspi_wr(xspi, SPI_ERR, 0);	/* error clear */

                /* calcurate frame and write count at frame
 		 * 8bit address mode
 		 * Maximam 62 ( 2byte command + 2 byte + 4 byte * 15 fifos write
 		 * 24bit address mode
                 * Maximam 60 ( 4byte command + 4 byte * 15 fifos) byte write
                 *  per frame transcation 
                 */
		if(width != 8){
			/* 24 bit */
                	if(write_cnt > 60) trans = 60;
                	else            trans = write_cnt;
			dat_cnt = trans;
                        write_bit = 31;
                        ope = (SPI_WR_CMD << 24) | (cur_pos & 0x00ffffff);
			/* 1st command and data(8bit only) */
                	xspi_wr(xspi,SPI_FIFO,ope);
			/* 2nd command and data(8bit only) */
			write_bit = write_bit + 8;
                        ope = (data&0xff)<<24;
                	xspi_wr(xspi,SPI_FIFO,ope);
		}else{
			/*  8 bit */
                	if(write_cnt > 62) trans = 62;
                	else               trans = write_cnt;
			dat_cnt = trans;
                        write_bit = 15;
                        ope = (SPI_WR_CMD << 24) | (cur_pos & 0xff) << 16;
			trans = 0;
			ope = ope | ((data&0xff) << 8);
			write_bit = write_bit + 8;
			/* 1st command and data(8bit only) */
                	xspi_wr(xspi,SPI_FIFO,ope);
		}
#ifdef	XSPI_DEBUG_FUNC
	printk(KERN_ERR "# %s write_bit %d read_bit %d\n",__func__, write_bit,read_bit);
#endif
                /* set command register */
		cmd = 0xe000c000 | (write_bit << 16) | read_bit;
                xspi_wr(xspi, SPI_CMD + reg_off, cmd);

		msleep(10);

		/* update variables */
                write_cnt--;
                cur_pos++;
        }
        xspi_wr(xspi, SPI_ERR, 0);      /* error clear */
        return  0;
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

	mutex_lock(&xspi_api_lib->lock);
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
	mutex_unlock(&xspi_api_lib->lock);
	return	ret;
}
EXPORT_SYMBOL(xspi_read);

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

	mutex_lock(&xspi_api_lib->lock);

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
	mutex_unlock(&xspi_api_lib->lock);

	return	ret;
}
EXPORT_SYMBOL(xspi_write);

#ifdef	CONFIG_DEBUG_FS
#endif

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
