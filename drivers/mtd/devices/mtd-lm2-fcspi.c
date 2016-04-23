/*
 * MTD driver for serial (SPI) flash chips via the 
 * Alma Technologies SPI-MEM-CTRL (FCSPI) controller
 *
 * Copyright (c) 2012 Cambridge Silicon Radio Ltd.
 *
 * Some parts are based on m25p80.c by Mike Lavender
 *
 * This code is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/of_platform.h>
#include <linux/mm.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/spi/flash.h>
#include <linux/sched.h>
#include <linux/workqueue.h>
#include <linux/delay.h>

#define DRIVER_NAME "fcspi"
/*#define DEBUG_FCSPI*/
/*#define DEBUG_FCSPI_WRITE*/

#define UBUFFSIZE PAGE_SIZE

/* Define max times to check status register before we give up. */
#define	MAX_READY_WAIT_JIFFIES	(40 * HZ)

#define SUSPND		(1<<0)
#define FCSPIBUSY	(1<<1)
#define RDBUSY		(1<<2)
#define WRBUSY		(1<<3)
#define ERASEBUSY	(1<<4)
#define FCSPIERR	(1<<5)

#define FCSPI_READ_MSG  0
#define FCSPI_WRITE_MSG 1
#define FCSPI_ERASE_MSG 2

#ifdef CONFIG_SOC_QUATRO5500
	/* 55xx has cache-coherency and we assume if it's on
	 * that fcspi will be coherent too
	 */
	#define FCSPI_COHERENT_DMA	(1<<28)
#else
	#define FCSPI_COHERENT_DMA	(0)
#endif

/****************************************************************************/

struct fcspi_ctl_regs {
	volatile u32 fcspi_ctrl;
	volatile u32 fcspi_stat;
	volatile u32 fcspi_accrr0;
	volatile u32 fcspi_accrr1;
	volatile u32 fcspi_accrr2;
	volatile u32 fcspi_ddpm;
	volatile u32 fcspi_rwdata;
	volatile u32 fcspi_ffstat;
	volatile u32 fcspi_defmem;     /* Write only!! */
	volatile u32 fcspi_exaddr;
	volatile u32 fcspi_memspec;
	volatile u32 fcspi_reserved1[53];
	volatile u32 fcspi_cfgram[64]; /* Write only!! */
	volatile u32 fcspi_reserved2[384];
	volatile u32 fcspi_dma_saddr;
	volatile u32 fcspi_dma_faddr;
	volatile u32 fcspi_dma_len;
	volatile u32 fcspi_dma_cst;
	volatile u32 fcspi_dma_debug;
	volatile u32 fcspi_dma_spare;
};

struct fcspi {
	struct platform_device        *pdev;
	struct flash_platform_data    *fdata;
	struct mtd_partition	      *parts;
	struct mtd_info		       mtd;
	struct fcspi_ctl_regs __iomem *regs;
	struct completion              xfer_completion;
	struct workqueue_struct	      *workqueue;
	struct work_struct             work;
	struct list_head               queue;
	spinlock_t                     lock;
	unsigned long                  rregs;
	unsigned long                  rregs_sz;
	u32					irq;
	volatile unsigned              state;
	int                            flags;
	u32                            block_size;
	u32                            sector_size;
	uint8_t                       *buffer;
};

struct fcspi_message {
	struct fcspi                  *ctl;
	int                            type;
	int                            result;
	loff_t                         flash_offset;
	size_t                         len;
	uint8_t                       *buf;
	struct completion              completion;
	struct list_head               queue;
};

static inline struct fcspi *mtd_to_fcspi(struct mtd_info *mtd)
{
	return container_of(mtd, struct fcspi, mtd);
}

#define FCSPI_ERASE_SECTOR 0
#define FCSPI_ERASE_BLOCK  1
#define FCSPI_ERASE_CHIP   2

#define CHIP_NAME_SZ 32

/* Spec sheets usually specify the flash size in megabits, but
 * we want bytes.  This macro converts from megabits to bytes
 */
#define MEGABITS(x) (((1024*1024)/8)*x)

#define NO_CHIP_ERASE  1
#define NO_BLOCK_ERASE 2

struct fcspi_flash_info {
	u32 id;
	char name[CHIP_NAME_SZ];
	u32 size;               /* size of the chip in bytes */
	u32 erase_size;		/* smallest erase size */
	u32 block_size;		/* block erase size, if there is one */
	int flags;
};

static const struct fcspi_flash_info fcspi_info[] = {
/* Spansion */
	{ 0x010216, "s25fl064",  MEGABITS(64), 4096, 64*1024, 0 },
	{ 0x010219, "s25fl256s", MEGABITS(256),4096, 64*1024, 0 },
	{ 0x010220, "s25fl512s", MEGABITS(512), 256*1024, 256*1024, 0 },
/* Eon */
	{ 0x1C3015, "en25d16",   MEGABITS(16), 4096, 64*1024, 0 },
	{ 0x1C2017, "en25b64",   MEGABITS(64), 4096, 64*1024, 0 },
/* Atmel */
	{ 0x1F6604, "at25fs040", MEGABITS(4),  4096, 64*1024, 0 },
/* Numonyx */
	{ 0x207117, "m25px64",   MEGABITS(64), 4096, 64*1024, 0 },
	{ 0x208015, "m25px64",   MEGABITS(16), 4096, 64*1024, 0 },
/* Micron */
	{ 0x20BA20, "n25q512",   MEGABITS(512),4096, 64*1024, 0 },
/* AMIC */
	{ 0x373016, "a25l032",   MEGABITS(32), 4096, 64*1024, 0 },
/* ESMT */
	{ 0x8C2015, "f25l16",    MEGABITS(16), 4096, 64*1024, 0 },
	{ 0x8C4016, "f25l32",    MEGABITS(32), 4096, 64*1024, 0 },
/* Chingis */
	{ 0x9D7F13, "pm25lv080", MEGABITS(8),  4096, 64*1024, 0 },
	{ 0x9D7F14, "pm25lv016", MEGABITS(16), 4096, 64*1024, 0 },
/* Macronix */
	{ 0xC22018, "mx25l128",  MEGABITS(128),4096, 64*1024, 0 },
	{ 0xC22019, "mx25l25735",MEGABITS(256),4096, 64*1024, 0 },
	{ 0xC2201A, "mx66l512",  MEGABITS(512),4096, 64*1024, 0 },
	{ 0xC25E16, "mx25l32",   MEGABITS(32), 4096, 64*1024, 0 },
/* Gigadevice */
	{ 0xC82013, "gd25f40",   MEGABITS(4),  4096, 64*1024, 0 },
	{ 0xC82014, "gd25f80",   MEGABITS(8),  4096, 64*1024, 0 },
	{ 0xC83013, "gd25d40",   MEGABITS(4),  4096, 64*1024, 0 },
	{ 0xC83014, "gd25d80",   MEGABITS(8),  4096, 64*1024, 0 },
/* Winbond */
	{ 0xEF3015, "w25x16",    MEGABITS(16), 4096, 64*1024, 0 },
	{ 0xEF3016, "w25x32",    MEGABITS(32), 4096, 64*1024, 0 },
	{ 0xEF3017, "w25x64",    MEGABITS(64), 4096, 64*1024, 0 },
	{ 0xEF4017, "w25q64",    MEGABITS(64), 4096, 64*1024, 0 },
	{ 0xEF4018, "w25q128",   MEGABITS(128),4096, 64*1024, 0 },
/* SST */
	{ 0xBF254B, "sst25vf064",MEGABITS(64), 4096, 64*1024, 0 },
	{ },
};

static irqreturn_t fcspi_intr(int irq, void *param)
{
	struct fcspi *ctl = param;
	struct fcspi_ctl_regs *regs = ctl->regs;

	writel(1<<24, &regs->fcspi_dma_cst);
	complete(&ctl->xfer_completion);
	return IRQ_HANDLED;
}

static int wait_for_ready(struct fcspi *ctl)
{
	struct fcspi_ctl_regs *regs = ctl->regs;
	unsigned long deadline = jiffies + MAX_READY_WAIT_JIFFIES;

	do {
		if ((readl(&regs->fcspi_stat) & (1<<3)))
			return 0;
		cond_resched();
	} while (!time_after_eq(jiffies, deadline));
	return 1;
}


/*
 * Erase one sector or block or the entire chip at ``offset'' which is any
 * address within the region which should be erased.
 *
 * Returns 0 if successful, non-zero otherwise.
 */
static int erase_region(struct fcspi *ctl, u32 offset, int type)
{
	struct fcspi_ctl_regs *regs = ctl->regs;

#ifdef DEBUG_FCSPI_WRITE
	printk("%s: offset %08x type %d\n", __func__, offset, type);
#endif

	if (wait_for_ready(ctl))
		return 1;

	writel(offset, &regs->fcspi_accrr0);
	writel(type, &regs->fcspi_accrr1);
	writel(2, &regs->fcspi_accrr2);

	return 0;
}

static int fcspi_erase_work(struct fcspi_message *msg)
{
	struct fcspi *ctl = msg->ctl;
	u32 addr,len;

	addr = msg->flash_offset;
	len = msg->len;

#ifdef DEBUG_FCSPI_WRITE
	printk("%s: addr %08x len %08x\n", 
	       __func__, addr, len);
#endif

	if (len == ctl->mtd.size && !(ctl->flags & NO_CHIP_ERASE)) {
		if (erase_region(ctl, 0, FCSPI_ERASE_CHIP)) {
			return -EIO;
		}
		return 0;
	}
	while (len > ctl->block_size && !(ctl->flags & NO_BLOCK_ERASE)) {
		if (erase_region(ctl, addr, FCSPI_ERASE_BLOCK)) {
			return -EIO;
		}
		addr += ctl->block_size;
		len -= ctl->block_size;
	}
	/* "sector"-at-a-time erase */
	while (len) {
		if (erase_region(ctl, addr, FCSPI_ERASE_SECTOR)) {
			return -EIO;
		}
		addr += ctl->sector_size;
		len -= ctl->sector_size;
	}
	return 0;
}

static int fcspi_write_work(struct fcspi_message *msg)
{
	struct fcspi *ctl = msg->ctl;
	struct fcspi_ctl_regs *regs = ctl->regs;
	struct device *dev = &ctl->pdev->dev;
	dma_addr_t tx_dma;
	unsigned long	tx_dma_36;

#ifdef DEBUG_FCSPI_WRITE
	printk("%s: to %llx len %d buf %p\n", __func__, 
           msg->flash_offset, msg->len, msg->buf);
#endif

	if ((msg->flash_offset & 3) || (msg->len & 3) || (u32)(msg->buf) & 3) {
		loff_t aligned_to = msg->flash_offset & ~3;
		unsigned int adj2, adj1 = msg->flash_offset - aligned_to;
		size_t aligned_len = msg->len + adj1;

		adj2 = (4 - (aligned_len & 3)) & 3;
		aligned_len += adj2;
        
		while (msg->len) {
			int copylen;
			int this_len;
			int retval;
			struct fcspi_message this_msg;
			if (aligned_len > UBUFFSIZE) {
				this_len = UBUFFSIZE;
				copylen = this_len;
			} else {
				this_len = aligned_len;
				copylen = this_len - adj1 - adj2;
			}
#ifdef DEBUG_FCSPI_WRITE
			printk("  %s: to %llx len %x buf %p\n", 
			       __func__, msg->flash_offset, 
			       msg->len, msg->buf);
			printk("      aligned_to %llx aligned_len %x "
			       "adj1 %d adj2 %d copylen %x\n",
			       aligned_to, aligned_len, 
			       adj1, adj2, 
			       copylen);
#endif
			*(u32 *)ctl->buffer = 0xffffffff;
			*(u32 *)(ctl->buffer + this_len - 4) = 0xffffffff;
			memcpy(ctl->buffer + adj1, msg->buf, copylen);
			this_msg.ctl = ctl;
			this_msg.type = FCSPI_WRITE_MSG;
			this_msg.result = 0;
			this_msg.flash_offset = aligned_to;
			this_msg.len = this_len;
			this_msg.buf = ctl->buffer;
			retval = fcspi_write_work(&this_msg);
			if (retval)
				return retval;
			msg->buf += copylen;
			msg->len -= copylen;
			aligned_to += this_len;
			adj1 = 0;
		}
		return 0;
	}

	tx_dma = dma_map_single(dev, (void *)msg->buf, msg->len, DMA_TO_DEVICE);
//		dev_err(dev, "dma_map_single Tx %llx\n",tx_dma);
	if (dma_mapping_error(dev, tx_dma)) {
		dev_err(dev, "dma_map_single Tx failed\n");
		return -ENOMEM;
	}

	if (wait_for_ready(ctl))
		return -EIO;

#ifdef DEBUG_FCSPI_WRITE
	printk("    tx_dma %08x\n", __func__, tx_dma);
#endif
	tx_dma_36 = tx_dma >> 4;	/* 36bit addressing */
//		dev_err(dev, "dma_map_single Tx %lx\n",tx_dma_36);
	writel(tx_dma_36, &regs->fcspi_dma_saddr);
	writel(msg->flash_offset, &regs->fcspi_dma_faddr);
	writel(msg->len, &regs->fcspi_dma_len);
//	reinit_completion(&ctl->xfer_completion);
	init_completion(&ctl->xfer_completion);
	writel(FCSPI_COHERENT_DMA | 1<<16 | 1<<4 | 1, &regs->fcspi_dma_cst);
    wait_for_completion(&ctl->xfer_completion);

	dma_unmap_single(dev, tx_dma, msg->len, DMA_TO_DEVICE);

	return 0;
}

static int fcspi_read_work(struct fcspi_message *msg)
{
	struct fcspi *ctl = msg->ctl;
	struct device *dev = &ctl->pdev->dev;
	struct fcspi_ctl_regs *regs = ctl->regs;
	dma_addr_t rx_dma;
	unsigned long	rx_dma_36;

	if ((msg->flash_offset & 3) || (msg->len & 3) || (u32)(msg->buf) & 3) {
		loff_t aligned_from = msg->flash_offset & ~3;
		unsigned int adj2, adj1 = msg->flash_offset - aligned_from;
		size_t aligned_len = msg->len + adj1;

		adj2 = (4 - (aligned_len & 3)) & 3;
		aligned_len += adj2;
		while (msg->len) {
			int copylen;
			int this_len;
			int retval;
			struct fcspi_message this_msg;
#ifdef DEBUG_FCSPI_READ
			printk("%s: from %llx msg->len %x buf %p\n", 
			       __func__, msg->flash_offset, msg->len, msg->buf);
#endif
			if (aligned_len > UBUFFSIZE) {
				this_len = UBUFFSIZE;
				copylen = this_len - adj1;
			} else {
				this_len = aligned_len;
				copylen = this_len - adj1 - adj2;
			}
#ifdef DEBUG_FCSPI_READ
			printk("len %08x aligned_from %llx aligned_len %x "
			       "adj1 %d adj2 %d copylen %x\n",
			       msg->len, aligned_from, aligned_len, adj1, adj2, 
			       copylen);
#endif
			this_msg.ctl = ctl;
			this_msg.type = FCSPI_READ_MSG;
			this_msg.result = 0;
			this_msg.flash_offset = aligned_from;
			this_msg.len = this_len;
			this_msg.buf = ctl->buffer;
			retval = fcspi_read_work(&this_msg);
			if (retval)
				return retval;
			memcpy(msg->buf, ctl->buffer+adj1, copylen);
			adj1 = 0;
			msg->buf += copylen;
			msg->len -= copylen;
			aligned_from += this_len;
		}
		return 0;
	}

	rx_dma = dma_map_single(dev, msg->buf, msg->len, DMA_FROM_DEVICE);
//		dev_err(dev, "dma_map_single Rx %llx\n",rx_dma);
	if (dma_mapping_error(dev, rx_dma)) {
		dev_err(dev, "dma_map_single Rx failed\n");
		return -ENOMEM;
	}

#ifdef DEBUG_FCSPI_READ
	printk("%s: from %llx len %x buf %p rx_dma %08x\n", 
	       __func__, msg->flash_offset, msg->len, msg->buf, rx_dma);
#endif
	if (wait_for_ready(ctl))
		return -EIO;

	rx_dma_36 = rx_dma >> 4;
//		dev_err(dev, "dma_map_single shift rx %lx\n",rx_dma_36);
	writel(msg->flash_offset, &regs->fcspi_dma_faddr);
	writel(rx_dma_36, &regs->fcspi_dma_saddr);
	writel(msg->len, &regs->fcspi_dma_len);

//	reinit_completion(&ctl->xfer_completion);
	init_completion(&ctl->xfer_completion);
	writel(FCSPI_COHERENT_DMA | 1<<16 | 1<<4, &regs->fcspi_dma_cst);
	wait_for_completion(&ctl->xfer_completion);

	dma_unmap_single(dev, rx_dma, msg->len, DMA_FROM_DEVICE);
	/*
	printk("%s: from %llx len %x buf %p %02X %02X %02X %02X\n", 
		__func__, msg->flash_offset, msg->len, msg->buf,
		msg->buf[0], msg->buf[1], msg->buf[2], msg->buf[3]);
	*/
	return 0;
}

static void handle_msg(struct fcspi_message *msg)
{
	switch (msg->type) {
	case FCSPI_READ_MSG:
		msg->result = fcspi_read_work(msg);
		break;
	case FCSPI_WRITE_MSG:
		msg->result = fcspi_write_work(msg);
		break;
	case FCSPI_ERASE_MSG:
		msg->result = fcspi_erase_work(msg);
		break;
	default:
		msg->result = -EINVAL;
		break;
	}
	complete(&msg->completion);
}

static void fcspi_work(struct work_struct *work)
{
	struct fcspi *ctl = container_of(work, struct fcspi, work);
	unsigned long flags;

	spin_lock_irqsave(&ctl->lock, flags);

	while (!list_empty(&ctl->queue)
	       && !(ctl->state & SUSPND)) {

		struct fcspi_message *msg;

		msg = container_of(ctl->queue.next, 
				   struct fcspi_message, queue);

		list_del_init(&msg->queue);

		/* Set Xfer busy flag */
		ctl->state |= FCSPIBUSY;

		spin_unlock_irqrestore(&ctl->lock, flags);

		handle_msg(msg);

		spin_lock_irqsave(&ctl->lock, flags);

		ctl->state &= ~FCSPIBUSY;
	}

	spin_unlock_irqrestore(&ctl->lock, flags);
}


static int fcspi_queue_work(struct fcspi_message *msg)
{
	struct fcspi *ctl = msg->ctl;
	unsigned long flags;

	init_completion(&msg->completion);
	spin_lock_irqsave(&ctl->lock, flags);
	if (ctl->state & SUSPND) {
		spin_unlock_irqrestore(&ctl->lock, flags);
		return -ESHUTDOWN;
	}
	list_add_tail(&msg->queue, &ctl->queue);
	queue_work(ctl->workqueue, &ctl->work);
	spin_unlock_irqrestore(&ctl->lock, flags);
	wait_for_completion(&msg->completion);
	return msg->result;
}

/**
 * fcspi_erase - [MTD Interface] erase block(s)
 * @mtd:	MTD device structure
 * @instr:	erase instruction
 *
 * Erase one or more regions
 */
static int fcspi_erase(struct mtd_info *mtd, struct erase_info *instr)
{
	struct fcspi *ctl = mtd_to_fcspi(mtd);
	struct fcspi_message msg;
	int    result;
	uint32_t rem;

#ifdef DEBUG_FCSPI
	printk("%s: %llx len %lld \n", 
	       __func__, (long long)instr->addr, (long long)instr->len);
#endif

	/* sanity checks */
	if (instr->addr + instr->len > ctl->mtd.size)
		return -EINVAL;
	div_u64_rem(instr->len, mtd->erasesize, &rem);
	if (rem)
		return -EINVAL;

	msg.ctl = ctl;
	msg.type = FCSPI_ERASE_MSG;
	msg.result = 0;
	msg.flash_offset = instr->addr;
	msg.len = instr->len;
	msg.buf = NULL;
	result = fcspi_queue_work(&msg);
	if (result) {
		instr->state = MTD_ERASE_FAILED;
	}
	else {
		instr->state = MTD_ERASE_DONE;
		mtd_erase_callback(instr);
	}
	return result;;
}

/**
 * fcspi_write - [MTD Interface] write to flash part
 * @mtd:	MTD device structure
 * @to:		offset to write to
 * @len:	number of bytes to write
 * @retlen:	pointer to variable to store the number of written bytes
 * @buf:	the data to write
 *
 */
static int fcspi_write(struct mtd_info *mtd, loff_t to, size_t len,
		       size_t *retlen, const uint8_t *buf)
{
	struct fcspi *ctl = mtd_to_fcspi(mtd);
	struct device *dev = &ctl->pdev->dev;
	struct fcspi_message msg;

#ifdef DEBUG_FCSPI
	printk("%s: to %llx len %d buf %p\n", __func__, to, len, buf);
#endif

	if (retlen)
		*retlen = len;

	/* sanity checks */
	if (len == 0)
		return(0);

	if (to + len > ctl->mtd.size) {
		dev_err(dev, "Write request overflow\n");
		return -EINVAL;
	}

	msg.ctl = ctl;
	msg.type = FCSPI_WRITE_MSG;
	msg.result = 0;
	msg.flash_offset = to;
	msg.len = len;
	msg.buf = (uint8_t *)buf;
	return (fcspi_queue_work(&msg));
}


/**
 * fcspi_read - [MTD Interface] read from flash part
 * @mtd:	MTD device structure
 * @from:	offset to read from
 * @len:	number of bytes to read
 * @retlen:	pointer to variable to store the number of read bytes
 * @buf:	the databuffer to put data
 *
 */
static int fcspi_read(struct mtd_info *mtd, loff_t from, size_t len,
		      size_t *retlen, uint8_t *buf)
{
	struct fcspi *ctl = mtd_to_fcspi(mtd);
	struct fcspi_message msg;

	if (retlen)
		*retlen = len;

	if (!len)
		return 0;

	if (from + len > ctl->mtd.size)
		return -EINVAL;

	msg.ctl = ctl;
	msg.type = FCSPI_READ_MSG;
	msg.result = 0;
	msg.flash_offset = from;
	msg.len = len;
	msg.buf = buf;
	return (fcspi_queue_work(&msg));
}

static int __init fcspi_probe(struct platform_device *pdev)
{
	struct fcspi *ctl;
	struct device_node *np = pdev->dev.of_node;
	struct flash_platform_data *fdata;
	struct mtd_part_parser_data	ppdata;
	struct mtd_partition *parts;
	struct mtd_info *mtd;
	volatile u32 __iomem *memspec;
	struct fcspi_flash_info *info;
	unsigned nr_parts;
	int ret = -ENODEV;
	struct resource	*rregs;
	struct resource	*rirq;
	u32 val;

	fdata = dev_get_platdata(&pdev->dev);
	if (fdata == NULL && np == NULL) {
		dev_err(&pdev->dev, "platform_data missing!\n");
		return -ENODEV;
	}
	ppdata.of_node = pdev->dev.of_node;

	/* Check for availability of necessary resource */
	rregs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (rregs == NULL) {
		dev_err(&pdev->dev, "Unable to get SPI MEM resource\n");
		return -ENXIO;
	}

	/* Check for availability of necessary resource */
	rirq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (rirq == NULL) {
		dev_err(&pdev->dev, "Unable to get SPI IRQ resource\n");
		return -ENXIO;
	}

	ctl = devm_kzalloc(&pdev->dev, sizeof(struct fcspi), GFP_KERNEL);
	if (!ctl) {
		dev_err(&pdev->dev, "Can't allocate control structure\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, ctl);

	ctl->irq = rirq->start;
	ctl->pdev = pdev;
	ctl->rregs = rregs->start;
	ctl->rregs_sz = resource_size(rregs);

	init_completion(&ctl->xfer_completion);
	INIT_WORK(&ctl->work, fcspi_work);
	INIT_LIST_HEAD(&ctl->queue);

	ctl->regs = devm_ioremap_resource(&pdev->dev, rregs);
	if (ctl->regs == NULL) {
		dev_err(&pdev->dev, "Unable to remap IO\n");
		ret = -ENXIO;
		goto err1;
	}

	if (devm_request_irq(&pdev->dev, rirq->start, fcspi_intr, 0,
			pdev->name, ctl)) {
		dev_err(&pdev->dev, "Unable to get fcspi IRQ\n");
		ret = -ENXIO;
		goto err2;
	}
	if (fdata) {
		parts = fdata->parts;
		nr_parts = fdata->nr_parts;
	}
	else {
		parts = NULL;
		nr_parts = 0;
	}

	mtd = &ctl->mtd;
	memspec = &ctl->regs->fcspi_memspec;
	val = readl(memspec);
	
	for (info = (void *)fcspi_info; info->id; info++) {
		if (info->id == val) {
			break;
		}
	}
	if (!info->id) {
		dev_err(&pdev->dev, "Unknown flash device %08x\n", val);
		ret =  -ENODEV;
		goto err3;
	}

	/* allocate a buffer for non-aligned accesses */
	if (!(ctl->buffer = kmalloc(UBUFFSIZE, GFP_KERNEL))) {
		dev_err(&pdev->dev, "Can't allocate buffer\n");
		ret = -ENOMEM;
		goto err3;
	}

	ctl->workqueue = 
		create_singlethread_workqueue(DRIVER_NAME);

	if (ctl->workqueue == NULL) {
		dev_err(&pdev->dev, "Unable to create workqueue\n");
		ret = -ENOMEM;
		goto err4;
	}
	ctl->flags = info->flags;
	ctl->sector_size = info->erase_size;
	ctl->block_size = info->block_size;
	mtd->type = MTD_NORFLASH;
	mtd->writesize = 1;
	mtd->flags = MTD_CAP_NORFLASH;
	mtd->_erase = fcspi_erase;
	mtd->_read = fcspi_read;
	mtd->_write = fcspi_write;
	mtd->name = info->name;
	mtd->owner = THIS_MODULE;
	mtd->dev.parent = &pdev->dev;
	mtd->size = info->size;
	mtd->erasesize = info->erase_size;
	if (mtd->erasesize < 16 * 1024)
		mtd->erasesize = 16 * 1024;

	spin_lock_init(&ctl->lock);

	if (!mtd_device_parse_register(mtd, NULL, &ppdata,
			parts, nr_parts)) {
		return 0;
	}

err4:
	kfree(ctl->buffer);
err3:
	free_irq(ctl->irq, ctl);
	iounmap(ctl->regs);
err2:
	release_mem_region(rregs->start, resource_size(rregs));
err1:
	kfree(ctl);
	platform_set_drvdata(pdev, NULL);
	return ret;
}

static int fcspi_remove(struct platform_device *pdev)
{
	struct fcspi *ctl;
	unsigned long flags;
	printk(KERN_INFO "%s\n", __func__);
	ctl = platform_get_drvdata(pdev);

	spin_lock_irqsave(&ctl->lock, flags);
	ctl->state |= SUSPND;
	spin_unlock_irqrestore(&ctl->lock, flags);

	while (ctl->state & FCSPIBUSY)
		msleep(10);

	destroy_workqueue(ctl->workqueue);
	free_irq(ctl->irq, ctl);
	iounmap(ctl->regs);
	release_mem_region(ctl->rregs, ctl->rregs_sz);
	kfree(ctl->buffer);
	kfree(ctl);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static int fcspi_suspend(struct device *dev)
{
	printk(KERN_INFO "%s\n", __func__);
	return 0;
}

static int fcspi_resume(struct device *dev)
{
	printk(KERN_INFO "%s\n", __func__);
	return 0;
}

static SIMPLE_DEV_PM_OPS(csr_fcspi_pm_ops, fcspi_suspend, fcspi_resume);

static const struct of_device_id csr_fcspi_id_table[] = {
	{ .compatible = "csr,fcspi" },
	{}
};
MODULE_DEVICE_TABLE(of, csr_fcspi_id_table);

static struct platform_driver csr_fcspi_driver = {
	.driver = {
		.name = "fcspi",
		.bus = &platform_bus_type,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(csr_fcspi_id_table),
		.pm = &csr_fcspi_pm_ops,
	},
	.probe = fcspi_probe,
	.remove = fcspi_remove,
};
module_platform_driver(csr_fcspi_driver);

MODULE_AUTHOR("Cambridge Silicon Radio Ltd.");
MODULE_DESCRIPTION("MTD SPI driver for FCSPI controller");
MODULE_LICENSE("GPL v2");
