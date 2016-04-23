/*
 * i2c - I2C like a EEPROM driver for the waikiki board
 *
 * Copyright 2014 Wind River Systems, Inc.
 *	base on i2c-sh7760.c
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 *
 */

#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/kernel.h>

#define	DEV_NAME	"seeprom"
#define	DEFAULT_CHIPADR	0xa8
/* register offsets */
#define	EMSR		0x0000	/* EEPROM Mode Setting Register		*/
#define	ESAR		0x0004	/* EEPROM Slave Address Register	*/
#define	EWAR		0x0008	/* EEPROM Word Address Register		*/
#define	EDSTR		0x0018	/* EEPROM Data Status Register		*/
#define	ESTR		0x001C	/* EEPROM Status Register		*/
#define	EECR		0x0020	/* EEPROM Error Clear Register		*/
#define	EWWT		0x0024	/* EEPROM Wait Timer Register		*/
#define	EDR1		0x0040	/* EEPROM Read/Write Data Register 1	*/

/* Misc Interrupt status / Mask register bit */
#define	MISC_INT_STS	0x0040
#define	MISC_INT_MASK	0x0044
#define	MISC_INT_CLEAR	0x0048	/* ??? */


#define	E2P_ACK_ERR	0x00000020
#define	E2P_TRANS_COMP	0x00000010
#define	E2P_TRANS_DONE	1

#define	DEVICE_TYPE	0x000000a0	/* X24CXX */

#define REGSIZE		0xC0

#define DATA_SIZE	32		/* data registers */

struct lm2_i2c_seeprom {
	void __iomem 	*iobase;	/* self register base		*/
	void __iomem	*miscbase;	/* interupt status in CIP	*/
	u32		addr;		/* offset address		*/
	u32		cadr;		/* chip address			*/
	u32		size;
	int		adr_sel;	/* 1byte/2byte address		*/
	unsigned int	len;		/* transaction length		*/
	int		irq;		/* irq				*/
	struct completion	xfer_done;
	int		status;		/* execution status */
	struct mutex	lock;
};

/*
 * local variable
 */
static	int	seeprom_major;	/* Dynamic allocation */
module_param(seeprom_major, int, 0);
MODULE_AUTHOR("Wind River Systems,Inc.");
MODULE_LICENSE("GPL");

static	int	seeprom_devs;
static	struct cdev	seeprom_cdev;
static	struct class	*seeprom_class;
static	struct lm2_i2c_seeprom	*seeprom_device;

/*
 * Interrupt handler
 */
static irqreturn_t lm2_i2c_irq(int irq, void *ptr)
{
	struct lm2_i2c_seeprom *id = ptr;
	unsigned long stat;

	stat = readl(id->miscbase + MISC_INT_STS);
	if((stat & E2P_TRANS_COMP) || (stat & E2P_ACK_ERR)){
		if(stat & E2P_TRANS_COMP){
			/* complete */
			id->status = E2P_TRANS_DONE;
		}
		if(stat & E2P_ACK_ERR){
			id->status = 0;
		}
		stat |= E2P_TRANS_COMP;
		writel(stat, id->miscbase + MISC_INT_CLEAR);

		complete(&id->xfer_done);

		return IRQ_HANDLED;
	}
	return	IRQ_NONE;
}


/* prepare and start a master receive operation */
static void lm2_i2c_mrecv(struct lm2_i2c_seeprom *id)
{
	u32	reg;

//	dev_info(id->adap.dev.parent, "mrecv call\n");
//printk("read transaction \n");
//printk("read count = %d offset address %d chip address %x\n",
//	id->len,id->addr,id->cadr);
	/* set the read count and address width */
	reg = readl(id->iobase + EMSR);
	reg |= ((id->len-1) << 14);
	if(id->adr_sel)
		reg |= (0x1 << 13);

	writel(reg, id->iobase + EMSR);

	/* set chip address */
	writel(id->cadr, id->iobase + ESAR);

	/* set read start address */
	writel(id->addr, id->iobase + EWAR);

	/* start read */
	writel(0x00000001, id->iobase + ESTR);

}

/* prepare and start a master send operation */
static void lm2_i2c_msend(struct lm2_i2c_seeprom *id)
{
	u32	reg;

//	dev_info(id->adap.dev.parent, "msend call\n");

	/* set data length and address byte */
	reg = readl(id->iobase + EMSR);
	reg = reg | ((id->len-1) << 14);
	if(id->adr_sel)
		reg |= (0x1 << 13);

	writel(reg, id->iobase + EMSR);

	/* set chip address */
	writel(id->cadr, id->iobase + ESAR);

	/* set write address */
	writel(id->addr, id->iobase + EWAR);

	/* transfer start */
	writel(0x00000002, id->iobase + ESTR);
}

static inline int lm2_i2c_busy_check(struct lm2_i2c_seeprom *id)
{
	u32	result;

	result = readl(id->iobase + ESTR);
	result &= 0x01;
	return result;
}

#if 0
static int lm2_i2c_master_xfer(struct i2c_adapter *adap,
				  struct i2c_msg *msgs,
				  int num)
{
	struct lm2_i2c_eeprom *id = adap->algo_data;
	int 	i, ret;
	struct	i2c_msg	dat;
	u32	length;
	u32	idx;
	u32	addr;

	if (lm2_i2c_busy_check(id)) {
		dev_err(&adap->dev, "lm2-i2c %d: bus busy!\n", adap->nr);
		return -EBUSY;
	}
//	dev_info( &adap->dev, "call master xfer %x %d\n", msgs, num);
//	dev_info(&adap->dev, "slave addr = %d\n",msgs->addr);
//	dev_info(&adap->dev, "flags = %x\n",msgs->flags);
//	dev_info(&adap->dev, "msg length = %d\n",msgs->len);
//	dev_info(&adap->dev, "buf = %x\n",msgs->buf);
	addr = (msgs->addr)<<1;
	writel(addr,id->iobase + ESAR);
	id->msg = msgs;
	length = msgs->len;
	idx = 0;
	id->addr =0;
	while (length) {
		if(length > 32){
			id->len = 32;
		}else{
			id->len = length;
		}
		/* setup synchronize */
		init_completion(&id->xfer_done);

// dev_info(&adap->dev, "idx = %d length= %d\n", idx, length);
		if (msgs->flags & I2C_M_RD){
			id->addr = idx;
			lm2_i2c_mrecv(id);	/* recv */
		}else{
			for(i = 0 ; i < id->len ; i++){
				writel(msgs->buf[idx],id->iobase+EDR1+i*4);
				idx++;
			}
			lm2_i2c_msend(id);	/* send */
		}

		/* transfer complete wait for isr */
		wait_for_completion(&id->xfer_done);

		if (id->status == 0) {
			num = -EIO;
			break;
		}

		if(msgs->flags & I2C_M_RD) {
			for(i = 0 ; i < id->len ; i++){
				msgs->buf[idx] = (char)readl(id->iobase+EDR1+i*4);
				idx++;
			}
		}
		length = length - id->len;
	}
	return	num;
}
#endif
/*
 * file handle
 */
static	int	seeprom_open(struct inode *inode, struct file *filp)
{
	filp->private_data = seeprom_device;
//	pr_info("open\n");

	return	0;
}

static	int	seeprom_release(struct inode *inode, struct file *filp)
{
	struct lm2_i2c_seeprom	*seeprom = filp->private_data;

//	pr_info("close\n");
	filp->private_data = NULL;

	return	0;
}

static	ssize_t seeprom_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	struct lm2_i2c_seeprom	*seeprom = filp->private_data;
	int	retval = 0;
	char	*kern_buf;
	int	cur_pos;
	int	trans,idx,length;

//	pr_info("# %s count %d offset %lld\n", __func__, count, *f_pos );

	kern_buf = (char *)kmalloc(count, GFP_KERNEL);
	if(!kern_buf){
		pr_err("Could not malloc memory\n");
		return	-ENOMEM;
	}
	if(mutex_lock_interruptible(&seeprom->lock)){
		kfree(kern_buf);
		return -ERESTARTSYS;
	}

	cur_pos = (unsigned int)*f_pos;
	length = count;
	idx = 0;
	while(length > 0){
		if(length > 32 )
			seeprom->len = 32;
		else
			seeprom->len = length;

		seeprom->addr = cur_pos;

                /* setup synchronize */
                init_completion(&seeprom->xfer_done);

		lm2_i2c_mrecv(seeprom);      /* recv */

		/* transfer complete wait for isr */
		wait_for_completion(&seeprom->xfer_done);

		for(trans = 0 ; trans < seeprom->len ; trans++){
			kern_buf[idx] = (char)readl(seeprom->iobase + EDR1 + trans * 4);
			idx++;
		}
		length = length - seeprom->len;
		cur_pos = cur_pos + seeprom->len;
	}

	copy_to_user(buf,kern_buf,count);
	*f_pos = *f_pos + count;

	mutex_unlock(&seeprom->lock);
	kfree(kern_buf);
	return	count;

}

static	ssize_t	seeprom_write(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	struct lm2_i2c_seeprom	*seeprom = filp->private_data;
	char	*kern_buf;
	int	cur_pos;
	int	trans,idx,length;
	
//	pr_info("# %s count %d offset %lld\n", __func__, count, *f_pos);

	kern_buf = kmalloc(count, GFP_KERNEL);
	if(!kern_buf){
		pr_err("Could not malloc memory\n");
		return	-ENOMEM;
	}
	if(mutex_lock_interruptible(&seeprom->lock)){
		kfree(kern_buf);
		return	-ERESTARTSYS;
	}

        copy_from_user(kern_buf, buf, count);

        cur_pos = (unsigned int)*f_pos;
        length = count;
        idx = 0;
        while(length > 0){
                if(length > 32 )
                        seeprom->len = 32;
                else
                        seeprom->len = length;

                seeprom->addr = cur_pos;

                for(trans = 0 ; trans < seeprom->len ; trans++){
			writel(kern_buf[idx], seeprom->iobase + EDR1 + trans * 4);
                        idx++;
                }

                /* setup synchronize */
                init_completion(&seeprom->xfer_done);

                lm2_i2c_msend(seeprom);      /* recv */

                /* transfer complete wait for isr */
                wait_for_completion(&seeprom->xfer_done);

                length = length - seeprom->len;
                cur_pos = cur_pos + seeprom->len;
        }

        *f_pos = *f_pos + count;

	mutex_unlock(&seeprom->lock);
	kfree(kern_buf);

	return	count;
	
}

static	loff_t	seeprom_llseek(struct file *filp, loff_t offset, int origin)
{
	struct lm2_i2c_seeprom * seeprom = filp->private_data;
//	pr_info("lseek\n");

	if(mutex_lock_interruptible(&seeprom->lock)){
		return	-ERESTARTSYS;
	}

	switch(origin){
	case	SEEK_END:
		offset += seeprom->size;
		break;
	case	SEEK_CUR:
		offset += filp->f_pos;
		break;
	}
	if(offset < 0 || offset > seeprom->size){
		offset = -EINVAL;
	}else{
		filp->f_pos = offset;
	}
	mutex_unlock(&seeprom->lock);

	return	offset;
}

static	long	seeprom_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct lm2_i2c_seeprom *seeprom = filp->private_data;
	return	0;
}

static	char	*seeprom_devnode(struct device *dev, umode_t *mode)
{
	*mode = S_IRUGO | S_IWUSR;
	return	kasprintf( GFP_KERNEL, "seeprom/%s",dev_name(dev));
}

struct file_operations seeprom_fops = {
	.owner		= THIS_MODULE,
	.llseek		= seeprom_llseek,
	.open		= seeprom_open,
	.release	= seeprom_release,
	.read		= seeprom_read,
	.write		= seeprom_write,
	.unlocked_ioctl	= seeprom_ioctl,
};


static int lm2_seeprom_probe(struct platform_device *pdev)
{
	struct lm2_i2c_platdata	*pd;
	struct resource		*res;
	struct resource		*misc_res;
	struct lm2_i2c_seeprom	*id;
	int 	ret;
	unsigned int	mask;
	dev_t	dev = MKDEV(seeprom_major,0);

	/* sanity check */
//	dev_info(&pdev->dev, "LM2 I2C EEPROM Driver loading\n");

	/* 2 iomem 1 irq */
	if(pdev->num_resources < 3){
		dev_err(&pdev->dev,"no resource data\n");
		return	-ENODEV;
	}
	/* driver info */
	id = kzalloc(sizeof(struct lm2_i2c_seeprom), GFP_KERNEL);
	if (!id) {
		dev_err(&pdev->dev, "no mem for private data\n");
		ret = -ENOMEM;
		goto out0;
	}

	platform_set_drvdata(pdev, id);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "no mmio resources\n");
		ret = -ENODEV;
		goto out1;
	}

	/* I2C Device base */
	id->iobase = ioremap(res->start, 0x200 );
	if(!id->iobase){
		dev_err(&pdev->dev, "cannot I2C REG ioremap %x \n", res->start);
	}

	/* UICPI Device base */
	misc_res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	id->miscbase = ioremap(misc_res->start, REGSIZE);
	if (!id->miscbase) {
		dev_err(&pdev->dev, "cannot MISC REG ioremap %x \n", misc_res->start);
		ret = -ENODEV;
		goto out2;
	}

	id->irq = platform_get_irq(pdev, 0);

	id->cadr = DEFAULT_CHIPADR;
	id->adr_sel = 0;
	id->size = 256;	/* 8 bit address max byte */

	init_completion(&id->xfer_done);


	if (request_irq(id->irq, lm2_i2c_irq, IRQF_SHARED,
			"lm2-i2c", id)) {
		dev_err(&pdev->dev, "cannot get irq %d\n", id->irq);
		ret = -EBUSY;
		goto out3;
	}

	mask = 0xffffffff;
	writel(mask, id->miscbase + 0x48);

	mask = readl(id->miscbase + 0x44);
	mask &= 0xffffffcf;
//	dev_info(&pdev->dev, "INt MASK %x\n",mask);
	writel(mask, id->miscbase + 0x44);
	mask = readl(id->miscbase + 0x44);
//	dev_info(&pdev->dev, "INt MASK %x\n",mask);

	/* device register */
	if(seeprom_major){
		ret = register_chrdev_region(dev, 1, "seeprom");
	}else{
		ret = alloc_chrdev_region(&dev, 0, 1, "seeprom");
		seeprom_major = MAJOR(dev);
	}
	if(ret < 0 )
		goto	out4;

	cdev_init(&seeprom_cdev, &seeprom_fops);
	seeprom_cdev.owner = THIS_MODULE;
	seeprom_cdev.ops   = &seeprom_fops;
	ret = cdev_add(&seeprom_cdev, dev, 1);
	if(ret < 0)
		goto	out4;

	seeprom_class = class_create(THIS_MODULE, "seeprom");
	if(IS_ERR(seeprom_class)){
		ret = PTR_ERR(seeprom_class);
		goto	fail_cdev;
	}
//	seeprom_class->dev_attrs = seeprom_dev_attrs;
	seeprom_class->devnode = seeprom_devnode;
	mutex_init(&id->lock);

	seeprom_device = id;
//	dev_info(&pdev->dev, "LM2 I2C EEPROM Driver \n");
	return 0;
fail_cdev:

out4:
	free_irq(id->irq, id);
out3:
	iounmap(id->iobase);
out2:
out1:
	kfree(id);
out0:
	return ret;
}

static int lm2_seeprom_remove(struct platform_device *pdev)
{
	struct lm2_i2c_seeprom *id = platform_get_drvdata(pdev);

	free_irq(id->irq, id);
	iounmap(id->iobase);
	iounmap(id->miscbase);
	kfree(id);

	return 0;
}

#ifdef  CONFIG_ARCH_LM2
#define LM2_REGBAK_SIZE 12
static unsigned int     reg_bak[LM2_REGBAK_SIZE];
static unsigned int     reg_bak_chksum;
extern unsigned int     chksum_info;
void i2c_reg_save(void __iomem *base, int *bak_adr, int offset, int size)
{
	int i;
	int adr = *bak_adr;
	for(i=adr; i<(adr+size); i++ ) {
		reg_bak[i] = readl(base + offset);
		offset +=4;
	}
	*bak_adr = i;
}

void i2c_reg_load(void __iomem *base, int *bak_adr, int offset, int size)
{
	int i;
	int adr = *bak_adr;
	
	for(i=adr; i<(adr+size); i++ ) {
		writel( reg_bak[i], base + offset);
		wmb();
		offset +=4;
	}
	*bak_adr = i;
}
static	int lm2_seeprom_suspend(struct platform_device *pdev, pm_message_t state)
{
	int i=0;
	void __iomem *base;

        base = ioremap_nocache(0x041F0000, 0x200);	/* misc:000-0ff eerrom:100-1ff*/
        i2c_reg_save(base, &i, 0x040,  3);
        i2c_reg_save(base, &i, 0x100,  1);
        i2c_reg_save(base, &i, 0x104,  2);
        i2c_reg_save(base, &i, 0x118,  4);
        i2c_reg_save(base, &i, 0x140,  1);
        i2c_reg_save(base, &i, 0x14c,  1);
        iounmap(base);

        /* chksum gen */
	reg_bak_chksum=0;
	for(i=0; i<LM2_REGBAK_SIZE; i++)
		reg_bak_chksum += reg_bak[i];

	return 0;
}

static	int	 lm2_seeprom_resume(struct platform_device *pdev)
{
	int i;
	void __iomem *base;
	unsigned int    tmp;

	/* chksum chk */
	tmp=0;
	for(i=0; i<LM2_REGBAK_SIZE; i++)
		tmp += reg_bak[i];
	if ( tmp != reg_bak_chksum ){
		chksum_info |= 0x400;
	}
	i=0;
        base = ioremap_nocache(0x041F0000, 0x200);	/* misc:000-0ff eerrom:100-1ff*/
        i2c_reg_load(base, &i, 0x040,  3);
        i2c_reg_load(base, &i, 0x100,  1);
        i2c_reg_load(base, &i, 0x104,  2);
        i2c_reg_load(base, &i, 0x118,  4);
        i2c_reg_load(base, &i, 0x140,  1);
        i2c_reg_load(base, &i, 0x14c,  1);
        iounmap(base);
	return	0;
}

#endif  /* CONFIG_ARCH_LM2 */

#ifdef	CONFIG_OF
static	struct of_device_id	seeprom_dt_ids[]={
	{ .compatible = "seeprom" },
	{},
};
#endif

static struct platform_driver lm2_seeprom_drv = {
	.driver	= {
		.name	= "lm2-eeprom",
		.owner	= THIS_MODULE,
#ifdef	CONFIG_OF
		.of_match_table = of_match_ptr(seeprom_dt_ids),
#endif
	},
	.probe		= lm2_seeprom_probe,
	.remove		= lm2_seeprom_remove,
#ifdef  CONFIG_ARCH_LM2
	.suspend        = lm2_seeprom_suspend,
	.resume         = lm2_seeprom_resume,
#endif  /* CONFIG_ARCH_LM2 */
};

/* module_platform_driver(lm2_i2c_drv); */

static	int __init	lm2_seeprom_init(void)
{
	return	platform_driver_register(&lm2_seeprom_drv);
}

static	void __exit	lm2_seeprom_exit(void)
{
	platform_driver_unregister(&lm2_seeprom_drv);
}

module_init(lm2_seeprom_init);
module_exit(lm2_seeprom_exit);

MODULE_DESCRIPTION("LM2 SEEPROM driver");
