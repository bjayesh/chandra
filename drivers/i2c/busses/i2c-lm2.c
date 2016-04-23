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
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/module.h>

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

struct lm2_i2c_eeprom {
	struct i2c_adapter	adap;
	void __iomem 	*iobase;	/* self register base		*/
	void __iomem	*miscbase;	/* interupt status in CIP	*/
	struct i2c_msg	*msg;		/* transfer data message	*/
	u32		addr;		/* buffer index */
	int		flags;		/* mode flags			*/
	int		status;		/* status 			*/
	int		irq;		/* irq				*/
	struct completion	xfer_done;
	struct resource *ioarea;	/* save resource data		*/
	unsigned int	len;
	unsigned char	offset;
	unsigned char	buf[32];
	struct mutex lock;
};

extern int lm2_board_is_A0(void);

/*
 * Interrupt handler
 */
static irqreturn_t lm2_i2c_irq(int irq, void *ptr)
{
	struct lm2_i2c_eeprom *id = ptr;
	struct i2c_msg *msg = id->msg;
	char *data = msg->buf;
	unsigned long msr, fsr, fier, len, stat;

	stat = readl(id->miscbase + MISC_INT_STS);
	if((stat & E2P_TRANS_COMP) || (stat & E2P_ACK_ERR)){
		if(stat & E2P_TRANS_COMP){
			/* complete */
			id->status = E2P_TRANS_DONE;
		}
		if(stat & E2P_ACK_ERR){
			id->status = 0;
		}
finish:
		stat |= E2P_TRANS_COMP;
		writel(stat, id->miscbase + MISC_INT_CLEAR);

		complete(&id->xfer_done);

		return IRQ_HANDLED;
	}
node:
	return	IRQ_NONE;
}


/* prepare and start a master receive operation */
static void lm2_i2c_mrecv(struct lm2_i2c_eeprom *id)
{
	int len;
	unsigned int	reg;

//	dev_info(id->adap.dev.parent, "mrecv call\n");
	/* set the read count */
	reg = readl(id->iobase + EMSR);
	reg = reg & ~0x0003c000; /* clear */
	reg |= ((id->len-1) << 14);
	writel(reg, id->iobase + EMSR);

	/* set the slave addr reg; otherwise xmit wont work! */
//	writel(id->msg->addr,id->iobase+ESAR);

	/* set read start address */
	writel(id->addr, id->iobase + EWAR);

	/* start read */
	writel(0x00000001,id->iobase + ESTR);

}

/* prepare and start a master send operation */
static void lm2_i2c_msend(struct lm2_i2c_eeprom *id)
{
	u32	val;

//	dev_info(id->adap.dev.parent, "msend call\n");
	val = readl(id->iobase + EMSR);
	val = val & ~0x0003c000; /* clear */
	val = val | ((id->len-1) << 14);
	writel(val, id->iobase+EMSR);

	/* set address */
	writel(id->addr, id->iobase + EWAR);

	/* transfer start */
	writel(0x00000002,id->iobase + ESTR);
}

static inline int lm2_i2c_busy_check(struct lm2_i2c_eeprom *id)
{
	u32	result;
	result = readl(id->iobase+ESTR);
	result &= 0x01;
	return result;
}

static int lm2_i2c_master_xfer(struct i2c_adapter *adap,
				  struct i2c_msg *msgs,
				  int num)
{
	struct lm2_i2c_eeprom *id = adap->algo_data;
	int 	i, ret;
	struct	i2c_msg	dat;
	unsigned int delay_msec = lm2_board_is_A0() ? 50 : 5;

	u32	length;
	u32	idx;
	u32	start;
	u32	addr;

	if (lm2_i2c_busy_check(id)) {
		dev_err(&adap->dev, "lm2-i2c %d: bus busy!\n", adap->nr);
		return -EBUSY;
	}
	mutex_lock(&id->lock);
//	dev_info( &adap->dev, "call master xfer %x %d\n", msgs, num);
//	dev_info(&adap->dev, "slave addr = %d\n",msgs->addr);
//	dev_info(&adap->dev, "flags = %x\n",msgs->flags);
//	dev_info(&adap->dev, "msg length = %d\n",msgs->len);
//	dev_info(&adap->dev, "buf = %x\n",msgs->buf);
	/* FX */
	start = msgs->buf[0];
	msgs++;
	/* FX */
	addr = (msgs->addr)<<1;
	writel(addr,id->iobase + ESAR);
	id->msg = msgs;
	length = msgs->len;
	idx = 0;
	id->addr =0;
	while (length) {
		if(idx == 0 && (start % 16) != 0){
			id->len = 16 - (start % 16);
			if (id->len > length){
				id->len = length;
			}
		} else if(length > 16){
			id->len = 16;
		} else {
			id->len = length;
		}
		/* setup synchronize */
		init_completion(&id->xfer_done);

// dev_info(&adap->dev, "idx = %d length= %d\n", idx, length);
		if (msgs->flags & I2C_M_RD){
			id->addr = start + idx;
			lm2_i2c_mrecv(id);	/* recv */
		}else{
			id->addr = start + idx;
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
		mdelay(delay_msec);
	}
	mutex_unlock(&id->lock);
	return	num;
}

static u32 lm2_i2c_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C;
}

static const struct i2c_algorithm lm2_i2c_algo = {
	.master_xfer	= lm2_i2c_master_xfer,
	.functionality	= lm2_i2c_func,
};

static int lm2_i2c_probe(struct platform_device *pdev)
{
	struct lm2_i2c_platdata	*pd;
	struct resource		*res;
	struct resource		*misc_res;
	struct lm2_i2c_eeprom	*id;
	int 	ret;
	unsigned int	mask;

	/* sanity check */
	dev_info(&pdev->dev, "LM2 I2C EEPROM Driver loading\n");

	/* 2 iomem 1 irq */
	if(pdev->num_resources < 3){
		dev_err(&pdev->dev,"no resource data\n");
		return	-ENODEV;
	}
	/* get clock data */
#if 0
	pd = dev_get_platdata(&pdev->dev);
	if (!pd) {
		dev_err(&pdev->dev, "no platform_data!\n");
		ret = -ENODEV;
		goto out0;
	}
#endif
	/* driver info */
	id = kzalloc(sizeof(struct lm2_i2c_eeprom), GFP_KERNEL);
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
	id->iobase = ioremap(res->start, 0x200 );
	if(!id->iobase){
		dev_err(&pdev->dev, "cannot I2C REG ioremap %x \n", res->start);
	}
#if 0
	id->ioarea = request_mem_region(res->start, REGSIZE, pdev->name);
	if (!id->ioarea) {
		dev_err(&pdev->dev, "mmio already reserved\n");
		ret = -EBUSY;
		goto out1;
	}
#endif
	misc_res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	id->miscbase = ioremap(misc_res->start, REGSIZE);
	if (!id->miscbase) {
		dev_err(&pdev->dev, "cannot MISC REG ioremap %x \n", misc_res->start);
		ret = -ENODEV;
		goto out2;
	}

	id->irq = platform_get_irq(pdev, 0);

	id->adap.nr = pdev->id;
	id->adap.algo = &lm2_i2c_algo;
	id->adap.class = I2C_CLASS_SPD;
	id->adap.retries = 3;
	id->adap.algo_data = id;
	id->adap.dev.parent = &pdev->dev;

	init_completion(&id->xfer_done);

	snprintf(id->adap.name, sizeof(id->adap.name),
		"LM2 I2C at %08lx", (unsigned long)res->start);

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
	dev_info(&pdev->dev, "INt MASK %x\n",mask);
	writel(mask, id->miscbase + 0x44);
	mask = readl(id->miscbase + 0x44);
	dev_info(&pdev->dev, "INt MASK %x\n",mask);

	ret = i2c_add_numbered_adapter(&id->adap);
	if (ret < 0) {
		dev_err(&pdev->dev, "reg adap failed: %d\n", ret);
		goto out4;
	}
	dev_info(&pdev->dev, "I2C Driver Registered iomem = %x misc = %x IRQ = %d\n",
		 id->iobase, id->miscbase, id->irq);
#if 0
	dev_info(&pdev->dev, "EMSR=%x\n",(int)*((int *)(id->iobase+EMSR)));
	dev_info(&pdev->dev, "ESAR=%x\n",(int)*((int *)(id->iobase+ESAR)));
	dev_info(&pdev->dev, "EWAR=%x\n",(int)*((int *)(id->iobase+EWAR)));
	dev_info(&pdev->dev, "EDSTR=%x\n",(int)*((int *)(id->iobase+EDSTR)));
	dev_info(&pdev->dev, "ESTR=%x\n",(int)*((int *)(id->iobase+ESTR)));
	dev_info(&pdev->dev, "EECR=%x\n",(int)*((int *)(id->iobase+EECR)));
	dev_info(&pdev->dev, "EWWT=%x\n",(int)*((int *)(id->iobase+EWWT)));
#endif
	return 0;

out4:
	free_irq(id->irq, id);
out3:
	iounmap(id->iobase);
out2:
	release_resource(id->ioarea);
	kfree(id->ioarea);
out1:
	kfree(id);
out0:
	return ret;
}

static int lm2_i2c_remove(struct platform_device *pdev)
{
	struct lm2_i2c_eeprom *id = platform_get_drvdata(pdev);

	i2c_del_adapter(&id->adap);
	free_irq(id->irq, id);
	iounmap(id->iobase);
	iounmap(id->miscbase);
	release_resource(id->ioarea);
	kfree(id->ioarea);
	kfree(id);

	return 0;
}

#ifdef  CONFIG_ARCH_LM2
#define LM2_REGBAK_SIZE 12
static unsigned int     reg_bak[LM2_REGBAK_SIZE];
static unsigned int     reg_bak_chksum;
extern unsigned int     chksum_info;
static void i2c_reg_save(void __iomem *base, int *bak_adr, int offset, int size)
{
	int i;
	int adr = *bak_adr;
	for(i=adr; i<(adr+size); i++ ) {
		reg_bak[i] = readl(base + offset);
		offset +=4;
	}
	*bak_adr = i;
}
static void i2c_reg_load(void __iomem *base, int *bak_adr, int offset, int size)
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
static int lm2_i2c_suspend(struct platform_device *pdev)
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

static int lm2_i2c_resume(struct platform_device *pdev)
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

}
#endif  /* CONFIG_ARCH_LM2 */

static struct platform_driver lm2_i2c_drv = {
	.driver	= {
		.name	= "lm2-eeprom",
		.owner	= THIS_MODULE,
	},
	.probe		= lm2_i2c_probe,
	.remove		= lm2_i2c_remove,
#ifdef  CONFIG_ARCH_LM2
	.suspend        = lm2_i2c_suspend,
	.resume         = lm2_i2c_resume,
#endif  /* CONFIG_ARCH_LM2 */
};

/* module_platform_driver(lm2_i2c_drv); */

static	int __init	lm2_i2c_init(void)
{
	return	platform_driver_register(&lm2_i2c_drv);
}

static	void __exit	lm2_i2c_exit(void)
{
	platform_driver_unregister(&lm2_i2c_drv);
}

module_init(lm2_i2c_init);
module_exit(lm2_i2c_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("LM2 EEPROM driver");
