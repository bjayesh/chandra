/*
 * arch/arm/plat-waikiki/rtc.c	- rtc low level driver
 *
 * Copyright (C) 2014 Wind River Systems, Inc.
 * Koki Yamano <koki.yamano@windriver.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */
/*
 * revision:
 * 0.2  connection fail
 * 0.1  added alram function. All Interface clean. where alarm register?
 * 0.0	initial 
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/rtc.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/irq.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/printk.h>
#include <asm/barrier.h>

#define	RTCCNT		0x010
#define	RTCSECOND	0x018
#define	RTCCTL		0x01c

/* bye operation address */
#if 1
#define	RTCCTL_CTL	(RTCCTL + 0)
#define	RTCCTL_TM	(RTCCTL + 1)
#define	RTCCTL_CON	(RTCCTL + 2)
#define	RTCCTL_CLK	(RTCCTL + 3)
#else
#define	RTCCTL_CTL	(RTCCTL + 3)
#define	RTCCTL_TM	(RTCCTL + 2)
#define	RTCCTL_CON	(RTCCTL + 1)
#define	RTCCTL_CLK	(RTCCTL + 0)
#endif	/* */

#define	RTC_MAX_WAIT	32	/* 24MHz system clock cycle ? */

#define	RTC_START	0x00000001	/* BY */
#define	RTC_RDWR_R	0x00000002	/* RW */
#define	RTC_RDWR_W	0x00000000	/* RW */
#define	RTC_INIT	0x00000004	/* IN */
#define	RTC_TEST_N	0x00000000	/* TM */
#define	RTC_CONNECT	0x00010000	/* CN */
#define	RTC_HTOL_START	0x20000000	/* HS */
#define	RTC_CODE_INPUT	0x40000000	/* CI */
#define	RTC_CODE_CLK	0x80000000	/* CC */

#define	RTC_STAT_BY	0x01
#define	RTC_CON_EN	0x01

struct	lm2_rtc	{
	volatile u8 __iomem	*rtc_base;
	struct rtc_device	*rtc;
	unsigned long		count;
//	spinlock_t		lock;
	struct mutex		lock;
};

/*
 * rtc class menbers
 */
static	int	rtc_busy(volatile u8 __iomem *reg_base)
{
	u8	ctrl;
	int	retry;

	retry = 0;
	do{
		ctrl = readb(reg_base + RTCCTL_CTL);
		barrier();
		if(ctrl & RTC_STAT_BY)
			msleep(5);
		else
			goto	end;
		retry++;
	}while(retry < RTC_MAX_WAIT);
	return	-1;	/* retry over */
end:
	return	0;
}

static	int	rtc_connect(volatile void __iomem *reg_base)
{
#if 0
	u32	val;
	u32	cmd;


        val = readl(reg_base + RTCCTL);
	val &= 0x00ffffff;

	/* connection set */
	val |= RTC_CONNECT;
	writel(val, reg_base + RTCCTL);
	udelay(1);

	/* init sequence */
	cmd = val |= RTC_CODE_CLK;
	writel(cmd, reg_base + RTCCTL);
	udelay(1);
	cmd = val |= RTC_CODE_INPUT;
	writel(cmd, reg_base + RTCCTL);
	udelay(1);
	cmd = val |= RTC_CODE_CLK | RTC_CODE_INPUT;
	writel(cmd, reg_base + RTCCTL);
	udelay(1);
	cmd = val;
	writel(cmd, reg_base + RTCCTL);
	udelay(1);
	cmd = val |= RTC_CODE_CLK;
	writel(cmd, reg_base + RTCCTL);
	udelay(1);
	cmd = val |= RTC_CODE_INPUT;
	writel(cmd, reg_base + RTCCTL);
	udelay(1);
	cmd = val |= RTC_CODE_CLK | RTC_CODE_INPUT;
	writel(cmd, reg_base + RTCCTL);
	udelay(1);
	cmd = val;
	writel(cmd, reg_base + RTCCTL);
	udelay(1);
#else
	writeb(0x01, reg_base + RTCCTL_CON);
	barrier();
	udelay(1);

	writeb(0x80, reg_base + RTCCTL_CLK);
	barrier();
	writeb(0x40, reg_base + RTCCTL_CLK);
	barrier();
	writeb(0xc0, reg_base + RTCCTL_CLK);
	barrier();
	writeb(0x00, reg_base + RTCCTL_CLK);
	barrier();
	writeb(0x80, reg_base + RTCCTL_CLK);
	barrier();
	writeb(0x40, reg_base + RTCCTL_CLK);
	barrier();
	writeb(0xc0, reg_base + RTCCTL_CLK);
	barrier();
	writeb(0x00, reg_base + RTCCTL_CLK);
	barrier();
	udelay(1);
#endif		
	return	0;

}

static	void	rtc_disconnect(volatile u8 __iomem *reg_base)
{
#if 0
	u32	val;

        val = readl(reg_base + RTCCTL);

	val &= 0x00ffffff;

        val |= RTC_CODE_CLK;
        writel(val, reg_base + RTCCTL);	/* 0x80XXXXXX*/
	udelay(1);

        val &= ~RTC_CODE_CLK;
        writel(val, reg_base + RTCCTL);	/* 0x00XXXXXX */
	udelay(1);

        val = val | RTC_CODE_CLK;
        writel(val, reg_base + RTCCTL);	/* 0x80XXXXXX */
	udelay(1);

        val &= ~RTC_CODE_CLK;
        writel(val, reg_base + RTCCTL);
	udelay(1);

        val |= RTC_CODE_CLK;
        writel(val, reg_base + RTCCTL);
	udelay(1);

        val &= ~RTC_CODE_CLK;
        writel(val, reg_base + RTCCTL);
	udelay(1);

        val |= RTC_CODE_CLK;
        writel(val, reg_base + RTCCTL);
	udelay(1);

        val &= ~RTC_CODE_CLK;
        writel(val, reg_base + RTCCTL);
	udelay(1);

	/* connection bit reset */
	val &= RTC_CONNECT;
        writel(val, reg_base + RTCCTL);
#else
	writeb(0x80, reg_base + RTCCTL_CLK);
	barrier();
	writeb(0x00, reg_base + RTCCTL_CLK);
	barrier();
	writeb(0x80, reg_base + RTCCTL_CLK);
	barrier();
	writeb(0x00, reg_base + RTCCTL_CLK);
	barrier();
	writeb(0x80, reg_base + RTCCTL_CLK);
	barrier();
	writeb(0x00, reg_base + RTCCTL_CLK);
	barrier();
	writeb(0x80, reg_base + RTCCTL_CLK);
	barrier();
	writeb(0x00, reg_base + RTCCTL_CLK);
	barrier();
	writeb(0x00, reg_base + RTCCTL_CON);
#endif
	return;
}

static	int	lm2_set_time(struct device *dev, struct rtc_time *tm)
{
	struct lm2_rtc	*rtc = dev_get_drvdata(dev);
	u32	current_sec;
	int	result;
	u32	reg;

	printk( KERN_WARNING "RTC set date and time\n");
	result = rtc_tm_to_time(tm, &current_sec);
	if(result){
		printk(KERN_WARNING "wrong Date and time\n");
		return	result;
	}

	mutex_lock(&rtc->lock);

	rtc_connect(rtc->rtc_base);

	writel(current_sec, rtc->rtc_base + RTCCNT);

//	reg = readl(rtc->rtc_base + RTCCTL);
//	reg &= ~RTC_RDWR_R;
//	reg |= RTC_START;
//	writel(reg, rtc->rtc_base+ RTCCTL);
	writeb(0x01, rtc->rtc_base + RTCCTL_CTL);
	barrier();
	udelay(1);
	result = rtc_busy(rtc->rtc_base);
	if(result != 0){
		printk(KERN_ERR "No responce Watch Dog Timer\n");
		mutex_unlock(&rtc->lock);
		return	-1;
	}

	/* stored to RTC Block counter */

//	reg = readl(rtc->rtc_base + RTCCTL);
//	reg |= RTC_INIT;
//	writel(reg, rtc->rtc_base + RTCCTL);
	writeb(0x04, rtc->rtc_base + RTCCTL_CTL);
	barrier();
	udelay(1);
	result = rtc_busy(rtc->rtc_base);
	if(result != 0){
		printk(KERN_ERR "No responce store operation Watch Dog Timer\n");
		mutex_unlock(&rtc->lock);
		return	-1;
	}

	rtc_disconnect(rtc->rtc_base);

	mutex_unlock(&rtc->lock);

	printk( KERN_WARNING "RTC set date and timecompleted\n");
	
	return	0;
}


static	int	lm2_get_time(struct device *dev, struct rtc_time *tm)
{
	struct lm2_rtc	*rtc = dev_get_drvdata(dev);
	u32	current_sec;
	int	result;

//	if(rtc->rtc_base == NULL){
//		dev_err(dev,"ioremap error lm2_rtc_get %x\n",rtc->rtc_base);
//		return 0;
//	}

	result = rtc_busy(rtc->rtc_base);
	if(result != 0){
		printk(KERN_ERR "No responce Watch Dog Timer\n");
		return	-1;
	}
	
	current_sec = readl(rtc->rtc_base + RTCCNT);
	rtc_time_to_tm(current_sec,tm);

	return	0;
}

const static struct of_device_id rtc_of_match[] __initconst = {
	{ .compatible = "waikiki,lm2-rtc", },
	{ },
};

static const	struct	rtc_class_ops	lm2_rtc_ops = {
	.read_time	= lm2_get_time,
	.set_time	= lm2_set_time,
};

static int __init lm2_rtc_probe(struct platform_device *pdev)
{
	struct lm2_rtc	*dev;
	struct resource	*mem;
	u32	val;
	u8	reg;
	
	dev_info(&pdev->dev,"RTC Driver Probing\n");

	/* Private date get */
	dev = devm_kzalloc(&pdev->dev,sizeof(struct lm2_rtc), GFP_KERNEL);
	if(!dev){
		dev_err(&pdev->dev,"Could not allocate RTC driver space \n");
		return	-ENOMEM;
	}


	mem = platform_get_resource(pdev,IORESOURCE_MEM,0);
	if(!mem){
		dev_err(&pdev->dev,"no mmio space\n");
		kfree(dev);
		return	-EINVAL;
	}

	dev->rtc_base = ioremap(mem->start,0x100);
	if(!dev->rtc_base){
		dev_err(&pdev->dev,"can't mapping mmio\n");
		kfree(dev);
		return	-ENOMEM;
	}

	mutex_init(&dev->lock);

	dev->count = 0;
	
	platform_set_drvdata(pdev,dev);

	dev->rtc = rtc_device_register("lm2-rtc", &pdev->dev, &lm2_rtc_ops, THIS_MODULE);

	device_init_wakeup(&pdev->dev, 1);

	/*
 	 * To Do check connection and start
 	 */
	rtc_connect(dev->rtc_base);
	rtc_busy(dev->rtc_base);
	reg = readb(dev->rtc_base + RTCCTL_CTL);
	if((reg & 0x04) == 0){
		printk( KERN_WARNING "RTC first time access\n");
		writel(0x00000000,dev->rtc_base + RTCCNT);
		barrier();
		writeb(0x01, dev->rtc_base + RTCCTL_CTL);
		barrier();
		udelay(1);
		rtc_busy(dev->rtc_base);
		writeb(0x04, dev->rtc_base + RTCCTL_CTL);
		barrier();
		udelay(1);
		rtc_busy(dev->rtc_base);
	}
 	writeb(0x03, dev->rtc_base + RTCCTL_CTL);
	rtc_busy(dev->rtc_base);
	barrier();

	rtc_disconnect(dev->rtc_base);
#if 0	/* to used access timing */
 	writel(RTC_CONNECT, dev->rtc_base + RTCCTL);
	val = RTC_CODE_CLK | RTC_CODE_INPUT | RTC_HTOL_START;
 	writel(RTC_CODE_CLK, dev->rtc_base + RTCCTL);
 	writel(RTC_CODE_CLK|RTC_CODE_INPUT, dev->rtc_base + RTCCTL);
 	writel(val, dev->rtc_base + RTCCTL);
 	writel(0 , dev->rtc_base + RTCCTL);
 	writel(RTC_CODE_CLK, dev-> rtc_base + RTCCTL);
 	writel(RTC_CODE_CLK|RTC_CODE_INPUT, dev->rtc_base + RTCCTL);
 	writel(val, dev->rtc_base + RTCCTL);
 	writel(0, dev->rtc_base + RTCCTL);
	msleep(1);	/* RTC establish wait */	
#endif
	return	0;
}

static	int	__exit	lm2_rtc_remove(struct platform_device *pdev)
{
	struct lm2_rtc	*rtc = platform_get_drvdata(pdev);

	device_init_wakeup(&pdev->dev,0);


	/*
 	 * ToDo: remove or shutdown 
 	 * internel counter backup for the battery ?
 	 */
#if 0
	u32	val;
	u32	reg;

	val = readl(rtc->rtc_base + RTCCTL);

	val = val | RTC_CODE_CLK;
	writel(val, rtc->rtc_base + RTCCTL);
	val &= ~RTC_CODE_CLK;
	writel(val, rtc->rtc_base + RTCCTL);
	val = val | RTC_CODE_CLK;
	writel(val, rtc->rtc_base + RTCCTL);
	val &= ~RTC_CODE_CLK;
	writel(val, rtc->rtc_base + RTCCTL);
	val = val | RTC_CODE_CLK;
	writel(val, rtc->rtc_base + RTCCTL);
	val &= ~RTC_CODE_CLK;
	writel(val, rtc->rtc_base + RTCCTL);
	val = val | RTC_CODE_CLK;
	writel(val, rtc->rtc_base + RTCCTL);
	val &= ~RTC_CODE_CLK;
	writel(val, rtc->rtc_base + RTCCTL);
#endif
	iounmap(rtc->rtc_base);
	rtc_device_unregister(rtc->rtc);
	platform_set_drvdata(pdev,NULL);
	kfree(rtc);
	return	0;
}
static	const struct platform_device_id lm2_rtc_id_table[] = {
	{ "lm2-rtc",},
	{},
};

static	struct	platform_driver lm2_rtc_driver = {
	.driver = {
		.name	= "lm2-rtc",
		.owner	= THIS_MODULE,
	},
	.id_table	= lm2_rtc_id_table,
	.probe		= lm2_rtc_probe,
	.remove		= __exit_p(lm2_rtc_remove),
};

#if 0
module_platform_driver_probe(lm2_rtc_driver,lm2_rtc_probe);
#else
module_platform_driver(lm2_rtc_driver);
#endif

MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:lm2-rtc");
