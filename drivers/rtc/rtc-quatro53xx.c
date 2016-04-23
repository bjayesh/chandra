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

#define	RTCCNT		0x010
#define	RTCSECOND	0x018
#define	RTCCTL		0x01c

#define	RTC_START	0x00000001	/* BY */
#define	RTC_RDWR_R	0x00000002	/* RW */
#define	RTC_RDWR_W	0x00000000	/* RW */
#define	RTC_INIT	0x00000004	/* IN */
#define	RTC_TEST_N	0x00000000	/* TM */
#define	RTC_CONNECT	0x00010000	/* CN */
#define	RTC_HTOL_START	0x20000000	/* HS */
#define	RTC_CODE_INPUT	0x40000000	/* CI */
#define	RTC_CODE_CLK	0x80000000	/* CC */

struct	lm2_rtc	{
	void __iomem		*rtc_base;
	struct rtc_device	*rtc;
	unsigned long		count;
};

/*
 * rtc class menbers
 */
static	int	lm2_open(struct device *dev)
{
	/* need a mutex ? */
	return	0;
}

static	int	lm2_set_time(struct device *dev, struct rtc_time *tm)
{
	struct lm2_rtc	*rtc = dev;
	u32	current_sec;

	current_sec = mktime(tm->tm_year, tm->tm_mon,tm->tm_mday,tm->tm_hour,tm->tm_min,tm->tm_sec);
	writel(current_sec,rtc->rtc_base + RTCSECOND);

	return	0;
}


static	int	lm2_get_time(struct device *dev, struct rtc_time *tm)
{
	struct lm2_rtc	*rtc = dev;
	u32	current_sec;

	if(rtc->rtc_base == NULL){
		dev_err(dev,"ioremap error lm2_rtc_get %x\n",rtc->rtc_base);
		return 0;
	}
	current_sec = readl(rtc->rtc_base + RTCSECOND);
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

static void __init lm2_rtc_probe(struct platform_device *pdev)
{
	struct lm2_rtc	*dev;
	struct resource	*mem;
	u32	val;

	dev_info(&pdev->dev,"RTC Driver Probing\n");

	dev = kzalloc(sizeof(struct lm2_rtc), GFP_KERNEL);
	if(!dev){
		dev_err(&pdev->dev,"Could not allocate RTC driver space \n");
		return	-ENOMEM;
	}

	dev_info(&pdev->dev,"RTC Driver Probing\n");
	mem = platform_get_resource(pdev,IORESOURCE_MEM,0);
	if(!mem){
		dev_err(&pdev->dev,"no mmio space\n");
		return	-EINVAL;
	}

	dev_info(&pdev->dev,"RTC Driver Probing\n");
	dev->rtc_base = ioremap(mem->start,0x100);
//	dev->rtc_base = devm_ioremap(pdev,mem->start,resource_size(mem));
//	if(!dev->rtc_base){
//		dev_err(&pdev->dev,"can't mapping mmio\n");
//		return	-ENOMEM;
//	}
	dev->count = 0;

	dev_info(&pdev->dev,"RTC Driver Probing\n");
	platform_set_drvdata(pdev,dev);

	dev->rtc = rtc_device_register("lm2-rtc", &pdev->dev, &lm2_rtc_ops, THIS_MODULE);
#if 0 
	rtc =devm_rtc_device_register(&pdev->dev, "lm2-rtc",&lm2_rtc_ops, THIS_MODULE);
	if(IS_ERR(rtc))
		return	-1;
#endif
	/*
 	 * To Do check connection
 	 */
#if 0
	val = RTC_CODE_CLK | RTC_CODE_INPUT | RTC_HTOL_START;
 	writel(RTC_CODE_CLK, dev->rtc_base + RTCCTL);
 	writel(RTC_CODE_CLK|RTC_CODE_INPUT, dev->rtc_base + RTCCTL);
 	writel(val, dev->rtc_base + RTCCTL);
 	writel(0 , dev->rtc_base + RTCCTL);
 	writel(RTC_CODE_CLK, dev-> rtc_base + RTCCTL);
 	writel(RTC_CODE_CLK|RTC_CODE_INPUT, dev->rtc_base + RTCCTL);
 	writel(val, dev->rtc_base + RTCCTL);
 	writel(0, dev->rtc_base + RTCCTL);
	msleep(300);	/* RTC establish wait */	
 	writel(RTC_CONNECT, dev->rtc_base + RTCCTL);
#endif
}

static	int	__exit	lm2_rtc_remove(struct platform_device *dev)
{
	struct lm2_rtc	*rtc = platform_get_drvdata(dev);

	/*
 	 * ToDo: remove or shutdown 
 	 * internel counter backup for the battery ?
 	 */
#if 0
	u32	val;
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
