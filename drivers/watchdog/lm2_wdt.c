/*
 * lm2_wdt.c -  watchdog driver
 *
 * Copyright (C) Wind River Systems, Inc.
 *
 * Based on code written by Amit Kucheria and Michael Buesch.
 * Rewritten by Aaro Koskinen.
 *
 * This file is subject to the terms and conditions of the GNU General
 * Public License. See the file "COPYING" in the main directory of this
 * archive for more details.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/init.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/watchdog.h>
#include <linux/io.h>
#include <linux/platform_device.h>

/* Watchdog timer values in seconds */
#define WDT_MAX_TIMER	0xffffffff
#define	WDT_MIN_TIMER	0x00000002

#define	WDT_TIMER_START	0x00000001
#define	WDT_TIMER_STOP	0x00000000

#define	WDTTC		0x080
#define	WDTCNT		0x084
#define	WDTEN		0x088
#define	WDTLD		0x08C
#define	WDTBND		0x090

#define	WDT_TIMER_LOAD	0x00007a1c

//#define	DEFAULT_TIMEOUT_10s	32*1000*10	/* 32KHz from RTCCLK2 */
#define	DEFAULT_TIMEOUT_10s	1	/* 32KHz from RTCCLK2 */

struct lm2_wdt_dev {
	volatile u32 __iomem	*reg_base;
	struct device		*dev;
	unsigned int		timeout;
};


static int lm2_wdt_start(struct watchdog_device *wdog)
{
	struct lm2_wdt_dev *wdev = watchdog_get_drvdata(wdog);


//	printk(KERN_WARNING "Watch Dog Start\n");
	writel(WDT_TIMER_START, wdev->reg_base + WDTEN);
	barrier();
	writel(WDT_TIMER_LOAD, wdev->reg_base + WDTLD);
	barrier();

	return 0;
}

static int lm2_wdt_stop(struct watchdog_device *wdog)
{
	struct lm2_wdt_dev *wdev = watchdog_get_drvdata(wdog);

//	printk(KERN_WARNING "Watch Dog Stop\n");
	writel(WDT_TIMER_STOP, wdev->reg_base + WDTEN);
	barrier();
	return 0;
}

static int lm2_wdt_set_timeout(struct watchdog_device *wdog,
				unsigned int timeout)
{
	struct lm2_wdt_dev *wdev = watchdog_get_drvdata(wdog);

	printk(KERN_WARNING "Watch Dog Timer Set %d\n",timeout);
	wdog->timeout = timeout;

	writel(timeout, wdev->reg_base + WDTTC);
	barrier();

	return 0;
}

static const struct watchdog_info lm2_wdt_info = {
	.options = WDIOF_SETTIMEOUT ,
	.identity = "LM2 watchdog",
};

static const struct watchdog_ops lm2_wdt_ops = {
	.owner		= THIS_MODULE,
	.start		= lm2_wdt_start,
	.stop		= lm2_wdt_stop,
	.set_timeout	= lm2_wdt_set_timeout,
};


static int lm2_wdt_probe(struct platform_device *pdev)
{
	bool nowayout = WATCHDOG_NOWAYOUT;
	struct watchdog_device *lm2_wdt;
	struct lm2_wdt_dev *wdev;
	struct resource	*mem;
	int ret;

	lm2_wdt = devm_kzalloc(&pdev->dev, sizeof(*lm2_wdt), GFP_KERNEL);
	if (!lm2_wdt){
		printk( KERN_ERR "Watch Dog Timer NOMEM install failed\n");
		return -ENOMEM;
	}

	wdev = devm_kzalloc(&pdev->dev, sizeof(*wdev), GFP_KERNEL);
	if (!wdev){
		printk(KERN_ERR "Watch Dog driver could not allocate private memory\n");
		return -ENOMEM;
	}

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if(!mem){
		dev_err(&pdev->dev,"could not get IOMEM space\n");
		return	-EINVAL;
	}

	wdev->reg_base = ioremap(mem->start,0x100);

	lm2_wdt->info		= &lm2_wdt_info;
	lm2_wdt->ops		= &lm2_wdt_ops;
	lm2_wdt->timeout	= DEFAULT_TIMEOUT_10s;
	lm2_wdt->min_timeout	= 2;
	lm2_wdt->max_timeout	= WDT_MAX_TIMER;

	watchdog_set_drvdata(lm2_wdt, wdev);

	platform_set_drvdata(pdev, lm2_wdt);

	watchdog_set_nowayout(lm2_wdt, nowayout);

	wdev->dev		= &pdev->dev;

	ret = watchdog_register_device(lm2_wdt);
	if (ret < 0)
		return ret;

	/* init */
	writel(0x00, wdev->reg_base + WDTEN);
	barrier();
	writel(WDT_MAX_TIMER, wdev->reg_base + WDTBND);
	barrier();
	writel(DEFAULT_TIMEOUT_10s, wdev->reg_base + WDTTC);

	return 0;
}

static int lm2_wdt_remove(struct platform_device *pdev)
{
	struct watchdog_device *wdog = platform_get_drvdata(pdev);
	struct lm2_wdt_dev *wdev = watchdog_get_drvdata(wdog);

	watchdog_unregister_device(wdog);
	iounmap(wdev->reg_base);
	kfree(wdev);
	kfree(wdog);
	return 0;
}

static struct platform_driver lm2_wdt_driver = {
	.probe		= lm2_wdt_probe,
	.remove		= lm2_wdt_remove,
	.driver		= {
		.name	= "lm2-wdt",
		.owner	= THIS_MODULE,
	},
};

module_platform_driver(lm2_wdt_driver);

MODULE_ALIAS("platform:lm2-wdt");
MODULE_DESCRIPTION("watchdog Timer");
MODULE_LICENSE("GPL");
