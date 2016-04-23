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
#include <linux/timer.h>
#include <linux/jiffies.h>

#undef	DEBUG_LM2_WDT_IO
#undef	DEBUG_LM2_WDT_TRACE
#undef	DEBUG_LM2_WDT_RELOAD
#undef	DEBUG_LM2_WDT_DEBUG

/* Watchdog timer values in seconds */
#define	WDT_TIMER_START	0x00000001
#define	WDT_TIMER_STOP	0x00000000
#define	WDT_TIMER_LOAD	0x7a1c

#define	WDTTC		0x080
#define	WDTCNT		0x084	/*	Read Only	*/
#define	WDTEN		0x088	/* only ones set */
#define	WDTLD		0x08C
#define	WDTBND		0x090

//                              300*1000*1000	/* 300MHz supply clock */
#define	DEFAULT_TIMEOUT_10s	(unsigned long)(300*1000*1000 *10)	/* 32KHz from RTCCLK2 */
#define WDT_MAX_TIMER	(DEFAULT_TIMEOUT_10s -10)
#define	WDT_MIN_TIMER	0x00000002

static	void __iomem	*lm2_regbase;
static	unsigned long	lm2_ninesec;	/* 9 sec */
static	int		lm2_wdt_free;	/* 0: user keepalive control 1:wdt free */

struct lm2_wdt_dev {
	volatile void __iomem	*reg_base;
	struct device		*dev;
	unsigned int		timeout;
	int			running;	/* start flag */
};

#ifdef	DEBUG_LM2_WDT_IO
static	u32 inline	lm2_wdt_readl(void __iomem *addr)
{
	u32	val;

	val = readl(addr);
	printk(KERN_ERR "%s : Addr=0x%8.8x Data=0x%8.8x\n", __FUNCTION__, addr, val);
	return	val;
}
static	u32 inline	lm2_wdt_readw(void __iomem *addr)
{
	u32	val;

	val = readw(addr);
	printk(KERN_ERR "%s : Addr=0x%8.8x Data=0x%4.4x\n", __FUNCTION__, addr, val);
	return	(val & 0x0000ffff);
}
static	u32 inline	lm2_wdt_readb(void __iomem *addr)
{
	u32	val;

	val = readb(addr);
	printk(KERN_ERR "%s : Addr=0x%8.8x Data=0x%2.2x\n", __FUNCTION__, addr, val);
	return	(val & 0x000000ff);
}

static	void inline	lm2_wdt_writel(u32 val, void __iomem *addr)
{
	printk(KERN_ERR "%s : Addr=0x%8.8x Data=0x%8.8x\n", __FUNCTION__, addr, val);
	writel(val, addr);
}
static	void inline	lm2_wdt_writew(u32 val, void __iomem *addr)
{
	printk(KERN_ERR "%s : Addr=0x%8.8x Data=0x%4.4x\n", __FUNCTION__, addr, val);
	writew(val, addr);
}
static	void inline	lm2_wdt_writeb(u32 val, void __iomem *addr)
{
	printk(KERN_ERR "%s : Addr=0x%8.8x Data=0x%2.2x\n", __FUNCTION__, addr, val);
	writeb(val, addr);
}
#else
#define	lm2_wdt_readl(addr)		readl(addr)
#define	lm2_wdt_readw(addr)		readw(addr)
#define	lm2_wdt_readb(addr)		readb(addr)
#define	lm2_wdt_writel(value, addr)	writel(value, addr)
#define	lm2_wdt_writew(value, addr)	writew(value, addr)
#define	lm2_wdt_writeb(value, addr)	writeb(value, addr)
#endif	/* DEBUG_LM2_WDT_IO */	 

static	void	lm2_reload_timer_handler(unsigned long data);
DEFINE_TIMER(lm2_reload,lm2_reload_timer_handler,0,0);

static	void	lm2_reload_timer_handler(unsigned long data)
{
//	unsigned long j = jiffies;

#ifdef	DEBUG_LM2_WDT_RELOAD
	printk( KERN_WARNING "Reload timer handler\n");
#endif
	if(lm2_wdt_free == 1){
		lm2_reload.expires = jiffies + lm2_ninesec;
		add_timer(&lm2_reload);
		lm2_wdt_writew(WDT_TIMER_LOAD, lm2_regbase + WDTLD);
	}else{	/* user contorol */
#ifdef	DEBUG_LM2_WDT_RELOAD
		printk( KERN_WARNING "Reload timer exit\n");
#endif
		/* To Do */
	}
	return;
}
#if 0
static	void	dump_wdreg(struct lm2_wdt_dev *wdev,int loc)
{
	printk(KERN_WARNING "\n<<Watch Dog Timer Reg Dump at %d\n",loc);
	printk(KERN_WARNING "WDTTC = %xH, WDTCNT(RO) = %xH\n",
		   *(unsigned int *)(wdev->reg_base + WDTTC),
		   *(unsigned int *)(wdev->reg_base + WDTCNT));
	printk(KERN_WARNING "WDTEN = %xH,  WDTLD = %xH\n",
		   *(unsigned int *)(wdev->reg_base + WDTEN),
		   *(unsigned int *)(wdev->reg_base + WDTLD));
	printk(KERN_WARNING "WDTBND  = %xH>>\n\n",*(unsigned int *)(wdev->reg_base + WDTBND));
}
#endif
static int lm2_wdt_start(struct watchdog_device *wdog)
{

	struct lm2_wdt_dev *wdev = watchdog_get_drvdata(wdog);
	int		val;
//	unsigned long flags = 0;

#ifdef	DEBUG_LM2_WDT_TRACE
	printk(KERN_WARNING "Watch Dog Timer Start Set=%d\n",wdog->timeout);
#endif
	if(wdev->running != 0){
#ifdef	DEBUG_LM2_WDT_TRACE
		printk(KERN_WARNING "Watch Dog is Running!\n");
#endif
		lm2_wdt_writew(WDT_TIMER_LOAD, wdev->reg_base + WDTLD);
		lm2_wdt_free = 0;
		wdev->running = 1;
		return	0;
	}

	lm2_wdt_writel(0x00000000, wdev->reg_base + WDTEN);
	val = lm2_wdt_readl(wdev->reg_base + WDTEN);
	if(val != 0){
		printk(KERN_ERR "%s could not stop watch dog timer %x\n", __FUNCTION__, val);
		return	-EINVAL;
	}

//	lm2_wdt_writel(wdev->timeout, wdev->reg_base + WDTTC);
	lm2_wdt_writel(DEFAULT_TIMEOUT_10s, wdev->reg_base + WDTTC);
	val = lm2_wdt_readl(wdev->reg_base + WDTTC);
	if(val != DEFAULT_TIMEOUT_10s){
		printk(KERN_ERR "%s could not set terminal count 0x%8.8x\n", __FUNCTION__, val);
		return	-EINVAL;
	}
	lm2_wdt_writel(WDT_MAX_TIMER, wdev->reg_base + WDTBND);
	val = lm2_wdt_readl(wdev->reg_base + WDTBND);
	if(val != WDT_MAX_TIMER){
		printk(KERN_ERR "%s could not reset terminal count 0x%8.8x\n", __FUNCTION__, val);
		return	-EINVAL;
	}

	wdev->running = 1;
	lm2_wdt_free = 0;
	lm2_wdt_writel(WDT_TIMER_START, wdev->reg_base + WDTEN);
	lm2_wdt_writew(WDT_TIMER_LOAD, wdev->reg_base + WDTLD);
#ifdef	DEBUG_LM2_WDT_TRACE
	printk(KERN_WARNING "%s exit\n",__FUNCTION__);
#endif
	return 0;
}

/*
 * lm2_wdt_ping - keepalive ping
 * WDT reload WDTTC counter
 */
static int lm2_wdt_ping(struct watchdog_device *wdog)
{
	struct lm2_wdt_dev *wdev = watchdog_get_drvdata(wdog);
//	u32	val;

#ifdef	DEBUG_LM2_WDT_TRACE
	printk(KERN_WARNING "%s call\n",__FUNCTION__);
#endif
	lm2_wdt_writew(WDT_TIMER_LOAD, wdev->reg_base + WDTLD);

	return	0;	
}

/*
 * lm2_wdt_stop - user request timer stop
 * after run with self rest timer
 */
static int lm2_wdt_stop(struct watchdog_device *wdog)
{
	struct lm2_wdt_dev *wdev = watchdog_get_drvdata(wdog);

#ifdef	DEBUG_LM2_WDT_TRACE
	printk(KERN_WARNING "Watch Dog Stop\n");
#endif
	if(wdev->running == 0){
		return	0;
	}
	lm2_wdt_writew(WDT_TIMER_LOAD, wdev->reg_base + WDTLD);
	lm2_wdt_free = 1; /* driver has control */
#ifdef	DEBUG_LM2_WDT_RELOAD
		printk( KERN_WARNING "Reload timer start %x %x\n",jiffies,lm2_ninesec);
#endif
	lm2_reload.expires = jiffies + lm2_ninesec;
	add_timer(&lm2_reload);
	return 0;
}

/*
 * Timer set value
 * don't touch register
 */
static int lm2_wdt_set_timeout(struct watchdog_device *wdog,
				unsigned int timeout)
{
//	struct lm2_wdt_dev *wdev = watchdog_get_drvdata(wdog);

	printk(KERN_WARNING "Watch Dog Timer Out: %d\n",timeout);
	wdog->timeout = timeout;
	return 0;
}

static const struct watchdog_info lm2_wdt_info = {
	.options = WDIOF_KEEPALIVEPING ,
	.identity = "LM2 watchdog",
};

static const struct watchdog_ops lm2_wdt_ops = {
	.owner		= THIS_MODULE,
	.start		= lm2_wdt_start,
	.stop		= lm2_wdt_stop,
	.ping		= lm2_wdt_ping,
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
		dev_err(&pdev->dev,"could not get IOMEM space\n");
		printk( KERN_ERR "Watch Dog Timer NOMEM install failed\n");
		return -ENOMEM;
	}

	wdev = devm_kzalloc(&pdev->dev, sizeof(*wdev), GFP_KERNEL);
	if (!wdev){
		dev_err(&pdev->dev,"could not get IOMEM space\n");
		printk(KERN_ERR "Watch Dog driver could not allocate private memory\n");
		return -ENOMEM;
	}

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if(!mem){
		dev_err(&pdev->dev,"could not get IOMEM space\n");
		return	-EINVAL;
	}

	wdev->reg_base = ioremap(mem->start,0x100);
	lm2_regbase = wdev->reg_base;

	lm2_wdt->info		= &lm2_wdt_info;
	lm2_wdt->ops		= &lm2_wdt_ops;
	lm2_wdt->timeout	= DEFAULT_TIMEOUT_10s;
	lm2_wdt->min_timeout	= 2;
	lm2_wdt->max_timeout	= WDT_MAX_TIMER;
	wdev->running = 0;
	lm2_wdt_free = 0;
	lm2_ninesec = msecs_to_jiffies(1000 * 9);

	watchdog_set_drvdata(lm2_wdt, wdev);

	platform_set_drvdata(pdev, lm2_wdt);

	watchdog_set_nowayout(lm2_wdt, nowayout);

	wdev->dev		= &pdev->dev;

	ret = watchdog_register_device(lm2_wdt);
	if (ret < 0)
		return ret;
#ifdef	LM2_DEBUG_TRACE
	dev_info(&pdev->dev,"Watch Dog Timer Installed Virt =%x\n",wdev->reg_base);
#endif
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
MODULE_LICENSE("GP:L");

