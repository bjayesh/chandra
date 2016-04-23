/*
 * drivers/rtc/rtc-quatro55xx.c - RTC driver
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
 * 1.0	update software alarm
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

#define	LM2_INFOREG1_READ	0x0100
#define	LM2_INFOREG2_READ	0x0200

#define	LM2_GET		1
#define	LM2_SET		0

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

#define	RTC_BY	0x01
#define	RTC_RW	0x02
#define	RTC_IN	0x04

#define	RTC_TM	0x07

#define	RTC_CN	0x01

#define	RTC_HS	0x20
#define	RTC_CI	0x40
#define	RTC_CC	0x80

#define	RTC_EN	0x01

struct	lm2_rtc	{
	volatile u8 __iomem	*rtc_base;
	int			irq;
	struct rtc_device	*rtc;
	unsigned long		count;
	struct mutex		lock;
	unsigned long		alm_en;
	unsigned long		alm_sec;
	int			tim_en;
	struct work_struct 	irqwork;
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
		if(ctrl & RTC_BY)
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

	/* Clock Connect */
	writeb(RTC_CN, reg_base + RTCCTL_CON);	/* RTCCTR.CN = 1 */
	barrier();
	udelay(1);

	/* Initial Sequence */
	writeb(RTC_CC, reg_base + RTCCTL_CLK);	/* RTCCTL.CC=1 CI=0 HS=0 */
	barrier();
	writeb(RTC_CI, reg_base + RTCCTL_CLK);	/* RTCCTL.CC=0 CI=1 HS=0 */
	barrier();
	writeb(RTC_CC|RTC_CI, reg_base + RTCCTL_CLK);	/* RTCCTL.CC=1 CI=1 HS=0 */
	barrier();
	writeb(0x00, reg_base + RTCCTL_CLK);	/* RTCCTL.CC=0 CI=0 HS=0 */
	barrier();
	writeb(RTC_CC, reg_base + RTCCTL_CLK);	/* RTCCTL.CC=1 CI=0 HS=0 */
	barrier();
	writeb(RTC_CI, reg_base + RTCCTL_CLK);	/* RTCCTL.CC=0 CI=1 HS=0 */
	barrier();
	writeb(RTC_CC|RTC_CI, reg_base + RTCCTL_CLK);	/* RTCCTL.CC=1 CI=1 HS=0 */
	barrier();
	writeb(0x00, reg_base + RTCCTL_CLK);	/* RTCCTL.CC=0 CI=0 HS=0 */
	barrier();
	udelay(1);
	/* Wait another 300ns RTC is established */
	return	0;

}

static	void	rtc_disconnect(volatile u8 __iomem *reg_base)
{
	/* Setup Sequence */

	writeb(RTC_CC, reg_base + RTCCTL_CLK);	/* RTCCTL.CC=1 CI=0 HS=0 */
	barrier();
	writeb(0x00, reg_base + RTCCTL_CLK);	/* RTCCTL.CC=0 CI=0 HS=0 */
	barrier();
	writeb(RTC_CC, reg_base + RTCCTL_CLK);
	barrier();
	writeb(0x00, reg_base + RTCCTL_CLK);
	barrier();
	writeb(RTC_CC, reg_base + RTCCTL_CLK);
	barrier();
	writeb(0x00, reg_base + RTCCTL_CLK);
	barrier();
	writeb(RTC_CC, reg_base + RTCCTL_CLK);
	barrier();
	writeb(0x00, reg_base + RTCCTL_CLK);
	barrier();

	/* Disconnect Clock */
	writeb(0x00, reg_base + RTCCTL_CON);
	/* 300ns before power off */

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
	writeb(RTC_BY, rtc->rtc_base + RTCCTL_CTL);
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
	writeb(RTC_IN, rtc->rtc_base + RTCCTL_CTL);
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
static	irqreturn_t	lm2_timer_irq_handler(int irq, void *ptr)
{
	struct	lm2_rtc	*rtc = ptr;
	unsigned int	currect;

	current = readl(rtc->rtc_base+ RTCCNT);

	if(rtc->alm_en){
		if(current >= rtc->alm_sec ){
			rtc_update_irq(rtc->rtc, 1, RTC_AF | RTC_IRQF);
		}
	}

	return	IRQ_HANDLED;
}

static	int	lm2_read_alarm(struct device *dev, struct rtc_wkalrm *alm)
{
	struct lm2_rtc *rtc = dev_get_drvdata(dev);

	dev_info(dev, "lm2_read_alarm call\n");
	rtc_time_to_tm(rtc->alm_sec, &alm->time);

	return	0;
}

static	int	lm2_set_alarm(struct device *dev, struct rtc_wkalrm *alm)
{
	struct lm2_rtc *rtc = dev_get_drvdata(dev);
	unsigned long	time;
	u32	current_sec;
	int	result;

	dev_info(dev, "lm2_set_alarm call %lx \n", alm->time);
	rtc_tm_to_time(&alm->time, &time);
	rtc->alm_sec = time;
	result = rtc_busy(rtc->rtc_base);
	if(result != 0){
		printk(KERN_ERR "No responce Watch Dog Timer\n");
		return	-1;
	}
	
	current_sec = readl(rtc->rtc_base + RTCCNT);
	if( current_sec < time ){
		schedule_work(&rtc->irqwork);
		return 0;
	}else{
		return	-ETIME;
	}
}

static	int	lm2_alarm_enable(struct device *dev, unsigned int enable)
{
	struct lm2_rtc *rtc = dev_get_drvdata(dev);

	dev_info(dev, "lm2_alarm_enable call % d\n", enable);
	dev_info(dev, "lm2_alarm tim_en %d\n",rtc->tim_en);
	if(enable == 1){
		//if(request_irq(rtc->irq, lm2_timer_irq_handler, 0, "lm2_rtc", rtc)){
		//	return	-EINVAL;
		//}
		rtc->alm_en = 1;
		rtc->tim_en = 1;
	}else{
		if(rtc->tim_en == 0)	return	0;
		//free_irq(rtc->irq,dev);
		rtc->alm_en = 0;
	}
	return	0;
}

//extern unsigned long volatile __jiffy_data  jiffies;
void lm2_timer_do_work(struct work_struct *work)
{
	u32	current_sec;
	int result, period;

	struct lm2_rtc *rtc =
	container_of(work, struct lm2_rtc, irqwork);
	result = rtc_busy(rtc->rtc_base);
	if(result != 0){
		printk(KERN_ERR "No responce Watch Dog Timer\n");
		return	-1;
	}
	
	current_sec = readl(rtc->rtc_base + RTCCNT);
	//printk("rtc->alm_sec:%d\n", rtc->alm_sec);
	//printk("current_sec:%d\n", current_sec);
	if( rtc->alm_sec > current_sec ){
		period = (rtc->alm_sec - current_sec) * 1000;
		//printk("jiffies before:%d period:%d\n", jiffies, period);
		msleep(period);
		//printk("jiffies after :%d\n", jiffies);
	}
	rtc_update_irq(rtc->rtc, 1, RTC_AF | RTC_IRQF);
	//printk("lm2_timer_do_work done\n");
}

static	int	lm2_get_inforeg(struct device *dev, unsigned long *value, int info)
{
	struct lm2_rtc	*rtc = dev_get_drvdata(dev);
	int	result;

	mutex_lock(&rtc->lock);
	rtc_connect(rtc->rtc_base);
	writeb(RTC_IN, rtc->rtc_base + RTCCTL_CTL);
	barrier();
	udelay(1);
	result = rtc_busy(rtc->rtc_base);
	if(result != 0){
		printk(KERN_ERR "Time out error\n");
		rtc_disconnect(rtc->rtc_base);
		mutex_unlock(&rtc->lock);
		return	-1;
	}
	writeb(info, rtc->rtc_base + RTCCTL_TM);
	udelay(1);
	writeb(RTC_RW|RTC_BY, rtc->rtc_base + RTCCTL_CTL);
	result = rtc_busy(rtc->rtc_base);
	if(result != 0){
		printk(KERN_ERR "Time out error\n");
		rtc_disconnect(rtc->rtc_base);
		mutex_unlock(&rtc->lock);
		return	-1;
	}
	*value = readl(rtc->rtc_base + RTCCNT);
	rtc_disconnect(rtc->rtc_base);
	mutex_unlock(&rtc->lock);
	return	0;
}

static	int	lm2_voltage_status(struct device *dev, unsigned long *value, int flag)
{
	struct lm2_rtc	*rtc = dev_get_drvdata(dev);
	char	result;

	mutex_lock(&rtc->lock);
	if(flag){
		result = readb(rtc->rtc_base + RTCCTL_CTL);
		result &= ~RTC_IN; 
		writeb(result, rtc->rtc_base + RTCCTL_CTL);
	}else{
		result = readb(rtc->rtc_base + RTCCTL_CTL);
		if(result & RTC_IN)
			*value = 1;
		else
			*value = 0;
	}
	mutex_unlock(&rtc->lock);
	return	0;
}

static	int	lm2_ioctl(struct device *dev,unsigned int opecode, unsigned long value)
{
	int	result;

	switch(opecode){
	case	LM2_INFOREG1_READ:
		result = lm2_get_inforeg(dev, (unsigned long *)value, 0x04);
		break;
	case	LM2_INFOREG2_READ:
		result = lm2_get_inforeg(dev, (unsigned long *)value, 0x06);
		break;
	case	RTC_VL_READ:
		result = lm2_voltage_status(dev, (unsigned long *)value, LM2_GET); 
		break;
	case	RTC_VL_CLR:
		result = lm2_voltage_status(dev, 0, LM2_SET); 
		break;
	default:
		printk( KERN_ERR "%s Unknown opecode = %d\n", __func__, opecode);
		result = -1;
		break;
	}
	return	result;
}

const static struct of_device_id rtc_of_match[] __initconst = {
	{ .compatible = "waikiki,lm2-rtc", },
	{ },
};

static const	struct	rtc_class_ops	lm2_rtc_ops = {
	.read_time	= lm2_get_time,
	.set_time	= lm2_set_time,
	.ioctl		=lm2_ioctl,
	.read_alarm	= lm2_read_alarm,
	.set_alarm	= lm2_set_alarm,
	.alarm_irq_enable	= lm2_alarm_enable,
};

static int __init lm2_rtc_probe(struct platform_device *pdev)
{
	struct lm2_rtc	*dev;
	struct resource	*mem;
	struct resource *irq;
	u32	val;
	u8	reg;
	
	dev_info(&pdev->dev,"RTC Driver Probing\n");

	/* Private date get */
	dev = devm_kzalloc(&pdev->dev,sizeof(struct lm2_rtc), GFP_KERNEL);
//	dev = kzalloc(sizeof(struct lm2_rtc), GFP_KERNEL);
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

	dev->irq = platform_get_irq(pdev, 0);
	if(!dev->irq){
		kfree(dev);
		dev_err(&pdev->dev, " Could not get IRQ resource\n");
		return	-ENOMEM;
	}	
	INIT_WORK(&dev->irqwork, lm2_timer_do_work);

	mutex_init(&dev->lock);

	dev->count = 0;
	dev->alm_en =0;
	dev->tim_en =0;	

	platform_set_drvdata(pdev,dev);

	dev->rtc = rtc_device_register("lm2-rtc", &pdev->dev, &lm2_rtc_ops, THIS_MODULE);
	dev->rtc->uie_unsupported = 1;

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

#ifdef  CONFIG_ARCH_LM2
#define LM2_REGBAK_SIZE 10
static unsigned int     reg_bak[LM2_REGBAK_SIZE];
static unsigned int     reg_bak_chksum;
extern unsigned int     chksum_info;
void rtc_reg_save(void __iomem *base, int *bak_adr, int offset, int size)
{
	int i;
	int adr = *bak_adr;

	for(i=adr; i<(adr+size); i++ ) {
		reg_bak[i] = readl(base + offset);
		offset +=4;
	}
	*bak_adr = i;
}

void rtc_reg_load(void __iomem *base, int *bak_adr, int offset, int size)
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

static int lm2_rtc_suspend(struct device *dev)
{
//	struct lm2_rtc  *rtc = dev_get_drvdata(dev);
	int i=0;
	void __iomem *base;
        base = ioremap_nocache(0x04030000, 0x100);
        rtc_reg_save(base, &i, 0x010,  1);
        rtc_reg_save(base, &i, 0x018,  1);
        rtc_reg_save(base, &i, 0x01c,  1);
        iounmap(base);

        /* chksum gen */
	reg_bak_chksum=0;
	for(i=0; i<LM2_REGBAK_SIZE; i++)
		reg_bak_chksum += reg_bak[i];

        return 0;
}

static int lm2_rtc_resume(struct device *dev)
{
//	struct lm2_rtc  *rtc = dev_get_drvdata(dev);
	int i=0;
	void __iomem *base;
	unsigned int    tmp;

        /* chksum chk */
	tmp=0;
	for(i=0; i<LM2_REGBAK_SIZE; i++)
		tmp += reg_bak[i];
	if ( tmp != reg_bak_chksum ){
		chksum_info |= 0x40;
	}

	i=0;
        base = ioremap_nocache(0x04030000, 0x100);
        rtc_reg_load(base, &i, 0x010,  1);
        rtc_reg_load(base, &i, 0x018,  1);
        rtc_reg_load(base, &i, 0x01c,  1);
        iounmap(base);

        return 0;
}
#endif	/* CONFIG_ARCH_LM2 */

static SIMPLE_DEV_PM_OPS(lm2_rtc_pm_ops, lm2_rtc_suspend, lm2_rtc_resume);

static	const struct platform_device_id lm2_rtc_id_table[] = {
	{ "lm2-rtc",},
	{},
};

static	struct	platform_driver lm2_rtc_driver = {
	.driver = {
		.name	= "lm2-rtc",
		.owner	= THIS_MODULE,
		.pm	= &lm2_rtc_pm_ops,
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
