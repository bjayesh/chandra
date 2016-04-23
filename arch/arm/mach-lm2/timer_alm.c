/*
 * FujiXerox LM2 Waikiki Motherboard Support
 * Copyright (c) 2013-2014 Wind River Systems, Inc
 * Koki Yamano < koki.yamano@windriver.com >
 * This file is released under the GPLv2
 *
 * arch/arm/mach-lm2/tim_alm.c
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/debugfs.h>
#include <mach/motherboard.h>
#include <mach/irqs.h>

//#define	DEBUG_TIMALM

#define SYSCLK        		(3*1000*1000) /* 3MHz */
#define	CIPUI_IRQ		0x041F0040
#define	CIPUI_TIMER0		0x041F0420
#define	CIPUI_TIMER1		0x041F0440
#define	CIPUI_TIMER2		0x041F0460
#define	CIPUI_TIMER0_IRQBIT	0x00010000
#define	CIPUI_TIMER1_IRQBIT	0x00020000
#define	CIPUI_TIMER2_IRQBIT	0x00040000

#define	USE_TIMER		CIPUI_TIMER0
#define	USE_IRQBIT		CIPUI_TIMER0_IRQBIT

#define CUPUI_TCOUNT_L		(0x00)
#define CUPUI_TCOUNT_H		(0x04)
#define CUPUI_TCOUNTV_L		(0x08)
#define CUPUI_TCOUNTV_H		(0x0c)
#define CUPUI_TCONTROL		(0x10)

struct cipui_tim_device {
	struct platform_device	*pdev;
	int		irq;
	u32		tim_val[32];
	void __iomem	*cipui_irq_base;
};

struct cipui_tim_device *cipui_tim;
void __iomem            *cipui_tim_base;

/* CIP UI Interrupt handling */
void	(* osddiIntTimerEvent)(void);
EXPORT_SYMBOL_GPL(osddiIntTimerEvent);

#ifdef	DEBUG_TIMALM
static void cipui_tim_dump(void) {
	printk(KERN_ERR "%s: *** reg dump ***\n", __func__);
	printk(KERN_ERR "%s: TCOUNT_L  = 0x%08x\n", __func__, readl(cipui_tim_base + CUPUI_TCOUNT_L));
	printk(KERN_ERR "%s: TCOUNT_H  = 0x%08x\n", __func__, readl(cipui_tim_base + CUPUI_TCOUNT_H));
	printk(KERN_ERR "%s: TCOUNTV_L = 0x%08x\n", __func__, readl(cipui_tim_base + CUPUI_TCOUNTV_L));
	printk(KERN_ERR "%s: TCOUNTV_H = 0x%08x\n", __func__, readl(cipui_tim_base + CUPUI_TCOUNTV_H));
}
#else
#define	cipui_tim_dump()
#endif

static ssize_t
timalm_read(struct file *filp, char __user *buf, size_t len, loff_t *ppos)
{
	u32	low,hi;
	u64	read_val;
	
	cipui_tim_dump();
	low = readl(cipui_tim_base + CUPUI_TCOUNTV_L);
	hi  = readl(cipui_tim_base + CUPUI_TCOUNTV_H);
	read_val = (hi << 32 ) | low;
#ifdef	DEBUG_TIMALM
printk(KERN_ERR "%s: count_rval=0x%08x %08x(%llx)\n", __func__, hi, low, read_val);
#endif
	return simple_read_from_buffer(buf, len, ppos, &cipui_tim->tim_val, sizeof(cipui_tim->tim_val));
}

static ssize_t
timalm_write(struct file *filp, const char __user *buf, size_t len, loff_t *ppos)
{
	ssize_t ret;
	u32	tim_val;
	u64	set_val;

	ret = simple_write_to_buffer(&cipui_tim->tim_val, sizeof(cipui_tim->tim_val), ppos, buf, len);
	tim_val = simple_strtoul(&cipui_tim->tim_val, NULL, 10);
	if( tim_val < 0 ) {
		printk(KERN_ERR "%s: tim_val error\n", __func__);
		return ret;
	}
	set_val = (u64)tim_val * SYSCLK;
	if( set_val > 0xffffffffffff ) {
		printk(KERN_ERR "%s: tim_val error\n", __func__);
		return ret;
	}

#ifdef	DEBUG_TIMALM
printk(KERN_ERR "%s: sec=%d set_val=0x%llx\n", __func__, tim_val, set_val);
#endif
	if ( tim_val != 0 ) {
		tim_val = set_val>>32;
		writel(tim_val, cipui_tim_base + CUPUI_TCOUNT_H);
		tim_val = set_val & 0xffffffff;
		writel(tim_val, cipui_tim_base + CUPUI_TCOUNT_L);
		cipui_tim_dump();
		writel(0x3,                  cipui_tim_base + CUPUI_TCONTROL);
	} else {
		/* stop */
		writel(0x0,                  cipui_tim_base + CUPUI_TCONTROL);
	}

	return ret;
}

static struct file_operations tim_alm_fops = {
	.owner = THIS_MODULE,
	.read  = timalm_read,
	.write = timalm_write,
};

static irqreturn_t cipui_tim_irq(int irq, void *ptr)
{
	u32	status=0;
	struct	cipui_tim_device *cipui_tim=(struct cipui_tim_device*)ptr;
	struct	device *dev = cipui_tim->pdev;
	
	if (unlikely(!dev)) {
	        pr_err("%s: invalid dev pointer\n", __func__);
		return IRQ_NONE;
        }

	status = readl(cipui_tim->cipui_irq_base + 0x0);
#if 0
printk(KERN_ERR "%s: IRQ found.(0x%x status=0x%x)\n", __func__,irq, status);
#endif
	if ( (status & USE_IRQBIT) ) {
		printk(KERN_ERR "%s: TimerAlam IRQ found.(0x%x)\n", __func__,irq);
		writel(0x0,                  cipui_tim_base + CUPUI_TCONTROL);
		writel(USE_IRQBIT,           cipui_tim->cipui_irq_base + 0x08);
		if(osddiIntTimerEvent)
			(osddiIntTimerEvent)();
		return IRQ_HANDLED;
	}
	return IRQ_NONE;
}

static int cipui_tim_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *res;
	struct dentry *d;
	int	ret;

	cipui_tim = devm_kzalloc(dev, sizeof(struct cipui_tim_device), GFP_KERNEL);
	if (!cipui_tim) {
		dev_err(dev, "failed to memory alloc\n");
		return  -ENOMEM;
	}
	cipui_tim->pdev = pdev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(dev, "failed to ioresource\n");
		return -ENOENT;
	}

	cipui_tim_base = ioremap(res->start,0x20);
	if (IS_ERR(cipui_tim_base)) {
		dev_err(dev, "failed to ioremap\n");
		return PTR_ERR(cipui_tim_base);
	}

	cipui_tim->cipui_irq_base = ioremap(CIPUI_IRQ,0x10);
	if (IS_ERR(cipui_tim->cipui_irq_base)) {
		dev_err(dev, "failed to ioremap\n");
		return PTR_ERR(cipui_tim->cipui_irq_base);
	}
#ifdef	DEBUG_TIMALM
printk(KERN_ERR "%s: cipui_irq_base=0x%lx\n", __func__, cipui_tim->cipui_irq_base);
#endif

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (res == NULL) {
		dev_warn(dev, "failed to get resource: irq\n");
	 } else {
		cipui_tim->irq = res->start;
		ret = devm_request_irq(dev, cipui_tim->irq, cipui_tim_irq, IRQF_SHARED, dev_name(dev), cipui_tim );
		if (ret < 0) {
			dev_err(dev, "failed to attach cipui_tim irq\n");
			return ret;
		}
	}

	/* /sys/kernel/debug/tim_alm */
	d = debugfs_create_file("tim_alm", S_IRUGO | S_IWUSR, NULL, &cipui_tim->tim_val, &tim_alm_fops);
	if (!d)
		return -ENOMEM;
	strcpy(&cipui_tim->tim_val, "0\n");


	dev_info(dev, "attached cipui_tim driver\n");
	cipui_tim_dump();

	platform_set_drvdata(pdev, cipui_tim);
	return 0;
}

static int cipui_tim_remove(struct platform_device *pdev)
{
	return 0;
}

static int cipui_tim_suspend(struct device *dev)
{
	return 0;
}

static int cipui_tim_resume(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops cipui_tim_pm_ops = {
	.suspend	= cipui_tim_suspend,
	.resume		= cipui_tim_resume,
};

static struct platform_driver cipui_tim_driver = {
	.driver		= {
		.name	= "lm2-timalm",
		.owner	= THIS_MODULE,
		.pm	= &cipui_tim_pm_ops,
	},
	.probe		= cipui_tim_probe,
	.remove		= cipui_tim_remove,
};

static int __init cipui_tim_init(void)
{
	int ret;
	
	ret = platform_driver_register(&cipui_tim_driver);
	if (ret)
		printk(KERN_ERR "%s: failed to add CIP_UI Timer driver\n", __func__);
	
	return ret;
}
module_init(cipui_tim_init);




static struct resource lm2_timalm_resources[] = {
	[0] = {
		.start	= USE_TIMER,
                .end    = USE_TIMER + 0x20,
                .flags  = IORESOURCE_MEM,
        },
        [1] = {
                .start  = LM2_IRQ_CIPUI,
                .end    = LM2_IRQ_CIPUI,
                .flags  = IORESOURCE_IRQ,
        },
};
static struct platform_device lm2_timalm_device = {
        .name           = "lm2-timalm",
        .id             = 0,
        .num_resources  = ARRAY_SIZE(lm2_timalm_resources),
        .resource       = lm2_timalm_resources,
};
void __init lm2_cipui_tim_init(void)
{
#ifdef  DEBUG_TIMALM
printk(KERN_ERR "%s: \n", __func__);
#endif
	platform_device_register(&lm2_timalm_device);
}
