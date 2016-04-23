/*
 * arch/arm/plat-waikiki/time.c
 *
 * Copyright (C) 2014 Wind River Systems, Inc.
 * Koki Yamano <koki.yamano@windriver.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/clk.h>
#include <linux/clockchips.h>
#include <linux/clocksource.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/time.h>
#include <linux/irq.h>
#include <asm/mach/time.h>

/*
 * We would use TIMER0 and TIMER1 as clockevent and clocksource.
 * Timer0 and Timer1 both belong to same gpt block in cpu subbsystem. Further
 * they share same functional clock. Any change in one's functional clock will
 * also affect other timer.
 */

#define	SYSCLK		(300*1000*1000)	/* 300MHz */
#define	DIVISOR		3000	/* 100KHz 10msec */
#define	PRESCALE	6	/* 50MHz 20nsec */

/* Register offsets, x is channel number */
#define	PRESCL(x)	((x)+0)
#define	HTCNTR_H(x)	((x)+0x04)
#define	HTCNTR_L(x)	((x)+0x08)
#define	HTCTLR(x)		((x)+0x0c)

#define CTLR(x)		((x)+8)
#define LOAD(x)		((x)+0)
#define COUNT(x)	((x)+4)

/* Controll Register bit definitions */
#define	HTCTLR_START		0x00000002
#define	HTCTLR_CLR		0x00000001
#define	HTCTLR_STOP		0x00000000

#define CTLR_LOAD		0x00000001
#define CTLR_START		0x00000002
#define CTLR_CONTINOUS		0x00000004


extern	lm2_printk(unsigned long base, const char *str);

static	void __iomem *clksrc_base;
static	void __iomem *clkevt_base;

static void clockevent_set_mode(enum clock_event_mode mode,
				struct clock_event_device *clk_event_dev);
static int clockevent_next_event(unsigned long evt,
				 struct clock_event_device *clk_event_dev);

/*
 * Clock source driver (kernel timer)
 */
void	lm2_clocksource_init(__iomem void *gpt_base)
{
	u32	tick_rate;
	u32	val;
	int	result;

	/* secured memory I/O base */
	clksrc_base = gpt_base;
	/* To Do : find out actual clock driving Timer driver/clk */
	/* 
 	 * of_clk_init and clocksource_of_init with flattend device tree
 	 */

	/* Timer stop (initialize) */
	writel(HTCTLR_STOP,HTCTLR(clksrc_base));

	/* prescale setting */
	writel(PRESCALE-1,PRESCL(clksrc_base));

	/* Start Timer */
	writel(HTCTLR_START,HTCTLR(clksrc_base));

	/* rate culculate */
	tick_rate = SYSCLK / PRESCALE;

	/* register the clocksource */
	result = clocksource_mmio_init(HTCNTR_L(clksrc_base),"system_timer",tick_rate,
		200, 32, clocksource_mmio_readl_up);

//	if(result != 0){
//		lm2_printk(0xfc000000,"clocksource error \n");
//	}
}

static struct clock_event_device clkevt = {
	.name = "tmr0",
	.features = CLOCK_EVT_FEAT_PERIODIC | CLOCK_EVT_FEAT_ONESHOT,
	.set_mode = clockevent_set_mode,
	.set_next_event = clockevent_next_event,
	.shift = 0,	/* to be computed */
};

static void clockevent_set_mode(enum clock_event_mode mode,
				struct clock_event_device *clk_event_dev)
{
	u32	period;
	u32	val;

	/* stop the timer */
	val = readl(CTLR(clkevt_base));
	val &= ~CTLR_START;
	writel(val, CTLR(clkevt_base));

	switch (mode) {
	case CLOCK_EVT_MODE_PERIODIC:
		writel((SYSCLK/DIVISOR), LOAD(clkevt_base));
		val = readl( CTLR(clkevt_base));
/*		val |= CTLR_START | CTLR_CONTINOUS | CTLR_LOAD;*/
		val = CTLR_START | CTLR_LOAD;
		writel(val, CTLR(clkevt_base));

		break;
	case CLOCK_EVT_MODE_ONESHOT:
//		val = readl(CTLR(clkevt_base));
//		val &= ~CTLR_CONTINOUS;
		val = CTLR_START | CTLR_LOAD; 
		writel(val, CTLR(clkevt_base));

		break;
	case CLOCK_EVT_MODE_UNUSED:
	case CLOCK_EVT_MODE_SHUTDOWN:
	case CLOCK_EVT_MODE_RESUME:

		break;
	default:
		pr_err("Invalid mode requested\n");
		break;
	}
}

static int clockevent_next_event(unsigned long cycles,
				 struct clock_event_device *clk_event_dev)
{
	u32	val;
	char	buf[128];

	val = readl(CTLR(clkevt_base));

	if (val & CTLR_START)
		writew(val & ~CTLR_START, CTLR(clkevt_base));
#if 0	/* yamano debug */
sprintf(buf,"clockevent_next_event cycle=%x\n",cycles);
lm2_printk(0xfc000000,buf);
#endif	/* yamano debug */
	writel(cycles, LOAD(clkevt_base));

	val = CTLR_LOAD;
	writel(val, CTLR(clkevt_base));
	val = CTLR_START;
	writel(val, CTLR(clkevt_base));

	return 0;
}

/*
 * Interrupt handler by general timer
 */
static	u32	int_cnt=0;

static irqreturn_t lm2_timer_interrupt(int irq, void *dev_id)
{
	unsigned long	val;

	struct clock_event_device *evt = &clkevt;

	evt->event_handler(evt);
#if 0	/* yamano debug */
if(int_cnt >1000){
	lm2_printk(0xfc000000,".");
	int_cnt =0;
}else{
	int_cnt++;
}
#endif	/* yamano debug */
#if 0
	val = readl(CTLR(clkevt_base));
#endif	/* yamano debug */
	val = CTLR_LOAD|CTLR_START;
//	val = CTLR_START;
	writel(val,CTLR(clkevt_base));
	return IRQ_HANDLED;
}

/*
 * Interrupt resource for general timer
 */
static struct irqaction lm2_timer_irq = {
	.name = "timer",
	.flags = IRQF_DISABLED | IRQF_TIMER,
	.handler = lm2_timer_interrupt,
	.dev_id		= &clkevt,
};

/*
 * kernel timer event initialyzer
 */
void __init lm2_clockevent_init(int irq,void __iomem *gpt_base)
{
	u32 tick_rate;

	clkevt_base = gpt_base;

	tick_rate = SYSCLK / DIVISOR;

	clkevt.cpumask = cpumask_of(0);

	clockevents_config_and_register(&clkevt, tick_rate, 30, SYSCLK);

	/* register interrupt */
	setup_irq(irq, &lm2_timer_irq);
}


#if 0	/* yamano debug */
const static struct of_device_id timer_of_match[] __initconst = {
	{ .compatible = "quatro55xx,lm2-timer", },
	{ },
};

void __init lm2_setup_of_timer(void)
{
	struct device_node *np;
	int irq, ret;

	np = of_find_matching_node(NULL, timer_of_match);
	if (!np) {
		pr_err("%s: No timer passed via DT\n", __func__);
		return;
	}

	irq = irq_of_parse_and_map(np, 0);
	if (!irq) {
		pr_err("%s: No irq passed for timer via DT\n", __func__);
		return;
	}

	gpt_base = of_iomap(np, 0);
	if (!gpt_base) {
		pr_err("%s: of iomap failed\n", __func__);
		return;
	}

	gpt_clk = clk_get_sys("gpt0", NULL);
	if (!gpt_clk) {
		pr_err("%s:couldn't get clk for gpt\n", __func__);
		goto err_iomap;
	}

	ret = clk_prepare_enable(gpt_clk);
	if (ret < 0) {
		pr_err("%s:couldn't prepare-enable gpt clock\n", __func__);
		goto err_prepare_enable_clk;
	}

	lm2_clockevent_init(irq, gpt_base);
	lm2_clocksource_init(gpt_base);

	return;

err_prepare_enable_clk:
	clk_put(gpt_clk);
err_iomap:
	iounmap(gpt_base);
}
#endif

