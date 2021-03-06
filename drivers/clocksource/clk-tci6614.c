/*
 * TCI6614 timer implementation
 *
 * Cloned from davinci timer64 code.  Original copyrights follow
 *
 * Author: Kevin Hilman, MontaVista Software, Inc. <source@mvista.com>
 *
 * 2007 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#include <linux/kernel.h>
#include <linux/irq.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/clocksource.h>
#include <linux/clockchips.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>

#include <asm/sched_clock.h>

static struct clock_event_device tci6614_event;
static unsigned int tci6614_clock_tick_rate;

/*
 * This driver configures the 2 64-bit count-up timers as 4 independent
 * 32-bit count-up timers used as follows:
 */

enum {
	TID_CLOCKEVENT,
	TID_CLOCKSOURCE,
};

/* values for 'opts' field of struct timer_s */
#define TIMER_OPTS_DISABLED		0x01
#define TIMER_OPTS_ONESHOT		0x02
#define TIMER_OPTS_PERIODIC		0x04
#define TIMER_OPTS_STATE_MASK		0x07

/* Timer register offsets */
#define PID12			0x00
#define TIM12			0x10
#define TIM34			0x14
#define PRD12			0x18
#define PRD34			0x1c
#define TCR			0x20
#define TGCR			0x24

/* Timer register bitfields */
#define TCR_ENAMODE_DISABLE          0x0
#define TCR_ENAMODE_ONESHOT          0x1
#define TCR_ENAMODE_PERIODIC         0x2
#define TCR_ENAMODE_MASK             0x3

#define TGCR_TIMMODE_SHIFT           2
#define TGCR_TIMMODE_64BIT_GP        0x0
#define TGCR_TIMMODE_32BIT_UNCHAINED 0x1
#define TGCR_TIMMODE_64BIT_WDOG      0x2
#define TGCR_TIMMODE_32BIT_CHAINED   0x3

#define TGCR_TIM12RS_SHIFT           0
#define TGCR_TIM34RS_SHIFT           1
#define TGCR_RESET                   0x0
#define TGCR_UNRESET                 0x1
#define TGCR_RESET_MASK              0x3

struct timer_s {
	char *name;
	unsigned long period;
	unsigned long opts;
	unsigned long flags;
	void __iomem *base;
	unsigned long tim_off;
	unsigned long prd_off;
	unsigned long enamode_shift;
	struct irqaction irqaction;
};

static struct timer_s timers[];

static int timer32_config(struct timer_s *t)
{
	u32 tcr;

	tcr = __raw_readl(t->base + TCR);

	/* disable timer */
	tcr &= ~(TCR_ENAMODE_MASK << t->enamode_shift);
	__raw_writel(tcr, t->base + TCR);

	/* reset counter to zero, set new period */
	__raw_writel(0, t->base + t->tim_off);
	__raw_writel(t->period, t->base + t->prd_off);

	/* Set enable mode */
	if (t->opts & TIMER_OPTS_ONESHOT)
		tcr |= TCR_ENAMODE_ONESHOT << t->enamode_shift;
	else if (t->opts & TIMER_OPTS_PERIODIC)
		tcr |= TCR_ENAMODE_PERIODIC << t->enamode_shift;

	__raw_writel(tcr, t->base + TCR);

	return 0;
}

static inline u32 timer32_read(struct timer_s *t)
{
	return __raw_readl(t->base + t->tim_off);
}

static irqreturn_t timer_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *evt = &tci6614_event;

	evt->event_handler(evt);
	return IRQ_HANDLED;
}

/* called when 32-bit counter wraps */
static irqreturn_t freerun_interrupt(int irq, void *dev_id)
{
	return IRQ_HANDLED;
}

static struct timer_s timers[] = {
	[TID_CLOCKEVENT] = {
		.name		= "timer64-event",
		.opts		= TIMER_OPTS_DISABLED,
		.enamode_shift	= 6,
		.tim_off	= TIM12,
		.prd_off	= PRD12,
		.irqaction = {
			.flags   = IRQF_DISABLED | IRQF_TIMER,
			.handler = timer_interrupt,
		}
	},
	[TID_CLOCKSOURCE] = {
		.name		= "timer64-source",
		.period		= ~0,
		.opts		= TIMER_OPTS_PERIODIC,
		.enamode_shift	= 22,
		.tim_off	= TIM34,
		.prd_off	= PRD34,
		.irqaction = {
			.flags   = IRQF_DISABLED | IRQF_TIMER,
			.handler = freerun_interrupt,
		}
	},
};

/*
 * clocksource
 */
static cycle_t tci6614_read_cycles(struct clocksource *cs)
{
	struct timer_s *t = &timers[TID_CLOCKSOURCE];

	return (cycles_t)timer32_read(t);
}

static struct clocksource tci6614_source = {
	.rating		= 300,
	.read		= tci6614_read_cycles,
	.mask		= CLOCKSOURCE_MASK(32),
	.flags		= CLOCK_SOURCE_IS_CONTINUOUS,
};

/*
 * Overwrite weak default sched_clock with something more precise
 */
static u32 notrace tci6614_read_sched_clock(void)
{
	return timer32_read(&timers[TID_CLOCKSOURCE]);
}

/*
 * clockevent
 */
static int tci6614_set_next_event(unsigned long cycles,
				  struct clock_event_device *evt)
{
	struct timer_s *t = &timers[TID_CLOCKEVENT];

	t->period = cycles;
	timer32_config(t);
	return 0;
}

static void tci6614_set_mode(enum clock_event_mode mode,
			     struct clock_event_device *evt)
{
	struct timer_s *t = &timers[TID_CLOCKEVENT];

	switch (mode) {
	case CLOCK_EVT_MODE_PERIODIC:
		t->period = tci6614_clock_tick_rate / (HZ);
		t->opts &= ~TIMER_OPTS_STATE_MASK;
		t->opts |= TIMER_OPTS_PERIODIC;
		timer32_config(t);
		break;
	case CLOCK_EVT_MODE_ONESHOT:
		t->opts &= ~TIMER_OPTS_STATE_MASK;
		t->opts |= TIMER_OPTS_ONESHOT;
		break;
	case CLOCK_EVT_MODE_UNUSED:
	case CLOCK_EVT_MODE_SHUTDOWN:
		t->opts &= ~TIMER_OPTS_STATE_MASK;
		t->opts |= TIMER_OPTS_DISABLED;
		break;
	case CLOCK_EVT_MODE_RESUME:
		break;
	}
}

static struct clock_event_device tci6614_event = {
	.features       = CLOCK_EVT_FEAT_PERIODIC | CLOCK_EVT_FEAT_ONESHOT,
	.shift		= 32,
	.set_next_event	= tci6614_set_next_event,
	.set_mode	= tci6614_set_mode,
};


static const struct of_device_id tci6614_timer_ids[] __initconst = {
	{ .compatible = "ti,tci6614-timer" },
	{ }
};

int __init tci6614_timer_init(void)
{
	struct device_node *node;
	struct clk *clk;
	static char err[] __initdata = KERN_ERR
		"%s: can't register clocksource!\n";
	void __iomem *base;
	int irqs[2];
	int i, error;
	u32 tgcr;

	node = of_find_matching_node(NULL, tci6614_timer_ids);
	if (!node) {
		pr_err("tci6614-timer: no matching node\n");
		return -ENODEV;
	}

	irqs[0]  = irq_of_parse_and_map(node, 0);
	irqs[1]  = irq_of_parse_and_map(node, 1);
	if (irqs[0] == NO_IRQ || irqs[1] == NO_IRQ) {
		pr_err("tci6614-timer: failed to map interrupts\n");
		return -ENODEV;
	}

	base = of_iomap(node, 0);
	if (!base) {
		pr_err("tci6614-timer: failed to map registers\n");
		return -ENODEV;
	}

	clk = of_clk_get(node, 0);
	if (!clk) {
		pr_err("tci6614-timer: failed to get clock\n");
		iounmap(base);
		return -ENODEV;
	}

	error = clk_prepare_enable(clk);
	if (error) {
		pr_err("tci6614-timer: failed to enable clock\n");
		iounmap(base);
		clk_put(clk);
		return -ENODEV;
	}

	/* Disabled, Internal clock source */
	__raw_writel(0, base + TCR);

	/* reset both timers, no pre-scaler for timer34 */
	tgcr = 0;
	__raw_writel(tgcr, base + TGCR);

	/* Set both timers to unchained 32-bit */
	tgcr = TGCR_TIMMODE_32BIT_UNCHAINED << TGCR_TIMMODE_SHIFT;
	__raw_writel(tgcr, base + TGCR);

	/* Unreset timers */
	tgcr |= (TGCR_UNRESET << TGCR_TIM12RS_SHIFT) |
		(TGCR_UNRESET << TGCR_TIM34RS_SHIFT);
	__raw_writel(tgcr, base + TGCR);

	/* Init both counters to zero */
	__raw_writel(0, base + TIM12);
	__raw_writel(0, base + TIM34);

	/* Init of each timer as a 32-bit timer */
	for (i=0; i< ARRAY_SIZE(timers); i++) {
		struct timer_s *t = &timers[i];

		t->base = base;
		t->irqaction.name = t->name;
		t->irqaction.dev_id = (void *)t;

		setup_irq(irqs[i], &t->irqaction);
	}

	tci6614_clock_tick_rate = clk_get_rate(clk);

	/* setup clocksource */
	tci6614_source.name = timers[TID_CLOCKSOURCE].name;
	if (clocksource_register_hz(&tci6614_source, tci6614_clock_tick_rate))
		printk(err, tci6614_source.name);

	setup_sched_clock(tci6614_read_sched_clock, 32,
			  tci6614_clock_tick_rate);

	/* setup clockevent */
	tci6614_event.name = timers[TID_CLOCKEVENT].name;
	tci6614_event.mult = div_sc(tci6614_clock_tick_rate, NSEC_PER_SEC,
					 tci6614_event.shift);
	tci6614_event.max_delta_ns =
		clockevent_delta2ns(0xfffffffe, &tci6614_event);
	tci6614_event.min_delta_ns = 50000; /* 50 usec */

	tci6614_event.cpumask = cpumask_of(0);
	clockevents_register_device(&tci6614_event);

	for (i=0; i< ARRAY_SIZE(timers); i++)
		timer32_config(&timers[i]);

	pr_info("tci6614 clock @%d MHz\n", tci6614_clock_tick_rate);

	return 0;
}
