/*
 * gpio.c - GPIO driver
 *
 * Copyright (C) Wind River Systems, Inc.
 *
 */
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/platform_device.h>

/* Global contorl */
#define	PIOXMSK		0x010
#define	PIOINTSEL	0x014
#define	PIODSEL		0x018
#define	PIOINTSTS	0x01c

/* Port Control */
#define	PIODATA		0x020
#define	PIODATB		0x024
#define	PIODATC		0x028

#define	PIODIRA		0x040	/* Port Direction Register A */
#define	PIODIRB		0x044
#define	PIODIRC		0x048

#define	PIOMSKA		0x060	/* PIO Mask register A */
#define	PIOMSKB		0x064
#define	PIOMSKC		0x068

#define	PIOINTA		0x080	/* PIO Interrupt Cause Register A */
#define	PIOINTB		0x084
#define	PIOINTC		0x088

#define	PIODINA		0x0a0
#define	PIODINB		0x0a4
#define	PIODINC		0x0a8

#define	PIOSETA		0x0c0
#define	PIOSETB		0x0c4
#define	PIOSETC		0x0c8

#define	PIORESA		0x0e0
#define	PIORESB		0x0e4
#define	PIORESC		0x0e8

struct	mmgpios {
	struct	gpio_chip	chip;
	void __iomem		*regbase;
	spinlock_t		mmgpio_lock;
};

static	int	gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	struct mmgpios	*gpio = container_of(chip, struct mmgpios, chip);
	unsigned int	dat;
	int		shift;

//	dev_info(&chip->dev,"GOIP Get Function call offset = %d\n",offset);

	dat = 0;
	shift = 0;

	if(offset < 32){
		dat = readl(gpio->regbase + PIODATA);
		shift = offset;
	}
	if(offset > 31 && offset < 64){
		dat = readl(gpio->regbase + PIODATB);
		shift = offset - 32;
	}
	if(offset > 63 && offset < 96){
		dat = readl(gpio->regbase + PIODATC);
		shift = offset - 64;
	}
	dat = dat >> shift;
	dat &= 0x00000001;
	return	dat;

}

static	void	gpio_set(struct gpio_chip *chip, unsigned int offset, int value)
{
	struct mmgpios	*gpio = container_of(chip, struct mmgpios, chip);
	unsigned int	dat;

//	dev_info(&chip->dev,"GOIP Set Function call offset = %d value = %d \n",offset,value);

	if(value == 0){
		if(offset < 32){
			dat = readl(gpio->regbase + PIODATA);
			dat &= ~(1 << offset);
			writel(dat, gpio->regbase + PIODATA);
		}else if(offset > 31 && offset < 64){
			dat = readl(gpio->regbase + PIODATB);
			dat &= ~(1  << (offset - 32));
			writel(dat, gpio->regbase + PIODATB);
		}else if(offset > 63 && offset < 96){
			dat = readl(gpio->regbase + PIODATC);
			dat &= ~(1 << (offset - 64));
			writel(dat, gpio->regbase + PIODATC);
		}else{
			dev_err(&chip->dev, "GPIO could not access port\n");
		}
	}else{
		if(offset < 32){
			dat = readl(gpio->regbase + PIODATA);
			dat |= value << offset;
			writel(dat, gpio->regbase + PIODATA);
		}else if(offset > 31 && offset < 64){
			dat = readl(gpio->regbase + PIODATB);
			dat |= value  << (offset - 32);
			writel(dat, gpio->regbase + PIODATB);
		}else if(offset > 63 && offset < 96){
			dat = readl(gpio->regbase + PIODATC);
			dat |= value << (offset - 64);
			writel(dat, gpio->regbase + PIODATC);
		}else{
			dev_err(&chip->dev, "GPIO could not access port\n");
		}
	}
}

static	int	gpio_probe(struct platform_device *pdev)
{
	struct device		*dev = &pdev->dev;
	struct resource		*res;
	struct mmgpios		*gpio;
	struct gpio_chip	*chip;
	int			result;

	dev_info(&pdev->dev,"GPIO Driver probing\n");

	gpio = kzalloc(sizeof(struct mmgpios), GFP_KERNEL);
	if(!gpio){
		dev_err(&pdev->dev,"Could't allocate memory for GPIO private space\n");
		return	-ENOMEM;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if(!res){
		dev_err(&pdev->dev,"no mmio space\n");
		return	-EINVAL;
	}
	gpio->regbase = ioremap(res->start, 0x100 );

	spin_lock_init(&gpio->mmgpio_lock);

	platform_set_drvdata(pdev, gpio);

	chip = &gpio->chip;
	chip->label ="mmio-gpio";
	chip->owner = THIS_MODULE;
	chip->dev = &pdev->dev;
	chip->get = gpio_get;
	chip->set = gpio_set;
	chip->ngpio = 96;

	result = gpiochip_add(chip);
	if(result < 0){
		dev_err(&pdev->dev,"GPIO resiter fail\n");
	}else{
		dev_info(&pdev->dev,"GPIO mapped\n");
	}
	return	0;
}

static	int	gpio_remove(struct platform_device *pdev)
{
	return	0;
}

static	const struct platform_device_id gpio_id_table[] = {
	{ "mmio-gpio",},
	{},
};

MODULE_DEVICE_TABLE( platform, gpio_id_table );

static	struct platform_driver gpio_driver = {
	.driver = {
		.name	= "mmio-gpio",
	},
	.id_table	= gpio_id_table,
	.probe		= gpio_probe,
	.remove		= gpio_remove,
};

module_platform_driver(gpio_driver);

MODULE_DESCRIPTION("Driver for MMIO GPIO COntroller");
MODULE_LICENSE("GPL");
