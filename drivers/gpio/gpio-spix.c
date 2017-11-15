/*
 * Copyright 2017 Dell-EMC.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/gpio/driver.h>
#include <linux/pinctrl/consumer.h>

#define DEFAULT_NUM_GPIOS  256;

/**
 * struct spix_gpio - GPIO device private data structure
 * @chip:	instance of the gpio_chip
 * @lock:	synchronization lock to prevent I/O race conditions
 * @base:	base port address of the GPIO device
 */
struct spix_gpio {
	struct gpio_chip chip;
	spinlock_t lock;                //TODO: Make this per-addressable area (8-bit or 32-bit), instead of 1 per chip
	void __iomem *base;
        unsigned char io_state[128];    
};

static int spix_gpio_get_direction(struct gpio_chip *gc, unsigned int offset)
{
#if 0 
	struct spix_gpio *const gpio = gpiochip_get_data(chip);
        const unsigned port = offset / 8;
        const unsigned mask = BIT(offset % 8);

        return !!(gpio->io_state[port] & mask);         TODO: drive from device tree
#endif
        return 0;
}

#if 0 
static int spix_gpio_dir_in(struct gpio_chip *gc, unsigned int offset)
{
	return 0;
}


static int spix_gpio_dir_out(struct gpio_chip *gc,
			       unsigned int offset, int val)
{
	return 0;
}
#endif

static int spix_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	struct spix_gpio *const gpio = gpiochip_get_data(chip);
	const unsigned int port = offset / 8;
	const unsigned int mask = BIT(offset % 8);
	unsigned char port_state;

	port_state = ioread8(gpio->base + port);

	return !!(port_state & mask);
}


static void spix_gpio_set(struct gpio_chip *chip, unsigned int offset,
	int value)
{
	struct spix_gpio *const gpio = gpiochip_get_data(chip);
	const unsigned int port = offset / 8;
	const unsigned int mask = BIT(offset % 8);
	unsigned long flags;
	unsigned char port_state;

	spin_lock_irqsave(&gpio->lock, flags);
	port_state = ioread8(gpio->base + port);

	if (value)
		port_state |= mask;
	else
		port_state &= ~mask;

	iowrite8(port_state, gpio->base + port);

	spin_unlock_irqrestore(&gpio->lock, flags);

}



static int __init spix_gpio_probe(struct platform_device *pdev)
{
	struct spix_gpio *gpio;
	struct resource *res;
        unsigned int ngpios;
	int rc;

	gpio = devm_kzalloc(&pdev->dev, sizeof(*gpio), GFP_KERNEL);
	if (!gpio)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	gpio->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(gpio->base))
		return PTR_ERR(gpio->base);

	spin_lock_init(&gpio->lock);

	gpio->chip.parent = &pdev->dev;

        if(of_property_read_u32(pdev->dev.of_node, "ngpios", &ngpios)){
            ngpios = DEFAULT_NUM_GPIOS;
        }

        gpio->chip.ngpio = (u16) ngpios;

#if 0   //TODO
        const unsigned port = offset / 8;
        const unsigned mask = BIT(offset % 8);
        return !!(gpio->io_state[port] & mask);

        for_each_available_child_of_node(pdev->dev.of_node, np) {
                if (of_property_read_bool(np, "input"))
                        printk(KERN_EMERG "%s(): input\n",np->name);
                if (of_property_read_bool(np, "output"))
                        printk(KERN_EMERG "%s(): output\n",np->name);

                offset = of_gpio_simple_xlate(&gpio->chip,

        }
#endif


	gpio->chip.get_direction = spix_gpio_get_direction;

	gpio->chip.get = spix_gpio_get;
	gpio->chip.set = spix_gpio_set;
	gpio->chip.label = dev_name(&pdev->dev);

	gpio->chip.base = -1;   /* dynamically calculate base */

	rc = devm_gpiochip_add_data(&pdev->dev, &gpio->chip, gpio);
	if (rc < 0)
		return rc;




	return 0;
}

static const struct of_device_id spix_gpio_of_table[] = {
	{ .compatible = "nuvoton,npcm750-spix" },
	{}
};
MODULE_DEVICE_TABLE(of, spix_gpio_of_table);

static struct platform_driver spix_gpio_driver = {
	.driver = {
		.name = KBUILD_MODNAME,
		.of_match_table = spix_gpio_of_table,
	},
};

module_platform_driver_probe(spix_gpio_driver, spix_gpio_probe);

MODULE_DESCRIPTION("SPIX GPIO Driver");
MODULE_LICENSE("GPL");
