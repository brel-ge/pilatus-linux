// SPDX-License-Identifier: GPL-2.0-only
/*
 * Toggles a GPIO pin to power down a device
 *
 * Jamie Lentin <jm@lentin.co.uk>
 * Andrew Lunn <andrew@lunn.ch>
 *
 * Copyright (C) 2012 Jamie Lentin
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/gpio/consumer.h>
#include <linux/of_platform.h>
#include <linux/module.h>

/*
 * Hold configuration here, cannot be more than one instance of the driver
 * since pm_power_off itself is global.
 */
static struct gpio_desc *hold_gpio;
static u32 delay = 100;

static void gpio_hold_pm_power_off(void)
{
	BUG_ON(!hold_gpio);

	mdelay(delay);
	gpiod_set_value_cansleep(hold_gpio, 0);
}

static int gpio_hold_probe(struct platform_device *pdev)
{
	/* If a pm_power_off function has already been added, leave it alone */
	/* if (pm_power_off != NULL) { */
	/* 	dev_err(&pdev->dev, */
	/* 		"%s: pm_power_off function already registered, %p %pf\n", */
	/* 	       __func__, pm_power_off, pm_power_off); */
	/* 	return -EBUSY; */
	/* } */

	hold_gpio = devm_gpiod_get(&pdev->dev, NULL, GPIOD_OUT_HIGH);
	if (IS_ERR(hold_gpio))
		return PTR_ERR(hold_gpio);

	pm_power_off = &gpio_hold_pm_power_off;
	return 0;
}

static int gpio_hold_remove(struct platform_device *pdev)
{
	if (pm_power_off == &gpio_hold_pm_power_off)
		pm_power_off = NULL;

	return 0;
}

static const struct of_device_id of_gpio_hold_match[] = {
	{ .compatible = "gpio-hold", },
	{},
};
MODULE_DEVICE_TABLE(of, of_gpio_hold_match);

static struct platform_driver gpio_hold_driver = {
	.probe = gpio_hold_probe,
	.remove = gpio_hold_remove,
	.driver = {
		.name = "hold-gpio",
		.of_match_table = of_gpio_hold_match,
	},
};

//module_platform_driver(gpio_hold_driver);
static int __init gpio_hold_init(void)
{
	return platform_driver_register(&gpio_hold_driver);
}
subsys_initcall(gpio_hold_init);

MODULE_AUTHOR("Uwe Kaiser <uwe.kaiser@brel.ch>");
MODULE_DESCRIPTION("GPIO hold driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:poweroff-gpio");
