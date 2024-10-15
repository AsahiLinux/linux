/*
 * SPDX-License-Identifier: GPL-2.0
 *
 * Apple SPI HID transport driver - Open Firmware
 *
 * Copyright (C) The Asahi Linux Contributors
 */

#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_irq.h>

#include "spi-hid-apple.h"


struct spihid_apple_of {
	struct spihid_apple_ops ops;

	struct gpio_desc *enable_gpio;
	int irq;
};

static int spihid_apple_of_power_on(struct spihid_apple_ops *ops)
{
	struct spihid_apple_of *sh_of = container_of(ops, struct spihid_apple_of, ops);

	/* reset the controller on boot */
	gpiod_direction_output(sh_of->enable_gpio, 1);
	msleep(5);
	gpiod_direction_output(sh_of->enable_gpio, 0);
	msleep(5);
	/* turn SPI device on */
	gpiod_direction_output(sh_of->enable_gpio, 1);
	msleep(50);

	return 0;
}

static int spihid_apple_of_power_off(struct spihid_apple_ops *ops)
{
	struct spihid_apple_of *sh_of = container_of(ops, struct spihid_apple_of, ops);

	/* turn SPI device off */
	gpiod_direction_output(sh_of->enable_gpio, 0);

	return 0;
}

static int spihid_apple_of_enable_irq(struct spihid_apple_ops *ops)
{
	struct spihid_apple_of *sh_of = container_of(ops, struct spihid_apple_of, ops);

	enable_irq(sh_of->irq);

	return 0;
}

static int spihid_apple_of_disable_irq(struct spihid_apple_ops *ops)
{
	struct spihid_apple_of *sh_of = container_of(ops, struct spihid_apple_of, ops);

	disable_irq(sh_of->irq);

	return 0;
}

static int spihid_apple_of_enable_irq_wake(struct spihid_apple_ops *ops)
{
	struct spihid_apple_of *sh_of = container_of(ops, struct spihid_apple_of, ops);

	return enable_irq_wake(sh_of->irq);
}

static int spihid_apple_of_disable_irq_wake(struct spihid_apple_ops *ops)
{
	struct spihid_apple_of *sh_of = container_of(ops, struct spihid_apple_of, ops);

	return disable_irq_wake(sh_of->irq);
}

static int spihid_apple_of_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct spihid_apple_of *spihid_of;
	int err;

	spihid_of = devm_kzalloc(dev, sizeof(*spihid_of), GFP_KERNEL);
	if (!spihid_of)
		return -ENOMEM;

	spihid_of->ops.power_on = spihid_apple_of_power_on;
	spihid_of->ops.power_off = spihid_apple_of_power_off;
	spihid_of->ops.enable_irq = spihid_apple_of_enable_irq;
	spihid_of->ops.disable_irq = spihid_apple_of_disable_irq;
	spihid_of->ops.enable_irq_wake = spihid_apple_of_enable_irq_wake;
	spihid_of->ops.disable_irq_wake = spihid_apple_of_disable_irq_wake;

	spihid_of->enable_gpio = devm_gpiod_get_index(dev, "spien", 0, 0);
	if (IS_ERR(spihid_of->enable_gpio)) {
		err = PTR_ERR(spihid_of->enable_gpio);
		dev_err(dev, "failed to get 'spien' gpio pin: %d", err);
		return err;
	}

	spihid_of->irq = of_irq_get(dev->of_node, 0);
	if (spihid_of->irq < 0) {
		err = spihid_of->irq;
		dev_err(dev, "failed to get 'extended-irq': %d", err);
		return err;
	}
	err = devm_request_threaded_irq(dev, spihid_of->irq, NULL,
					spihid_apple_core_irq, IRQF_ONESHOT | IRQF_NO_AUTOEN,
					"spi-hid-apple-irq", spi);
	if (err < 0) {
		dev_err(dev, "failed to request extended-irq %d: %d",
			spihid_of->irq, err);
		return err;
	}

	return spihid_apple_core_probe(spi, &spihid_of->ops);
}

static const struct of_device_id spihid_apple_of_match[] = {
	{ .compatible = "apple,spi-hid-transport" },
	{},
};
MODULE_DEVICE_TABLE(of, spihid_apple_of_match);

static struct spi_device_id spihid_apple_of_id[] = {
	{ "spi-hid-transport", 0 },
	{}
};
MODULE_DEVICE_TABLE(spi, spihid_apple_of_id);

static struct spi_driver spihid_apple_of_driver = {
	.driver = {
		.name	= "spi-hid-apple-of",
		.pm	= &spihid_apple_core_pm,
		.of_match_table = of_match_ptr(spihid_apple_of_match),
	},

	.id_table	= spihid_apple_of_id,
	.probe		= spihid_apple_of_probe,
	.remove		= spihid_apple_core_remove,
	.shutdown	= spihid_apple_core_shutdown,
};

module_spi_driver(spihid_apple_of_driver);

MODULE_LICENSE("GPL");
