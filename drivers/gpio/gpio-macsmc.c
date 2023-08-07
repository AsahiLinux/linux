// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Apple SMC GPIO driver
 * Copyright The Asahi Linux Contributors
 *
 * This driver implements basic SMC PMU GPIO support that can read inputs
 * and write outputs. Mode changes and IRQ config are not yet implemented.
 */

#include <linux/bitmap.h>
#include <linux/device.h>
#include <linux/gpio/driver.h>
#include <linux/irq.h>
#include <linux/mfd/core.h>
#include <linux/mfd/macsmc.h>

#define MAX_GPIO 64

/*
 * Commands 0-6 are, presumably, the intended API.
 * Command 0xff lets you get/set the pin configuration in detail directly,
 * but the bit meanings seem not to be stable between devices/PMU hardware
 * versions.
 *
 * We're going to try to make do with the low commands for now.
 * We don't implement pin mode changes at this time.
 */

#define CMD_ACTION	(0 << 24)
#define CMD_OUTPUT	(1 << 24)
#define CMD_INPUT	(2 << 24)
#define CMD_PINMODE	(3 << 24)
#define CMD_IRQ_ENABLE	(4 << 24)
#define CMD_IRQ_ACK	(5 << 24)
#define CMD_IRQ_MODE	(6 << 24)
#define CMD_CONFIG	(0xff << 24)

#define MODE_INPUT	0
#define MODE_OUTPUT	1
#define MODE_VALUE_0	0
#define MODE_VALUE_1	2

#define IRQ_MODE_HIGH		0
#define IRQ_MODE_LOW		1
#define IRQ_MODE_RISING		2
#define IRQ_MODE_FALLING	3
#define IRQ_MODE_BOTH		4

#define CONFIG_MASK	GENMASK(23, 16)
#define CONFIG_VAL	GENMASK(7, 0)

#define CONFIG_OUTMODE	GENMASK(7, 6)
#define CONFIG_IRQMODE	GENMASK(5, 3)
#define CONFIG_PULLDOWN	BIT(2)
#define CONFIG_PULLUP	BIT(1)
#define CONFIG_OUTVAL	BIT(0)

/*
 * output modes seem to differ depending on the PMU in use... ?
 * j274 / M1 (Sera PMU):
 *   0 = input
 *   1 = output
 *   2 = open drain
 *   3 = disable
 * j314 / M1Pro (Maverick PMU):
 *   0 = input
 *   1 = open drain
 *   2 = output
 *   3 = ?
 */

#define SMC_EV_GPIO 0x7202

struct macsmc_gpio {
	struct device *dev;
	struct apple_smc *smc;
	struct gpio_chip gc;
	struct irq_chip ic;
	struct notifier_block nb;

	struct mutex irq_mutex;
	DECLARE_BITMAP(irq_supported, MAX_GPIO);
	DECLARE_BITMAP(irq_enable_shadow, MAX_GPIO);
	DECLARE_BITMAP(irq_enable, MAX_GPIO);
	u32 irq_mode_shadow[MAX_GPIO];
	u32 irq_mode[MAX_GPIO];

	int first_index;
};

static int macsmc_gpio_nr(smc_key key)
{
	int low = hex_to_bin(key & 0xff);
	int high = hex_to_bin((key >> 8) & 0xff);

	if (low < 0 || high < 0)
		return -1;

	return low | (high << 4);
}

static int macsmc_gpio_key(unsigned int offset)
{
	return _SMC_KEY("gP\0\0") | (hex_asc_hi(offset) << 8) | hex_asc_lo(offset);
}

static int macsmc_gpio_get_direction(struct gpio_chip *gc, unsigned int offset)
{
	struct macsmc_gpio *smcgp = gpiochip_get_data(gc);
	smc_key key = macsmc_gpio_key(offset);
	u32 val;
	int ret;

	/* First try reading the explicit pin mode register */
	ret = apple_smc_rw_u32(smcgp->smc, key, CMD_PINMODE, &val);
	if (!ret)
		return (val & MODE_OUTPUT) ? GPIO_LINE_DIRECTION_OUT : GPIO_LINE_DIRECTION_IN;

	/*
	 * Less common IRQ configs cause CMD_PINMODE to fail, and so does open drain mode.
	 * Fall back to reading IRQ mode, which will only succeed for inputs.
	 */
	ret = apple_smc_rw_u32(smcgp->smc, key, CMD_IRQ_MODE, &val);
	return (!ret) ? GPIO_LINE_DIRECTION_IN : GPIO_LINE_DIRECTION_OUT;
}

static int macsmc_gpio_get(struct gpio_chip *gc, unsigned int offset)
{
	struct macsmc_gpio *smcgp = gpiochip_get_data(gc);
	smc_key key = macsmc_gpio_key(offset);
	u32 val;
	int ret;

	ret = macsmc_gpio_get_direction(gc, offset);
	if (ret < 0)
		return ret;

	if (ret == GPIO_LINE_DIRECTION_OUT)
		ret = apple_smc_rw_u32(smcgp->smc, key, CMD_OUTPUT, &val);
	else
		ret = apple_smc_rw_u32(smcgp->smc, key, CMD_INPUT, &val);

	if (ret < 0)
		return ret;

	return val ? 1 : 0;
}

static void macsmc_gpio_set(struct gpio_chip *gc, unsigned int offset, int value)
{
	struct macsmc_gpio *smcgp = gpiochip_get_data(gc);
	smc_key key = macsmc_gpio_key(offset);
	int ret;

	value |= CMD_OUTPUT;
	ret = apple_smc_write_u32(smcgp->smc, key, CMD_OUTPUT | value);
	if (ret < 0)
		dev_err(smcgp->dev, "GPIO set failed %p4ch = 0x%x\n", &key, value);
}

static int macsmc_gpio_init_valid_mask(struct gpio_chip *gc,
				       unsigned long *valid_mask, unsigned int ngpios)
{
	struct macsmc_gpio *smcgp = gpiochip_get_data(gc);
	int count = apple_smc_get_key_count(smcgp->smc) - smcgp->first_index;
	int i;

	if (count > MAX_GPIO)
		count = MAX_GPIO;

	bitmap_zero(valid_mask, ngpios);

	for (i = 0; i < count; i++) {
		smc_key key;
		int gpio_nr;
		u32 val;
		int ret = apple_smc_get_key_by_index(smcgp->smc, smcgp->first_index + i, &key);

		if (ret < 0)
			return ret;

		if (key > SMC_KEY(gPff))
			break;

		gpio_nr = macsmc_gpio_nr(key);
		if (gpio_nr < 0 || gpio_nr > MAX_GPIO) {
			dev_err(smcgp->dev, "Bad GPIO key %p4ch\n", &key);
			continue;
		}

		set_bit(gpio_nr, valid_mask);

		/* Check for IRQ support */
		ret = apple_smc_rw_u32(smcgp->smc, key, CMD_IRQ_MODE, &val);
		if (!ret)
			set_bit(gpio_nr, smcgp->irq_supported);
	}

	return 0;
}

static int macsmc_gpio_event(struct notifier_block *nb, unsigned long event, void *data)
{
	struct macsmc_gpio *smcgp = container_of(nb, struct macsmc_gpio, nb);
	u16 type = event >> 16;
	u8 offset = (event >> 8) & 0xff;
	smc_key key = macsmc_gpio_key(offset);
	unsigned long flags;

	if (type != SMC_EV_GPIO)
		return NOTIFY_DONE;

	if (offset > MAX_GPIO) {
		dev_err(smcgp->dev, "GPIO event index %d out of range\n", offset);
		return NOTIFY_BAD;
	}

	local_irq_save(flags);
	generic_handle_irq_desc(irq_resolve_mapping(smcgp->gc.irq.domain, offset));
	local_irq_restore(flags);

	if (apple_smc_write_u32(smcgp->smc, key, CMD_IRQ_ACK | 1) < 0)
		dev_err(smcgp->dev, "GPIO IRQ ack failed for %p4ch\n", &key);

	return NOTIFY_OK;
}

static void macsmc_gpio_irq_enable(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct macsmc_gpio *smcgp = gpiochip_get_data(gc);

	set_bit(irqd_to_hwirq(d), smcgp->irq_enable_shadow);
}

static void macsmc_gpio_irq_disable(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct macsmc_gpio *smcgp = gpiochip_get_data(gc);

	clear_bit(irqd_to_hwirq(d), smcgp->irq_enable_shadow);
}

static int macsmc_gpio_irq_set_type(struct irq_data *d, unsigned int type)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct macsmc_gpio *smcgp = gpiochip_get_data(gc);
	int offset = irqd_to_hwirq(d);
	u32 mode;

	if (!test_bit(offset, smcgp->irq_supported))
		return -EINVAL;

	switch (type & IRQ_TYPE_SENSE_MASK) {
	case IRQ_TYPE_LEVEL_HIGH:
		mode = IRQ_MODE_HIGH;
		break;
	case IRQ_TYPE_LEVEL_LOW:
		mode = IRQ_MODE_LOW;
		break;
	case IRQ_TYPE_EDGE_RISING:
		mode = IRQ_MODE_RISING;
		break;
	case IRQ_TYPE_EDGE_FALLING:
		mode = IRQ_MODE_FALLING;
		break;
	case IRQ_TYPE_EDGE_BOTH:
		mode = IRQ_MODE_BOTH;
		break;
	default:
		return -EINVAL;
	}

	smcgp->irq_mode_shadow[offset] = mode;
	return 0;
}

static void macsmc_gpio_irq_bus_lock(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct macsmc_gpio *smcgp = gpiochip_get_data(gc);

	mutex_lock(&smcgp->irq_mutex);
}

static void macsmc_gpio_irq_bus_sync_unlock(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct macsmc_gpio *smcgp = gpiochip_get_data(gc);
	smc_key key = macsmc_gpio_key(irqd_to_hwirq(d));
	int offset = irqd_to_hwirq(d);
	bool val;

	if (smcgp->irq_mode_shadow[offset] != smcgp->irq_mode[offset]) {
		u32 cmd = CMD_IRQ_MODE | smcgp->irq_mode_shadow[offset];
		if (apple_smc_write_u32(smcgp->smc, key, cmd) < 0)
			dev_err(smcgp->dev, "GPIO IRQ config failed for %p4ch = 0x%x\n", &key, cmd);
		else
			smcgp->irq_mode_shadow[offset] = smcgp->irq_mode[offset];
	}

	val = test_bit(offset, smcgp->irq_enable_shadow);
	if (test_bit(offset, smcgp->irq_enable) != val) {
		if (apple_smc_write_u32(smcgp->smc, key, CMD_IRQ_ENABLE | val) < 0)
			dev_err(smcgp->dev, "GPIO IRQ en/disable failed for %p4ch\n", &key);
		else
			change_bit(offset, smcgp->irq_enable);
	}

	mutex_unlock(&smcgp->irq_mutex);
}

static int macsmc_gpio_probe(struct platform_device *pdev)
{
	struct macsmc_gpio *smcgp;
	struct apple_smc *smc = dev_get_drvdata(pdev->dev.parent);
	smc_key key;
	int ret;

	smcgp = devm_kzalloc(&pdev->dev, sizeof(*smcgp), GFP_KERNEL);
	if (!smcgp)
		return -ENOMEM;

	pdev->dev.of_node = of_get_child_by_name(pdev->dev.parent->of_node, "gpio");

	smcgp->dev = &pdev->dev;
	smcgp->smc = smc;
	smcgp->first_index = apple_smc_find_first_key_index(smc, SMC_KEY(gP00));

	if (smcgp->first_index >= apple_smc_get_key_count(smc))
		return -ENODEV;

	ret = apple_smc_get_key_by_index(smc, smcgp->first_index, &key);
	if (ret < 0)
		return ret;

	if (key > macsmc_gpio_key(MAX_GPIO - 1))
		return -ENODEV;

	dev_info(smcgp->dev, "First GPIO key: %p4ch\n", &key);

	smcgp->gc.label = "macsmc-pmu-gpio";
	smcgp->gc.owner = THIS_MODULE;
	smcgp->gc.get = macsmc_gpio_get;
	smcgp->gc.set = macsmc_gpio_set;
	smcgp->gc.get_direction = macsmc_gpio_get_direction;
	smcgp->gc.init_valid_mask = macsmc_gpio_init_valid_mask;
	smcgp->gc.can_sleep = true;
	smcgp->gc.ngpio = MAX_GPIO;
	smcgp->gc.base = -1;
	smcgp->gc.parent = &pdev->dev;

	smcgp->ic.name = "macsmc-pmu-gpio";
	smcgp->ic.irq_mask = macsmc_gpio_irq_disable;
	smcgp->ic.irq_unmask = macsmc_gpio_irq_enable;
	smcgp->ic.irq_set_type = macsmc_gpio_irq_set_type;
	smcgp->ic.irq_bus_lock = macsmc_gpio_irq_bus_lock;
	smcgp->ic.irq_bus_sync_unlock = macsmc_gpio_irq_bus_sync_unlock;
	smcgp->ic.irq_set_type = macsmc_gpio_irq_set_type;
	smcgp->ic.flags = IRQCHIP_SET_TYPE_MASKED | IRQCHIP_MASK_ON_SUSPEND;

	smcgp->gc.irq.chip = &smcgp->ic;
	smcgp->gc.irq.parent_handler = NULL;
	smcgp->gc.irq.num_parents = 0;
	smcgp->gc.irq.parents = NULL;
	smcgp->gc.irq.default_type = IRQ_TYPE_NONE;
	smcgp->gc.irq.handler = handle_simple_irq;

	mutex_init(&smcgp->irq_mutex);

	smcgp->nb.notifier_call = macsmc_gpio_event;
	apple_smc_register_notifier(smc, &smcgp->nb);

	return devm_gpiochip_add_data(&pdev->dev, &smcgp->gc, smcgp);
}

static struct platform_driver macsmc_gpio_driver = {
	.driver = {
		.name = "macsmc-gpio",
	},
	.probe = macsmc_gpio_probe,
};
module_platform_driver(macsmc_gpio_driver);

MODULE_AUTHOR("Hector Martin <marcan@marcan.st>");
MODULE_LICENSE("Dual MIT/GPL");
MODULE_DESCRIPTION("Apple SMC GPIO driver");
MODULE_ALIAS("platform:macsmc-gpio");
