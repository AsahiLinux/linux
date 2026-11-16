// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for TI TPS6598x USB Power Delivery controller family
 *
 * Copyright (C) The Asahi Linux Contributors
 */

#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/regmap.h>

#include "tps6598x.h"

static int tps6598x_probe_i2c(struct i2c_client *client)
{
	enum tipd_hw_variant variant;
	const struct tipd_data *data;
	struct tps6598x *tps;
	int ret;

	variant = (enum tipd_hw_variant)i2c_get_match_data(client);
	if (!variant || variant >= TIPD_VARIANT_MAX)
		return -EINVAL;
	data = tipd_hw_variants[variant];

	tps = devm_kzalloc(&client->dev, data->tps_struct_size, GFP_KERNEL);
	if (!tps)
		return -ENOMEM;

	mutex_init(&tps->lock);
	tps->dev = &client->dev;
	tps->data = data;
	tps->irq = client->irq;

	tps->regmap = devm_regmap_init_i2c(client, &tps6598x_regmap_config);
	if (IS_ERR(tps->regmap))
		return PTR_ERR(tps->regmap);

	/*
	 * Checking can the adapter handle SMBus protocol. If it can not, the
	 * driver needs to take care of block reads separately.
	 */
	if (i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		tps->i2c_protocol = true;

	ret = tipd_init(tps);

	if (ret == 0)
		i2c_set_clientdata(client, tps);

	return ret;
}

static void tps6598x_remove_i2c(struct i2c_client *client)
{
	struct tps6598x *tps = i2c_get_clientdata(client);

	tipd_remove(tps);
}

static int __maybe_unused tps6598x_suspend(struct device *dev)
{
	struct tps6598x *tps = dev_get_drvdata(dev);

	return tipd_suspend(tps);
}

static int __maybe_unused tps6598x_resume(struct device *dev)
{
	struct tps6598x *tps = dev_get_drvdata(dev);

	return tipd_resume(tps);
}

static const struct dev_pm_ops tps6598x_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(tps6598x_suspend, tps6598x_resume)
};

static const struct of_device_id tps6598x_of_match[] = {
	{ .compatible = "ti,tps6598x", (void *)TIPD_VARIANT_TPS6598X},
	{ .compatible = "apple,cd321x",  (void *)TIPD_VARIANT_CD321X},
	{ .compatible = "ti,tps25750",  (void *)TIPD_VARIANT_TPS25750},
	{}
};
MODULE_DEVICE_TABLE(of, tps6598x_of_match);

static const struct i2c_device_id tps6598x_id[] = {
	{ .name = "tps6598x", .driver_data = (kernel_ulong_t)TIPD_VARIANT_TPS6598X },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tps6598x_id);

static struct i2c_driver tps6598x_i2c_driver = {
	.driver = {
		.name = "tps6598x",
		.pm = &tps6598x_pm_ops,
		.of_match_table = tps6598x_of_match,
	},
	.probe = tps6598x_probe_i2c,
	.remove = tps6598x_remove_i2c,
	.id_table = tps6598x_id,
};
module_i2c_driver(tps6598x_i2c_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("TI TPS6598x USB Power Delivery Controller Driver");
