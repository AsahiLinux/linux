// SPDX-License-Identifier: GPL-2.0-only

#include <linux/backlight.h>
#include <drm/drm_mipi_dsi.h>
#include <video/mipi_display.h>

struct summit_data {
	struct mipi_dsi_device *dsi;
	struct backlight_device *bl;
};

static int summit_set_brightness(struct device *dev)
{
	struct summit_data *panel = dev_get_drvdata(dev);
	int level = backlight_get_brightness(panel->bl);
	ssize_t err = mipi_dsi_dcs_write(panel->dsi, MIPI_DCS_SET_DISPLAY_BRIGHTNESS,
					 &level, 1);
	if (err < 0)
		return err;
	return 0;
}

static int summit_bl_update_status(struct backlight_device *dev)
{
	return summit_set_brightness(&dev->dev);
}

static int summit_bl_get_brightness(struct backlight_device *dev)
{
	return backlight_get_brightness(dev);
}

static const struct backlight_ops summit_bl_ops = {
	.get_brightness = summit_bl_get_brightness,
	.update_status	= summit_bl_update_status,
};

static int summit_probe(struct mipi_dsi_device *dsi)
{
	struct backlight_properties props = { 0 };
	struct device *dev = &dsi->dev;
	struct summit_data *panel;
	panel = devm_kzalloc(dev, sizeof(*panel), GFP_KERNEL);
	if (!panel)
		return -ENOMEM;

	mipi_dsi_set_drvdata(dsi, panel);
	panel->dsi = dsi;
	props.max_brightness = 255;
	props.type = BACKLIGHT_RAW;

	panel->bl = devm_backlight_device_register(dev, dev_name(dev),
						   dev, panel, &summit_bl_ops, &props);
	if (IS_ERR(panel->bl)) {
		return PTR_ERR(panel->bl);
	}

	return mipi_dsi_attach(dsi);
}

static void summit_remove(struct mipi_dsi_device *dsi)
{
	mipi_dsi_detach(dsi);
}

static int summit_resume(struct device *dev)
{
	return summit_set_brightness(dev);
}

static int summit_suspend(struct device *dev)
{
	int level = 0;
	struct summit_data *panel = dev_get_drvdata(dev);
	ssize_t err = mipi_dsi_dcs_write(panel->dsi, MIPI_DCS_SET_DISPLAY_BRIGHTNESS,
					 &level, 1);
	if (err < 0)
		return err;
	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(summit_pm_ops, summit_suspend,
				summit_resume);

static const struct of_device_id summit_of_match[] = {
	{ .compatible = "apple,summit" },
	{},
};

MODULE_DEVICE_TABLE(of, summit_of_match);

static struct mipi_dsi_driver summit_driver = {
	.probe = summit_probe,
	.remove = summit_remove,
	.driver = {
		.name = "panel-summit",
		.of_match_table = summit_of_match,
		.pm = pm_sleep_ptr(&summit_pm_ops),
	},
};
module_mipi_dsi_driver(summit_driver);

MODULE_DESCRIPTION("Summit Display Panel Driver");
MODULE_LICENSE("GPL");
