// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Apple SMC input event driver
 * Copyright The Asahi Linux Contributors
 *
 * This driver exposes HID events from the SMC as an input device.
 * This includes the lid open/close and power button notifications.
 */

#include <linux/device.h>
#include <linux/input.h>
#include <linux/mfd/core.h>
#include <linux/mfd/macsmc.h>
#include <linux/reboot.h>

struct macsmc_hid {
	struct device *dev;
	struct apple_smc *smc;
	struct input_dev *input;
	struct notifier_block nb;
	bool wakeup_mode;
};

#define SMC_EV_BTN 0x7201
#define SMC_EV_LID 0x7203

#define BTN_POWER	0x01
#define BTN_TOUCHID	0x06
#define BTN_POWER_HELD1	0xfe
#define BTN_POWER_HELD2	0x00

static int macsmc_hid_event(struct notifier_block *nb, unsigned long event, void *data)
{
	struct macsmc_hid *smchid = container_of(nb, struct macsmc_hid, nb);
	u16 type = event >> 16;
	u8 d1 = (event >> 8) & 0xff;
	u8 d2 = event & 0xff;

	switch (type) {
	case SMC_EV_BTN:
		switch (d1) {
		case BTN_POWER:
		case BTN_TOUCHID:
			if (smchid->wakeup_mode) {
				if (d2) {
					dev_info(smchid->dev, "Button wakeup\n");
					pm_wakeup_hard_event(smchid->dev);
				}
			} else {
				input_report_key(smchid->input, KEY_POWER, d2);
				input_sync(smchid->input);
			}
			break;
		case BTN_POWER_HELD1:
			/*
			 * TODO: is this pre-warning useful?
			 */
			if (d2)
				dev_warn(smchid->dev, "Power button held down\n");
			break;
		case BTN_POWER_HELD2:
			/*
			 * If we get here, we have about 4 seconds before forced shutdown.
			 * Try to do an emergency shutdown to make sure the NVMe cache is
			 * flushed. macOS actually does this by panicing (!)...
			 */
			if (d2) {
				dev_crit(smchid->dev, "Triggering forced shutdown!\n");
				if (kernel_can_power_off())
					kernel_power_off();
				else /* Missing macsmc-reboot driver? */
					kernel_restart("SMC power button triggered restart");
			}
			break;
		default:
			dev_info(smchid->dev, "Unknown SMC button event: %02x %02x\n", d1, d2);
			break;
		}
		return NOTIFY_OK;
	case SMC_EV_LID:
		if (smchid->wakeup_mode && !d1) {
			dev_info(smchid->dev, "Lid wakeup\n");
			pm_wakeup_hard_event(smchid->dev);
		}
		input_report_switch(smchid->input, SW_LID, d1);
		input_sync(smchid->input);
		return NOTIFY_OK;
	}

	return NOTIFY_DONE;
}

static int macsmc_hid_probe(struct platform_device *pdev)
{
	struct apple_smc *smc = dev_get_drvdata(pdev->dev.parent);
	struct macsmc_hid *smchid;
	bool have_lid, have_power;
	int ret;

	have_lid = apple_smc_key_exists(smc, SMC_KEY(MSLD));
	have_power = apple_smc_key_exists(smc, SMC_KEY(bHLD));

	if (!have_lid && !have_power)
		return -ENODEV;

	smchid = devm_kzalloc(&pdev->dev, sizeof(*smchid), GFP_KERNEL);
	if (!smchid)
		return -ENOMEM;

	smchid->dev = &pdev->dev;
	smchid->smc = smc;
	platform_set_drvdata(pdev, smchid);

	smchid->input = devm_input_allocate_device(&pdev->dev);
	if (!smchid->input)
		return -ENOMEM;

	smchid->input->phys = "macsmc-hid (0)";
	smchid->input->name = "Apple SMC power/lid events";

	if (have_lid)
		input_set_capability(smchid->input, EV_SW, SW_LID);
	if (have_power)
		input_set_capability(smchid->input, EV_KEY, KEY_POWER);

	ret = input_register_device(smchid->input);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register input device: %d\n", ret);
		return ret;
	}

	if (have_lid) {
		u8 val;

		ret = apple_smc_read_u8(smc, SMC_KEY(MSLD), &val);
		if (ret < 0) {
			dev_err(&pdev->dev, "Failed to read initial lid state\n");
		} else {
			input_report_switch(smchid->input, SW_LID, val);
		}
	}
	if (have_power) {
		u32 val;

		ret = apple_smc_read_u32(smc, SMC_KEY(bHLD), &val);
		if (ret < 0) {
			dev_err(&pdev->dev, "Failed to read initial power button state\n");
		} else {
			input_report_key(smchid->input, KEY_POWER, val & 1);
		}
	}

	input_sync(smchid->input);

	smchid->nb.notifier_call = macsmc_hid_event;
	apple_smc_register_notifier(smc, &smchid->nb);

	device_init_wakeup(&pdev->dev, 1);

	return 0;
}

static int macsmc_hid_pm_prepare(struct device *dev)
{
	struct macsmc_hid *smchid = dev_get_drvdata(dev);

	smchid->wakeup_mode = true;
	return 0;
}

static void macsmc_hid_pm_complete(struct device *dev)
{
	struct macsmc_hid *smchid = dev_get_drvdata(dev);

	smchid->wakeup_mode = false;
}

static const struct dev_pm_ops macsmc_hid_pm_ops = {
	.prepare = macsmc_hid_pm_prepare,
	.complete = macsmc_hid_pm_complete,
};

static struct platform_driver macsmc_hid_driver = {
	.driver = {
		.name = "macsmc-hid",
		.pm = &macsmc_hid_pm_ops,
	},
	.probe = macsmc_hid_probe,
};
module_platform_driver(macsmc_hid_driver);

MODULE_AUTHOR("Hector Martin <marcan@marcan.st>");
MODULE_LICENSE("Dual MIT/GPL");
MODULE_DESCRIPTION("Apple SMC GPIO driver");
MODULE_ALIAS("platform:macsmc-hid");
