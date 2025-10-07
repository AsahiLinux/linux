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
#include <linux/module.h>
#include <linux/reboot.h>

/**
 * struct macsmc_input
 * @dev: Underlying struct device for the input sub-device
 * @smc: Pointer to apple_smc struct of the mfd parent
 * @input: Allocated input_dev; devres managed
 * @nb: Notifier block used for incoming events from SMC (e.g. button pressed down)
 * @wakeup_mode: Set to true when system is suspended and power button events should wake it
 */
struct macsmc_input {
	struct device *dev;
	struct apple_smc *smc;
	struct input_dev *input;
	struct notifier_block nb;
	bool wakeup_mode;
};

#define SMC_EV_BTN 0x7201
#define SMC_EV_LID 0x7203

#define BTN_POWER		0x01 /* power button on e.g. Mac Mini chasis pressed */
#define BTN_TOUCHID		0x06 /* combined TouchID / power button on MacBooks pressed */
#define BTN_POWER_HELD_SHORT	0xfe /* power button briefly held down */
#define BTN_POWER_HELD_LONG	0x00 /* power button held down; sent just before forced poweroff */

static void macsmc_input_event_button(struct macsmc_input *smcin, unsigned long event)
{
	u8 button = (event >> 8) & 0xff;
	u8 state = !!(event & 0xff);

	switch (button) {
	case BTN_POWER:
	case BTN_TOUCHID:
		if (smcin->wakeup_mode) {
			if (state)
				pm_wakeup_event(smcin->dev, 0);
		} else {
			input_report_key(smcin->input, KEY_POWER, state);
			input_sync(smcin->input);
		}
		break;
	case BTN_POWER_HELD_SHORT: /* power button held down; ignore */
		break;
	case BTN_POWER_HELD_LONG:
		/*
		 * If we get here the power button has been held down for a while and
		 * we have about 4 seconds before forced power-off is triggered by SMC.
		 * Try to do an emergency shutdown to make sure the NVMe cache is
		 * flushed. macOS actually does this by panicing (!)...
		 */
		if (state) {
			dev_crit(smcin->dev, "Triggering forced shutdown!\n");
			if (kernel_can_power_off())
				kernel_power_off();
			else /* Missing macsmc-reboot driver? */
				kernel_restart("SMC power button triggered restart");
		}
		break;
	default:
		dev_warn(smcin->dev, "Unknown SMC button event: %04lx\n", event & 0xffff);
	}
}

static void macsmc_input_event_lid(struct macsmc_input *smcin, unsigned long event)
{
	u8 lid_state = !!((event >> 8) & 0xff);

	if (smcin->wakeup_mode && !lid_state)
		pm_wakeup_event(smcin->dev, 0);

	input_report_switch(smcin->input, SW_LID, lid_state);
	input_sync(smcin->input);
}

static int macsmc_input_event(struct notifier_block *nb, unsigned long event, void *data)
{
	struct macsmc_input *smcin = container_of(nb, struct macsmc_input, nb);
	u16 type = event >> 16;

	switch (type) {
	case SMC_EV_BTN:
		macsmc_input_event_button(smcin, event);
		return NOTIFY_OK;
	case SMC_EV_LID:
		macsmc_input_event_lid(smcin, event);
		return NOTIFY_OK;
	default:
		/* SMC event meant for another driver */
		return NOTIFY_DONE;
	}
}

static int macsmc_input_probe(struct platform_device *pdev)
{
	struct apple_smc *smc = dev_get_drvdata(pdev->dev.parent);
	struct macsmc_input *smcin;
	bool have_lid, have_power;
	int error;

	/* Bail early if this SMC neither supports power button nor lid events */
	have_lid = apple_smc_key_exists(smc, SMC_KEY(MSLD));
	have_power = apple_smc_key_exists(smc, SMC_KEY(bHLD));
	if (!have_lid && !have_power)
		return -ENODEV;

	smcin = devm_kzalloc(&pdev->dev, sizeof(*smcin), GFP_KERNEL);
	if (!smcin)
		return -ENOMEM;

	smcin->dev = &pdev->dev;
	smcin->smc = smc;
	platform_set_drvdata(pdev, smcin);

	smcin->input = devm_input_allocate_device(&pdev->dev);
	if (!smcin->input)
		return -ENOMEM;

	smcin->input->phys = "macsmc-input (0)";
	smcin->input->name = "Apple SMC power/lid events";

	if (have_lid)
		input_set_capability(smcin->input, EV_SW, SW_LID);
	if (have_power)
		input_set_capability(smcin->input, EV_KEY, KEY_POWER);

	if (have_lid) {
		u8 val;

		error = apple_smc_read_u8(smc, SMC_KEY(MSLD), &val);
		if (error < 0)
			dev_warn(&pdev->dev, "Failed to read initial lid state\n");
		else
			input_report_switch(smcin->input, SW_LID, val);
	}

	if (have_power) {
		u32 val;

		error = apple_smc_read_u32(smc, SMC_KEY(bHLD), &val);
		if (error < 0)
			dev_warn(&pdev->dev, "Failed to read initial power button state\n");
		else
			input_report_key(smcin->input, KEY_POWER, val & 1);
	}

	error = input_register_device(smcin->input);
	if (error) {
		dev_err(&pdev->dev, "Failed to register input device: %d\n", error);
		return error;
	}

	input_sync(smcin->input);

	smcin->nb.notifier_call = macsmc_input_event;
	blocking_notifier_chain_register(&smc->event_handlers, &smcin->nb);

	device_init_wakeup(&pdev->dev, 1);

	return 0;
}

static int macsmc_input_pm_prepare(struct device *dev)
{
	struct macsmc_input *smcin = dev_get_drvdata(dev);

	smcin->wakeup_mode = true;
	return 0;
}

static void macsmc_input_pm_complete(struct device *dev)
{
	struct macsmc_input *smcin = dev_get_drvdata(dev);

	smcin->wakeup_mode = false;
}

static const struct dev_pm_ops macsmc_input_pm_ops = {
	.prepare = macsmc_input_pm_prepare,
	.complete = macsmc_input_pm_complete,
};

static struct platform_driver macsmc_input_driver = {
	.driver = {
		.name = "macsmc-input",
		.pm = &macsmc_input_pm_ops,
	},
	.probe = macsmc_input_probe,
};
module_platform_driver(macsmc_input_driver);

MODULE_AUTHOR("Hector Martin <marcan@marcan.st>");
MODULE_LICENSE("Dual MIT/GPL");
MODULE_DESCRIPTION("Apple SMC input driver");
MODULE_ALIAS("platform:macsmc-input");
