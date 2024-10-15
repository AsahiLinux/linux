// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Copyright The Asahi Linux Contributors
 *
 * CPU idle support for Apple SoCs
 */

#include <linux/init.h>
#include <linux/bitfield.h>
#include <linux/cpuidle.h>
#include <linux/cpu_pm.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <asm/cpuidle.h>

#define DEEP_WFI_STATE_RETENTION BIT(2) // retains base CPU registers in deep WFI

enum idle_state {
	STATE_WFI,
	STATE_PWRDOWN,
	STATE_COUNT
};

asm(
	".pushsection .cpuidle.text, \"ax\"\n"
	".type apple_cpu_deep_wfi, @function\n"
	"apple_cpu_deep_wfi:\n"
		"str x30, [sp, #-16]!\n"
		"stp x28, x29, [sp, #-16]!\n"
		"stp x26, x27, [sp, #-16]!\n"
		"stp x24, x25, [sp, #-16]!\n"
		"stp x22, x23, [sp, #-16]!\n"
		"stp x20, x21, [sp, #-16]!\n"
		"stp x18, x19, [sp, #-16]!\n"

		"mrs x0, s3_5_c15_c5_0\n"
		"orr x0, x0, #(3L << 24)\n"
		"msr s3_5_c15_c5_0, x0\n"

	"1:\n"
		"dsb sy\n"
		"wfi\n"

		"mrs x0, ISR_EL1\n"
		"cbz x0, 1b\n"

		"mrs x0, s3_5_c15_c5_0\n"
		"bic x0, x0, #(1L << 24)\n"
		"msr s3_5_c15_c5_0, x0\n"

		"ldp x18, x19, [sp], #16\n"
		"ldp x20, x21, [sp], #16\n"
		"ldp x22, x23, [sp], #16\n"
		"ldp x24, x25, [sp], #16\n"
		"ldp x26, x27, [sp], #16\n"
		"ldp x28, x29, [sp], #16\n"
		"ldr x30, [sp], #16\n"

		"ret\n"
	".popsection\n"
);

void apple_cpu_deep_wfi(void);

static __cpuidle int apple_enter_wfi(struct cpuidle_device *dev, struct cpuidle_driver *drv, int index)
{
	cpu_do_idle();
	return index;
}

static __cpuidle int apple_enter_idle(struct cpuidle_device *dev, struct cpuidle_driver *drv, int index)
{
	/*
	 * Deep WFI will clobber FP state, among other things.
	 * The CPU PM notifier will take care of saving that and anything else
	 * that needs to be notified of the CPU powering down.
	 */
	if (cpu_pm_enter())
		return -1;

	ct_cpuidle_enter();

	switch(index) {
	case STATE_PWRDOWN:
		apple_cpu_deep_wfi();
		break;
	default:
		WARN_ON(1);
		break;
	}

	ct_cpuidle_exit();

	cpu_pm_exit();

	return index;
}

static struct cpuidle_driver apple_idle_driver = {
	.name = "apple_idle",
	.owner = THIS_MODULE,
	.states = {
		[STATE_WFI] = {
			.enter			= apple_enter_wfi,
			.enter_s2idle		= apple_enter_wfi,
			.exit_latency		= 1,
			.target_residency	= 1,
			.power_usage            = UINT_MAX,
			.name			= "WFI",
			.desc			= "CPU clock-gated",
			.flags			= 0,
		},
		[STATE_PWRDOWN] = {
			.enter			= apple_enter_idle,
			.enter_s2idle		= apple_enter_idle,
			.exit_latency		= 10,
			.target_residency	= 10000,
			.power_usage            = 0,
			.name			= "CPU PD",
			.desc			= "CPU/cluster powered down",
			.flags			= CPUIDLE_FLAG_RCU_IDLE,
		},
	},
	.safe_state_index = STATE_WFI,
	.state_count = STATE_COUNT,
};

static int apple_cpuidle_probe(struct platform_device *pdev)
{
	return cpuidle_register(&apple_idle_driver, NULL);
}

static struct platform_driver apple_cpuidle_driver = {
	.driver = {
		.name = "cpuidle-apple",
	},
	.probe = apple_cpuidle_probe,
};

static int __init apple_cpuidle_init(void)
{
	struct platform_device *pdev;
	int ret;

	ret = platform_driver_register(&apple_cpuidle_driver);
	if (ret)
		return ret;

	if (!of_machine_is_compatible("apple,arm-platform"))
		return 0;

	if (!FIELD_GET(DEEP_WFI_STATE_RETENTION, read_sysreg(aidr_el1))) {
		pr_info("cpuidle-apple: CPU does not retain state in deep WFI\n");
		return 0;
	}

	pdev = platform_device_register_simple("cpuidle-apple", -1, NULL, 0);
	if (IS_ERR(pdev)) {
		platform_driver_unregister(&apple_cpuidle_driver);
		return PTR_ERR(pdev);
	}

	return 0;
}
device_initcall(apple_cpuidle_init);
