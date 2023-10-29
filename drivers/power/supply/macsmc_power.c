// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Apple SMC Power/Battery Management
 * Copyright The Asahi Linux Contributors
 */

#include <linux/ctype.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/mfd/core.h>
#include <linux/mfd/macsmc.h>
#include <linux/power_supply.h>
#include <linux/reboot.h>
#include <linux/delay.h>
#include <linux/workqueue.h>

#define MAX_STRING_LENGTH 256

struct macsmc_power {
	struct device *dev;
	struct apple_smc *smc;
	struct power_supply_desc batt_desc;

	struct power_supply *batt;
	char model_name[MAX_STRING_LENGTH];
	char serial_number[MAX_STRING_LENGTH];
	char mfg_date[MAX_STRING_LENGTH];
	bool has_chwa;

	struct power_supply *ac;

	struct notifier_block nb;

	struct work_struct critical_work;
	bool shutdown_started;

	struct delayed_work dbg_log_work;
};

static int macsmc_log_power_set(const char *val, const struct kernel_param *kp);

static const struct kernel_param_ops macsmc_log_power_ops = {
        .set = macsmc_log_power_set,
        .get = param_get_bool,
};

static bool log_power = false;
module_param_cb(log_power, &macsmc_log_power_ops, &log_power, 0644);
MODULE_PARM_DESC(log_power, "Periodically log power consumption for debugging");

#define POWER_LOG_INTERVAL (HZ)

static struct macsmc_power *g_power;

#define CHNC_BATTERY_FULL	BIT(0)
#define CHNC_NO_CHARGER		BIT(7)
#define CHNC_NOCHG_CH0C		BIT(14)
#define CHNC_NOCHG_CH0B_CH0K	BIT(15)
#define CHNC_BATTERY_FULL_2	BIT(18)
#define CHNC_BMS_BUSY		BIT(23)
#define CHNC_NOAC_CH0J		BIT(53)
#define CHNC_NOAC_CH0I		BIT(54)

#define CH0R_LOWER_FLAGS	GENMASK(15, 0)
#define CH0R_NOAC_CH0I		BIT(0)
#define CH0R_NOAC_DISCONNECTED	BIT(4)
#define CH0R_NOAC_CH0J		BIT(5)
#define CH0R_BMS_BUSY		BIT(8)
#define CH0R_NOAC_CH0K		BIT(9)
#define CH0R_NOAC_CHWA		BIT(11)

#define CH0X_CH0C		BIT(0)
#define CH0X_CH0B		BIT(1)

#define ACSt_CAN_BOOT_AP	BIT(2)
#define ACSt_CAN_BOOT_IBOOT	BIT(1)

#define CHWA_FIXED_START_THRESHOLD	75
#define CHWA_FIXED_END_THRESHOLD	80
#define CHWA_PROP_WRITE_THRESHOLD	95

static void macsmc_do_dbg(struct macsmc_power *power)
{
	int p_in = 0, p_sys = 0, p_3v8 = 0, p_mpmu = 0, p_spmu = 0, p_clvr = 0, p_cpu = 0;
	s32 p_bat = 0;
	s16 t_full = 0, t_empty = 0;
	u8 charge = 0;

	apple_smc_read_f32_scaled(power->smc, SMC_KEY(PDTR), &p_in, 1000);
	apple_smc_read_f32_scaled(power->smc, SMC_KEY(PSTR), &p_sys, 1000);
	apple_smc_read_f32_scaled(power->smc, SMC_KEY(PMVR), &p_3v8, 1000);
	apple_smc_read_f32_scaled(power->smc, SMC_KEY(PHPC), &p_cpu, 1000);
	apple_smc_read_f32_scaled(power->smc, SMC_KEY(PSVR), &p_clvr, 1000);
	apple_smc_read_f32_scaled(power->smc, SMC_KEY(PPMC), &p_mpmu, 1000);
	apple_smc_read_f32_scaled(power->smc, SMC_KEY(PPSC), &p_spmu, 1000);
	apple_smc_read_s32(power->smc, SMC_KEY(B0AP), &p_bat);
	apple_smc_read_s16(power->smc, SMC_KEY(B0TE), &t_empty);
	apple_smc_read_s16(power->smc, SMC_KEY(B0TF), &t_full);
	apple_smc_read_u8(power->smc, SMC_KEY(BUIC), &charge);

#define FD3(x) ((x) / 1000), abs((x) % 1000)
	dev_info(power->dev,
		 "In %2d.%03dW Sys %2d.%03dW 3V8 %2d.%03dW MPMU %2d.%03dW SPMU %2d.%03dW "
		 "CLVR %2d.%03dW CPU %2d.%03dW Batt %2d.%03dW %d%% T%s %dm\n",
		 FD3(p_in), FD3(p_sys), FD3(p_3v8), FD3(p_mpmu), FD3(p_spmu), FD3(p_clvr),
		 FD3(p_cpu), FD3(p_bat), charge,
		 t_full >= 0 ? "full" : "empty",
		 t_full >= 0 ? t_full : t_empty);
#undef FD3
}

static int macsmc_battery_get_status(struct macsmc_power *power)
{
	u64 nocharge_flags;
	u32 nopower_flags;
	u16 ac_current;
	bool chwa_limit = false;
	int ret;

	/*
	 * Note: there are fallbacks in case some of these SMC keys disappear in the future
	 * or are not present on some machines. We treat the absence of the CHCE/CHCC/BSFC/CHSC
	 * flags as an error, since they are quite fundamental and simple booleans.
	 */

	/*
	 * If power input is inhibited, we are definitely discharging.
	 * However, if the only reason is the BMS is doing a balancing cycle,
	 * go ahead and ignore that one to avoid spooking users.
	 */
	ret = apple_smc_read_u32(power->smc, SMC_KEY(CH0R), &nopower_flags);
	if (!ret && (nopower_flags & CH0R_LOWER_FLAGS & ~CH0R_BMS_BUSY))
		return POWER_SUPPLY_STATUS_DISCHARGING;

	/* If no charger is present, we are definitely discharging. */
	ret = apple_smc_read_flag(power->smc, SMC_KEY(CHCE));
	if (ret < 0)
		return ret;
	else if (!ret)
		return POWER_SUPPLY_STATUS_DISCHARGING;

	/* If AC is not charge capable, we are definitely discharging. */
	ret = apple_smc_read_flag(power->smc, SMC_KEY(CHCC));
	if (ret < 0)
		return ret;
	else if (!ret)
		return POWER_SUPPLY_STATUS_DISCHARGING;

	/*
	 * If the AC input current limit is tiny or 0, we are discharging no matter
	 * how much the BMS believes it can charge.
	 */
	ret = apple_smc_read_u16(power->smc, SMC_KEY(AC-i), &ac_current);
	if (!ret && ac_current < 100)
		return POWER_SUPPLY_STATUS_DISCHARGING;

	/* If the battery is full, report it as such. */
	ret = apple_smc_read_flag(power->smc, SMC_KEY(BSFC));
	if (ret < 0)
		return ret;
	else if (ret)
		return POWER_SUPPLY_STATUS_FULL;

	/*
	 * If we have charge limits supported and enabled and the SoC is > 75%,
	 * that means we are not charging for that reason (if not charging).
	 */
	if (power->has_chwa && apple_smc_read_flag(power->smc, SMC_KEY(CHWA)) == 1) {
		u8 buic = 0;

		if (apple_smc_read_u8(power->smc, SMC_KEY(BUIC), &buic) >= 0 &&
			buic >= CHWA_FIXED_START_THRESHOLD)
			chwa_limit = true;
	}

	/* If there are reasons we aren't charging... */
	ret = apple_smc_read_u64(power->smc, SMC_KEY(CHNC), &nocharge_flags);
	if (!ret) {
		/* Perhaps the battery is full after all */
		if (nocharge_flags & CHNC_BATTERY_FULL)
			return POWER_SUPPLY_STATUS_FULL;
		/*
		 * Or maybe the BMS is just busy doing something, if so call it charging anyway.
		 * But CHWA limits show up as this, so exclude those.
		 */
		else if (nocharge_flags == CHNC_BMS_BUSY && !chwa_limit)
			return POWER_SUPPLY_STATUS_CHARGING;
		/* If we have other reasons we aren't charging, say we aren't */
		else if (nocharge_flags)
			return POWER_SUPPLY_STATUS_NOT_CHARGING;
		/* Else we're either charging or about to charge */
		else
			return POWER_SUPPLY_STATUS_CHARGING;
	}

	/* As a fallback, use the system charging flag. */
	ret = apple_smc_read_flag(power->smc, SMC_KEY(CHSC));
	if (ret < 0)
		return ret;
	if (!ret)
		return POWER_SUPPLY_STATUS_NOT_CHARGING;
	else
		return POWER_SUPPLY_STATUS_CHARGING;
}

static int macsmc_battery_get_charge_behaviour(struct macsmc_power *power)
{
	int ret;
	u8 val;

	/* CH0I returns a bitmask like the low byte of CH0R */
	ret = apple_smc_read_u8(power->smc, SMC_KEY(CH0I), &val);
	if (ret)
		return ret;
	if (val & CH0R_NOAC_CH0I)
		return POWER_SUPPLY_CHARGE_BEHAVIOUR_FORCE_DISCHARGE;

	/* CH0C returns a bitmask containing CH0B/CH0C flags */
	ret = apple_smc_read_u8(power->smc, SMC_KEY(CH0C), &val);
	if (ret)
		return ret;
	if (val & CH0X_CH0C)
		return POWER_SUPPLY_CHARGE_BEHAVIOUR_INHIBIT_CHARGE;
	else
		return POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO;
}

static int macsmc_battery_set_charge_behaviour(struct macsmc_power *power, int val)
{
	u8 ch0i, ch0c;
	int ret;

	/*
	 * CH0I/CH0C are "hard" controls that will allow the battery to run down to 0.
	 * CH0K/CH0B are "soft" controls that are reset to 0 when SOC drops below 50%;
	 * we don't expose these yet.
	 */

	switch (val) {
	case POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO:
		ch0i = ch0c = 0;
		break;
	case POWER_SUPPLY_CHARGE_BEHAVIOUR_INHIBIT_CHARGE:
		ch0i = 0;
		ch0c = 1;
		break;
	case POWER_SUPPLY_CHARGE_BEHAVIOUR_FORCE_DISCHARGE:
		ch0i = 1;
		ch0c = 0;
		break;
	default:
		return -EINVAL;
	}

	ret = apple_smc_write_u8(power->smc, SMC_KEY(CH0I), ch0i);
	if (ret)
		return ret;
	return apple_smc_write_u8(power->smc, SMC_KEY(CH0C), ch0c);
}

static int macsmc_battery_get_date(const char *s, int *out)
{
	if (!isdigit(s[0]) || !isdigit(s[1]))
		return -ENOTSUPP;

	*out = (s[0] - '0') * 10 + s[1] - '0';
	return 0;
}

static int macsmc_battery_get_capacity_level(struct macsmc_power *power)
{
	u32 val;
	int ret;

	/* Check for emergency shutdown condition */
	if (apple_smc_read_u32(power->smc, SMC_KEY(BCF0), &val) >= 0 && val)
		return POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;

	/* Check AC status for whether we could boot in this state */
	if (apple_smc_read_u32(power->smc, SMC_KEY(ACSt), &val) >= 0) {
		if (!(val & ACSt_CAN_BOOT_IBOOT))
			return POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;

		if (!(val & ACSt_CAN_BOOT_AP))
			return POWER_SUPPLY_CAPACITY_LEVEL_LOW;
	}

	/* Check battery full flag */
	ret = apple_smc_read_flag(power->smc, SMC_KEY(BSFC));
	if (ret > 0)
		return POWER_SUPPLY_CAPACITY_LEVEL_FULL;
	else if (ret == 0)
		return POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;
	else
		return POWER_SUPPLY_CAPACITY_LEVEL_UNKNOWN;
}

static int macsmc_battery_get_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
	struct macsmc_power *power = power_supply_get_drvdata(psy);
	int ret = 0;
	u8 vu8;
	u16 vu16;
	u32 vu32;
	s16 vs16;
	s32 vs32;
	s64 vs64;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = macsmc_battery_get_status(power);
		ret = val->intval < 0 ? val->intval : 0;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR:
		val->intval = macsmc_battery_get_charge_behaviour(power);
		ret = val->intval < 0 ? val->intval : 0;
		break;
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW:
		ret = apple_smc_read_u16(power->smc, SMC_KEY(B0TE), &vu16);
		val->intval = vu16 == 0xffff ? 0 : vu16 * 60;
		break;
	case POWER_SUPPLY_PROP_TIME_TO_FULL_NOW:
		ret = apple_smc_read_u16(power->smc, SMC_KEY(B0TF), &vu16);
		val->intval = vu16 == 0xffff ? 0 : vu16 * 60;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		ret = apple_smc_read_u8(power->smc, SMC_KEY(BUIC), &vu8);
		val->intval = vu8;
		break;
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		val->intval = macsmc_battery_get_capacity_level(power);
		ret = val->intval < 0 ? val->intval : 0;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = apple_smc_read_u16(power->smc, SMC_KEY(B0AV), &vu16);
		val->intval = vu16 * 1000;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = apple_smc_read_s16(power->smc, SMC_KEY(B0AC), &vs16);
		val->intval = vs16 * 1000;
		break;
	case POWER_SUPPLY_PROP_POWER_NOW:
		ret = apple_smc_read_s32(power->smc, SMC_KEY(B0AP), &vs32);
		val->intval = vs32 * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		ret = apple_smc_read_u16(power->smc, SMC_KEY(BITV), &vu16);
		val->intval = vu16 * 1000;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
		ret = apple_smc_read_u16(power->smc, SMC_KEY(B0RC), &vu16);
		val->intval = vu16 * 1000;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		ret = apple_smc_read_u32(power->smc, SMC_KEY(CSIL), &vu32);
		val->intval = vu32 * 1000;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		ret = apple_smc_read_u16(power->smc, SMC_KEY(B0RI), &vu16);
		val->intval = vu16 * 1000;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		ret = apple_smc_read_u16(power->smc, SMC_KEY(B0RV), &vu16);
		val->intval = vu16 * 1000;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		ret = apple_smc_read_u16(power->smc, SMC_KEY(B0DC), &vu16);
		val->intval = vu16 * 1000;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		ret = apple_smc_read_u16(power->smc, SMC_KEY(B0FC), &vu16);
		val->intval = vu16 * 1000;
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		ret = apple_smc_read_u16(power->smc, SMC_KEY(B0RM), &vu16);
		val->intval = swab16(vu16) * 1000;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		ret = apple_smc_read_u16(power->smc, SMC_KEY(B0AT), &vu16);
		val->intval = vu16 - 2732;
		break;
	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
		ret = apple_smc_read_s64(power->smc, SMC_KEY(BAAC), &vs64);
		val->intval = vs64;
		break;
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		ret = apple_smc_read_u16(power->smc, SMC_KEY(B0CT), &vu16);
		val->intval = vu16;
		break;
	case POWER_SUPPLY_PROP_SCOPE:
		val->intval = POWER_SUPPLY_SCOPE_SYSTEM;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		ret = apple_smc_read_flag(power->smc, SMC_KEY(BBAD));
		val->intval = ret == 1 ? POWER_SUPPLY_HEALTH_DEAD : POWER_SUPPLY_HEALTH_GOOD;
		ret = ret < 0 ? ret : 0;
		break;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = power->model_name;
		break;
	case POWER_SUPPLY_PROP_SERIAL_NUMBER:
		val->strval = power->serial_number;
		break;
	case POWER_SUPPLY_PROP_MANUFACTURE_YEAR:
		ret = macsmc_battery_get_date(&power->mfg_date[0], &val->intval);
		val->intval += 2000 - 8; /* -8 is a fixup for a firmware bug... */
		break;
	case POWER_SUPPLY_PROP_MANUFACTURE_MONTH:
		ret = macsmc_battery_get_date(&power->mfg_date[2], &val->intval);
		break;
	case POWER_SUPPLY_PROP_MANUFACTURE_DAY:
		ret = macsmc_battery_get_date(&power->mfg_date[4], &val->intval);
		break;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD:
		ret = apple_smc_read_flag(power->smc, SMC_KEY(CHWA));
		val->intval = ret == 1 ? CHWA_FIXED_START_THRESHOLD : 100;
		ret = ret < 0 ? ret : 0;
		break;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD:
		ret = apple_smc_read_flag(power->smc, SMC_KEY(CHWA));
		val->intval = ret == 1 ? CHWA_FIXED_END_THRESHOLD : 100;
		ret = ret < 0 ? ret : 0;
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

static int macsmc_battery_set_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       const union power_supply_propval *val)
{
	struct macsmc_power *power = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR:
		return macsmc_battery_set_charge_behaviour(power, val->intval);
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD:
		/*
		 * Ignore, we allow writes so userspace isn't confused but this is
		 * not configurable independently, it always is 75 or 100 depending
		 * on the end_threshold boolean setting.
		 */
		return 0;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD:
		return apple_smc_write_flag(power->smc, SMC_KEY(CHWA),
					    val->intval <= CHWA_PROP_WRITE_THRESHOLD);
	default:
		return -EINVAL;
	}
}

static int macsmc_battery_property_is_writeable(struct power_supply *psy,
						enum power_supply_property psp)
{
	struct macsmc_power *power = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR:
		return true;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD:
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD:
		return power->has_chwa;
	default:
		return false;
	}
}

static const enum power_supply_property macsmc_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
	POWER_SUPPLY_PROP_TIME_TO_FULL_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_POWER_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_SCOPE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_SERIAL_NUMBER,
	POWER_SUPPLY_PROP_MANUFACTURE_YEAR,
	POWER_SUPPLY_PROP_MANUFACTURE_MONTH,
	POWER_SUPPLY_PROP_MANUFACTURE_DAY,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD
};

static const struct power_supply_desc macsmc_battery_desc = {
	.name			= "macsmc-battery",
	.type			= POWER_SUPPLY_TYPE_BATTERY,
	.get_property		= macsmc_battery_get_property,
	.set_property		= macsmc_battery_set_property,
	.property_is_writeable	= macsmc_battery_property_is_writeable,
	.properties		= macsmc_battery_props,
	.num_properties		= ARRAY_SIZE(macsmc_battery_props),
};

static int macsmc_ac_get_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
	struct macsmc_power *power = power_supply_get_drvdata(psy);
	int ret = 0;
	u16 vu16;
	u32 vu32;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		ret = apple_smc_read_u32(power->smc, SMC_KEY(CHIS), &vu32);
		val->intval = !!vu32;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = apple_smc_read_u16(power->smc, SMC_KEY(AC-n), &vu16);
		val->intval = vu16 * 1000;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = apple_smc_read_u16(power->smc, SMC_KEY(AC-i), &vu16);
		val->intval = vu16 * 1000;
		break;
	case POWER_SUPPLY_PROP_INPUT_POWER_LIMIT:
		ret = apple_smc_read_u32(power->smc, SMC_KEY(ACPW), &vu32);
		val->intval = vu32 * 1000;
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

static enum power_supply_property macsmc_ac_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_INPUT_POWER_LIMIT,
};

static const struct power_supply_desc macsmc_ac_desc = {
	.name			= "macsmc-ac",
	.type			= POWER_SUPPLY_TYPE_MAINS,
	.get_property		= macsmc_ac_get_property,
	.properties		= macsmc_ac_props,
	.num_properties		= ARRAY_SIZE(macsmc_ac_props),
};

static int macsmc_log_power_set(const char *val, const struct kernel_param *kp)
{
	int ret = param_set_bool(val, kp);

	if (ret < 0)
		return ret;

	if (log_power && g_power)
		schedule_delayed_work(&g_power->dbg_log_work, 0);

	return 0;
}

static void macsmc_dbg_work(struct work_struct *wrk)
{
	struct macsmc_power *power = container_of(to_delayed_work(wrk),
						  struct macsmc_power, dbg_log_work);

	macsmc_do_dbg(power);

	if (log_power)
		schedule_delayed_work(&power->dbg_log_work, POWER_LOG_INTERVAL);
}

static void macsmc_power_critical_work(struct work_struct *wrk)
{
	struct macsmc_power *power = container_of(wrk, struct macsmc_power, critical_work);
	int ret;
	u32 bcf0;
	u16 bitv, b0av;

	/*
	 * Check if the battery voltage is below the design voltage. If it is,
	 * we have a few seconds until the machine dies. Explicitly shut down,
	 * which at least gets the NVMe controller to flush its cache.
	 */
	if (apple_smc_read_u16(power->smc, SMC_KEY(BITV), &bitv) >= 0 &&
	    apple_smc_read_u16(power->smc, SMC_KEY(B0AV), &b0av) >= 0 &&
	    b0av < bitv) {
		dev_crit(power->dev, "Emergency notification: Battery is critical\n");
		if (kernel_can_power_off())
			kernel_power_off();
		else /* Missing macsmc-reboot driver? In this state, this will not boot anyway. */
			kernel_restart("Battery is critical");
	}

	/* This spams once per second, so make sure we only trigger shutdown once. */
	if (power->shutdown_started)
		return;

	/* Check for battery empty condition */
	ret = apple_smc_read_u32(power->smc, SMC_KEY(BCF0), &bcf0);
	if (ret < 0) {
		dev_err(power->dev,
				"Emergency notification: Failed to read battery status\n");
	} else if (bcf0 == 0) {
		dev_warn(power->dev, "Emergency notification: Battery status is OK?\n");
		return;
	} else {
		dev_warn(power->dev, "Emergency notification: Battery is empty\n");
	}

	power->shutdown_started = true;

	/*
	 * Attempt to trigger an orderly shutdown. At this point, we should have a few
	 * minutes of reserve capacity left, enough to do a clean shutdown.
	 */
	dev_warn(power->dev, "Shutting down in 10 seconds\n");
	ssleep(10);

	/*
	 * Don't force it; if this stalls or fails, the last-resort check above will
	 * trigger a hard shutdown when shutdown is truly imminent.
	 */
	orderly_poweroff(false);
}

static int macsmc_power_event(struct notifier_block *nb, unsigned long event, void *data)
{
	struct macsmc_power *power = container_of(nb, struct macsmc_power, nb);

	if ((event & 0xffffff00) == 0x71010100) {
		bool charging = (event & 0xff) != 0;

		dev_info(power->dev, "Charging: %d\n", charging);
		power_supply_changed(power->batt);
		power_supply_changed(power->ac);

		return NOTIFY_OK;
	} else if (event == 0x71020000) {
		schedule_work(&power->critical_work);

		return NOTIFY_OK;
	} else if ((event & 0xffff0000) == 0x71060000) {
		u8 changed_port = event >> 8;
		u8 cur_port;

		/* Port charging state change? */
		if (apple_smc_read_u8(power->smc, SMC_KEY(AC-W), &cur_port) >= 0) {
			dev_info(power->dev, "Port %d state change (charge port: %d)\n",
				 changed_port + 1, cur_port);
		}

		power_supply_changed(power->batt);
		power_supply_changed(power->ac);

		return NOTIFY_OK;
	} else if ((event & 0xff000000) == 0x71000000) {
		dev_info(power->dev, "Unknown charger event 0x%lx\n", event);

		return NOTIFY_OK;
	} else if ((event & 0xffff0000) == 0x72010000) {
		/* Button event handled by macsmc-hid, but let's do a debug print */
		if (log_power)
			macsmc_do_dbg(power);

		return NOTIFY_OK;
	}

	return NOTIFY_DONE;
}

static int macsmc_power_probe(struct platform_device *pdev)
{
	struct apple_smc *smc = dev_get_drvdata(pdev->dev.parent);
	struct power_supply_config psy_cfg = {};
	struct macsmc_power *power;
	u32 val;
	int ret;

	power = devm_kzalloc(&pdev->dev, sizeof(*power), GFP_KERNEL);
	if (!power)
		return -ENOMEM;

	power->dev = &pdev->dev;
	power->smc = smc;
	power->batt_desc = macsmc_battery_desc;
	dev_set_drvdata(&pdev->dev, power);

	/* Ignore devices without a charger/battery */
	if (macsmc_battery_get_status(power) <= POWER_SUPPLY_STATUS_UNKNOWN)
		return -ENODEV;

	/* Fetch string properties */
	apple_smc_read(smc, SMC_KEY(BMDN), power->model_name, sizeof(power->model_name) - 1);
	apple_smc_read(smc, SMC_KEY(BMSN), power->serial_number, sizeof(power->serial_number) - 1);
	apple_smc_read(smc, SMC_KEY(BMDT), power->mfg_date, sizeof(power->mfg_date) - 1);

	/* Turn off the "optimized battery charging" flags, in case macOS left them on */
	apple_smc_write_u8(power->smc, SMC_KEY(CH0K), 0);
	apple_smc_write_u8(power->smc, SMC_KEY(CH0B), 0);

	if (apple_smc_read_flag(power->smc, SMC_KEY(CHWA)) >= 0) {
		power->has_chwa = true;
	} else {
		/* Remove the last 2 properties that control the charge threshold */
		power->batt_desc.num_properties -= 2;
	}

	/* Doing one read of this flag enables critical shutdown notifications */
	apple_smc_read_u32(power->smc, SMC_KEY(BCF0), &val);

	psy_cfg.drv_data = power;
	power->batt = devm_power_supply_register(&pdev->dev, &power->batt_desc, &psy_cfg);
	if (IS_ERR(power->batt)) {
		dev_err(&pdev->dev, "Failed to register battery\n");
		ret = PTR_ERR(power->batt);
		return ret;
	}

	power->ac = devm_power_supply_register(&pdev->dev, &macsmc_ac_desc, &psy_cfg);
	if (IS_ERR(power->ac)) {
		dev_err(&pdev->dev, "Failed to register AC adapter\n");
		ret = PTR_ERR(power->ac);
		return ret;
	}

	power->nb.notifier_call = macsmc_power_event;
	apple_smc_register_notifier(power->smc, &power->nb);

	INIT_WORK(&power->critical_work, macsmc_power_critical_work);
	INIT_DELAYED_WORK(&power->dbg_log_work, macsmc_dbg_work);

	g_power = power;

	if (log_power)
		schedule_delayed_work(&power->dbg_log_work, 0);

	return 0;
}

static int macsmc_power_remove(struct platform_device *pdev)
{
	struct macsmc_power *power = dev_get_drvdata(&pdev->dev);

	cancel_work(&power->critical_work);
	cancel_delayed_work(&power->dbg_log_work);

	g_power = NULL;

	apple_smc_unregister_notifier(power->smc, &power->nb);

	return 0;
}

static struct platform_driver macsmc_power_driver = {
	.driver = {
		.name = "macsmc-power",
		.owner = THIS_MODULE,
	},
	.probe = macsmc_power_probe,
	.remove = macsmc_power_remove,
};
module_platform_driver(macsmc_power_driver);

MODULE_LICENSE("Dual MIT/GPL");
MODULE_DESCRIPTION("Apple SMC battery and power management driver");
MODULE_AUTHOR("Hector Martin <marcan@marcan.st>");
MODULE_ALIAS("platform:macsmc-power");
