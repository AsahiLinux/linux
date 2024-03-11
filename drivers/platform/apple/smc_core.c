// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Apple SMC core framework
 * Copyright The Asahi Linux Contributors
 */

#include <linux/bitfield.h>
#include <linux/device.h>
#include <linux/mfd/core.h>
#include <linux/mutex.h>
#include <linux/notifier.h>
#include "smc.h"

struct apple_smc {
	struct device *dev;

	void *be_cookie;
	const struct apple_smc_backend_ops *be;

	struct mutex mutex;

	u32 key_count;
	smc_key first_key;
	smc_key last_key;

	struct blocking_notifier_head event_handlers;
};

static const struct mfd_cell apple_smc_devs[] = {
	{
		.name = "macsmc-gpio",
	},
	{
		.name = "macsmc-hid",
	},
	{
		.name = "macsmc-power",
	},
	{
		.name = "macsmc-reboot",
	},
	{
		.name = "macsmc-rtc",
	},
};

int apple_smc_read(struct apple_smc *smc, smc_key key, void *buf, size_t size)
{
	int ret;

	mutex_lock(&smc->mutex);
	ret = smc->be->read_key(smc->be_cookie, key, buf, size);
	mutex_unlock(&smc->mutex);

	return ret;
}
EXPORT_SYMBOL(apple_smc_read);

int apple_smc_write(struct apple_smc *smc, smc_key key, void *buf, size_t size)
{
	int ret;

	mutex_lock(&smc->mutex);
	ret = smc->be->write_key(smc->be_cookie, key, buf, size);
	mutex_unlock(&smc->mutex);

	return ret;
}
EXPORT_SYMBOL(apple_smc_write);

int apple_smc_write_atomic(struct apple_smc *smc, smc_key key, void *buf, size_t size)
{
	int ret;

	/*
	 * Will fail if SMC is busy. This is only used by SMC reboot/poweroff
	 * final calls, so it doesn't really matter at that point.
	 */
	if (!mutex_trylock(&smc->mutex))
		return -EBUSY;

	ret = smc->be->write_key_atomic(smc->be_cookie, key, buf, size);
	mutex_unlock(&smc->mutex);

	return ret;
}
EXPORT_SYMBOL(apple_smc_write_atomic);

int apple_smc_rw(struct apple_smc *smc, smc_key key, void *wbuf, size_t wsize,
		 void *rbuf, size_t rsize)
{
	int ret;

	mutex_lock(&smc->mutex);
	ret = smc->be->rw_key(smc->be_cookie, key, wbuf, wsize, rbuf, rsize);
	mutex_unlock(&smc->mutex);

	return ret;
}
EXPORT_SYMBOL(apple_smc_rw);

int apple_smc_read_f32_scaled(struct apple_smc *smc, smc_key key, int *p, int scale)
{
	u32 fval;
	u64 val;
	int ret, exp;

	ret = apple_smc_read_u32(smc, key, &fval);
	if (ret < 0)
		return ret;

	val = ((u64)((fval & GENMASK(22, 0)) | BIT(23)));
	exp = ((fval >> 23) & 0xff) - 127 - 23;
	if (scale < 0) {
		val <<= 32;
		exp -= 32;
		val /= -scale;
	} else {
		val *= scale;
	}

	if (exp > 63)
		val = U64_MAX;
	else if (exp < -63)
		val = 0;
	else if (exp < 0)
		val >>= -exp;
	else if (exp != 0 && (val & ~((1UL << (64 - exp)) - 1))) /* overflow */
		val = U64_MAX;
	else
		val <<= exp;

	if (fval & BIT(31)) {
		if (val > (-(s64)INT_MIN))
			*p = INT_MIN;
		else
			*p = -val;
	} else {
		if (val > INT_MAX)
			*p = INT_MAX;
		else
			*p = val;
	}

	return ret;
}
EXPORT_SYMBOL(apple_smc_read_f32_scaled);

int apple_smc_get_key_by_index(struct apple_smc *smc, int index, smc_key *key)
{
	int ret;

	mutex_lock(&smc->mutex);
	ret = smc->be->get_key_by_index(smc->be_cookie, index, key);
	mutex_unlock(&smc->mutex);

	return ret;
}
EXPORT_SYMBOL(apple_smc_get_key_by_index);

int apple_smc_get_key_info(struct apple_smc *smc, smc_key key, struct apple_smc_key_info *info)
{
	int ret;

	mutex_lock(&smc->mutex);
	ret = smc->be->get_key_info(smc->be_cookie, key, info);
	mutex_unlock(&smc->mutex);

	return ret;
}
EXPORT_SYMBOL(apple_smc_get_key_info);

int apple_smc_find_first_key_index(struct apple_smc *smc, smc_key key)
{
	int start = 0, count = smc->key_count;
	int ret;

	if (key <= smc->first_key)
		return 0;
	if (key > smc->last_key)
		return smc->key_count;

	while (count > 1) {
		int pivot = start + ((count - 1) >> 1);
		smc_key pkey;

		ret = apple_smc_get_key_by_index(smc, pivot, &pkey);
		if (ret < 0)
			return ret;

		if (pkey == key)
			return pivot;

		pivot++;

		if (pkey < key) {
			count -= pivot - start;
			start = pivot;
		} else {
			count = pivot - start;
		}
	}

	return start;
}
EXPORT_SYMBOL(apple_smc_find_first_key_index);

int apple_smc_get_key_count(struct apple_smc *smc)
{
	return smc->key_count;
}
EXPORT_SYMBOL(apple_smc_get_key_count);

void apple_smc_event_received(struct apple_smc *smc, uint32_t event)
{
	dev_dbg(smc->dev, "Event: 0x%08x\n", event);
	blocking_notifier_call_chain(&smc->event_handlers, event, NULL);
}
EXPORT_SYMBOL(apple_smc_event_received);

int apple_smc_register_notifier(struct apple_smc *smc, struct notifier_block *n)
{
	return blocking_notifier_chain_register(&smc->event_handlers, n);
}
EXPORT_SYMBOL(apple_smc_register_notifier);

int apple_smc_unregister_notifier(struct apple_smc *smc, struct notifier_block *n)
{
	return blocking_notifier_chain_unregister(&smc->event_handlers, n);
}
EXPORT_SYMBOL(apple_smc_unregister_notifier);

void *apple_smc_get_cookie(struct apple_smc *smc)
{
	return smc->be_cookie;
}
EXPORT_SYMBOL(apple_smc_get_cookie);

int apple_smc_probe(struct device *dev, const struct apple_smc_backend_ops *ops, void *cookie)
{
	struct apple_smc *smc;
	u32 count;
	int ret;

	smc = devm_kzalloc(dev, sizeof(*smc), GFP_KERNEL);
	if (!smc)
		return -ENOMEM;

	smc->dev = dev;
	smc->be_cookie = cookie;
	smc->be = ops;
	mutex_init(&smc->mutex);
	BLOCKING_INIT_NOTIFIER_HEAD(&smc->event_handlers);

	ret = apple_smc_read_u32(smc, SMC_KEY(#KEY), &count);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to get key count");
	smc->key_count = be32_to_cpu(count);

	ret = apple_smc_get_key_by_index(smc, 0, &smc->first_key);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to get first key");

	ret = apple_smc_get_key_by_index(smc, smc->key_count - 1, &smc->last_key);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to get last key");

	dev_set_drvdata(dev, smc);

	/* Enable notifications */
	apple_smc_write_flag(smc, SMC_KEY(NTAP), 1);

	dev_info(dev, "Initialized (%d keys %p4ch..%p4ch)\n",
		 smc->key_count, &smc->first_key, &smc->last_key);

	ret = mfd_add_devices(dev, -1, apple_smc_devs, ARRAY_SIZE(apple_smc_devs), NULL, 0, NULL);
	if (ret)
		return dev_err_probe(dev, ret, "Subdevice initialization failed");

	return 0;
}
EXPORT_SYMBOL(apple_smc_probe);

int apple_smc_remove(struct apple_smc *smc)
{
	mfd_remove_devices(smc->dev);

	/* Disable notifications */
	apple_smc_write_flag(smc, SMC_KEY(NTAP), 1);

	return 0;
}
EXPORT_SYMBOL(apple_smc_remove);

MODULE_AUTHOR("Hector Martin <marcan@marcan.st>");
MODULE_LICENSE("Dual MIT/GPL");
MODULE_DESCRIPTION("Apple SMC core");
