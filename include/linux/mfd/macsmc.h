// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Apple SMC core definitions
 * Copyright (C) The Asahi Linux Contributors
 */

#ifndef _LINUX_MFD_MACSMC_H
#define _LINUX_MFD_MACSMC_H

struct apple_smc;

typedef u32 smc_key;

#define SMC_KEY(s) (smc_key)(_SMC_KEY(#s))
#define _SMC_KEY(s) (((s)[0] << 24) | ((s)[1] << 16) | ((s)[2] << 8) | (s)[3])

#define APPLE_SMC_READABLE BIT(7)
#define APPLE_SMC_WRITABLE BIT(6)
#define APPLE_SMC_FUNCTION BIT(4)

struct apple_smc_key_info {
	u8 size;
	u32 type_code;
	u8 flags;
};

int apple_smc_read(struct apple_smc *smc, smc_key key, void *buf, size_t size);
int apple_smc_write(struct apple_smc *smc, smc_key key, void *buf, size_t size);
int apple_smc_write_atomic(struct apple_smc *smc, smc_key key, void *buf, size_t size);
int apple_smc_rw(struct apple_smc *smc, smc_key key, void *wbuf, size_t wsize,
		 void *rbuf, size_t rsize);

int apple_smc_get_key_count(struct apple_smc *smc);
int apple_smc_find_first_key_index(struct apple_smc *smc, smc_key key);
int apple_smc_get_key_by_index(struct apple_smc *smc, int index, smc_key *key);
int apple_smc_get_key_info(struct apple_smc *smc, smc_key key, struct apple_smc_key_info *info);

static inline bool apple_smc_key_exists(struct apple_smc *smc, smc_key key)
{
	return apple_smc_get_key_info(smc, key, NULL) >= 0;
}

#define APPLE_SMC_TYPE_OPS(type) \
	static inline int apple_smc_read_##type(struct apple_smc *smc, smc_key key, type *p) \
	{ \
		int ret = apple_smc_read(smc, key, p, sizeof(*p)); \
		return (ret < 0) ? ret : ((ret != sizeof(*p)) ? -EINVAL : 0); \
	} \
	static inline int apple_smc_write_##type(struct apple_smc *smc, smc_key key, type p) \
	{ \
		return apple_smc_write(smc, key, &p, sizeof(p)); \
	} \
	static inline int apple_smc_write_##type##_atomic(struct apple_smc *smc, smc_key key, type p) \
	{ \
		return apple_smc_write_atomic(smc, key, &p, sizeof(p)); \
	} \
	static inline int apple_smc_rw_##type(struct apple_smc *smc, smc_key key, \
					      type w, type *r) \
	{ \
		int ret = apple_smc_rw(smc, key, &w, sizeof(w), r, sizeof(*r)); \
		return (ret < 0) ? ret : ((ret != sizeof(*r)) ? -EINVAL : 0); \
	}

APPLE_SMC_TYPE_OPS(u64)
APPLE_SMC_TYPE_OPS(u32)
APPLE_SMC_TYPE_OPS(u16)
APPLE_SMC_TYPE_OPS(u8)
APPLE_SMC_TYPE_OPS(s64)
APPLE_SMC_TYPE_OPS(s32)
APPLE_SMC_TYPE_OPS(s16)
APPLE_SMC_TYPE_OPS(s8)

static inline int apple_smc_read_flag(struct apple_smc *smc, smc_key key)
{
	u8 val;
	int ret = apple_smc_read_u8(smc, key, &val);
	if (ret < 0)
		return ret;
	return val ? 1 : 0;
}
#define apple_smc_write_flag apple_smc_write_u8

int apple_smc_read_f32_scaled(struct apple_smc *smc, smc_key key, int *p, int scale);

int apple_smc_register_notifier(struct apple_smc *smc, struct notifier_block *n);
int apple_smc_unregister_notifier(struct apple_smc *smc, struct notifier_block *n);

#endif
