// SPDX-License-Identifier: GPL-2.0

#include <linux/timekeeping.h>

ktime_t rust_helper_ktime_get_real(void) {
	return ktime_get_real();
}

ktime_t rust_helper_ktime_get_boottime(void) {
	return ktime_get_boottime();
}

ktime_t rust_helper_ktime_get_clocktai(void) {
	return ktime_get_clocktai();
}
