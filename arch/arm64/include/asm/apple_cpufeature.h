// SPDX-License-Identifier: GPL-2.0

#ifndef __ASM_APPLE_CPUFEATURES_H
#define __ASM_APPLE_CPUFEATURES_H

#include <linux/bits.h>
#include <asm/sysreg.h>

#define AIDR_APPLE_TSO_SHIFT	9
#define AIDR_APPLE_TSO		BIT(9)

#define ACTLR_APPLE_TSO_SHIFT	1
#define ACTLR_APPLE_TSO		BIT(1)

#endif
