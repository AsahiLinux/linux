/* SPDX-License-Identifier: GPL-2.0-only OR MIT */

#ifndef SPI_HID_APPLE_H
#define SPI_HID_APPLE_H

#include <linux/interrupt.h>
#include <linux/spi/spi.h>

/**
 * struct spihid_apple_ops - Ops to control the device from the core driver.
 *
 * @power_on: reset and power the device on.
 * @power_off: power the device off.
 * @enable_irq: enable irq or ACPI gpe.
 * @disable_irq: disable irq or ACPI gpe.
 */

struct spihid_apple_ops {
    int (*power_on)(struct spihid_apple_ops *ops);
    int (*power_off)(struct spihid_apple_ops *ops);
    int (*enable_irq)(struct spihid_apple_ops *ops);
    int (*disable_irq)(struct spihid_apple_ops *ops);
    int (*enable_irq_wake)(struct spihid_apple_ops *ops);
    int (*disable_irq_wake)(struct spihid_apple_ops *ops);
};

irqreturn_t spihid_apple_core_irq(int irq, void *data);

int spihid_apple_core_probe(struct spi_device *spi, struct spihid_apple_ops *ops);
void spihid_apple_core_remove(struct spi_device *spi);
void spihid_apple_core_shutdown(struct spi_device *spi);

extern const struct dev_pm_ops spihid_apple_core_pm;

#endif /* SPI_HID_APPLE_H */
