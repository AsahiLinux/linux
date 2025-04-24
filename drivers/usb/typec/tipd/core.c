// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for TI TPS6598x USB Power Delivery controller family
 * Enhanced for Asahi Linux and Apple Silicon (CD321x) support
 *
 * Copyright (C) 2017, Intel Corporation
 * Author: Heikki Krogerus <heikki.krogerus@linux.intel.com>
 * Enhancements: Asahi Linux contributors
 */

#include <linux/i2c.h>
#include <linux/acpi.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/interrupt.h>
#include <linux/usb/typec.h>
#include <linux/usb/role.h>
#include <linux/workqueue.h>
#include <linux/firmware.h>
#include <linux/delay.h>

/* Assumed definitions for tps6598x.h */
#define TPS_POWER_STATUS_PWROPMODE(status)  ((status) & 0x07)
#define TPS_POWER_STATUS_CONNECTION(status) ((status) & BIT(0))
#define TPS_POWER_STATUS_SOURCESINK(status) ((status) & BIT(1))
#define TPS_STATUS_TO_TYPEC_PORTROLE(status) (((status) & BIT(2)) ? TYPEC_SOURCE : TYPEC_SINK)
#define TPS_STATUS_TO_TYPEC_DATAROLE(status) (((status) & BIT(3)) ? TYPEC_HOST : TYPEC_DEVICE)
#define TPS_STATUS_TO_TYPEC_VCONN(status)   (((status) & BIT(4)) ? TYPEC_SOURCE : TYPEC_SINK)
#define TPS_STATUS_TO_UPSIDE_DOWN(status)   ((status) & BIT(5))
#define TPS_STATUS_PLUG_PRESENT             BIT(6)
#define TPS_BOOT_STATUS_I2C_EEPROM_PRESENT  BIT(0)
#define TPS_BOOT_STATUS_DEAD_BATTERY_FLAG   BIT(1)
#define TPS_SYSTEM_POWER_STATE_S0           0x00
#define TPS_SLEEP_CONF_SLEEP_MODE_ALLOWED   BIT(0)
#define TPS_PD_STATUS_PORT_TYPE(status)     ((status) & 0x03)
#define TPS_PTCS_CONTENT_DEV                BIT(0)
#define TPS_PTCS_CONTENT_APP                BIT(1)
#define TPS_PTCS_OUT_BYTES                  8
#define TPS_PTCD_OUT_BYTES                  8
#define TPS_PTCC_OUT_BYTES                  8
#define TPS_PTCS_STATUS_FAIL                0xFF
#define TPS_PTCD_TRANSFER_STATUS            0
#define TPS_PTCD_LOADING_STATE              1
#define TPS_PTCD_LOAD_ERR                  0xFF
#define TPS_PTCC_DEV                        0
#define TPS_PTCC_APP                        1
#define TPS_65981_2_6_INTEVENT_LEN          8
#define TPS_65987_8_INTEVENT_LEN            11
#define TPS_VERSION_HW_VERSION(version)     ((version) >> 16)
#define TPS_VERSION_HW_65987_8_DH           0xDH
#define TPS_VERSION_HW_65987_8_DK           0xDK
#define APPLE_CD_REG_INT_POWER_STATUS_UPDATE BIT(0)
#define APPLE_CD_REG_INT_DATA_STATUS_UPDATE  BIT(1)
#define APPLE_CD_REG_INT_PLUG_EVENT          BIT(2)

/* Register offsets */
#define TPS_REG_VID                 0x00
#define TPS_REG_MODE                0x03
#define TPS_REG_CMD1                0x08
#define TPS_REG_DATA1               0x09
#define TPS_REG_VERSION             0x0F
#define TPS_REG_INT_EVENT1          0x14
#define TPS_REG_INT_EVENT2          0x15
#define TPS_REG_INT_MASK1           0x16
#define TPS_REG_INT_MASK2           0x17
#define TPS_REG_INT_CLEAR1          0x18
#define TPS_REG_INT_CLEAR2          0x19
#define TPS_REG_SYSTEM_POWER_STATE  0x20
#define TPS_REG_STATUS              0x1A
#define TPS_REG_SYSTEM_CONF         0x28
#define TPS_REG_CTRL_CONF           0x29
#define TPS_REG_BOOT_STATUS         0x2D
#define TPS_REG_POWER_STATUS        0x3F
#define TPS_REG_PD_STATUS           0x40
#define TPS_REG_RX_IDENTITY_SOP     0x48
#define TPS_REG_DATA_STATUS         0x5F
#define TPS_REG_SLEEP_CONF          0x70

/* TPS_REG_SYSTEM_CONF bits */
#define TPS_SYSCONF_PORTINFO(c)     ((c) & 7)

/* BPMs task timeout, recommended 5 seconds */
#define TPS_BUNDLE_TIMEOUT          0x32

/* BPMs return code */
#define TPS_TASK_BPMS_INVALID_BUNDLE_SIZE   0x4
#define TPS_TASK_BPMS_INVALID_SLAVE_ADDR    0x5
#define TPS_TASK_BPMS_INVALID_TIMEOUT       0x6

/* PBMc data out */
#define TPS_PBMC_RC                 0
#define TPS_PBMC_DPCS               2

/* Reset de-assertion to ready for operation */
#define TPS_SETUP_MS                1000

/* Polling interval for non-interrupt mode */
#define POLL_INTERVAL               500 /* msecs */

/* Max data bytes for Data1, Data2, etc. */
#define TPS_MAX_LEN                 64

enum {
    TPS_PORTINFO_SINK,
    TPS_PORTINFO_SINK_ACCESSORY,
    TPS_PORTINFO_DRP_UFP,
    TPS_PORTINFO_DRP_UFP_DRD,
    TPS_PORTINFO_DRP_DFP,
    TPS_PORTINFO_DRP_DFP_DRD,
    TPS_PORTINFO_SOURCE,
};

struct tps6598x_rx_identity_reg {
    u8 status;
    struct usb_pd_identity identity;
} __packed;

enum {
    TPS_MODE_APP,
    TPS_MODE_BOOT,
    TPS_MODE_BIST,
    TPS_MODE_DISC,
    TPS_MODE_PTCH,
};

static const char *const modes[] = {
    [TPS_MODE_APP]  = "APP ",
    [TPS_MODE_BOOT] = "BOOT",
    [TPS_MODE_BIST] = "BIST",
    [TPS_MODE_DISC] = "DISC",
    [TPS_MODE_PTCH] = "PTCH",
};

/* Unrecognized commands */
#define INVALID_CMD(_cmd_)  (_cmd_ == 0x444d4321)

/* Standard Task return codes */
#define TPS_TASK_TIMEOUT    1
#define TPS_TASK_REJECTED   3

struct tps6598x;

struct tipd_data {
    irq_handler_t irq_handler;
    int (*register_port)(struct tps6598x *tps, struct fwnode_handle *node);
    void (*trace_power_status)(u16 status);
    void (*trace_status)(u32 status);
    int (*apply_patch)(struct tps6598x *tps);
    int (*init)(struct tps6598x *tps);
    int (*reset)(struct tps6598x *tps);
};

struct tps6598x {
    struct device *dev;
    struct regmap *regmap;
    struct mutex lock; /* Device lock */
    u8 i2c_protocol:1;

    struct gpio_desc *reset;
    struct typec_port *port;
    struct typec_partner *partner;
    struct usb_pd_identity partner_identity;
    struct usb_role_switch *role_sw;
    struct typec_capability typec_cap;

    struct power_supply *psy;
    struct power_supply_desc psy_desc;
    enum power_supply_usb_type usb_type;

    int wakeup;
    u32 status; /* Status register */
    u16 pwr_status;
    struct delayed_work wq_poll;

    const struct tipd_data *data;
};

static enum power_supply_property tps6598x_psy_props[] = {
    POWER_SUPPLY_PROP_USB_TYPE,
    POWER_SUPPLY_PROP_ONLINE,
};

static const char *tps6598x_psy_name_prefix = "tps6598x-source-psy-";

static int tps6598x_block_read(struct tps6598x *tps, u8 reg, void *val, size_t len)
{
    u8 data[TPS_MAX_LEN + 1];
    int ret;

    if (!tps || !val || len + 1 > sizeof(data)) {
        dev_err(tps ? tps->dev : NULL, "Invalid block read: reg=0x%02x, len=%zu\n", reg, len);
        return -EINVAL;
    }

    if (!tps->i2c_protocol)
        return regmap_raw_read(tps->regmap, reg, val, len);

    ret = regmap_raw_read(tps->regmap, reg, data, len + 1);
    if (ret) {
        dev_err(tps->dev, "Block read failed: reg=0x%02x, ret=%d\n", reg, ret);
        return ret;
    }

    if (data[0] < len) {
        dev_err(tps->dev, "Block read short: expected %zu, got %d\n", len, data[0]);
        return -EIO;
    }

    memcpy(val, &data[1], len);
    return 0;
}

static int tps6598x_block_write(struct tps6598x *tps, u8 reg, const void *val, size_t len)
{
    u8 data[TPS_MAX_LEN + 1];

    if (!tps || !val || len + 1 > sizeof(data)) {
        dev_err(tps ? tps->dev : NULL, "Invalid block write: reg=0x%02x, len=%zu\n", reg, len);
        return -EINVAL;
    }

    if (!tps->i2c_protocol)
        return regmap_raw_write(tps->regmap, reg, val, len);

    data[0] = len;
    memcpy(&data[1], val, len);

    return regmap_raw_write(tps->regmap, reg, data, len + 1);
}

static inline int tps6598x_read8(struct tps6598x *tps, u8 reg, u8 *val)
{
    return tps6598x_block_read(tps, reg, val, sizeof(u8));
}

static inline int tps6598x_read16(struct tps6598x *tps, u8 reg, u16 *val)
{
    return tps6598x_block_read(tps, reg, val, sizeof(u16));
}

static inline int tps6598x_read32(struct tps6598x *tps, u8 reg, u32 *val)
{
    return tps6598x_block_read(tps, reg, val, sizeof(u32));
}

static inline int tps6598x_read64(struct tps6598x *tps, u8 reg, u64 *val)
{
    return tps6598x_block_read(tps, reg, val, sizeof(u64));
}

static inline int tps6598x_write8(struct tps6598x *tps, u8 reg, u8 val)
{
    return tps6598x_block_write(tps, reg, &val, sizeof(u8));
}

static inline int tps6598x_write64(struct tps6598x *tps, u8 reg, u64 val)
{
    return tps6598x_block_write(tps, reg, &val, sizeof(u64));
}

static inline int tps6598x_write_4cc(struct tps6598x *tps, u8 reg, const char *val)
{
    return tps6598x_block_write(tps, reg, val, 4);
}

static int tps6598x_read_partner_identity(struct tps6598x *tps)
{
    struct tps6598x_rx_identity_reg id;
    int ret;

    ret = tps6598x_block_read(tps, TPS_REG_RX_IDENTITY_SOP, &id, sizeof(id));
    if (ret) {
        dev_err(tps->dev, "Failed to read partner identity: %d\n", ret);
        return ret;
    }

    tps->partner_identity = id.identity;
    return 0;
}

static void tps6598x_set_data_role(struct tps6598x *tps, enum typec_data_role role, bool connected)
{
    enum usb_role role_val;

    if (!tps || !tps->port || !tps->role_sw) {
        dev_err(tps ? tps->dev : NULL, "Invalid data role set: tps=%p, port=%p, role_sw=%p\n",
                tps, tps ? tps->port : NULL, tps ? tps->role_sw : NULL);
        return;
    }

    role_val = (role == TYPEC_HOST) ? USB_ROLE_HOST : USB_ROLE_DEVICE;
    if (!connected)
        role_val = USB_ROLE_NONE;

    usb_role_switch_set_role(tps->role_sw, role_val);
    typec_set_data_role(tps->port, role);
}

static int tps6598x_connect(struct tps6598x *tps, u32 status)
{
    struct typec_partner_desc desc = {};
    enum typec_pwr_opmode mode;
    int ret;

    if (!tps || !tps->port) {
        dev_err(tps ? tps->dev : NULL, "Invalid connect: tps=%p, port=%p\n", tps, tps ? tps->port : NULL);
        return -EINVAL;
    }

    if (tps->partner)
        return 0;

    mode = TPS_POWER_STATUS_PWROPMODE(tps->pwr_status);
    desc.usb_pd = (mode == TYPEC_PWR_MODE_PD);
    desc.accessory = TYPEC_ACCESSORY_NONE;

    if (desc.usb_pd) {
        ret = tps6598x_read_partner_identity(tps);
        if (ret)
            return ret;
        desc.identity = &tps->partner_identity;
    }

    typec_set_pwr_opmode(tps->port, mode);
    typec_set_pwr_role(tps->port, TPS_STATUS_TO_TYPEC_PORTROLE(status));
    typec_set_vconn_role(tps->port, TPS_STATUS_TO_TYPEC_VCONN(status));
    typec_set_orientation(tps->port, TPS_STATUS_TO_UPSIDE_DOWN(status) ?
                          TYPEC_ORIENTATION_REVERSE : TYPEC_ORIENTATION_NORMAL);
    typec_set_mode(tps->port, TYPEC_STATE_USB);
    tps6598x_set_data_role(tps, TPS_STATUS_TO_TYPEC_DATAROLE(status), true);

    tps->partner = typec_register_partner(tps->port, &desc);
    if (IS_ERR(tps->partner)) {
        ret = PTR_ERR(tps->partner);
        dev_err(tps->dev, "Failed to register partner: %d\n", ret);
        return ret;
    }

    if (desc.identity)
        typec_partner_set_identity(tps->partner);

    power_supply_changed(tps->psy);
    return 0;
}

static void tps6598x_disconnect(struct tps6598x *tps, u32 status)
{
    if (!tps || !tps->port) {
        dev_err(tps ? tps->dev : NULL, "Invalid disconnect: tps=%p, port=%p\n", tps, tps ? tps->port : NULL);
        return;
    }

    if (!IS_ERR(tps->partner))
        typec_unregister_partner(tps->partner);
    tps->partner = NULL;
    typec_set_pwr_opmode(tps->port, TYPEC_PWR_MODE_USB);
    typec_set_pwr_role(tps->port, TPS_STATUS_TO_TYPEC_PORTROLE(status));
    typec_set_vconn_role(tps->port, TPS_STATUS_TO_TYPEC_VCONN(status));
    typec_set_orientation(tps->port, TYPEC_ORIENTATION_NONE);
    typec_set_mode(tps->port, TYPEC_STATE_SAFE);
    tps6598x_set_data_role(tps, TPS_STATUS_TO_TYPEC_DATAROLE(status), false);

    power_supply_changed(tps->psy);
}

static int tps6598x_exec_cmd_tmo(struct tps6598x *tps, const char *cmd,
                                 size_t in_len, const u8 *in_data,
                                 size_t out_len, u8 *out_data,
                                 u32 cmd_timeout_ms, u32 res_delay_ms)
{
    unsigned long timeout;
    u32 val;
    int ret;

    if (!tps || !cmd) {
        dev_err(tps ? tps->dev : NULL, "Invalid command exec: tps=%p, cmd=%p\n", tps, cmd);
        return -EINVAL;
    }

    ret = tps6598x_read32(tps, TPS_REG_CMD1, &val);
    if (ret) {
        dev_err(tps->dev, "Failed to read CMD1: %d\n", ret);
        return ret;
    }
    if (val && !INVALID_CMD(val)) {
        dev_err(tps->dev, "Command busy: 0x%08x\n", val);
        return -EBUSY;
    }

    if (in_len) {
        ret = tps6598x_block_write(tps, TPS_REG_DATA1, in_data, in_len);
        if (ret) {
            dev_err(tps->dev, "Failed to write DATA1: %d\n", ret);
            return ret;
        }
    }

    ret = tps6598x_write_4cc(tps, TPS_REG_CMD1, cmd);
    if (ret) {
        dev_err(tps->dev, "Failed to write CMD1 (%s): %d\n", cmd, ret);
        return ret;
    }

    timeout = jiffies + msecs_to_jiffies(cmd_timeout_ms);
    do {
        ret = tps6598x_read32(tps, TPS_REG_CMD1, &val);
        if (ret) {
            dev_err(tps->dev, "Failed to poll CMD1: %d\n", ret);
            return ret;
        }
        if (INVALID_CMD(val)) {
            dev_err(tps->dev, "Invalid command: %s\n", cmd);
            return -EINVAL;
        }
        if (time_is_before_jiffies(timeout)) {
            dev_err(tps->dev, "Command timeout: %s\n", cmd);
            return -ETIMEDOUT;
        }
    } while (val);

    msleep(res_delay_ms);

    if (out_len) {
        ret = tps6598x_block_read(tps, TPS_REG_DATA1, out_data, out_len);
        if (ret) {
            dev_err(tps->dev, "Failed to read DATA1 output: %d\n", ret);
            return ret;
        }
        val = out_data[0];
    } else {
        ret = tps6598x_read8(tps, TPS_REG_DATA1, (u8 *)&val);
        if (ret) {
            dev_err(tps->dev, "Failed to read DATA1 status: %d\n", ret);
            return ret;
        }
    }

    switch (val) {
    case TPS_TASK_TIMEOUT:
        dev_err(tps->dev, "Task timeout: %s\n", cmd);
        return -ETIMEDOUT;
    case TPS_TASK_REJECTED:
        dev_err(tps->dev, "Task rejected: %s\n", cmd);
        return -EPERM;
    default:
        break;
    }

    return 0;
}

static int tps6598x_exec_cmd(struct tps6598x *tps, const char *cmd,
                             size_t in_len, const u8 *in_data,
                             size_t out_len, u8 *out_data)
{
    return tps6598x_exec_cmd_tmo(tps, cmd, in_len, in_data, out_len, out_data, 1000, 0);
}

static int tps6598x_dr_set(struct typec_port *port, enum typec_data_role role)
{
    struct tps6598x *tps = typec_get_drvdata(port);
    const char *cmd = (role == TYPEC_DEVICE) ? "SWUF" : "SWDF";
    u32 status;
    int ret;

    if (!tps || !port) {
        dev_err(tps ? tps->dev : NULL, "Invalid dr_set: tps=%p, port=%p\n", tps, port);
        return -EINVAL;
    }

    mutex_lock(&tps->lock);

    ret = tps6598x_exec_cmd(tps, cmd, 0, NULL, 0, NULL);
    if (ret) {
        dev_err(tps->dev, "Failed to set data role (%s): %d\n", cmd, ret);
        goto out_unlock;
    }

    ret = tps6598x_read32(tps, TPS_REG_STATUS, &status);
    if (ret) {
        dev_err(tps->dev, "Failed to read status: %d\n", ret);
        goto out_unlock;
    }

    if (role != TPS_STATUS_TO_TYPEC_DATAROLE(status)) {
        dev_err(tps->dev, "Data role mismatch: expected %d, got %d\n", role,
                TPS_STATUS_TO_TYPEC_DATAROLE(status));
        ret = -EPROTO;
        goto out_unlock;
    }

    tps6598x_set_data_role(tps, role, true);

out_unlock:
    mutex_unlock(&tps->lock);
    return ret;
}

static int tps6598x_pr_set(struct typec_port *port, enum typec_role role)
{
    struct tps6598x *tps = typec_get_drvdata(port);
    const char *cmd = (role == TYPEC_SINK) ? "SWSk" : "SWSr";
    u32 status;
    int ret;

    if (!tps || !port) {
        dev_err(tps ? tps->dev : NULL, "Invalid pr_set: tps=%p, port=%p\n", tps, port);
        return -EINVAL;
    }

    mutex_lock(&tps->lock);

    ret = tps6598x_exec_cmd(tps, cmd, 0, NULL, 0, NULL);
    if (ret) {
        dev_err(tps->dev, "Failed to set power role (%s): %d\n", cmd, ret);
        goto out_unlock;
    }

    ret = tps6598x_read32(tps, TPS_REG_STATUS, &status);
    if (ret) {
        dev_err(tps->dev, "Failed to read status: %d\n", ret);
        goto out_unlock;
    }

    if (role != TPS_STATUS_TO_TYPEC_PORTROLE(status)) {
        dev_err(tps->dev, "Power role mismatch: expected %d, got %d\n", role,
                TPS_STATUS_TO_TYPEC_PORTROLE(status));
        ret = -EPROTO;
        goto out_unlock;
    }

    typec_set_pwr_role(tps->port, role);

out_unlock:
    mutex_unlock(&tps->lock);
    return ret;
}

static const struct typec_operations tps6598x_ops = {
    .dr_set = tps6598x_dr_set,
    .pr_set = tps6598x_pr_set,
};

static bool tps6598x_read_status(struct tps6598x *tps, u32 *status)
{
    int ret;

    if (!tps || !status) {
        dev_err(tps ? tps->dev : NULL, "Invalid read_status: tps=%p, status=%p\n", tps, status);
        return false;
    }

    ret = tps6598x_read32(tps, TPS_REG_STATUS, status);
    if (ret) {
        dev_err(tps->dev, "Failed to read status: %d\n", ret);
        return false;
    }

    if (tps->data->trace_status)
        tps->data->trace_status(*status);

    return true;
}

static bool tps6598x_read_data_status(struct tps6598x *tps)
{
    u32 data_status;
    int ret;

    if (!tps) {
        dev_err(NULL, "Invalid read_data_status: tps=%p\n", tps);
        return false;
    }

    ret = tps6598x_read32(tps, TPS_REG_DATA_STATUS, &data_status);
    if (ret) {
        dev_err(tps->dev, "Failed to read data status: %d\n", ret);
        return false;
    }
    /* Assume trace_tps6598x_data_status exists */
    trace_tps6598x_data_status(data_status);

    return true;
}

static bool tps6598x_read_power_status(struct tps6598x *tps)
{
    u16 pwr_status;
    int ret;

    if (!tps) {
        dev_err(NULL, "Invalid read_power_status: tps=%p\n", tps);
        return false;
    }

    ret = tps6598x_read16(tps, TPS_REG_POWER_STATUS, &pwr_status);
    if (ret) {
        dev_err(tps->dev, "Failed to read power status: %d\n", ret);
        return false;
    }
    tps->pwr_status = pwr_status;

    if (tps->data->trace_power_status)
        tps->data->trace_power_status(pwr_status);

    return true;
}

static void tps6598x_handle_plug_event(struct tps6598x *tps, u32 status)
{
    int ret;

    if (!tps) {
        dev_err(NULL, "Invalid plug event: tps=%p\n", tps);
        return;
    }

    if (status & TPS_STATUS_PLUG_PRESENT) {
        ret = tps6598x_connect(tps, status);
        if (ret)
            dev_err(tps->dev, "Failed to register partner: %d\n", ret);
    } else {
        tps6598x_disconnect(tps, status);
    }
}

static irqreturn_t cd321x_interrupt(int irq, void *data)
{
    struct tps6598x *tps = data;
    u64 event = 0;
    u32 status;
    int ret;

    if (!tps) {
        pr_err("Invalid CD321x interrupt: tps=%p\n", tps);
        return IRQ_NONE;
    }

    mutex_lock(&tps->lock);

    ret = tps6598x_read64(tps, TPS_REG_INT_EVENT1, &event);
    if (ret) {
        dev_err(tps->dev, "Failed to read events: %d\n", ret);
        goto err_unlock;
    }
    trace_cd321x_irq(event);

    if (!event)
        goto err_unlock;

    tps6598x_write64(tps, TPS_REG_INT_CLEAR1, event);

    if (!tps6598x_read_status(tps, &status))
        goto err_unlock;

    if (event & APPLE_CD_REG_INT_POWER_STATUS_UPDATE)
        if (!tps6598x_read_power_status(tps))
            goto err_unlock;

    if (event & APPLE_CD_REG_INT_DATA_STATUS_UPDATE)
        if (!tps6598x_read_data_status(tps))
            goto err_unlock;

    if (event & APPLE_CD_REG_INT_PLUG_EVENT)
        tps6598x_handle_plug_event(tps, status);

err_unlock:
    mutex_unlock(&tps->lock);
    return event ? IRQ_HANDLED : IRQ_NONE;
}

static bool tps6598x_has_role_changed(struct tps6598x *tps, u32 status)
{
    if (!tps)
        return false;
    return (status ^ tps->status) & (TPS_STATUS_PORTROLE | TPS_STATUS_DATAROLE);
}

static irqreturn_t tps25750_interrupt(int irq, void *data)
{
    struct tps6598x *tps = data;
    u64 event[2] = {0};
    u32 status;
    int ret;

    if (!tps) {
        pr_err("Invalid TPS25750 interrupt: tps=%p\n", tps);
        return IRQ_NONE;
    }

    mutex_lock(&tps->lock);

    ret = tps6598x_block_read(tps, TPS_REG_INT_EVENT1, event, 11);
    if (ret) {
        dev_err(tps->dev, "Failed to read events: %d\n", ret);
        goto err_unlock;
    }
    trace_tps25750_irq(event[0]);

    if (!(event[0] | event[1]))
        goto err_unlock;

    if (!tps6598x_read_status(tps, &status))
        goto err_clear_ints;

    if (event[0] & TPS_REG_INT_POWER_STATUS_UPDATE)
        if (!tps6598x_read_power_status(tps))
            goto err_clear_ints;

    if (event[0] & TPS_REG_INT_DATA_STATUS_UPDATE)
        if (!tps6598x_read_data_status(tps))
            goto err_clear_ints;

    if (event[0] & TPS_REG_INT_PLUG_EVENT || tps6598x_has_role_changed(tps, status))
        tps6598x_handle_plug_event(tps, status);

    tps->status = status;

err_clear_ints:
    tps6598x_block_write(tps, TPS_REG_INT_CLEAR1, event, 11);

err_unlock:
    mutex_unlock(&tps->lock);
    return (event[0] | event[1]) ? IRQ_HANDLED : IRQ_NONE;
}

static irqreturn_t tps6598x_interrupt(int irq, void *data)
{
    struct tps6598x *tps = data;
    u64 event1[2] = {0};
    u64 event2[2] = {0};
    u32 version, status;
    int intev_len = TPS_65981_2_6_INTEVENT_LEN;
    int ret;

    if (!tps) {
        pr_err("Invalid TPS6598x interrupt: tps=%p\n", tps);
        return IRQ_NONE;
    }

    mutex_lock(&tps->lock);

    ret = tps6598x_read32(tps, TPS_REG_VERSION, &version);
    if (ret)
        dev_warn(tps->dev, "Failed to read version: %d\n", ret);

    if (TPS_VERSION_HW_VERSION(version) == TPS_VERSION_HW_65987_8_DH ||
        TPS_VERSION_HW_VERSION(version) == TPS_VERSION_HW_65987_8_DK)
        intev_len = TPS_65987_8_INTEVENT_LEN;

    ret = tps6598x_block_read(tps, TPS_REG_INT_EVENT1, event1, intev_len);
    if (ret) {
        dev_err(tps->dev, "Failed to read event1: %d\n", ret);
        goto err_unlock;
    }
    ret = tps6598x_block_read(tps, TPS_REG_INT_EVENT2, event2, intev_len);
    if (ret) {
        dev_err(tps->dev, "Failed to read event2: %d\n", ret);
        goto err_unlock;
    }
    trace_tps6598x_irq(event1[0], event2[0]);

    if (!(event1[0] | event1[1] | event2[0] | event2[1]))
        goto err_unlock;

    tps6598x_block_write(tps, TPS_REG_INT_CLEAR1, event1, intev_len);
    tps6598x_block_write(tps, TPS_REG_INT_CLEAR2, event2, intev_len);

    if (!tps6598x_read_status(tps, &status))
        goto err_unlock;

    if ((event1[0] | event2[0]) & TPS_REG_INT_POWER_STATUS_UPDATE)
        if (!tps6598x_read_power_status(tps))
            goto err_unlock;

    if ((event1[0] | event2[0]) & TPS_REG_INT_DATA_STATUS_UPDATE)
        if (!tps6598x_read_data_status(tps))
            goto err_unlock;

    if ((event1[0] | event2[0]) & TPS_REG_INT_PLUG_EVENT)
        tps6598x_handle_plug_event(tps, status);

err_unlock:
    mutex_unlock(&tps->lock);
    return (event1[0] | event1[1] | event2[0] | event2[1]) ? IRQ_HANDLED : IRQ_NONE;
}

static void tps6598x_poll_work(struct work_struct *work)
{
    struct tps6598x *tps = container_of(to_delayed_work(work), struct tps6598x, wq_poll);

    if (!tps || !tps->data || !tps->data->irq_handler) {
        pr_err("Invalid poll work: tps=%p\n", tps);
        return;
    }

    tps->data->irq_handler(0, tps);
    queue_delayed_work(system_power_efficient_wq, &tps->wq_poll, msecs_to_jiffies(POLL_INTERVAL));
}

static int tps6598x_check_mode(struct tps6598x *tps)
{
    char mode[5] = {0};
    int ret;

    if (!tps) {
        pr_err("Invalid check mode: tps=%p\n", tps);
        return -EINVAL;
    }

    ret = tps6598x_read32(tps, TPS_REG_MODE, (u32 *)mode);
    if (ret) {
        dev_err(tps->dev, "Failed to read mode: %d\n", ret);
        return ret;
    }

    ret = match_string(modes, ARRAY_SIZE(modes), mode);
    switch (ret) {
    case TPS_MODE_APP:
    case TPS_MODE_PTCH:
        return ret;
    case TPS_MODE_BOOT:
        dev_warn(tps->dev, "Dead-battery condition detected\n");
        return ret;
    case TPS_MODE_BIST:
    case TPS_MODE_DISC:
    default:
        dev_err(tps->dev, "Unsupported mode: %s\n", mode);
        return -ENODEV;
    }
}

static const struct regmap_config tps6598x_regmap_config = {
    .reg_bits = 8,
    .val_bits = 8,
    .max_register = 0x7F,
};

static int tps6598x_psy_get_online(struct tps6598x *tps, union power_supply_propval *val)
{
    if (!tps || !val) {
        dev_err(tps ? tps->dev : NULL, "Invalid psy_get_online: tps=%p, val=%p\n", tps, val);
        return -EINVAL;
    }

    val->intval = (TPS_POWER_STATUS_CONNECTION(tps->pwr_status) &&
                   TPS_POWER_STATUS_SOURCESINK(tps->pwr_status)) ? 1 : 0;
    return 0;
}

static int tps6598x_psy_get_prop(struct power_supply *psy,
                                 enum power_supply_property psp,
                                 union power_supply_propval *val)
{
    struct tps6598x *tps = power_supply_get_drvdata(psy);
    int ret = 0;

    if (!tps || !psy || !val) {
        dev_err(tps ? tps->dev : NULL, "Invalid psy_get_prop: tps=%p, psy=%p, val=%p\n",
                tps, psy, val);
        return -EINVAL;
    }

    switch (psp) {
    case POWER_SUPPLY_PROP_USB_TYPE:
        val->intval = (TPS_POWER_STATUS_PWROPMODE(tps->pwr_status) == TYPEC_PWR_MODE_PD) ?
                      POWER_SUPPLY_USB_TYPE_PD : POWER_SUPPLY_USB_TYPE_C;
        break;
    case POWER_SUPPLY_PROP_ONLINE:
        ret = tps6598x_psy_get_online(tps, val);
        break;
    default:
        ret = -EINVAL;
        break;
    }

    return ret;
}

static int cd321x_switch_power_state(struct tps6598x *tps, u8 target_state)
{
    u8 state;
    int ret;

    if (!tps) {
        pr_err("Invalid switch_power_state: tps=%p\n", tps);
        return -EINVAL;
    }

    ret = tps6598x_read8(tps, TPS_REG_SYSTEM_POWER_STATE, &state);
    if (ret) {
        dev_err(tps->dev, "Failed to read power state: %d\n", ret);
        return ret;
    }

    if (state == target_state)
        return 0;

    ret = tps6598x_exec_cmd(tps, "SSPS", sizeof(u8), &target_state, 0, NULL);
    if (ret) {
        dev_err(tps->dev, "Failed to set power state: %d\n", ret);
        return ret;
    }

    ret = tps6598x_read8(tps, TPS_REG_SYSTEM_POWER_STATE, &state);
    if (ret) {
        dev_err(tps->dev, "Failed to verify power state: %d\n", ret);
        return ret;
    }

    if (state != target_state) {
        dev_err(tps->dev, "Power state mismatch: expected %d, got %d\n", target_state, state);
        return -EINVAL;
    }

    return 0;
}

static int devm_tps6598_psy_register(struct tps6598x *tps)
{
    struct power_supply_config psy_cfg = {};
    char *psy_name;

    if (!tps || !tps->dev) {
        pr_err("Invalid psy_register: tps=%p\n", tps);
        return -EINVAL;
    }

    psy_cfg.drv_data = tps;
    psy_cfg.fwnode = dev_fwnode(tps->dev);

    psy_name = devm_kasprintf(tps->dev, GFP_KERNEL, "%s%s", tps6598x_psy_name_prefix,
                              dev_name(tps->dev));
    if (!psy_name)
        return -ENOMEM;

    tps->psy_desc.name = psy_name;
    tps->psy_desc.type = POWER_SUPPLY_TYPE_USB;
    tps->psy_desc.usb_types = BIT(POWER_SUPPLY_USB_TYPE_C) | BIT(POWER_SUPPLY_USB_TYPE_PD);
    tps->psy_desc.properties = tps6598x_psy_props;
    tps->psy_desc.num_properties = ARRAY_SIZE(tps6598x_psy_props);
    tps->psy_desc.get_property = tps6598x_psy_get_prop;

    tps->usb_type = POWER_SUPPLY_USB_TYPE_C;

    tps->psy = devm_power_supply_register(tps->dev, &tps->psy_desc, &psy_cfg);
    return PTR_ERR_OR_ZERO(tps->psy);
}

static int tps6598x_register_port(struct tps6598x *tps, struct fwnode_handle *fwnode)
{
    struct typec_capability typec_cap = {};
    u32 conf;
    int ret;

    if (!tps || !fwnode) {
        dev_err(tps ? tps->dev : NULL, "Invalid register_port: tps=%p, fwnode=%p\n", tps, fwnode);
        return -EINVAL;
    }

    ret = tps6598x_read32(tps, TPS_REG_SYSTEM_CONF, &conf);
    if (ret) {
        dev_err(tps->dev, "Failed to read system config: %d\n", ret);
        return ret;
    }

    typec_cap.revision = USB_TYPEC_REV_1_2;
    typec_cap.pd_revision = 0x200;
    typec_cap.prefer_role = TYPEC_NO_PREFERRED_ROLE;
    typec_cap.driver_data = tps;
    typec_cap.ops = &tps6598x_ops;
    typec_cap.fwnode = fwnode;

    switch (TPS_SYSCONF_PORTINFO(conf)) {
    case TPS_PORTINFO_SINK_ACCESSORY:
    case TPS_PORTINFO_SINK:
        typec_cap.type = TYPEC_PORT_SNK;
        typec_cap.data = TYPEC_PORT_UFP;
        break;
    case TPS_PORTINFO_DRP_UFP_DRD:
    case TPS_PORTINFO_DRP_DFP_DRD:
        typec_cap.type = TYPEC_PORT_DRP;
        typec_cap.data = TYPEC_PORT_DRD;
        break;
    case TPS_PORTINFO_DRP_UFP:
        typec_cap.type = TYPEC_PORT_DRP;
        typec_cap.data = TYPEC_PORT_UFP;
        break;
    case TPS_PORTINFO_DRP_DFP:
        typec_cap.type = TYPEC_PORT_DRP;
        typec_cap.data = TYPEC_PORT_DFP;
        break;
    case TPS_PORTINFO_SOURCE:
        typec_cap.type = TYPEC_PORT_SRC;
        typec_cap.data = TYPEC_PORT_DFP;
        break;
    default:
        dev_err(tps->dev, "Invalid port info: %d\n", TPS_SYSCONF_PORTINFO(conf));
        return -ENODEV;
    }

    tps->port = typec_register_port(tps->dev, &typec_cap);
    if (IS_ERR(tps->port)) {
        ret = PTR_ERR(tps->port);
        dev_err(tps->dev, "Failed to register port: %d\n", ret);
        return ret;
    }

    return 0;
}

static int tps_request_firmware(struct tps6598x *tps, const struct firmware **fw,
                                const char **firmware_name)
{
    int ret;

    if (!tps || !fw || !firmware_name) {
        dev_err(tps ? tps->dev : NULL, "Invalid request_firmware: tps=%p, fw=%p, name=%p\n",
                tps, fw, firmware_name);
        return -EINVAL;
    }

    ret = device_property_read_string(tps->dev, "firmware-name", firmware_name);
    if (ret) {
        dev_err(tps->dev, "Failed to read firmware-name: %d\n", ret);
        return ret;
    }

    ret = request_firmware(fw, *firmware_name, tps->dev);
    if (ret) {
        dev_err(tps->dev, "Failed to retrieve firmware %s: %d\n", *firmware_name, ret);
        return ret;
    }

    if ((*fw)->size == 0) {
        dev_err(tps->dev, "Empty firmware: %s\n", *firmware_name);
        release_firmware(*fw);
        return -EINVAL;
    }

    return 0;
}

static int tps25750_write_firmware(struct tps6598x *tps, u8 bpms_addr, const u8 *data, size_t len)
{
    struct i2c_client *client = to_i2c_client(tps->dev);
    int ret;
    u8 slave_addr;
    int timeout;

    if (!tps || !data || len == 0) {
        dev_err(tps ? tps->dev : NULL, "Invalid write_firmware: tps=%p, data=%p, len=%zu\n",
                tps, data, len);
        return -EINVAL;
    }

    slave_addr = client->addr;
    timeout = client->adapter->timeout;
    client->adapter->timeout = msecs_to_jiffies(5000);
    client->addr = bpms_addr;

    ret = regmap_raw_write(tps->regmap, data[0], &data[1], len - 1);

    client->addr = slave_addr;
    client->adapter->timeout = timeout;

    if (ret)
        dev_err(tps->dev, "Failed to write firmware: %d\n", ret);

    return ret;
}

static int tps25750_exec_pbms(struct tps6598x *tps, u8 *in_data, size_t in_len)
{
    u8 rc;
    int ret;

    if (!tps || !in_data) {
        dev_err(tps ? tps->dev : NULL, "Invalid exec_pbms: tps=%p, in_data=%p\n", tps, in_data);
        return -EINVAL;
    }

    ret = tps6598x_exec_cmd_tmo(tps, "PBMs", in_len, in_data, sizeof(rc), &rc, 4000, 0);
    if (ret) {
        dev_err(tps->dev, "PBMs command failed: %d\n", ret);
        return ret;
    }

    switch (rc) {
    case TPS_TASK_BPMS_INVALID_BUNDLE_SIZE:
        dev_err(tps->dev, "Invalid firmware size\n");
        return -EINVAL;
    case TPS_TASK_BPMS_INVALID_SLAVE_ADDR:
        dev_err(tps->dev, "Invalid slave address\n");
        return -EINVAL;
    case TPS_TASK_BPMS_INVALID_TIMEOUT:
        dev_err(tps->dev, "PBMs timed out\n");
        return -ETIMEDOUT;
    default:
        break;
    }

    return 0;
}

static int tps25750_abort_patch_process(struct tps6598x *tps)
{
    int ret;

    if !tps) {
        pr_err("Invalid abort_patch: tps=%p\n", tps);
        return -EINVAL;
    }

    ret = tps6598x_exec_cmd(tps, "PBMe", 0, NULL, 0, NULL);
    if (ret) {
        dev_err(tps->dev, "Failed to abort patch: %d\n", ret);
        return ret;
    }

    ret = tps6598x_check_mode(tps);
    if (ret != TPS_MODE_PTCH)
        dev_err(tps->dev, "Failed to switch to PTCH mode: %d\n", ret);

    return ret;
}

static int tps25750_start_patch_burst_mode(struct tps6598x *tps)
{
    const struct firmware *fw;
    const char *firmware_name;
    struct {
        u32 fw_size;
        u8 addr;
        u8 timeout;
    } __packed bpms_data;
    u32 addr;
    int ret;

    if (!tps || !tps->dev->of_node) {
        dev_err(tps ? tps->dev : NULL, "Invalid start_patch: tps=%p, of_node=%p\n",
                tps, tps ? tps->dev->of_node : NULL);
        return -EINVAL;
    }

    ret = tps_request_firmware(tps, &fw, &firmware_name);
    if (ret)
        return ret;

    ret = of_property_match_string(tps->dev->of_node, "reg-names", "patch-address");
    if (ret < 0) {
        dev_err(tps->dev, "Failed to get patch-address: %d\n", ret);
        goto release_fw;
    }

    ret = of_property_read_u32_index(tps->dev->of_node, "reg", ret, &addr);
    if (ret) {
        dev_err(tps->dev, "Failed to read patch address: %d\n", ret);
        goto release_fw;
    }

    if (addr == 0 || (addr >= 0x20 && addr <= 0x23)) {
        dev_err(tps->dev, "Invalid patch address: %u\n", addr);
        ret = -EINVAL;
        goto release_fw;
    }

    bpms_data.addr = (u8)addr;
    bpms_data.fw_size = fw->size;
    bpms_data.timeout = TPS_BUNDLE_TIMEOUT;

    ret = tps25750_exec_pbms(tps, (u8 *)&bpms_data, sizeof(bpms_data));
    if (ret)
        goto release_fw;

    ret = tps25750_write_firmware(tps, bpms_data.addr, fw->data, fw->size);
    if (ret)
        goto release_fw;

    udelay(500); /* Required delay per TI manual */

release_fw:
    release_firmware(fw);
    return ret;
}

static int tps25750_complete_patch_process(struct tps6598x *tps)
{
    u8 out_data[40] = {0};
    u8 dummy[2] = {0};
    int ret;

    if (!tps) {
        pr_err("Invalid complete_patch: tps=%p\n", tps);
        return -EINVAL;
    }

    ret = tps6598x_exec_cmd_tmo(tps, "PBMc", sizeof(dummy), dummy, sizeof(out_data), out_data, 2000, 20);
    if (ret) {
        dev_err(tps->dev, "PBMc command failed: %d\n", ret);
        return ret;
    }

    if (out_data[TPS_PBMC_RC]) {
        dev_err(tps->dev, "PBMc failed: %u\n", out_data[TPS_PBMC_RC]);
        return -EIO;
    }

    if (out_data[TPS_PBMC_DPCS]) {
        dev_err(tps->dev, "Device patch complete status failed: %u\n", out_data[TPS_PBMC_DPCS]);
        return -EIO;
    }

    return 0;
}

static int tps25750_apply_patch(struct tps6598x *tps)
{
    u64 status = 0;
    int ret;

    if (!tps) {
        pr_err("Invalid apply_patch: tps=%p\n", tps);
        return -EINVAL;
    }

    ret = tps6598x_block_read(tps, TPS_REG_BOOT_STATUS, &status, 5);
    if (ret) {
        dev_err(tps->dev, "Failed to read boot status: %d\n", ret);
        return ret;
    }

    if (status & TPS_BOOT_STATUS_I2C_EEPROM_PRESENT)
        goto wait_for_app;

    ret = tps25750_start_patch_burst_mode(tps);
    if (ret) {
        tps25750_abort_patch_process(tps);
        return ret;
    }

    ret = tps25750_complete_patch_process(tps);
    if (ret)
        return ret;

wait_for_app:
    ret = wait_event_timeout(jiffies + msecs_to_jiffies(1000),
                             tps6598x_check_mode(tps) == TPS_MODE_APP, 1000);
    if (!ret) {
        dev_err(tps->dev, "Timeout waiting for APP mode\n");
        return -ETIMEDOUT;
    }

    if (status & TPS_BOOT_STATUS_DEAD_BATTERY_FLAG) {
        ret = tps6598x_exec_cmd(tps, "DBfg", 0, NULL, 0, NULL);
        if (ret) {
            dev_err(tps->dev, "Failed to clear dead battery flag: %d\n", ret);
            return ret;
        }
    }

    dev_info(tps->dev, "Controller switched to APP mode\n");
    return 0;
}

static int tps6598x_apply_patch(struct tps6598x *tps)
{
    const struct firmware *fw;
    const char *firmware_name;
    u8 in = TPS_PTCS_CONTENT_DEV | TPS_PTCS_CONTENT_APP;
    u8 out[TPS_MAX_LEN] = {0};
    size_t in_len = sizeof(in);
    size_t copied_bytes = 0;
    size_t bytes_left;
    int ret;

    if (!tps) {
        pr_err("Invalid apply_patch: tps=%p\n", tps);
        return -EINVAL;
    }

    ret = tps_request_firmware(tps, &fw, &firmware_name);
    if (ret)
        return ret;

    ret = tps6598x_exec_cmd(tps, "PTCs", in_len, &in, TPS_PTCS_OUT_BYTES, out);
    if (ret || out[TPS_PTCS_STATUS] == TPS_PTCS_STATUS_FAIL) {
        dev_err(tps->dev, "Patch start failed: %d\n", ret ? ret : -EBUSY);
        goto release_fw;
    }

    bytes_left = fw->size;
    while (bytes_left) {
        in_len = min(bytes_left, (size_t)TPS_MAX_LEN);
        ret = tps6598x_exec_cmd(tps, "PTCd", in_len, fw->data + copied_bytes,
                                TPS_PTCD_OUT_BYTES, out);
        if (ret || out[TPS_PTCD_TRANSFER_STATUS] ||
            out[TPS_PTCD_LOADING_STATE] == TPS_PTCD_LOAD_ERR) {
            dev_err(tps->dev, "Patch download failed: %d\n", ret ? ret : -EBUSY);
            goto release_fw;
        }
        copied_bytes += in_len;
        bytes_left -= in_len;
    }

    ret = tps6598x_exec_cmd(tps, "PTCc", 0, NULL, TPS_PTCC_OUT_BYTES, out);
    if (ret || out[TPS_PTCC_DEV] || out[TPS_PTCC_APP]) {
        dev_err(tps->dev, "Patch completion failed: %d\n", ret ? ret : -EBUSY);
        goto release_fw;
    }

    msleep(TPS_SETUP_MS);
    dev_info(tps->dev, "Firmware update succeeded\n");

release_fw:
    release_firmware(fw);
    if (ret)
        dev_err(tps->dev, "Failed to apply firmware %s: %d\n", firmware_name, ret);
    return ret;
}

static int cd321x_init(struct tps6598x *tps)
{
    if (!tps) {
        pr_err("Invalid cd321x_init: tps=%p\n", tps);
        return -EINVAL;
    }

    /* Additional initialization for Apple Silicon */
    return cd321x_switch_power_state(tps, TPS_SYSTEM_POWER_STATE_S0);
}

static int tps25750_init(struct tps6598x *tps)
{
    int ret;

    if (!tps || !tps->data) {
        pr_err("Invalid tps25750_init: tps=%p\n", tps);
        return -EINVAL;
    }

    ret = tps->data->apply_patch(tps);
    if (ret) {
        dev_err(tps->dev, "Failed to apply patch: %d\n", ret);
        return ret;
    }

    ret = tps6598x_write8(tps, TPS_REG_SLEEP_CONF, TPS_SLEEP_CONF_SLEEP_MODE_ALLOWED);
    if (ret)
        dev_warn(tps->dev, "Failed to enable sleep mode: %d\n", ret);

    return 0;
}

static int tps6598x_init(struct tps6598x *tps)
{
    if (!tps || !tps->data) {
        pr_err("Invalid tps6598x_init: tps=%p\n", tps);
        return -EINVAL;
    }

    return tps->data->apply_patch(tps);
}

static int cd321x_reset(struct tps6598x *tps)
{
    if (!tps) {
        pr_err("Invalid cd321x_reset: tps=%p\n", tps);
        return -EINVAL;
    }

    /* Implement reset for Apple Silicon if needed */
    return 0;
}

static int tps25750_reset(struct tps6598x *tps)
{
    if (!tps) {
        pr_err("Invalid tps25750_reset: tps=%p\n", tps);
        return -EINVAL;
    }

    return tps6598x_exec_cmd_tmo(tps, "GAID", 0, NULL, 0, NULL, 2000, 0);
}

static int tps6598x_reset(struct tps6598x *tps)
{
    if (!tps) {
        pr_err("Invalid tps6598x_reset: tps=%p\n", tps);
        return -EINVAL;
    }

    return 0;
}

static int tps25750_register_port(struct tps6598x *tps, struct fwnode_handle *fwnode)
{
    struct typec_capability typec_cap = {};
    const char *data_role;
    u8 pd_status;
    int ret;

    if (!tps || !fwnode) {
        dev_err(tps ? tps->dev : NULL, "Invalid register_port: tps=%p, fwnode=%p\n", tps, fwnode);
        return -EINVAL;
    }

    ret = tps6598x_read8(tps, TPS_REG_PD_STATUS, &pd_status);
    if (ret) {
        dev_err(tps->dev, "Failed to read PD status: %d\n", ret);
        return ret;
    }

    ret = fwnode_property_read_string(fwnode, "data-role", &data_role);
    if (ret) {
        dev_err(tps->dev, "Failed to read data-role: %d\n", ret);
        return ret;
    }

    ret = typec_find_port_data_role(data_role);
    if (ret < 0) {
        dev_err(tps->dev, "Unknown data-role: %s\n", data_role);
        return ret;
    }

    typec_cap.data = ret;
    typec_cap.revision = USB_TYPEC_REV_1_3;
    typec_cap.pd_revision = 0x300;
    typec_cap.driver_data = tps;
    typec_cap.ops = &tps6598x_ops;
    typec_cap.fwnode = fwnode;
    typec_cap.prefer_role = TYPEC_NO_PREFERRED_ROLE;

    switch (TPS_PD_STATUS_PORT_TYPE(pd_status)) {
    case TPS_PD_STATUS_PORT_TYPE_SINK_SOURCE:
    case TPS_PD_STATUS_PORT_TYPE_SOURCE_SINK:
        typec_cap.type = TYPEC_PORT_DRP;
        break;
    case TPS_PD_STATUS_PORT_TYPE_SINK:
        typec_cap.type = TYPEC_PORT_SNK;
        break;
    case TPS_PD_STATUS_PORT_TYPE_SOURCE:
        typec_cap.type = TYPEC_PORT_SRC;
        break;
    default:
        dev_err(tps->dev, "Invalid port type: %d\n", TPS_PD_STATUS_PORT_TYPE(pd_status));
        return -ENODEV;
    }

    tps->port = typec_register_port(tps->dev, &typec_cap);
    if (IS_ERR(tps->port)) {
        ret = PTR_ERR(tps->port);
        dev_err(tps->dev, "Failed to register port: %d\n", ret);
        return ret;
    }

    return 0;
}

static int tps6598x_probe(struct i2c_client *client)
{
    struct device_node *np = client->dev.of_node;
    struct tps6598x *tps;
    struct fwnode_handle *fwnode;
    u32 status, vid;
    u64 mask1;
    int ret;

    tps = devm_kzalloc(&client->dev, sizeof(*tps), GFP_KERNEL);
    if (!tps)
        return -ENOMEM;

    mutex_init(&tps->lock);
    tps->dev = &client->dev;

    tps->reset = devm_gpiod_get_optional(tps->dev, "reset", GPIOD_OUT_LOW);
    if (IS_ERR(tps->reset))
        return dev_err_probe(tps->dev, PTR_ERR(tps->reset), "Failed to get reset GPIO\n");
    if (tps->reset)
        msleep(TPS_SETUP_MS);

    tps->regmap = devm_regmap_init_i2c(client, &tps6598x_regmap_config);
    if (IS_ERR(tps->regmap))
        return PTR_ERR(tps->regmap);

    if (!device_is_compatible(tps->dev, "ti,tps25750")) {
        ret = tps6598x_read32(tps, TPS_REG_VID, &vid);
        if (ret || !vid) {
            dev_err(tps->dev, "Failed to read VID: %d\n", ret);
            return -ENODEV;
        }
    }

    tps->i2c_protocol = i2c_check_functionality(client->adapter, I2C_FUNC_I2C);

    tps->data = i2c_get_match_data(client);
    if (!tps->data) {
        dev_err(tps->dev, "No matching device data\n");
        return -EINVAL;
    }

    if (np && of_device_is_compatible(np, "apple,cd321x")) {
        ret = cd321x_switch_power_state(tps, TPS_SYSTEM_POWER_STATE_S0);
        if (ret) {
            dev_err(tps->dev, "Failed to switch power state: %d\n", ret);
            return ret;
        }
        mask1 = APPLE_CD_REG_INT_POWER_STATUS_UPDATE |
                APPLE_CD_REG_INT_DATA_STATUS_UPDATE |
                APPLE_CD_REG_INT_PLUG_EVENT;
    } else {
        mask1 = TPS_REG_INT_POWER_STATUS_UPDATE |
                TPS_REG_INT_DATA_STATUS_UPDATE |
                TPS_REG_INT_PLUG_EVENT;
    }

    ret = tps6598x_check_mode(tps);
    if (ret < 0) {
        dev_err(tps->dev, "Failed to check mode: %d\n", ret);
        return ret;
    }

    if (ret == TPS_MODE_PTCH) {
        ret = tps->data->init(tps);
        if (ret) {
            dev_err(tps->dev, "Failed to initialize: %d\n", ret);
            return ret;
        }
    }

    ret = tps6598x_write64(tps, TPS_REG_INT_MASK1, mask1);
    if (ret) {
        dev_err(tps->dev, "Failed to set interrupt mask: %d\n", ret);
        goto err_reset_controller;
    }

    if (!tps6598x_read_status(tps, &status)) {
        ret = -ENODEV;
        goto err_clear_mask;
    }

    fwnode = device_get_named_child_node(&client->dev, "connector");
    if (fwnode)
        fw_devlink_purge_absent_suppliers(fwnode);

    tps->role_sw = fwnode_usb_role_switch_get(fwnode);
    if (IS_ERR(tps->role_sw)) {
        ret = PTR_ERR(tps->role_sw);
        dev_err(tps->dev, "Failed to get role switch: %d\n", ret);
        goto err_fwnode_put;
    }

    ret = devm_tps6598_psy_register(tps);
    if (ret) {
        dev_err(tps->dev, "Failed to register power supply: %d\n", ret);
        goto err_role_put;
    }

    ret = tps->data->register_port(tps, fwnode);
    if (ret) {
        dev_err(tps->dev, "Failed to register port: %d\n", ret);
        goto err_role_put;
    }

    if (status & TPS_STATUS_PLUG_PRESENT) {
        ret = tps6598x_read16(tps, TPS_REG_POWER_STATUS, &tps->pwr_status);
        if (ret) {
            dev_err(tps->dev, "Failed to read power status: %d\n", ret);
            goto err_unregister_port;
        }
        ret = tps6598x_connect(tps, status);
        if (ret)
            dev_err(tps->dev, "Failed to register partner: %d\n", ret);
    }

    if (client->irq) {
        ret = devm_request_threaded_irq(&client->dev, client->irq, NULL,
                                        tps->data->irq_handler,
                                        IRQF_SHARED | IRQF_ONESHOT,
                                        dev_name(&client->dev), tps);
        if (ret)
            dev_err(tps->dev, "Failed to request IRQ: %d\n", ret);
    } else {
        dev_info(tps->dev, "No IRQ, using polling\n");
        INIT_DELAYED_WORK(&tps->wq_poll, tps6598x_poll_work);
        queue_delayed_work(system_power_efficient_wq, &tps->wq_poll,
                           msecs_to_jiffies(POLL_INTERVAL));
    }

    if (ret)
        goto err_disconnect;

    i2c_set_clientdata(client, tps);
    fwnode_handle_put(fwnode);

    tps->wakeup = device_property_read_bool(tps->dev, "wakeup-source");
    if (tps->wakeup && client->irq) {
        device_init_wakeup(&client->dev, true);
        enable_irq_wake(client->irq);
    }

    return 0;

err_disconnect:
    tps6598x_disconnect(tps, 0);
err_unregister_port:
    typec_unregister_port(tps->port);
err_role_put:
    usb_role_switch_put(tps->role_sw);
err_fwnode_put:
    fwnode_handle_put(fwnode);
err_clear_mask:
    tps6598x_write64(tps, TPS_REG_INT_MASK1, 0);
err_reset_controller:
    tps->data->reset(tps);
    return ret;
}

static void tps6598x_remove(struct i2c_client *client)
{
    struct tps6598x *tps = i2c_get_clientdata(client);

    if (!client->irq)
        cancel_delayed_work_sync(&tps->wq_poll);
    else
        devm_free_irq(tps->dev, client->irq, tps);

    tps6598x_disconnect(tps, 0);
    typec_unregister_port(tps->port);
    usb_role_switch_put(tps->role_sw);

    tps->data->reset(tps);

    if (tps->reset)
        gpiod_set_value_cansleep(tps->reset, 1);
}

static int tps6598x_suspend(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct tps6598x *tps = i2c_get_clientdata(client);

    if (!tps || !client) {
        dev_err(dev, "Invalid suspend: tps=%p, client=%p\n", tps, client);
        return -EINVAL;
    }

    if (tps->wakeup) {
        disable_irq(client->irq);
        enable_irq_wake(client->irq);
    } else if (tps->reset) {
        gpiod_set_value_cansleep(tps->reset, 1);
    }

    if (!client->irq)
        cancel_delayed_work_sync(&tps->wq_poll);

    return 0;
}

static int tps6598x_resume(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct tps6598x *tps = i2c_get_clientdata(client);
    int ret;

    if (!tps || !client) {
        dev_err(dev, "Invalid resume: tps=%p, client=%p\n", tps, client);
        return -EINVAL;
    }

    ret = tps6598x_check_mode(tps);
    if (ret < 0) {
        dev_err(tps->dev, "Failed to check mode: %d\n", ret);
        return ret;
    }

    if (ret == TPS_MODE_PTCH) {
        ret = tps->data->init(tps);
        if (ret) {
            dev_err(tps->dev, "Failed to initialize: %d\n", ret);
            return ret;
        }
    }

    if (tps->wakeup) {
        disable_irq_wake(client->irq);
        enable_irq(client->irq);
    } else if (tps->reset) {
        gpiod_set_value_cansleep(tps->reset, 0);
        msleep(TPS_SETUP_MS);
    }

    if (!client->irq)
        queue_delayed_work(system_power_efficient_wq, &tps->wq_poll,
                           msecs_to_jiffies(POLL_INTERVAL));

    return 0;
}

static const struct dev_pm_ops tps6598x_pm_ops = {
    SET_SYSTEM_SLEEP_PM_OPS(tps6598x_suspend, tps6598x_resume)
};

static const struct tipd_data cd321x_data = {
    .irq_handler = cd321x_interrupt,
    .register_port = tps6598x_register_port,
    .trace_power_status = trace_tps6598x_power_status,
    .trace_status = trace_tps6598x_status,
    .init = cd321x_init,
    .reset = cd321x_reset,
};

static const struct tipd_data tps6598x_data = {
    .irq_handler = tps6598x_interrupt,
    .register_port = tps6598x_register_port,
    .trace_power_status = trace_tps6598x_power_status,
    .trace_status = trace_tps6598x_status,
    .apply_patch = tps6598x_apply_patch,
    .init = tps6598x_init,
    .reset = tps6598x_reset,
};

static const struct tipd_data tps25750_data = {
    .irq_handler = tps25750_interrupt,
    .register_port = tps25750_register_port,
    .trace_power_status = trace_tps25750_power_status,
    .trace_status = trace_tps25750_status,
    .apply_patch = tps25750_apply_patch,
    .init = tps25750_init,
    .reset = tps25750_reset,
};

static const struct of_device_id tps6598x_of_match[] = {
    { .compatible = "ti,tps6598x", .data = &tps6598x_data },
    { .compatible = "apple,cd321x", .data = &cd321x_data },
    { .compatible = "ti,tps25750", .data = &tps25750_data },
    {}
};
MODULE_DEVICE_TABLE(of, tps6598x_of_match);

static const struct i2c_device_id tps6598x_id[] = {
    { "tps6598x", (kernel_ulong_t)&tps6598x_data },
    {}
};
MODULE_DEVICE_TABLE(i2c, tps6598x_id);

static struct i2c_driver tps6598x_i2c_driver = {
    .driver = {
        .name = "tps6598x",
        .pm = &tps6598x_pm_ops,
        .of_match_table = tps6598x_of_match,
    },
    .probe = tps6598x_probe,
    .remove = tps6598x_remove,
    .id_table = tps6598x_id,
};
module_i2c_driver(tps6598x_i2c_driver);

MODULE_AUTHOR("Heikki Krogerus <heikki.krogerus@linux.intel.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("TI TPS6598x USB Power Delivery Controller Driver");
