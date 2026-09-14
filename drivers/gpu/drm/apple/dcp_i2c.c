// SPDX-License-Identifier: GPL-2.0-only OR MIT
/* Copyright The Asahi Linux Contributors */

/*
 * DDC/CI through the DCP firmware.
 *
 * The DCP owns the DisplayPort link, including its AUX channel, so the AP
 * cannot run I2C-over-AUX itself and no DDC bus exists for external displays.
 * The firmware will however run a transaction on request, which is how macOS
 * implements IOAVServiceReadI2C() / IOAVServiceWriteI2C() and hence DDC/CI
 * backlight control. Expose that as an ordinary I2C adapter on the connector,
 * so ddcutil and the DDC/CI backlight helpers work unmodified.
 *
 * A firmware transaction is atomic: address, payload and STOP all happen
 * inside one call, and the first payload byte is passed separately as the
 * "data address" (0x51 for DDC/CI). Reads take a data address of their own,
 * which is 0 for DDC/CI replies and the block offset when reading an EDID.
 */

#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/string.h>

#include "connector.h"
#include "dcp.h"
#include "epic/dpavservep.h"

/* 7 bit address of the DDC/CI command register set. */
#define DCP_I2C_DDC_ADDR	0x37

/*
 * DDC/CI is slow and the firmware does not wait for the display on our behalf.
 * The delays macOS tools settled on: ~10ms for a write to land, ~40ms before a
 * reply can be read. Only DDC/CI needs them; EDID reads are unaffected.
 */
#define DCP_I2C_WRITE_DELAY_US	10000
#define DCP_I2C_READ_DELAY_US	40000

static int dcp_i2c_read(struct apple_dcp *dcp, struct i2c_msg *msg,
			u32 data_addr)
{
	if (msg->addr == DCP_I2C_DDC_ADDR)
		usleep_range(DCP_I2C_READ_DELAY_US, DCP_I2C_READ_DELAY_US + 10000);

	return dcpavserv_read_i2c(dcp->dcpavserv.service, msg->addr, data_addr,
				  msg->buf, msg->len);
}

static int dcp_i2c_write(struct apple_dcp *dcp, struct i2c_msg *msg)
{
	u32 data_addr = msg->len ? msg->buf[0] : 0;
	const void *data = msg->len ? msg->buf + 1 : NULL;
	size_t len = msg->len ? msg->len - 1 : 0;
	int ret;

	ret = dcpavserv_write_i2c(dcp->dcpavserv.service, msg->addr, data_addr,
				  data, len);
	if (!ret && msg->addr == DCP_I2C_DDC_ADDR)
		usleep_range(DCP_I2C_WRITE_DELAY_US, DCP_I2C_WRITE_DELAY_US + 2000);

	return ret;
}

static int dcp_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg *msgs, int num)
{
	struct platform_device *pdev = i2c_get_adapdata(adap);
	struct apple_dcp *dcp = platform_get_drvdata(pdev);
	int i, ret;

	if (!dcp->dcpavserv.enabled || !dcp->dcpavserv.service)
		return -ENODEV;

	for (i = 0; i < num; i++) {
		struct i2c_msg *msg = &msgs[i];

		if (msg->flags & I2C_M_RD) {
			ret = dcp_i2c_read(dcp, msg, 0);
		} else if (msg->len == 1 && i + 1 < num &&
			   (msgs[i + 1].flags & I2C_M_RD) &&
			   msgs[i + 1].addr == msg->addr) {
			/*
			 * An offset write in front of a read, as used to read
			 * an EDID block. The firmware takes the offset as the
			 * read's data address instead.
			 */
			ret = dcp_i2c_read(dcp, &msgs[i + 1], msg->buf[0]);
			i++;
		} else {
			ret = dcp_i2c_write(dcp, msg);
		}

		if (ret)
			return ret;
	}

	return num;
}

static u32 dcp_i2c_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static const struct i2c_algorithm dcp_i2c_algo = {
	.xfer = dcp_i2c_xfer,
	.functionality = dcp_i2c_func,
};

struct i2c_adapter *dcp_i2c_create(struct platform_device *pdev)
{
	struct apple_dcp *dcp = platform_get_drvdata(pdev);
	struct i2c_adapter *adap;
	int ret;

	adap = devm_kzalloc(dcp->dev, sizeof(*adap), GFP_KERNEL);
	if (!adap)
		return NULL;

	adap->owner = THIS_MODULE;
	adap->algo = &dcp_i2c_algo;
	adap->dev.parent = dcp->dev;
	adap->retries = 3;
	strscpy(adap->name, "Apple DCP DDC", sizeof(adap->name));
	i2c_set_adapdata(adap, pdev);

	ret = devm_i2c_add_adapter(dcp->dev, adap);
	if (ret) {
		dev_warn(dcp->dev, "failed to add DDC adapter: %d\n", ret);
		return NULL;
	}

	return adap;
}
