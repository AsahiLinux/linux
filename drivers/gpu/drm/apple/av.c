// SPDX-License-Identifier: GPL-2.0-only OR MIT
/* Copyright 2023 Martin Povišer <povik+lin@cutebit.org> */

// #define DEBUG

#include <linux/debugfs.h>
#include <linux/kconfig.h>
#include <linux/rwsem.h>
#include <linux/types.h>

#include "audio.h"
#include "afk.h"
#include "dcp.h"

struct audiosrv_data {
	struct device *audio_dev;
	dcp_audio_hotplug_callback hotplug_cb;
	bool plugged;
	struct mutex plug_lock;

	struct apple_epic_service *srv;
	struct rw_semaphore srv_rwsem;
};

static void av_interface_init(struct apple_epic_service *service, const char *name,
			      const char *class, s64 unit)
{
}

static void av_audiosrv_init(struct apple_epic_service *service, const char *name,
			     const char *class, s64 unit)
{
	struct apple_dcp *dcp = service->ep->dcp;
	struct audiosrv_data *asrv = dcp->audiosrv;
	int err;

	mutex_lock(&asrv->plug_lock);

	down_write(&asrv->srv_rwsem);
	asrv->srv = service;
	up_write(&asrv->srv_rwsem);

	/* TODO: this must be done elsewhere */
	err = afk_service_call(asrv->srv, 0, 6, NULL, 0, 32, NULL, 0, 32);
	if (err)
		dev_err(dcp->dev, "error opening audio service: %d\n", err);

	asrv->plugged = true;
	if (asrv->hotplug_cb)
		asrv->hotplug_cb(asrv->audio_dev, true);

	mutex_unlock(&asrv->plug_lock);
}

static void av_audiosrv_teardown(struct apple_epic_service *service)
{
	struct apple_dcp *dcp = service->ep->dcp;
	struct audiosrv_data *asrv = dcp->audiosrv;

	mutex_lock(&asrv->plug_lock);

	down_write(&asrv->srv_rwsem);
	asrv->srv = NULL;
	up_write(&asrv->srv_rwsem);

	asrv->plugged = false;
	if (asrv->hotplug_cb)
		asrv->hotplug_cb(asrv->audio_dev, false);

	mutex_unlock(&asrv->plug_lock);
}

void dcp_audiosrv_set_hotplug_cb(struct device *dev, struct device *audio_dev,
								 dcp_audio_hotplug_callback cb)
{
	struct apple_dcp *dcp = dev_get_drvdata(dev);
	struct audiosrv_data *asrv = dcp->audiosrv;

	mutex_lock(&asrv->plug_lock);
	asrv->audio_dev = audio_dev;
	asrv->hotplug_cb = cb;

	if (cb)
		cb(audio_dev, asrv->plugged);
	mutex_unlock(&asrv->plug_lock);
}
EXPORT_SYMBOL_GPL(dcp_audiosrv_set_hotplug_cb);

int dcp_audiosrv_prepare(struct device *dev, struct dcp_sound_cookie *cookie)
{
	struct apple_dcp *dcp = dev_get_drvdata(dev);
	struct audiosrv_data *asrv = dcp->audiosrv;
	int ret;

	down_write(&asrv->srv_rwsem);
	ret = afk_service_call(asrv->srv, 0, 8, cookie, sizeof(*cookie),
			       64 - sizeof(*cookie), NULL, 0, 64);
	up_write(&asrv->srv_rwsem);

	return ret;
}
EXPORT_SYMBOL_GPL(dcp_audiosrv_prepare);

int dcp_audiosrv_startlink(struct device *dev, struct dcp_sound_cookie *cookie)
{
	struct apple_dcp *dcp = dev_get_drvdata(dev);
	struct audiosrv_data *asrv = dcp->audiosrv;
	int ret;

	down_write(&asrv->srv_rwsem);
	ret = afk_service_call(asrv->srv, 0, 9, cookie, sizeof(*cookie),
			       64 - sizeof(*cookie), NULL, 0, 64);
	up_write(&asrv->srv_rwsem);

	return ret;
}
EXPORT_SYMBOL_GPL(dcp_audiosrv_startlink);

int dcp_audiosrv_stoplink(struct device *dev)
{
	struct apple_dcp *dcp = dev_get_drvdata(dev);
	struct audiosrv_data *asrv = dcp->audiosrv;
	int ret;

	down_write(&asrv->srv_rwsem);
	ret = afk_service_call(asrv->srv, 0, 12, NULL, 0, 64, NULL, 0, 64);
	up_write(&asrv->srv_rwsem);

	return ret;
}
EXPORT_SYMBOL_GPL(dcp_audiosrv_stoplink);

int dcp_audiosrv_unprepare(struct device *dev)
{
	struct apple_dcp *dcp = dev_get_drvdata(dev);
	struct audiosrv_data *asrv = dcp->audiosrv;
	int ret;

	down_write(&asrv->srv_rwsem);
	ret = afk_service_call(asrv->srv, 0, 13, NULL, 0, 64, NULL, 0, 64);
	up_write(&asrv->srv_rwsem);

	return ret;
}
EXPORT_SYMBOL_GPL(dcp_audiosrv_unprepare);

static int
dcp_audiosrv_osobject_call(struct apple_epic_service *service, u16 group,
			   u32 command, void *output, size_t output_maxsize,
			   size_t *output_size)
{
	struct {
		__le64 max_size;
		u8 _pad1[24];
		__le64 used_size;
		u8 _pad2[8];
	} __attribute__((packed)) *hdr;
	static_assert(sizeof(*hdr) == 48);
	size_t bfr_len = output_maxsize + sizeof(*hdr);
	void *bfr;
	int ret;

	bfr = kzalloc(bfr_len, GFP_KERNEL);
	if (!bfr)
		return -ENOMEM;

	hdr = bfr;
	hdr->max_size = cpu_to_le64(output_maxsize);
	ret = afk_service_call(service, group, command, hdr, sizeof(*hdr), output_maxsize,
			       bfr, sizeof(*hdr) + output_maxsize, 0);
	if (ret)
		return ret;

	if (output)
		memcpy(output, bfr + sizeof(*hdr), output_maxsize);

	if (output_size)
		*output_size = le64_to_cpu(hdr->used_size);

	return 0;
}

int dcp_audiosrv_get_elements(struct device *dev, void *elements, size_t maxsize)
{
	struct apple_dcp *dcp = dev_get_drvdata(dev);
	struct audiosrv_data *asrv = dcp->audiosrv;
	size_t size;
	int ret;

	down_write(&asrv->srv_rwsem);
	ret = dcp_audiosrv_osobject_call(asrv->srv, 1, 18, elements, maxsize, &size);
	up_write(&asrv->srv_rwsem);

	if (ret)
		dev_err(dev, "audiosrv: error getting elements: %d\n", ret);
	else
		dev_dbg(dev, "audiosrv: got %zd bytes worth of elements\n", size);

	return ret;
}
EXPORT_SYMBOL_GPL(dcp_audiosrv_get_elements);

int dcp_audiosrv_get_product_attrs(struct device *dev, void *attrs, size_t maxsize)
{
	struct apple_dcp *dcp = dev_get_drvdata(dev);
	struct audiosrv_data *asrv = dcp->audiosrv;
	size_t size;
	int ret;

	down_write(&asrv->srv_rwsem);
	ret = dcp_audiosrv_osobject_call(asrv->srv, 1, 20, attrs, maxsize, &size);
	up_write(&asrv->srv_rwsem);

	if (ret)
		dev_err(dev, "audiosrv: error getting product attributes: %d\n", ret);
	else
		dev_dbg(dev, "audiosrv: got %zd bytes worth of product attributes\n", size);

	return ret;
}
EXPORT_SYMBOL_GPL(dcp_audiosrv_get_product_attrs);

static int av_audiosrv_report(struct apple_epic_service *service, u32 idx,
						  const void *data, size_t data_size)
{
	dev_dbg(service->ep->dcp->dev, "got audio report %d size %zx\n", idx, data_size);
#ifdef DEBUG
	print_hex_dump(KERN_DEBUG, "audio report: ", DUMP_PREFIX_NONE, 16, 1, data, data_size, true);
#endif

	return 0;
}

static const struct apple_epic_service_ops avep_ops[] = {
	{
		.name = "DCPAVSimpleVideoInterface",
		.init = av_interface_init,
	},
	{
		.name = "DCPAVAudioInterface",
		.init = av_audiosrv_init,
		.report = av_audiosrv_report,
		.teardown = av_audiosrv_teardown,
	},
	{}
};

int avep_init(struct apple_dcp *dcp)
{
	struct dcp_audio_pdata *audio_pdata;
	struct platform_device *audio_pdev;
	struct audiosrv_data *audiosrv_data;
	struct device *dev = dcp->dev;

	audiosrv_data = devm_kzalloc(dcp->dev, sizeof(*audiosrv_data), GFP_KERNEL);
	audio_pdata = devm_kzalloc(dcp->dev, sizeof(*audio_pdata), GFP_KERNEL);
	if (!audiosrv_data || !audio_pdata)
		return -ENOMEM;
	init_rwsem(&audiosrv_data->srv_rwsem);
	mutex_init(&audiosrv_data->plug_lock);
	dcp->audiosrv = audiosrv_data;

	audio_pdata->dcp_dev = dcp->dev;
	/* TODO: free OF reference */
	audio_pdata->dpaudio_node = \
			of_parse_phandle(dev->of_node, "apple,audio-xmitter", 0);
	if (!audio_pdata->dpaudio_node ||
	    !of_device_is_available(audio_pdata->dpaudio_node)) {
		dev_info(dev, "No audio support\n");
		return 0;
	}

	audio_pdev = platform_device_register_data(dev, "dcp-hdmi-audio",
						   PLATFORM_DEVID_AUTO,
						   audio_pdata, sizeof(*audio_pdata));
	if (IS_ERR(audio_pdev))
		return dev_err_probe(dev, PTR_ERR(audio_pdev), "registering audio device\n");

	dcp->avep = afk_init(dcp, AV_ENDPOINT, avep_ops);
	if (IS_ERR(dcp->avep))
		return PTR_ERR(dcp->avep);
	dcp->avep->debugfs_entry = dcp->ep_debugfs[AV_ENDPOINT - 0x20];
	return afk_start(dcp->avep);
}
