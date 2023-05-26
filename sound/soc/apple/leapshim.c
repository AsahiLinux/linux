// SPDX-License-Identifier: GPL-2.0-only OR MIT
#include <linux/dma-mapping.h>
#include <linux/firmware.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_dma.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/io.h>

#include <sound/core.h>
#include <sound/dmaengine_pcm.h>
#include <sound/soc.h>
#include <sound/soc-topology.h>

#include "leap.h"
#include "leapshim.h"

#define LEAPSHIM_NUM_RX_PDM_CHANNELS 	10
#define LEAPSHIM_RX_PDM_IO_PORT_BASE	0x20

#define REG_LEAPSHIM_DMA_CONTROL(id)		(0x690 + 0x4 * (id))
#define REG_LEAPSHIM_UNK04			0x004
#define LEAPSHIM_UNK04_UNK			BIT(31)
#define REG_LEAPSHIM_PERIPH_PORT_CTL(portno)	(0x188 + 4 * (portno))
#define LEAPSHIM_PERIPH_PORT_EN			BIT(5)
#define REG_LEAPSHIM_PDMCSR 			0x71c

struct leapshim_data {
	struct leap_cluster *leap;
	struct device *dev;
	__iomem void *base;
	const struct firmware *fw;
	void *leap_image;
	size_t leap_image_size;

	/* TODO: technically should be per substream */
	int pcm_nchannels;
};

static char *force_firmware = NULL;
module_param(force_firmware, charp, 0660);

static void leapshim_modify(struct leapshim_data *shim, int regoffset, u32 mask, u32 val)
{
	__iomem void *ptr = shim->base + regoffset;
	u32 newval;

	newval = (val & mask) | (readl_relaxed(ptr) & ~mask);
	writel_relaxed(newval, ptr);
}

static int leapshim_pdm_dai_trigger(struct snd_pcm_substream *substream, int cmd,
								struct snd_soc_dai *dai)
{
	struct leapshim_data *data = snd_soc_dai_get_drvdata(dai);
	int id2 = dai->id / 2;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		leapshim_modify(data, REG_LEAPSHIM_PDMCSR, BIT(id2), BIT(id2));
		break;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		leapshim_modify(data, REG_LEAPSHIM_PDMCSR, BIT(id2), 0);
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static int leapshim_pdm_dai_io_port(struct snd_soc_dai *dai)
{
	return LEAPSHIM_RX_PDM_IO_PORT_BASE + dai->id;
}

static int leapshim_pdm_dai_prepare(struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
	struct leapshim_data *data = snd_soc_dai_get_drvdata(dai);

	leapshim_modify(data, REG_LEAPSHIM_PERIPH_PORT_CTL(leapshim_pdm_dai_io_port(dai)),
			LEAPSHIM_PERIPH_PORT_EN, LEAPSHIM_PERIPH_PORT_EN);

	return 0;
}

static int leapshim_pdm_dai_hw_free(struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
	struct leapshim_data *data = snd_soc_dai_get_drvdata(dai);

	leapshim_modify(data, REG_LEAPSHIM_PERIPH_PORT_CTL(leapshim_pdm_dai_io_port(dai)),
			LEAPSHIM_PERIPH_PORT_EN, 0);

	return 0;
}

static const struct snd_soc_dai_ops leapshim_pdm_ops = {
	.trigger = leapshim_pdm_dai_trigger,
	.prepare = leapshim_pdm_dai_prepare,
	.hw_free = leapshim_pdm_dai_hw_free,
};

static const struct snd_soc_dapm_widget leapshim_dapm_widgets[] = {
	SND_SOC_DAPM_CLOCK_SUPPLY("clk_pdm"),
};

static const struct snd_soc_dapm_route leapshim_dapm_routes[] = {
	{"PDM0L RX", NULL, "clk_pdm"},
	{"PDM0R RX", NULL, "clk_pdm"},
	{"PDM1L RX", NULL, "clk_pdm"},
	{"PDM1R RX", NULL, "clk_pdm"},
	{"PDM2L RX", NULL, "clk_pdm"},
	{"PDM2R RX", NULL, "clk_pdm"},
	{"PDM3L RX", NULL, "clk_pdm"},
	{"PDM3R RX", NULL, "clk_pdm"},
	{"PDM4L RX", NULL, "clk_pdm"},
	{"PDM4R RX", NULL, "clk_pdm"},
};

bool leap_snd_is_dma_rtd(struct snd_soc_pcm_runtime *rtd)
{
	return !rtd->dai_link->no_pcm;
}

bool leap_snd_is_pdm_rtd(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_dai *dai;
	int i;

	for_each_rtd_codec_dais(rtd, i, dai)
		if (dai->driver->ops == &leapshim_pdm_ops)
			return true;

	return false;
}

static int leap_snd_hw_params(struct snd_soc_component *component,
			      struct snd_pcm_substream *substream,
			      struct snd_pcm_hw_params *params)
{
	struct leapshim_data *data = snd_soc_component_get_drvdata(component);
	struct snd_soc_pcm_runtime *rtd = asoc_substream_to_rtd(substream);
	struct dma_chan *chan = snd_dmaengine_pcm_get_chan(substream);
	struct dma_slave_config slave_config;
	int ret;

	if (!leap_snd_is_dma_rtd(rtd))
		return 0;

	memset(&slave_config, 0, sizeof(slave_config));
	ret = snd_hwparams_to_dma_slave_config(substream, params,
					       &slave_config);
	if (ret < 0)
		return ret;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		slave_config.dst_port_window_size =
			min_t(u32, params_channels(params), 4);
	else
		slave_config.src_port_window_size =
			min_t(u32, params_channels(params), 4);

	ret = dmaengine_slave_config(chan, &slave_config);

	if (ret == -EINVAL && params_channels(params) != 1) {
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			slave_config.dst_port_window_size = 1;
		else
			slave_config.src_port_window_size = 1;

		ret = dmaengine_slave_config(chan, &slave_config);
	}

	data->pcm_nchannels = params_channels(params);

	return ret;
}

static int leap_snd_prepare(struct snd_soc_component *component,
				struct snd_pcm_substream *substream)
{
	struct leapshim_data *data = snd_soc_component_get_drvdata(component);
	struct snd_soc_pcm_runtime *rtd = asoc_substream_to_rtd(substream);

	if (leap_snd_is_dma_rtd(rtd)) {
		leap_load_image(leap_get_core(data->leap, 1),
						data->leap_image, data->leap_image_size, true);
		leap_enable(leap_get_core(data->leap, 1));
	}

	if (leap_snd_is_pdm_rtd(rtd))
		leapshim_modify(data, REG_LEAPSHIM_UNK04, LEAPSHIM_UNK04_UNK,
				LEAPSHIM_UNK04_UNK);

	return 0;
}

static int leap_snd_hw_free(struct snd_soc_component *component,
			    struct snd_pcm_substream *substream)
{
	struct leapshim_data *data = snd_soc_component_get_drvdata(component);
	struct snd_soc_pcm_runtime *rtd = asoc_substream_to_rtd(substream);

	if (leap_snd_is_dma_rtd(rtd))
		leap_disable(leap_get_core(data->leap, 1));

	if (leap_snd_is_pdm_rtd(rtd))
		leapshim_modify(data, REG_LEAPSHIM_UNK04, LEAPSHIM_UNK04_UNK, 0);

	return 0;
}

static int leap_snd_dma_set_runtime_hwparams(struct snd_soc_component *component,
					     struct snd_pcm_substream *substream,
					     struct dma_chan *chan)
{
	struct device *dma_dev = chan->device->dev;
	struct snd_dmaengine_dai_dma_data dma_data = {};
	struct snd_pcm_hardware hw = {
		.info = SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_MMAP_VALID |
			SNDRV_PCM_INFO_INTERLEAVED,
		.periods_min = 2,
		.periods_max = UINT_MAX,
		.period_bytes_min = 256,
		.period_bytes_max = dma_get_max_seg_size(dma_dev),
		.buffer_bytes_max = SIZE_MAX,
		.fifo_size = 16,
	};
	int ret;

	ret = snd_dmaengine_pcm_refine_runtime_hwparams(substream, &dma_data,
							&hw, chan);
	if (ret)
		return ret;

	return snd_soc_set_runtime_hwparams(substream, &hw);
}


static int leap_snd_pcm_open(struct snd_soc_component *component,
			     struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = asoc_substream_to_rtd(substream);
	struct snd_soc_dai *dai = asoc_rtd_to_cpu(rtd, 0);
	struct dma_chan *chan = dai->stream[substream->stream].dma_data;
	int ret;

	if (!leap_snd_is_dma_rtd(rtd))
		return 0;

	ret = leap_snd_dma_set_runtime_hwparams(component, substream, chan);
	if (ret)
		return ret;

	return snd_dmaengine_pcm_open(substream, chan);
}

static int leap_snd_pcm_close(struct snd_soc_component *component,
			      struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = asoc_substream_to_rtd(substream);

	if (!leap_snd_is_dma_rtd(rtd))
		return 0;

	return snd_dmaengine_pcm_close(substream);
}

static int leap_snd_trigger(struct snd_soc_component *component,
			    struct snd_pcm_substream *substream, int cmd)
{
	struct snd_soc_pcm_runtime *rtd = asoc_substream_to_rtd(substream);
	struct leapshim_data *data = snd_soc_component_get_drvdata(component);
	u32 takeout;
	int ret;

	if (!leap_snd_is_dma_rtd(rtd))
		return 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		ret = snd_dmaengine_pcm_trigger(substream, cmd);
		if (ret)
			return ret;
		leap_io_put(data->leap, 0x60, 0x80000000 | data->pcm_nchannels);
		return 0;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		leap_io_update(data->leap, 0x60, 0);
		leap_io_take(data->leap, 0x60, &takeout);
		ret = snd_dmaengine_pcm_trigger(substream, cmd);
		if (ret)
			return ret;
		return 0;

	default:
		return snd_dmaengine_pcm_trigger(substream, cmd);
	}
}

static snd_pcm_uframes_t leap_snd_pointer(struct snd_soc_component *component,
					  struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = asoc_substream_to_rtd(substream);

	if (!leap_snd_is_dma_rtd(rtd))
		return -ENOTSUPP;

	return snd_dmaengine_pcm_pointer(substream);
}

static void leap_snd_pcm_free(struct snd_soc_component *component,
			      struct snd_pcm *pcm)
{
	struct leapshim_data *data = snd_soc_component_get_drvdata(component);
	struct snd_soc_pcm_runtime *rtd = snd_pcm_chip(pcm);
	struct snd_soc_dai *dai = asoc_rtd_to_cpu(rtd, 0);
	int i;

	if (!leap_snd_is_dma_rtd(rtd))
		return;

	for_each_pcm_streams(i) {
		struct snd_pcm_substream *substream =
			rtd->pcm->streams[i].substream;
		struct dma_chan *chan = dai->stream[i].dma_data;

		if (!substream)
			continue;

		leapshim_disable_dma(data, dai->id);

		if (!chan)
			continue;

		dma_release_channel(chan);
		dai->stream[i].dma_data = NULL;
	}
}

static int leap_snd_pcm_new(struct snd_soc_component *component,
			    struct snd_soc_pcm_runtime *rtd)
{
	struct leapshim_data *data = snd_soc_component_get_drvdata(component);
	struct snd_soc_dai *dai = asoc_rtd_to_cpu(rtd, 0);
	int i;

	if (!leap_snd_is_dma_rtd(rtd))
		return 0;

	for_each_pcm_streams(i) {
		struct snd_pcm_substream *substream =
			rtd->pcm->streams[i].substream;
		struct dma_chan *chan;

		if (!substream)
			continue;

		leapshim_enable_dma(data, dai->id);

		chan = leapshim_request_dma_channel(data, dai->id);
		if (IS_ERR_OR_NULL(chan)) {
			dev_err(data->dev, "dai %d: failed to obtain DMA channel: %pe\n",
				dai->id, chan);
			leap_snd_pcm_free(component, rtd->pcm);
			return -EINVAL;
		}

		dai->stream[i].dma_data = chan;

		snd_pcm_set_managed_buffer(substream, SNDRV_DMA_TYPE_DEV_IRAM,
					   chan->device->dev, 512 * 1024, SIZE_MAX);
	}

	return 0;
}

static const struct firmware *leap_snd_request_firmware(struct snd_soc_component *component)
{
	struct leapshim_data *data = snd_soc_component_get_drvdata(component);
	const struct firmware *fw;
	char filename[64];
	int ret;

	if (force_firmware) {
		ret = request_firmware(&fw, force_firmware, data->dev);

		if (ret)
			return NULL;

		return fw;
	}

	snprintf(filename, sizeof(filename), "leapmic-j%s.bin",
		 component->card->name + strlen(component->card->name) - 3);

	ret = request_firmware(&fw, filename, data->dev);

	if (ret) {
		dev_warn(data->dev, "requesting topology firmware '%s': %d\n", filename, ret);
		return NULL;
	}

	return fw;
}

static int leap_snd_tplg_manifest(struct snd_soc_component *component, int index,
				  struct snd_soc_tplg_manifest *manifest)
{
	struct leapshim_data *data = snd_soc_component_get_drvdata(component);
	u32 size = le32_to_cpu(manifest->priv.size);
	void *image = manifest->priv.data;

	data->leap_image = image;
	data->leap_image_size = size;

	leap_load_image(leap_get_core(data->leap, 1), image, size, false);

	return 0;
}

static struct snd_soc_tplg_ops leap_snd_tplg_ops = {
	.manifest = leap_snd_tplg_manifest,
};

static int leap_snd_component_probe(struct snd_soc_component *component)
{
	struct leapshim_data *data = snd_soc_component_get_drvdata(component);
	const struct firmware *fw;
	int ret;

	fw = leap_snd_request_firmware(component);
	if (!fw) {
		dev_warn(data->dev, "internal mics unavailable due to missing topology firmware\n");
		return 0;
	}

	ret = snd_soc_tplg_component_load(component, &leap_snd_tplg_ops, fw);
	if (ret < 0) {
		release_firmware(fw);
		return ret;
	}

	data->fw = fw;
	return 0;
}

static void leap_snd_component_remove(struct snd_soc_component *component)
{
	struct leapshim_data *data = snd_soc_component_get_drvdata(component);

	snd_soc_tplg_component_remove(component);

	if (data->fw) {
		release_firmware(data->fw);
		data->fw = NULL;
	}
}

static unsigned int leap_snd_read(struct snd_soc_component *component, unsigned int reg)
{
	struct leapshim_data *data = snd_soc_component_get_drvdata(component);
	unsigned int takeout = 0;
	int ret;

	ret = leap_io_peek(data->leap, reg, &takeout);

	if (ret < 0)
		dev_err(component->dev, "read of %x: %pE\n", reg, ERR_PTR(ret));

	return takeout;
}

static int leap_snd_write(struct snd_soc_component *component, unsigned int reg,
			  unsigned int val)
{
	struct leapshim_data *data = snd_soc_component_get_drvdata(component);

	return leap_io_put(data->leap, reg, val);
}

static const struct snd_soc_component_driver leap_snd_component = {
	.name = "apple-leapshim",
	.dapm_widgets = leapshim_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(leapshim_dapm_widgets),
	.dapm_routes = leapshim_dapm_routes,
	.num_dapm_routes = ARRAY_SIZE(leapshim_dapm_routes),
	.read = leap_snd_read,
	.write = leap_snd_write,
	.probe = leap_snd_component_probe,
	.remove = leap_snd_component_remove,
	.open = leap_snd_pcm_open,
	.close = leap_snd_pcm_close,
	.pcm_construct = leap_snd_pcm_new,
	.pcm_destruct = leap_snd_pcm_free,
	.trigger = leap_snd_trigger,
	.pointer = leap_snd_pointer,
	.prepare = leap_snd_prepare,
	.hw_free = leap_snd_hw_free,
	.hw_params = leap_snd_hw_params,
};

struct dma_chan *leapshim_request_dma_channel(struct leapshim_data *shim, int id)
{
	struct device_node *node = shim->dev->of_node;
	const char *name;

	if (of_property_read_string_index(node, "dma-names", id, &name))
		return ERR_PTR(-EINVAL);

	return of_dma_request_slave_channel(node, name);
}

void leapshim_enable_dma(struct leapshim_data *shim, int id)
{
	writel_relaxed(0x42, shim->base + REG_LEAPSHIM_DMA_CONTROL(id)); /* TODO */
}

void leapshim_disable_dma(struct leapshim_data *shim, int id)
{
	writel_relaxed(0x0, shim->base + REG_LEAPSHIM_DMA_CONTROL(id));
}

static const struct of_device_id apple_leap_of_match[] = {
	{ .compatible = "apple,leap", },
	{}
};

struct platform_device *leapshim_find_leap_device(struct leapshim_data *shim)
{
	struct platform_device *pdev = NULL;
	struct device_node *np;

	for_each_matching_node(np, apple_leap_of_match) {
		if (pdev) {
			/*
			 * We expect a single LEAP instance on the chip, which is the one
			 * we can assume we are paired with. If there ever will be a chip
			 * with multiple LEAP instances, we can expand the binding and
			 * the logic here.
			 */
			dev_err(shim->dev, "multiple LEAP instances found, that's unsupported\n");
			return NULL;
		}

		pdev = of_find_device_by_node(np);
	}

	return pdev;
}

static void leapshim_release(struct leapshim_data *shim)
{
	snd_soc_unregister_component(shim->dev);
}

static int leapshim_link_to_leap(struct platform_device *pdev)
{
	struct leapshim_data *shim = platform_get_drvdata(pdev);
	struct platform_device *leap_pdev;
	struct device_link *leap_link;

	leap_pdev = leapshim_find_leap_device(shim);
	if (!leap_pdev)
		return -ENODEV;

	leap_link = device_link_add(&pdev->dev, &leap_pdev->dev,
				    DL_FLAG_AUTOREMOVE_CONSUMER);
	if (!leap_link)
		return dev_err_probe(&pdev->dev, -EINVAL, "failed to link to LEAP\n");

	if (leap_link->supplier->links.status != DL_DEV_DRIVER_BOUND)
		return -EPROBE_DEFER;

	shim->leap = platform_get_drvdata(leap_pdev);

	return 0;
}

static int leapshim_probe(struct platform_device *pdev)
{
	struct snd_soc_dai_driver *dai_drivers;
	struct leapshim_data *shim;
	int ret, i;

	shim = devm_kzalloc(&pdev->dev, sizeof(*shim), GFP_KERNEL);
	if (!shim)
		return -ENOMEM;
	shim->dev = &pdev->dev;
	shim->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(shim->base))
		return PTR_ERR(shim->base);
	platform_set_drvdata(pdev, shim);

	ret = leapshim_link_to_leap(pdev);
	if (ret)
		return ret;

	dai_drivers = devm_kzalloc(
		&pdev->dev, sizeof(*dai_drivers) * LEAPSHIM_NUM_RX_PDM_CHANNELS, GFP_KERNEL);
	if (!dai_drivers)
		return -ENOMEM;

	for (i = 0; i < LEAPSHIM_NUM_RX_PDM_CHANNELS; i++) {
		struct snd_soc_dai_driver *drv = &dai_drivers[i];

		drv->id = i;
		drv->name = devm_kasprintf(&pdev->dev, GFP_KERNEL, "leapshim-pdm-%d-%s", 
					   i / 2, (i % 2 == 0 ? "left" : "right"));
		if (!drv->name)
			return -ENOMEM;

		drv->ops = &leapshim_pdm_ops;
		drv->capture.channels_min = 1;
		drv->capture.channels_max = 1;
		drv->capture.rates = SNDRV_PCM_RATE_8000_192000;
		drv->capture.formats = SNDRV_PCM_FMTBIT_SPECIAL | SNDRV_PCM_FMTBIT_FLOAT_LE; /* TODO: n/a */

		drv->capture.stream_name =
			devm_kasprintf(&pdev->dev, GFP_KERNEL, "PDM%d%s RX", i / 2,
				       (i % 2 == 0 ? "L" : "R"));

		if (!drv->capture.stream_name)
			return -ENOMEM;
	}

	ret = snd_soc_register_component(&pdev->dev, &leap_snd_component,
					 dai_drivers, LEAPSHIM_NUM_RX_PDM_CHANNELS);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "unable to register ASoC component");

	return 0;
}

static int leapshim_remove(struct platform_device *pdev)
{
	struct leapshim_data *shim = platform_get_drvdata(pdev);

	leapshim_release(shim);
	return 0;
}

static const struct of_device_id apple_leapshim_of_match[] = {
	{ .compatible = "apple,leapshim", },
	{}
};
MODULE_DEVICE_TABLE(of, apple_leapshim_of_match);

static struct platform_driver apple_leapshim_driver = {
	.driver = {
		.name = "apple-leapshim",
		.of_match_table = apple_leapshim_of_match,
	},
	.probe = leapshim_probe,
	.remove = leapshim_remove,
};
module_platform_driver(apple_leapshim_driver);

MODULE_AUTHOR("Martin Povišer <povik+lin@cutebit.org>");
MODULE_DESCRIPTION("Apple LEAP shim driver");
MODULE_LICENSE("Dual MIT/GPL");
