// SPDX-License-Identifier: GPL-2.0-only
/*
 * ASoC machine driver for Apple Silicon Macs
 *
 * Copyright (C) The Asahi Linux Contributors
 *
 * Based on sound/soc/qcom/{sc7180.c|common.c}
 * Copyright (c) 2018, Linaro Limited.
 * Copyright (c) 2020, The Linux Foundation. All rights reserved.
 *
 *
 * The platform driver has independent frontend and backend DAIs with the
 * option of routing backends to any of the frontends. The platform
 * driver configures the routing based on DPCM couplings in ASoC runtime
 * structures, which in turn are determined from DAPM paths by ASoC. But the
 * platform driver doesn't supply relevant DAPM paths and leaves that up for
 * the machine driver to fill in. The filled-in virtual topology can be
 * anything as long as any backend isn't connected to more than one frontend
 * at any given time. (The limitation is due to the unsupported case of
 * reparenting of live BEs.)
 */

#define DEBUG

#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/jack.h>
#include <sound/pcm.h>
#include <sound/simple_card_utils.h>
#include <sound/soc.h>
#include <sound/soc-jack.h>
#include <uapi/linux/input-event-codes.h>

#define DRIVER_NAME "snd-soc-macaudio"

/*
 * CPU side is bit and frame clock provider
 * I2S has both clocks inverted
 */
#define MACAUDIO_DAI_FMT	(SND_SOC_DAIFMT_I2S | \
				 SND_SOC_DAIFMT_CBC_CFC | \
				 SND_SOC_DAIFMT_GATED | \
				 SND_SOC_DAIFMT_IB_IF)
#define MACAUDIO_JACK_MASK	(SND_JACK_HEADSET | SND_JACK_HEADPHONE)
#define MACAUDIO_SLOTWIDTH	32

struct macaudio_snd_data {
	struct snd_soc_card card;
	struct snd_soc_jack jack;
	int jack_plugin_state;

	bool has_speakers;
	unsigned int max_channels;

	struct macaudio_link_props {
		/* frontend props */
		unsigned int bclk_ratio;

		/* backend props */
		bool is_speakers;
		bool is_headphones;
		unsigned int tdm_mask;
	} *link_props;

	unsigned int speaker_nchans_array[2];
	struct snd_pcm_hw_constraint_list speaker_nchans_list;
};

static bool please_blow_up_my_speakers;
module_param(please_blow_up_my_speakers, bool, 0644);
MODULE_PARM_DESC(please_blow_up_my_speakers, "Allow unsafe or untested operating configurations");

SND_SOC_DAILINK_DEFS(primary,
	DAILINK_COMP_ARRAY(COMP_CPU("mca-pcm-0")), // CPU
	DAILINK_COMP_ARRAY(COMP_DUMMY()), // CODEC
	DAILINK_COMP_ARRAY(COMP_EMPTY())); // platform (filled at runtime)

SND_SOC_DAILINK_DEFS(secondary,
	DAILINK_COMP_ARRAY(COMP_CPU("mca-pcm-1")), // CPU
	DAILINK_COMP_ARRAY(COMP_DUMMY()), // CODEC
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

static struct snd_soc_dai_link macaudio_fe_links[] = {
	{
		.name = "Primary",
		.stream_name = "Primary",
		.dynamic = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		.dpcm_merged_rate = 1,
		.dpcm_merged_chan = 1,
		.dpcm_merged_format = 1,
		.dai_fmt = MACAUDIO_DAI_FMT,
		SND_SOC_DAILINK_REG(primary),
	},
	{
		.name = "Secondary",
		.stream_name = "Secondary",
		.dynamic = 1,
		.dpcm_playback = 1,
		.dpcm_merged_rate = 1,
		.dpcm_merged_chan = 1,
		.dpcm_merged_format = 1,
		.dai_fmt = MACAUDIO_DAI_FMT,
		SND_SOC_DAILINK_REG(secondary),
	},
};

static struct macaudio_link_props macaudio_fe_link_props[] = {
	{
		/*
		 * Primary FE
		 *
		 * The bclk ratio at 64 for the primary frontend is important
		 * to ensure that the headphones codec's idea of left and right
		 * in a stereo stream over I2S fits in nicely with everyone else's.
		 * (This is until the headphones codec's driver supports
		 * set_tdm_slot.)
		 *
		 * The low bclk ratio precludes transmitting more than two
		 * channels over I2S, but that's okay since there is the secondary
		 * FE for speaker arrays anyway.
		 */
		.bclk_ratio = 64,
	},
	{
		/*
		 * Secondary FE
		 *
		 * Here we want frames plenty long to be able to drive all
		 * those fancy speaker arrays.
		 */
		.bclk_ratio = 256,
	}
};

static int macaudio_copy_link(struct device *dev, struct snd_soc_dai_link *target,
			       struct snd_soc_dai_link *source)
{
	memcpy(target, source, sizeof(struct snd_soc_dai_link));

	target->cpus = devm_kmemdup(dev, target->cpus,
				sizeof(*target->cpus) * target->num_cpus,
				GFP_KERNEL);
	target->codecs = devm_kmemdup(dev, target->codecs,
				sizeof(*target->codecs) * target->num_codecs,
				GFP_KERNEL);
	target->platforms = devm_kmemdup(dev, target->platforms,
				sizeof(*target->platforms) * target->num_platforms,
				GFP_KERNEL);

	if (!target->cpus || !target->codecs || !target->platforms)
		return -ENOMEM;

	return 0;
}

static int macaudio_parse_of_component(struct device_node *node, int index,
				struct snd_soc_dai_link_component *comp)
{
	struct of_phandle_args args;
	int ret;

	ret = of_parse_phandle_with_args(node, "sound-dai", "#sound-dai-cells",
						index, &args);
	if (ret)
		return ret;
	comp->of_node = args.np;
	return snd_soc_get_dai_name(&args, &comp->dai_name);
}

/*
 * Parse one DPCM backend from the devicetree. This means taking one
 * of the CPU DAIs and combining it with one or more CODEC DAIs.
 */
static int macaudio_parse_of_be_dai_link(struct macaudio_snd_data *ma,
				struct snd_soc_dai_link *link,
				int be_index, int ncodecs_per_be,
				struct device_node *cpu,
				struct device_node *codec)
{
	struct snd_soc_dai_link_component *comp;
	struct device *dev = ma->card.dev;
	int codec_base = be_index * ncodecs_per_be;
	int ret, i;

	link->no_pcm = 1;
	link->dpcm_playback = 1;
	link->dpcm_capture = 1;

	link->dai_fmt = MACAUDIO_DAI_FMT;

	link->num_codecs = ncodecs_per_be;
	link->codecs = devm_kcalloc(dev, ncodecs_per_be,
				    sizeof(*comp), GFP_KERNEL);
	link->num_cpus = 1;
	link->cpus = devm_kzalloc(dev, sizeof(*comp), GFP_KERNEL);

	if (!link->codecs || !link->cpus)
		return -ENOMEM;

	link->num_platforms = 0;

	for_each_link_codecs(link, i, comp) {
		ret = macaudio_parse_of_component(codec, codec_base + i, comp);
		if (ret)
			return ret;
	}

	ret = macaudio_parse_of_component(cpu, be_index, link->cpus);
	if (ret)
		return ret;

	link->name = link->cpus[0].dai_name;

	return 0;
}

static int macaudio_parse_of(struct macaudio_snd_data *ma)
{
	struct device_node *codec = NULL;
	struct device_node *cpu = NULL;
	struct device_node *np = NULL;
	struct device_node *platform = NULL;
	struct snd_soc_dai_link *link = NULL;
	struct snd_soc_card *card = &ma->card;
	struct device *dev = card->dev;
	struct macaudio_link_props *link_props;
	int ret, num_links, i;

	ret = snd_soc_of_parse_card_name(card, "model");
	if (ret) {
		dev_err(dev, "Error parsing card name: %d\n", ret);
		return ret;
	}

	/* Populate links, start with the fixed number of FE links */
	num_links = ARRAY_SIZE(macaudio_fe_links);

	/* Now add together the (dynamic) number of BE links */
	for_each_available_child_of_node(dev->of_node, np) {
		int num_cpus;

		cpu = of_get_child_by_name(np, "cpu");
		if (!cpu) {
			dev_err(dev, "missing CPU DAI node at %pOF\n", np);
			ret = -EINVAL;
			goto err_free;
		}

		num_cpus = of_count_phandle_with_args(cpu, "sound-dai",
						"#sound-dai-cells");

		if (num_cpus <= 0) {
			dev_err(card->dev, "missing sound-dai property at %pOF\n", cpu);
			ret = -EINVAL;
			goto err_free;
		}
		of_node_put(cpu);
		cpu = NULL;

		/* Each CPU specified counts as one BE link */
		num_links += num_cpus;
	}

	/* Allocate the DAI link array */
	card->dai_link = devm_kcalloc(dev, num_links, sizeof(*link), GFP_KERNEL);
	ma->link_props = devm_kcalloc(dev, num_links, sizeof(*ma->link_props), GFP_KERNEL);
	if (!card->dai_link || !ma->link_props)
		return -ENOMEM;

	card->num_links = num_links;
	link = card->dai_link;
	link_props = ma->link_props;

	for (i = 0; i < ARRAY_SIZE(macaudio_fe_links); i++) {
		ret = macaudio_copy_link(dev, link, &macaudio_fe_links[i]);
		if (ret)
			goto err_free;

		memcpy(link_props, &macaudio_fe_link_props[i], sizeof(struct macaudio_link_props));
		link++; link_props++;
	}

	for (i = 0; i < num_links; i++)
		card->dai_link[i].id = i;

	/* Fill in the BEs */
	for_each_available_child_of_node(dev->of_node, np) {
		const char *link_name;
		bool speakers;
		int be_index, num_codecs, num_bes, ncodecs_per_cpu, nchannels;
		unsigned int left_mask, right_mask;

		ret = of_property_read_string(np, "link-name", &link_name);
		if (ret) {
			dev_err(card->dev, "missing link name\n");
			goto err_free;
		}

		speakers = !strcmp(link_name, "Speaker")
			   || !strcmp(link_name, "Speakers");
		if (speakers)
			ma->has_speakers = 1;

		cpu = of_get_child_by_name(np, "cpu");
		codec = of_get_child_by_name(np, "codec");

		if (!codec || !cpu) {
			dev_err(dev, "missing DAI specifications for '%s'\n", link_name);
			ret = -EINVAL;
			goto err_free;
		}

		num_bes = of_count_phandle_with_args(cpu, "sound-dai",
						     "#sound-dai-cells");
		if (num_bes <= 0) {
			dev_err(card->dev, "missing sound-dai property at %pOF\n", cpu);
			ret = -EINVAL;
			goto err_free;
		}

		num_codecs = of_count_phandle_with_args(codec, "sound-dai",
							"#sound-dai-cells");
		if (num_codecs <= 0) {
			dev_err(card->dev, "missing sound-dai property at %pOF\n", codec);
			ret = -EINVAL;
			goto err_free;
		}

		if (num_codecs % num_bes != 0) {
			dev_err(card->dev, "bad combination of CODEC (%d) and CPU (%d) number at %pOF\n",
				num_codecs, num_bes, np);
			ret = -EINVAL;
			goto err_free;
		}

		/*
		 * Now parse the cpu/codec lists into a number of DPCM backend links.
		 * In each link there will be one DAI from the cpu list paired with
		 * an evenly distributed number of DAIs from the codec list. (As is
		 * the binding semantics.)
		 */
		ncodecs_per_cpu = num_codecs / num_bes;
		nchannels = num_codecs * (speakers ? 1 : 2);

		/* Save the max number of channels on the platform */
		if (nchannels > ma->max_channels)
			ma->max_channels = nchannels;

		/*
		 * If there is a single speaker, assign two channels to it, because
		 * it can do downmix.
		 */
		if (nchannels < 2)
			nchannels = 2;

		left_mask = 0;
		for (i = 0; i < nchannels; i += 2)
			left_mask = left_mask << 2 | 1;
		right_mask = left_mask << 1;

		for (be_index = 0; be_index < num_bes; be_index++) {
			ret = macaudio_parse_of_be_dai_link(ma, link, be_index,
							    ncodecs_per_cpu, cpu, codec);
			if (ret)
				goto err_free;

			link_props->is_speakers = speakers;
			link_props->is_headphones = !speakers;

			if (num_bes == 2)
				/* This sound peripheral is split between left and right BE */
				link_props->tdm_mask = be_index ? right_mask : left_mask;
			else
				/* One BE covers all of the peripheral */
				link_props->tdm_mask = left_mask | right_mask;

			/* Steal platform OF reference for use in FE links later */
			platform = link->cpus->of_node;

			link++; link_props++;
		}

		of_node_put(codec);
		of_node_put(cpu);
		cpu = codec = NULL;
	}

	for (i = 0; i < ARRAY_SIZE(macaudio_fe_links); i++)
		card->dai_link[i].platforms->of_node = platform;

	return 0;

err_free:
	of_node_put(codec);
	of_node_put(cpu);
	of_node_put(np);

	if (!card->dai_link)
		return ret;

	for (i = 0; i < num_links; i++) {
		/*
		 * TODO: If we don't go through this path are the references
		 * freed inside ASoC?
		 */
		snd_soc_of_put_dai_link_codecs(&card->dai_link[i]);
		snd_soc_of_put_dai_link_cpus(&card->dai_link[i]);
	}

	return ret;
}

static int macaudio_get_runtime_bclk_ratio(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = asoc_substream_to_rtd(substream);
	struct macaudio_snd_data *ma = snd_soc_card_get_drvdata(rtd->card);
	struct snd_soc_dpcm *dpcm;

	/*
	 * If this is a FE, look it up in link_props directly.
	 * If this is a BE, look it up in the respective FE.
	 */
	if (!rtd->dai_link->no_pcm)
		return ma->link_props[rtd->dai_link->id].bclk_ratio;

	for_each_dpcm_fe(rtd, substream->stream, dpcm) {
		int fe_id = dpcm->fe->dai_link->id;

		return ma->link_props[fe_id].bclk_ratio;
	}

	return 0;
}

static int macaudio_dpcm_hw_params(struct snd_pcm_substream *substream,
				   struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = asoc_substream_to_rtd(substream);
	struct snd_soc_dai *cpu_dai = asoc_rtd_to_cpu(rtd, 0);
	int bclk_ratio = macaudio_get_runtime_bclk_ratio(substream);
	int i;

	if (bclk_ratio) {
		struct snd_soc_dai *dai;
		int mclk = params_rate(params) * bclk_ratio;

		for_each_rtd_codec_dais(rtd, i, dai) {
			snd_soc_dai_set_sysclk(dai, 0, mclk, SND_SOC_CLOCK_IN);
			snd_soc_dai_set_bclk_ratio(dai, bclk_ratio);
		}

		snd_soc_dai_set_sysclk(cpu_dai, 0, mclk, SND_SOC_CLOCK_OUT);
		snd_soc_dai_set_bclk_ratio(cpu_dai, bclk_ratio);
	}

	return 0;
}

static int macaudio_fe_startup(struct snd_pcm_substream *substream)
{

	struct snd_soc_pcm_runtime *rtd = asoc_substream_to_rtd(substream);
	struct macaudio_snd_data *ma = snd_soc_card_get_drvdata(rtd->card);
	int ret;

	/* The FEs must never have more channels than the hardware */
	ret = snd_pcm_hw_constraint_minmax(substream->runtime,
					SNDRV_PCM_HW_PARAM_CHANNELS, 0, ma->max_channels);

	if (ret < 0) {
		dev_err(rtd->dev, "Failed to constrain FE %d! %d", rtd->dai_link->id, ret);
		return ret;
		}

	return 0;
}

static int macaudio_fe_hw_params(struct snd_pcm_substream *substream,
				   struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = asoc_substream_to_rtd(substream);
	struct snd_soc_pcm_runtime *be;
	struct snd_soc_dpcm *dpcm;

	be = NULL;
	for_each_dpcm_be(rtd, substream->stream, dpcm) {
		be = dpcm->be;
		break;
	}

	if (!be) {
		dev_err(rtd->dev, "opening PCM device '%s' with no audio route configured (bad settings applied to the sound card)\n",
				rtd->dai_link->name);
		return -EINVAL;
	}

	return macaudio_dpcm_hw_params(substream, params);
}


static void macaudio_dpcm_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = asoc_substream_to_rtd(substream);
	struct snd_soc_dai *cpu_dai = asoc_rtd_to_cpu(rtd, 0);
	struct snd_soc_dai *dai;
	int bclk_ratio = macaudio_get_runtime_bclk_ratio(substream);
	int i;

	if (bclk_ratio) {
		for_each_rtd_codec_dais(rtd, i, dai)
			snd_soc_dai_set_sysclk(dai, 0, 0, SND_SOC_CLOCK_IN);

		snd_soc_dai_set_sysclk(cpu_dai, 0, 0, SND_SOC_CLOCK_OUT);
	}
}

static const struct snd_soc_ops macaudio_fe_ops = {
	.startup	= macaudio_fe_startup,
	.shutdown	= macaudio_dpcm_shutdown,
	.hw_params	= macaudio_fe_hw_params,
};

static const struct snd_soc_ops macaudio_be_ops = {
	.shutdown	= macaudio_dpcm_shutdown,
	.hw_params	= macaudio_dpcm_hw_params,
};

static int macaudio_be_assign_tdm(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_card *card = rtd->card;
	struct macaudio_snd_data *ma = snd_soc_card_get_drvdata(card);
	struct macaudio_link_props *props = &ma->link_props[rtd->dai_link->id];
	struct snd_soc_dai *dai;
	unsigned int mask;
	int nslots, ret, i;

	if (!props->tdm_mask)
		return 0;

	mask = props->tdm_mask;
	nslots = __fls(mask) + 1;

	if (rtd->dai_link->num_codecs == 1) {
		ret = snd_soc_dai_set_tdm_slot(asoc_rtd_to_codec(rtd, 0), mask,
					       0, nslots, MACAUDIO_SLOTWIDTH);

		/*
		 * Headphones get a pass on -ENOTSUPP (see the comment
		 * around bclk_ratio value for primary FE).
		 */
		if (ret == -ENOTSUPP && props->is_headphones)
			return 0;

		return ret;
	}

	for_each_rtd_codec_dais(rtd, i, dai) {
		int slot = __ffs(mask);

		mask &= ~(1 << slot);
		ret = snd_soc_dai_set_tdm_slot(dai, 1 << slot, 0, nslots,
					       MACAUDIO_SLOTWIDTH);
		if (ret)
			return ret;
	}

	return 0;
}

static int macaudio_be_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_card *card = rtd->card;
	struct macaudio_snd_data *ma = snd_soc_card_get_drvdata(card);
	struct macaudio_link_props *props = &ma->link_props[rtd->dai_link->id];
	struct snd_soc_dai *dai;
	int i, ret;

	ret = macaudio_be_assign_tdm(rtd);
	if (ret < 0)
		return ret;

	if (props->is_headphones) {
		for_each_rtd_codec_dais(rtd, i, dai)
			snd_soc_component_set_jack(dai->component, &ma->jack, NULL);
	}

	return 0;
}

static void macaudio_be_exit(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_card *card = rtd->card;
	struct macaudio_snd_data *ma = snd_soc_card_get_drvdata(card);
	struct macaudio_link_props *props = &ma->link_props[rtd->dai_link->id];
	struct snd_soc_dai *dai;
	int i;

	if (props->is_headphones) {
		for_each_rtd_codec_dais(rtd, i, dai)
			snd_soc_component_set_jack(dai->component, NULL, NULL);
	}
}

static int macaudio_fe_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_card *card = rtd->card;
	struct macaudio_snd_data *ma = snd_soc_card_get_drvdata(card);
	struct macaudio_link_props *props = &ma->link_props[rtd->dai_link->id];
	int nslots = props->bclk_ratio / MACAUDIO_SLOTWIDTH;

	return snd_soc_dai_set_tdm_slot(asoc_rtd_to_cpu(rtd, 0), (1 << nslots) - 1,
					(1 << nslots) - 1, nslots, MACAUDIO_SLOTWIDTH);
}

static struct snd_soc_jack_pin macaudio_jack_pins[] = {
	{
		.pin = "Headphone",
		.mask = SND_JACK_HEADPHONE,
	},
	{
		.pin = "Headset Mic",
		.mask = SND_JACK_MICROPHONE,
	},
};

static int macaudio_probe(struct snd_soc_card *card)
{
	struct macaudio_snd_data *ma = snd_soc_card_get_drvdata(card);
	int ret;

	dev_dbg(card->dev, "%s!\n", __func__);

	ret = snd_soc_card_jack_new_pins(card, "Headphone Jack",
			SND_JACK_HEADSET | SND_JACK_HEADPHONE,
			&ma->jack, macaudio_jack_pins,
			ARRAY_SIZE(macaudio_jack_pins));
	if (ret < 0) {
		dev_err(card->dev, "jack creation failed: %d\n", ret);
		return ret;
	}

	return ret;
}

static int macaudio_add_backend_dai_route(struct snd_soc_card *card, struct snd_soc_dai *dai,
					  bool is_speakers)
{
	struct snd_soc_dapm_route routes[2];
	struct snd_soc_dapm_route *r;
	int nroutes = 0;
	int ret;

	memset(routes, 0, sizeof(routes));

	dev_dbg(card->dev, "adding routes for '%s'\n", dai->name);

	r = &routes[nroutes++];
	if (is_speakers)
		r->source = "Speaker Playback";
	else
		r->source = "Headphone Playback";
	r->sink = dai->stream[SNDRV_PCM_STREAM_PLAYBACK].widget->name;

	/* If headphone jack, add capture path */
	if (!is_speakers) {
		r = &routes[nroutes++];
		r->source = dai->stream[SNDRV_PCM_STREAM_CAPTURE].widget->name;
		r->sink = "Headset Capture";
	}

	ret = snd_soc_dapm_add_routes(&card->dapm, routes, nroutes);
	if (ret)
		dev_err(card->dev, "failed adding dynamic DAPM routes for %s\n",
			dai->name);
	return ret;
}

static int macaudio_add_pin_routes(struct snd_soc_card *card, struct snd_soc_component *component,
				   bool is_speakers)
{
	struct snd_soc_dapm_route routes[2];
	struct snd_soc_dapm_route *r;
	int nroutes = 0;
	char buf[32];
	int ret;

	memset(routes, 0, sizeof(routes));

	/* Connect the far ends of CODECs to pins */
	if (is_speakers) {
		r = &routes[nroutes++];
		r->source = "OUT";
		if (component->name_prefix) {
			snprintf(buf, sizeof(buf) - 1, "%s OUT", component->name_prefix);
			r->source = buf;
		}	
		r->sink = "Speaker";
	} else {
		r = &routes[nroutes++];
		r->source = "Jack HP";
		r->sink = "Headphone";
		r = &routes[nroutes++];
		r->source = "Headset Mic";
		r->sink = "Jack HS";
	}

	ret = snd_soc_dapm_add_routes(&card->dapm, routes, nroutes);
	if (ret)
		dev_err(card->dev, "failed adding dynamic DAPM routes for %s\n",
			component->name);
	return ret;
}

static int macaudio_late_probe(struct snd_soc_card *card)
{
	struct macaudio_snd_data *ma = snd_soc_card_get_drvdata(card);
	struct snd_soc_pcm_runtime *rtd;
	struct snd_soc_dai *dai;
	int ret, i;

	/* Add the dynamic DAPM routes */
	for_each_card_rtds(card, rtd) {
		struct macaudio_link_props *props = &ma->link_props[rtd->dai_link->id];

		if (!rtd->dai_link->no_pcm)
			continue;

		for_each_rtd_cpu_dais(rtd, i, dai) {
			ret = macaudio_add_backend_dai_route(card, dai, props->is_speakers);

			if (ret)
				return ret;
		}

		for_each_rtd_codec_dais(rtd, i, dai) {
			ret = macaudio_add_pin_routes(card, dai->component,
						      props->is_speakers);

			if (ret)
				return ret;
		}
	}

	return 0;
}

#define CHECK(call, pattern, value) \
	{ \
		int ret = call(card, pattern, value); \
		if (ret < 1 && !please_blow_up_my_speakers) { \
			dev_err(card->dev, "%s on '%s': %d\n", #call, pattern, ret); \
			return ret; \
		} \
		dev_dbg(card->dev, "%s on '%s': %d hits\n", #call, pattern, ret); \
	}


static int macaudio_j274_fixup_controls(struct snd_soc_card *card)
{
	struct macaudio_snd_data *ma = snd_soc_card_get_drvdata(card);

	if (ma->has_speakers) {
		CHECK(snd_soc_limit_volume, "* Amp Gain Volume", 14); // 20 set by macOS, this is 3 dB below
	}

	return 0;	
}

static int macaudio_j313_fixup_controls(struct snd_soc_card *card) {
	struct macaudio_snd_data *ma = snd_soc_card_get_drvdata(card);

	if (ma->has_speakers) {
		if (!please_blow_up_my_speakers) {
			dev_err(card->dev, "driver can't assure safety on this model, refusing probe\n");
			return -EINVAL;
		}

		CHECK(snd_soc_set_enum_kctl, "* ASI1 Sel", "Left");
		CHECK(snd_soc_deactivate_kctl, "* ASI1 Sel", 0);

		/* !!! This is copied from j274, not obtained by looking at
		 *     what macOS sets.
		 */
		CHECK(snd_soc_limit_volume, "* Amp Gain Volume", 14);

		/*
		 * Since we don't set the right slots yet to avoid
		 * driver conflict on the I2S bus sending ISENSE/VSENSE
		 * samples from the codecs back to us, disable the
		 * controls.
		 */
		CHECK(snd_soc_deactivate_kctl, "* VSENSE Switch", 0);
		CHECK(snd_soc_deactivate_kctl, "* ISENSE Switch", 0);
	}

	return 0;
}

static int macaudio_j314_fixup_controls(struct snd_soc_card *card)
{
	struct macaudio_snd_data *ma = snd_soc_card_get_drvdata(card);

	if (ma->has_speakers) {
		if (!please_blow_up_my_speakers) {
			dev_err(card->dev, "driver can't assure safety on this model, refusing probe\n");
			return -EINVAL;
		}

		CHECK(snd_soc_set_enum_kctl, "* ASI1 Sel", "Left");
		CHECK(snd_soc_deactivate_kctl, "* ASI1 Sel", 0);
		CHECK(snd_soc_limit_volume, "* Amp Gain Volume", 9); // 15 set by macOS, this is 3 dB below
		CHECK(snd_soc_set_enum_kctl, "* Tweeter HPF Corner Frequency", "800 Hz");
		CHECK(snd_soc_deactivate_kctl, "* Tweeter HPF Corner Frequency", 0);

		/*
		 * The speaker amps suffer from spurious overcurrent
		 * events on their unmute, so enable autoretry.
		 */
		CHECK(snd_soc_set_enum_kctl, "* OCE Handling", "Retry");
		CHECK(snd_soc_deactivate_kctl, "* OCE Handling", 0);

		/*
		 * Since we don't set the right slots yet to avoid
		 * driver conflict on the I2S bus sending ISENSE/VSENSE
		 * samples from the codecs back to us, disable the
		 * controls.
		 */
		CHECK(snd_soc_deactivate_kctl, "* VSENSE Switch", 0);
		CHECK(snd_soc_deactivate_kctl, "* ISENSE Switch", 0);
	}

	return 0;
}

static int macaudio_j375_fixup_controls(struct snd_soc_card *card)
{
	struct macaudio_snd_data *ma = snd_soc_card_get_drvdata(card);

	if (ma->has_speakers) {
		if (!please_blow_up_my_speakers) {
			dev_err(card->dev, "driver can't assure safety on this model, refusing probe\n");
			return -EINVAL;
		}

		CHECK(snd_soc_limit_volume, "* Amp Gain Volume", 14); // 20 set by macOS, this is 3 dB below
	}

	return 0;
}

static int macaudio_j493_fixup_controls(struct snd_soc_card *card)
{
	struct macaudio_snd_data *ma = snd_soc_card_get_drvdata(card);

	if (ma->has_speakers) {
		if (!please_blow_up_my_speakers) {
			dev_err(card->dev, "driver can't assure safety on this model, refusing probe\n");
			return -EINVAL;
		}

		CHECK(snd_soc_limit_volume, "* Amp Gain Volume", 9); // 15 set by macOS, this is 3 dB below
	}

	return 0;	
}

static int macaudio_fallback_fixup_controls(struct snd_soc_card *card)
{
	struct macaudio_snd_data *ma = snd_soc_card_get_drvdata(card);

	if (ma->has_speakers && !please_blow_up_my_speakers) {
		dev_err(card->dev, "driver can't assure safety on this model, refusing probe\n");
		return -EINVAL;
	}

	return 0;
}

#undef CHECK

static const char * const macaudio_spk_mux_texts[] = {
	"Primary",
	"Secondary"
};

SOC_ENUM_SINGLE_VIRT_DECL(macaudio_spk_mux_enum, macaudio_spk_mux_texts);

static const struct snd_kcontrol_new macaudio_spk_mux =
	SOC_DAPM_ENUM("Speaker Playback Mux", macaudio_spk_mux_enum);

static const char * const macaudio_hp_mux_texts[] = {
	"Primary",
	"Secondary"
};

SOC_ENUM_SINGLE_VIRT_DECL(macaudio_hp_mux_enum, macaudio_hp_mux_texts);

static const struct snd_kcontrol_new macaudio_hp_mux =
	SOC_DAPM_ENUM("Headphones Playback Mux", macaudio_hp_mux_enum);

static const struct snd_soc_dapm_widget macaudio_snd_widgets[] = {
	SND_SOC_DAPM_SPK("Speaker", NULL),
	SND_SOC_DAPM_SPK("Speaker (Static)", NULL),
	SND_SOC_DAPM_HP("Headphone", NULL),
	SND_SOC_DAPM_MIC("Headset Mic", NULL),

	SND_SOC_DAPM_MUX("Speaker Playback Mux", SND_SOC_NOPM, 0, 0, &macaudio_spk_mux),
	SND_SOC_DAPM_MUX("Headphone Playback Mux", SND_SOC_NOPM, 0, 0, &macaudio_hp_mux),

	SND_SOC_DAPM_AIF_OUT("Speaker Playback", NULL, 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("Headphone Playback", NULL, 0, SND_SOC_NOPM, 0, 0),

	SND_SOC_DAPM_AIF_IN("Headset Capture", NULL, 0, SND_SOC_NOPM, 0, 0),
};

static const struct snd_kcontrol_new macaudio_controls[] = {
	SOC_DAPM_PIN_SWITCH("Speaker"),
	SOC_DAPM_PIN_SWITCH("Headphone"),
	SOC_DAPM_PIN_SWITCH("Headset Mic"),
};

static const struct snd_soc_dapm_route macaudio_dapm_routes[] = {
	/* Playback paths */
	{ "Speaker Playback Mux", "Primary", "PCM0 TX" },
	{ "Speaker Playback Mux", "Secondary", "PCM1 TX" },
	{ "Speaker Playback", NULL, "Speaker Playback Mux"},

	{ "Headphone Playback Mux", "Primary", "PCM0 TX" },
	{ "Headphone Playback Mux", "Secondary", "PCM1 TX" },
	{ "Headphone Playback", NULL, "Headphone Playback Mux"},
	/*
	 * Additional paths (to specific I2S ports) are added dynamically.
	 */

	/* Capture paths */
	{ "PCM0 RX", NULL, "Headset Capture" },
};

static const struct of_device_id macaudio_snd_device_id[]  = {
	{ .compatible = "apple,j274-macaudio", .data = macaudio_j274_fixup_controls },
	{ .compatible = "apple,j313-macaudio", .data = macaudio_j313_fixup_controls },
	{ .compatible = "apple,j314-macaudio", .data = macaudio_j314_fixup_controls },
	{ .compatible = "apple,j375-macaudio", .data = macaudio_j375_fixup_controls },
	{ .compatible = "apple,j413-macaudio", .data = macaudio_j314_fixup_controls },
	{ .compatible = "apple,j493-macaudio", .data = macaudio_j493_fixup_controls },
	{ .compatible = "apple,macaudio"},
	{ }
};
MODULE_DEVICE_TABLE(of, macaudio_snd_device_id);

static int macaudio_snd_platform_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card;
	struct macaudio_snd_data *data;
	struct device *dev = &pdev->dev;
	struct snd_soc_dai_link *link;
	const struct of_device_id *of_id;
	int ret;
	int i;

	of_id = of_match_device(macaudio_snd_device_id, dev);
	if (!of_id)
		return -EINVAL;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;
	card = &data->card;
	snd_soc_card_set_drvdata(card, data);

	card->owner = THIS_MODULE;
	card->driver_name = "macaudio";
	card->dev = dev;
	card->dapm_widgets = macaudio_snd_widgets;
	card->num_dapm_widgets = ARRAY_SIZE(macaudio_snd_widgets);
	card->dapm_routes = macaudio_dapm_routes;
	card->num_dapm_routes = ARRAY_SIZE(macaudio_dapm_routes);
	card->controls = macaudio_controls;
	card->num_controls = ARRAY_SIZE(macaudio_controls);
	card->probe = macaudio_probe;
	card->late_probe = macaudio_late_probe;
	card->component_chaining = true;
	card->fully_routed = true;

	if (of_id->data)
		card->fixup_controls = of_id->data;
	else
		card->fixup_controls = macaudio_fallback_fixup_controls;

	ret = macaudio_parse_of(data);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "failed OF parsing\n");

	for_each_card_prelinks(card, i, link) {
		if (link->no_pcm) {
			link->ops = &macaudio_be_ops;
			link->init = macaudio_be_init;
			link->exit = macaudio_be_exit;
		} else {
			link->ops = &macaudio_fe_ops;
			link->init = macaudio_fe_init;
		}
	}

	return devm_snd_soc_register_card(dev, card);
}

static struct platform_driver macaudio_snd_driver = {
	.probe = macaudio_snd_platform_probe,
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = macaudio_snd_device_id,
		.pm = &snd_soc_pm_ops,
	},
};
module_platform_driver(macaudio_snd_driver);

MODULE_AUTHOR("Martin Povišer <povik+lin@cutebit.org>");
MODULE_DESCRIPTION("Apple Silicon Macs machine-level sound driver");
MODULE_LICENSE("GPL");
