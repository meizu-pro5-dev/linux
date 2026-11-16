// SPDX-License-Identifier: GPL-2.0+
/*
 * Samsung Exynos7420 Meizu PRO 5 (m86) ASoC machine driver
 *
 * Copyright (C) 2026 Antigravity Project
 * Author: Pair Programmer <kophapro@gmail.com>
 *
 * Audio topology:
 *   - Primary PCM (hw:x,0): AP I2S1 <-> ES705 DSP (DHWPT) <-> WM8998 AIF1
 *   - Hi-Fi PCM (hw:x,1):   AP I2S0 <-> ES9018K2M Hi-Fi DAC
 *   - Speaker C2C link:     WM8998 AIF3 <-> TFA9890 smart amplifier
 */

#include <linux/atomic.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_qos.h>
#include <linux/regmap.h>
#include <linux/soc/samsung/exynos7420-bus-qos.h>
#include <sound/jack.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include <linux/mfd/arizona/registers.h>
#include "i2s.h"
#include "../codecs/wm8998.h"
#include "../codecs/es9018k2m.h"

#define MCLK_RATE		24000000U
#define M86_MAX_DAI_LINKS	3

struct m86_machine_priv {
	struct snd_soc_card card;
	struct snd_soc_dai_link dai_links[M86_MAX_DAI_LINKS];
	struct snd_soc_dai_link_component aif1_cpus[1];
	struct snd_soc_dai_link_component aif1_plats[1];
	struct snd_soc_dai_link_component aif1_codecs[2];
	struct snd_soc_dai_link_component hifi_cpus[1];
	struct snd_soc_dai_link_component hifi_plats[1];
	struct snd_soc_dai_link_component hifi_codecs[1];
	struct snd_soc_dai_link_component spk_cpus[1];
	struct snd_soc_dai_link_component spk_codecs[1];
	struct snd_soc_pcm_stream speaker_params;
	struct snd_soc_component *component;
	unsigned int sysclk_rate;
	unsigned int fll_rate; /* FLL1 output currently programmed, 0 = off */
	struct device_node *cpu_np;
	struct device_node *codec_np;
	struct device_node *dsp_np;
	struct device_node *amp_np;
	struct device_node *hifi_cpu_np;
	struct device_node *hifi_np;
	/* PM QoS protection for active audio streaming */
	struct pm_qos_request cpu_latency_req;
	bool cpu_latency_active;
	struct device *bus_dev;
	struct dev_pm_qos_request bus_qos_req;
	bool bus_qos_active;
	atomic_t active_streams;
	struct mutex qos_lock; /* Protects CPU/MIF QoS request updates */
	/* 3.5 mm jack, detected by the WM8998 JD1/MICDET accessory logic */
	struct snd_soc_jack jack;
};

static struct snd_soc_jack_pin m86_jack_pins[] = {
	{ .pin = "Headphone",	.mask = SND_JACK_HEADPHONE },
	{ .pin = "Headset Mic",	.mask = SND_JACK_MICROPHONE },
};

static int m86_start_sysclk(struct snd_soc_card *card)
{
	struct m86_machine_priv *priv = snd_soc_card_get_drvdata(card);
	struct snd_soc_component *component = priv->component;
	int ret;

	if (!component)
		return -EAGAIN;

	/* Reprogramming FLL1 unchanged would glitch a running stream. */
	if (priv->fll_rate != priv->sysclk_rate) {
		/* Disable FLL1 REFCLK synchroniser per vendor driver */
		ret = snd_soc_component_set_pll(component, WM8998_FLL1_REFCLK,
						ARIZONA_FLL_SRC_NONE, 0, 0);
		if (ret < 0) {
			dev_err(component->dev, "Failed to stop FLL1 REF: %d\n", ret);
			return ret;
		}

		/* FLL1: MCLK1 24 MHz -> 24.576 MHz (48k) or 22.5792 MHz (44.1k) */
		ret = snd_soc_component_set_pll(component, WM8998_FLL1,
						ARIZONA_FLL_SRC_MCLK1,
						MCLK_RATE, priv->sysclk_rate);
		if (ret < 0) {
			dev_err(component->dev, "Failed to start FLL1: %d\n", ret);
			return ret;
		}

		ret = snd_soc_component_set_sysclk(component, ARIZONA_CLK_SYSCLK,
						   ARIZONA_CLK_SRC_FLL1,
						   priv->sysclk_rate,
						   SND_SOC_CLOCK_IN);
		if (ret < 0) {
			dev_err(component->dev, "Failed to set SYSCLK source: %d\n", ret);
			return ret;
		}
		priv->fll_rate = priv->sysclk_rate;
	}

	regmap_update_bits(component->regmap, ARIZONA_SYSTEM_CLOCK_1,
			   ARIZONA_SYSCLK_ENA, ARIZONA_SYSCLK_ENA);

	/* Force continuous BCLK and LRCLK on AIF1 in Master mode for ES705 */
	regmap_update_bits(component->regmap, ARIZONA_AIF1_BCLK_CTRL,
			   ARIZONA_AIF1_BCLK_FRC, ARIZONA_AIF1_BCLK_FRC);
	regmap_update_bits(component->regmap, ARIZONA_AIF1_TX_PIN_CTRL,
			   ARIZONA_AIF1TX_LRCLK_FRC, ARIZONA_AIF1TX_LRCLK_FRC);
	regmap_update_bits(component->regmap, ARIZONA_AIF1_RX_PIN_CTRL,
			   ARIZONA_AIF1RX_LRCLK_FRC, ARIZONA_AIF1RX_LRCLK_FRC);

	return 0;
}

static int m86_stop_sysclk(struct snd_soc_card *card)
{
	struct m86_machine_priv *priv = snd_soc_card_get_drvdata(card);
	struct snd_soc_component *component = priv->component;
	int ret;

	if (!component)
		return 0;

	regmap_update_bits(component->regmap, ARIZONA_AIF1_BCLK_CTRL,
			   ARIZONA_AIF1_BCLK_FRC, 0);
	regmap_update_bits(component->regmap, ARIZONA_AIF1_TX_PIN_CTRL,
			   ARIZONA_AIF1TX_LRCLK_FRC, 0);
	regmap_update_bits(component->regmap, ARIZONA_AIF1_RX_PIN_CTRL,
			   ARIZONA_AIF1RX_LRCLK_FRC, 0);
	regmap_update_bits(component->regmap, ARIZONA_SYSTEM_CLOCK_1,
			   ARIZONA_SYSCLK_ENA, 0);

	ret = snd_soc_component_set_sysclk(component, ARIZONA_CLK_SYSCLK,
					   ARIZONA_CLK_SRC_FLL1, 0, 0);
	if (ret < 0)
		dev_err(component->dev, "Failed to stop SYSCLK: %d\n", ret);

	ret = snd_soc_component_set_pll(component, WM8998_FLL1, 0, 0, 0);
	if (ret < 0)
		dev_err(component->dev, "Failed to stop FLL1: %d\n", ret);

	priv->fll_rate = 0;

	return 0;
}

static int m86_set_bias_level(struct snd_soc_card *card,
			      struct snd_soc_dapm_context *dapm,
			      enum snd_soc_bias_level level)
{
	struct snd_soc_pcm_runtime *rtd;

	rtd = snd_soc_get_pcm_runtime(card, &card->dai_link[0]);

	if (dapm->dev != asoc_rtd_to_codec(rtd, 0)->dev)
		return 0;

	switch (level) {
	case SND_SOC_BIAS_STANDBY:
		if (snd_soc_dapm_get_bias_level(dapm) == SND_SOC_BIAS_OFF)
			return m86_start_sysclk(card);
		break;
	case SND_SOC_BIAS_OFF:
		m86_stop_sysclk(card);
		break;
	default:
		break;
	}

	return 0;
}

static void m86_audio_pm_qos_start(struct m86_machine_priv *priv)
{
	int active = atomic_inc_return(&priv->active_streams);

	if (active == 1) {
		mutex_lock(&priv->qos_lock);
		/*
		 * Request max 50us CPU DMA wakeup latency to keep cores out of
		 * C2 power gating during active ALSA/I2S DMA transfers.
		 */
		if (!priv->cpu_latency_active) {
			cpu_latency_qos_add_request(&priv->cpu_latency_req, 50);
			priv->cpu_latency_active = true;
		}

		/* Floor bus-mif >= 416 MHz for multi-master DMA traffic. */
		if (!priv->bus_dev)
			priv->bus_dev = exynos_bus_get_dev("bus-mif");
		if (priv->bus_dev && !priv->bus_qos_active) {
			int ret = exynos_bus_qos_add_request(priv->bus_dev,
							     &priv->bus_qos_req,
							     416000);
			if (ret >= 0)
				priv->bus_qos_active = true;
		}
		mutex_unlock(&priv->qos_lock);
		pr_info("m86-audio: locked PM QoS (CPU latency 50us, bus-mif 416MHz)\n");
	}
}

static void m86_audio_pm_qos_stop(struct m86_machine_priv *priv)
{
	int active = atomic_dec_return(&priv->active_streams);

	if (active == 0) {
		mutex_lock(&priv->qos_lock);
		if (priv->bus_qos_active) {
			exynos_bus_qos_remove_request(&priv->bus_qos_req);
			priv->bus_qos_active = false;
		}
		if (priv->cpu_latency_active) {
			cpu_latency_qos_remove_request(&priv->cpu_latency_req);
			priv->cpu_latency_active = false;
		}
		mutex_unlock(&priv->qos_lock);
		pr_info("m86-audio: released PM QoS\n");
	} else if (active < 0) {
		atomic_set(&priv->active_streams, 0);
	}
}

static int m86_aif1_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = asoc_substream_to_rtd(substream);
	struct m86_machine_priv *priv = snd_soc_card_get_drvdata(rtd->card);

	m86_audio_pm_qos_start(priv);
	return 0;
}

static void m86_aif1_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = asoc_substream_to_rtd(substream);
	struct m86_machine_priv *priv = snd_soc_card_get_drvdata(rtd->card);

	m86_audio_pm_qos_stop(priv);
}

static int m86_aif1_hw_params(struct snd_pcm_substream *substream,
			      struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = asoc_substream_to_rtd(substream);
	struct snd_soc_component *component = asoc_rtd_to_codec(rtd, 0)->component;
	struct snd_soc_dai *cpu_dai = asoc_rtd_to_cpu(rtd, 0);
	struct snd_soc_dai *codec_dai = asoc_rtd_to_codec(rtd, 0);
	struct m86_machine_priv *priv = snd_soc_card_get_drvdata(rtd->card);
	int ret;

	switch (params_rate(params)) {
	case 8000:
	case 12000:
	case 16000:
	case 24000:
	case 32000:
	case 48000:
	case 64000:
	case 96000:
	case 192000:
		priv->sysclk_rate = 24576000;
		break;
	case 11025:
	case 22050:
	case 44100:
	case 88200:
	case 176400:
		priv->sysclk_rate = 22579200;
		break;
	default:
		dev_err(component->dev, "Unsupported sample rate: %d\n",
			params_rate(params));
		return -EINVAL;
	}

	/* Configure Samsung I2S1 CPU DAI in slave mode */
	ret = snd_soc_dai_set_sysclk(cpu_dai, SAMSUNG_I2S_CDCLK,
				     0, SND_SOC_CLOCK_IN);
	if (ret < 0)
		dev_warn(cpu_dai->dev, "Failed to set I2S_CDCLK: %d\n", ret);

	ret = snd_soc_dai_set_sysclk(cpu_dai, SAMSUNG_I2S_OPCLK,
				     0, SAMSUNG_I2S_OPCLK_PCLK);
	if (ret < 0)
		dev_warn(cpu_dai->dev, "Failed to set I2S_OPCLK: %d\n", ret);

	/* Switch WM8998 AIF1 DAI to SYSCLK */
	ret = snd_soc_dai_set_sysclk(codec_dai, ARIZONA_CLK_SYSCLK, 0, 0);
	if (ret < 0)
		dev_warn(codec_dai->dev, "Failed to set codec DAI SYSCLK: %d\n", ret);

	return m86_start_sysclk(rtd->card);
}

static int m86_aif1_prepare(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = asoc_substream_to_rtd(substream);

	return m86_start_sysclk(rtd->card);
}

static const struct snd_soc_ops m86_aif1_ops = {
	.startup   = m86_aif1_startup,
	.shutdown  = m86_aif1_shutdown,
	.hw_params = m86_aif1_hw_params,
	.prepare   = m86_aif1_prepare,
};

static int m86_hifi_hw_params(struct snd_pcm_substream *substream,
			      struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = asoc_substream_to_rtd(substream);
	struct snd_soc_dai *cpu_dai = asoc_rtd_to_cpu(rtd, 0);
	struct snd_soc_dai *codec_dai = asoc_rtd_to_codec(rtd, 0);
	unsigned int pll_out;
	int ret;

	switch (params_rate(params)) {
	case 8000:
	case 16000:
	case 24000:
	case 32000:
	case 48000:
	case 64000:
	case 96000:
	case 192000:
		pll_out = 49152000;
		break;
	case 11025:
	case 22050:
	case 44100:
	case 88200:
	case 176400:
		pll_out = 45158400;
		break;
	default:
		dev_err(codec_dai->dev, "Unsupported HiFi sample rate: %d\n",
			params_rate(params));
		return -EINVAL;
	}

	ret = snd_soc_dai_set_sysclk(codec_dai, ES9018K2M_SYSCLK_MCLK,
				     pll_out, SND_SOC_CLOCK_IN);
	if (ret < 0) {
		dev_err(codec_dai->dev, "Failed to set HiFi MCLK: %d\n", ret);
		return ret;
	}

	/* Configure LPASS I2S0 in slave mode receiving 64fs BCLK/LRCK from ES9018K2M */
	ret = snd_soc_dai_set_sysclk(cpu_dai, SAMSUNG_I2S_CDCLK,
				     0, SND_SOC_CLOCK_IN);
	if (ret < 0)
		dev_warn(cpu_dai->dev, "Failed to set HiFi I2S_CDCLK: %d\n", ret);

	ret = snd_soc_dai_set_sysclk(cpu_dai, SAMSUNG_I2S_OPCLK,
				     0, SAMSUNG_I2S_OPCLK_PCLK);
	if (ret < 0)
		dev_warn(cpu_dai->dev, "Failed to set HiFi I2S_OPCLK: %d\n", ret);

	ret = snd_soc_dai_set_clkdiv(cpu_dai, SAMSUNG_I2S_DIV_BCLK, 64);
	if (ret < 0) {
		dev_err(cpu_dai->dev, "Failed to set HiFi BCLK 64fs: %d\n", ret);
		return ret;
	}

	return 0;
}

static const struct snd_soc_ops m86_hifi_ops = {
	.startup   = m86_aif1_startup,
	.shutdown  = m86_aif1_shutdown,
	.hw_params = m86_hifi_hw_params,
};

static int m86_speaker_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_dai *aif3_dai = asoc_rtd_to_cpu(rtd, 0);

	return snd_soc_dai_set_sysclk(aif3_dai, ARIZONA_CLK_SYSCLK, 0, 0);
}

static int m86_speaker_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = asoc_substream_to_rtd(substream);
	struct m86_machine_priv *priv = snd_soc_card_get_drvdata(rtd->card);
	unsigned int rate = (priv->sysclk_rate == 22579200) ? 44100 : 48000;

	/* Match active SYSCLK family (48 kHz vs 44.1 kHz) before C2C hw_params */
	priv->speaker_params.rate_min = rate;
	priv->speaker_params.rate_max = rate;

	return 0;
}

static const struct snd_soc_ops m86_speaker_ops = {
	.startup = m86_speaker_startup,
};

static const struct snd_kcontrol_new m86_controls[] = {
	SOC_DAPM_PIN_SWITCH("Headphone"),
	SOC_DAPM_PIN_SWITCH("Earpiece"),
	SOC_DAPM_PIN_SWITCH("Speaker"),
	SOC_DAPM_PIN_SWITCH("Headset Mic"),
	SOC_DAPM_PIN_SWITCH("Main Mic"),
};

static const struct snd_soc_dapm_widget m86_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone", NULL),
	SND_SOC_DAPM_SPK("Earpiece", NULL),
	SND_SOC_DAPM_SPK("Speaker", NULL),
	SND_SOC_DAPM_MIC("Headset Mic", NULL),
	SND_SOC_DAPM_MIC("Main Mic", NULL),
};

static int m86_late_probe(struct snd_soc_card *card)
{
	struct m86_machine_priv *priv = snd_soc_card_get_drvdata(card);
	struct snd_soc_pcm_runtime *rtd = snd_soc_get_pcm_runtime(card, &card->dai_link[0]);
	struct snd_soc_dai *aif1_dai = asoc_rtd_to_codec(rtd, 0);
	int ret;

	priv->component = aif1_dai->component;
	priv->sysclk_rate = 24576000;

	ret = snd_soc_dai_set_sysclk(aif1_dai, ARIZONA_CLK_SYSCLK, 0, 0);
	if (ret < 0) {
		dev_err(aif1_dai->dev, "Failed to set SYSCLK: %d\n", ret);
		return ret;
	}

	/* Keep endpoints disabled until UCM selects a path or jack fires. */
	snd_soc_dapm_disable_pin(&card->dapm, "Earpiece");
	snd_soc_dapm_disable_pin(&card->dapm, "Speaker");
	snd_soc_dapm_disable_pin(&card->dapm, "Main Mic");

	ret = snd_soc_card_jack_new_pins(card, "Headset",
					 SND_JACK_HEADSET | SND_JACK_LINEOUT |
					 SND_JACK_MECHANICAL | SND_JACK_BTN_0 |
					 SND_JACK_BTN_1 | SND_JACK_BTN_2,
					 &priv->jack, m86_jack_pins,
					 ARRAY_SIZE(m86_jack_pins));
	if (ret) {
		dev_err(card->dev, "Failed to create headset jack: %d\n", ret);
		return ret;
	}

	ret = snd_soc_component_set_jack(priv->component, &priv->jack, NULL);
	if (ret) {
		dev_err(card->dev, "Failed to enable jack detection: %d\n", ret);
		return ret;
	}
	snd_soc_dapm_sync(&card->dapm);

	return 0;
}

static int m86_card_remove(struct snd_soc_card *card)
{
	struct m86_machine_priv *priv = snd_soc_card_get_drvdata(card);

	if (priv->component)
		snd_soc_component_set_jack(priv->component, NULL, NULL);

	return 0;
}

static void m86_cleanup_resources(void *data)
{
	struct m86_machine_priv *priv = data;

	mutex_lock(&priv->qos_lock);
	if (priv->bus_qos_active) {
		exynos_bus_qos_remove_request(&priv->bus_qos_req);
		priv->bus_qos_active = false;
	}
	if (priv->cpu_latency_active) {
		cpu_latency_qos_remove_request(&priv->cpu_latency_req);
		priv->cpu_latency_active = false;
	}
	mutex_unlock(&priv->qos_lock);

	of_node_put(priv->cpu_np);
	of_node_put(priv->codec_np);
	of_node_put(priv->dsp_np);
	of_node_put(priv->amp_np);
	of_node_put(priv->hifi_cpu_np);
	of_node_put(priv->hifi_np);
}

static void m86_setup_dai_links(struct m86_machine_priv *priv)
{
	struct snd_soc_card *card = &priv->card;
	int idx = 0;

	priv->aif1_cpus[0].dai_name = SAMSUNG_I2S_DAI;
	priv->aif1_cpus[0].of_node = priv->cpu_np;
	priv->aif1_plats[0].of_node = priv->cpu_np;

	priv->aif1_codecs[0].dai_name = "wm8998-aif1";
	priv->aif1_codecs[0].of_node = priv->codec_np;
	priv->aif1_codecs[1].dai_name = "es705-dhwpt";
	priv->aif1_codecs[1].of_node = priv->dsp_np;

	priv->dai_links[idx] = (struct snd_soc_dai_link){
		.name		= "WM8998 AIF1",
		.stream_name	= "Primary Audio",
		.ops		= &m86_aif1_ops,
		.dai_fmt	= SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
				  SND_SOC_DAIFMT_CBM_CFM,
		.symmetric_rate	= 1,
		.cpus		= priv->aif1_cpus,
		.num_cpus	= ARRAY_SIZE(priv->aif1_cpus),
		.platforms	= priv->aif1_plats,
		.num_platforms	= ARRAY_SIZE(priv->aif1_plats),
		.codecs		= priv->aif1_codecs,
		.num_codecs	= priv->dsp_np ? 2 : 1,
	};
	idx++;

	if (priv->hifi_cpu_np && priv->hifi_np) {
		priv->hifi_cpus[0].dai_name = SAMSUNG_I2S_DAI;
		priv->hifi_cpus[0].of_node = priv->hifi_cpu_np;
		priv->hifi_plats[0].of_node = priv->hifi_cpu_np;
		priv->hifi_codecs[0].dai_name = "ess9018k2m-hifi";
		priv->hifi_codecs[0].of_node = priv->hifi_np;

		priv->dai_links[idx] = (struct snd_soc_dai_link){
			.name		= "ES9018K2M HiFi",
			.stream_name	= "HiFi Playback",
			.ops		= &m86_hifi_ops,
			.dai_fmt	= SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
					  SND_SOC_DAIFMT_CBM_CFM,
			.playback_only	= 1,
			.cpus		= priv->hifi_cpus,
			.num_cpus	= ARRAY_SIZE(priv->hifi_cpus),
			.platforms	= priv->hifi_plats,
			.num_platforms	= ARRAY_SIZE(priv->hifi_plats),
			.codecs		= priv->hifi_codecs,
			.num_codecs	= ARRAY_SIZE(priv->hifi_codecs),
		};
		idx++;
	}

	if (priv->amp_np) {
		priv->speaker_params = (struct snd_soc_pcm_stream){
			.formats	= SNDRV_PCM_FMTBIT_S16_LE,
			.rate_min	= 48000,
			.rate_max	= 48000,
			.channels_min	= 2,
			.channels_max	= 2,
		};

		priv->spk_cpus[0].dai_name = "wm8998-aif3";
		priv->spk_cpus[0].of_node = priv->codec_np;
		priv->spk_codecs[0].dai_name = "tfa989x-hifi";
		priv->spk_codecs[0].of_node = priv->amp_np;

		priv->dai_links[idx] = (struct snd_soc_dai_link){
			.name		= "WM8998 AIF3-TFA9890",
			.stream_name	= "Speaker Amp",
			.init		= m86_speaker_init,
			.ops		= &m86_speaker_ops,
			.dai_fmt	= SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
					  SND_SOC_DAIFMT_CBS_CFS,
			.c2c_params	= &priv->speaker_params,
			.num_c2c_params	= 1,
			.ignore_suspend	= 1,
			.cpus		= priv->spk_cpus,
			.num_cpus	= ARRAY_SIZE(priv->spk_cpus),
			.codecs		= priv->spk_codecs,
			.num_codecs	= ARRAY_SIZE(priv->spk_codecs),
		};
		idx++;
	}

	card->dai_link = priv->dai_links;
	card->num_links = idx;
}

static int m86_audio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct m86_machine_priv *priv;
	struct snd_soc_card *card;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	mutex_init(&priv->qos_lock);
	atomic_set(&priv->active_streams, 0);
	priv->bus_dev = exynos_bus_get_dev("bus-mif");
	if (!priv->bus_dev)
		priv->bus_dev = exynos_bus_get_dev(NULL);

	priv->cpu_np = of_parse_phandle(dev->of_node, "i2s-controller", 0);
	if (!priv->cpu_np)
		return dev_err_probe(dev, -EINVAL, "i2s-controller property missing\n");

	priv->codec_np = of_parse_phandle(dev->of_node, "audio-codec", 0);
	if (!priv->codec_np) {
		of_node_put(priv->cpu_np);
		return dev_err_probe(dev, -EINVAL, "audio-codec property missing\n");
	}

	priv->dsp_np = of_parse_phandle(dev->of_node, "samsung,dsp", 0);
	priv->amp_np = of_parse_phandle(dev->of_node, "audio-amplifier", 0);
	priv->hifi_cpu_np = of_parse_phandle(dev->of_node, "hifi-cpu", 0);
	priv->hifi_np = of_parse_phandle(dev->of_node, "hifi-dac", 0);

	ret = devm_add_action_or_reset(dev, m86_cleanup_resources, priv);
	if (ret) {
		of_node_put(priv->cpu_np);
		of_node_put(priv->codec_np);
		of_node_put(priv->dsp_np);
		of_node_put(priv->amp_np);
		of_node_put(priv->hifi_cpu_np);
		of_node_put(priv->hifi_np);
		return ret;
	}

	card = &priv->card;
	card->owner = THIS_MODULE;
	card->dev = dev;
	card->controls = m86_controls;
	card->num_controls = ARRAY_SIZE(m86_controls);
	card->dapm_widgets = m86_dapm_widgets;
	card->num_dapm_widgets = ARRAY_SIZE(m86_dapm_widgets);
	card->late_probe = m86_late_probe;
	card->remove = m86_card_remove;
	card->set_bias_level = m86_set_bias_level;

	snd_soc_card_set_drvdata(card, priv);

	ret = snd_soc_of_parse_card_name(card, "model");
	if (ret < 0) {
		dev_err(dev, "Card name is not specified\n");
		return ret;
	}

	ret = snd_soc_of_parse_audio_routing(card, "samsung,audio-routing");
	if (ret < 0) {
		ret = snd_soc_of_parse_audio_routing(card, "audio-routing");
		if (ret < 0)
			dev_warn(dev, "Audio routing is not specified\n");
	}

	m86_setup_dai_links(priv);

	ret = devm_snd_soc_register_card(dev, card);
	if (ret < 0)
		return dev_err_probe(dev, ret, "Failed to register card\n");

	return 0;
}

static int m86_pm_prepare(struct device *dev)
{
	struct snd_soc_card *card = dev_get_drvdata(dev);

	return m86_stop_sysclk(card);
}

/*
 * No .complete: after resume the next STANDBY transition or hw_params calls
 * m86_start_sysclk(), so an idle card keeps FLL1 and BCLK forcing off.
 */
static const struct dev_pm_ops m86_pm_ops = {
	.prepare	= m86_pm_prepare,
	.suspend	= snd_soc_suspend,
	.resume		= snd_soc_resume,
	.freeze		= snd_soc_suspend,
	.thaw		= snd_soc_resume,
	.poweroff	= snd_soc_poweroff,
	.restore	= snd_soc_resume,
};

static const struct of_device_id m86_audio_of_match[] = {
	{ .compatible = "samsung,meizu-m86-audio" },
	{ },
};
MODULE_DEVICE_TABLE(of, m86_audio_of_match);

static struct platform_driver m86_audio_driver = {
	.driver = {
		.name		= "m86-audio",
		.pm		= &m86_pm_ops,
		.of_match_table	= m86_audio_of_match,
	},
	.probe	= m86_audio_probe,
};
module_platform_driver(m86_audio_driver);

MODULE_AUTHOR("Pair Programmer <kophapro@gmail.com>");
MODULE_DESCRIPTION("ALSA SoC Meizu PRO 5 Audio Support");
MODULE_LICENSE("GPL");
