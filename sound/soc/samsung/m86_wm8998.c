// SPDX-License-Identifier: GPL-2.0+
/*
 * Samsung Exynos7420 Meizu PRO 5 (m86) ASoC machine driver
 *
 * Copyright (C) 2026 Antigravity Project
 * Author: Pair Programmer <kophapro@gmail.com>
 *
 * Based on sound/soc/samsung/tm2_wm5110.c and vendor meizu_audio_primary.c
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/mfd/syscon.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include "i2s.h"
#include <linux/mfd/arizona/registers.h>
#include "../codecs/wm8998.h"

#define MCLK_RATE		24000000U
#define PMU_DEBUG_REG		0x0A00
#define PMU_DEBUG_MCLK_EN	0x1000
#define PMU_RTC_CLKO_SEL_REG	0x001C

struct m86_machine_priv {
	struct snd_soc_component *component;
	struct regmap *pmu;
	struct clk *mclk;
	struct gpio_desc *amp_reset;
	struct gpio_desc *dsp_reset;
	struct i2c_adapter *i2c_adap;
	unsigned int sysclk_rate;
	struct snd_soc_aux_dev amp_dev;
};

static int m86_es705_enable_dhwpt(struct i2c_adapter *adap)
{
	u8 cmd[4] = { 0x80, 0x52, 0x00, 0xf3 };
	u8 resp[4] = { 0 };
	struct i2c_msg msgs[2] = {
		{
			.addr = 0x3e,
			.flags = 0,
			.len = sizeof(cmd),
			.buf = cmd,
		},
		{
			.addr = 0x3e,
			.flags = I2C_M_RD,
			.len = sizeof(resp),
			.buf = resp,
		},
	};
	int ret;

	if (!adap)
		return -ENODEV;

	ret = i2c_transfer(adap, msgs, 2);
	if (ret == 2) {
		pr_info("m86-audio: ES705 DHWPT pass-through active (resp 0x%02x%02x%02x%02x)\n",
			resp[0], resp[1], resp[2], resp[3]);
		return 0;
	}
	pr_warn("m86-audio: Failed to send ES705 DHWPT command: %d\n", ret);
	return ret < 0 ? ret : -EIO;
}

static int m86_start_sysclk(struct snd_soc_card *card)
{
	struct m86_machine_priv *priv = snd_soc_card_get_drvdata(card);
	struct snd_soc_component *component = priv->component;
	int ret;

	if (!component)
		return -EAGAIN;

	/* Enable 24MHz MCLK and RTC CLKO from PMU if PMU regmap is available */
	if (priv->pmu) {
		regmap_write(priv->pmu, PMU_DEBUG_REG, PMU_DEBUG_MCLK_EN);
		regmap_write(priv->pmu, PMU_RTC_CLKO_SEL_REG, 0x0);
	}

	if (priv->mclk) {
		ret = clk_prepare_enable(priv->mclk);
		if (ret < 0)
			dev_warn(card->dev, "Failed to enable MCLK clock: %d\n", ret);
	}

	/* Disable FLL1 REFCLK synchroniser per vendor driver */
	ret = snd_soc_component_set_pll(component, WM8998_FLL1_REFCLK,
					ARIZONA_FLL_SRC_NONE,
					0, 0);
	if (ret < 0) {
		dev_err(component->dev, "Failed to stop FLL1 REF: %d\n", ret);
		return ret;
	}

	/* FLL1 locks MCLK1 (24MHz) to 24.576MHz (48k) or 22.5792MHz (44.1k) */
	ret = snd_soc_component_set_pll(component, WM8998_FLL1,
					ARIZONA_FLL_SRC_MCLK1,
					MCLK_RATE,
					priv->sysclk_rate);
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

	/* Explicitly enable SYSCLK in ARIZONA_SYSTEM_CLOCK_1 */
	regmap_update_bits(component->regmap, ARIZONA_SYSTEM_CLOCK_1,
			   ARIZONA_SYSCLK_ENA, ARIZONA_SYSCLK_ENA);

	/* Force continuous BCLK and LRCLK in Master mode per vendor driver */
	regmap_update_bits(component->regmap, ARIZONA_AIF1_BCLK_CTRL,
			   ARIZONA_AIF1_BCLK_FRC, ARIZONA_AIF1_BCLK_FRC);
	regmap_update_bits(component->regmap, ARIZONA_AIF1_TX_PIN_CTRL,
			   ARIZONA_AIF1TX_LRCLK_FRC, ARIZONA_AIF1TX_LRCLK_FRC);
	regmap_update_bits(component->regmap, ARIZONA_AIF1_RX_PIN_CTRL,
			   ARIZONA_AIF1RX_LRCLK_FRC, ARIZONA_AIF1RX_LRCLK_FRC);

	/* Ensure Audience ES705 DSP is in Direct Hardware Pass-Through mode */
	if (priv->i2c_adap)
		m86_es705_enable_dhwpt(priv->i2c_adap);

	pr_info("m86-audio: start_sysclk FLL1=%d rate=%u wm8998: 0x101=0x%04x 0x171=0x%04x 0x500=0x%04x 0x501=0x%04x\n",
		ret, priv->sysclk_rate,
		snd_soc_component_read(component, ARIZONA_SYSTEM_CLOCK_1),
		snd_soc_component_read(component, ARIZONA_FLL1_CONTROL_1),
		snd_soc_component_read(component, ARIZONA_AIF1_BCLK_CTRL),
		snd_soc_component_read(component, ARIZONA_AIF1_TX_PIN_CTRL));

	return 0;
}

static int m86_stop_sysclk(struct snd_soc_card *card)
{
	struct m86_machine_priv *priv = snd_soc_card_get_drvdata(card);
	struct snd_soc_component *component = priv->component;
	int ret;

	if (!component)
		return 0;

	pr_info("m86-audio: stop_sysclk\n");

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

	if (priv->mclk)
		clk_disable_unprepare(priv->mclk);

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
		if (card->dapm.bias_level == SND_SOC_BIAS_OFF)
			m86_start_sysclk(card);
		break;
	case SND_SOC_BIAS_OFF:
		m86_stop_sysclk(card);
		break;
	default:
		break;
	}

	return 0;
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

	/* Switch codec DAI to SYSCLK */
	ret = snd_soc_dai_set_sysclk(codec_dai, ARIZONA_CLK_SYSCLK, 0, 0);
	if (ret < 0)
		dev_warn(codec_dai->dev, "Failed to set codec DAI SYSCLK: %d\n", ret);

	return m86_start_sysclk(rtd->card);
}

static const struct snd_soc_ops m86_aif1_ops = {
	.hw_params = m86_aif1_hw_params,
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

static const struct snd_soc_dapm_route m86_audio_routes[] = {
	{"Headphone", NULL, "HPOUTL"},
	{"Headphone", NULL, "HPOUTR"},
	{"Earpiece", NULL, "EPOUT"},
	{"Speaker", NULL, "SPKOUTLN"},
	{"Speaker", NULL, "SPKOUTLP"},
	{"Speaker", NULL, "SPKOUTRN"},
	{"Speaker", NULL, "SPKOUTRP"},
	{"Headset Mic", NULL, "MICBIAS1"},
	{"IN2B", NULL, "Headset Mic"},
	{"Main Mic", NULL, "MICBIAS2"},
	{"IN1AL", NULL, "Main Mic"},
};

SND_SOC_DAILINK_DEFS(aif1,
	DAILINK_COMP_ARRAY(COMP_CPU(SAMSUNG_I2S_DAI)),
	DAILINK_COMP_ARRAY(COMP_CODEC(NULL, "wm8998-aif1")),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

static struct snd_soc_dai_link m86_dai_links[] = {
	{
		.name		= "WM8998 AIF1",
		.stream_name	= "Primary Audio",
		.ops		= &m86_aif1_ops,
		.dai_fmt	= SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
				  SND_SOC_DAIFMT_CBM_CFM,
		SND_SOC_DAILINK_REG(aif1),
	},
};

static struct snd_soc_card m86_card = {
	.owner			= THIS_MODULE,
	.dai_link		= m86_dai_links,
	.num_links		= ARRAY_SIZE(m86_dai_links),
	.controls		= m86_controls,
	.num_controls		= ARRAY_SIZE(m86_controls),
	.dapm_widgets		= m86_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(m86_dapm_widgets),
	.dapm_routes		= m86_audio_routes,
	.num_dapm_routes	= ARRAY_SIZE(m86_audio_routes),
	.set_bias_level		= m86_set_bias_level,
};

static int m86_late_probe(struct snd_soc_card *card)
{
	struct m86_machine_priv *priv = snd_soc_card_get_drvdata(card);
	struct snd_soc_pcm_runtime *rtd = list_first_entry(&card->rtd_list,
							    struct snd_soc_pcm_runtime,
							    list);
	struct snd_soc_dai *aif1_dai;
	int ret;

	priv->component = asoc_rtd_to_codec(rtd, 0)->component;
	aif1_dai = asoc_rtd_to_codec(rtd, 0);

	/* Default sysclk to 48kHz family (24.576 MHz) */
	priv->sysclk_rate = 24576000;

	/* Enable MCLK so WM8998 has reference clock available */
	if (priv->pmu) {
		regmap_write(priv->pmu, PMU_DEBUG_REG, PMU_DEBUG_MCLK_EN);
		regmap_write(priv->pmu, PMU_RTC_CLKO_SEL_REG, 0x0);
	}

	ret = snd_soc_dai_set_sysclk(aif1_dai, ARIZONA_CLK_SYSCLK, 0, 0);
	if (ret < 0) {
		dev_err(aif1_dai->dev, "Failed to set SYSCLK: %d\n", ret);
		return ret;
	}

	/* Enable Headphone output by default; keep other endpoints disabled until selected */
	snd_soc_dapm_enable_pin(&card->dapm, "Headphone");
	snd_soc_dapm_disable_pin(&card->dapm, "Earpiece");
	snd_soc_dapm_disable_pin(&card->dapm, "Speaker");
	snd_soc_dapm_disable_pin(&card->dapm, "Headset Mic");
	snd_soc_dapm_disable_pin(&card->dapm, "Main Mic");
	snd_soc_dapm_sync(&card->dapm);

	return 0;
}

static int m86_audio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct snd_soc_card *card = &m86_card;
	struct m86_machine_priv *priv;
	struct device_node *cpu_np, *codec_np, *amp_np;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	snd_soc_card_set_drvdata(card, priv);
	card->dev = dev;
	card->late_probe = m86_late_probe;

	priv->pmu = syscon_regmap_lookup_by_phandle(dev->of_node, "samsung,pmu");
	if (IS_ERR(priv->pmu))
		priv->pmu = NULL;

	priv->mclk = devm_clk_get_optional(dev, "mclk");
	if (IS_ERR(priv->mclk))
		priv->mclk = NULL;

	/* Hardware reset pulse for TFA9890 amplifier */
	priv->amp_reset = devm_gpiod_get_optional(dev, "amp-reset", GPIOD_OUT_LOW);
	if (IS_ERR(priv->amp_reset)) {
		dev_warn(dev, "Failed to get amp-reset GPIO: %ld\n", PTR_ERR(priv->amp_reset));
		priv->amp_reset = NULL;
	} else if (priv->amp_reset) {
		gpiod_set_value_cansleep(priv->amp_reset, 1);
		usleep_range(1000, 2000);
		gpiod_set_value_cansleep(priv->amp_reset, 0);
	}

	/* Hardware reset / enable for Audience ES705 DSP */
	priv->dsp_reset = devm_gpiod_get_optional(dev, "dsp-reset", GPIOD_OUT_LOW);
	if (IS_ERR(priv->dsp_reset)) {
		dev_warn(dev, "Failed to get dsp-reset GPIO: %ld\n", PTR_ERR(priv->dsp_reset));
		priv->dsp_reset = NULL;
	} else if (priv->dsp_reset) {
		gpiod_set_value_cansleep(priv->dsp_reset, 0);
		usleep_range(2000, 5000);
		gpiod_set_value_cansleep(priv->dsp_reset, 1);
		msleep(20);
	}

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

	cpu_np = of_parse_phandle(dev->of_node, "i2s-controller", 0);
	if (!cpu_np) {
		dev_err(dev, "i2s-controller property missing\n");
		return -EINVAL;
	}

	codec_np = of_parse_phandle(dev->of_node, "audio-codec", 0);
	if (!codec_np) {
		dev_err(dev, "audio-codec property missing\n");
		of_node_put(cpu_np);
		return -EINVAL;
	}

	/* Obtain I2C adapter to communicate with Audience ES705 on same bus */
	if (codec_np->parent) {
		priv->i2c_adap = of_find_i2c_adapter_by_node(codec_np->parent);
		if (priv->i2c_adap)
			m86_es705_enable_dhwpt(priv->i2c_adap);
	}

	card->dai_link[0].cpus->of_node = cpu_np;
	card->dai_link[0].platforms->of_node = cpu_np;
	card->dai_link[0].codecs->of_node = codec_np;

	amp_np = of_parse_phandle(dev->of_node, "audio-amplifier", 0);
	if (amp_np) {
		priv->amp_dev.dlc.of_node = amp_np;
		card->aux_dev = &priv->amp_dev;
		card->num_aux_devs = 1;
	}

	ret = devm_snd_soc_register_card(dev, card);
	if (ret < 0)
		dev_err_probe(dev, ret, "Failed to register card\n");

	of_node_put(cpu_np);
	of_node_put(codec_np);
	of_node_put(amp_np);

	return ret;
}

static int m86_audio_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	struct m86_machine_priv *priv = snd_soc_card_get_drvdata(card);

	if (priv && priv->i2c_adap)
		i2c_put_adapter(priv->i2c_adap);

	return 0;
}

static int m86_pm_prepare(struct device *dev)
{
	struct snd_soc_card *card = dev_get_drvdata(dev);

	return m86_stop_sysclk(card);
}

static void m86_pm_complete(struct device *dev)
{
	struct snd_soc_card *card = dev_get_drvdata(dev);

	m86_start_sysclk(card);
}

static const struct dev_pm_ops m86_pm_ops = {
	.prepare	= m86_pm_prepare,
	.suspend	= snd_soc_suspend,
	.resume		= snd_soc_resume,
	.complete	= m86_pm_complete,
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
	.remove	= m86_audio_remove,
};
module_platform_driver(m86_audio_driver);

MODULE_AUTHOR("Pair Programmer <kophapro@gmail.com>");
MODULE_DESCRIPTION("ALSA SoC Meizu PRO 5 Audio Support");
MODULE_LICENSE("GPL");
