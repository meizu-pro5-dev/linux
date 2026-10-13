// SPDX-License-Identifier: GPL-2.0+
/*
 * Samsung Exynos7420 Meizu PRO 5 (m86) ASoC machine driver
 *
 * Copyright (C) 2026 Antigravity Project
 * Author: Pair Programmer <kophapro@gmail.com>
 *
 * Based on sound/soc/samsung/tm2_wm5110.c and vendor meizu_audio_primary.c
 *
 * Audio path: AP I2S1 (slave) <-> Audience ES705 Port D <-> Port A <-> WM8998
 * AIF1 (master).  The ES705 is booted over SPI3 and left in Direct Hardware
 * Pass-Through (DHWPT); its firmware is loaded from /lib/firmware on the first
 * hw_params, so the root filesystem must be mounted before audio is used.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/gpio/consumer.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mfd/syscon.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include "i2s.h"
#include <linux/mfd/arizona/registers.h>
#include "../codecs/wm8998.h"

#define MCLK_RATE		32000000U

/* PMU DEBUG: CLKOUT_SEL[12:8], CLKOUT_DISABLE[0]; sel 0x10 = XXTI 24 MHz */
#define PMU_DEBUG_REG		0x0A00
#define PMU_DEBUG_CLKOUT_MASK	0x1f01
#define PMU_DEBUG_CLKOUT_XXTI	0x1000
#define PMU_DEBUG_CLKOUT_DIS	0x0001
#define PMU_RTC_CLKO_SEL_REG	0x001C
#define EXYNOS_RTCCON_REG	0x0040
#define EXYNOS_RTCCON_CLKOUTEN	(1 << 9)

#define ES_SYNC_ACK		0x80000000
#define ES_SPI_SBL_SYNC_CMD	0x80000000
#define ES_SPI_SBL_SYNC_ACK	0x8000FFFF
#define ES_SPI_SYNC_CMD		0x0000
#define ES_SPI_SYNC_ACK		0x0000
#define ES_SPI_BOOT_CMD		0x0001
#define ES_SPI_BOOT_ACK		0x0001
#define ES705_FW_NAME		"audience-es705-fw.bin"
/* ESCORE_SPI_PACKET_LEN: also the bounce buffer size */
#define ES705_SPI_PACKET	64

struct m86_machine_priv {
	struct snd_soc_component *component;
	struct regmap *pmu;
	struct clk *mclk;
	bool mclk_enabled;
	void __iomem *rtc_base;
	struct regulator *dsp_core;
	bool dsp_core_enabled;
	struct gpio_desc *amp_reset;
	struct gpio_desc *dsp_reset;
	struct gpio_desc *dsp_wakeup;
	struct spi_device *spi_dsp;
	/* Serialises ES705 SPI traffic and es705_booted */
	struct mutex es705_lock;
	u8 *spi_buf;		/* DMA-safe SPI bounce buffer */
	bool es705_booted;
	unsigned int sysclk_rate;
	unsigned int fll_rate;	/* FLL1 output currently programmed, 0 = off */
	struct snd_soc_aux_dev amp_dev;
};

static void m86_enable_32khz(struct m86_machine_priv *priv, bool on)
{
	u32 reg;
	int ret;

	if (priv->pmu) {
		ret = regmap_write(priv->pmu, PMU_RTC_CLKO_SEL_REG, 0x0);
		if (ret)
			pr_err("m86-audio: regmap_write PMU_RTC_CLKO_SEL_REG failed: %d\n", ret);
	}

	/*
	 * RTCCON is shared with rtc-s3c, which only rewrites it in its
	 * probe/suspend/resume paths; CLKOUTEN[9] is not touched there.
	 */
	if (priv->rtc_base) {
		reg = readl(priv->rtc_base + EXYNOS_RTCCON_REG);
		if (on)
			reg |= EXYNOS_RTCCON_CLKOUTEN;
		else
			reg &= ~EXYNOS_RTCCON_CLKOUTEN;
		writel(reg, priv->rtc_base + EXYNOS_RTCCON_REG);
	}
}

static void m86_enable_mclk(struct m86_machine_priv *priv, bool on)
{
	int ret;

	if (priv->pmu) {
		ret = regmap_update_bits(priv->pmu, PMU_DEBUG_REG,
					 PMU_DEBUG_CLKOUT_MASK,
					 on ? PMU_DEBUG_CLKOUT_XXTI :
					      PMU_DEBUG_CLKOUT_XXTI | PMU_DEBUG_CLKOUT_DIS);
		if (ret)
			pr_err("m86-audio: PMU_DEBUG update failed: %d\n", ret);
	}

	if (priv->mclk && on != priv->mclk_enabled) {
		if (on) {
			ret = clk_prepare_enable(priv->mclk);
			if (ret) {
				pr_err("m86-audio: mclk enable failed: %d\n", ret);
				return;
			}
		} else {
			clk_disable_unprepare(priv->mclk);
		}
		priv->mclk_enabled = on;
	}
}

static void m86_es705_init_hw(struct device *dev, struct m86_machine_priv *priv)
{
	int ret;

	/* RESET_N must not be released without CLK_IN present. */
	m86_enable_mclk(priv, true);
	m86_enable_32khz(priv, true);
	usleep_range(2000, 3000);

	/* Assert reset while cycling power */
	if (priv->dsp_reset)
		gpiod_set_value_cansleep(priv->dsp_reset, 0);

	/* VDD_CORE (LDO22 1.1V vdd11_nr): power cycle to ensure sequence */
	if (priv->dsp_core) {
		if (!priv->dsp_core_enabled) {
			ret = regulator_enable(priv->dsp_core);
			if (ret == 0)
				priv->dsp_core_enabled = true;
			else
				dev_warn(dev, "Failed to initially enable dsp-core: %d\n", ret);
		}
		if (priv->dsp_core_enabled) {
			ret = regulator_disable(priv->dsp_core);
			if (ret == 0)
				priv->dsp_core_enabled = false;
			else
				dev_warn(dev, "regulator_disable dsp-core returned %d\n", ret);
			usleep_range(10000, 12000);
			ret = regulator_enable(priv->dsp_core);
			if (ret == 0)
				priv->dsp_core_enabled = true;
			else
				dev_err(dev, "Failed to re-enable dsp-core: %d\n", ret);
		}
		usleep_range(5000, 6000);
	}

	/* Reset pulse: assert 5 ms, release, settle 20 ms */
	if (priv->dsp_reset) {
		gpiod_set_value_cansleep(priv->dsp_reset, 0);
		usleep_range(5000, 6000);
		gpiod_set_value_cansleep(priv->dsp_reset, 1);
		msleep(20);
	}

	if (priv->dsp_wakeup) {
		gpiod_set_value_cansleep(priv->dsp_wakeup, 1);
		usleep_range(1000, 2000);
	}
}

static int m86_es705_write(struct m86_machine_priv *priv, const void *data,
			   size_t len)
{
	if (WARN_ON(len > ES705_SPI_PACKET))
		return -EINVAL;
	memcpy(priv->spi_buf, data, len);
	return spi_write(priv->spi_dsp, priv->spi_buf, len);
}

static int m86_es705_read(struct m86_machine_priv *priv, void *data, size_t len)
{
	int ret;

	if (WARN_ON(len > ES705_SPI_PACKET))
		return -EINVAL;
	ret = spi_read(priv->spi_dsp, priv->spi_buf, len);
	if (ret == 0)
		memcpy(data, priv->spi_buf, len);
	return ret;
}

static int m86_es705_spi_cmd(struct m86_machine_priv *priv, u32 cmd, u32 *resp)
{
	struct device *dev = &priv->spi_dsp->dev;
	__be32 cmd_be = cpu_to_be32(cmd);
	__be16 resp16 = 0, resp16_lo = 0;
	bool no_resp = (cmd & BIT(28));
	int ret, retry = 60;

	*resp = 0;

	ret = m86_es705_write(priv, &cmd_be, sizeof(cmd_be));
	if (ret < 0) {
		dev_err(dev, "Failed to write SPI cmd 0x%08x: %d\n", cmd, ret);
		return ret;
	}

	if (no_resp) {
		/* Commands with BIT(28) set are write-only without chip response */
		usleep_range(3000, 5000);
		return 0;
	}

	usleep_range(1000, 1500);

	while (retry-- > 0) {
		ret = m86_es705_read(priv, &resp16, sizeof(resp16));
		if (ret < 0) {
			dev_err(dev, "Failed to read SPI cmd response: %d\n", ret);
			return ret;
		}

		if (resp16 != 0) {
			ret = m86_es705_read(priv, &resp16_lo, sizeof(resp16_lo));
			if (ret < 0) {
				dev_err(dev, "Failed to read SPI cmd response low: %d\n", ret);
				return ret;
			}

			*resp = ((u32)be16_to_cpu(resp16) << 16) | be16_to_cpu(resp16_lo);

			if ((*resp & 0xffff0000) == 0xffff0000) {
				dev_err(dev, "ES705 rejected cmd 0x%08x (resp 0x%08x)\n",
					cmd, *resp);
				return -EINVAL;
			}
			return 0;
		}

		usleep_range(1000, 1500);
	}

	dev_warn(dev, "ES705 cmd 0x%08x timed out waiting for response\n", cmd);
	return -ETIMEDOUT;
}

static const u32 m86_es705_dhwpt_cmds[] = {
	0x904e0000, /* smooth rate to 0 */
	0x903101f7, /* Preset 503: Port A slave */
	0x90310200, /* Preset 512: Port D slave */
	0x90310482, /* Preset 1154: Inactive DSP route */
	0x80620008, /* Stop voice call algo */
	0x805200f3, /* DHWPT: Port D to A, clock from Port D */
};

static int m86_es705_enable_dhwpt_sequence(struct m86_machine_priv *priv,
					   struct device *dev)
{
	u32 resp = 0;
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(m86_es705_dhwpt_cmds); i++) {
		ret = m86_es705_spi_cmd(priv, m86_es705_dhwpt_cmds[i], &resp);
		if (ret < 0) {
			dev_err(dev, "Failed to send ES705 DHWPT cmd 0x%08x: %d\n",
				m86_es705_dhwpt_cmds[i], ret);
			return ret;
		}
		dev_dbg(dev, "ES705 DHWPT cmd 0x%08x -> resp 0x%08x\n",
			m86_es705_dhwpt_cmds[i], resp);
		usleep_range(2000, 5000);
	}
	dev_info(dev, "ES705 DHWPT active (Port D <-> Port A pass-through)\n");
	return 0;
}

static int m86_es705_sbl_handshake(struct m86_machine_priv *priv,
				   struct device *dev)
{
	__be32 sync_cmd = cpu_to_be32(ES_SPI_SBL_SYNC_CMD);
	__be32 sync_ack = 0;
	__be16 sbl_sync_cmd = cpu_to_be16(ES_SPI_SYNC_CMD);
	__be16 sbl_sync_ack = 0;
	__be16 boot_cmd = cpu_to_be16(ES_SPI_BOOT_CMD);
	__be16 boot_ack = 0;
	int ret, retry, i;

	/* SBL sync: write 0x80000000, expect 0x8000FFFF */
	for (retry = 0; retry < 10; retry++) {
		sync_ack = 0;
		ret = m86_es705_write(priv, &sync_cmd, sizeof(sync_cmd));
		if (ret == 0) {
			for (i = 0; i < 5; i++) {
				ret = m86_es705_read(priv, &sync_ack, sizeof(sync_ack));
				if (ret == 0 && be32_to_cpu(sync_ack) == ES_SPI_SBL_SYNC_ACK)
					break;
				msleep(10);
			}
			if (be32_to_cpu(sync_ack) == ES_SPI_SBL_SYNC_ACK)
				break;
		}
		dev_dbg(dev, "ES705 SBL sync retry %d: read 0x%08x\n",
			retry, be32_to_cpu(sync_ack));
		msleep(10);
	}
	if (be32_to_cpu(sync_ack) != ES_SPI_SBL_SYNC_ACK) {
		dev_err(dev, "ES705 SBL sync failed: got 0x%08x, expected 0x%08x\n",
			be32_to_cpu(sync_ack), ES_SPI_SBL_SYNC_ACK);
		return -EIO;
	}

	/* SBL prepare: write 0x0000, wait 4 ms, expect 0x0000 */
	ret = m86_es705_write(priv, &sbl_sync_cmd, sizeof(sbl_sync_cmd));
	if (ret < 0)
		return ret;
	msleep(4);
	ret = m86_es705_read(priv, &sbl_sync_ack, sizeof(sbl_sync_ack));
	if (ret < 0 || be16_to_cpu(sbl_sync_ack) != ES_SPI_SYNC_ACK) {
		dev_err(dev, "ES705 SBL prepare ack mismatch: 0x%04x (ret %d)\n",
			be16_to_cpu(sbl_sync_ack), ret);
		return -EIO;
	}

	/* SBL boot: write 0x0001, wait 1 ms, expect 0x0001 */
	for (retry = 0; retry < 5; retry++) {
		ret = m86_es705_write(priv, &boot_cmd, sizeof(boot_cmd));
		if (ret == 0) {
			usleep_range(1000, 1500);
			ret = m86_es705_read(priv, &boot_ack, sizeof(boot_ack));
			if (ret == 0 && be16_to_cpu(boot_ack) == ES_SPI_BOOT_ACK)
				return 0;
		}
		usleep_range(1000, 1500);
	}
	dev_err(dev, "ES705 SBL boot ack mismatch: 0x%04x\n", be16_to_cpu(boot_ack));
	return -EIO;
}

static int m86_es705_boot_and_dhwpt(struct m86_machine_priv *priv,
				    struct device *dev)
{
	struct spi_device *spi = priv->spi_dsp;
	const struct firmware *fw;
	size_t offset, chunk;
	u32 resp = 0;
	int ret, retry;

	lockdep_assert_held(&priv->es705_lock);

	/* Fetch firmware before touching the DSP so a missing file is cheap. */
	ret = request_firmware(&fw, ES705_FW_NAME, dev);
	if (ret) {
		dev_err(dev, "ES705 firmware /lib/firmware/%s unavailable: %d\n",
			ES705_FW_NAME, ret);
		return ret;
	}

	/* SPI mode 1 (CPOL=0, CPHA=1), 4 MHz, 8-bit; matches the DT node */
	spi->mode = SPI_MODE_1;
	spi->max_speed_hz = 4000000;
	spi->bits_per_word = 8;
	ret = spi_setup(spi);
	if (ret < 0) {
		dev_err(dev, "Failed to setup SPI for ES705: %d\n", ret);
		goto out;
	}

	m86_es705_init_hw(dev, priv);

	ret = m86_es705_sbl_handshake(priv, dev);
	if (ret)
		goto out;

	for (offset = 0; offset < fw->size; offset += chunk) {
		chunk = min_t(size_t, fw->size - offset, ES705_SPI_PACKET);
		ret = m86_es705_write(priv, fw->data + offset, chunk);
		if (ret < 0) {
			dev_err(dev, "ES705 firmware write failed at %zu: %d\n",
				offset, ret);
			goto out;
		}
	}

	/* Wait for firmware to boot and sync */
	msleep(50);
	for (retry = 0; retry < 50; retry++) {
		ret = m86_es705_spi_cmd(priv, ES_SPI_SBL_SYNC_CMD, &resp);
		if (ret == 0 && resp == ES_SYNC_ACK)
			break;
		msleep(10);
	}
	if (resp != ES_SYNC_ACK) {
		dev_err(dev, "ES705 post-boot sync failed: resp=0x%08x\n", resp);
		ret = -EIO;
		goto out;
	}

	ret = m86_es705_enable_dhwpt_sequence(priv, dev);
	if (ret == 0) {
		priv->es705_booted = true;
		dev_info(dev, "ES705 booted with %zu-byte firmware\n", fw->size);
	}
out:
	release_firmware(fw);
	return ret;
}

/* Boot the ES705 once; later calls are no-ops until es705_booted is cleared. */
static int m86_es705_ensure(struct m86_machine_priv *priv, struct device *dev)
{
	int ret = 0;

	if (!priv->spi_dsp)
		return 0;

	mutex_lock(&priv->es705_lock);
	if (!priv->es705_booted)
		ret = m86_es705_boot_and_dhwpt(priv, dev);
	mutex_unlock(&priv->es705_lock);

	return ret;
}

static int m86_start_sysclk(struct snd_soc_card *card)
{
	struct m86_machine_priv *priv = snd_soc_card_get_drvdata(card);
	struct snd_soc_component *component = priv->component;
	int ret;

	if (!component)
		return -EAGAIN;

	/* Enable 24MHz MCLK and RTC 32kHz reference clocks */
	m86_enable_mclk(priv, true);
	m86_enable_32khz(priv, true);

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

	/* Force continuous BCLK and LRCLK in Master mode per vendor driver */
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
		if (card->dapm.bias_level == SND_SOC_BIAS_OFF)
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

	ret = m86_start_sysclk(rtd->card);
	if (ret)
		return ret;

	/* Without DHWPT the ES705 does not forward Port D to the codec. */
	return m86_es705_ensure(priv, rtd->card->dev);
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
		/* TXR=2 (I2S1) and one FLL1 serve both directions */
		.symmetric_rate	= 1,
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

	/* Keep reference clocks running */
	m86_enable_mclk(priv, true);
	m86_enable_32khz(priv, true);

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

	/* Route AIF1RX1 to HPOUTL and AIF1RX2 to HPOUTR with default volume */
	regmap_write(priv->component->regmap, ARIZONA_OUT1LMIX_INPUT_1_SOURCE, 0x20);
	regmap_write(priv->component->regmap, ARIZONA_OUT1RMIX_INPUT_1_SOURCE, 0x21);
	regmap_write(priv->component->regmap, ARIZONA_OUT1LMIX_INPUT_1_VOLUME, 0x80);
	regmap_write(priv->component->regmap, ARIZONA_OUT1RMIX_INPUT_1_VOLUME, 0x80);
	regmap_write(priv->component->regmap, ARIZONA_DAC_DIGITAL_VOLUME_1L, 0x280);
	regmap_write(priv->component->regmap, ARIZONA_DAC_DIGITAL_VOLUME_1R, 0x280);

	return 0;
}

static void m86_put_device(void *data)
{
	put_device(data);
}

static int m86_get_dsp(struct device *dev, struct m86_machine_priv *priv)
{
	struct device_node *dsp_np;
	struct device *dsp_dev;
	int ret;

	dsp_np = of_parse_phandle(dev->of_node, "samsung,dsp", 0);
	if (!dsp_np)
		return 0;

	dsp_dev = bus_find_device_by_of_node(&spi_bus_type, dsp_np);
	of_node_put(dsp_np);
	if (!dsp_dev)
		return -EPROBE_DEFER;

	ret = devm_add_action_or_reset(dev, m86_put_device, dsp_dev);
	if (ret)
		return ret;

	priv->spi_buf = devm_kzalloc(dev, ES705_SPI_PACKET, GFP_KERNEL);
	if (!priv->spi_buf)
		return -ENOMEM;

	priv->spi_dsp = to_spi_device(dsp_dev);
	return 0;
}

static void __iomem *m86_map_rtc(struct device *dev)
{
	struct device_node *np;
	struct resource res;
	void __iomem *base = NULL;

	np = of_parse_phandle(dev->of_node, "samsung,rtc", 0);
	if (!np)
		return NULL;
	/* rtc-s3c owns the region; map only, do not request it. */
	if (!of_address_to_resource(np, 0, &res))
		base = devm_ioremap(dev, res.start, resource_size(&res));
	of_node_put(np);
	return base;
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

	mutex_init(&priv->es705_lock);

	snd_soc_card_set_drvdata(card, priv);
	card->dev = dev;
	card->late_probe = m86_late_probe;

	ret = m86_get_dsp(dev, priv);
	if (ret)
		return dev_err_probe(dev, ret, "Audience ES705 SPI device unavailable\n");

	priv->pmu = syscon_regmap_lookup_by_phandle(dev->of_node, "samsung,pmu");
	if (IS_ERR(priv->pmu)) {
		dev_warn(dev, "Failed to get samsung,pmu: %ld\n", PTR_ERR(priv->pmu));
		priv->pmu = NULL;
	}

	priv->mclk = devm_clk_get_optional(dev, "mclk");
	if (IS_ERR(priv->mclk))
		priv->mclk = NULL;

	priv->rtc_base = m86_map_rtc(dev);
	if (!priv->rtc_base)
		dev_warn(dev, "RTC not mapped; 32 kHz clkout left unchanged\n");

	priv->dsp_core = devm_regulator_get_optional(dev, "dsp-core");
	if (IS_ERR(priv->dsp_core)) {
		dev_warn(dev, "Failed to get dsp-core supply: %ld\n", PTR_ERR(priv->dsp_core));
		priv->dsp_core = NULL;
	}

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

	/* Hardware reset and wakeup pins for Audience ES705 DSP */
	priv->dsp_reset = devm_gpiod_get_optional(dev, "dsp-reset", GPIOD_OUT_LOW);
	if (IS_ERR(priv->dsp_reset)) {
		dev_warn(dev, "Failed to get dsp-reset GPIO: %ld\n", PTR_ERR(priv->dsp_reset));
		priv->dsp_reset = NULL;
	}

	priv->dsp_wakeup = devm_gpiod_get_optional(dev, "dsp-wakeup", GPIOD_OUT_HIGH);
	if (IS_ERR(priv->dsp_wakeup))
		priv->dsp_wakeup = NULL;

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

static int m86_pm_prepare(struct device *dev)
{
	struct snd_soc_card *card = dev_get_drvdata(dev);
	struct m86_machine_priv *priv = snd_soc_card_get_drvdata(card);

	/* LDO22 may drop in suspend: re-download ES705 firmware on next use. */
	mutex_lock(&priv->es705_lock);
	priv->es705_booted = false;
	mutex_unlock(&priv->es705_lock);

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
MODULE_FIRMWARE(ES705_FW_NAME);
