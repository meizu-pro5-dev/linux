// SPDX-License-Identifier: GPL-2.0
/*
 * es9018k2m.c  --  ESS ES9018K2M ALSA SoC Audio driver for Meizu PRO 5
 *
 * Copyright 2015 Meizu Technology Co., Ltd.
 * Copyright 2026 Antigravity Meizu Mainline Team
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <sound/core.h>
#include <sound/initval.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/tlv.h>

#include "es9018k2m.h"

static const struct reg_default es9018k2m_reg_defaults[] = {
	{ 0, 0x00 },
	{ 1, 0x8c },
	{ 4, 0x00 },
	{ 5, 0x68 },
	{ 6, 0x42 },
	{ 7, 0x80 },
	{ 8, 0x10 },
	{ 9, 0x00 },
	{ 10, 0x05 },
	{ 11, 0x02 },
	{ 12, 0x5a },
	{ 13, 0x40 },
	{ 14, 0x8a },
	{ 15, 0x00 },
	{ 16, 0x00 },
	{ 17, 0xff },
	{ 18, 0xff },
	{ 19, 0xff },
	{ 20, 0x7f },
	{ 21, 0x00 },
	{ 26, 0x00 },
	{ 27, 0x00 },
	{ 28, 0x00 },
	{ 29, 0x00 },
	{ 30, 0x00 },
};

/* Flyme OS custom FIR filter coefficients (byte-identical to vendor driver) */
static const int es9018k2m_fir_coeff_stage1[128] = {
	3, -311, -598, -252, 793, 861, -804, -1836,
	409, 3051, 742, -4297, -2885, 5108, 6187, -4881,
	-10547, 2855, 15558, 1721, -20381, -9464, 23759, 20616,
	-24041, -34855, 19365, 51077, -7882, -67284, -11882, 80554,
	40653, -87165, -77990, 82861, 121942, -63283, -168781, 24498,
	212903, 36377, -246867, -120579, 261594, 227137, -246607, -352448,
	190163, 489915, -78901, -629544, -103690, 756883, 381738, -849479,
	-799238, 863537, 1456648, -673704, -2640570, -320086, 5331246, 8388607,
	5331246, -320086, -2640570, -673704, 1456648, 863537, -799238, -849479,
	381738, 756883, -103690, -629544, -78901, 489915, 190163, -352448,
	-246607, 227137, 261594, -120579, -246867, 36377, 212903, 24498,
	-168781, -63283, 121942, 82861, -77990, -87165, 40653, 80554,
	-11882, -67284, -7882, 51077, 19365, -34855, -24041, 20616,
	23759, -9464, -20381, 1721, 15558, 2855, -10547, -4881,
	6187, 5108, -2885, -4297, 742, 3051, 409, -1836,
	-804, 861, 793, -252, -598, -311, 3, 0
};

static const int es9018k2m_fir_coeff_stage2[16] = {
	0, 0, 2725, 16497, 60953, 171433, 393618, 773372,
	1332567, 2045216, 2828936, 3549914, 4060791, 4246290, 0, 0
};

struct es9018k2m_priv {
	struct device *dev;
	struct regmap *regmap;
	struct mutex lock;

	struct gpio_desc *osc48k_gpio;
	struct gpio_desc *osc44k_gpio;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *mute_gpio;
	struct gpio_desc *open_gpio;
	struct gpio_desc *low_gain_gpio;
	struct gpio_desc *out_select_gpio;
	struct gpio_desc *dvdd18_gpio;

	bool power_state;
	bool manual_power;
	bool stream_active;
	bool out_select;
	bool gain;		/* false = Low Gain, true = High Gain */
	unsigned int filter_mode;/* 0=Fast, 1=Slow, 2=Min Phase, 3=Meizu FIR */
	unsigned int sysclk;
	bool master;
	bool is_v_version;
};

static bool es9018k2m_readable(struct device *dev, unsigned int reg)
{
	if (reg <= ES9018K2M_CACHEREGNUM && reg != 2 && reg != 3)
		return true;
	if (reg == ES9018K2M_CHIP_STATUS)
		return true;
	if (reg >= 65 && reg <= 93)
		return true;
	return false;
}

static bool es9018k2m_writeable(struct device *dev, unsigned int reg)
{
	if (reg <= ES9018K2M_CACHEREGNUM && reg != 2 && reg != 3)
		return true;
	return false;
}

static bool es9018k2m_volatile(struct device *dev, unsigned int reg)
{
	if (reg == ES9018K2M_CHIP_STATUS || (reg >= 65 && reg <= 93))
		return true;
	return false;
}

static const struct regmap_config es9018k2m_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 93,
	.readable_reg = es9018k2m_readable,
	.writeable_reg = es9018k2m_writeable,
	.volatile_reg = es9018k2m_volatile,
	.cache_type = REGCACHE_RBTREE,
	.reg_defaults = es9018k2m_reg_defaults,
	.num_reg_defaults = ARRAY_SIZE(es9018k2m_reg_defaults),
};

static void es9018k2m_load_custom_fir(struct es9018k2m_priv *priv)
{
	int i;

	if (!priv->is_v_version)
		return;

	for (i = 0; i < 128; i++) {
		regmap_write(priv->regmap, ES9018K2M_PROGRAM_FIR_ADDR, i);
		regmap_write(priv->regmap, ES9018K2M_PROGRAM_FIR_DATA1,
			     es9018k2m_fir_coeff_stage1[i] & 0xff);
		regmap_write(priv->regmap, ES9018K2M_PROGRAM_FIR_DATA2,
			     (es9018k2m_fir_coeff_stage1[i] >> 8) & 0xff);
		regmap_write(priv->regmap, ES9018K2M_PROGRAM_FIR_DATAC,
			     (es9018k2m_fir_coeff_stage1[i] >> 16) & 0xff);
		regmap_write(priv->regmap, ES9018K2M_PROGRAM_FIR_CONTROL, 0x02);
	}

	for (i = 0; i < 16; i++) {
		regmap_write(priv->regmap, ES9018K2M_PROGRAM_FIR_ADDR, 0x80 + i);
		regmap_write(priv->regmap, ES9018K2M_PROGRAM_FIR_DATA1,
			     es9018k2m_fir_coeff_stage2[i] & 0xff);
		regmap_write(priv->regmap, ES9018K2M_PROGRAM_FIR_DATA2,
			     (es9018k2m_fir_coeff_stage2[i] >> 8) & 0xff);
		regmap_write(priv->regmap, ES9018K2M_PROGRAM_FIR_DATAC,
			     (es9018k2m_fir_coeff_stage2[i] >> 16) & 0xff);
		regmap_write(priv->regmap, ES9018K2M_PROGRAM_FIR_CONTROL, 0x02);
	}

	/* Enable custom FIR: bit 0 = enable, bit 2 = cosine stage 2 -> 0x05 */
	regmap_write(priv->regmap, ES9018K2M_PROGRAM_FIR_CONTROL, 0x05);
}

static void es9018k2m_apply_filter(struct es9018k2m_priv *priv)
{
	if (!priv->power_state)
		return;

	if (priv->filter_mode < 3) {
		regmap_write(priv->regmap, ES9018K2M_PROGRAM_FIR_CONTROL, 0x00);
		regmap_update_bits(priv->regmap, ES9018K2M_GENERAL_SET,
				   0x60, priv->filter_mode << 5);
	} else {
		es9018k2m_load_custom_fir(priv);
	}
}

/*
 * osc48k (gpf2-1) and osc44k (gpf2-3) share the same Samsung GPIO bank (GPF2DAT).
 * Because samsung_gpio_set_value() reads the physical pad pin state from GPF2DAT,
 * back-to-back writes to two pins in the same bank without a settling delay can
 * read the old pad voltage and clobber the first write. Always clear the inactive
 * oscillator first, wait 1ms, and then assert the active oscillator.
 */
static void es9018k2m_select_osc_locked(struct es9018k2m_priv *priv, bool use_44k)
{
	if (use_44k) {
		gpiod_set_value_cansleep(priv->osc48k_gpio, 0);
		usleep_range(1000, 1500);
		gpiod_set_value_cansleep(priv->osc44k_gpio, 1);
	} else {
		gpiod_set_value_cansleep(priv->osc44k_gpio, 0);
		usleep_range(1000, 1500);
		gpiod_set_value_cansleep(priv->osc48k_gpio, 1);
	}
	usleep_range(2000, 3000);
}

/*
 * On Meizu PRO 5, ES9018K2M VCCA/AVCC is OR-wired between low_gain_gpio (gpf1-0)
 * and ES9018K2M's internal GPIO1 (Reg 8: 0x07=LOW, 0x0f=HIGH).
 * To prevent dropping VCCA during gain transitions or reset:
 * - During reset/power-on (when Reg 8 resets to 0x10 input), low_gain_gpio MUST be 1.
 * - Switching to High Gain: drive Reg 8 = 0x0f (GPIO1 HIGH) BEFORE setting low_gain_gpio = 0.
 * - Switching to Low Gain: drive low_gain_gpio = 1 BEFORE setting Reg 8 = 0x07 (GPIO1 LOW).
 */
static void es9018k2m_apply_gain_locked(struct es9018k2m_priv *priv)
{
	if (!priv->power_state)
		return;

	if (priv->gain) {
		regmap_update_bits(priv->regmap, ES9018K2M_GPIO_CONFIG,
				   0x0f, 0x0f);
		usleep_range(1000, 1500);
		gpiod_set_value_cansleep(priv->low_gain_gpio, 0);
	} else {
		gpiod_set_value_cansleep(priv->low_gain_gpio, 1);
		usleep_range(1000, 1500);
		regmap_update_bits(priv->regmap, ES9018K2M_GPIO_CONFIG,
				   0x0f, 0x07);
	}
	usleep_range(1000, 1500);
}

/*
 * Must be called with priv->lock held.
 * Performs pop-free power-up / power-down of ES9018K2M + TI OPA1612.
 */
static int es9018k2m_hw_power_locked(struct es9018k2m_priv *priv, bool enable)
{
	bool use_44k = (priv->sysclk == 45158400);

	if (enable == priv->power_state)
		return 0;

	if (enable) {
		regcache_cache_only(priv->regmap, false);

		/* Mute analog switch and assert VCCA (low_gain_gpio=1) with bank settle delay */
		gpiod_set_value_cansleep(priv->mute_gpio, 1);
		usleep_range(1000, 1500);
		gpiod_set_value_cansleep(priv->low_gain_gpio, 1);
		usleep_range(1000, 1500);
		gpiod_set_value_cansleep(priv->dvdd18_gpio, 1);
		gpiod_set_value_cansleep(priv->open_gpio, 1);
		es9018k2m_select_osc_locked(priv, use_44k);

		/* Pulse RESETB: low -> 2ms -> high -> 10ms */
		gpiod_set_value_cansleep(priv->reset_gpio, 0);
		usleep_range(2000, 3000);
		gpiod_set_value_cansleep(priv->reset_gpio, 1);
		usleep_range(10000, 12000);

		priv->power_state = true;

		regcache_mark_dirty(priv->regmap);
		regcache_sync(priv->regmap);

		/* Configure hardware gain & GPIO1 output safely after reset */
		es9018k2m_apply_gain_locked(priv);

		/* Configure digital reconstruction filter */
		es9018k2m_apply_filter(priv);

		/* Soft start & unmute */
		regmap_update_bits(priv->regmap, ES9018K2M_SOFT_START, 0x80, 0x80);
		regmap_update_bits(priv->regmap, ES9018K2M_GENERAL_SET, 0x03, 0x00);
		usleep_range(5000, 6000);
		gpiod_set_value_cansleep(priv->mute_gpio, 0);
		usleep_range(1000, 1500);
	} else {
		/* Pop-free power-off sequence */
		gpiod_set_value_cansleep(priv->mute_gpio, 1);
		usleep_range(1000, 1500);
		regmap_update_bits(priv->regmap, ES9018K2M_GENERAL_SET, 0x03, 0x03);
		regmap_update_bits(priv->regmap, ES9018K2M_SOFT_START, 0x80, 0x00);
		usleep_range(5000, 6000);

		/* Return analog headphone switch to WM8998 before cutting OPA1612 rails */
		priv->out_select = false;
		gpiod_set_value_cansleep(priv->out_select_gpio, 0);
		usleep_range(5000, 6000);

		gpiod_set_value_cansleep(priv->reset_gpio, 0);
		gpiod_set_value_cansleep(priv->osc48k_gpio, 0);
		usleep_range(1000, 1500);
		gpiod_set_value_cansleep(priv->osc44k_gpio, 0);
		gpiod_set_value_cansleep(priv->low_gain_gpio, 0);
		usleep_range(1000, 1500);
		gpiod_set_value_cansleep(priv->open_gpio, 0);
		gpiod_set_value_cansleep(priv->dvdd18_gpio, 0);
		gpiod_set_value_cansleep(priv->mute_gpio, 0);
		usleep_range(1000, 1500);

		priv->power_state = false;
		regcache_mark_dirty(priv->regmap);
		regcache_cache_only(priv->regmap, true);
	}
	return 0;
}

static void es9018k2m_select_output_locked(struct es9018k2m_priv *priv, bool hifi)
{
	if (hifi == priv->out_select)
		return;

	gpiod_set_value_cansleep(priv->mute_gpio, 1);
	usleep_range(5000, 6000);
	priv->out_select = hifi;
	gpiod_set_value_cansleep(priv->out_select_gpio, hifi ? 1 : 0);
	usleep_range(5000, 6000);
	gpiod_set_value_cansleep(priv->mute_gpio, 0);
}

static const DECLARE_TLV_DB_SCALE(es9018k2m_vol_tlv, -12750, 50, 1);

static int es9018k2m_power_switch_get(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_soc_kcontrol_component(kcontrol);
	struct es9018k2m_priv *priv = snd_soc_component_get_drvdata(comp);

	ucontrol->value.integer.value[0] = priv->power_state ? 1 : 0;
	return 0;
}

static int es9018k2m_power_switch_put(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_soc_kcontrol_component(kcontrol);
	struct es9018k2m_priv *priv = snd_soc_component_get_drvdata(comp);
	bool enable = !!ucontrol->value.integer.value[0];
	bool changed = false;

	mutex_lock(&priv->lock);
	priv->manual_power = enable;
	if ((enable || priv->stream_active) != priv->power_state) {
		es9018k2m_hw_power_locked(priv, enable || priv->stream_active);
		changed = true;
	}
	mutex_unlock(&priv->lock);
	return changed ? 1 : 0;
}

static const char * const es9018k2m_path_texts[] = { "WM8998", "ES9018K2M HiFi" };
static SOC_ENUM_SINGLE_EXT_DECL(es9018k2m_path_enum, es9018k2m_path_texts);

static int es9018k2m_path_get(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_soc_kcontrol_component(kcontrol);
	struct es9018k2m_priv *priv = snd_soc_component_get_drvdata(comp);

	ucontrol->value.enumerated.item[0] = priv->out_select ? 1 : 0;
	return 0;
}

static int es9018k2m_path_put(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_soc_kcontrol_component(kcontrol);
	struct es9018k2m_priv *priv = snd_soc_component_get_drvdata(comp);
	bool val = !!ucontrol->value.enumerated.item[0];
	bool changed = false;

	mutex_lock(&priv->lock);
	if (val != priv->out_select) {
		es9018k2m_select_output_locked(priv, val);
		changed = true;
	}
	mutex_unlock(&priv->lock);
	return changed ? 1 : 0;
}

static const char * const es9018k2m_gain_texts[] = { "Low Gain", "High Gain" };
static SOC_ENUM_SINGLE_EXT_DECL(es9018k2m_gain_enum, es9018k2m_gain_texts);

static int es9018k2m_gain_get(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_soc_kcontrol_component(kcontrol);
	struct es9018k2m_priv *priv = snd_soc_component_get_drvdata(comp);

	ucontrol->value.enumerated.item[0] = priv->gain ? 1 : 0;
	return 0;
}

static int es9018k2m_gain_put(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_soc_kcontrol_component(kcontrol);
	struct es9018k2m_priv *priv = snd_soc_component_get_drvdata(comp);
	bool val = !!ucontrol->value.enumerated.item[0];
	bool changed = false;

	mutex_lock(&priv->lock);
	if (val != priv->gain) {
		priv->gain = val;
		if (priv->power_state) {
			gpiod_set_value_cansleep(priv->mute_gpio, 1);
			usleep_range(1000, 1500);
			regmap_update_bits(priv->regmap, ES9018K2M_GENERAL_SET,
					   0x03, 0x03);
			es9018k2m_apply_gain_locked(priv);
			usleep_range(5000, 6000);
			regmap_update_bits(priv->regmap, ES9018K2M_GENERAL_SET,
					   0x03, 0x00);
			gpiod_set_value_cansleep(priv->mute_gpio, 0);
			usleep_range(1000, 1500);
		}
		changed = true;
	}
	mutex_unlock(&priv->lock);
	return changed ? 1 : 0;
}

static const char * const es9018k2m_filter_texts[] = {
	"Fast Roll-Off", "Slow Roll-Off", "Minimum Phase", "Meizu Custom FIR"
};
static SOC_ENUM_SINGLE_EXT_DECL(es9018k2m_filter_enum, es9018k2m_filter_texts);

static int es9018k2m_filter_get(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_soc_kcontrol_component(kcontrol);
	struct es9018k2m_priv *priv = snd_soc_component_get_drvdata(comp);

	ucontrol->value.enumerated.item[0] = priv->filter_mode;
	return 0;
}

static int es9018k2m_filter_put(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_soc_kcontrol_component(kcontrol);
	struct es9018k2m_priv *priv = snd_soc_component_get_drvdata(comp);
	unsigned int val = ucontrol->value.enumerated.item[0];
	bool changed = false;

	if (val >= ARRAY_SIZE(es9018k2m_filter_texts))
		return -EINVAL;

	mutex_lock(&priv->lock);
	if (val != priv->filter_mode) {
		priv->filter_mode = val;
		es9018k2m_apply_filter(priv);
		changed = true;
	}
	mutex_unlock(&priv->lock);
	return changed ? 1 : 0;
}

static const struct snd_kcontrol_new es9018k2m_snd_controls[] = {
	SOC_SINGLE_BOOL_EXT("HiFi DAC Power Switch", 0,
			    es9018k2m_power_switch_get, es9018k2m_power_switch_put),
	SOC_ENUM_EXT("Headphone Path", es9018k2m_path_enum,
		     es9018k2m_path_get, es9018k2m_path_put),
	SOC_ENUM_EXT("HiFi Gain", es9018k2m_gain_enum,
		     es9018k2m_gain_get, es9018k2m_gain_put),
	SOC_ENUM_EXT("HiFi Filter", es9018k2m_filter_enum,
		     es9018k2m_filter_get, es9018k2m_filter_put),
	SOC_DOUBLE_R_TLV("HiFi Playback Volume",
			 ES9018K2M_VOLUME1, ES9018K2M_VOLUME2,
			 0, 255, 1, es9018k2m_vol_tlv),
	SOC_SINGLE_TLV("HiFi Left Volume", ES9018K2M_VOLUME1, 0, 255, 1, es9018k2m_vol_tlv),
	SOC_SINGLE_TLV("HiFi Right Volume", ES9018K2M_VOLUME2, 0, 255, 1, es9018k2m_vol_tlv),
};

static const struct snd_soc_component_driver soc_component_dev_es9018k2m = {
	.controls = es9018k2m_snd_controls,
	.num_controls = ARRAY_SIZE(es9018k2m_snd_controls),
};

static int es9018k2m_dai_startup(struct snd_pcm_substream *substream,
				 struct snd_soc_dai *dai)
{
	struct snd_soc_component *comp = dai->component;
	struct es9018k2m_priv *priv = snd_soc_component_get_drvdata(comp);

	mutex_lock(&priv->lock);
	priv->stream_active = true;
	es9018k2m_hw_power_locked(priv, true);
	mutex_unlock(&priv->lock);

	return 0;
}

static void es9018k2m_dai_shutdown(struct snd_pcm_substream *substream,
				   struct snd_soc_dai *dai)
{
	struct snd_soc_component *comp = dai->component;
	struct es9018k2m_priv *priv = snd_soc_component_get_drvdata(comp);

	mutex_lock(&priv->lock);
	priv->stream_active = false;
	if (!priv->manual_power)
		es9018k2m_hw_power_locked(priv, false);
	mutex_unlock(&priv->lock);
}

static int es9018k2m_set_dai_sysclk(struct snd_soc_dai *dai,
				    int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_component *comp = dai->component;
	struct es9018k2m_priv *priv = snd_soc_component_get_drvdata(comp);
	bool use_44k;

	if (freq != 49152000 && freq != 45158400) {
		dev_err(comp->dev, "Unsupported MCLK frequency: %u\n", freq);
		return -EINVAL;
	}

	use_44k = (freq == 45158400);

	mutex_lock(&priv->lock);
	if (priv->sysclk != freq) {
		priv->sysclk = freq;
		if (priv->power_state) {
			/* Re-sequence oscillator and reset on clock family switch */
			gpiod_set_value_cansleep(priv->mute_gpio, 1);
			usleep_range(1000, 1500);
			gpiod_set_value_cansleep(priv->low_gain_gpio, 1);
			usleep_range(1000, 1500);
			gpiod_set_value_cansleep(priv->reset_gpio, 0);
			es9018k2m_select_osc_locked(priv, use_44k);
			gpiod_set_value_cansleep(priv->reset_gpio, 1);
			usleep_range(10000, 12000);

			regcache_mark_dirty(priv->regmap);
			regcache_sync(priv->regmap);
			es9018k2m_apply_gain_locked(priv);
			es9018k2m_apply_filter(priv);
			regmap_update_bits(priv->regmap, ES9018K2M_SOFT_START,
					   0x80, 0x80);
			regmap_update_bits(priv->regmap, ES9018K2M_GENERAL_SET,
					   0x03, 0x00);
			usleep_range(5000, 6000);
			gpiod_set_value_cansleep(priv->mute_gpio, 0);
			usleep_range(1000, 1500);
		}
	}
	mutex_unlock(&priv->lock);

	return 0;
}

static int es9018k2m_set_dai_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct snd_soc_component *comp = dai->component;
	struct es9018k2m_priv *priv = snd_soc_component_get_drvdata(comp);
	unsigned int mode_reg;
	u8 iface = 0;
	u8 format = 0;

	switch (fmt & SND_SOC_DAIFMT_CLOCK_PROVIDER_MASK) {
	case SND_SOC_DAIFMT_CBP_CFP:
		iface |= 0x80;
		priv->master = true;
		break;
	case SND_SOC_DAIFMT_CBC_CFC:
		priv->master = false;
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		format &= ~0x30;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		format |= 0x10;
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		break;
	default:
		return -EINVAL;
	}

	mode_reg = priv->is_v_version ? ES9018K2M_V_MODE_CONTROL :
					ES9018K2M_W_MODE_CONTROL;

	mutex_lock(&priv->lock);
	regmap_update_bits(priv->regmap, mode_reg, 0x80, iface);
	regmap_update_bits(priv->regmap, ES9018K2M_INPUT_CONFIG, 0x30, format);
	mutex_unlock(&priv->lock);

	return 0;
}

static int es9018k2m_dai_hw_params(struct snd_pcm_substream *substream,
				   struct snd_pcm_hw_params *params,
				   struct snd_soc_dai *dai)
{
	struct snd_soc_component *comp = dai->component;
	struct es9018k2m_priv *priv = snd_soc_component_get_drvdata(comp);
	unsigned int rate = params_rate(params);
	unsigned int sysclk = (rate % 8000 == 0) ? 49152000 : 45158400;
	unsigned int bick_div = sysclk / rate / 64;
	unsigned int mode_reg;
	unsigned int iface = 0;
	u8 bick = 0;
	u8 word_len = 0;

	switch (params_width(params)) {
	case 16:
		word_len = 0x00;
		break;
	case 24:
		word_len = 0x40;
		break;
	case 32:
		word_len = 0x80;
		break;
	default:
		return -EINVAL;
	}

	if (priv->master) {
		switch (bick_div) {
		case 16:
			bick = 0x40;
			break;
		case 8:
			bick = 0x20;
			break;
		case 4:
			bick = 0x00;
			break;
		default:
			dev_err(comp->dev, "Unsupported bick_div %u for rate %u\n",
				bick_div, rate);
			return -EINVAL;
		}
		/* Vendor driver sets 32-bit I2S frame word length (0x80) in master mode */
		word_len = 0x80;
	}

	mode_reg = priv->is_v_version ? ES9018K2M_V_MODE_CONTROL :
					ES9018K2M_W_MODE_CONTROL;

	mutex_lock(&priv->lock);
	regmap_read(priv->regmap, ES9018K2M_INPUT_CONFIG, &iface);
	iface = (iface & 0x3f) | word_len;
	regmap_write(priv->regmap, ES9018K2M_INPUT_CONFIG, iface);

	if (priv->master)
		regmap_update_bits(priv->regmap, mode_reg, 0xe0, 0x80 | bick);

	regmap_update_bits(priv->regmap, ES9018K2M_SOFT_START, 0x80, 0x80);
	regmap_update_bits(priv->regmap, ES9018K2M_GENERAL_SET, 0x03, 0x00);

	/* Route analog headphone switch to ES9018K2M + OPA1612 */
	es9018k2m_select_output_locked(priv, true);
	mutex_unlock(&priv->lock);

	return 0;
}

static int es9018k2m_mute_stream(struct snd_soc_dai *dai, int mute, int direction)
{
	struct snd_soc_component *comp = dai->component;
	struct es9018k2m_priv *priv = snd_soc_component_get_drvdata(comp);

	mutex_lock(&priv->lock);
	if (priv->power_state) {
		regmap_update_bits(priv->regmap, ES9018K2M_GENERAL_SET,
				   0x03, mute ? 0x03 : 0x00);
	}
	mutex_unlock(&priv->lock);

	return 0;
}

static const struct snd_soc_dai_ops es9018k2m_dai_ops = {
	.startup	= es9018k2m_dai_startup,
	.shutdown	= es9018k2m_dai_shutdown,
	.set_sysclk	= es9018k2m_set_dai_sysclk,
	.set_fmt	= es9018k2m_set_dai_fmt,
	.hw_params	= es9018k2m_dai_hw_params,
	.mute_stream	= es9018k2m_mute_stream,
};

static struct snd_soc_dai_driver es9018k2m_dai = {
	.name = "ess9018k2m-hifi",
	.playback = {
		.stream_name = "HiFi Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_192000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE |
			   SNDRV_PCM_FMTBIT_S24_LE |
			   SNDRV_PCM_FMTBIT_S32_LE,
	},
	.ops = &es9018k2m_dai_ops,
	.symmetric_rate = 1,
};

static int es9018k2m_i2c_probe(struct i2c_client *i2c)
{
	struct device *dev = &i2c->dev;
	struct es9018k2m_priv *priv;
	unsigned int reg64 = 0;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = dev;
	mutex_init(&priv->lock);
	priv->filter_mode = 3; /* Default to Meizu Custom FIR */
	priv->sysclk = 49152000;
	priv->master = true;
	priv->is_v_version = true;
	i2c_set_clientdata(i2c, priv);

	/* Parse GPIOs */
	priv->osc48k_gpio = devm_gpiod_get_optional(dev, "osc48k", GPIOD_OUT_LOW);
	if (IS_ERR(priv->osc48k_gpio))
		return dev_err_probe(dev, PTR_ERR(priv->osc48k_gpio), "failed to get osc48k-gpio\n");

	priv->osc44k_gpio = devm_gpiod_get_optional(dev, "osc44k", GPIOD_OUT_LOW);
	if (IS_ERR(priv->osc44k_gpio))
		return dev_err_probe(dev, PTR_ERR(priv->osc44k_gpio), "failed to get osc44k-gpio\n");

	priv->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(priv->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(priv->reset_gpio), "failed to get reset-gpio\n");

	/* Keep mute_gpio deasserted (0) at idle so WM8998 headphone path is not muted */
	priv->mute_gpio = devm_gpiod_get_optional(dev, "mute", GPIOD_OUT_LOW);
	if (IS_ERR(priv->mute_gpio))
		return dev_err_probe(dev, PTR_ERR(priv->mute_gpio), "failed to get mute-gpio\n");

	priv->open_gpio = devm_gpiod_get_optional(dev, "open", GPIOD_OUT_LOW);
	if (IS_ERR(priv->open_gpio))
		return dev_err_probe(dev, PTR_ERR(priv->open_gpio), "failed to get open-gpio\n");

	priv->low_gain_gpio = devm_gpiod_get_optional(dev, "low-gain", GPIOD_OUT_LOW);
	if (IS_ERR(priv->low_gain_gpio))
		return dev_err_probe(dev, PTR_ERR(priv->low_gain_gpio), "failed to get low-gain-gpio\n");

	priv->out_select_gpio = devm_gpiod_get_optional(dev, "out-select", GPIOD_OUT_LOW);
	if (IS_ERR(priv->out_select_gpio))
		return dev_err_probe(dev, PTR_ERR(priv->out_select_gpio), "failed to get out-select-gpio\n");

	priv->dvdd18_gpio = devm_gpiod_get_optional(dev, "dvdd18", GPIOD_OUT_LOW);
	if (IS_ERR(priv->dvdd18_gpio))
		return dev_err_probe(dev, PTR_ERR(priv->dvdd18_gpio), "failed to get dvdd18-gpio\n");

	priv->regmap = devm_regmap_init_i2c(i2c, &es9018k2m_regmap_config);
	if (IS_ERR(priv->regmap))
		return dev_err_probe(dev, PTR_ERR(priv->regmap), "failed to init regmap\n");

	/* Briefly power on to detect chip ID (reg 64) */
	gpiod_set_value_cansleep(priv->low_gain_gpio, 1);
	usleep_range(1000, 1500);
	gpiod_set_value_cansleep(priv->dvdd18_gpio, 1);
	gpiod_set_value_cansleep(priv->open_gpio, 1);
	es9018k2m_select_osc_locked(priv, false);
	gpiod_set_value_cansleep(priv->reset_gpio, 0);
	usleep_range(2000, 3000);
	gpiod_set_value_cansleep(priv->reset_gpio, 1);
	usleep_range(10000, 12000);

	ret = regmap_read(priv->regmap, ES9018K2M_CHIP_STATUS, &reg64);
	if (ret == 0) {
		unsigned int rev = (reg64 >> 2) & 0x0f;
		priv->is_v_version = (rev == 0xc);
		dev_info(dev, "ES9018K2M Hi-Fi DAC detected (reg64=0x%02x rev=0x%x)\n", reg64, rev);
	} else {
		dev_warn(dev, "failed to read ES9018K2M reg64: %d\n", ret);
	}

	/* Power back down to standby, leaving 3.5mm jack unmuted on WM8998 */
	gpiod_set_value_cansleep(priv->reset_gpio, 0);
	gpiod_set_value_cansleep(priv->osc48k_gpio, 0);
	usleep_range(1000, 1500);
	gpiod_set_value_cansleep(priv->osc44k_gpio, 0);
	gpiod_set_value_cansleep(priv->low_gain_gpio, 0);
	usleep_range(1000, 1500);
	gpiod_set_value_cansleep(priv->open_gpio, 0);
	gpiod_set_value_cansleep(priv->dvdd18_gpio, 0);
	gpiod_set_value_cansleep(priv->out_select_gpio, 0);
	usleep_range(1000, 1500);
	gpiod_set_value_cansleep(priv->mute_gpio, 0);
	priv->power_state = false;
	priv->out_select = false;
	regcache_mark_dirty(priv->regmap);
	regcache_cache_only(priv->regmap, true);

	ret = devm_snd_soc_register_component(dev, &soc_component_dev_es9018k2m,
					      &es9018k2m_dai, 1);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register component\n");

	return 0;
}

static void es9018k2m_i2c_remove(struct i2c_client *i2c)
{
	struct es9018k2m_priv *priv = i2c_get_clientdata(i2c);

	mutex_lock(&priv->lock);
	if (priv->power_state)
		es9018k2m_hw_power_locked(priv, false);
	mutex_unlock(&priv->lock);
}

static const struct of_device_id es9018k2m_of_match[] = {
	{ .compatible = "ess,ess9018k2m" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, es9018k2m_of_match);

static const struct i2c_device_id es9018k2m_i2c_id[] = {
	{ "ess9018k2m", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, es9018k2m_i2c_id);

static struct i2c_driver es9018k2m_i2c_driver = {
	.driver = {
		.name = "ess9018k2m",
		.of_match_table = es9018k2m_of_match,
	},
	.probe = es9018k2m_i2c_probe,
	.remove = es9018k2m_i2c_remove,
	.id_table = es9018k2m_i2c_id,
};
module_i2c_driver(es9018k2m_i2c_driver);

MODULE_DESCRIPTION("ASoC ESS ES9018K2M Hi-Fi DAC driver for Meizu PRO 5");
MODULE_AUTHOR("Antigravity Meizu Mainline Team");
MODULE_LICENSE("GPL");
