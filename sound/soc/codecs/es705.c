// SPDX-License-Identifier: GPL-2.0+
/*
 * Audience ES705 Audio DSP driver (SPI control + DHWPT ASoC pass-through DAI)
 *
 * Boots the ES705 over SPI on first audio prepare so /lib/firmware is available
 * on the root filesystem, then places it into Direct Hardware Pass-Through
 * (Port D <-> Port A).
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/gpio/consumer.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/pm.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <sound/soc.h>

#define ES_SYNC_ACK		0x80000000
#define ES_SPI_SBL_SYNC_CMD	0x80000000
#define ES_SPI_SBL_SYNC_ACK	0x8000FFFF
#define ES_SPI_SYNC_CMD		0x0000
#define ES_SPI_SYNC_ACK		0x0000
#define ES_SPI_BOOT_CMD		0x0001
#define ES_SPI_BOOT_ACK		0x0001
#define ES705_FW_NAME		"audience-es705-fw.bin"

/*
 * Bounce buffer and firmware chunk size. Each spi_write() incurs ~5.8 ms
 * of PIO transfer overhead on s3c64xx SPI, so 4 KiB keeps boot ~1.1 s.
 */
#define ES705_SPI_PACKET	4096

#define ES705_RATES		SNDRV_PCM_RATE_8000_192000
#define ES705_FORMATS		(SNDRV_PCM_FMTBIT_S16_LE | \
				 SNDRV_PCM_FMTBIT_S20_3LE | \
				 SNDRV_PCM_FMTBIT_S24_LE | \
				 SNDRV_PCM_FMTBIT_S32_LE)

struct es705_priv {
	struct spi_device *spi;
	struct device *dev;
	struct clk *mclk;
	struct clk *clk32k;
	struct regulator *vdd_core;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *wakeup_gpio;
	struct mutex lock; /* Serializes SPI traffic and power/boot state */
	u8 *spi_buf;
	bool clks_on;
	bool vdd_on;
	bool booted;
};

static int es705_set_clocks(struct es705_priv *es705, bool on)
{
	int ret;

	if (on == es705->clks_on)
		return 0;

	if (on) {
		ret = clk_prepare_enable(es705->mclk);
		if (ret)
			return ret;
		ret = clk_prepare_enable(es705->clk32k);
		if (ret) {
			clk_disable_unprepare(es705->mclk);
			return ret;
		}
	} else {
		clk_disable_unprepare(es705->clk32k);
		clk_disable_unprepare(es705->mclk);
	}

	es705->clks_on = on;
	return 0;
}

static void es705_halt_locked(struct es705_priv *es705)
{
	es705->booted = false;

	if (es705->reset_gpio)
		gpiod_set_value_cansleep(es705->reset_gpio, 0);

	if (es705->vdd_on && es705->vdd_core) {
		regulator_disable(es705->vdd_core);
		es705->vdd_on = false;
	}

	es705_set_clocks(es705, false);
}

static int es705_init_hw(struct es705_priv *es705)
{
	struct device *dev = es705->dev;
	int ret;

	/* RESET_N must not be released without reference clocks present. */
	ret = es705_set_clocks(es705, true);
	if (ret) {
		dev_err(dev, "Failed to enable ES705 clocks: %d\n", ret);
		return ret;
	}
	usleep_range(2000, 3000);

	if (es705->reset_gpio)
		gpiod_set_value_cansleep(es705->reset_gpio, 0);

	if (es705->vdd_core) {
		if (!es705->vdd_on) {
			ret = regulator_enable(es705->vdd_core);
			if (ret == 0)
				es705->vdd_on = true;
			else
				dev_warn(dev, "Failed to enable vdd-core: %d\n", ret);
		}
		if (es705->vdd_on) {
			ret = regulator_disable(es705->vdd_core);
			if (ret == 0)
				es705->vdd_on = false;
			else
				dev_warn(dev, "Failed to disable vdd-core: %d\n", ret);
			usleep_range(10000, 12000);
			ret = regulator_enable(es705->vdd_core);
			if (ret == 0)
				es705->vdd_on = true;
			else
				dev_err(dev, "Failed to re-enable vdd-core: %d\n", ret);
		}
		usleep_range(5000, 6000);
	}

	if (es705->reset_gpio) {
		gpiod_set_value_cansleep(es705->reset_gpio, 0);
		usleep_range(5000, 6000);
		gpiod_set_value_cansleep(es705->reset_gpio, 1);
		msleep(20);
	}

	if (es705->wakeup_gpio) {
		gpiod_set_value_cansleep(es705->wakeup_gpio, 1);
		usleep_range(1000, 2000);
	}

	return 0;
}

static int es705_spi_write(struct es705_priv *es705, const void *data, size_t len)
{
	if (WARN_ON(len > ES705_SPI_PACKET))
		return -EINVAL;
	memcpy(es705->spi_buf, data, len);
	return spi_write(es705->spi, es705->spi_buf, len);
}

static int es705_spi_read(struct es705_priv *es705, void *data, size_t len)
{
	int ret;

	if (WARN_ON(len > ES705_SPI_PACKET))
		return -EINVAL;
	ret = spi_read(es705->spi, es705->spi_buf, len);
	if (ret == 0)
		memcpy(data, es705->spi_buf, len);
	return ret;
}

static int es705_spi_cmd(struct es705_priv *es705, u32 cmd, u32 *resp)
{
	struct device *dev = es705->dev;
	__be32 cmd_be = cpu_to_be32(cmd);
	__be16 resp16 = 0, resp16_lo = 0;
	bool no_resp = (cmd & BIT(28));
	int ret, retry = 60;

	*resp = 0;

	ret = es705_spi_write(es705, &cmd_be, sizeof(cmd_be));
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
		ret = es705_spi_read(es705, &resp16, sizeof(resp16));
		if (ret < 0) {
			dev_err(dev, "Failed to read SPI cmd response: %d\n", ret);
			return ret;
		}

		if (resp16 != 0) {
			ret = es705_spi_read(es705, &resp16_lo, sizeof(resp16_lo));
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

static const u32 es705_dhwpt_cmds[] = {
	0x904e0000, /* smooth rate to 0 */
	0x903101f7, /* Preset 503: Port A slave */
	0x90310200, /* Preset 512: Port D slave */
	0x90310482, /* Preset 1154: Inactive DSP route */
	0x80620008, /* Stop voice call algo */
	0x805200f3, /* DHWPT: Port D to A, clock from Port D */
};

static int es705_enable_dhwpt(struct es705_priv *es705)
{
	struct device *dev = es705->dev;
	u32 resp = 0;
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(es705_dhwpt_cmds); i++) {
		ret = es705_spi_cmd(es705, es705_dhwpt_cmds[i], &resp);
		if (ret < 0) {
			dev_err(dev, "Failed to send ES705 DHWPT cmd 0x%08x: %d\n",
				es705_dhwpt_cmds[i], ret);
			return ret;
		}
		dev_dbg(dev, "ES705 DHWPT cmd 0x%08x -> resp 0x%08x\n",
			es705_dhwpt_cmds[i], resp);
		usleep_range(2000, 5000);
	}

	dev_info(dev, "ES705 DHWPT active (Port D <-> Port A pass-through)\n");
	return 0;
}

static int es705_sbl_handshake(struct es705_priv *es705)
{
	struct device *dev = es705->dev;
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
		ret = es705_spi_write(es705, &sync_cmd, sizeof(sync_cmd));
		if (ret == 0) {
			for (i = 0; i < 5; i++) {
				ret = es705_spi_read(es705, &sync_ack, sizeof(sync_ack));
				if (ret == 0 && be32_to_cpu(sync_ack) == ES_SPI_SBL_SYNC_ACK)
					break;
				usleep_range(10000, 11000);
			}
			if (be32_to_cpu(sync_ack) == ES_SPI_SBL_SYNC_ACK)
				break;
		}
		dev_dbg(dev, "ES705 SBL sync retry %d: read 0x%08x\n",
			retry, be32_to_cpu(sync_ack));
		usleep_range(10000, 11000);
	}
	if (be32_to_cpu(sync_ack) != ES_SPI_SBL_SYNC_ACK) {
		dev_err(dev, "ES705 SBL sync failed: got 0x%08x, expected 0x%08x\n",
			be32_to_cpu(sync_ack), ES_SPI_SBL_SYNC_ACK);
		return -EIO;
	}

	/* SBL prepare: write 0x0000, wait 4 ms, expect 0x0000 */
	ret = es705_spi_write(es705, &sbl_sync_cmd, sizeof(sbl_sync_cmd));
	if (ret < 0)
		return ret;
	usleep_range(4000, 5000);
	ret = es705_spi_read(es705, &sbl_sync_ack, sizeof(sbl_sync_ack));
	if (ret < 0 || be16_to_cpu(sbl_sync_ack) != ES_SPI_SYNC_ACK) {
		dev_err(dev, "ES705 SBL prepare ack mismatch: 0x%04x (ret %d)\n",
			be16_to_cpu(sbl_sync_ack), ret);
		return -EIO;
	}

	/* SBL boot: write 0x0001, wait 1 ms, expect 0x0001 */
	for (retry = 0; retry < 5; retry++) {
		ret = es705_spi_write(es705, &boot_cmd, sizeof(boot_cmd));
		if (ret == 0) {
			usleep_range(1000, 1500);
			ret = es705_spi_read(es705, &boot_ack, sizeof(boot_ack));
			if (ret == 0 && be16_to_cpu(boot_ack) == ES_SPI_BOOT_ACK)
				return 0;
		}
		usleep_range(1000, 1500);
	}

	dev_err(dev, "ES705 SBL boot ack mismatch: 0x%04x\n", be16_to_cpu(boot_ack));
	return -EIO;
}

static int es705_boot_and_dhwpt(struct es705_priv *es705)
{
	struct spi_device *spi = es705->spi;
	struct device *dev = es705->dev;
	const struct firmware *fw;
	size_t offset, chunk;
	ktime_t start;
	u32 resp = 0;
	int ret, retry;

	lockdep_assert_held(&es705->lock);

	ret = request_firmware(&fw, ES705_FW_NAME, dev);
	if (ret) {
		dev_err(dev, "ES705 firmware /lib/firmware/%s unavailable: %d\n",
			ES705_FW_NAME, ret);
		return ret;
	}

	/* SPI mode 1 (CPOL=0, CPHA=1), 4 MHz, 8-bit */
	spi->mode = SPI_MODE_1;
	spi->max_speed_hz = 4000000;
	spi->bits_per_word = 8;
	ret = spi_setup(spi);
	if (ret < 0) {
		dev_err(dev, "Failed to setup SPI for ES705: %d\n", ret);
		goto err_fw;
	}

	start = ktime_get();
	ret = es705_init_hw(es705);
	if (ret)
		goto err_halt;

	ret = es705_sbl_handshake(es705);
	if (ret)
		goto err_halt;

	for (offset = 0; offset < fw->size; offset += chunk) {
		chunk = min_t(size_t, fw->size - offset, ES705_SPI_PACKET);
		ret = es705_spi_write(es705, fw->data + offset, chunk);
		if (ret < 0) {
			dev_err(dev, "ES705 firmware write failed at %zu: %d\n",
				offset, ret);
			goto err_halt;
		}
	}

	msleep(50);
	for (retry = 0; retry < 50; retry++) {
		ret = es705_spi_cmd(es705, ES_SPI_SBL_SYNC_CMD, &resp);
		if (ret == 0 && resp == ES_SYNC_ACK)
			break;
		usleep_range(10000, 11000);
	}
	if (resp != ES_SYNC_ACK) {
		dev_err(dev, "ES705 post-boot sync failed: resp=0x%08x\n", resp);
		ret = -EIO;
		goto err_halt;
	}

	ret = es705_enable_dhwpt(es705);
	if (ret)
		goto err_halt;

	es705->booted = true;
	dev_info(dev, "ES705 booted with %zu-byte firmware in %lld ms\n",
		 fw->size, ktime_ms_delta(ktime_get(), start));
	release_firmware(fw);
	return 0;

err_halt:
	es705_halt_locked(es705);
err_fw:
	release_firmware(fw);
	return ret;
}

static int es705_dai_prepare(struct snd_pcm_substream *substream,
			     struct snd_soc_dai *dai)
{
	struct es705_priv *es705 = snd_soc_component_get_drvdata(dai->component);
	int ret = 0;

	mutex_lock(&es705->lock);
	if (!es705->booted)
		ret = es705_boot_and_dhwpt(es705);
	mutex_unlock(&es705->lock);

	return ret;
}

static const struct snd_soc_dai_ops es705_dai_ops = {
	.prepare = es705_dai_prepare,
};

static struct snd_soc_dai_driver es705_dai = {
	.name = "es705-dhwpt",
	.playback = {
		.stream_name = "DHWPT Playback",
		.channels_min = 1,
		.channels_max = 6,
		.rates = ES705_RATES,
		.formats = ES705_FORMATS,
	},
	.capture = {
		.stream_name = "DHWPT Capture",
		.channels_min = 1,
		.channels_max = 6,
		.rates = ES705_RATES,
		.formats = ES705_FORMATS,
	},
	.ops = &es705_dai_ops,
};

static const struct snd_soc_dapm_widget es705_dapm_widgets[] = {
	SND_SOC_DAPM_INPUT("PORTA IN"),
	SND_SOC_DAPM_OUTPUT("PORTA OUT"),
};

static const struct snd_soc_dapm_route es705_dapm_routes[] = {
	{ "PORTA OUT", NULL, "DHWPT Playback" },
	{ "DHWPT Capture", NULL, "PORTA IN" },
};

static const struct snd_soc_component_driver es705_component_driver = {
	.dapm_widgets		= es705_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(es705_dapm_widgets),
	.dapm_routes		= es705_dapm_routes,
	.num_dapm_routes	= ARRAY_SIZE(es705_dapm_routes),
	.use_pmdown_time	= 1,
	.endianness		= 1,
};

static void es705_cleanup(void *data)
{
	struct es705_priv *es705 = data;

	mutex_lock(&es705->lock);
	es705_halt_locked(es705);
	mutex_unlock(&es705->lock);
}

static int es705_pm_prepare(struct device *dev)
{
	struct es705_priv *es705 = dev_get_drvdata(dev);

	mutex_lock(&es705->lock);
	es705_halt_locked(es705);
	mutex_unlock(&es705->lock);

	return 0;
}

static const struct dev_pm_ops es705_pm_ops = {
	.prepare = es705_pm_prepare,
};

static int es705_spi_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct es705_priv *es705;
	int ret;

	es705 = devm_kzalloc(dev, sizeof(*es705), GFP_KERNEL);
	if (!es705)
		return -ENOMEM;

	es705->spi = spi;
	es705->dev = dev;
	mutex_init(&es705->lock);
	spi_set_drvdata(spi, es705);

	es705->spi_buf = devm_kzalloc(dev, ES705_SPI_PACKET, GFP_KERNEL);
	if (!es705->spi_buf)
		return -ENOMEM;

	es705->mclk = devm_clk_get_optional(dev, "mclk");
	if (IS_ERR(es705->mclk))
		return dev_err_probe(dev, PTR_ERR(es705->mclk),
				     "failed to get mclk\n");

	es705->clk32k = devm_clk_get_optional(dev, "clk32k");
	if (IS_ERR(es705->clk32k))
		return dev_err_probe(dev, PTR_ERR(es705->clk32k),
				     "failed to get clk32k\n");

	es705->vdd_core = devm_regulator_get_optional(dev, "vdd-core");
	if (IS_ERR(es705->vdd_core))
		return dev_err_probe(dev, PTR_ERR(es705->vdd_core),
				     "failed to get vdd-core supply\n");

	es705->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(es705->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(es705->reset_gpio),
				     "failed to get reset GPIO\n");

	es705->wakeup_gpio = devm_gpiod_get_optional(dev, "wakeup", GPIOD_OUT_HIGH);
	if (IS_ERR(es705->wakeup_gpio))
		return dev_err_probe(dev, PTR_ERR(es705->wakeup_gpio),
				     "failed to get wakeup GPIO\n");

	ret = devm_add_action_or_reset(dev, es705_cleanup, es705);
	if (ret)
		return ret;

	return devm_snd_soc_register_component(dev, &es705_component_driver,
					       &es705_dai, 1);
}

static const struct of_device_id es705_of_match[] = {
	{ .compatible = "audience,es705" },
	{ }
};
MODULE_DEVICE_TABLE(of, es705_of_match);

static const struct spi_device_id es705_spi_id[] = {
	{ "es705", 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, es705_spi_id);

static struct spi_driver es705_spi_driver = {
	.driver = {
		.name		= "es705",
		.pm		= &es705_pm_ops,
		.of_match_table	= es705_of_match,
	},
	.probe		= es705_spi_probe,
	.id_table	= es705_spi_id,
};
module_spi_driver(es705_spi_driver);

MODULE_DESCRIPTION("Audience ES705 SPI Audio DSP driver");
MODULE_LICENSE("GPL");
MODULE_FIRMWARE(ES705_FW_NAME);
