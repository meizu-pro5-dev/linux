// SPDX-License-Identifier: GPL-2.0-only
/*
 * Texas Instruments LM3644 Dual Flash/Torch LED Driver
 *
 * Copyright (C) 2014-2015 MEIZU Technology Co., Ltd.
 * Copyright (C) 2026 Linux Mainline Porting Project
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/leds.h>
#include <linux/led-class-flash.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/slab.h>

#define LM3644_NAME			"leds-lm3644"
#define LM3644_ADDR			0x63

/* Registers */
#define LM3644_REG_ENABLE		0x01
#define LM3644_REG_IVFM			0x02
#define LM3644_REG_FLASH_LED0_BR	0x03
#define LM3644_REG_FLASH_LED1_BR	0x04
#define LM3644_REG_TORCH_LED0_BR	0x05
#define LM3644_REG_TORCH_LED1_BR	0x06
#define LM3644_REG_FLASH_TOUT		0x08
#define LM3644_REG_TIMING		0x08
#define LM3644_REG_TEMP			0x09
#define LM3644_REG_FLAG0		0x0A
#define LM3644_REG_FLAGS1		0x0A
#define LM3644_REG_FLAG1		0x0B
#define LM3644_REG_FLAGS2		0x0B
#define LM3644_REG_DEV_ID		0x0C

/* LM3644_REG_ENABLE (0x01) */
#define LM3644_LED0_EN			BIT(0)
#define LM3644_LED1_EN			BIT(1)
#define LM3644_MODE_MASK		(BIT(2) | BIT(3))
#define LM3644_MODE_STANDBY		(0 << 2)
#define LM3644_MODE_IR			(1 << 2)
#define LM3644_MODE_TORCH		(2 << 2)
#define LM3644_MODE_FLASH		(3 << 2)
#define LM3644_TORCH_NTC_EN		BIT(4)
#define LM3644_STROBE_EN		BIT(5)
#define LM3644_TX_EN			BIT(7)

/* LM3644_REG_IVFM (0x02) */
#define LM3644_UVLO_EN			BIT(6)

/* LM3644_REG_FLASH_TOUT (0x08) */
#define LM3644_FLASH_TOUT_MAX		0x0F

/* Current limits */
#define LM3644_TORCH_MAX_LEVEL		0x7F
#define LM3644_FLASH_MAX_LEVEL		0x7F

enum lm3644_channel_id {
	LM3644_ID_TORCH0 = 0,
	LM3644_ID_TORCH1,
	LM3644_ID_FLASH0,
	LM3644_ID_FLASH1,
	LM3644_ID_MAX
};

struct lm3644_chip;

struct lm3644_led {
	struct lm3644_chip *chip;
	struct led_classdev_flash fled_cdev;
	enum lm3644_channel_id id;
	int led_index;	/* 0 = LED0 (cool white), 1 = LED1 (warm white) */
	bool is_flash;
};

#define to_lm3644_led(c) container_of(c, struct lm3644_led, fled_cdev.led_cdev)

struct lm3644_chip {
	struct device *dev;
	struct regmap *regmap;
	struct mutex lock;
	struct gpio_desc *hwen_gpio;
	struct gpio_desc *strobe_gpio;
	bool hw_enabled;
	u8 torch_en_mask;
	u8 flash_en_mask;
	struct lm3644_led leds[LM3644_ID_MAX];
};

static const struct regmap_config lm3644_regmap_cfg = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0x0F,
};

static u8 lm3644_timeout_to_reg(u32 timeout_us)
{
	if (timeout_us <= 100000) {
		if (timeout_us < 10000)
			return 0;
		return (timeout_us / 10000) - 1;
	}
	if (timeout_us >= 400000)
		return 0x0f;
	return 9 + (timeout_us - 100000 + 49999) / 50000;
}

static int lm3644_hw_init(struct lm3644_chip *chip)
{
	int ret;

	/* Enable UVLO (under-voltage lockout) */
	ret = regmap_update_bits(chip->regmap, LM3644_REG_IVFM,
				 LM3644_UVLO_EN, LM3644_UVLO_EN);
	if (ret)
		return ret;

	/* Set flash timeout to default (150ms -> 0x0a) */
	ret = regmap_update_bits(chip->regmap, LM3644_REG_FLASH_TOUT,
				 0x0F, lm3644_timeout_to_reg(150000));
	if (ret)
		return ret;

	/* Enable TORCH_NTC and STROBE pins, disable TX pin, standby mode */
	ret = regmap_write(chip->regmap, LM3644_REG_ENABLE,
			   LM3644_TORCH_NTC_EN | LM3644_STROBE_EN | LM3644_MODE_STANDBY);
	if (ret)
		return ret;

	/* Initialize brightness registers to safe baseline */
	regmap_write(chip->regmap, LM3644_REG_TORCH_LED0_BR, 0x20);
	regmap_write(chip->regmap, LM3644_REG_TORCH_LED1_BR, 0x20);
	regmap_write(chip->regmap, LM3644_REG_FLASH_LED0_BR, 0x40);
	regmap_write(chip->regmap, LM3644_REG_FLASH_LED1_BR, 0x40);

	return 0;
}

static int lm3644_set_hw_power(struct lm3644_chip *chip, bool on)
{
	int ret;

	if (on) {
		if (chip->hw_enabled)
			return 0;
		if (chip->hwen_gpio) {
			gpiod_set_value_cansleep(chip->hwen_gpio, 1);
			usleep_range(1500, 2500);
		}
		ret = lm3644_hw_init(chip);
		if (ret) {
			if (chip->hwen_gpio)
				gpiod_set_value_cansleep(chip->hwen_gpio, 0);
			return ret;
		}
		chip->hw_enabled = true;
	} else {
		if (!chip->hw_enabled)
			return 0;
		if (chip->hwen_gpio) {
			gpiod_set_value_cansleep(chip->hwen_gpio, 0);
			chip->hw_enabled = false;
		}
	}

	return 0;
}

static int lm3644_brightness_set_blocking(struct led_classdev *cdev,
					  enum led_brightness brightness)
{
	struct lm3644_led *led = to_lm3644_led(cdev);
	struct lm3644_chip *chip = led->chip;
	u8 reg_br, led_bit;
	int ret = 0;

	mutex_lock(&chip->lock);

	led_bit = (led->led_index == 0) ? LM3644_LED0_EN : LM3644_LED1_EN;

	if (brightness > 0) {
		ret = lm3644_set_hw_power(chip, true);
		if (ret)
			goto out;
	} else if (!chip->hw_enabled) {
		chip->flash_en_mask &= ~led_bit;
		chip->torch_en_mask &= ~led_bit;
		goto out;
	}

	if (led->is_flash) {
		reg_br = (led->led_index == 0) ?
			 LM3644_REG_FLASH_LED0_BR : LM3644_REG_FLASH_LED1_BR;

		if (brightness > 0) {
			u8 val = min_t(u32, brightness, LM3644_FLASH_MAX_LEVEL);

			ret = regmap_write(chip->regmap, reg_br, val);
			if (ret)
				goto out;

			chip->flash_en_mask |= led_bit;
			ret = regmap_update_bits(chip->regmap, LM3644_REG_ENABLE,
						 LM3644_LED0_EN | LM3644_LED1_EN | LM3644_MODE_MASK,
						 chip->flash_en_mask | LM3644_MODE_FLASH);
		} else {
			chip->flash_en_mask &= ~led_bit;
			if (chip->flash_en_mask == 0) {
				if (chip->torch_en_mask > 0) {
					ret = regmap_update_bits(chip->regmap, LM3644_REG_ENABLE,
								 LM3644_LED0_EN | LM3644_LED1_EN | LM3644_MODE_MASK,
								 chip->torch_en_mask | LM3644_MODE_TORCH);
				} else {
					ret = regmap_update_bits(chip->regmap, LM3644_REG_ENABLE,
								 LM3644_LED0_EN | LM3644_LED1_EN | LM3644_MODE_MASK,
								 LM3644_MODE_STANDBY);
					lm3644_set_hw_power(chip, false);
				}
			} else {
				ret = regmap_update_bits(chip->regmap, LM3644_REG_ENABLE,
							 LM3644_LED0_EN | LM3644_LED1_EN,
							 chip->flash_en_mask);
			}
		}
	} else {
		reg_br = (led->led_index == 0) ?
			 LM3644_REG_TORCH_LED0_BR : LM3644_REG_TORCH_LED1_BR;

		if (brightness > 0) {
			u8 val = min_t(u32, brightness, LM3644_TORCH_MAX_LEVEL);

			ret = regmap_write(chip->regmap, reg_br, val);
			if (ret)
				goto out;

			chip->torch_en_mask |= led_bit;
			ret = regmap_update_bits(chip->regmap, LM3644_REG_ENABLE,
						 LM3644_LED0_EN | LM3644_LED1_EN | LM3644_MODE_MASK,
						 chip->torch_en_mask | LM3644_MODE_TORCH);
		} else {
			chip->torch_en_mask &= ~led_bit;
			if (chip->torch_en_mask == 0) {
				if (chip->flash_en_mask > 0) {
					ret = regmap_update_bits(chip->regmap, LM3644_REG_ENABLE,
								 LM3644_LED0_EN | LM3644_LED1_EN | LM3644_MODE_MASK,
								 chip->flash_en_mask | LM3644_MODE_FLASH);
				} else {
					ret = regmap_update_bits(chip->regmap, LM3644_REG_ENABLE,
								 LM3644_LED0_EN | LM3644_LED1_EN | LM3644_MODE_MASK,
								 LM3644_MODE_STANDBY);
					lm3644_set_hw_power(chip, false);
				}
			} else {
				ret = regmap_update_bits(chip->regmap, LM3644_REG_ENABLE,
							 LM3644_LED0_EN | LM3644_LED1_EN,
							 chip->torch_en_mask);
			}
		}
	}

out:
	mutex_unlock(&chip->lock);
	return ret;
}

static int lm3644_flash_brightness_set(struct led_classdev_flash *fled_cdev, u32 brightness)
{
	struct lm3644_led *led = container_of(fled_cdev, struct lm3644_led, fled_cdev);
	struct lm3644_chip *chip = led->chip;
	u8 reg_br;
	int ret = 0;

	fled_cdev->brightness.val = brightness;

	mutex_lock(&chip->lock);
	if (chip->hw_enabled && (chip->flash_en_mask & ((led->led_index == 0) ? LM3644_LED0_EN : LM3644_LED1_EN))) {
		reg_br = (led->led_index == 0) ?
			 LM3644_REG_FLASH_LED0_BR : LM3644_REG_FLASH_LED1_BR;
		ret = regmap_write(chip->regmap, reg_br, min_t(u32, brightness, LM3644_FLASH_MAX_LEVEL));
	}
	mutex_unlock(&chip->lock);

	return ret;
}

static int lm3644_flash_strobe_set(struct led_classdev_flash *fled_cdev, bool state)
{
	return lm3644_brightness_set_blocking(&fled_cdev->led_cdev,
					      state ? fled_cdev->brightness.val : 0);
}

static int lm3644_flash_strobe_get(struct led_classdev_flash *fled_cdev, bool *state)
{
	struct lm3644_led *led = container_of(fled_cdev, struct lm3644_led, fled_cdev);
	struct lm3644_chip *chip = led->chip;
	u8 led_bit = (led->led_index == 0) ? LM3644_LED0_EN : LM3644_LED1_EN;

	mutex_lock(&chip->lock);
	*state = !!(chip->flash_en_mask & led_bit);
	mutex_unlock(&chip->lock);

	return 0;
}

static int lm3644_flash_timeout_set(struct led_classdev_flash *fled_cdev, u32 timeout)
{
	struct lm3644_led *led = container_of(fled_cdev, struct lm3644_led, fled_cdev);
	struct lm3644_chip *chip = led->chip;
	u8 reg_val;
	int ret;

	reg_val = lm3644_timeout_to_reg(timeout);

	mutex_lock(&chip->lock);
	ret = lm3644_set_hw_power(chip, true);
	if (ret) {
		mutex_unlock(&chip->lock);
		return ret;
	}
	ret = regmap_update_bits(chip->regmap, LM3644_REG_TIMING, 0x0F, reg_val);
	if (!chip->flash_en_mask && !chip->torch_en_mask)
		lm3644_set_hw_power(chip, false);
	mutex_unlock(&chip->lock);

	if (!ret)
		fled_cdev->timeout.val = timeout;
	return ret;
}

static int lm3644_flash_fault_get(struct led_classdev_flash *fled_cdev, u32 *fault)
{
	struct lm3644_led *led = container_of(fled_cdev, struct lm3644_led, fled_cdev);
	struct lm3644_chip *chip = led->chip;
	unsigned int flag0 = 0, flag1 = 0;
	int ret = 0;

	*fault = 0;

	mutex_lock(&chip->lock);
	if (chip->hw_enabled) {
		ret = regmap_read(chip->regmap, LM3644_REG_FLAGS1, &flag0);
		if (!ret)
			ret = regmap_read(chip->regmap, LM3644_REG_FLAGS2, &flag1);
	}
	mutex_unlock(&chip->lock);

	if (ret)
		return ret;

	if (flag0 & BIT(0))
		*fault |= LED_FAULT_TIMEOUT;
	if (flag0 & BIT(1))
		*fault |= LED_FAULT_UNDER_VOLTAGE;
	if (flag0 & BIT(2))
		*fault |= LED_FAULT_OVER_TEMPERATURE;
	if (flag0 & BIT(3))
		*fault |= LED_FAULT_LED_OVER_TEMPERATURE;
	if (flag0 & (BIT(4) | BIT(5)))
		*fault |= LED_FAULT_SHORT_CIRCUIT;
	if (flag0 & BIT(6))
		*fault |= LED_FAULT_OVER_VOLTAGE;
	if (flag0 & BIT(7))
		*fault |= LED_FAULT_INPUT_VOLTAGE;

	return 0;
}

static const struct led_flash_ops lm3644_flash_ops = {
	.flash_brightness_set = lm3644_flash_brightness_set,
	.strobe_set = lm3644_flash_strobe_set,
	.strobe_get = lm3644_flash_strobe_get,
	.timeout_set = lm3644_flash_timeout_set,
	.fault_get = lm3644_flash_fault_get,
};

static ssize_t flags_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct lm3644_chip *chip = dev_get_drvdata(dev);
	unsigned int flag0 = 0, flag1 = 0;
	int ret = 0;

	mutex_lock(&chip->lock);
	if (chip->hw_enabled) {
		ret = regmap_read(chip->regmap, LM3644_REG_FLAGS1, &flag0);
		if (!ret)
			ret = regmap_read(chip->regmap, LM3644_REG_FLAGS2, &flag1);
	}
	mutex_unlock(&chip->lock);

	if (ret)
		return ret;

	return sysfs_emit(buf, "flag0=0x%02x flag1=0x%02x\n", flag0, flag1);
}
static DEVICE_ATTR_RO(flags);

static struct attribute *lm3644_attrs[] = {
	&dev_attr_flags.attr,
	NULL
};
ATTRIBUTE_GROUPS(lm3644);

static const struct {
	const char *name;
	const char *default_trigger;
	int led_index;
	bool is_flash;
	u32 max_brightness;
} lm3644_led_defs[LM3644_ID_MAX] = {
	[LM3644_ID_TORCH0] = {
		.name = "torch0",
		.default_trigger = "torch0",
		.led_index = 0,
		.is_flash = false,
		.max_brightness = LM3644_TORCH_MAX_LEVEL,
	},
	[LM3644_ID_TORCH1] = {
		.name = "torch1",
		.default_trigger = "torch1",
		.led_index = 1,
		.is_flash = false,
		.max_brightness = LM3644_TORCH_MAX_LEVEL,
	},
	[LM3644_ID_FLASH0] = {
		.name = "flash0",
		.default_trigger = "flash0",
		.led_index = 0,
		.is_flash = true,
		.max_brightness = LM3644_FLASH_MAX_LEVEL,
	},
	[LM3644_ID_FLASH1] = {
		.name = "flash1",
		.default_trigger = "flash1",
		.led_index = 1,
		.is_flash = true,
		.max_brightness = LM3644_FLASH_MAX_LEVEL,
	},
};

static int lm3644_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct lm3644_chip *chip;
	unsigned int dev_id = 0;
	int i, ret;

	chip = devm_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->dev = dev;
	mutex_init(&chip->lock);
	i2c_set_clientdata(client, chip);

	chip->regmap = devm_regmap_init_i2c(client, &lm3644_regmap_cfg);
	if (IS_ERR(chip->regmap)) {
		ret = PTR_ERR(chip->regmap);
		dev_err(dev, "Failed to allocate register map: %d\n", ret);
		return ret;
	}

	/* HWEN hardware enable GPIO: active high */
	chip->hwen_gpio = devm_gpiod_get_optional(dev, "hwen", GPIOD_OUT_HIGH);
	if (IS_ERR(chip->hwen_gpio))
		return dev_err_probe(dev, PTR_ERR(chip->hwen_gpio),
				      "Failed to get hwen-gpios\n");

	if (chip->hwen_gpio) {
		gpiod_set_value_cansleep(chip->hwen_gpio, 1);
		usleep_range(1500, 2500);
	}

	/* STROBE trigger GPIO: active low initially */
	chip->strobe_gpio = devm_gpiod_get_optional(dev, "strobe", GPIOD_OUT_LOW);
	if (IS_ERR(chip->strobe_gpio))
		return dev_err_probe(dev, PTR_ERR(chip->strobe_gpio),
				      "Failed to get strobe-gpios\n");

	/* Check device ID */
	ret = regmap_read(chip->regmap, LM3644_REG_DEV_ID, &dev_id);
	if (ret) {
		dev_err(dev, "Failed to read LM3644 device ID: %d\n", ret);
		if (chip->hwen_gpio)
			gpiod_set_value_cansleep(chip->hwen_gpio, 0);
		return ret;
	}

	ret = lm3644_hw_init(chip);
	if (ret) {
		dev_err(dev, "Failed to initialize LM3644 hardware: %d\n", ret);
		if (chip->hwen_gpio)
			gpiod_set_value_cansleep(chip->hwen_gpio, 0);
		return ret;
	}

	/* Keep hardware disabled when all LEDs are off (matching 3.10 vendor driver) */
	if (chip->hwen_gpio) {
		gpiod_set_value_cansleep(chip->hwen_gpio, 0);
		chip->hw_enabled = false;
	} else {
		chip->hw_enabled = true;
	}

	dev_info(dev, "TI LM3644 Flash/Torch driver probed (dev_id=0x%02x)\n", dev_id);

	/* Register LEDs: torch0/1 as led_classdev, flash0/1 as led_classdev_flash */
	for (i = 0; i < LM3644_ID_MAX; i++) {
		struct lm3644_led *led = &chip->leds[i];

		led->chip = chip;
		led->id = i;
		led->led_index = lm3644_led_defs[i].led_index;
		led->is_flash = lm3644_led_defs[i].is_flash;

		led->fled_cdev.led_cdev.name = lm3644_led_defs[i].name;
		led->fled_cdev.led_cdev.default_trigger = lm3644_led_defs[i].default_trigger;
		led->fled_cdev.led_cdev.max_brightness = lm3644_led_defs[i].max_brightness;
		led->fled_cdev.led_cdev.brightness = 0;
		led->fled_cdev.led_cdev.brightness_set_blocking = lm3644_brightness_set_blocking;

		if (led->is_flash) {
			led->fled_cdev.ops = &lm3644_flash_ops;
			led->fled_cdev.led_cdev.flags |= LED_DEV_CAP_FLASH;

			led->fled_cdev.timeout.min = 10000;
			led->fled_cdev.timeout.max = 400000;
			led->fled_cdev.timeout.step = 10000;
			led->fled_cdev.timeout.val = 150000;

			led->fled_cdev.brightness.min = 1;
			led->fled_cdev.brightness.max = LM3644_FLASH_MAX_LEVEL;
			led->fled_cdev.brightness.step = 1;
			led->fled_cdev.brightness.val = LM3644_FLASH_MAX_LEVEL;

			ret = devm_led_classdev_flash_register_ext(dev, &led->fled_cdev, NULL);
			if (ret) {
				dev_err(dev, "Failed to register flash LED %s: %d\n",
					led->fled_cdev.led_cdev.name, ret);
				return ret;
			}
		} else {
			ret = devm_led_classdev_register(dev, &led->fled_cdev.led_cdev);
			if (ret) {
				dev_err(dev, "Failed to register torch LED %s: %d\n",
					led->fled_cdev.led_cdev.name, ret);
				return ret;
			}
		}
	}

	return 0;
}

static void lm3644_remove(struct i2c_client *client)
{
	struct lm3644_chip *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->lock);
	if (chip->hw_enabled) {
		regmap_write(chip->regmap, LM3644_REG_ENABLE, LM3644_MODE_STANDBY);
		lm3644_set_hw_power(chip, false);
	}
	mutex_unlock(&chip->lock);
}

static const struct of_device_id lm3644_of_match[] = {
	{ .compatible = "ti,leds-lm3644" },
	{ .compatible = "ti,lm3644" },
	{ }
};
MODULE_DEVICE_TABLE(of, lm3644_of_match);

static const struct i2c_device_id lm3644_id[] = {
	{ "lm3644", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, lm3644_id);

static struct i2c_driver lm3644_driver = {
	.driver = {
		.name = LM3644_NAME,
		.of_match_table = lm3644_of_match,
		.dev_groups = lm3644_groups,
	},
	.probe = lm3644_probe,
	.remove = lm3644_remove,
	.id_table = lm3644_id,
};

module_i2c_driver(lm3644_driver);

MODULE_DESCRIPTION("TI LM3644 Dual Flash/Torch LED Driver");
MODULE_AUTHOR("Meizu Technology / Mainline Port");
MODULE_LICENSE("GPL");
