// SPDX-License-Identifier: GPL-2.0-only
/* Meizu PRO 5 (M86) TI TSU6721 MUIC USB path switch driver */
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regmap.h>

static const struct regmap_config tsu_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0x43,
};

static int m86_tsu6721_route_usb(struct regmap *muic)
{
	unsigned int route;
	int ret = regmap_read(muic, 0x13, &route);
	if (!ret && route != 0x25)
		ret = regmap_write(muic, 0x13, 0x25);
	if (!ret)
		ret = regmap_update_bits(muic, 0x02, 0x05, 0x01);
	return ret;
}

static int m86_tsu6721_probe(struct i2c_client *client)
{
	struct regmap *map;
	unsigned int id;
	int ret;

	if (!of_machine_is_compatible("meizu,m86"))
		return -ENODEV;

	map = devm_regmap_init_i2c(client, &tsu_regmap_config);
	if (IS_ERR(map))
		return PTR_ERR(map);

	ret = regmap_read(map, 1, &id);
	if (ret)
		return ret;

	if (id != 0x12) {
		dev_err(&client->dev, "unexpected TSU6721 chip ID: 0x%02x\n", id);
		return -ENODEV;
	}

	ret = m86_tsu6721_route_usb(map);
	if (ret) {
		dev_err(&client->dev, "failed to route USB lines: %d\n", ret);
		return ret;
	}

	dev_info(&client->dev, "Meizu PRO 5 TSU6721 USB switch initialized (id=0x%02x)\n", id);
	return 0;
}

static const struct of_device_id m86_tsu6721_of[] = {
	{ .compatible = "meizu,m86-tsu6721" },
	{ }
};
MODULE_DEVICE_TABLE(of, m86_tsu6721_of);

static struct i2c_driver m86_tsu6721_driver = {
	.driver = {
		.name = "m86-tsu6721",
		.of_match_table = m86_tsu6721_of,
	},
	.probe = m86_tsu6721_probe,
};
module_i2c_driver(m86_tsu6721_driver);

MODULE_AUTHOR("Meizu PRO 5 Porting Team");
MODULE_DESCRIPTION("Meizu PRO 5 TI TSU6721 USB switch driver");
MODULE_LICENSE("GPL");
