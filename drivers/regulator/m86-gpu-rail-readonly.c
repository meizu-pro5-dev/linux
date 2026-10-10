// SPDX-License-Identifier: GPL-2.0-only
/* Expose inherited BUCK6 encoded selector; never control GPIO, DVS or PMIC. */
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
struct m86_gpu_rail {
 struct device *pmic;
 void __iomem *dvs, *gpio;
 struct regulator_desc desc;
};
static int read_selector(struct regulator_dev *rdev)
{
 struct m86_gpu_rail *d = rdev_get_drvdata(rdev);
 struct regmap *map;
 unsigned int value, status, control;
 int ret;
 device_lock(d->pmic);
 if (!d->pmic->driver) { ret = -ENODEV; goto out; }
 map = dev_get_regmap(d->pmic, NULL);
 if (!map) { ret = -ENODEV; goto out; }
 control = readl(d->dvs); status = readl(d->dvs + 4);
 /* Only the measured DVS-off ownership is supported in this first provider. */
 if (((readl(d->gpio) >> 16) & 15) != 2 || control != 0 || status != 1) { ret = -EOPNOTSUPP; goto out; }
 ret = regmap_read(map, 0x24, &value);
 if (ret) goto out;
 if (control != readl(d->dvs) || status != readl(d->dvs + 4)) {
  ret = -EAGAIN; goto out;
 }
 ret = value & 0xff;
out:
 device_unlock(d->pmic);
 return ret;
}
static int inherited_enabled(struct regulator_dev *rdev)
{
 int sel = read_selector(rdev);
 return sel < 0 ? sel : 1;
}
static const struct regulator_ops readonly_ops = {
 .list_voltage = regulator_list_voltage_linear,
 .is_enabled = inherited_enabled,
 .get_voltage_sel = read_selector,
};
static void put_pmic(void *data) { put_device(data); }
static int m86_gpu_rail_probe(struct platform_device *pdev)
{
 struct m86_gpu_rail *d;
 struct regulator_config cfg = { };
 struct regulator_dev *rdev;
 struct i2c_client *client;
 struct regmap *map;
 void __iomem *gpio;
 unsigned int id, control, ocp;
 int ret;
 if (!of_machine_is_compatible("meizu,m86")) return -ENODEV;
 d = devm_kzalloc(&pdev->dev, sizeof(*d), GFP_KERNEL);
 if (!d) return -ENOMEM;
 d->pmic = bus_find_device_by_name(&i2c_bus_type, NULL, "5-0066");
 if (!d->pmic) return -EPROBE_DEFER;
 ret = devm_add_action_or_reset(&pdev->dev, put_pmic, d->pmic);
 if (ret) return ret;
 /* Ensure supplier unbind removes this consumer before MFD regmap teardown. */
 if (!device_link_add(&pdev->dev, d->pmic, DL_FLAG_AUTOREMOVE_CONSUMER)) return -EINVAL;
 device_lock(d->pmic);
 client = i2c_verify_client(d->pmic);
 map = dev_get_regmap(d->pmic, NULL);
 if (!client || strcmp(client->name, "s2mps15-pmic") || !d->pmic->driver || !map) {
  ret = -EPROBE_DEFER; goto unlock;
 }
 ret = regmap_read(map, 0, &id);
 if (ret) goto unlock;
 ret = regmap_read(map, 0x23, &control);
 if (ret) goto unlock;
 if (regmap_get_max_register(map) < 0x59) { ret = -EOPNOTSUPP; goto unlock; }
 ret = regmap_read(map, 0x59, &ocp);
 if (ret) goto unlock;
 /* Vendor preserves undefined CTRL1 bit0; only Normal ON f8/f9 accepted.
  * GPIO38 remains exact. This observer never writes CTRL1. */
 if (id != 0x12 || (control != 0x38 && (control & 0xfe) != 0xf8)) ret = -EOPNOTSUPP;
unlock:
 device_unlock(d->pmic);
 if (ret) return ret;
 d->dvs = devm_ioremap(&pdev->dev, 0x105c6100, 8);
 d->gpio = gpio = devm_ioremap(&pdev->dev, 0x13470000, 4);
 if (!d->dvs || !gpio) return -ENOMEM;
 if (((readl(gpio) >> 16) & 15) != 2 || readl(d->dvs) != 0 || readl(d->dvs + 4) != 1)
  return -EOPNOTSUPP;
 d->desc.name = "m86-gpu-buck6-encoded-readonly";
 d->desc.type = REGULATOR_VOLTAGE;
 d->desc.owner = THIS_MODULE;
 d->desc.ops = &readonly_ops;
 d->desc.n_voltages = 256;
 d->desc.min_uV = 300000;
 d->desc.uV_step = 6250;
 cfg.dev = &pdev->dev; cfg.of_node = pdev->dev.of_node; cfg.driver_data = d;
 /* No init constraints, enable/disable/set_voltage or OPP: probe writes none. */
 rdev = devm_regulator_register(&pdev->dev, &d->desc, &cfg);
 if (IS_ERR(rdev)) return PTR_ERR(rdev);
 dev_info(&pdev->dev, "raw selector rail; OCPbit5=%u, vendor logical correction=%u codes; no voltage changes\n",
  !!(ocp & 0x20), (ocp & 0x20) ? 0 : 5);
 return 0;
}
static const struct of_device_id matches[] = {
 { .compatible = "meizu,m86-gpu-buck6-readonly" }, { }
};
MODULE_DEVICE_TABLE(of, matches);
static struct platform_driver m86_gpu_rail_driver = {
 .probe = m86_gpu_rail_probe,
 .driver = { .name = "m86-gpu-buck6-readonly", .of_match_table = matches },
};
module_platform_driver(m86_gpu_rail_driver);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Read-only inherited PRO5 BUCK6 encoded voltage provider");
