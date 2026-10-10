// SPDX-License-Identifier: GPL-2.0-only
/* BQ27532 telemetry + OEM CHEM_ID query; no unseal/reset/data-flash writes. */
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/workqueue.h>

struct m86_gauge {
	struct i2c_client *client;
	struct power_supply *psy;
	struct delayed_work poll;
	const char *manufacturer;
};
static enum power_supply_property m86_props[] = {
	POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
};
static int m86_read(struct m86_gauge *g, u8 reg)
{
	return i2c_smbus_read_word_data(g->client, reg);
}
static int m86_get(struct power_supply *psy, enum power_supply_property prop,
		   union power_supply_propval *val)
{
	struct m86_gauge *g = power_supply_get_drvdata(psy);
	int reg, v, battery_current, supplied;

	switch (prop) {
	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = g->manufacturer; return 0;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION; return 0;
	case POWER_SUPPLY_PROP_PRESENT:
	case POWER_SUPPLY_PROP_STATUS: reg = 0x0a; break;
	case POWER_SUPPLY_PROP_HEALTH: reg = 0x06; break;
	case POWER_SUPPLY_PROP_CAPACITY: reg = 0x20; break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW: reg = 0x08; break;
	case POWER_SUPPLY_PROP_CURRENT_NOW: reg = 0x14; break;
	case POWER_SUPPLY_PROP_TEMP: reg = 0x06; break;
	case POWER_SUPPLY_PROP_CHARGE_NOW: reg = 0x10; break;
	case POWER_SUPPLY_PROP_CHARGE_FULL: reg = 0x12; break;
	case POWER_SUPPLY_PROP_CYCLE_COUNT: reg = 0x1e; break;
	default: return -EINVAL;
	}
	v = m86_read(g, reg);
	if (v < 0) return v;
	switch (prop) {
	case POWER_SUPPLY_PROP_PRESENT: val->intval = !!(v & BIT(3)); break;
	case POWER_SUPPLY_PROP_CAPACITY:
		if (v > 100) return -ERANGE;
		val->intval = v; break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
	case POWER_SUPPLY_PROP_CHARGE_NOW:
	case POWER_SUPPLY_PROP_CHARGE_FULL: val->intval = v * 1000; break;
	case POWER_SUPPLY_PROP_CURRENT_NOW: val->intval = (s16)v * 1000; break;
	case POWER_SUPPLY_PROP_TEMP: val->intval = v - 2731; break;
	case POWER_SUPPLY_PROP_CYCLE_COUNT: val->intval = v; break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = v - 2731 < 0 ? POWER_SUPPLY_HEALTH_COLD :
			(v - 2731 > 450 ? POWER_SUPPLY_HEALTH_OVERHEAT : POWER_SUPPLY_HEALTH_GOOD); break;
	case POWER_SUPPLY_PROP_STATUS:
		if (!(v & BIT(3))) { val->intval = POWER_SUPPLY_STATUS_UNKNOWN; break; }
		battery_current = m86_read(g, 0x14);
		if (battery_current < 0) return battery_current;
		battery_current = (s16)battery_current;
		supplied = power_supply_am_i_supplied(psy);
		if (supplied < 0) supplied = 0;
		/* Net battery current matters: USB online can still discharge. */
		if (battery_current < -20) val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		else if (supplied && (v & BIT(9))) val->intval = POWER_SUPPLY_STATUS_FULL;
		else if (battery_current > 20) val->intval = POWER_SUPPLY_STATUS_CHARGING;
		else val->intval = supplied ? POWER_SUPPLY_STATUS_NOT_CHARGING :
			POWER_SUPPLY_STATUS_DISCHARGING;
		break;
	default: return -EINVAL;
	}
	return 0;
}
static void gauge_poll(struct work_struct *work)
{
	struct m86_gauge *g = container_of(to_delayed_work(work), struct m86_gauge, poll);
	power_supply_changed(g->psy);
	queue_delayed_work(system_power_efficient_wq, &g->poll, 5 * HZ);
}
static void gauge_changed(struct power_supply *psy)
{
	struct m86_gauge *g = power_supply_get_drvdata(psy);
	mod_delayed_work(system_power_efficient_wq, &g->poll, 0);
}
static void gauge_stop(void *data)
{
	struct m86_gauge *g = data;
	cancel_delayed_work_sync(&g->poll);
}
static const struct power_supply_desc m86_desc = {
	.name = "m86-bq27532", .type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = m86_props, .num_properties = ARRAY_SIZE(m86_props),
	.get_property = m86_get, .external_power_changed = gauge_changed,
};
static int m86_probe(struct i2c_client *client)
{
	struct power_supply_config cfg = {};
	struct m86_gauge *g;
	int ret;
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_WORD_DATA))
		return -EOPNOTSUPP;
	g = devm_kzalloc(&client->dev, sizeof(*g), GFP_KERNEL);
	if (!g) return -ENOMEM;
	g->client = client;
	g->manufacturer = "Unknown";
	ret = i2c_smbus_write_word_data(client, 0x00, 0x0008);
	if (!ret) {
		msleep(10);
		ret = i2c_smbus_read_word_data(client, 0x00);
		switch (ret) {
		case 0x3481: case 0x3602: case 0x3419: g->manufacturer = "Sony"; break;
		case 0x3460: case 0x3575: g->manufacturer = "ATL"; break;
		}
	}
	dev_info(&client->dev, "OEM cell CHEM_ID=%#x manufacturer=%s\n", ret, g->manufacturer);
	INIT_DELAYED_WORK(&g->poll, gauge_poll);
	cfg.drv_data = g; cfg.of_node = client->dev.of_node;
	g->psy = devm_power_supply_register(&client->dev, &m86_desc, &cfg);
	if (IS_ERR(g->psy)) return PTR_ERR(g->psy);
	ret = devm_add_action_or_reset(&client->dev, gauge_stop, g);
	if (ret) return ret;
	queue_delayed_work(system_power_efficient_wq, &g->poll, 0);
	return 0;
}
static const struct of_device_id m86_of[] = {
	{ .compatible = "meizu,m86-bq27532-readonly" }, {}
};
MODULE_DEVICE_TABLE(of, m86_of);
static struct i2c_driver m86_driver = {
	.driver = { .name = "m86-bq27532-readonly", .of_match_table = m86_of },
	.probe = m86_probe,
};
module_i2c_driver(m86_driver);
MODULE_LICENSE("GPL");
