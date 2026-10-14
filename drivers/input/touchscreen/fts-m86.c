// SPDX-License-Identifier: GPL-2.0-only
/* Meizu PRO 5 ST FTS 0x3980 protocol, from pinned m576 vendor evidence.
 * Keep existing firmware and bootloader supply state; no firmware writes.
 */
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/interrupt.h>
#include <linux/m86-fts.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/pm.h>

#define FTS_FINGERS 10
#define FTS_EVENT_SIZE 8
#define FTS_FIFO_SIZE 32

struct m86_fts {
	struct i2c_client *client;
	struct input_dev *input;
	struct touchscreen_properties prop;
	struct mutex lock;
	bool dpms_blanked;
	bool pm_suspended;
	bool hw_asleep;
};

static DEFINE_MUTEX(m86_fts_global_lock);
static struct m86_fts *m86_fts_instance;

static int m86_fts_read(struct m86_fts *fts, const u8 *cmd, unsigned int ncmd,
			u8 *data, unsigned int size)
{
	struct i2c_msg messages[] = {
		{ .addr = fts->client->addr, .len = ncmd, .buf = (u8 *)cmd },
		{ .addr = fts->client->addr, .flags = I2C_M_RD,
		  .len = size, .buf = data },
	};
	int ret = i2c_transfer(fts->client->adapter, messages, 2);

	return ret < 0 ? ret : ret == 2 ? 0 : -EIO;
}

static int m86_fts_write(struct m86_fts *fts, const u8 *cmd, unsigned int size)
{
	int ret = i2c_master_send(fts->client, cmd, size);

	return ret < 0 ? ret : ret == size ? 0 : -EIO;
}

static int m86_fts_command(struct m86_fts *fts, u8 cmd)
{
	return m86_fts_write(fts, &cmd, 1);
}

static int m86_fts_wait_ready(struct m86_fts *fts, u8 expected)
{
	const u8 cmd = 0x85;
	u8 event[FTS_EVENT_SIZE];
	int ret, attempt;

	for (attempt = 0; attempt < 100; attempt++) {
		ret = m86_fts_read(fts, &cmd, 1, event, sizeof(event));
		if (ret)
			return ret;
		if (event[0] == expected)
			return 0;
		if (event[0] == 0x0f)
			return -EIO;
		usleep_range(10000, 11000);
	}
	return -ETIMEDOUT;
}

static void m86_fts_release_all(struct m86_fts *fts)
{
	int slot;

	for (slot = 0; slot < FTS_FINGERS; slot++) {
		input_mt_slot(fts->input, slot);
		input_mt_report_slot_state(fts->input, MT_TOOL_FINGER, false);
	}
}

static void m86_fts_event(struct m86_fts *fts, const u8 *event)
{
	unsigned int slot = event[1] & 0x0f;
	unsigned int x, y, pressure;

	switch (event[0]) {
	case 0x03:
	case 0x05:
		if (slot >= FTS_FINGERS)
			return;
		x = (event[2] << 4) | (event[4] >> 4);
		y = (event[3] << 4) | (event[4] & 0x0f);
		if (x > 1080 || y > 1920)
			return;
		/* Vendor treats the inclusive panel endpoint as its last pixel. */
		x = min(x, 1079U);
		y = min(y, 1919U);
		pressure = event[5] & 0x3f;
		input_mt_slot(fts->input, slot);
		input_mt_report_slot_state(fts->input, MT_TOOL_FINGER, true);
		touchscreen_report_pos(fts->input, &fts->prop, x, y, true);
		input_report_abs(fts->input, ABS_MT_TOUCH_MAJOR, pressure);
		input_report_abs(fts->input, ABS_MT_TOUCH_MINOR, pressure);
		break;
	case 0x04:
		if (slot >= FTS_FINGERS)
			return;
		input_mt_slot(fts->input, slot);
		input_mt_report_slot_state(fts->input, MT_TOOL_FINGER, false);
		break;
	case 0x10:
	case 0x0f:
		m86_fts_release_all(fts);
		dev_warn_ratelimited(&fts->client->dev,
			"controller event %#x; touch state released\n", event[0]);
		break;
	}
}

static irqreturn_t m86_fts_irq(int irq, void *data)
{
	struct m86_fts *fts = data;
	u8 events[FTS_EVENT_SIZE * FTS_FIFO_SIZE];
	u8 cmd = 0x85;
	unsigned int count, index;
	int ret;

	ret = m86_fts_read(fts, &cmd, 1, events, FTS_EVENT_SIZE);
	if (ret)
		goto error;
	count = events[7] & 0x1f;
	if (count) {
		cmd = 0x86;
		ret = m86_fts_read(fts, &cmd, 1, events + FTS_EVENT_SIZE,
				   count * FTS_EVENT_SIZE);
		if (ret)
			goto error;
	}
	for (index = 0; index <= count; index++)
		m86_fts_event(fts, events + index * FTS_EVENT_SIZE);
	input_mt_sync_frame(fts->input);
	input_sync(fts->input);
	return IRQ_HANDLED;
error:
	m86_fts_release_all(fts);
	input_mt_sync_frame(fts->input);
	input_sync(fts->input);
	dev_err_ratelimited(&fts->client->dev, "event FIFO read failed: %d\n", ret);
	return IRQ_HANDLED;
}

static void m86_fts_update_power_locked(struct m86_fts *fts)
{
	const u8 disable[] = { 0xb6, 0x00, 0x1c, 0x00 };
	const u8 enable[] = { 0xb6, 0x00, 0x1c, 0x41 };
	bool want_sleep = fts->dpms_blanked || fts->pm_suspended;

	if (want_sleep == fts->hw_asleep)
		return;

	if (want_sleep) {
		disable_irq(fts->client->irq);
		m86_fts_write(fts, disable, sizeof(disable));
		/* 3.10 fts.c FB_BLANK_POWERDOWN: SLEEPIN (0x90) + FLUSHBUFFER (0xa1) */
		m86_fts_command(fts, 0x90);
		m86_fts_command(fts, 0xa1);
		m86_fts_release_all(fts);
		input_mt_sync_frame(fts->input);
		input_sync(fts->input);
		fts->hw_asleep = true;
		dev_info(&fts->client->dev, "entered SLEEPIN (0x90) on display blank/suspend\n");
	} else {
		/* 3.10 fts.c FB_BLANK_UNBLANK: SLEEPOUT (0x91) + SENSEON (0x93) + FLUSHBUFFER (0xa1) */
		m86_fts_command(fts, 0x91);
		msleep(20);
		m86_fts_command(fts, 0x93);
		m86_fts_command(fts, 0xa1);
		m86_fts_write(fts, enable, sizeof(enable));
		fts->hw_asleep = false;
		enable_irq(fts->client->irq);
		dev_info(&fts->client->dev, "resumed SENSEON (0x93) on display unblank/resume\n");
	}
}

void m86_fts_set_blank(bool blank)
{
	struct m86_fts *fts;

	mutex_lock(&m86_fts_global_lock);
	fts = m86_fts_instance;
	if (fts) {
		mutex_lock(&fts->lock);
		if (fts->dpms_blanked != blank) {
			fts->dpms_blanked = blank;
			m86_fts_update_power_locked(fts);
		}
		mutex_unlock(&fts->lock);
	}
	mutex_unlock(&m86_fts_global_lock);
}
EXPORT_SYMBOL_GPL(m86_fts_set_blank);

static int m86_fts_pm_suspend(struct device *dev)
{
	struct m86_fts *fts = dev_get_drvdata(dev);

	if (!fts)
		return 0;

	mutex_lock(&fts->lock);
	fts->pm_suspended = true;
	m86_fts_update_power_locked(fts);
	mutex_unlock(&fts->lock);
	return 0;
}

static int m86_fts_pm_resume(struct device *dev)
{
	struct m86_fts *fts = dev_get_drvdata(dev);

	if (!fts)
		return 0;

	mutex_lock(&fts->lock);
	fts->pm_suspended = false;
	m86_fts_update_power_locked(fts);
	mutex_unlock(&fts->lock);
	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(m86_fts_pm_ops, m86_fts_pm_suspend, m86_fts_pm_resume);

static void m86_fts_disable(void *data)
{
	struct m86_fts *fts = data;
	const u8 disable[] = { 0xb6, 0x00, 0x1c, 0x00 };

	mutex_lock(&m86_fts_global_lock);
	if (m86_fts_instance == fts)
		m86_fts_instance = NULL;
	mutex_unlock(&m86_fts_global_lock);

	mutex_lock(&fts->lock);
	if (!fts->hw_asleep) {
		disable_irq(fts->client->irq);
		m86_fts_write(fts, disable, sizeof(disable));
		m86_fts_command(fts, 0x92);
		fts->hw_asleep = true;
	}
	mutex_unlock(&fts->lock);
}

static int m86_fts_probe(struct i2c_client *client)
{
	const u8 id_cmd[] = { 0xb6, 0x00, 0x07 };
	const u8 reset[] = { 0xb6, 0x00, 0x23, 0x01 };
	const u8 disable[] = { 0xb6, 0x00, 0x1c, 0x00 };
	const u8 enable[] = { 0xb6, 0x00, 0x1c, 0x41 };
	struct m86_fts *fts;
	u8 id[8];
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -ENODEV;
	if (client->irq <= 0)
		return -EINVAL;
	fts = devm_kzalloc(&client->dev, sizeof(*fts), GFP_KERNEL);
	if (!fts)
		return -ENOMEM;
	fts->client = client;
	mutex_init(&fts->lock);
	ret = m86_fts_read(fts, id_cmd, sizeof(id_cmd), id, sizeof(id));
	if (ret)
		return dev_err_probe(&client->dev, ret, "cannot read FTS ID\n");
	if (id[1] != 0x39 || id[2] != 0x80)
		return dev_err_probe(&client->dev, -ENODEV,
			"unexpected FTS ID %02x%02x\n", id[1], id[2]);
	dev_info(&client->dev, "FTS ID 3980, existing firmware %02x%02x\n",
		 id[5], id[4]);
	fts->input = devm_input_allocate_device(&client->dev);
	if (!fts->input)
		return -ENOMEM;
	fts->input->name = "Meizu PRO 5 ST FTS";
	fts->input->id.bustype = BUS_I2C;
	input_set_abs_params(fts->input, ABS_MT_POSITION_X, 0, 1079, 0, 0);
	input_set_abs_params(fts->input, ABS_MT_POSITION_Y, 0, 1919, 0, 0);
	input_set_abs_params(fts->input, ABS_MT_TOUCH_MAJOR, 0, 63, 0, 0);
	input_set_abs_params(fts->input, ABS_MT_TOUCH_MINOR, 0, 63, 0, 0);
	touchscreen_parse_properties(fts->input, true, &fts->prop);
	ret = input_mt_init_slots(fts->input, FTS_FINGERS, INPUT_MT_DIRECT);
	if (ret)
		return ret;
	ret = input_register_device(fts->input);
	if (ret)
		return ret;
	ret = devm_request_threaded_irq(&client->dev, client->irq, NULL,
		m86_fts_irq, IRQF_ONESHOT | IRQF_NO_AUTOEN, "fts-m86", fts);
	if (ret)
		return ret;
	ret = m86_fts_write(fts, disable, sizeof(disable));
	if (ret)
		return ret;
	ret = m86_fts_write(fts, reset, sizeof(reset));
	if (ret)
		return ret;
	usleep_range(5000, 6000);
	ret = m86_fts_wait_ready(fts, 0x10);
	if (ret)
		return dev_err_probe(&client->dev, ret, "reset-ready timeout\n");
	ret = m86_fts_command(fts, 0x91);
	if (ret)
		return ret;
	ret = m86_fts_wait_ready(fts, 0x11);
	if (ret)
		return dev_err_probe(&client->dev, ret, "sleep-out-ready timeout\n");
	ret = m86_fts_command(fts, 0x93);
	if (ret)
		return ret;
	ret = m86_fts_command(fts, 0xa1);
	if (ret)
		return ret;
	ret = m86_fts_write(fts, enable, sizeof(enable));
	if (ret)
		return ret;
	enable_irq(client->irq);
	i2c_set_clientdata(client, fts);
	mutex_lock(&m86_fts_global_lock);
	m86_fts_instance = fts;
	mutex_unlock(&m86_fts_global_lock);
	return devm_add_action_or_reset(&client->dev, m86_fts_disable, fts);
}

static const struct of_device_id m86_fts_match[] = {
	{ .compatible = "meizu,m86-fts" },
	{ }
};
MODULE_DEVICE_TABLE(of, m86_fts_match);

static struct i2c_driver m86_fts_driver = {
	.driver = {
		.name = "fts-m86",
		.of_match_table = m86_fts_match,
		.pm = pm_sleep_ptr(&m86_fts_pm_ops),
	},
	.probe = m86_fts_probe,
};
module_i2c_driver(m86_fts_driver);

MODULE_DESCRIPTION("Meizu PRO 5 ST FTS 3980 touchscreen");
MODULE_LICENSE("GPL");
