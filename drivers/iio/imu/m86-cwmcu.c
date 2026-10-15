// SPDX-License-Identifier: GPL-2.0-only
/*
 * Meizu PRO 5 (m86) STM32 / CyWee CWMCU SensorHub IIO Driver
 *
 * Communicates with STM32 MCU running CyWee SensorHub firmware
 * over High-Speed I2C (hsi2c_0 @ 0x3a) with AP wakeup handshake GPIOs.
 * Provides 9-axis MEMS (accel, gyro, mag)
 * and step counter via standard Linux IIO subsystem.
 *
 * Copyright (C) 2026 Antigravity Meizu PRO 5 Mainline Team
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/jiffies.h>
#include <linux/pm.h>
#include <linux/regulator/consumer.h>
#include <linux/sysfs.h>
#include <linux/workqueue.h>
#include <asm/unaligned.h>

#include <linux/iio/buffer.h>
#include <linux/iio/iio.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/iio/sysfs.h>

/* CWMCU Registers */
#define CWMCU_REG_SET_ENABLE		0x01
#define CWMCU_REG_SET_DISABLE		0x02
#define CWMCU_REG_ENABLE_LIST		0x05
#define CWMCU_REG_ERROR			0x0e
#define CWMCU_REG_INFO_COUNT		0x26
#define CWMCU_REG_INFO_EVENT		0x27
#define CWMCU_REG_INT_STATUS		0x0f
#define CWMCU_REG_STREAM_COUNT		0x20
#define CWMCU_REG_STREAM_EVENT		0x21
#define CWMCU_REG_HOST_STATUS		0x5b
#define CWMCU_REG_READ_ACCEL		0x60
#define CWMCU_REG_READ_MAG		0x61
#define CWMCU_REG_READ_GYRO		0x62
#define CWMCU_REG_FW_VERSION		0x80
#define CWMCU_REG_LIB_VERSION		0x87

/* Host status values */
#define CWMCU_HOST_STATUS_PROBE		1
#define CWMCU_HOST_STATUS_SUSPEND	2
#define CWMCU_HOST_STATUS_RESUME	3
#define CWMCU_HOST_STATUS_SHUTDOWN	4

/* Interrupt status bitmasks */
#define CWMCU_INT_INIT			BIT(0)
#define CWMCU_INT_INFO			BIT(5)
#define CWMCU_INT_ERROR			BIT(7)
#define CWMCU_INT_DATA_READY		BIT(1)
#define CWMCU_INT_BATCH_TIMEOUT		BIT(3)
#define CWMCU_INT_BATCH_FULL		BIT(4)

/* Firmware 3.0 / Flyme sensors.m86.so handles (not older SensorSupport.h). */
#define CWMCU_ID_ACCEL			0
#define CWMCU_ID_MAG			1
#define CWMCU_ID_GYRO			2
#define CWMCU_ID_STEP_COUNTER		15
#define CWMCU_MAX_SENSORS		32

/* Vendor 3.10 mx-hub.c active_set() defaults to 200ms (5 Hz) report period */
#define CWMCU_DEFAULT_RATE_MS		200
#define CWMCU_IDLE_DISABLE_MS		5000
#define CWMCU_CACHE_FRESH_MS		500
#define CWMCU_STREAM_SENSORS_MASK	(BIT(CWMCU_ID_ACCEL) | \
					 BIT(CWMCU_ID_MAG) | \
					 BIT(CWMCU_ID_GYRO))

struct m86_cwmcu_data {
	struct i2c_client *client;
	struct gpio_desc *wakeup_gpiod;
	struct gpio_desc *sleep_gpiod;
	struct gpio_desc *busy_gpiod;
	struct gpio_desc *reset_gpiod;
	struct gpio_desc *boot0_gpiod;

	struct mutex lock;
	struct delayed_work idle_work;
	u32 enabled_mask;
	u32 valid_mask;
	int last_error;
	u32 reset_count;
	u32 sample_seq[CWMCU_MAX_SENSORS];
	unsigned long last_sample_jiffies[CWMCU_MAX_SENSORS];
	u32 mcu_error;
	bool suspended;
	bool stopping;
	u16 sample[CWMCU_MAX_SENSORS][3];
	u32 steps;
	struct iio_mount_matrix orientation;

	u8 fw_version[4];
	u8 lib_version[4];
};

/* All wire access, wakeup and MCU state transitions share data->lock.
 * Handshake timing matches vendor 3.10 power_pin_sw() (200us wake, 100us sleep).
 */
static int m86_cwmcu_wake_mcu(struct m86_cwmcu_data *data)
{
	int attempt, awake;

	if (gpiod_get_value_cansleep(data->wakeup_gpiod) > 0 &&
	    (!data->sleep_gpiod || gpiod_get_value_cansleep(data->sleep_gpiod) > 0))
		return 0;

	for (attempt = 0; attempt < 3; attempt++) {
		gpiod_set_value_cansleep(data->wakeup_gpiod, 1);
		usleep_range(200, 250);
		awake = data->sleep_gpiod ?
			gpiod_get_value_cansleep(data->sleep_gpiod) : 1;
		if (awake > 0)
			return 0;
		gpiod_set_value_cansleep(data->wakeup_gpiod, 0);
		if (awake < 0)
			return awake;
		usleep_range(100, 150);
	}
	return -ETIMEDOUT;
}

static void m86_cwmcu_sleep_mcu(struct m86_cwmcu_data *data)
{
	gpiod_set_value_cansleep(data->wakeup_gpiod, 0);
	usleep_range(100, 150);
}

/* Caller holds lock and keeps the MCU awake for the whole transaction. */
static int m86_cwmcu_read(struct m86_cwmcu_data *data, u8 reg,
			    u8 *buf, u8 len)
{
	int attempt, ret;

	for (attempt = 0; attempt < 3; attempt++) {
		ret = i2c_smbus_read_i2c_block_data(data->client, reg, len, buf);
		if (ret == len)
			return 0;
		gpiod_set_value_cansleep(data->wakeup_gpiod, 0);
		usleep_range(100, 300);
		gpiod_set_value_cansleep(data->wakeup_gpiod, 1);
		usleep_range(300, 500);
	}
	return ret < 0 ? ret : -EIO;
}

static int m86_cwmcu_write(struct m86_cwmcu_data *data, u8 reg,
			     const u8 *buf, u8 len)
{
	int attempt, ret;

	for (attempt = 0; attempt < 3; attempt++) {
		ret = i2c_smbus_write_i2c_block_data(data->client, reg, len, buf);
		if (!ret)
			return 0;
		gpiod_set_value_cansleep(data->wakeup_gpiod, 0);
		usleep_range(100, 300);
		gpiod_set_value_cansleep(data->wakeup_gpiod, 1);
		usleep_range(300, 500);
	}
	return ret;
}

static int m86_cwmcu_host_status(struct m86_cwmcu_data *data, u8 status)
{
	return m86_cwmcu_write(data, CWMCU_REG_HOST_STATUS, &status, 1);
}

static void m86_cwmcu_invalidate(struct m86_cwmcu_data *data)
{
	data->enabled_mask = 0;
	data->valid_mask = 0;
	memset(data->sample_seq, 0, sizeof(data->sample_seq));
	memset(data->last_sample_jiffies, 0, sizeof(data->last_sample_jiffies));
}

static int m86_cwmcu_begin(struct m86_cwmcu_data *data)
{
	if (data->stopping)
		return -ENODEV;
	if (data->suspended)
		return -EBUSY;
	return m86_cwmcu_wake_mcu(data);
}

static int m86_cwmcu_end(struct m86_cwmcu_data *data, int ret)
{
	m86_cwmcu_sleep_mcu(data);
	data->last_error = ret;
	if (ret < 0 && ret != -EAGAIN && ret != -EBUSY)
		m86_cwmcu_invalidate(data);
	return ret;
}

static int m86_cwmcu_enable(struct m86_cwmcu_data *data, u8 id)
{
	u8 cmd[5] = { 0, id, CWMCU_DEFAULT_RATE_MS, 0, 0 };
	u8 list[8];
	int attempt, poll, ret;

	for (attempt = 0; attempt < 3; attempt++) {
		if (attempt > 0) {
			m86_cwmcu_end(data, 0);
			msleep(10);
			ret = m86_cwmcu_begin(data);
			if (ret)
				return ret;
		}
		ret = m86_cwmcu_write(data, CWMCU_REG_SET_ENABLE, cmd, sizeof(cmd));
		if (ret)
			return ret;
		/* Vendor active_set keeps WAKE high through this delay and readback,
		 * and check_enable_list() re-issues SetEnable if the MCU has not
		 * yet committed BIT(id) into RegMapR_GetHostEnableList (0x05).
		 */
		for (poll = 0; poll < 3; poll++) {
			usleep_range(5000, 6000);
			ret = m86_cwmcu_read(data, CWMCU_REG_ENABLE_LIST, list, sizeof(list));
			if (ret)
				return ret;
			if (get_unaligned_le32(list) & BIT(id)) {
				data->enabled_mask |= BIT(id);
				if (id == CWMCU_ID_STEP_COUNTER)
					data->valid_mask |= BIT(id);
				else
					data->valid_mask &= ~BIT(id);
				return 0;
			}
		}
	}
	return -EIO;
}

static int m86_cwmcu_disable(struct m86_cwmcu_data *data, u8 id)
{
	u8 cmd[5] = { 0, id, 0, 0, 0 };
	int ret;

	ret = m86_cwmcu_write(data, CWMCU_REG_SET_DISABLE, cmd, sizeof(cmd));
	if (ret)
		return ret;
	usleep_range(5000, 6000);
	data->enabled_mask &= ~BIT(id);
	data->valid_mask &= ~BIT(id);
	return 0;
}

static int m86_cwmcu_disable_stream_sensors(struct m86_cwmcu_data *data)
{
	u32 mask = data->enabled_mask & CWMCU_STREAM_SENSORS_MASK;
	int id, ret = 0;

	for (id = 0; id < CWMCU_MAX_SENSORS; id++) {
		if (!(mask & BIT(id)))
			continue;
		ret = m86_cwmcu_disable(data, id);
		if (ret)
			break;
	}
	return ret;
}

static int m86_cwmcu_status(struct m86_cwmcu_data *data)
{
	u8 buf[30], count;
	u16 status;
	int i, ret;

	ret = m86_cwmcu_read(data, CWMCU_REG_INT_STATUS, buf, 2);
	if (ret)
		return ret;
	status = get_unaligned_le16(buf);
	if (status & CWMCU_INT_INIT) {
		m86_cwmcu_invalidate(data);
		data->reset_count++;
		ret = m86_cwmcu_host_status(data, CWMCU_HOST_STATUS_RESUME);
		if (ret)
			return ret;
	}
	if (status & CWMCU_INT_INFO) {
		ret = m86_cwmcu_read(data, CWMCU_REG_INFO_COUNT, &count, 1);
		if (ret)
			return ret;
		if (count != 0xff) {
			for (i = 0; i < min_t(int, count, 16); i++) {
				ret = m86_cwmcu_read(data, CWMCU_REG_INFO_EVENT, buf, 30);
				if (ret)
					return ret;
			}
		}
	}
	if (status & CWMCU_INT_ERROR) {
		ret = m86_cwmcu_read(data, CWMCU_REG_ERROR, buf, 4);
		if (ret)
			return ret;
		data->mcu_error = get_unaligned_le32(buf);
		dev_warn_ratelimited(&data->client->dev, "MCU error: %#x\n",
				     data->mcu_error);
	}
	return 0;
}

static int m86_cwmcu_drain(struct m86_cwmcu_data *data)
{
	u8 buf[9];
	u16 count;
	int i, id, ret;

	ret = m86_cwmcu_read(data, CWMCU_REG_STREAM_COUNT, buf, 2);
	if (ret)
		return ret;
	count = get_unaligned_le16(buf);
	if (count == 0xffff)
		return 0;
	for (i = 0; i < min_t(int, count, 64); i++) {
		ret = m86_cwmcu_read(data, CWMCU_REG_STREAM_EVENT, buf, 9);
		if (ret)
			return ret;
		id = buf[0];
		/* The FIFO also carries metadata IDs outside the sensor range. */
		if (id >= CWMCU_MAX_SENSORS || !(data->enabled_mask & BIT(id)))
			continue;
		if (id == CWMCU_ID_STEP_COUNTER) {
			data->steps = get_unaligned_le32(buf + 1);
		} else if (!(data->valid_mask & BIT(id)) &&
			   !get_unaligned_le16(buf + 1) &&
			   !get_unaligned_le16(buf + 3) &&
			   !get_unaligned_le16(buf + 5)) {
			/* Skip initial all-zero FIFO tick before first physical sample. */
			continue;
		}
		data->sample[id][0] = get_unaligned_le16(buf + 1);
		data->sample[id][1] = get_unaligned_le16(buf + 3);
		data->sample[id][2] = get_unaligned_le16(buf + 5);
		data->valid_mask |= BIT(id);
		data->sample_seq[id]++;
		data->last_sample_jiffies[id] = jiffies;
	}
	return 0;
}

static void m86_cwmcu_idle_work(struct work_struct *work)
{
	struct m86_cwmcu_data *data =
		container_of(work, struct m86_cwmcu_data, idle_work.work);
	int ret;

	mutex_lock(&data->lock);
	if (!data->stopping && !data->suspended &&
	    (data->enabled_mask & CWMCU_STREAM_SENSORS_MASK)) {
		ret = m86_cwmcu_begin(data);
		if (!ret) {
			ret = m86_cwmcu_disable_stream_sensors(data);
			m86_cwmcu_drain(data);
		}
		m86_cwmcu_end(data, ret);
	}
	mutex_unlock(&data->lock);
}

static int m86_cwmcu_hw_reset(struct m86_cwmcu_data *data)
{
	int ret;

	m86_cwmcu_invalidate(data);
	gpiod_set_value_cansleep(data->wakeup_gpiod, 0);
	if (data->boot0_gpiod)
		gpiod_set_value_cansleep(data->boot0_gpiod, 0);

	usleep_range(500, 1000);

	/* Assert reset pin low for 10ms */
	ret = gpiod_direction_output(data->reset_gpiod, 0);
	if (ret)
		return ret;
	usleep_range(10000, 12000);

	/* Deassert reset pin high for 20ms */
	gpiod_set_value_cansleep(data->reset_gpiod, 1);
	msleep(20);

	/* Switch reset GPIO to input as required by STM32 NRST */
	ret = gpiod_direction_input(data->reset_gpiod);
	if (ret)
		return ret;
	/* Firmware version is readable before physical sensor init completes. */
	msleep(300);

	return 0;
}

static irqreturn_t m86_cwmcu_irq_thread(int irq, void *dev_id)
{
	struct m86_cwmcu_data *data = iio_priv(dev_id);
	int ret;

	mutex_lock(&data->lock);
	if (!data->stopping && !data->suspended) {
		ret = m86_cwmcu_begin(data);
		if (!ret)
			ret = m86_cwmcu_status(data);
		if (!ret)
			ret = m86_cwmcu_drain(data);
		m86_cwmcu_end(data, ret);
	}
	mutex_unlock(&data->lock);
	return IRQ_HANDLED;
}

/* IEEE binary16 -> integer micro-units, round nearest, ties away from zero.
 * Keep floating point out of the kernel; reject NaN/Inf and integer overflow.
 */
static int m86_cwmcu_half_to_micro(u16 bits, int *val)
{
	unsigned int exponent = (bits >> 10) & 0x1f;
	u64 magnitude = bits & 0x3ff;
	int shift = exponent ? (int)exponent - 25 : -24;

	if (exponent == 31)
		return -ERANGE;
	if (exponent)
		magnitude |= 0x400;
	magnitude *= 1000000;
	if (shift < 0)
		magnitude = (magnitude + (1ULL << (-shift - 1))) >> -shift;
	else
		magnitude <<= shift;
	if (magnitude > INT_MAX)
		return -ERANGE;
	*val = (bits & 0x8000) ? -(int)magnitude : (int)magnitude;
	return 0;
}

static int m86_cwmcu_read_raw(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan,
			      int *val, int *val2, long mask)
{
	struct m86_cwmcu_data *data = iio_priv(indio_dev);
	int sensor_id = chan->address;
	int axis = 0;
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
	case IIO_CHAN_INFO_PROCESSED:
		if (chan->modified) {
			switch (chan->channel2) {
			case IIO_MOD_X:
				axis = 0;
				break;
			case IIO_MOD_Y:
				axis = 1;
				break;
			case IIO_MOD_Z:
				axis = 2;
				break;
			default:
				axis = 0;
				break;
			}
		}

		mutex_lock(&data->lock);
		if (BIT(sensor_id) & CWMCU_STREAM_SENSORS_MASK)
			mod_delayed_work(system_wq, &data->idle_work,
					 msecs_to_jiffies(CWMCU_IDLE_DISABLE_MS));

		/* Fast path: matching vendor 3.10 cw_read_raw(), if the sensor
		 * is already enabled and a fresh sample was cached within
		 * CWMCU_CACHE_FRESH_MS (e.g. by the IRQ thread or the preceding
		 * X/Y axis read), return the cached value without waking the MCU.
		 */
		if ((data->enabled_mask & BIT(sensor_id)) &&
		    (data->valid_mask & BIT(sensor_id)) &&
		    sensor_id != CWMCU_ID_STEP_COUNTER &&
		    time_before(jiffies, data->last_sample_jiffies[sensor_id] +
				msecs_to_jiffies(CWMCU_CACHE_FRESH_MS))) {
			ret = m86_cwmcu_half_to_micro(data->sample[sensor_id][axis], val);
			mutex_unlock(&data->lock);
			return ret ? ret : IIO_VAL_INT;
		}

		ret = m86_cwmcu_begin(data);
		if (ret)
			goto out;
		if (!(data->enabled_mask & BIT(sensor_id))) {
			ret = m86_cwmcu_enable(data, sensor_id);
			if (ret)
				goto out;
			m86_cwmcu_end(data, 0);
			msleep(sensor_id == CWMCU_ID_GYRO ? 180 : 35);
			ret = m86_cwmcu_begin(data);
			if (ret)
				goto out;
		}

		{
			u32 seq = data->sample_seq[sensor_id];
			unsigned int timeout_ms =
				(data->valid_mask & BIT(sensor_id)) ? 500 : 3000;
			unsigned long deadline =
				jiffies + msecs_to_jiffies(timeout_ms);

			/* Continuous channels require a new FIFO sample. The step
			 * counter reports on change, so a valid cached total is useful.
			 */
			do {
				ret = m86_cwmcu_drain(data);
				if (ret)
					goto out;
				if (data->sample_seq[sensor_id] != seq ||
				    (sensor_id == CWMCU_ID_STEP_COUNTER &&
				     (data->valid_mask & BIT(sensor_id))))
					break;
				m86_cwmcu_end(data, 0);
				msleep((data->valid_mask & BIT(sensor_id)) ? 20 : 50);
				ret = m86_cwmcu_begin(data);
				if (ret)
					goto out;
			} while (time_before(jiffies, deadline));
			if (!(data->valid_mask & BIT(sensor_id)) ||
			    (sensor_id != CWMCU_ID_STEP_COUNTER &&
			     data->sample_seq[sensor_id] == seq)) {
				ret = -EAGAIN;
				goto out;
			}
			if (sensor_id == CWMCU_ID_STEP_COUNTER) {
				if (data->steps > INT_MAX) {
					ret = -ERANGE;
					goto out;
				}
				*val = data->steps;
			} else {
				ret = m86_cwmcu_half_to_micro(data->sample[sensor_id][axis], val);
			}
		}
	out:
		ret = m86_cwmcu_end(data, ret);
		mutex_unlock(&data->lock);
		return ret ? ret : IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		switch (sensor_id) {
		case CWMCU_ID_ACCEL:
		case CWMCU_ID_GYRO:
			*val = 0;
			*val2 = 1;
			return IIO_VAL_INT_PLUS_MICRO;
		case CWMCU_ID_MAG:
			/* Firmware uses microtesla; IIO magnetometer units are gauss. */
			*val = 0;
			*val2 = 10;
			return IIO_VAL_INT_PLUS_NANO;
		default:
			return -EINVAL;
		}

	default:
		return -EINVAL;
	}
}

static ssize_t fw_version_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct m86_cwmcu_data *data = iio_priv(indio_dev);

	return sysfs_emit(buf, "firmware: %u.%u (date: %02u/%02u), lib: %u.%u.%u.%u\n",
			  data->fw_version[1], data->fw_version[0],
			  data->fw_version[3], data->fw_version[2],
			  data->lib_version[3], data->lib_version[2],
			  data->lib_version[1], data->lib_version[0]);
}
static DEVICE_ATTR_RO(fw_version);

/* State snapshots do not access the bus or consume FIFO records. */
static ssize_t health_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct m86_cwmcu_data *data = iio_priv(dev_to_iio_dev(dev));
	ssize_t ret;

	mutex_lock(&data->lock);
	ret = sysfs_emit(buf, "enabled=%#x valid=%#x error=%d resets=%u mcu_error=%#x irq=%d\n",
			 data->enabled_mask, data->valid_mask, data->last_error,
			 data->reset_count, data->mcu_error, data->client->irq);
	mutex_unlock(&data->lock);
	return ret;
}
static DEVICE_ATTR_RO(health);

static struct attribute *m86_cwmcu_attributes[] = {
	&dev_attr_fw_version.attr,
	&dev_attr_health.attr,
	NULL,
};

static const struct attribute_group m86_cwmcu_attribute_group = {
	.attrs = m86_cwmcu_attributes,
};

static const struct iio_mount_matrix *
m86_cwmcu_get_mount_matrix(const struct iio_dev *indio_dev,
			   const struct iio_chan_spec *chan)
{
	struct m86_cwmcu_data *data = iio_priv(indio_dev);

	return &data->orientation;
}

static const struct iio_chan_spec_ext_info m86_cwmcu_ext_info[] = {
	IIO_MOUNT_MATRIX(IIO_SHARED_BY_DIR, m86_cwmcu_get_mount_matrix),
	{ }
};

#define CWMCU_CHANNEL_ACCEL(_axis, _mod) { \
	.type = IIO_ACCEL, \
	.modified = 1, \
	.channel2 = _mod, \
	.address = CWMCU_ID_ACCEL, \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW), \
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE), \
	.ext_info = m86_cwmcu_ext_info, \
}

#define CWMCU_CHANNEL_MAG(_axis, _mod) { \
	.type = IIO_MAGN, \
	.modified = 1, \
	.channel2 = _mod, \
	.address = CWMCU_ID_MAG, \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW), \
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE), \
	.ext_info = m86_cwmcu_ext_info, \
}

#define CWMCU_CHANNEL_GYRO(_axis, _mod) { \
	.type = IIO_ANGL_VEL, \
	.modified = 1, \
	.channel2 = _mod, \
	.address = CWMCU_ID_GYRO, \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW), \
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE), \
	.ext_info = m86_cwmcu_ext_info, \
}

static const struct iio_chan_spec m86_cwmcu_channels[] = {
	CWMCU_CHANNEL_ACCEL(X, IIO_MOD_X),
	CWMCU_CHANNEL_ACCEL(Y, IIO_MOD_Y),
	CWMCU_CHANNEL_ACCEL(Z, IIO_MOD_Z),
	CWMCU_CHANNEL_MAG(X, IIO_MOD_X),
	CWMCU_CHANNEL_MAG(Y, IIO_MOD_Y),
	CWMCU_CHANNEL_MAG(Z, IIO_MOD_Z),
	CWMCU_CHANNEL_GYRO(X, IIO_MOD_X),
	CWMCU_CHANNEL_GYRO(Y, IIO_MOD_Y),
	CWMCU_CHANNEL_GYRO(Z, IIO_MOD_Z),
	{
		.type = IIO_STEPS,
		.address = CWMCU_ID_STEP_COUNTER,
		.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED) | BIT(IIO_CHAN_INFO_RAW),
	},
};

static int m86_cwmcu_reg_access(struct iio_dev *indio_dev, unsigned int reg,
				  unsigned int writeval, unsigned int *readval)
{
	struct m86_cwmcu_data *data = iio_priv(indio_dev);
	u8 buf[8] = { 0 };
	int len, ret;

	if (!readval)
		return -EPERM;
	switch (reg) {
	case CWMCU_REG_ENABLE_LIST: len = 8; break;
	case CWMCU_REG_READ_ACCEL:
	case CWMCU_REG_READ_MAG:
	case CWMCU_REG_READ_GYRO: len = 6; break;
	case CWMCU_REG_FW_VERSION:
	case CWMCU_REG_LIB_VERSION:
	case CWMCU_REG_ERROR: len = 4; break;
	case CWMCU_REG_STREAM_COUNT: len = 2; break;
	default: return -EINVAL;
	}
	mutex_lock(&data->lock);
	ret = m86_cwmcu_begin(data);
	if (!ret)
		ret = m86_cwmcu_read(data, reg, buf, len);
	if (!ret)
		*readval = get_unaligned_le32(buf);
	m86_cwmcu_end(data, ret);
	mutex_unlock(&data->lock);
	return ret;
}

static const struct iio_info m86_cwmcu_info = {
	.read_raw = m86_cwmcu_read_raw,
	.debugfs_reg_access = m86_cwmcu_reg_access,
	.attrs = &m86_cwmcu_attribute_group,
};

static int m86_cwmcu_probe(struct i2c_client *client)
{
	struct iio_dev *indio_dev;
	struct m86_cwmcu_data *data;
	int ret;

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	data = iio_priv(indio_dev);
	data->client = client;
	mutex_init(&data->lock);
	INIT_DELAYED_WORK(&data->idle_work, m86_cwmcu_idle_work);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_I2C_BLOCK))
		return -EOPNOTSUPP;

	ret = iio_read_mount_matrix(&client->dev, &data->orientation);
	if (ret)
		return ret;

	/* VDD is required by the board binding. */
	ret = devm_regulator_get_enable(&client->dev, "vdd");
	if (ret < 0)
		return dev_err_probe(&client->dev, ret, "Failed to enable vdd regulator\n");
	usleep_range(2000, 3000);

	/* Hardware control GPIOs */
	data->wakeup_gpiod = devm_gpiod_get(&client->dev, "wakeup", GPIOD_OUT_LOW);
	if (IS_ERR(data->wakeup_gpiod))
		return dev_err_probe(&client->dev, PTR_ERR(data->wakeup_gpiod),
				     "Failed to get wakeup GPIO\n");

	data->sleep_gpiod = devm_gpiod_get_optional(&client->dev, "sleep", GPIOD_IN);
	if (IS_ERR(data->sleep_gpiod))
		return dev_err_probe(&client->dev, PTR_ERR(data->sleep_gpiod),
				     "Failed to get sleep GPIO\n");

	data->busy_gpiod = devm_gpiod_get_optional(&client->dev, "busy", GPIOD_IN);
	if (IS_ERR(data->busy_gpiod))
		return dev_err_probe(&client->dev, PTR_ERR(data->busy_gpiod),
				     "Failed to get busy GPIO\n");

	data->reset_gpiod = devm_gpiod_get(&client->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(data->reset_gpiod))
		return dev_err_probe(&client->dev, PTR_ERR(data->reset_gpiod),
				     "Failed to get reset GPIO\n");

	data->boot0_gpiod = devm_gpiod_get_optional(&client->dev, "boot0", GPIOD_OUT_LOW);
	if (IS_ERR(data->boot0_gpiod))
		return dev_err_probe(&client->dev, PTR_ERR(data->boot0_gpiod),
				     "Failed to get boot0 GPIO\n");

	/* Resolve a required IRQ even if the I2C core could not map it earlier. */
	if (client->irq <= 0) {
		ret = of_irq_get(client->dev.of_node, 0);
		if (ret <= 0)
			return dev_err_probe(&client->dev, ret ? ret : -EPROBE_DEFER,
					     "SensorHub IRQ unavailable\n");
		client->irq = ret;
	}

	/* Perform STM32 hardware reset sequence */
	ret = m86_cwmcu_hw_reset(data);
	if (ret)
		return ret;

	ret = m86_cwmcu_wake_mcu(data);
	if (ret)
		goto init_done;
	ret = m86_cwmcu_read(data, CWMCU_REG_FW_VERSION, data->fw_version, 4);
	if (ret)
		goto init_done;
	ret = m86_cwmcu_read(data, CWMCU_REG_LIB_VERSION, data->lib_version, 4);
	if (ret)
		goto init_done;
	ret = m86_cwmcu_host_status(data, CWMCU_HOST_STATUS_PROBE);
	if (ret)
		goto init_done;
	ret = m86_cwmcu_status(data);
	if (!ret)
		ret = m86_cwmcu_host_status(data, CWMCU_HOST_STATUS_RESUME);
init_done:
	m86_cwmcu_end(data, ret);
	if (ret)
		return dev_err_probe(&client->dev, ret, "MCU initialization failed\n");
	dev_info(&client->dev, "firmware %u.%u, irq %d, protocol fix v2\n",
		 data->fw_version[1], data->fw_version[0], client->irq);

	/* Threaded IRQ for streaming data */
	if (client->irq > 0) {
		ret = devm_request_threaded_irq(&client->dev, client->irq,
						NULL, m86_cwmcu_irq_thread,
						IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
						"m86-cwmcu", indio_dev);
		if (ret)
			return dev_err_probe(&client->dev, ret,
					     "Failed to request threaded IRQ\n");
	}

	indio_dev->name = "m86-cwmcu";
	indio_dev->info = &m86_cwmcu_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = m86_cwmcu_channels;
	indio_dev->num_channels = ARRAY_SIZE(m86_cwmcu_channels);

	i2c_set_clientdata(client, indio_dev);

	return devm_iio_device_register(&client->dev, indio_dev);
}

static void m86_cwmcu_remove(struct i2c_client *client)
{
	struct m86_cwmcu_data *data = iio_priv(i2c_get_clientdata(client));

	cancel_delayed_work_sync(&data->idle_work);
	mutex_lock(&data->lock);
	data->stopping = true;
	if (!m86_cwmcu_wake_mcu(data)) {
		m86_cwmcu_disable_stream_sensors(data);
		m86_cwmcu_host_status(data, CWMCU_HOST_STATUS_SHUTDOWN);
	}
	m86_cwmcu_sleep_mcu(data);
	mutex_unlock(&data->lock);
}

static int m86_cwmcu_pm_status(struct device *dev, bool suspend)
{
	struct m86_cwmcu_data *data = iio_priv(dev_get_drvdata(dev));
	int ret;

	if (suspend)
		cancel_delayed_work_sync(&data->idle_work);

	mutex_lock(&data->lock);
	ret = m86_cwmcu_wake_mcu(data);
	if (!ret && suspend && (data->enabled_mask & CWMCU_STREAM_SENSORS_MASK))
		m86_cwmcu_disable_stream_sensors(data);
	if (!ret)
		ret = m86_cwmcu_status(data);
	if (!ret)
		ret = m86_cwmcu_host_status(data, suspend ?
			CWMCU_HOST_STATUS_SUSPEND : CWMCU_HOST_STATUS_RESUME);
	if (!ret)
		data->suspended = suspend;
	m86_cwmcu_end(data, ret);
	mutex_unlock(&data->lock);
	return ret;
}

static int m86_cwmcu_suspend(struct device *dev)
{
	return m86_cwmcu_pm_status(dev, true);
}

static int m86_cwmcu_resume(struct device *dev)
{
	return m86_cwmcu_pm_status(dev, false);
}

static DEFINE_SIMPLE_DEV_PM_OPS(m86_cwmcu_pm_ops, m86_cwmcu_suspend, m86_cwmcu_resume);

static const struct of_device_id m86_cwmcu_of_match[] = {
	{ .compatible = "meizu,m86-cwmcu" },
	{ .compatible = "st,sensor_hub" },
	{ }
};
MODULE_DEVICE_TABLE(of, m86_cwmcu_of_match);

static const struct i2c_device_id m86_cwmcu_id[] = {
	{ "m86-cwmcu", 0 },
	{ "sensor_hub", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, m86_cwmcu_id);

static struct i2c_driver m86_cwmcu_driver = {
	.driver = {
		.name = "m86-cwmcu",
		.of_match_table = m86_cwmcu_of_match,
		.pm = pm_sleep_ptr(&m86_cwmcu_pm_ops),
	},
	.probe = m86_cwmcu_probe,
	.remove = m86_cwmcu_remove,
	.id_table = m86_cwmcu_id,
};
module_i2c_driver(m86_cwmcu_driver);

MODULE_AUTHOR("Antigravity Meizu PRO 5 Team");
MODULE_DESCRIPTION("Meizu PRO 5 STM32 CyWee SensorHub IIO driver");
MODULE_LICENSE("GPL");
