// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * exynos_tmu.c - Samsung Exynos TMU (Thermal Management Unit)
 *
 *  Copyright (C) 2014 Samsung Electronics
 *  Bartlomiej Zolnierkiewicz <b.zolnierkie@samsung.com>
 *  Lukasz Majewski <l.majewski@samsung.com>
 *
 *  Copyright (C) 2011 Samsung Electronics
 *  Donggeun Kim <dg77.kim@samsung.com>
 *  Amit Daniel Kachhap <amit.kachhap@linaro.org>
 */

#include <linux/clk.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/interrupt.h>
#include <linux/soc/samsung/exynos7420-tmu.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/spinlock.h>
#include <linux/thermal.h>

#include <dt-bindings/thermal/thermal_exynos.h>

/* Exynos generic registers */
#define EXYNOS_TMU_REG_TRIMINFO		0x0
#define EXYNOS_TMU_REG_CONTROL		0x20
#define EXYNOS_TMU_REG_STATUS		0x28
#define EXYNOS_TMU_REG_CURRENT_TEMP	0x40
#define EXYNOS_TMU_REG_INTEN		0x70
#define EXYNOS_TMU_REG_INTSTAT		0x74
#define EXYNOS_TMU_REG_INTCLEAR		0x78

#define EXYNOS_TMU_TEMP_MASK		0xff
#define EXYNOS_TMU_REF_VOLTAGE_SHIFT	24
#define EXYNOS_TMU_REF_VOLTAGE_MASK	0x1f
#define EXYNOS_TMU_BUF_SLOPE_SEL_MASK	0xf
#define EXYNOS_TMU_BUF_SLOPE_SEL_SHIFT	8
#define EXYNOS_TMU_CORE_EN_SHIFT	0

/* Exynos3250 specific registers */
#define EXYNOS_TMU_TRIMINFO_CON1	0x10

/* Exynos4210 specific registers */
#define EXYNOS4210_TMU_REG_THRESHOLD_TEMP	0x44
#define EXYNOS4210_TMU_REG_TRIG_LEVEL0	0x50

/* Exynos5250, Exynos4412, Exynos3250 specific registers */
#define EXYNOS_TMU_TRIMINFO_CON2	0x14
#define EXYNOS_THD_TEMP_RISE		0x50
#define EXYNOS_THD_TEMP_FALL		0x54
#define EXYNOS_EMUL_CON		0x80

#define EXYNOS_TRIMINFO_RELOAD_ENABLE	1
#define EXYNOS_TRIMINFO_25_SHIFT	0
#define EXYNOS_TRIMINFO_85_SHIFT	8
#define EXYNOS_TMU_TRIP_MODE_SHIFT	13
#define EXYNOS_TMU_TRIP_MODE_MASK	0x7
#define EXYNOS_TMU_THERM_TRIP_EN_SHIFT	12

#define EXYNOS_TMU_INTEN_RISE0_SHIFT	0
#define EXYNOS_TMU_INTEN_FALL0_SHIFT	16

#define EXYNOS_EMUL_TIME	0x57F0
#define EXYNOS_EMUL_TIME_MASK	0xffff
#define EXYNOS_EMUL_TIME_SHIFT	16
#define EXYNOS_EMUL_DATA_SHIFT	8
#define EXYNOS_EMUL_DATA_MASK	0xFF
#define EXYNOS_EMUL_ENABLE	0x1

/* Exynos5260 specific */
#define EXYNOS5260_TMU_REG_INTEN		0xC0
#define EXYNOS5260_TMU_REG_INTSTAT		0xC4
#define EXYNOS5260_TMU_REG_INTCLEAR		0xC8
#define EXYNOS5260_EMUL_CON			0x100

/* Exynos4412 specific */
#define EXYNOS4412_MUX_ADDR_VALUE          6
#define EXYNOS4412_MUX_ADDR_SHIFT          20

/* Exynos5433 specific registers */
#define EXYNOS5433_THD_TEMP_RISE3_0		0x050
#define EXYNOS5433_THD_TEMP_RISE7_4		0x054
#define EXYNOS5433_THD_TEMP_FALL3_0		0x060
#define EXYNOS5433_THD_TEMP_FALL7_4		0x064
#define EXYNOS5433_TMU_REG_INTEN		0x0c0
#define EXYNOS5433_TMU_REG_INTPEND		0x0c8
#define EXYNOS5433_TMU_EMUL_CON			0x110
#define EXYNOS5433_TMU_PD_DET_EN		0x130

#define EXYNOS5433_TRIMINFO_SENSOR_ID_SHIFT	16
#define EXYNOS5433_TRIMINFO_CALIB_SEL_SHIFT	23
#define EXYNOS5433_TRIMINFO_SENSOR_ID_MASK	\
			(0xf << EXYNOS5433_TRIMINFO_SENSOR_ID_SHIFT)
#define EXYNOS5433_TRIMINFO_CALIB_SEL_MASK	BIT(23)

#define EXYNOS5433_TRIMINFO_ONE_POINT_TRIMMING	0
#define EXYNOS5433_TRIMINFO_TWO_POINT_TRIMMING	1

#define EXYNOS5433_PD_DET_EN			1

#define EXYNOS5433_G3D_BASE			0x10070000

/* Exynos7 specific registers */
#define EXYNOS7_TMU_REG_SAMPLING_INTERVAL	0x2c
#define EXYNOS7_THD_TEMP_RISE7_6		0x50
#define EXYNOS7_THD_TEMP_FALL7_6		0x60
#define EXYNOS7_TMU_REG_INTEN			0x110
#define EXYNOS7_TMU_REG_INTPEND			0x118
#define EXYNOS7_TMU_REG_EMUL_CON		0x160

#define EXYNOS7_TMU_TEMP_MASK			0x1ff
#define EXYNOS7_TMU_SAMPLING_DEFAULT		0xe10
#define EXYNOS7_TMU_CLEAR_ALL_INT		0x00ff00ff
#define EXYNOS7_TMU_STATUS_IDLE			BIT(0)
#define EXYNOS7_TMU_STATUS_TRIM_BUSY_MASK	(0x7 << 8)
#define EXYNOS7_TMU_STATUS_VALID_P0		BIT(12)
#define EXYNOS7_TMU_TRIMINFO_85_SHIFT		9
#define EXYNOS7_TMU_TRIMINFO_VREF_SHIFT		18
#define EXYNOS7_TMU_TRIMINFO_VREF_MASK		0x1f
#define EXYNOS7_TMU_TRIMINFO_CALIB_SEL_MASK	BIT(23)
#define EXYNOS7_PD_DET_EN_SHIFT			23
#define EXYNOS7_TMU_INTEN_RISE0_SHIFT		0
#define EXYNOS7_EMUL_DATA_SHIFT			7
#define EXYNOS7_EMUL_DATA_MASK			0x1ff
#define EXYNOS7_NUM_BANKS			4

#define EXYNOS_FIRST_POINT_TRIM			25
#define EXYNOS_SECOND_POINT_TRIM		85

#define EXYNOS_NOISE_CANCEL_MODE		4

#define MCELSIUS	1000

enum soc_type {
	SOC_ARCH_EXYNOS3250 = 1,
	SOC_ARCH_EXYNOS4210,
	SOC_ARCH_EXYNOS4412,
	SOC_ARCH_EXYNOS5250,
	SOC_ARCH_EXYNOS5260,
	SOC_ARCH_EXYNOS5420,
	SOC_ARCH_EXYNOS5420_TRIMINFO,
	SOC_ARCH_EXYNOS5433,
	SOC_ARCH_EXYNOS7,
};

/**
 * struct exynos_tmu_data : A structure to hold the private data of the TMU
 *			    driver
 * @id: identifier of the one instance of the TMU controller.
 * @base: base address of the single instance of the TMU controller.
 * @base_second: base address of the common registers of the TMU controller.
 * @irq: irq number of the TMU controller.
 * @soc: id of the SOC type.
 * @irq_work: pointer to the irq work structure.
 * @lock: lock to implement synchronization.
 * @clk: pointer to the clock structure.
 * @clk_sec: pointer to the clock structure for accessing the base_second.
 * @sclk: pointer to the clock structure for accessing the tmu special clk.
 * @cal_type: calibration type for temperature
 * @efuse_value: SoC defined fuse value
 * @min_efuse_value: minimum valid trimming data
 * @max_efuse_value: maximum valid trimming data
 * @temp_error1: fused value of the first point trim.
 * @temp_error2: fused value of the second point trim.
 * @gain: gain of amplifier in the positive-TC generator block
 *	0 < gain <= 15
 * @reference_voltage: reference voltage of amplifier
 *	in the positive-TC generator block
 *	0 < reference_voltage <= 31
 * @regulator: pointer to the TMU regulator structure.
 * @reg_conf: pointer to structure to register with core thermal.
 * @tzd: pointer to thermal_zone_device structure
 * @ntrip: number of supported trip points.
 * @enabled: current status of TMU device
 * @tmu_set_trip_temp: SoC specific method to set trip (rising threshold)
 * @tmu_set_trip_hyst: SoC specific to set hysteresis (falling threshold)
 * @tmu_initialize: SoC specific TMU initialization method
 * @tmu_control: SoC specific TMU control method
 * @tmu_read: SoC specific TMU temperature read method
 * @tmu_set_emulation: SoC specific TMU emulation setting method
 * @tmu_clear_irqs: SoC specific TMU interrupts clearing method
 */
struct exynos_tmu_data {
	int id;
	void __iomem *base;
	void __iomem *base_second;
	int irq;
	enum soc_type soc;
	struct work_struct irq_work;
	struct mutex lock;
	struct clk *clk, *clk_sec, *sclk;
	u32 cal_type;
	u32 efuse_value;
	u32 min_efuse_value;
	u32 max_efuse_value;
	u16 temp_error1, temp_error2;
	u8 gain;
	u8 reference_voltage;
	struct regulator *regulator;
	struct thermal_zone_device *tzd;
	unsigned int ntrip;
	bool enabled;
	u32 saved_control;
	u32 irq_en;
	int last_temp;
	bool temp_valid;
	bool gpu_off;
	bool init_failed;

	void (*tmu_set_trip_temp)(struct exynos_tmu_data *data, int trip,
				 u8 temp);
	void (*tmu_set_trip_hyst)(struct exynos_tmu_data *data, int trip,
				 u8 temp, u8 hyst);
	void (*tmu_initialize)(struct platform_device *pdev);
	void (*tmu_control)(struct platform_device *pdev, bool on);
	int (*tmu_read)(struct exynos_tmu_data *data);
	void (*tmu_set_emulation)(struct exynos_tmu_data *data, int temp);
	void (*tmu_clear_irqs)(struct exynos_tmu_data *data);
};

/*
 * Exynos7420 consumer coordination across tmuctrl_0..3:
 * Lock hierarchy: exynos7_tmu_access -> data->lock -> exynos7_tmu_cache_lock.
 */
static DEFINE_MUTEX(exynos7_tmu_access);
static DEFINE_SPINLOCK(exynos7_tmu_cache_lock);
static struct exynos_tmu_data *exynos7_tmu_banks[EXYNOS7_NUM_BANKS];
static BLOCKING_NOTIFIER_HEAD(exynos7_tmu_chain);

/*
 * TMU treats temperature as a mapped temperature code.
 * The temperature is converted differently depending on the calibration type.
 */
static int temp_to_code(struct exynos_tmu_data *data, u8 temp)
{
	int code;

	if (data->cal_type == TYPE_ONE_POINT_TRIMMING)
		code = temp + data->temp_error1 - EXYNOS_FIRST_POINT_TRIM;
	else
		code = (temp - EXYNOS_FIRST_POINT_TRIM) *
			(data->temp_error2 - data->temp_error1) /
			(EXYNOS_SECOND_POINT_TRIM - EXYNOS_FIRST_POINT_TRIM) +
			data->temp_error1;

	if (data->soc == SOC_ARCH_EXYNOS7)
		return clamp_val(code, 0, EXYNOS7_TMU_TEMP_MASK);
	return code;
}

/*
 * Calculate a temperature value from a temperature code.
 * The unit of the temperature is degree Celsius.
 */
static int code_to_temp(struct exynos_tmu_data *data, u16 temp_code)
{
	if (data->cal_type == TYPE_ONE_POINT_TRIMMING)
		return temp_code - data->temp_error1 + EXYNOS_FIRST_POINT_TRIM;

	return (temp_code - data->temp_error1) *
		(EXYNOS_SECOND_POINT_TRIM - EXYNOS_FIRST_POINT_TRIM) /
		(data->temp_error2 - data->temp_error1) +
		EXYNOS_FIRST_POINT_TRIM;
}

static int code_to_mcelsius(struct exynos_tmu_data *data, u16 temp_code)
{
	if (data->soc == SOC_ARCH_EXYNOS7 &&
	    data->cal_type == TYPE_TWO_POINT_TRIMMING)
		return ((int)temp_code - data->temp_error1) * 60000 /
			(data->temp_error2 - data->temp_error1) + 25000;

	return code_to_temp(data, temp_code) * MCELSIUS;
}

static void sanitize_temp_error(struct exynos_tmu_data *data, u32 trim_info)
{
	u16 tmu_temp_mask =
		(data->soc == SOC_ARCH_EXYNOS7) ? EXYNOS7_TMU_TEMP_MASK
						: EXYNOS_TMU_TEMP_MASK;

	data->temp_error1 = trim_info & tmu_temp_mask;
	data->temp_error2 = ((trim_info >> EXYNOS_TRIMINFO_85_SHIFT) &
				EXYNOS_TMU_TEMP_MASK);

	if (!data->temp_error1 ||
	    (data->min_efuse_value > data->temp_error1) ||
	    (data->temp_error1 > data->max_efuse_value))
		data->temp_error1 = data->efuse_value & EXYNOS_TMU_TEMP_MASK;

	if (!data->temp_error2)
		data->temp_error2 =
			(data->efuse_value >> EXYNOS_TRIMINFO_85_SHIFT) &
			EXYNOS_TMU_TEMP_MASK;
}

static int exynos_tmu_initialize(struct platform_device *pdev)
{
	struct exynos_tmu_data *data = platform_get_drvdata(pdev);
	struct thermal_zone_device *tzd = data->tzd;
	int num_trips = thermal_zone_get_num_trips(tzd);
	unsigned int status;
	int ret = 0, temp;

	ret = thermal_zone_get_crit_temp(tzd, &temp);
	if (ret && data->soc != SOC_ARCH_EXYNOS5433) { /* FIXME */
		dev_err(&pdev->dev,
			"No CRITICAL trip point defined in device tree!\n");
		goto out;
	}

	if (num_trips > data->ntrip) {
		dev_info(&pdev->dev,
			 "More trip points than supported by this TMU.\n");
		dev_info(&pdev->dev,
			 "%d trip points should be configured in polling mode.\n",
			 num_trips - data->ntrip);
	}

	mutex_lock(&data->lock);
	clk_enable(data->clk);
	if (!IS_ERR(data->clk_sec))
		clk_enable(data->clk_sec);

	if (data->soc == SOC_ARCH_EXYNOS7) {
		int i, ntrips = min_t(int, num_trips, data->ntrip);
		u32 r7, f7, th;

		data->init_failed = false;
		data->tmu_initialize(pdev);
		if (data->init_failed) {
			ret = -ENODATA;
			goto err;
		}

		for (i = 0; i < ntrips; i++) {
			struct thermal_trip trip;

			ret = thermal_zone_get_trip(tzd, i, &trip);
			if (ret)
				goto err;

			data->tmu_set_trip_temp(data, i, trip.temperature / MCELSIUS);
			data->tmu_set_trip_hyst(data, i, trip.temperature / MCELSIUS,
						trip.hysteresis / MCELSIUS);
		}

		/* Preserve firmware level-6 nibble and ensure level-7 emergency shutdown trip. */
		if (ntrips < 8) {
			r7 = temp_to_code(data, 115);
			f7 = temp_to_code(data, 110);
			th = readl(data->base + EXYNOS7_THD_TEMP_RISE7_6) & 0xffff;
			writel((r7 << 16) | th, data->base + EXYNOS7_THD_TEMP_RISE7_6);
			th = readl(data->base + EXYNOS7_THD_TEMP_FALL7_6) & 0xffff;
			writel((f7 << 16) | th, data->base + EXYNOS7_THD_TEMP_FALL7_6);
		}

		data->tmu_clear_irqs(data);
		goto err;
	}

	status = readb(data->base + EXYNOS_TMU_REG_STATUS);
	if (!status) {
		ret = -EBUSY;
	} else {
		int i, ntrips =
			min_t(int, num_trips, data->ntrip);

		data->tmu_initialize(pdev);

		/* Write temperature code for rising and falling threshold */
		for (i = 0; i < ntrips; i++) {

			struct thermal_trip trip;

			ret = thermal_zone_get_trip(tzd, i, &trip);
			if (ret)
				goto err;

			data->tmu_set_trip_temp(data, i, trip.temperature / MCELSIUS);
			data->tmu_set_trip_hyst(data, i, trip.temperature / MCELSIUS,
						trip.hysteresis / MCELSIUS);
		}

		data->tmu_clear_irqs(data);
	}
err:
	clk_disable(data->clk);
	mutex_unlock(&data->lock);
	if (!IS_ERR(data->clk_sec))
		clk_disable(data->clk_sec);
out:
	return ret;
}

static u32 get_con_reg(struct exynos_tmu_data *data, u32 con)
{
	if (data->soc == SOC_ARCH_EXYNOS4412 ||
	    data->soc == SOC_ARCH_EXYNOS3250)
		con |= (EXYNOS4412_MUX_ADDR_VALUE << EXYNOS4412_MUX_ADDR_SHIFT);

	con &= ~(EXYNOS_TMU_REF_VOLTAGE_MASK << EXYNOS_TMU_REF_VOLTAGE_SHIFT);
	con |= data->reference_voltage << EXYNOS_TMU_REF_VOLTAGE_SHIFT;

	con &= ~(EXYNOS_TMU_BUF_SLOPE_SEL_MASK << EXYNOS_TMU_BUF_SLOPE_SEL_SHIFT);
	con |= (data->gain << EXYNOS_TMU_BUF_SLOPE_SEL_SHIFT);

	con &= ~(EXYNOS_TMU_TRIP_MODE_MASK << EXYNOS_TMU_TRIP_MODE_SHIFT);
	con |= (EXYNOS_NOISE_CANCEL_MODE << EXYNOS_TMU_TRIP_MODE_SHIFT);

	return con;
}

static void exynos_tmu_control(struct platform_device *pdev, bool on)
{
	struct exynos_tmu_data *data = platform_get_drvdata(pdev);

	mutex_lock(&data->lock);
	clk_enable(data->clk);
	data->tmu_control(pdev, on);
	data->enabled = on;
	clk_disable(data->clk);
	mutex_unlock(&data->lock);
}

static void exynos4210_tmu_set_trip_temp(struct exynos_tmu_data *data,
					 int trip_id, u8 temp)
{
	struct thermal_trip trip;
	u8 ref, th_code;

	if (thermal_zone_get_trip(data->tzd, 0, &trip))
		return;

	ref = trip.temperature / MCELSIUS;

	if (trip_id == 0) {
		th_code = temp_to_code(data, ref);
		writeb(th_code, data->base + EXYNOS4210_TMU_REG_THRESHOLD_TEMP);
	}

	temp -= ref;
	writeb(temp, data->base + EXYNOS4210_TMU_REG_TRIG_LEVEL0 + trip_id * 4);
}

/* failing thresholds are not supported on Exynos4210 */
static void exynos4210_tmu_set_trip_hyst(struct exynos_tmu_data *data,
					 int trip, u8 temp, u8 hyst)
{
}

static void exynos4210_tmu_initialize(struct platform_device *pdev)
{
	struct exynos_tmu_data *data = platform_get_drvdata(pdev);

	sanitize_temp_error(data, readl(data->base + EXYNOS_TMU_REG_TRIMINFO));
}

static void exynos4412_tmu_set_trip_temp(struct exynos_tmu_data *data,
					 int trip, u8 temp)
{
	u32 th, con;

	th = readl(data->base + EXYNOS_THD_TEMP_RISE);
	th &= ~(0xff << 8 * trip);
	th |= temp_to_code(data, temp) << 8 * trip;
	writel(th, data->base + EXYNOS_THD_TEMP_RISE);

	if (trip == 3) {
		con = readl(data->base + EXYNOS_TMU_REG_CONTROL);
		con |= (1 << EXYNOS_TMU_THERM_TRIP_EN_SHIFT);
		writel(con, data->base + EXYNOS_TMU_REG_CONTROL);
	}
}

static void exynos4412_tmu_set_trip_hyst(struct exynos_tmu_data *data,
					 int trip, u8 temp, u8 hyst)
{
	u32 th;

	th = readl(data->base + EXYNOS_THD_TEMP_FALL);
	th &= ~(0xff << 8 * trip);
	if (hyst)
		th |= temp_to_code(data, temp - hyst) << 8 * trip;
	writel(th, data->base + EXYNOS_THD_TEMP_FALL);
}

static void exynos4412_tmu_initialize(struct platform_device *pdev)
{
	struct exynos_tmu_data *data = platform_get_drvdata(pdev);
	unsigned int trim_info, ctrl;

	if (data->soc == SOC_ARCH_EXYNOS3250 ||
	    data->soc == SOC_ARCH_EXYNOS4412 ||
	    data->soc == SOC_ARCH_EXYNOS5250) {
		if (data->soc == SOC_ARCH_EXYNOS3250) {
			ctrl = readl(data->base + EXYNOS_TMU_TRIMINFO_CON1);
			ctrl |= EXYNOS_TRIMINFO_RELOAD_ENABLE;
			writel(ctrl, data->base + EXYNOS_TMU_TRIMINFO_CON1);
		}
		ctrl = readl(data->base + EXYNOS_TMU_TRIMINFO_CON2);
		ctrl |= EXYNOS_TRIMINFO_RELOAD_ENABLE;
		writel(ctrl, data->base + EXYNOS_TMU_TRIMINFO_CON2);
	}

	/* On exynos5420 the triminfo register is in the shared space */
	if (data->soc == SOC_ARCH_EXYNOS5420_TRIMINFO)
		trim_info = readl(data->base_second + EXYNOS_TMU_REG_TRIMINFO);
	else
		trim_info = readl(data->base + EXYNOS_TMU_REG_TRIMINFO);

	sanitize_temp_error(data, trim_info);
}

static void exynos5433_tmu_set_trip_temp(struct exynos_tmu_data *data,
					 int trip, u8 temp)
{
	unsigned int reg_off, j;
	u32 th;

	if (trip > 3) {
		reg_off = EXYNOS5433_THD_TEMP_RISE7_4;
		j = trip - 4;
	} else {
		reg_off = EXYNOS5433_THD_TEMP_RISE3_0;
		j = trip;
	}

	th = readl(data->base + reg_off);
	th &= ~(0xff << j * 8);
	th |= (temp_to_code(data, temp) << j * 8);
	writel(th, data->base + reg_off);
}

static void exynos5433_tmu_set_trip_hyst(struct exynos_tmu_data *data,
					 int trip, u8 temp, u8 hyst)
{
	unsigned int reg_off, j;
	u32 th;

	if (trip > 3) {
		reg_off = EXYNOS5433_THD_TEMP_FALL7_4;
		j = trip - 4;
	} else {
		reg_off = EXYNOS5433_THD_TEMP_FALL3_0;
		j = trip;
	}

	th = readl(data->base + reg_off);
	th &= ~(0xff << j * 8);
	th |= (temp_to_code(data, temp - hyst) << j * 8);
	writel(th, data->base + reg_off);
}

static void exynos5433_tmu_initialize(struct platform_device *pdev)
{
	struct exynos_tmu_data *data = platform_get_drvdata(pdev);
	unsigned int trim_info;
	int sensor_id, cal_type;

	trim_info = readl(data->base + EXYNOS_TMU_REG_TRIMINFO);
	sanitize_temp_error(data, trim_info);

	/* Read the temperature sensor id */
	sensor_id = (trim_info & EXYNOS5433_TRIMINFO_SENSOR_ID_MASK)
				>> EXYNOS5433_TRIMINFO_SENSOR_ID_SHIFT;
	dev_info(&pdev->dev, "Temperature sensor ID: 0x%x\n", sensor_id);

	/* Read the calibration mode */
	writel(trim_info, data->base + EXYNOS_TMU_REG_TRIMINFO);
	cal_type = (trim_info & EXYNOS5433_TRIMINFO_CALIB_SEL_MASK)
				>> EXYNOS5433_TRIMINFO_CALIB_SEL_SHIFT;

	switch (cal_type) {
	case EXYNOS5433_TRIMINFO_TWO_POINT_TRIMMING:
		data->cal_type = TYPE_TWO_POINT_TRIMMING;
		break;
	case EXYNOS5433_TRIMINFO_ONE_POINT_TRIMMING:
	default:
		data->cal_type = TYPE_ONE_POINT_TRIMMING;
		break;
	}

	dev_info(&pdev->dev, "Calibration type is %d-point calibration\n",
			cal_type ?  2 : 1);
}

static void exynos7_tmu_set_trip_temp(struct exynos_tmu_data *data,
				      int trip, u8 temp)
{
	unsigned int reg_off, bit_off;
	u32 th;

	reg_off = ((7 - trip) / 2) * 4;
	bit_off = ((8 - trip) % 2);

	th = readl(data->base + EXYNOS7_THD_TEMP_RISE7_6 + reg_off);
	th &= ~(EXYNOS7_TMU_TEMP_MASK << (16 * bit_off));
	th |= temp_to_code(data, temp) << (16 * bit_off);
	writel(th, data->base + EXYNOS7_THD_TEMP_RISE7_6 + reg_off);
}

static void exynos7_tmu_set_trip_hyst(struct exynos_tmu_data *data,
				      int trip, u8 temp, u8 hyst)
{
	unsigned int reg_off, bit_off;
	u32 th;

	reg_off = ((7 - trip) / 2) * 4;
	bit_off = ((8 - trip) % 2);

	th = readl(data->base + EXYNOS7_THD_TEMP_FALL7_6 + reg_off);
	th &= ~(EXYNOS7_TMU_TEMP_MASK << (16 * bit_off));
	th |= temp_to_code(data, temp - hyst) << (16 * bit_off);
	writel(th, data->base + EXYNOS7_THD_TEMP_FALL7_6 + reg_off);
}

static void exynos7_tmu_initialize(struct platform_device *pdev)
{
	struct exynos_tmu_data *data = platform_get_drvdata(pdev);
	u32 control, status, trim_info, vref;
	int ret;

	/* Stop sensing before waiting for the banked P0 trim selection. */
	control = readl(data->base + EXYNOS_TMU_REG_CONTROL);
	writel(control & ~BIT(EXYNOS_TMU_CORE_EN_SHIFT),
	       data->base + EXYNOS_TMU_REG_CONTROL);
	ret = readl_poll_timeout(data->base + EXYNOS_TMU_REG_STATUS, status,
				 status & EXYNOS7_TMU_STATUS_IDLE, 10, 20000);
	if (ret) {
		data->init_failed = true;
		return;
	}

	writel(EXYNOS7_TMU_SAMPLING_DEFAULT,
	       data->base + EXYNOS7_TMU_REG_SAMPLING_INTERVAL);
	writel(control | BIT(EXYNOS_TMU_CORE_EN_SHIFT),
	       data->base + EXYNOS_TMU_REG_CONTROL);
	ret = readl_poll_timeout(data->base + EXYNOS_TMU_REG_STATUS, status,
				 !(status & EXYNOS7_TMU_STATUS_TRIM_BUSY_MASK),
				 1, 20000);
	if (ret) {
		data->init_failed = true;
		return;
	}

	trim_info = readl(data->base + EXYNOS_TMU_REG_TRIMINFO);
	if (readl(data->base + EXYNOS_TMU_REG_STATUS) &
	    EXYNOS7_TMU_STATUS_TRIM_BUSY_MASK) {
		data->init_failed = true;
		return;
	}

	data->temp_error1 = trim_info & EXYNOS7_TMU_TEMP_MASK;
	data->temp_error2 = (trim_info >> EXYNOS7_TMU_TRIMINFO_85_SHIFT) &
			    EXYNOS7_TMU_TEMP_MASK;
	if (!data->temp_error1) {
		data->init_failed = true;
		return;
	}

	if (trim_info & EXYNOS7_TMU_TRIMINFO_CALIB_SEL_MASK) {
		if (data->temp_error2 <= data->temp_error1) {
			data->init_failed = true;
			return;
		}
		data->cal_type = TYPE_TWO_POINT_TRIMMING;
	} else {
		data->cal_type = TYPE_ONE_POINT_TRIMMING;
	}

	if (data->id == 1) {
		data->gain = 8;
		data->reference_voltage = 16;
	} else {
		data->gain = 9;
		vref = (trim_info >> EXYNOS7_TMU_TRIMINFO_VREF_SHIFT) &
		       EXYNOS7_TMU_TRIMINFO_VREF_MASK;
		data->reference_voltage = vref ?: 17;
	}
}

static void exynos4210_tmu_control(struct platform_device *pdev, bool on)
{
	struct exynos_tmu_data *data = platform_get_drvdata(pdev);
	struct thermal_zone_device *tz = data->tzd;
	struct thermal_trip trip;
	unsigned int con, interrupt_en = 0, i;

	con = get_con_reg(data, readl(data->base + EXYNOS_TMU_REG_CONTROL));

	if (on) {
		for (i = 0; i < data->ntrip; i++) {
			if (thermal_zone_get_trip(tz, i, &trip))
				continue;

			interrupt_en |=
				(1 << (EXYNOS_TMU_INTEN_RISE0_SHIFT + i * 4));
		}

		if (data->soc != SOC_ARCH_EXYNOS4210)
			interrupt_en |=
				interrupt_en << EXYNOS_TMU_INTEN_FALL0_SHIFT;

		con |= (1 << EXYNOS_TMU_CORE_EN_SHIFT);
	} else {
		con &= ~(1 << EXYNOS_TMU_CORE_EN_SHIFT);
	}

	writel(interrupt_en, data->base + EXYNOS_TMU_REG_INTEN);
	writel(con, data->base + EXYNOS_TMU_REG_CONTROL);
}

static void exynos5433_tmu_control(struct platform_device *pdev, bool on)
{
	struct exynos_tmu_data *data = platform_get_drvdata(pdev);
	struct thermal_zone_device *tz = data->tzd;
	struct thermal_trip trip;
	unsigned int con, interrupt_en = 0, pd_det_en, i;

	con = get_con_reg(data, readl(data->base + EXYNOS_TMU_REG_CONTROL));

	if (on) {
		for (i = 0; i < data->ntrip; i++) {
			if (thermal_zone_get_trip(tz, i, &trip))
				continue;

			interrupt_en |=
				(1 << (EXYNOS7_TMU_INTEN_RISE0_SHIFT + i));
		}

		interrupt_en |=
			interrupt_en << EXYNOS_TMU_INTEN_FALL0_SHIFT;

		con |= (1 << EXYNOS_TMU_CORE_EN_SHIFT);
	} else
		con &= ~(1 << EXYNOS_TMU_CORE_EN_SHIFT);

	pd_det_en = on ? EXYNOS5433_PD_DET_EN : 0;

	writel(pd_det_en, data->base + EXYNOS5433_TMU_PD_DET_EN);
	writel(interrupt_en, data->base + EXYNOS5433_TMU_REG_INTEN);
	writel(con, data->base + EXYNOS_TMU_REG_CONTROL);
}

static void exynos7_mask_all_irqs(struct exynos_tmu_data *data)
{
	int j;

	/*
	 * Only probe 0 (0x110) is wired to this bank's GIC SPI. Keep remote
	 * probe INTEN registers masked so their INTPEND bits do not storm.
	 */
	for (j = 0; j < 5; j++)
		writel(0, data->base + EXYNOS7_TMU_REG_INTEN + j * 0x10);
	writel(EXYNOS7_TMU_CLEAR_ALL_INT, data->base + EXYNOS7_TMU_REG_INTPEND);
}

static void exynos7_tmu_control(struct platform_device *pdev, bool on)
{
	struct exynos_tmu_data *data = platform_get_drvdata(pdev);
	struct thermal_zone_device *tz = data->tzd;
	struct thermal_trip trip;
	u32 con, status, interrupt_en = 0;
	int i, ret;

	exynos7_mask_all_irqs(data);

	con = readl(data->base + EXYNOS_TMU_REG_CONTROL);
	writel(con & ~BIT(EXYNOS_TMU_CORE_EN_SHIFT),
	       data->base + EXYNOS_TMU_REG_CONTROL);
	readl_poll_timeout(data->base + EXYNOS_TMU_REG_STATUS, status,
			   status & EXYNOS7_TMU_STATUS_IDLE, 10, 20000);

	con = get_con_reg(data, con);

	if (on) {
		for (i = 0; i < data->ntrip; i++) {
			if (thermal_zone_get_trip(tz, i, &trip))
				continue;

			interrupt_en |=
				(1 << (EXYNOS7_TMU_INTEN_RISE0_SHIFT + i));
		}

		interrupt_en |=
			interrupt_en << EXYNOS_TMU_INTEN_FALL0_SHIFT;

		con |= BIT(EXYNOS7_PD_DET_EN_SHIFT);
		con &= ~BIT(EXYNOS_TMU_CORE_EN_SHIFT);
		data->saved_control = con | BIT(EXYNOS_TMU_CORE_EN_SHIFT);
		data->irq_en = interrupt_en;

		writel(con, data->base + EXYNOS_TMU_REG_CONTROL);
		writel(EXYNOS7_TMU_SAMPLING_DEFAULT,
		       data->base + EXYNOS7_TMU_REG_SAMPLING_INTERVAL);
		writel(data->saved_control, data->base + EXYNOS_TMU_REG_CONTROL);

		ret = readl_poll_timeout(data->base + EXYNOS_TMU_REG_STATUS,
					 status,
					 status & EXYNOS7_TMU_STATUS_VALID_P0,
					 100, 20000);
		if (ret)
			dev_warn(&pdev->dev,
				 "TMU%d VALID_P0 timeout on enable: status=%#x\n",
				 data->id, status);

		writel(EXYNOS7_TMU_CLEAR_ALL_INT,
		       data->base + EXYNOS7_TMU_REG_INTPEND);
		writel(interrupt_en, data->base + EXYNOS7_TMU_REG_INTEN);
	} else {
		con &= ~(BIT(EXYNOS_TMU_CORE_EN_SHIFT) |
			 BIT(EXYNOS7_PD_DET_EN_SHIFT));
		writel(con, data->base + EXYNOS_TMU_REG_CONTROL);
	}
}

static int exynos7_tmu_read(struct exynos_tmu_data *data);

static int exynos_get_temp_locked(struct exynos_tmu_data *data, int *temp)
{
	int value, ret = 0;

	if (!data || !data->tmu_read)
		return -EINVAL;
	if (!data->enabled)
		return -EAGAIN;
	if (data->soc == SOC_ARCH_EXYNOS7 && data->gpu_off)
		return -EAGAIN;

	clk_enable(data->clk);
	value = data->tmu_read(data);
	clk_disable(data->clk);

	if (value < 0) {
		ret = value;
	} else {
		*temp = code_to_mcelsius(data, value);
		if (data->soc == SOC_ARCH_EXYNOS7) {
			unsigned long flags;

			spin_lock_irqsave(&exynos7_tmu_cache_lock, flags);
			data->last_temp = *temp;
			data->temp_valid = true;
			spin_unlock_irqrestore(&exynos7_tmu_cache_lock, flags);
		}
	}

	return ret;
}

static int exynos_get_temp(struct thermal_zone_device *tz, int *temp)
{
	struct exynos_tmu_data *data = thermal_zone_device_priv(tz);
	int ret;

	if (!data || !data->tmu_read)
		return -EINVAL;
	if (!data->enabled)
		/*
		 * Called too early, probably
		 * from thermal_zone_of_sensor_register().
		 */
		return -EAGAIN;

	mutex_lock(&data->lock);
	ret = exynos_get_temp_locked(data, temp);
	mutex_unlock(&data->lock);

	return ret;
}

#ifdef CONFIG_THERMAL_EMULATION
static u32 get_emul_con_reg(struct exynos_tmu_data *data, unsigned int val,
			    int temp)
{
	if (temp) {
		temp /= MCELSIUS;

		val &= ~(EXYNOS_EMUL_TIME_MASK << EXYNOS_EMUL_TIME_SHIFT);
		val |= (EXYNOS_EMUL_TIME << EXYNOS_EMUL_TIME_SHIFT);
		if (data->soc == SOC_ARCH_EXYNOS7) {
			val &= ~(EXYNOS7_EMUL_DATA_MASK <<
				EXYNOS7_EMUL_DATA_SHIFT);
			val |= (temp_to_code(data, temp) <<
				EXYNOS7_EMUL_DATA_SHIFT) |
				EXYNOS_EMUL_ENABLE;
		} else {
			val &= ~(EXYNOS_EMUL_DATA_MASK <<
				EXYNOS_EMUL_DATA_SHIFT);
			val |= (temp_to_code(data, temp) <<
				EXYNOS_EMUL_DATA_SHIFT) |
				EXYNOS_EMUL_ENABLE;
		}
	} else {
		val &= ~EXYNOS_EMUL_ENABLE;
	}

	return val;
}

static void exynos4412_tmu_set_emulation(struct exynos_tmu_data *data,
					 int temp)
{
	unsigned int val;
	u32 emul_con;

	if (data->soc == SOC_ARCH_EXYNOS5260)
		emul_con = EXYNOS5260_EMUL_CON;
	else if (data->soc == SOC_ARCH_EXYNOS5433)
		emul_con = EXYNOS5433_TMU_EMUL_CON;
	else if (data->soc == SOC_ARCH_EXYNOS7)
		emul_con = EXYNOS7_TMU_REG_EMUL_CON;
	else
		emul_con = EXYNOS_EMUL_CON;

	val = readl(data->base + emul_con);
	val = get_emul_con_reg(data, val, temp);
	writel(val, data->base + emul_con);
}

static int exynos_tmu_set_emulation(struct thermal_zone_device *tz, int temp)
{
	struct exynos_tmu_data *data = thermal_zone_device_priv(tz);
	int ret = -EINVAL;

	if (data->soc == SOC_ARCH_EXYNOS4210)
		goto out;

	if (temp && temp < MCELSIUS)
		goto out;

	mutex_lock(&data->lock);
	clk_enable(data->clk);
	data->tmu_set_emulation(data, temp);
	clk_disable(data->clk);
	mutex_unlock(&data->lock);
	return 0;
out:
	return ret;
}
#else
#define exynos4412_tmu_set_emulation NULL
static int exynos_tmu_set_emulation(struct thermal_zone_device *tz, int temp)
	{ return -EINVAL; }
#endif /* CONFIG_THERMAL_EMULATION */

static int exynos4210_tmu_read(struct exynos_tmu_data *data)
{
	int ret = readb(data->base + EXYNOS_TMU_REG_CURRENT_TEMP);

	/* "temp_code" should range between 75 and 175 */
	return (ret < 75 || ret > 175) ? -ENODATA : ret;
}

static int exynos4412_tmu_read(struct exynos_tmu_data *data)
{
	return readb(data->base + EXYNOS_TMU_REG_CURRENT_TEMP);
}

static int exynos7_tmu_read(struct exynos_tmu_data *data)
{
	u32 status, code;
	int attempt, ret;

	if (data->gpu_off ||
	    !(readl(data->base + EXYNOS_TMU_REG_CONTROL) & BIT(EXYNOS_TMU_CORE_EN_SHIFT)))
		return -EAGAIN;

	for (attempt = 0; attempt < 2; attempt++) {
		ret = readl_poll_timeout(data->base + EXYNOS_TMU_REG_STATUS, status,
					 status & EXYNOS7_TMU_STATUS_VALID_P0,
					 100, 20000);
		if (ret)
			return ret;

		code = readl(data->base + EXYNOS_TMU_REG_CURRENT_TEMP) &
		       EXYNOS7_TMU_TEMP_MASK;
		if ((readl(data->base + EXYNOS_TMU_REG_STATUS) &
		     EXYNOS7_TMU_STATUS_VALID_P0) &&
		    (readl(data->base + EXYNOS_TMU_REG_CONTROL) &
		     BIT(EXYNOS_TMU_CORE_EN_SHIFT)))
			return (int)code;
	}

	return -EAGAIN;
}

static void exynos_tmu_work(struct work_struct *work)
{
	struct exynos_tmu_data *data = container_of(work,
			struct exynos_tmu_data, irq_work);

	thermal_zone_device_update(data->tzd, THERMAL_EVENT_UNSPECIFIED);

	mutex_lock(&data->lock);
	clk_enable(data->clk);

	/* TODO: take action based on particular interrupt */
	data->tmu_clear_irqs(data);

	clk_disable(data->clk);
	mutex_unlock(&data->lock);
	enable_irq(data->irq);

	if (data->soc == SOC_ARCH_EXYNOS7) {
		int temp;

		if (!m86_tmu_get_cached_temp(&temp))
			blocking_notifier_call_chain(&exynos7_tmu_chain,
						     (unsigned long)temp, NULL);
	}
}

static void exynos4210_tmu_clear_irqs(struct exynos_tmu_data *data)
{
	unsigned int val_irq;
	u32 tmu_intstat, tmu_intclear;

	if (data->soc == SOC_ARCH_EXYNOS5260) {
		tmu_intstat = EXYNOS5260_TMU_REG_INTSTAT;
		tmu_intclear = EXYNOS5260_TMU_REG_INTCLEAR;
	} else if (data->soc == SOC_ARCH_EXYNOS7) {
		writel(EXYNOS7_TMU_CLEAR_ALL_INT,
		       data->base + EXYNOS7_TMU_REG_INTPEND);
		return;
	} else if (data->soc == SOC_ARCH_EXYNOS5433) {
		tmu_intstat = EXYNOS5433_TMU_REG_INTPEND;
		tmu_intclear = EXYNOS5433_TMU_REG_INTPEND;
	} else {
		tmu_intstat = EXYNOS_TMU_REG_INTSTAT;
		tmu_intclear = EXYNOS_TMU_REG_INTCLEAR;
	}

	val_irq = readl(data->base + tmu_intstat);
	/*
	 * Clear the interrupts.  Please note that the documentation for
	 * Exynos3250, Exynos4412, Exynos5250 and Exynos5260 incorrectly
	 * states that INTCLEAR register has a different placing of bits
	 * responsible for FALL IRQs than INTSTAT register.  Exynos5420
	 * and Exynos5440 documentation is correct (Exynos4210 doesn't
	 * support FALL IRQs at all).
	 */
	writel(val_irq, data->base + tmu_intclear);
}

static irqreturn_t exynos_tmu_irq(int irq, void *id)
{
	struct exynos_tmu_data *data = id;

	disable_irq_nosync(irq);
	schedule_work(&data->irq_work);

	return IRQ_HANDLED;
}

static const struct of_device_id exynos_tmu_match[] = {
	{
		.compatible = "samsung,exynos3250-tmu",
		.data = (const void *)SOC_ARCH_EXYNOS3250,
	}, {
		.compatible = "samsung,exynos4210-tmu",
		.data = (const void *)SOC_ARCH_EXYNOS4210,
	}, {
		.compatible = "samsung,exynos4412-tmu",
		.data = (const void *)SOC_ARCH_EXYNOS4412,
	}, {
		.compatible = "samsung,exynos5250-tmu",
		.data = (const void *)SOC_ARCH_EXYNOS5250,
	}, {
		.compatible = "samsung,exynos5260-tmu",
		.data = (const void *)SOC_ARCH_EXYNOS5260,
	}, {
		.compatible = "samsung,exynos5420-tmu",
		.data = (const void *)SOC_ARCH_EXYNOS5420,
	}, {
		.compatible = "samsung,exynos5420-tmu-ext-triminfo",
		.data = (const void *)SOC_ARCH_EXYNOS5420_TRIMINFO,
	}, {
		.compatible = "samsung,exynos5433-tmu",
		.data = (const void *)SOC_ARCH_EXYNOS5433,
	}, {
		.compatible = "samsung,exynos7-tmu",
		.data = (const void *)SOC_ARCH_EXYNOS7,
	},
	{ },
};
MODULE_DEVICE_TABLE(of, exynos_tmu_match);

static int exynos_map_dt_data(struct platform_device *pdev)
{
	struct exynos_tmu_data *data = platform_get_drvdata(pdev);
	struct resource res;

	if (!data || !pdev->dev.of_node)
		return -ENODEV;

	data->id = of_alias_get_id(pdev->dev.of_node, "tmuctrl");
	if (data->id < 0)
		data->id = 0;

	data->irq = irq_of_parse_and_map(pdev->dev.of_node, 0);
	if (data->irq <= 0) {
		dev_err(&pdev->dev, "failed to get IRQ\n");
		return -ENODEV;
	}

	if (of_address_to_resource(pdev->dev.of_node, 0, &res)) {
		dev_err(&pdev->dev, "failed to get Resource 0\n");
		return -ENODEV;
	}

	data->base = devm_ioremap(&pdev->dev, res.start, resource_size(&res));
	if (!data->base) {
		dev_err(&pdev->dev, "Failed to ioremap memory\n");
		return -EADDRNOTAVAIL;
	}

	data->soc = (uintptr_t)of_device_get_match_data(&pdev->dev);

	switch (data->soc) {
	case SOC_ARCH_EXYNOS4210:
		data->tmu_set_trip_temp = exynos4210_tmu_set_trip_temp;
		data->tmu_set_trip_hyst = exynos4210_tmu_set_trip_hyst;
		data->tmu_initialize = exynos4210_tmu_initialize;
		data->tmu_control = exynos4210_tmu_control;
		data->tmu_read = exynos4210_tmu_read;
		data->tmu_clear_irqs = exynos4210_tmu_clear_irqs;
		data->ntrip = 4;
		data->gain = 15;
		data->reference_voltage = 7;
		data->efuse_value = 55;
		data->min_efuse_value = 40;
		data->max_efuse_value = 100;
		break;
	case SOC_ARCH_EXYNOS3250:
	case SOC_ARCH_EXYNOS4412:
	case SOC_ARCH_EXYNOS5250:
	case SOC_ARCH_EXYNOS5260:
	case SOC_ARCH_EXYNOS5420:
	case SOC_ARCH_EXYNOS5420_TRIMINFO:
		data->tmu_set_trip_temp = exynos4412_tmu_set_trip_temp;
		data->tmu_set_trip_hyst = exynos4412_tmu_set_trip_hyst;
		data->tmu_initialize = exynos4412_tmu_initialize;
		data->tmu_control = exynos4210_tmu_control;
		data->tmu_read = exynos4412_tmu_read;
		data->tmu_set_emulation = exynos4412_tmu_set_emulation;
		data->tmu_clear_irqs = exynos4210_tmu_clear_irqs;
		data->ntrip = 4;
		data->gain = 8;
		data->reference_voltage = 16;
		data->efuse_value = 55;
		if (data->soc != SOC_ARCH_EXYNOS5420 &&
		    data->soc != SOC_ARCH_EXYNOS5420_TRIMINFO)
			data->min_efuse_value = 40;
		else
			data->min_efuse_value = 0;
		data->max_efuse_value = 100;
		break;
	case SOC_ARCH_EXYNOS5433:
		data->tmu_set_trip_temp = exynos5433_tmu_set_trip_temp;
		data->tmu_set_trip_hyst = exynos5433_tmu_set_trip_hyst;
		data->tmu_initialize = exynos5433_tmu_initialize;
		data->tmu_control = exynos5433_tmu_control;
		data->tmu_read = exynos4412_tmu_read;
		data->tmu_set_emulation = exynos4412_tmu_set_emulation;
		data->tmu_clear_irqs = exynos4210_tmu_clear_irqs;
		data->ntrip = 8;
		data->gain = 8;
		if (res.start == EXYNOS5433_G3D_BASE)
			data->reference_voltage = 23;
		else
			data->reference_voltage = 16;
		data->efuse_value = 75;
		data->min_efuse_value = 40;
		data->max_efuse_value = 150;
		break;
	case SOC_ARCH_EXYNOS7:
		data->tmu_set_trip_temp = exynos7_tmu_set_trip_temp;
		data->tmu_set_trip_hyst = exynos7_tmu_set_trip_hyst;
		data->tmu_initialize = exynos7_tmu_initialize;
		data->tmu_control = exynos7_tmu_control;
		data->tmu_read = exynos7_tmu_read;
		data->tmu_set_emulation = exynos4412_tmu_set_emulation;
		data->tmu_clear_irqs = exynos4210_tmu_clear_irqs;
		data->ntrip = 8;
		data->gain = 9;
		data->reference_voltage = 17;
		data->efuse_value = 75;
		data->min_efuse_value = 15;
		data->max_efuse_value = 100;
		break;
	default:
		dev_err(&pdev->dev, "Platform not supported\n");
		return -EINVAL;
	}

	data->cal_type = TYPE_ONE_POINT_TRIMMING;

	/*
	 * Check if the TMU shares some registers and then try to map the
	 * memory of common registers.
	 */
	if (data->soc != SOC_ARCH_EXYNOS5420_TRIMINFO)
		return 0;

	if (of_address_to_resource(pdev->dev.of_node, 1, &res)) {
		dev_err(&pdev->dev, "failed to get Resource 1\n");
		return -ENODEV;
	}

	data->base_second = devm_ioremap(&pdev->dev, res.start,
					resource_size(&res));
	if (!data->base_second) {
		dev_err(&pdev->dev, "Failed to ioremap memory\n");
		return -ENOMEM;
	}

	return 0;
}

static const struct thermal_zone_device_ops exynos_sensor_ops = {
	.get_temp = exynos_get_temp,
	.set_emul_temp = exynos_tmu_set_emulation,
};

static int exynos_tmu_probe(struct platform_device *pdev)
{
	struct exynos_tmu_data *data;
	int ret;

	data = devm_kzalloc(&pdev->dev, sizeof(struct exynos_tmu_data),
					GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	platform_set_drvdata(pdev, data);
	mutex_init(&data->lock);

	/*
	 * Try enabling the regulator if found
	 * TODO: Add regulator as an SOC feature, so that regulator enable
	 * is a compulsory call.
	 */
	data->regulator = devm_regulator_get_optional(&pdev->dev, "vtmu");
	if (!IS_ERR(data->regulator)) {
		ret = regulator_enable(data->regulator);
		if (ret) {
			dev_err(&pdev->dev, "failed to enable vtmu\n");
			return ret;
		}
	} else {
		if (PTR_ERR(data->regulator) == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		dev_info(&pdev->dev, "Regulator node (vtmu) not found\n");
	}

	ret = exynos_map_dt_data(pdev);
	if (ret)
		goto err_sensor;

	INIT_WORK(&data->irq_work, exynos_tmu_work);

	data->clk = devm_clk_get(&pdev->dev, "tmu_apbif");
	if (IS_ERR(data->clk)) {
		dev_err(&pdev->dev, "Failed to get clock\n");
		ret = PTR_ERR(data->clk);
		goto err_sensor;
	}

	data->clk_sec = devm_clk_get(&pdev->dev, "tmu_triminfo_apbif");
	if (IS_ERR(data->clk_sec)) {
		if (data->soc == SOC_ARCH_EXYNOS5420_TRIMINFO) {
			dev_err(&pdev->dev, "Failed to get triminfo clock\n");
			ret = PTR_ERR(data->clk_sec);
			goto err_sensor;
		}
	} else {
		ret = clk_prepare(data->clk_sec);
		if (ret) {
			dev_err(&pdev->dev, "Failed to get clock\n");
			goto err_sensor;
		}
	}

	ret = clk_prepare(data->clk);
	if (ret) {
		dev_err(&pdev->dev, "Failed to get clock\n");
		goto err_clk_sec;
	}

	switch (data->soc) {
	case SOC_ARCH_EXYNOS5433:
	case SOC_ARCH_EXYNOS7:
		data->sclk = devm_clk_get(&pdev->dev, "tmu_sclk");
		if (IS_ERR(data->sclk)) {
			dev_err(&pdev->dev, "Failed to get sclk\n");
			ret = PTR_ERR(data->sclk);
			goto err_clk;
		} else {
			ret = clk_prepare_enable(data->sclk);
			if (ret) {
				dev_err(&pdev->dev, "Failed to enable sclk\n");
				goto err_clk;
			}
		}
		break;
	default:
		break;
	}

	/*
	 * data->tzd must be registered before calling exynos_tmu_initialize(),
	 * requesting irq and calling exynos_tmu_control().
	 */
	data->tzd = devm_thermal_of_zone_register(&pdev->dev, 0, data,
						  &exynos_sensor_ops);
	if (IS_ERR(data->tzd)) {
		ret = PTR_ERR(data->tzd);
		if (ret != -EPROBE_DEFER)
			dev_err(&pdev->dev, "Failed to register sensor: %d\n",
				ret);
		goto err_sclk;
	}

	ret = exynos_tmu_initialize(pdev);
	if (ret) {
		dev_err(&pdev->dev, "Failed to initialize TMU\n");
		goto err_sclk;
	}

	ret = devm_request_irq(&pdev->dev, data->irq, exynos_tmu_irq,
		data->soc == SOC_ARCH_EXYNOS7 ? 0 :
		(IRQF_TRIGGER_RISING | IRQF_SHARED),
		dev_name(&pdev->dev), data);
	if (ret) {
		dev_err(&pdev->dev, "Failed to request irq: %d\n", data->irq);
		goto err_sclk;
	}

	exynos_tmu_control(pdev, true);

	if (data->soc == SOC_ARCH_EXYNOS7) {
		int init_temp;

		if (data->id < 0 || data->id >= EXYNOS7_NUM_BANKS) {
			ret = -EINVAL;
			goto err_sclk;
		}

		mutex_lock(&data->lock);
		ret = exynos_get_temp_locked(data, &init_temp);
		mutex_unlock(&data->lock);
		if (ret) {
			dev_err(&pdev->dev, "TMU%d initial sample failed: %d\n",
				data->id, ret);
			goto err_sclk;
		}

		mutex_lock(&exynos7_tmu_access);
		if (exynos7_tmu_banks[data->id]) {
			mutex_unlock(&exynos7_tmu_access);
			ret = -EBUSY;
			goto err_sclk;
		}
		exynos7_tmu_banks[data->id] = data;
		mutex_unlock(&exynos7_tmu_access);

		dev_info(&pdev->dev,
			 "TMU%d registered (cal=%d gain=%u vref=%u init=%d mC)\n",
			 data->id, data->cal_type, data->gain,
			 data->reference_voltage, init_temp);
	}

	return 0;

err_sclk:
	clk_disable_unprepare(data->sclk);
err_clk:
	clk_unprepare(data->clk);
err_clk_sec:
	if (!IS_ERR(data->clk_sec))
		clk_unprepare(data->clk_sec);
err_sensor:
	if (!IS_ERR(data->regulator))
		regulator_disable(data->regulator);

	return ret;
}

static int exynos_tmu_remove(struct platform_device *pdev)
{
	struct exynos_tmu_data *data = platform_get_drvdata(pdev);

	if (data->soc == SOC_ARCH_EXYNOS7 &&
	    data->id >= 0 && data->id < EXYNOS7_NUM_BANKS) {
		mutex_lock(&exynos7_tmu_access);
		if (exynos7_tmu_banks[data->id] == data)
			exynos7_tmu_banks[data->id] = NULL;
		mutex_unlock(&exynos7_tmu_access);
	}

	exynos_tmu_control(pdev, false);
	cancel_work_sync(&data->irq_work);

	clk_disable_unprepare(data->sclk);
	clk_unprepare(data->clk);
	if (!IS_ERR(data->clk_sec))
		clk_unprepare(data->clk_sec);

	if (!IS_ERR(data->regulator))
		regulator_disable(data->regulator);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int exynos_tmu_suspend(struct device *dev)
{
	exynos_tmu_control(to_platform_device(dev), false);

	return 0;
}

static int exynos7_tmu_resume_gpu_off(struct exynos_tmu_data *data)
{
	int num_trips = thermal_zone_get_num_trips(data->tzd);
	int i, ntrips = min_t(int, num_trips, data->ntrip);
	int ret = 0;

	clk_enable(data->clk);
	writel(data->saved_control & ~(BIT(EXYNOS_TMU_THERM_TRIP_EN_SHIFT) |
				       BIT(EXYNOS_TMU_CORE_EN_SHIFT)),
	       data->base + EXYNOS_TMU_REG_CONTROL);
	writel(EXYNOS7_TMU_SAMPLING_DEFAULT,
	       data->base + EXYNOS7_TMU_REG_SAMPLING_INTERVAL);

	for (i = 0; i < ntrips; i++) {
		struct thermal_trip trip;

		ret = thermal_zone_get_trip(data->tzd, i, &trip);
		if (ret)
			break;

		data->tmu_set_trip_temp(data, i, trip.temperature / MCELSIUS);
		data->tmu_set_trip_hyst(data, i, trip.temperature / MCELSIUS,
					trip.hysteresis / MCELSIUS);
	}

	if (!ret && ntrips < 8) {
		u32 r7 = temp_to_code(data, 115);
		u32 f7 = temp_to_code(data, 110);
		u32 th = readl(data->base + EXYNOS7_THD_TEMP_RISE7_6) & 0xffff;

		writel((r7 << 16) | th, data->base + EXYNOS7_THD_TEMP_RISE7_6);
		th = readl(data->base + EXYNOS7_THD_TEMP_FALL7_6) & 0xffff;
		writel((f7 << 16) | th, data->base + EXYNOS7_THD_TEMP_FALL7_6);
	}

	exynos7_mask_all_irqs(data);
	data->enabled = true;
	clk_disable(data->clk);

	return ret;
}

static int exynos_tmu_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct exynos_tmu_data *data = platform_get_drvdata(pdev);

	if (data->soc == SOC_ARCH_EXYNOS7 && data->id == 2) {
		int ret;

		mutex_lock(&data->lock);
		if (data->gpu_off) {
			ret = exynos7_tmu_resume_gpu_off(data);
			mutex_unlock(&data->lock);
			return ret;
		}
		mutex_unlock(&data->lock);
	}

	exynos_tmu_initialize(pdev);
	exynos_tmu_control(pdev, true);

	return 0;
}

static SIMPLE_DEV_PM_OPS(exynos_tmu_pm,
			 exynos_tmu_suspend, exynos_tmu_resume);
#define EXYNOS_TMU_PM	(&exynos_tmu_pm)
#else
#define EXYNOS_TMU_PM	NULL
#endif

int m86_tmu_get_cached_temp(int *temp)
{
	unsigned long flags;
	int i, maximum = INT_MIN;
	bool found = false, have_valid = false;

	spin_lock_irqsave(&exynos7_tmu_cache_lock, flags);
	for (i = 0; i < EXYNOS7_NUM_BANKS; i++) {
		struct exynos_tmu_data *b = READ_ONCE(exynos7_tmu_banks[i]);

		if (!b)
			continue;
		found = true;
		if (i == 2 && b->gpu_off)
			continue;
		if (b->temp_valid) {
			maximum = max(maximum, b->last_temp);
			have_valid = true;
		}
	}
	spin_unlock_irqrestore(&exynos7_tmu_cache_lock, flags);

	if (!found)
		return -EPROBE_DEFER;
	if (!have_valid)
		return -ENODATA;
	*temp = maximum;
	return 0;
}
EXPORT_SYMBOL_GPL(m86_tmu_get_cached_temp);

int m86_tmu_read_max_temp(int *temp)
{
	int i, val, ret = 0, maximum = INT_MIN;

	mutex_lock(&exynos7_tmu_access);
	for (i = 0; i < EXYNOS7_NUM_BANKS; i++) {
		if (!exynos7_tmu_banks[i]) {
			ret = -EPROBE_DEFER;
			goto unlock;
		}
	}

	for (i = 0; i < EXYNOS7_NUM_BANKS; i++) {
		struct exynos_tmu_data *b = exynos7_tmu_banks[i];

		mutex_lock(&b->lock);
		if (i == 2 && b->gpu_off) {
			mutex_unlock(&b->lock);
			continue;
		}
		ret = exynos_get_temp_locked(b, &val);
		mutex_unlock(&b->lock);

		if (ret == -EAGAIN && i == 2 && READ_ONCE(b->gpu_off))
			continue;
		if (ret)
			goto unlock;
		maximum = max(maximum, val);
	}

	if (maximum == INT_MIN) {
		ret = -ENODATA;
		goto unlock;
	}
	*temp = maximum;
unlock:
	mutex_unlock(&exynos7_tmu_access);
	return ret;
}
EXPORT_SYMBOL_GPL(m86_tmu_read_max_temp);

int m86_tmu_register_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&exynos7_tmu_chain, nb);
}
EXPORT_SYMBOL_GPL(m86_tmu_register_notifier);

int m86_tmu_unregister_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&exynos7_tmu_chain, nb);
}
EXPORT_SYMBOL_GPL(m86_tmu_unregister_notifier);

int m86_tmu_gpu_power_off(void)
{
	struct exynos_tmu_data *t;
	u32 control, status;
	unsigned long flags;
	int ret = 0;

	mutex_lock(&exynos7_tmu_access);
	t = exynos7_tmu_banks[2];
	if (!t) {
		ret = -EPROBE_DEFER;
		goto unlock;
	}

	mutex_lock(&t->lock);
	if (t->gpu_off)
		goto unlock_dev;

	clk_enable(t->clk);
	control = readl(t->base + EXYNOS_TMU_REG_CONTROL);
	if (control & BIT(EXYNOS_TMU_CORE_EN_SHIFT))
		t->saved_control = control;

	exynos7_mask_all_irqs(t);
	writel(control & ~(BIT(EXYNOS_TMU_THERM_TRIP_EN_SHIFT) |
			   BIT(EXYNOS_TMU_CORE_EN_SHIFT)),
	       t->base + EXYNOS_TMU_REG_CONTROL);
	ret = readl_poll_timeout(t->base + EXYNOS_TMU_REG_STATUS, status,
				 status & EXYNOS7_TMU_STATUS_IDLE, 10, 20000);
	if (ret) {
		writel(t->saved_control, t->base + EXYNOS_TMU_REG_CONTROL);
		writel(t->irq_en, t->base + EXYNOS7_TMU_REG_INTEN);
		clk_disable(t->clk);
		dev_err(&t->tzd->device,
			"TMU2 power-off idle timeout: status=%#x\n", status);
		goto unlock_dev;
	}

	writel(EXYNOS7_TMU_CLEAR_ALL_INT, t->base + EXYNOS7_TMU_REG_INTPEND);
	clk_disable(t->clk);

	spin_lock_irqsave(&exynos7_tmu_cache_lock, flags);
	t->gpu_off = true;
	spin_unlock_irqrestore(&exynos7_tmu_cache_lock, flags);

unlock_dev:
	mutex_unlock(&t->lock);
unlock:
	mutex_unlock(&exynos7_tmu_access);
	return ret;
}
EXPORT_SYMBOL_GPL(m86_tmu_gpu_power_off);

int m86_tmu_gpu_power_on(void)
{
	struct exynos_tmu_data *t;
	unsigned long flags;
	int attempt, temp = 0, ret = 0;

	mutex_lock(&exynos7_tmu_access);
	t = exynos7_tmu_banks[2];
	if (!t) {
		ret = -EPROBE_DEFER;
		goto unlock;
	}

	mutex_lock(&t->lock);
	clk_enable(t->clk);
	if (!t->enabled) {
		/* System resume may power on pd-g3d during resume_noirq before
		 * exynos_tmu_resume() runs; clear gpu_off so resume enables TMU2.
		 */
		spin_lock_irqsave(&exynos7_tmu_cache_lock, flags);
		t->gpu_off = false;
		spin_unlock_irqrestore(&exynos7_tmu_cache_lock, flags);
		clk_disable(t->clk);
		goto unlock_dev;
	}
	if (!t->gpu_off) {
		if (!(readl(t->base + EXYNOS_TMU_REG_CONTROL) &
		      BIT(EXYNOS_TMU_CORE_EN_SHIFT)))
			ret = -EIO;
		clk_disable(t->clk);
		goto unlock_dev;
	}

	exynos7_mask_all_irqs(t);
	writel((t->saved_control & ~BIT(EXYNOS_TMU_THERM_TRIP_EN_SHIFT)) |
	       BIT(EXYNOS_TMU_CORE_EN_SHIFT),
	       t->base + EXYNOS_TMU_REG_CONTROL);

	spin_lock_irqsave(&exynos7_tmu_cache_lock, flags);
	t->gpu_off = false;
	spin_unlock_irqrestore(&exynos7_tmu_cache_lock, flags);

	clk_disable(t->clk);

	for (attempt = 0; attempt < 20; attempt++) {
		ret = exynos_get_temp_locked(t, &temp);
		if (!ret && temp >= 5000 && temp <= 105000)
			break;
		if (ret && ret != -EAGAIN)
			break;
		usleep_range(150, 250);
	}

	clk_enable(t->clk);
	if (ret || temp < 5000 || temp > 105000) {
		u32 status = readl(t->base + EXYNOS_TMU_REG_STATUS);

		writel(t->saved_control & ~(BIT(EXYNOS_TMU_THERM_TRIP_EN_SHIFT) |
					    BIT(EXYNOS_TMU_CORE_EN_SHIFT)),
		       t->base + EXYNOS_TMU_REG_CONTROL);
		writel(EXYNOS7_TMU_CLEAR_ALL_INT,
		       t->base + EXYNOS7_TMU_REG_INTPEND);
		spin_lock_irqsave(&exynos7_tmu_cache_lock, flags);
		t->gpu_off = true;
		spin_unlock_irqrestore(&exynos7_tmu_cache_lock, flags);
		clk_disable(t->clk);
		dev_err(&t->tzd->device,
			"TMU2 power-on settle failed: ret=%d temp=%d status=%#x\n",
			ret, temp, status);
		ret = ret ?: -ERANGE;
		goto unlock_dev;
	}

	writel(EXYNOS7_TMU_CLEAR_ALL_INT, t->base + EXYNOS7_TMU_REG_INTPEND);
	writel(t->saved_control | BIT(EXYNOS_TMU_CORE_EN_SHIFT),
	       t->base + EXYNOS_TMU_REG_CONTROL);
	writel(t->irq_en, t->base + EXYNOS7_TMU_REG_INTEN);
	clk_disable(t->clk);

unlock_dev:
	mutex_unlock(&t->lock);
unlock:
	mutex_unlock(&exynos7_tmu_access);
	return ret;
}
EXPORT_SYMBOL_GPL(m86_tmu_gpu_power_on);

static struct platform_driver exynos_tmu_driver = {
	.driver = {
		.name   = "exynos-tmu",
		.pm     = EXYNOS_TMU_PM,
		.of_match_table = exynos_tmu_match,
	},
	.probe = exynos_tmu_probe,
	.remove	= exynos_tmu_remove,
};

module_platform_driver(exynos_tmu_driver);

MODULE_DESCRIPTION("Exynos TMU Driver");
MODULE_AUTHOR("Donggeun Kim <dg77.kim@samsung.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:exynos-tmu");
