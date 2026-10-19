// SPDX-License-Identifier: GPL-2.0
/*
 * RNG driver for Exynos TRNGs
 *
 * Author: Łukasz Stelmach <l.stelmach@samsung.com>
 *
 * Copyright 2017 (c) Samsung Electronics Software, Inc.
 *
 * Based on the Exynos PRNG driver drivers/crypto/exynos-rng by
 * Krzysztof Kozłowski <krzk@kernel.org>
 */

#include <linux/arm-smccc.h>
#include <linux/clk.h>
#include <linux/crypto.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/hw_random.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/property.h>

#define EXYNOS_TRNG_CLKDIV         (0x0)

#define EXYNOS_TRNG_CTRL           (0x20)
#define EXYNOS_TRNG_CTRL_RNGEN     BIT(31)

#define EXYNOS_TRNG_POST_CTRL      (0x30)
#define EXYNOS_TRNG_ONLINE_CTRL    (0x40)
#define EXYNOS_TRNG_ONLINE_STAT    (0x44)
#define EXYNOS_TRNG_ONLINE_MAXCHI2 (0x48)
#define EXYNOS_TRNG_FIFO_CTRL      (0x50)
#define EXYNOS_TRNG_FIFO_0         (0x80)
#define EXYNOS_TRNG_FIFO_1         (0x84)
#define EXYNOS_TRNG_FIFO_2         (0x88)
#define EXYNOS_TRNG_FIFO_3         (0x8c)
#define EXYNOS_TRNG_FIFO_4         (0x90)
#define EXYNOS_TRNG_FIFO_5         (0x94)
#define EXYNOS_TRNG_FIFO_6         (0x98)
#define EXYNOS_TRNG_FIFO_7         (0x9c)
#define EXYNOS_TRNG_FIFO_LEN       (8)
#define EXYNOS_TRNG_CLOCK_RATE     (500000)

/* Driver feature flags */
#define EXYNOS_SMC                 BIT(0)

#define EXYNOS_SMC_CALL_VAL(func_num) \
	ARM_SMCCC_CALL_VAL(ARM_SMCCC_FAST_CALL, \
			   ARM_SMCCC_SMC_32, \
			   ARM_SMCCC_OWNER_SIP, \
			   func_num)

/* SMC command for DTRNG access */
#define SMC_CMD_RANDOM             EXYNOS_SMC_CALL_VAL(0x1012)

/* SMC_CMD_RANDOM: arguments */
#define HWRNG_INIT                 0x0
#define HWRNG_EXIT                 0x1
#define HWRNG_GET_DATA             0x2
#define HWRNG_RESUME               0x3

/* SMC_CMD_RANDOM: return values */
#define HWRNG_RET_OK               0x0
#define HWRNG_RET_RETRY_ERROR      0x2
#define HWRNG_RET_INVALID_FLAG_ERROR 0x3
#define HWRNG_RET_START_UP_TEST_DONE 0x5

#define HWRNG_MAX_TRIES            100

struct exynos_trng_dev {
	struct device    *dev;
	void __iomem     *mem;
	struct clk       *clk;  /* operating clock */
	struct clk       *pclk; /* bus clock */
	struct hwrng     rng;
	unsigned long    flags;
	struct mutex     smc_lock;
	bool             smc_session;
};

static int exynos_trng_do_read_reg(struct hwrng *rng, void *data, size_t max,
				   bool wait)
{
	struct exynos_trng_dev *trng = (struct exynos_trng_dev *)rng->priv;
	int val;

	max = min_t(size_t, max, (EXYNOS_TRNG_FIFO_LEN * 4));

	writel_relaxed(max * 8, trng->mem + EXYNOS_TRNG_FIFO_CTRL);
	val = readl_poll_timeout(trng->mem + EXYNOS_TRNG_FIFO_CTRL, val,
				 val == 0, 200, 1000000);
	if (val < 0)
		return val;

	memcpy_fromio(data, trng->mem + EXYNOS_TRNG_FIFO_0, max);

	return max;
}

static int exynos_trng_smc_init(struct exynos_trng_dev *trng)
{
	struct arm_smccc_res res;
	int tries;

	for (tries = 0; tries < HWRNG_MAX_TRIES; tries++) {
		arm_smccc_smc(SMC_CMD_RANDOM, HWRNG_INIT, 0, 0, 0, 0, 0, 0,
			      &res);
		if (res.a0 == HWRNG_RET_OK) {
			trng->smc_session = true;
			return 0;
		}
		if (res.a0 != HWRNG_RET_RETRY_ERROR)
			break;
		usleep_range(50, 100);
	}

	dev_warn(trng->dev, "SMC TRNG init failed (ret=%ld)\n", (long)res.a0);
	return -EIO;
}

static void exynos_trng_smc_exit(struct exynos_trng_dev *trng)
{
	struct arm_smccc_res res;
	int tries;

	for (tries = 0; tries < HWRNG_MAX_TRIES; tries++) {
		arm_smccc_smc(SMC_CMD_RANDOM, HWRNG_EXIT, 0, 0, 0, 0, 0, 0,
			      &res);
		if (res.a0 == HWRNG_RET_OK ||
		    res.a0 == HWRNG_RET_INVALID_FLAG_ERROR)
			break;
		if (res.a0 != HWRNG_RET_RETRY_ERROR)
			break;
		usleep_range(50, 100);
	}
	trng->smc_session = false;
}

/* FIPS start-up test: read 129 blocks from LDFW or wait for the done flag. */
static int exynos_trng_self_test(struct exynos_trng_dev *trng)
{
	struct arm_smccc_res res;
	int blocks = 0;
	int tries = 0;

	while (blocks < 129) {
		arm_smccc_smc(SMC_CMD_RANDOM, HWRNG_GET_DATA, 1, 0, 0, 0, 0, 0,
			      &res);
		switch (res.a0) {
		case HWRNG_RET_OK:
			blocks++;
			tries = 0;
			break;
		case HWRNG_RET_START_UP_TEST_DONE:
			return 0;
		case HWRNG_RET_RETRY_ERROR:
			if (++tries >= HWRNG_MAX_TRIES)
				return -ETIMEDOUT;
			usleep_range(50, 100);
			break;
		default:
			dev_err(trng->dev, "SMC TRNG start-up test failed (ret=%ld)\n",
				(long)res.a0);
			return -EIO;
		}
	}

	return 0;
}

static int exynos_trng_do_read_smc(struct hwrng *rng, void *data, size_t max,
				   bool wait)
{
	struct exynos_trng_dev *trng = (struct exynos_trng_dev *)rng->priv;
	struct arm_smccc_res res;
	unsigned char *buf = data;
	size_t copied = 0;
	int tries = 0;
	int ret;

	mutex_lock(&trng->smc_lock);

	ret = exynos_trng_smc_init(trng);
	if (ret)
		goto out_unlock;

	while (copied < max) {
		u32 tmp[2];

		arm_smccc_smc(SMC_CMD_RANDOM, HWRNG_GET_DATA, 0, 0, 0, 0, 0, 0,
			      &res);
		switch (res.a0) {
		case HWRNG_RET_OK:
			tmp[0] = (u32)res.a2;
			tmp[1] = (u32)res.a3;
			/* The tail may be shorter than the 8-byte SMC block. */
			memcpy(buf + copied, tmp,
			       min_t(size_t, sizeof(tmp), max - copied));
			copied += min_t(size_t, sizeof(tmp), max - copied);
			tries = 0;
			break;
		case HWRNG_RET_RETRY_ERROR:
			if (!wait) {
				ret = -EAGAIN;
				goto out_exit;
			}
			if (++tries >= HWRNG_MAX_TRIES) {
				ret = -ETIMEDOUT;
				goto out_exit;
			}
			usleep_range(50, 100);
			cond_resched();
			break;
		default:
			ret = -EIO;
			goto out_exit;
		}
	}
	ret = 0;

out_exit:
	if (trng->smc_session)
		exynos_trng_smc_exit(trng);
out_unlock:
	mutex_unlock(&trng->smc_lock);
	return copied ? (int)copied : ret;
}

static int exynos_trng_init_reg(struct hwrng *rng)
{
	struct exynos_trng_dev *trng = (struct exynos_trng_dev *)rng->priv;
	unsigned long sss_rate;
	u32 val;

	sss_rate = clk_get_rate(trng->clk);

	/*
	 * For most TRNG circuits the clock frequency of under 500 kHz
	 * is safe.
	 */
	val = sss_rate / (EXYNOS_TRNG_CLOCK_RATE * 2);
	if (val > 0x7fff) {
		dev_err(trng->dev, "clock divider too large: %d", val);
		return -ERANGE;
	}
	val = val << 1;
	writel_relaxed(val, trng->mem + EXYNOS_TRNG_CLKDIV);

	/* Enable the generator. */
	val = EXYNOS_TRNG_CTRL_RNGEN;
	writel_relaxed(val, trng->mem + EXYNOS_TRNG_CTRL);

	/*
	 * Disable post-processing. /dev/hwrng is supposed to deliver
	 * unprocessed data.
	 */
	writel_relaxed(0, trng->mem + EXYNOS_TRNG_POST_CTRL);

	return 0;
}

static int exynos_trng_init_smc(struct hwrng *rng)
{
	struct exynos_trng_dev *trng = (struct exynos_trng_dev *)rng->priv;
	int ret;

	mutex_lock(&trng->smc_lock);
	ret = exynos_trng_smc_init(trng);
	if (!ret) {
		ret = exynos_trng_self_test(trng);
		exynos_trng_smc_exit(trng);
	}
	mutex_unlock(&trng->smc_lock);

	return ret;
}

static int exynos_trng_probe(struct platform_device *pdev)
{
	struct exynos_trng_dev *trng;
	int ret = -ENOMEM;

	trng = devm_kzalloc(&pdev->dev, sizeof(*trng), GFP_KERNEL);
	if (!trng)
		return ret;

	platform_set_drvdata(pdev, trng);
	mutex_init(&trng->smc_lock);
	trng->dev = &pdev->dev;
	trng->flags = (unsigned long)device_get_match_data(&pdev->dev);

	trng->rng.name = devm_kstrdup(&pdev->dev, dev_name(&pdev->dev),
				      GFP_KERNEL);
	if (!trng->rng.name)
		return ret;

	trng->rng.priv = (unsigned long)trng;

	if (trng->flags & EXYNOS_SMC) {
		/* Pre-test SMC availability before runtime PM & registration. */
		struct arm_smccc_res res;
		int tries;

		for (tries = 0; tries < HWRNG_MAX_TRIES; tries++) {
			arm_smccc_smc(SMC_CMD_RANDOM, HWRNG_INIT, 0, 0, 0, 0, 0,
				      0, &res);
			if (res.a0 == HWRNG_RET_OK)
				break;
			if (res.a0 != HWRNG_RET_RETRY_ERROR)
				break;
			usleep_range(50, 100);
		}

		if (res.a0 != HWRNG_RET_OK) {
			dev_warn(trng->dev, "SMC TRNG unavailable (ret=%ld); LDFW not loaded\n",
				 (long)res.a0);
			return -ENODEV;
		}
		arm_smccc_smc(SMC_CMD_RANDOM, HWRNG_EXIT, 0, 0, 0, 0, 0, 0,
			      &res);

		trng->clk = devm_clk_get_optional_enabled(&pdev->dev, "secss");
		if (IS_ERR(trng->clk))
			return dev_err_probe(&pdev->dev, PTR_ERR(trng->clk),
					     "Could not get clock\n");

		trng->pclk = devm_clk_get_optional_enabled(&pdev->dev, "pclk");
		if (IS_ERR(trng->pclk))
			return dev_err_probe(&pdev->dev, PTR_ERR(trng->pclk),
					     "Could not get pclk\n");

		trng->rng.init = exynos_trng_init_smc;
		trng->rng.read = exynos_trng_do_read_smc;
	} else {
		trng->rng.init = exynos_trng_init_reg;
		trng->rng.read = exynos_trng_do_read_reg;

		trng->mem = devm_platform_ioremap_resource(pdev, 0);
		if (IS_ERR(trng->mem))
			return PTR_ERR(trng->mem);

		trng->clk = devm_clk_get_enabled(&pdev->dev, "secss");
		if (IS_ERR(trng->clk))
			return dev_err_probe(&pdev->dev, PTR_ERR(trng->clk),
					     "Could not get clock\n");

		trng->pclk = devm_clk_get_optional_enabled(&pdev->dev, "pclk");
		if (IS_ERR(trng->pclk))
			return dev_err_probe(&pdev->dev, PTR_ERR(trng->pclk),
					     "Could not get pclk\n");
	}

	pm_runtime_enable(&pdev->dev);
	ret = pm_runtime_resume_and_get(&pdev->dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "Could not get runtime PM.\n");
		goto err_pm_get;
	}

	ret = devm_hwrng_register(&pdev->dev, &trng->rng);
	if (ret) {
		dev_err(&pdev->dev, "Could not register hwrng device.\n");
		goto err_register;
	}

	dev_info(&pdev->dev, "Exynos True Random Number Generator.\n");
	return 0;

err_register:
	pm_runtime_put_noidle(&pdev->dev);
err_pm_get:
	pm_runtime_disable(&pdev->dev);
	return ret;
}

static int exynos_trng_remove(struct platform_device *pdev)
{
	struct exynos_trng_dev *trng = platform_get_drvdata(pdev);

	if (trng->flags & EXYNOS_SMC) {
		mutex_lock(&trng->smc_lock);
		if (trng->smc_session)
			exynos_trng_smc_exit(trng);
		mutex_unlock(&trng->smc_lock);
	}

	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);

	return 0;
}

static int exynos_trng_suspend(struct device *dev)
{
	struct exynos_trng_dev *trng = dev_get_drvdata(dev);

	if (trng->flags & EXYNOS_SMC) {
		mutex_lock(&trng->smc_lock);
		/* do_read_smc() already closes the LDFW session; only exit an
		 * actually active session, otherwise LDFW returns
		 * HWRNG_RET_INVALID_FLAG_ERROR and aborts system suspend.
		 */
		if (trng->smc_session)
			exynos_trng_smc_exit(trng);
		mutex_unlock(&trng->smc_lock);
	}

	pm_runtime_put_sync(dev);
	return 0;
}

static int exynos_trng_resume(struct device *dev)
{
	struct exynos_trng_dev *trng = dev_get_drvdata(dev);
	int ret;

	ret = pm_runtime_resume_and_get(dev);
	if (ret < 0) {
		dev_err(dev, "Could not get runtime PM.\n");
		return ret;
	}

	if (trng->flags & EXYNOS_SMC) {
		struct arm_smccc_res res;
		int tries;

		mutex_lock(&trng->smc_lock);
		for (tries = 0; tries < HWRNG_MAX_TRIES; tries++) {
			arm_smccc_smc(SMC_CMD_RANDOM, HWRNG_RESUME, 0, 0, 0,
				      0, 0, 0, &res);
			if (res.a0 == HWRNG_RET_OK)
				break;
			if (res.a0 != HWRNG_RET_RETRY_ERROR)
				break;
			usleep_range(50, 100);
		}
		if (res.a0 != HWRNG_RET_OK) {
			ret = -EIO;
		} else {
			/* Re-init once and close the session so the next read
			 * starts from a clean LDFW state.
			 */
			ret = exynos_trng_smc_init(trng);
			if (!ret)
				exynos_trng_smc_exit(trng);
		}
		mutex_unlock(&trng->smc_lock);
	}

	if (ret)
		pm_runtime_put_sync(dev);
	return ret;
}

static DEFINE_SIMPLE_DEV_PM_OPS(exynos_trng_pm_ops, exynos_trng_suspend,
				exynos_trng_resume);

static const struct of_device_id exynos_trng_dt_match[] = {
	{
		.compatible = "samsung,exynos5250-trng",
	}, {
		.compatible = "samsung,exynos7420-trng",
		.data = (void *)EXYNOS_SMC,
	}, {
		.compatible = "samsung,exynos850-trng",
		.data = (void *)EXYNOS_SMC,
	},
	{ },
};
MODULE_DEVICE_TABLE(of, exynos_trng_dt_match);

static struct platform_driver exynos_trng_driver = {
	.driver = {
		.name = "exynos-trng",
		.pm = pm_sleep_ptr(&exynos_trng_pm_ops),
		.of_match_table = exynos_trng_dt_match,
	},
	.probe = exynos_trng_probe,
	.remove = exynos_trng_remove,
};

module_platform_driver(exynos_trng_driver);
MODULE_AUTHOR("Łukasz Stelmach");
MODULE_DESCRIPTION("H/W TRNG driver for Exynos chips");
MODULE_LICENSE("GPL v2");
