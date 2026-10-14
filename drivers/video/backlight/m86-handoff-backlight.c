// SPDX-License-Identifier: GPL-2.0-only
/* Samsung S6E3FA3 FHD AMOLED panel and DSIM backlight driver.
 * Supports Meizu PRO 5 (m86 / Exynos 7420) command-mode display.
 * DCS sequence provenance: vendor s6e3fa3_mipi_lcd.c and regs-dsim.h.
 */
#include <linux/backlight.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/m86-fts.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/property.h>

#define DSIM_INTSRC 0x34
#define DSIM_INTMSK 0x38
#define DSIM_PKTHDR 0x3c
#define DSIM_PAYLOAD 0x40
#define DSIM_FIFOCTRL 0x4c
#define PH_EMPTY BIT(28)
#define PL_EMPTY BIT(29)
#define PH_FULL BIT(23)
#define PL_FULL BIT(21)
#define FIFO_TIMEOUT_US 50000

struct m86_handoff {
	void __iomem *regs;
	struct clk_bulk_data *clocks;
	int num_clocks;
	struct mutex lock;
	struct backlight_device *bl;
	bool ready;
	bool blanked;
	bool suspended;
	bool faulted;
};

struct s6e3fa3_cmd {
	u8 type;
	u8 len;
	u8 data[3];
	u16 delay_ms;
};

/* S6E3FA3 Sleep sequence (DISP_OFF -> SLEEP_IN) */
static const struct s6e3fa3_cmd s6e3fa3_sleep_seq[] = {
	{ 0x05, 1, { 0x28 }, 10 },   /* DISP_OFF, 10ms delay */
	{ 0x05, 1, { 0x10 }, 150 },  /* SLEEP_IN, 150ms delay */
};

/* S6E3FA3 Wake & initialization sequence (vendor s6e3fa3_mipi_lcd.c) */
static const struct s6e3fa3_cmd s6e3fa3_wake_seq[] = {
	{ 0x29, 3, { 0xF0, 0x5A, 0x5A }, 0 },    /* S_PASSWD_KEY_EN_0 */
	{ 0x05, 1, { 0x11 }, 120 },              /* SLEEP_OUT, 120ms */
	{ 0x15, 2, { 0xF9, 0x12 }, 0 },          /* J_MIC */
	{ 0x29, 3, { 0xF0, 0xA5, 0xA5 }, 0 },    /* S_PASSWD_KEY_EN_1 */
	{ 0x29, 3, { 0xFC, 0x5A, 0x5A }, 0 },    /* LEVEL3_UNLOCK */
	{ 0x15, 2, { 0xB0, 0x1E }, 0 },          /* GLOBAL_PARA */
	{ 0x15, 2, { 0xFD, 0x94 }, 0 },          /* AVC_SETTING */
	{ 0x29, 3, { 0xFC, 0xA5, 0xA5 }, 0 },    /* LEVEL3_LOCK */
	{ 0x15, 2, { 0x53, 0x20 }, 0 },          /* DIMMING_SPEED */
	{ 0x15, 2, { 0x55, 0x00 }, 10 },         /* ACL_MODE, 10ms */
	{ 0x05, 1, { 0x29 }, 0 },                /* DISP_ON */
	{ 0x15, 2, { 0x35, 0x00 }, 20 },         /* TEON (mode 0: V-blank), 20ms */
};

/*
 * Send DCS or Generic packet through DSIM command FIFO.
 * Caller holds ctx->lock.
 */
static int m86_send_packet(struct m86_handoff *ctx, u8 type, const u8 *payload, size_t len)
{
	bool long_write = (type == 0x39 || type == 0x29 || len > 2);
	u32 status, empty = long_write ? PL_EMPTY : PH_EMPTY;
	u32 clear = long_write ? (PH_EMPTY | PL_EMPTY) : PH_EMPTY;
	int ret;

	if (ctx->faulted)
		return -EIO;
	if (!ctx->ready)
		return -EBUSY;

	if ((readl(ctx->regs + DSIM_INTMSK) & empty) != empty)
		return -EBUSY;

	ret = readl_poll_timeout(ctx->regs + DSIM_FIFOCTRL, status,
				 !(status & (PH_FULL | PL_FULL)), 100,
				 FIFO_TIMEOUT_US);
	if (ret)
		goto fault;

	writel(clear, ctx->regs + DSIM_INTSRC);

	if (long_write) {
		size_t i;

		for (i = 0; i < len; i += 4) {
			u32 word = 0;
			size_t j;

			for (j = 0; j < 4 && (i + j) < len; j++)
				word |= ((u32)payload[i + j]) << (j * 8);
			writel(word, ctx->regs + DSIM_PAYLOAD);
		}
		writel(type | (((u32)(len & 0xff)) << 8) | (((u32)((len >> 8) & 0xff)) << 16),
		       ctx->regs + DSIM_PKTHDR);
	} else {
		u32 hdr = type;

		if (len > 0)
			hdr |= ((u32)payload[0]) << 8;
		if (len > 1)
			hdr |= ((u32)payload[1]) << 16;
		writel(hdr, ctx->regs + DSIM_PKTHDR);
	}

	ret = readl_poll_timeout(ctx->regs + DSIM_INTSRC, status,
				 (status & empty) == empty, 100, FIFO_TIMEOUT_US);
	if (ret)
		goto fault;

	writel(clear, ctx->regs + DSIM_INTSRC);
	return 0;

fault:
	ctx->faulted = true;
	return ret;
}

static int __maybe_unused s6e3fa3_enter_sleep(struct m86_handoff *ctx)
{
	int i, ret = 0;

	if (ctx->blanked)
		return 0;

	for (i = 0; i < ARRAY_SIZE(s6e3fa3_sleep_seq); i++) {
		const struct s6e3fa3_cmd *cmd = &s6e3fa3_sleep_seq[i];

		ret = m86_send_packet(ctx, cmd->type, cmd->data, cmd->len);
		if (ret) {
			if (ctx->bl)
				dev_err(&ctx->bl->dev,
					"failed to send sleep cmd 0x%02x: %d\n",
					cmd->data[0], ret);
			return ret;
		}
		if (cmd->delay_ms)
			msleep(cmd->delay_ms);
	}

	ctx->blanked = true;
	return 0;
}

static int __maybe_unused s6e3fa3_exit_sleep(struct m86_handoff *ctx)
{
	u32 mask;
	int i, ret = 0;

	if (!ctx->blanked)
		return 0;

	/* Re-establish FIFO interrupt masks */
	mask = readl(ctx->regs + DSIM_INTMSK);
	writel(mask | PH_EMPTY | PL_EMPTY, ctx->regs + DSIM_INTMSK);
	readl(ctx->regs + DSIM_INTMSK);
	ctx->ready = true;
	ctx->faulted = false;

	/* Send full S6E3FA3 wake-up and initialization sequence */
	for (i = 0; i < ARRAY_SIZE(s6e3fa3_wake_seq); i++) {
		const struct s6e3fa3_cmd *cmd = &s6e3fa3_wake_seq[i];

		ret = m86_send_packet(ctx, cmd->type, cmd->data, cmd->len);
		if (ret) {
			if (ctx->bl)
				dev_err(&ctx->bl->dev,
					"failed to send wake cmd 0x%02x: %d\n",
					cmd->data[0], ret);
			return ret;
		}
		if (cmd->delay_ms)
			msleep(cmd->delay_ms);
	}

	ctx->blanked = false;
	return 0;
}

static int m86_update_status(struct backlight_device *bl)
{
	struct m86_handoff *ctx = bl_get_data(bl);
	bool blank = backlight_is_blank(bl);
	int ret = 0;

	mutex_lock(&ctx->lock);
	if (ctx->suspended) {
		mutex_unlock(&ctx->lock);
		return 0;
	}

	if (blank) {
		if (!ctx->blanked) {
			u8 disp_off = 0x28;

			ret = m86_send_packet(ctx, 0x05, &disp_off, 1);
			if (!ret) {
				ctx->blanked = true;
				msleep(10); /* PRO5 DISP_OFF delay */
			}
		}
		goto out;
	}

	if (ctx->faulted) {
		/* Attempt recovery of faulted FIFO upon unblank request */
		u32 mask = readl(ctx->regs + DSIM_INTMSK);
		writel(mask | PH_EMPTY | PL_EMPTY, ctx->regs + DSIM_INTMSK);
		readl(ctx->regs + DSIM_INTMSK);
		mask = readl(ctx->regs + DSIM_INTMSK);
		ctx->ready = (mask & (PH_EMPTY | PL_EMPTY)) == (PH_EMPTY | PL_EMPTY);
		ctx->faulted = !ctx->ready;
	}

	/* PRO5 DIMMING_SPEED={0x53,0x20}; LUMINANCE={0x51,v} */
	{
		u8 dim[2] = { 0x53, 0x20 };
		u8 lum[2] = { 0x51, (u8)backlight_get_brightness(bl) };
		u8 disp_on = 0x29;

		ret = m86_send_packet(ctx, 0x15, dim, 2);
		if (!ret)
			ret = m86_send_packet(ctx, 0x39, lum, 2);
		if (!ret && ctx->blanked) {
			ret = m86_send_packet(ctx, 0x05, &disp_on, 1);
			if (!ret)
				ctx->blanked = false;
		}
	}
out:
	mutex_unlock(&ctx->lock);
	if (!ret)
		m86_fts_set_blank(blank);
	return ret;
}

static const struct backlight_ops m86_ops = {
	.options = BL_CORE_SUSPENDRESUME,
	.update_status = m86_update_status,
};

static void m86_clocks_off(void *data)
{
	struct m86_handoff *ctx = data;

	clk_bulk_disable_unprepare(ctx->num_clocks, ctx->clocks);
}

static int s6e3fa3_pm_suspend(struct device *dev)
{
	struct m86_handoff *ctx = dev_get_drvdata(dev);
	u8 disp_off = 0x28;
	int ret = 0;

	mutex_lock(&ctx->lock);
	if (!ctx->blanked) {
		ret = m86_send_packet(ctx, 0x05, &disp_off, 1);
		if (!ret) {
			ctx->blanked = true;
			msleep(10);
		} else {
			/* Suspend failed: PM core skips resume for failed devices;
			 * do not latch suspended, and clear faulted so system can retry.
			 */
			ctx->faulted = false;
			mutex_unlock(&ctx->lock);
			return ret;
		}
	}
	ctx->suspended = true;
	mutex_unlock(&ctx->lock);
	return 0;
}

static int s6e3fa3_pm_resume(struct device *dev)
{
	struct m86_handoff *ctx = dev_get_drvdata(dev);
	u32 mask;

	mutex_lock(&ctx->lock);
	mask = readl(ctx->regs + DSIM_INTMSK);
	writel(mask | PH_EMPTY | PL_EMPTY, ctx->regs + DSIM_INTMSK);
	readl(ctx->regs + DSIM_INTMSK);
	mask = readl(ctx->regs + DSIM_INTMSK);
	ctx->ready = (mask & (PH_EMPTY | PL_EMPTY)) == (PH_EMPTY | PL_EMPTY);
	ctx->faulted = false;
	ctx->suspended = false;
	mutex_unlock(&ctx->lock);
	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(m86_pm_ops, s6e3fa3_pm_suspend, s6e3fa3_pm_resume);

static int m86_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct backlight_properties props = {
		.type = BACKLIGHT_RAW, .max_brightness = 255,
		.power = FB_BLANK_UNBLANK,
	};
	struct backlight_device *bl;
	struct m86_handoff *ctx;
	u32 initial, mask;
	int ret;

	/* Initial value is bookkeeping only, supplied from known BL state. */
	ret = device_property_read_u32(dev, "default-brightness", &initial);
	if (ret || initial > 255)
		return dev_err_probe(dev, -EINVAL, "require default-brightness 0..255\n");
	props.brightness = initial;
	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;
	mutex_init(&ctx->lock);
	ctx->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(ctx->regs))
		return PTR_ERR(ctx->regs);
	ctx->num_clocks = devm_clk_bulk_get_all(dev, &ctx->clocks);
	if (ctx->num_clocks < 0)
		return dev_err_probe(dev, ctx->num_clocks, "handoff clocks\n");
	if (!ctx->num_clocks)
		return -EINVAL;
	ret = clk_bulk_prepare_enable(ctx->num_clocks, ctx->clocks);
	if (ret)
		return ret;
	ret = devm_add_action_or_reset(dev, m86_clocks_off, ctx);
	if (ret)
		return ret;
	platform_set_drvdata(pdev, ctx);

	/* Exclusively reserve DSIM above, then switch only FIFO completion
	 * sources to polling before publishing the backlight device.
	 * Preserve all other bootloader interrupt masks.
	 */
	mutex_lock(&ctx->lock);
	mask = readl(ctx->regs + DSIM_INTMSK);
	writel(mask | PH_EMPTY | PL_EMPTY, ctx->regs + DSIM_INTMSK);
	readl(ctx->regs + DSIM_INTMSK); /* Flush posted write before publication. */
	mask = readl(ctx->regs + DSIM_INTMSK);
	ctx->ready = (mask & (PH_EMPTY | PL_EMPTY)) == (PH_EMPTY | PL_EMPTY);
	mutex_unlock(&ctx->lock);
	if (!ctx->ready)
		return dev_err_probe(dev, -EIO, "FIFO IRQ mask did not latch\n");

	bl = devm_backlight_device_register(dev, "m86-panel", dev, ctx,
					  &m86_ops, &props);
	if (IS_ERR(bl))
		return PTR_ERR(bl);
	ctx->bl = bl;
	dev_info(dev, "polling FIFO completion; INTMSK=%#010x\n", mask);
	/* No backlight_update_status(): preserve BL brightness at probe. */
	return 0;
}

static const struct of_device_id m86_match[] = {
	{ .compatible = "meizu,m86-s6e3fa3-panel" },
	{ .compatible = "meizu,m86-dsim-handoff-backlight" },
	{ }
};
MODULE_DEVICE_TABLE(of, m86_match);

static struct platform_driver m86_driver = {
	.probe = m86_probe,
	.driver = {
		.name = "m86-s6e3fa3-panel",
		.of_match_table = m86_match,
		.pm = pm_sleep_ptr(&m86_pm_ops),
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(m86_driver);
MODULE_DESCRIPTION("Meizu PRO5 S6E3FA3 AMOLED panel and DSIM backlight controller");
MODULE_LICENSE("GPL");
