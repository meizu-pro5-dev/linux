// SPDX-License-Identifier: GPL-2.0-only
/* Command-only access to an already running Exynos7420 DSIM0.
 * Register/packet provenance: m576-ubuntu decon/{regs-dsim.h,
 * dsim_reg_7420.c,dsim_drv.c,panels/s6e3fa3_mipi_lcd.c}.
 * This is not a DSIM host and cannot restore a lost bootloader handoff.
 */
#include <linux/backlight.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/io.h>
#include <linux/iopoll.h>
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
	bool ready;
	bool blanked;
	bool faulted;
};

/* Caller holds lock. A timeout may have left a queued command: do not retry. */
static int m86_send(struct m86_handoff *ctx, u8 cmd, u8 value, u8 type)
{
	bool long_write = type == 0x39;
	u32 status, empty = long_write ? PL_EMPTY : PH_EMPTY;
	u32 clear = long_write ? PH_EMPTY | PL_EMPTY : PH_EMPTY;
	int ret;

	if (ctx->faulted)
		return -EIO;
	if (!ctx->ready)
		return -EBUSY;
	/* Mask bit=1 disables IRQ (vendor dsim_reg_set_int()).
	 * The backlight may become visible before probe finishes masking;
	 * refuse such a racing update rather than enqueue an unmasked packet.
	 */
	if ((readl(ctx->regs + DSIM_INTMSK) & empty) != empty)
		return -EBUSY;
	ret = readl_poll_timeout(ctx->regs + DSIM_FIFOCTRL, status,
				!(status & (PH_FULL | PL_FULL)), 100,
				FIFO_TIMEOUT_US);
	if (ret)
		goto fault;
	/* INTSRC is W1C; clear only completion bits belonging to this packet. */
	writel(clear, ctx->regs + DSIM_INTSRC);
	if (long_write) {
		writel(cmd | ((u32)value << 8), ctx->regs + DSIM_PAYLOAD);
		writel(0x39 | (2 << 8), ctx->regs + DSIM_PKTHDR);
	} else {
		/* PRO5 DISP_ON/OFF vendor INIT_CMD(..., 0x05, ...). */
		writel(type | ((u32)cmd << 8) | ((u32)value << 16),
		       ctx->regs + DSIM_PKTHDR);
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

static int m86_update_status(struct backlight_device *bl)
{
	struct m86_handoff *ctx = bl_get_data(bl);
	bool blank = backlight_is_blank(bl);
	int ret = 0;

	mutex_lock(&ctx->lock);
	if (blank) {
		if (!ctx->blanked) {
			ret = m86_send(ctx, 0x28, 0, 0x05);
			if (!ret) {
				ctx->blanked = true;
				msleep(10); /* PRO5 DISP_OFF delay. */
			}
		}
		goto out;
	}
	/* PRO5 DIMMING_SPEED/HBM_SWITCH={0x53,0x20}; LUMINANCE={0x51,v}.
	 * 0x53 uses vendor INIT_CMD short-with-param; 0x51 uses the long
	 * packet in vendor update_brightness(). No gamma/private-register writes.
	 */
	ret = m86_send(ctx, 0x53, 0x20, 0x15);
	if (!ret)
		ret = m86_send(ctx, 0x51, backlight_get_brightness(bl), 0x39);
	if (!ret && ctx->blanked) {
		ret = m86_send(ctx, 0x29, 0, 0x05);
		if (!ret)
			ctx->blanked = false;
	}
out:
	mutex_unlock(&ctx->lock);
	return ret;
}

static const struct backlight_ops m86_ops = {
	.update_status = m86_update_status,
};

static void m86_clocks_off(void *data)
{
	struct m86_handoff *ctx = data;

	clk_bulk_disable_unprepare(ctx->num_clocks, ctx->clocks);
}

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
	bl = devm_backlight_device_register(dev, "m86-handoff", dev, ctx,
					  &m86_ops, &props);
	if (IS_ERR(bl))
		return PTR_ERR(bl);
	/* Publication can race update_status; serialize mask setup with writes. */
	mutex_lock(&ctx->lock);
	/* Exclusively reserve DSIM above, then switch only FIFO completion
	 * sources to polling. Preserve all other bootloader interrupt masks.
	 */
	mask = readl(ctx->regs + DSIM_INTMSK);
	writel(mask | PH_EMPTY | PL_EMPTY, ctx->regs + DSIM_INTMSK);
	readl(ctx->regs + DSIM_INTMSK); /* Flush posted write before publication. */
	mask = readl(ctx->regs + DSIM_INTMSK);
	ctx->ready = (mask & (PH_EMPTY | PL_EMPTY)) == (PH_EMPTY | PL_EMPTY);
	mutex_unlock(&ctx->lock);
	if ((mask & (PH_EMPTY | PL_EMPTY)) != (PH_EMPTY | PL_EMPTY))
		return dev_err_probe(dev, -EIO, "FIFO IRQ mask did not latch\n");
	dev_info(dev, "polling FIFO completion; INTMSK=%#010x\n", mask);
	/* No backlight_update_status(): preserve BL brightness at probe. */
	return 0;
}

static int m86_suspend(struct device *dev)
{
	/* No legal reinitialization sequence exists in this handoff-only driver. */
	return -EBUSY;
}
static DEFINE_SIMPLE_DEV_PM_OPS(m86_pm_ops, m86_suspend, NULL);

static const struct of_device_id m86_match[] = {
	{ .compatible = "meizu,m86-dsim-handoff-backlight" }, { }
};
MODULE_DEVICE_TABLE(of, m86_match);
static struct platform_driver m86_driver = {
	.probe = m86_probe,
	.driver = {
		.name = "m86-handoff-backlight",
		.of_match_table = m86_match,
		.pm = pm_sleep_ptr(&m86_pm_ops),
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(m86_driver);
MODULE_DESCRIPTION("Meizu PRO5 bootloader DSIM handoff backlight");
MODULE_LICENSE("GPL");
