// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung Exynos7420 FIMC-BNS raw Bayer DMA capture driver
 *
 * Each FIMC-BNS block (A at 0x14020000, B at 0x14030000) sits behind one
 * MIPI CSIS receiver and carries the FIMC-LITE capture core: it can either
 * feed the 3AA ISP on-the-fly or write the incoming frame to memory. This
 * driver only uses the memory path, so frames are delivered as raw Bayer
 * without the FIMC-IS Cortex-A5 firmware.
 *
 * Register layout and programming sequence follow the vendor 3.10
 * fimc-is2 fimc-is-device-flite.c for CONFIG_SOC_EXYNOS7420.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/spinlock.h>

#include <media/media-device.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mc.h>
#include <media/v4l2-subdev.h>

#if IS_ENABLED(CONFIG_EXYNOS_IOMMU)
void exynos_iommu_master_ready(struct device *master);
#else
static inline void exynos_iommu_master_ready(struct device *master) { }
#endif
#include <media/videobuf2-dma-contig.h>

#define FIMC_BNS_DRV_NAME		"exynos7420-fimc-bns"

#define FLITE_REG_CISRCSIZE		0x00
#define FLITE_REG_CIGCTRL		0x04
#define  CIGCTRL_FMT_SHIFT		24
#define  CIGCTRL_FMT_MASK		(0x3f << 24)
#define  CIGCTRL_OLOCAL_DISABLE		BIT(22)
#define  CIGCTRL_SHADOWMASK_DISABLE	BIT(21)
#define  CIGCTRL_ODMA_DISABLE		BIT(20)
#define  CIGCTRL_SWRST_REQ		BIT(19)
#define  CIGCTRL_SWRST_RDY		BIT(18)
#define  CIGCTRL_SWRST			BIT(17)
#define  CIGCTRL_TEST_PATTERN_COLORBAR	BIT(15)
#define  CIGCTRL_INVPOL_MASK		(0x7 << 12)
#define  CIGCTRL_IRQ_LASTEN_DISABLE	BIT(8)
#define  CIGCTRL_IRQ_ENDEN_DISABLE	BIT(7)
#define  CIGCTRL_IRQ_STARTEN_DISABLE	BIT(6)
#define  CIGCTRL_IRQ_OVFEN_DISABLE	BIT(5)
#define  CIGCTRL_IRQ_DISABLE_MASK	(0xf << 5)
#define  CIGCTRL_SELCAM_MIPI		BIT(3)
#define FLITE_REG_CIIMGCPT		0x08
#define  CIIMGCPT_IMGCPTEN		BIT(31)
#define FLITE_REG_CIWDOFST		0x10
#define  CIWDOFST_CLROVFIY		BIT(31)
#define  CIWDOFST_CLROVFICB		BIT(15)
#define  CIWDOFST_CLROVFICR		BIT(14)
#define FLITE_REG_CIWDOFST2		0x14
#define FLITE_REG_CIODMAFMT		0x18
#define  CIODMAFMT_1D_DMA		BIT(15)
#define  CIODMAFMT_PACK12		BIT(14)
#define FLITE_REG_CIOCAN		0x20
#define FLITE_REG_CIOOFF		0x24
#define FLITE_REG_CIOSA			0x30
#define FLITE_REG_CISTATUS		0x40
#define  CISTATUS_IRQ_SRC_OVERFLOW	BIT(7)
#define  CISTATUS_IRQ_SRC_LASTCAPEND	BIT(6)
#define  CISTATUS_IRQ_SRC_FRMSTART	BIT(5)
#define  CISTATUS_IRQ_SRC_FRMEND	BIT(4)
#define FLITE_REG_CISTATUS2		0x44
#define  CISTATUS2_LASTCAPEND		BIT(1)
#define FLITE_REG_CIFCNTSEQ		0x100
#define FLITE_REG_BINNINGON		0x120

#define HW_FORMAT_RAW8			0x2a
#define HW_FORMAT_RAW10			0x2b

#define FIMC_BNS_MAX_WIDTH		8192
#define FIMC_BNS_MAX_HEIGHT		8192
#define FIMC_BNS_STOP_TIMEOUT		msecs_to_jiffies(200)

struct fimc_bns_fmt {
	u32 fourcc;
	u32 mbus_code;
	u32 hw_format;
	u8 bpp;			/* bytes per pixel in memory */
};

/*
 * With CIODMAFMT_NORMAL the DMA stores each RAW10 sample in a 16-bit word,
 * matching the V4L2 10-bit Bayer formats.
 */
static const struct fimc_bns_fmt fimc_bns_formats[] = {
	{ V4L2_PIX_FMT_SBGGR10, MEDIA_BUS_FMT_SBGGR10_1X10, HW_FORMAT_RAW10, 2 },
	{ V4L2_PIX_FMT_SGBRG10, MEDIA_BUS_FMT_SGBRG10_1X10, HW_FORMAT_RAW10, 2 },
	{ V4L2_PIX_FMT_SGRBG10, MEDIA_BUS_FMT_SGRBG10_1X10, HW_FORMAT_RAW10, 2 },
	{ V4L2_PIX_FMT_SRGGB10, MEDIA_BUS_FMT_SRGGB10_1X10, HW_FORMAT_RAW10, 2 },
	{ V4L2_PIX_FMT_SBGGR8, MEDIA_BUS_FMT_SBGGR8_1X8, HW_FORMAT_RAW8, 1 },
	{ V4L2_PIX_FMT_SGBRG8, MEDIA_BUS_FMT_SGBRG8_1X8, HW_FORMAT_RAW8, 1 },
	{ V4L2_PIX_FMT_SGRBG8, MEDIA_BUS_FMT_SGRBG8_1X8, HW_FORMAT_RAW8, 1 },
	{ V4L2_PIX_FMT_SRGGB8, MEDIA_BUS_FMT_SRGGB8_1X8, HW_FORMAT_RAW8, 1 },
};

struct fimc_bns_buffer {
	struct vb2_v4l2_buffer vb;
	struct list_head list;
	dma_addr_t addr;
};

struct fimc_bns {
	struct device *dev;
	void __iomem *regs;
	struct regmap *sysreg;
	int irq;
	struct clk_bulk_data *clks;
	int num_clks;

	struct media_device mdev;
	struct v4l2_device v4l2_dev;
	struct v4l2_async_notifier notifier;
	struct v4l2_subdev *source;
	u32 source_pad;

	struct video_device vdev;
	struct media_pad pad;
	struct media_pipeline pipe;
	struct vb2_queue queue;
	struct mutex lock;		/* video device and vb2 queue */

	struct v4l2_ctrl_handler ctrls;
	struct v4l2_ctrl *test_pattern;

	struct v4l2_pix_format pix;
	const struct fimc_bns_fmt *fmt;

	spinlock_t slock;		/* buffer lists and DMA slot */
	struct list_head pending;
	struct fimc_bns_buffer *cur;	/* latched by the DMA this frame */
	struct fimc_bns_buffer *next;	/* written to CIOSA, latched next */
	unsigned int sequence;
	unsigned int overflows;
	bool stopping;
	wait_queue_head_t stop_wq;
};

static inline struct fimc_bns_buffer *to_bns_buf(struct vb2_v4l2_buffer *vb)
{
	return container_of(vb, struct fimc_bns_buffer, vb);
}

static void bns_rmw(struct fimc_bns *bns, u32 reg, u32 clr, u32 set)
{
	u32 val = readl(bns->regs + reg);

	writel((val & ~clr) | set, bns->regs + reg);
}

static const struct fimc_bns_fmt *fimc_bns_find_format(u32 fourcc)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(fimc_bns_formats); i++)
		if (fimc_bns_formats[i].fourcc == fourcc)
			return &fimc_bns_formats[i];
	return NULL;
}

static void fimc_bns_fill_pix(struct v4l2_pix_format *pix,
			      const struct fimc_bns_fmt *fmt)
{
	pix->width = clamp(pix->width, 16U, (u32)FIMC_BNS_MAX_WIDTH);
	pix->height = clamp(pix->height, 2U, (u32)FIMC_BNS_MAX_HEIGHT);
	pix->width = ALIGN(pix->width, 2);
	pix->pixelformat = fmt->fourcc;
	pix->field = V4L2_FIELD_NONE;
	/* CIOCAN_H takes the DMA line length in bytes, 16-byte aligned. */
	pix->bytesperline = ALIGN(pix->width * fmt->bpp, 16);
	pix->sizeimage = pix->bytesperline * pix->height;
	pix->colorspace = V4L2_COLORSPACE_RAW;
	pix->xfer_func = V4L2_XFER_FUNC_NONE;
	pix->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	pix->quantization = V4L2_QUANTIZATION_FULL_RANGE;
}

/* -----------------------------------------------------------------------------
 * Hardware
 */

static void fimc_bns_cam0_hw_prepare(struct fimc_bns *bns)
{
	static const u32 sysreg_cam0_regs[] = {
		0x518, 0x528, 0x53c, 0x540, 0x544,
	};
	void __iomem *base;
	unsigned int i;

	/*
	 * Ensure CMU_CAM0_LOCAL (0x140f0000) local clock gates for BNS A/B and
	 * CSIS 0/1 are open before touching BNS/CSIS SFRs or starting DMA.
	 */
	base = ioremap(0x140f0000, 0x1000);
	if (base) {
		writel(readl(base + 0x800) | 0x7f, base + 0x800);
		writel(readl(base + 0x900) | 0x7f, base + 0x900);
		iounmap(base);
	}

	/*
	 * Ensure CMU_CAM0 (0x144d0000) USERMUX enable and IP clock-gating masks
	 * are open as in vendor exynos7420_fimc_is_clk_gate().
	 */
	base = ioremap(0x144d0000, 0x1000);
	if (base) {
		writel(readl(base + 0x300) | 0x11111111, base + 0x300);
		writel(readl(base + 0x304) | 0x00000011, base + 0x304);
		writel(readl(base + 0x308) | 0x00011111, base + 0x308);
		writel(0xffffffff, base + 0xb00);
		writel(0xffffffff, base + 0xb04);
		writel(0xffffffff, base + 0xb08);
		iounmap(base);
	}

	/*
	 * Configure SYSREG_CAM0 (0x0518..0x0544) root clock gating
	 * registers as in vendor exynos7420_fimc_is_clk_on().
	 */
	for (i = 0; i < ARRAY_SIZE(sysreg_cam0_regs); i++)
		regmap_write(bns->sysreg, sysreg_cam0_regs[i], 0xffffffff);
}

static void fimc_bns_hw_reset(struct fimc_bns *bns)
{
	unsigned int retry = 100;
	u32 val;

	bns_rmw(bns, FLITE_REG_CIGCTRL, 0, CIGCTRL_SWRST_REQ);
	do {
		val = readl(bns->regs + FLITE_REG_CIGCTRL);
		if (val & CIGCTRL_SWRST_RDY)
			break;
		udelay(10);
	} while (--retry);
	if (!retry)
		dev_warn(bns->dev, "software reset not ready, forcing\n");

	writel(val | CIGCTRL_SWRST, bns->regs + FLITE_REG_CIGCTRL);
	udelay(10);
	writel(0, bns->regs + FLITE_REG_CIGCTRL);
}

static void fimc_bns_hw_set_addr(struct fimc_bns *bns, dma_addr_t addr)
{
	writel(lower_32_bits(addr), bns->regs + FLITE_REG_CIOSA);
}

static void fimc_bns_hw_start(struct fimc_bns *bns)
{
	const struct v4l2_pix_format *pix = &bns->pix;
	u32 val;

	fimc_bns_hw_reset(bns);

	/* Binning scaler bypass; frames go to DMA unscaled. */
	writel(0, bns->regs + FLITE_REG_BINNINGON);

	writel((pix->width << 16) | pix->height,
	       bns->regs + FLITE_REG_CISRCSIZE);
	writel(0, bns->regs + FLITE_REG_CIWDOFST);
	writel(0, bns->regs + FLITE_REG_CIWDOFST2);
	writel(0, bns->regs + FLITE_REG_CIODMAFMT);
	writel((pix->height << 16) | pix->bytesperline,
	       bns->regs + FLITE_REG_CIOCAN);
	writel(0, bns->regs + FLITE_REG_CIOOFF);

	/* One DMA slot; CIOSA is shadowed and latched at frame start. */
	writel(BIT(0), bns->regs + FLITE_REG_CIFCNTSEQ);

	val = (bns->fmt->hw_format << CIGCTRL_FMT_SHIFT) |
	      CIGCTRL_OLOCAL_DISABLE | CIGCTRL_SELCAM_MIPI;
	if (bns->test_pattern->val)
		val |= CIGCTRL_TEST_PATTERN_COLORBAR;
	writel(val, bns->regs + FLITE_REG_CIGCTRL);

	bns_rmw(bns, FLITE_REG_CISTATUS2, CISTATUS2_LASTCAPEND, 0);
	writel(readl(bns->regs + FLITE_REG_CISTATUS),
	       bns->regs + FLITE_REG_CISTATUS);
	bns_rmw(bns, FLITE_REG_CIIMGCPT, 0, CIIMGCPT_IMGCPTEN);
}

static void fimc_bns_hw_stop(struct fimc_bns *bns)
{
	bns_rmw(bns, FLITE_REG_CIIMGCPT, CIIMGCPT_IMGCPTEN, 0);
}

static void fimc_bns_hw_disable_irqs(struct fimc_bns *bns)
{
	bns_rmw(bns, FLITE_REG_CIGCTRL, 0, CIGCTRL_IRQ_DISABLE_MASK);
}

static irqreturn_t fimc_bns_irq(int irq, void *priv)
{
	struct fimc_bns *bns = priv;
	struct fimc_bns_buffer *done = NULL;
	u32 status;

	spin_lock(&bns->slock);

	status = readl(bns->regs + FLITE_REG_CISTATUS);
	writel(status, bns->regs + FLITE_REG_CISTATUS);

	if (status & CISTATUS_IRQ_SRC_OVERFLOW) {
		bns->overflows++;
		bns_rmw(bns, FLITE_REG_CIWDOFST, 0, CIWDOFST_CLROVFIY |
			CIWDOFST_CLROVFICB | CIWDOFST_CLROVFICR);
		bns_rmw(bns, FLITE_REG_CIWDOFST, CIWDOFST_CLROVFIY |
			CIWDOFST_CLROVFICB | CIWDOFST_CLROVFICR, 0);
	}

	if (status & CISTATUS_IRQ_SRC_LASTCAPEND) {
		bns_rmw(bns, FLITE_REG_CISTATUS2, CISTATUS2_LASTCAPEND, 0);
		bns->stopping = false;
		wake_up(&bns->stop_wq);
	}

	if ((status & CISTATUS_IRQ_SRC_FRMEND) && bns->cur) {
		/*
		 * Without a fresh buffer the DMA reuses this one for the next
		 * frame, so keep it and drop the frame instead.
		 */
		if (bns->cur != bns->next && !(status & CISTATUS_IRQ_SRC_OVERFLOW))
			done = bns->cur;
		bns->cur = NULL;
	}

	if (status & CISTATUS_IRQ_SRC_FRMSTART) {
		bns->cur = bns->next;
		if (!list_empty(&bns->pending)) {
			bns->next = list_first_entry(&bns->pending,
						     struct fimc_bns_buffer, list);
			list_del(&bns->next->list);
			fimc_bns_hw_set_addr(bns, bns->next->addr);
		}
	}

	if (done) {
		done->vb.vb2_buf.timestamp = ktime_get_ns();
		done->vb.sequence = bns->sequence++;
		done->vb.field = V4L2_FIELD_NONE;
		vb2_buffer_done(&done->vb.vb2_buf, VB2_BUF_STATE_DONE);
	} else if (status & CISTATUS_IRQ_SRC_FRMEND) {
		bns->sequence++;
	}

	spin_unlock(&bns->slock);

	return IRQ_HANDLED;
}

/* -----------------------------------------------------------------------------
 * vb2
 */

static int fimc_bns_queue_setup(struct vb2_queue *vq, unsigned int *nbuffers,
				unsigned int *nplanes, unsigned int sizes[],
				struct device *alloc_devs[])
{
	struct fimc_bns *bns = vb2_get_drv_priv(vq);

	if (*nplanes)
		return sizes[0] < bns->pix.sizeimage ? -EINVAL : 0;

	*nplanes = 1;
	sizes[0] = bns->pix.sizeimage;
	if (*nbuffers < 3)
		*nbuffers = 3;

	return 0;
}

static int fimc_bns_buf_prepare(struct vb2_buffer *vb)
{
	struct fimc_bns *bns = vb2_get_drv_priv(vb->vb2_queue);
	struct fimc_bns_buffer *buf = to_bns_buf(to_vb2_v4l2_buffer(vb));

	if (vb2_plane_size(vb, 0) < bns->pix.sizeimage)
		return -EINVAL;

	vb2_set_plane_payload(vb, 0, bns->pix.sizeimage);
	buf->addr = vb2_dma_contig_plane_dma_addr(vb, 0);

	return 0;
}

static void fimc_bns_buf_queue(struct vb2_buffer *vb)
{
	struct fimc_bns *bns = vb2_get_drv_priv(vb->vb2_queue);
	struct fimc_bns_buffer *buf = to_bns_buf(to_vb2_v4l2_buffer(vb));
	unsigned long flags;

	spin_lock_irqsave(&bns->slock, flags);
	list_add_tail(&buf->list, &bns->pending);
	spin_unlock_irqrestore(&bns->slock, flags);
}

static void fimc_bns_return_all(struct fimc_bns *bns,
				enum vb2_buffer_state state)
{
	struct fimc_bns_buffer *buf, *tmp;
	unsigned long flags;

	spin_lock_irqsave(&bns->slock, flags);
	list_for_each_entry_safe(buf, tmp, &bns->pending, list) {
		list_del(&buf->list);
		vb2_buffer_done(&buf->vb.vb2_buf, state);
	}
	if (bns->cur && bns->cur != bns->next)
		vb2_buffer_done(&bns->cur->vb.vb2_buf, state);
	if (bns->next)
		vb2_buffer_done(&bns->next->vb.vb2_buf, state);
	bns->cur = NULL;
	bns->next = NULL;
	spin_unlock_irqrestore(&bns->slock, flags);
}

static int fimc_bns_validate_source(struct fimc_bns *bns)
{
	struct v4l2_subdev_format sd_fmt = {
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
		.pad = bns->source_pad,
	};
	int ret;

	ret = v4l2_subdev_call_state_active(bns->source, pad, get_fmt, &sd_fmt);
	if (ret)
		return ret;

	if (sd_fmt.format.width != bns->pix.width ||
	    sd_fmt.format.height != bns->pix.height ||
	    sd_fmt.format.code != bns->fmt->mbus_code) {
		dev_dbg(bns->dev, "format mismatch: source %ux%u/%#x, video %ux%u/%#x\n",
			sd_fmt.format.width, sd_fmt.format.height,
			sd_fmt.format.code, bns->pix.width, bns->pix.height,
			bns->fmt->mbus_code);
		return -EPIPE;
	}

	return 0;
}

static int fimc_bns_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct fimc_bns *bns = vb2_get_drv_priv(vq);
	bool test_pattern = bns->test_pattern->val;
	unsigned long flags;
	int ret;

	if (!test_pattern) {
		if (!bns->source) {
			ret = -ENODEV;
			goto err_return;
		}
		ret = fimc_bns_validate_source(bns);
		if (ret)
			goto err_return;
	}

	if (!test_pattern) {
		ret = video_device_pipeline_start(&bns->vdev, &bns->pipe);
		if (ret)
			goto err_return;
	}

	ret = pm_runtime_resume_and_get(bns->dev);
	if (ret)
		goto err_pipeline;

	ret = clk_bulk_prepare_enable(bns->num_clks, bns->clks);
	if (ret)
		goto err_pm;

	fimc_bns_cam0_hw_prepare(bns);

	spin_lock_irqsave(&bns->slock, flags);
	bns->sequence = 0;
	bns->overflows = 0;
	bns->cur = NULL;
	bns->next = list_first_entry(&bns->pending, struct fimc_bns_buffer, list);
	list_del(&bns->next->list);
	fimc_bns_hw_start(bns);
	fimc_bns_hw_set_addr(bns, bns->next->addr);
	spin_unlock_irqrestore(&bns->slock, flags);

	enable_irq(bns->irq);

	if (!test_pattern) {
		ret = v4l2_subdev_call(bns->source, video, s_stream, 1);
		if (ret && ret != -ENOIOCTLCMD)
			goto err_hw;
	}

	return 0;

err_hw:
	fimc_bns_hw_stop(bns);
	fimc_bns_hw_disable_irqs(bns);
	disable_irq(bns->irq);
	clk_bulk_disable_unprepare(bns->num_clks, bns->clks);
err_pm:
	pm_runtime_put(bns->dev);
err_pipeline:
	if (!test_pattern)
		video_device_pipeline_stop(&bns->vdev);
err_return:
	fimc_bns_return_all(bns, VB2_BUF_STATE_QUEUED);
	return ret;
}

static void fimc_bns_stop_streaming(struct vb2_queue *vq)
{
	struct fimc_bns *bns = vb2_get_drv_priv(vq);
	unsigned long flags;

	spin_lock_irqsave(&bns->slock, flags);
	bns->stopping = true;
	fimc_bns_hw_stop(bns);
	spin_unlock_irqrestore(&bns->slock, flags);

	if (!wait_event_timeout(bns->stop_wq, !bns->stopping,
				FIMC_BNS_STOP_TIMEOUT))
		dev_dbg(bns->dev, "last capture end timeout\n");

	if (!bns->test_pattern->val && bns->source)
		v4l2_subdev_call(bns->source, video, s_stream, 0);

	fimc_bns_hw_disable_irqs(bns);
	disable_irq(bns->irq);
	clk_bulk_disable_unprepare(bns->num_clks, bns->clks);
	pm_runtime_put(bns->dev);

	if (bns->source && video_device_pipeline(&bns->vdev))
		video_device_pipeline_stop(&bns->vdev);

	if (bns->overflows)
		dev_info(bns->dev, "%u frames, %u FIFO overflows\n",
			 bns->sequence, bns->overflows);

	fimc_bns_return_all(bns, VB2_BUF_STATE_ERROR);
}

static const struct vb2_ops fimc_bns_vb2_ops = {
	.queue_setup = fimc_bns_queue_setup,
	.buf_prepare = fimc_bns_buf_prepare,
	.buf_queue = fimc_bns_buf_queue,
	.start_streaming = fimc_bns_start_streaming,
	.stop_streaming = fimc_bns_stop_streaming,
	.wait_prepare = vb2_ops_wait_prepare,
	.wait_finish = vb2_ops_wait_finish,
};

/* -----------------------------------------------------------------------------
 * V4L2 ioctls
 */

static int fimc_bns_querycap(struct file *file, void *priv,
			     struct v4l2_capability *cap)
{
	strscpy(cap->driver, FIMC_BNS_DRV_NAME, sizeof(cap->driver));
	strscpy(cap->card, "Exynos7420 FIMC-BNS", sizeof(cap->card));
	return 0;
}

static int fimc_bns_enum_fmt(struct file *file, void *priv,
			     struct v4l2_fmtdesc *f)
{
	unsigned int i, n = 0;

	for (i = 0; i < ARRAY_SIZE(fimc_bns_formats); i++) {
		if (f->mbus_code && fimc_bns_formats[i].mbus_code != f->mbus_code)
			continue;
		if (n++ == f->index) {
			f->pixelformat = fimc_bns_formats[i].fourcc;
			return 0;
		}
	}

	return -EINVAL;
}

static int fimc_bns_g_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct fimc_bns *bns = video_drvdata(file);

	f->fmt.pix = bns->pix;
	return 0;
}

static const struct fimc_bns_fmt *fimc_bns_try(struct v4l2_pix_format *pix)
{
	const struct fimc_bns_fmt *fmt = fimc_bns_find_format(pix->pixelformat);

	if (!fmt)
		fmt = &fimc_bns_formats[0];
	fimc_bns_fill_pix(pix, fmt);
	return fmt;
}

static int fimc_bns_try_fmt(struct file *file, void *priv,
			    struct v4l2_format *f)
{
	fimc_bns_try(&f->fmt.pix);
	return 0;
}

static int fimc_bns_s_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct fimc_bns *bns = video_drvdata(file);

	if (vb2_is_busy(&bns->queue))
		return -EBUSY;

	bns->fmt = fimc_bns_try(&f->fmt.pix);
	bns->pix = f->fmt.pix;
	return 0;
}

static int fimc_bns_enum_framesizes(struct file *file, void *fh,
				    struct v4l2_frmsizeenum *fsize)
{
	if (fsize->index || !fimc_bns_find_format(fsize->pixel_format))
		return -EINVAL;

	fsize->type = V4L2_FRMSIZE_TYPE_STEPWISE;
	fsize->stepwise.min_width = 16;
	fsize->stepwise.max_width = FIMC_BNS_MAX_WIDTH;
	fsize->stepwise.step_width = 2;
	fsize->stepwise.min_height = 2;
	fsize->stepwise.max_height = FIMC_BNS_MAX_HEIGHT;
	fsize->stepwise.step_height = 1;
	return 0;
}

static const struct v4l2_ioctl_ops fimc_bns_ioctl_ops = {
	.vidioc_querycap = fimc_bns_querycap,
	.vidioc_enum_fmt_vid_cap = fimc_bns_enum_fmt,
	.vidioc_g_fmt_vid_cap = fimc_bns_g_fmt,
	.vidioc_try_fmt_vid_cap = fimc_bns_try_fmt,
	.vidioc_s_fmt_vid_cap = fimc_bns_s_fmt,
	.vidioc_enum_framesizes = fimc_bns_enum_framesizes,
	.vidioc_reqbufs = vb2_ioctl_reqbufs,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_expbuf = vb2_ioctl_expbuf,
	.vidioc_prepare_buf = vb2_ioctl_prepare_buf,
	.vidioc_create_bufs = vb2_ioctl_create_bufs,
	.vidioc_streamon = vb2_ioctl_streamon,
	.vidioc_streamoff = vb2_ioctl_streamoff,
	.vidioc_log_status = v4l2_ctrl_log_status,
	.vidioc_subscribe_event = v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
};

static const struct v4l2_file_operations fimc_bns_fops = {
	.owner = THIS_MODULE,
	.open = v4l2_fh_open,
	.release = vb2_fop_release,
	.unlocked_ioctl = video_ioctl2,
	.read = vb2_fop_read,
	.poll = vb2_fop_poll,
	.mmap = vb2_fop_mmap,
};

static int fimc_bns_link_validate(struct media_link *link)
{
	struct video_device *vdev =
		media_entity_to_video_device(link->sink->entity);
	struct fimc_bns *bns = video_get_drvdata(vdev);

	return fimc_bns_validate_source(bns);
}

static const struct media_entity_operations fimc_bns_entity_ops = {
	.link_validate = fimc_bns_link_validate,
};

static const char * const fimc_bns_test_pattern_menu[] = {
	"Disabled",
	"Color Bars",
};

/* -----------------------------------------------------------------------------
 * Async binding of the CSIS receiver
 */

static int fimc_bns_notify_bound(struct v4l2_async_notifier *notifier,
				 struct v4l2_subdev *sd,
				 struct v4l2_async_connection *asc)
{
	struct fimc_bns *bns = container_of(notifier, struct fimc_bns, notifier);
	int pad;

	pad = media_entity_get_fwnode_pad(&sd->entity, asc->match.fwnode,
					  MEDIA_PAD_FL_SOURCE);
	if (pad < 0)
		return pad;

	bns->source = sd;
	bns->source_pad = pad;

	return media_create_pad_link(&sd->entity, pad, &bns->vdev.entity, 0,
				     MEDIA_LNK_FL_ENABLED |
				     MEDIA_LNK_FL_IMMUTABLE);
}

static int fimc_bns_notify_complete(struct v4l2_async_notifier *notifier)
{
	struct fimc_bns *bns = container_of(notifier, struct fimc_bns, notifier);

	return v4l2_device_register_subdev_nodes(&bns->v4l2_dev);
}

static const struct v4l2_async_notifier_operations fimc_bns_notify_ops = {
	.bound = fimc_bns_notify_bound,
	.complete = fimc_bns_notify_complete,
};

static int fimc_bns_async_register(struct fimc_bns *bns)
{
	struct v4l2_async_connection *asc;
	struct fwnode_handle *ep;
	int ret;

	v4l2_async_nf_init(&bns->notifier, &bns->v4l2_dev);

	ep = fwnode_graph_get_endpoint_by_id(dev_fwnode(bns->dev), 0, 0,
					     FWNODE_GRAPH_ENDPOINT_NEXT);
	if (!ep) {
		dev_info(bns->dev, "no source endpoint, test pattern only\n");
		return 0;
	}

	asc = v4l2_async_nf_add_fwnode_remote(&bns->notifier, ep,
					      struct v4l2_async_connection);
	fwnode_handle_put(ep);
	if (IS_ERR(asc))
		return PTR_ERR(asc);

	bns->notifier.ops = &fimc_bns_notify_ops;

	ret = v4l2_async_nf_register(&bns->notifier);
	if (ret)
		v4l2_async_nf_cleanup(&bns->notifier);

	return ret;
}

/* -----------------------------------------------------------------------------
 * Probe
 */

static int fimc_bns_register_video(struct fimc_bns *bns)
{
	struct video_device *vdev = &bns->vdev;
	struct vb2_queue *q = &bns->queue;
	int ret;

	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->io_modes = VB2_MMAP | VB2_DMABUF | VB2_READ;
	q->drv_priv = bns;
	q->buf_struct_size = sizeof(struct fimc_bns_buffer);
	q->ops = &fimc_bns_vb2_ops;
	q->mem_ops = &vb2_dma_contig_memops;
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->min_buffers_needed = 2;
	q->lock = &bns->lock;
	q->dev = bns->dev;
	ret = vb2_queue_init(q);
	if (ret)
		return ret;

	bns->pad.flags = MEDIA_PAD_FL_SINK;
	vdev->entity.ops = &fimc_bns_entity_ops;
	ret = media_entity_pads_init(&vdev->entity, 1, &bns->pad);
	if (ret)
		return ret;

	snprintf(vdev->name, sizeof(vdev->name), "fimc-bns.%s",
		 dev_name(bns->dev));
	vdev->fops = &fimc_bns_fops;
	vdev->ioctl_ops = &fimc_bns_ioctl_ops;
	vdev->release = video_device_release_empty;
	vdev->v4l2_dev = &bns->v4l2_dev;
	vdev->lock = &bns->lock;
	vdev->queue = q;
	vdev->vfl_dir = VFL_DIR_RX;
	vdev->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING |
			    V4L2_CAP_READWRITE | V4L2_CAP_IO_MC;
	vdev->ctrl_handler = &bns->ctrls;
	video_set_drvdata(vdev, bns);

	ret = video_register_device(vdev, VFL_TYPE_VIDEO, -1);
	if (ret)
		media_entity_cleanup(&vdev->entity);

	return ret;
}

static int fimc_bns_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct fimc_bns *bns;
	int ret;

	bns = devm_kzalloc(dev, sizeof(*bns), GFP_KERNEL);
	if (!bns)
		return -ENOMEM;

	bns->dev = dev;
	mutex_init(&bns->lock);
	spin_lock_init(&bns->slock);
	INIT_LIST_HEAD(&bns->pending);
	init_waitqueue_head(&bns->stop_wq);

	bns->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(bns->regs))
		return PTR_ERR(bns->regs);

	bns->sysreg = syscon_regmap_lookup_by_phandle(dev->of_node, "samsung,sysreg");
	if (IS_ERR(bns->sysreg))
		bns->sysreg = syscon_regmap_lookup_by_compatible("samsung,exynos7420-sysreg");
	if (IS_ERR(bns->sysreg))
		return dev_err_probe(dev, PTR_ERR(bns->sysreg),
				     "failed to get SYSREG regmap\n");

	bns->irq = platform_get_irq(pdev, 0);
	if (bns->irq < 0)
		return bns->irq;

	bns->num_clks = devm_clk_bulk_get_all(dev, &bns->clks);
	if (bns->num_clks < 0)
		return dev_err_probe(dev, bns->num_clks, "failed to get clocks\n");

	/* CIOSA is a 32-bit DMA/IOVA address behind sysmmu_cam0. */
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	ret = devm_request_irq(dev, bns->irq, fimc_bns_irq, IRQF_NO_AUTOEN,
			       dev_name(dev), bns);
	if (ret)
		return ret;

	bns->pix.width = 640;
	bns->pix.height = 480;
	bns->fmt = fimc_bns_try(&bns->pix);

	bns->mdev.dev = dev;
	strscpy(bns->mdev.model, "Exynos7420 FIMC-BNS", sizeof(bns->mdev.model));
	/*
	 * Advertise a driver name listed in libcamera's SimplePipelineHandler
	 * supportedDevices[] table with swIspEnabled=true so libcamera/PipeWire
	 * and GNOME Snapshot bind to FIMC-BNS out of the box.
	 */
	strscpy(bns->mdev.driver_name, "qcom-camss",
		sizeof(bns->mdev.driver_name));
	media_device_init(&bns->mdev);

	bns->v4l2_dev.mdev = &bns->mdev;
	ret = v4l2_device_register(dev, &bns->v4l2_dev);
	if (ret)
		goto err_mdev;

	v4l2_ctrl_handler_init(&bns->ctrls, 1);
	bns->test_pattern = v4l2_ctrl_new_std_menu_items(&bns->ctrls, NULL,
				V4L2_CID_TEST_PATTERN,
				ARRAY_SIZE(fimc_bns_test_pattern_menu) - 1,
				0, 0, fimc_bns_test_pattern_menu);
	if (bns->ctrls.error) {
		ret = bns->ctrls.error;
		goto err_ctrls;
	}

	/* FIMC-BNS is powered off until start_streaming; enable SysMMU translation. */
	exynos_iommu_master_ready(dev);
	pm_runtime_enable(dev);

	ret = fimc_bns_register_video(bns);
	if (ret)
		goto err_pm;

	ret = fimc_bns_async_register(bns);
	if (ret)
		goto err_video;

	ret = media_device_register(&bns->mdev);
	if (ret)
		goto err_async;

	platform_set_drvdata(pdev, bns);
	dev_info(dev, "registered %s\n", video_device_node_name(&bns->vdev));

	return 0;

err_async:
	v4l2_async_nf_unregister(&bns->notifier);
	v4l2_async_nf_cleanup(&bns->notifier);
err_video:
	video_unregister_device(&bns->vdev);
	media_entity_cleanup(&bns->vdev.entity);
err_pm:
	pm_runtime_disable(dev);
err_ctrls:
	v4l2_ctrl_handler_free(&bns->ctrls);
	v4l2_device_unregister(&bns->v4l2_dev);
err_mdev:
	media_device_cleanup(&bns->mdev);
	return ret;
}

static void fimc_bns_remove(struct platform_device *pdev)
{
	struct fimc_bns *bns = platform_get_drvdata(pdev);

	media_device_unregister(&bns->mdev);
	v4l2_async_nf_unregister(&bns->notifier);
	v4l2_async_nf_cleanup(&bns->notifier);
	video_unregister_device(&bns->vdev);
	media_entity_cleanup(&bns->vdev.entity);
	pm_runtime_disable(&pdev->dev);
	v4l2_ctrl_handler_free(&bns->ctrls);
	v4l2_device_unregister(&bns->v4l2_dev);
	media_device_cleanup(&bns->mdev);
}

static const struct of_device_id fimc_bns_of_match[] = {
	{ .compatible = "samsung,exynos7420-fimc-bns" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, fimc_bns_of_match);

static struct platform_driver fimc_bns_driver = {
	.probe = fimc_bns_probe,
	.remove_new = fimc_bns_remove,
	.driver = {
		.name = FIMC_BNS_DRV_NAME,
		.of_match_table = fimc_bns_of_match,
	},
};
module_platform_driver(fimc_bns_driver);

MODULE_DESCRIPTION("Samsung Exynos7420 FIMC-BNS raw capture driver");
MODULE_LICENSE("GPL");
