// SPDX-License-Identifier: GPL-2.0-only
/*
 * Sony IMX230 21 Mpixel CMOS image sensor driver
 *
 * The sensor is SMIA-style: standard integration time, gain, frame and line
 * length registers at 0x0202/0x0204/0x020e/0x0340/0x0342 and the CSI-2 output
 * setup in the 0x0100-0x0900 window. Initialisation and mode tables come from
 * the Meizu PRO 5 FIMC-IS firmware (24 MHz INCK, four lanes, RAW10).
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>

#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#include "imx230-regs.h"

#define IMX230_REG_CHIP_ID		CCI_REG16(0x0016)
#define IMX230_CHIP_ID			0x0230

#define IMX230_REG_MODE_SELECT		CCI_REG8(0x0100)
#define IMX230_MODE_STANDBY		0x00
#define IMX230_MODE_STREAMING		0x01

#define IMX230_REG_ORIENTATION		CCI_REG8(0x0101)
#define IMX230_REG_GROUP_HOLD		CCI_REG8(0x0104)

#define IMX230_REG_EXPOSURE		CCI_REG16(0x0202)
#define IMX230_EXPOSURE_MIN		1
#define IMX230_EXPOSURE_OFFSET		10
#define IMX230_EXPOSURE_STEP		1

/* Analogue gain = 512 / (512 - code) */
#define IMX230_REG_ANALOG_GAIN		CCI_REG16(0x0204)
#define IMX230_ANA_GAIN_MIN		0
#define IMX230_ANA_GAIN_MAX		480
#define IMX230_ANA_GAIN_DEFAULT		0

#define IMX230_REG_DIGITAL_GAIN		CCI_REG16(0x020e)
#define IMX230_DGTL_GAIN_MIN		0x0100
#define IMX230_DGTL_GAIN_MAX		0x0fff
#define IMX230_DGTL_GAIN_DEFAULT	0x0100

#define IMX230_REG_FRAME_LENGTH		CCI_REG16(0x0340)
#define IMX230_FLL_MAX			0xffff

#define IMX230_REG_TEST_PATTERN		CCI_REG16(0x0600)

#define IMX230_NATIVE_WIDTH		5344
#define IMX230_NATIVE_HEIGHT		4016

#define IMX230_XCLK_FREQ		24000000

struct imx230_mode {
	u32 width;
	u32 height;
	u32 fll;		/* default frame length, lines */
	u32 llp;		/* line length, pixel clocks */
	u64 pixel_rate;
	u64 link_freq;
	struct v4l2_rect crop;
	const struct cci_reg_sequence *regs;
	unsigned int num_regs;
};

#define IMX230_MODE(w, h, f, l, pr, lf, x0, y0, x1, y1, r) {	\
	.width = (w), .height = (h), .fll = (f), .llp = (l),		\
	.pixel_rate = (pr), .link_freq = (lf),				\
	.crop = { (x0), (y0), (x1) - (x0) + 1, (y1) - (y0) + 1 },	\
	.regs = (r), .num_regs = ARRAY_SIZE(r),				\
}

static const struct imx230_mode imx230_modes[] = {
	/* 2x2 binned full field of view, 30 fps */
	IMX230_MODE(2672, 2002, 2108, 6024, 381000000, 240000000,
		    0, 8, 5343, 4011, imx230_mode3_regs),
	/* full resolution, 24 fps */
	IMX230_MODE(5344, 4016, 4108, 6024, 594000000, 699428571,
		    0, 0, 5343, 4015, imx230_mode0_regs),
	/* 16:9 crop, 30 fps */
	IMX230_MODE(3856, 2170, 2274, 6024, 411000000, 360000000,
		    0, 924, 5343, 3093, imx230_mode2_regs),
	/* 4x4 binned 16:9, 30 fps */
	IMX230_MODE(1296, 730, 1112, 6024, 201000000, 199714286,
		    0, 552, 5343, 3471, imx230_mode5_regs),
};

static const s64 imx230_link_freqs[] = {
	240000000,
	699428571,
	360000000,
	199714286,
};

static const char * const imx230_test_pattern_menu[] = {
	"Disabled",
	"Solid Colour",
	"Eight Vertical Colour Bars",
	"Colour Bars With Fade to Grey",
	"Pseudorandom Sequence (PN9)",
};

static const char * const imx230_supply_names[] = {
	"vana",		/* 2.5 V analogue */
	"vdig",		/* 1.15 V core */
	"vif",		/* 1.8 V interface */
};

#define IMX230_NUM_SUPPLIES ARRAY_SIZE(imx230_supply_names)

struct imx230 {
	struct device *dev;
	struct regmap *regmap;
	struct v4l2_subdev sd;
	struct media_pad pad;

	struct clk *xclk;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[IMX230_NUM_SUPPLIES];

	struct v4l2_ctrl_handler ctrls;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *exposure;

	const struct imx230_mode *mode;
	bool streaming;
};

static inline struct imx230 *to_imx230(struct v4l2_subdev *sd)
{
	return container_of(sd, struct imx230, sd);
}

/* Bayer order at 0x0101 = 0 (no mirror/flip). */
#define IMX230_MBUS_CODE		MEDIA_BUS_FMT_SRGGB10_1X10

static void imx230_update_pad_format(const struct imx230_mode *mode,
				     struct v4l2_mbus_framefmt *fmt)
{
	fmt->width = mode->width;
	fmt->height = mode->height;
	fmt->code = IMX230_MBUS_CODE;
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_YCBCR_ENC_601;
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_XFER_FUNC_NONE;
}

static int imx230_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx230 *imx230 = container_of(ctrl->handler, struct imx230, ctrls);
	int ret = 0;

	if (ctrl->id == V4L2_CID_VBLANK) {
		s64 max = imx230->mode->height + ctrl->val - IMX230_EXPOSURE_OFFSET;

		__v4l2_ctrl_modify_range(imx230->exposure,
					 imx230->exposure->minimum, max,
					 imx230->exposure->step,
					 min(imx230->exposure->default_value, max));
	}

	if (!pm_runtime_get_if_in_use(imx230->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_ANALOGUE_GAIN:
		cci_write(imx230->regmap, IMX230_REG_ANALOG_GAIN, ctrl->val, &ret);
		break;
	case V4L2_CID_DIGITAL_GAIN:
		cci_write(imx230->regmap, IMX230_REG_DIGITAL_GAIN, ctrl->val, &ret);
		break;
	case V4L2_CID_EXPOSURE:
		cci_write(imx230->regmap, IMX230_REG_EXPOSURE, ctrl->val, &ret);
		break;
	case V4L2_CID_VBLANK:
		cci_write(imx230->regmap, IMX230_REG_FRAME_LENGTH,
			  imx230->mode->height + ctrl->val, &ret);
		break;
	case V4L2_CID_TEST_PATTERN:
		cci_write(imx230->regmap, IMX230_REG_TEST_PATTERN, ctrl->val, &ret);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	pm_runtime_put(imx230->dev);

	return ret;
}

static const struct v4l2_ctrl_ops imx230_ctrl_ops = {
	.s_ctrl = imx230_set_ctrl,
};

static void imx230_update_ctrl_ranges(struct imx230 *imx230)
{
	const struct imx230_mode *mode = imx230->mode;
	s64 vblank_def = mode->fll - mode->height;
	s64 hblank = mode->llp - mode->width;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(imx230_modes); i++)
		if (mode == &imx230_modes[i])
			__v4l2_ctrl_s_ctrl(imx230->link_freq, i);

	__v4l2_ctrl_s_ctrl_int64(imx230->pixel_rate, mode->pixel_rate);
	__v4l2_ctrl_modify_range(imx230->hblank, hblank, hblank, 1, hblank);
	__v4l2_ctrl_modify_range(imx230->vblank, vblank_def,
				 IMX230_FLL_MAX - mode->height, 1, vblank_def);
	__v4l2_ctrl_s_ctrl(imx230->vblank, vblank_def);
}

static int imx230_init_controls(struct imx230 *imx230)
{
	const struct imx230_mode *mode = imx230->mode;
	struct v4l2_ctrl_handler *hdl = &imx230->ctrls;
	struct v4l2_fwnode_device_properties props;
	s64 vblank_def = mode->fll - mode->height;
	s64 hblank = mode->llp - mode->width;
	int ret;

	v4l2_ctrl_handler_init(hdl, 11);

	imx230->link_freq = v4l2_ctrl_new_int_menu(hdl, &imx230_ctrl_ops,
						   V4L2_CID_LINK_FREQ,
						   ARRAY_SIZE(imx230_link_freqs) - 1,
						   0, imx230_link_freqs);
	if (imx230->link_freq)
		imx230->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	imx230->pixel_rate = v4l2_ctrl_new_std(hdl, &imx230_ctrl_ops,
					       V4L2_CID_PIXEL_RATE, 1,
					       INT_MAX, 1, mode->pixel_rate);

	imx230->hblank = v4l2_ctrl_new_std(hdl, &imx230_ctrl_ops,
					   V4L2_CID_HBLANK, hblank, hblank, 1,
					   hblank);
	if (imx230->hblank)
		imx230->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	imx230->vblank = v4l2_ctrl_new_std(hdl, &imx230_ctrl_ops,
					   V4L2_CID_VBLANK, vblank_def,
					   IMX230_FLL_MAX - mode->height, 1,
					   vblank_def);

	imx230->exposure = v4l2_ctrl_new_std(hdl, &imx230_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     IMX230_EXPOSURE_MIN,
					     mode->fll - IMX230_EXPOSURE_OFFSET,
					     IMX230_EXPOSURE_STEP,
					     mode->fll - IMX230_EXPOSURE_OFFSET);

	v4l2_ctrl_new_std(hdl, &imx230_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  IMX230_ANA_GAIN_MIN, IMX230_ANA_GAIN_MAX, 1,
			  IMX230_ANA_GAIN_DEFAULT);
	v4l2_ctrl_new_std(hdl, &imx230_ctrl_ops, V4L2_CID_DIGITAL_GAIN,
			  IMX230_DGTL_GAIN_MIN, IMX230_DGTL_GAIN_MAX, 1,
			  IMX230_DGTL_GAIN_DEFAULT);
	v4l2_ctrl_new_std_menu_items(hdl, &imx230_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(imx230_test_pattern_menu) - 1,
				     0, 0, imx230_test_pattern_menu);

	ret = v4l2_fwnode_device_parse(imx230->dev, &props);
	if (!ret)
		v4l2_ctrl_new_fwnode_properties(hdl, &imx230_ctrl_ops, &props);

	if (hdl->error) {
		ret = hdl->error;
		v4l2_ctrl_handler_free(hdl);
		return ret;
	}

	imx230->sd.ctrl_handler = hdl;
	return 0;
}

static int imx230_start_streaming(struct imx230 *imx230)
{
	const struct imx230_mode *mode = imx230->mode;
	int ret;

	ret = pm_runtime_resume_and_get(imx230->dev);
	if (ret)
		return ret;

	ret = cci_multi_reg_write(imx230->regmap, imx230_init_regs,
				  ARRAY_SIZE(imx230_init_regs), NULL);
	if (ret)
		goto err_pm;

	ret = cci_multi_reg_write(imx230->regmap, mode->regs, mode->num_regs,
				  NULL);
	if (ret)
		goto err_pm;

	ret = __v4l2_ctrl_handler_setup(imx230->sd.ctrl_handler);
	if (ret)
		goto err_pm;

	ret = cci_write(imx230->regmap, IMX230_REG_MODE_SELECT,
			IMX230_MODE_STREAMING, NULL);
	if (ret)
		goto err_pm;

	return 0;

err_pm:
	pm_runtime_put(imx230->dev);
	return ret;
}

static void imx230_stop_streaming(struct imx230 *imx230)
{
	cci_write(imx230->regmap, IMX230_REG_MODE_SELECT, IMX230_MODE_STANDBY,
		  NULL);
	pm_runtime_put(imx230->dev);
}

static int imx230_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct imx230 *imx230 = to_imx230(sd);
	struct v4l2_subdev_state *state;
	int ret = 0;

	state = v4l2_subdev_lock_and_get_active_state(sd);

	if (imx230->streaming == !!enable)
		goto out;

	if (enable) {
		ret = imx230_start_streaming(imx230);
		if (ret)
			goto out;
	} else {
		imx230_stop_streaming(imx230);
	}

	imx230->streaming = enable;

out:
	v4l2_subdev_unlock_state(state);
	return ret;
}

static int imx230_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index)
		return -EINVAL;

	code->code = IMX230_MBUS_CODE;
	return 0;
}

static int imx230_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index >= ARRAY_SIZE(imx230_modes) ||
	    fse->code != IMX230_MBUS_CODE)
		return -EINVAL;

	fse->min_width = imx230_modes[fse->index].width;
	fse->max_width = fse->min_width;
	fse->min_height = imx230_modes[fse->index].height;
	fse->max_height = fse->min_height;
	return 0;
}

static int imx230_set_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_format *fmt)
{
	struct imx230 *imx230 = to_imx230(sd);
	const struct imx230_mode *mode;
	struct v4l2_mbus_framefmt *format;
	struct v4l2_rect *crop;

	mode = v4l2_find_nearest_size(imx230_modes, ARRAY_SIZE(imx230_modes),
				      width, height, fmt->format.width,
				      fmt->format.height);

	imx230_update_pad_format(mode, &fmt->format);

	format = v4l2_subdev_get_pad_format(sd, state, 0);
	*format = fmt->format;
	crop = v4l2_subdev_get_pad_crop(sd, state, 0);
	*crop = mode->crop;

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		if (imx230->streaming)
			return -EBUSY;
		imx230->mode = mode;
		imx230_update_ctrl_ranges(imx230);
	}

	return 0;
}

static int imx230_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *state,
				struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
		sel->r = *v4l2_subdev_get_pad_crop(sd, state, 0);
		return 0;
	case V4L2_SEL_TGT_NATIVE_SIZE:
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.top = 0;
		sel->r.left = 0;
		sel->r.width = IMX230_NATIVE_WIDTH;
		sel->r.height = IMX230_NATIVE_HEIGHT;
		return 0;
	}

	return -EINVAL;
}

static int imx230_init_cfg(struct v4l2_subdev *sd,
			   struct v4l2_subdev_state *state)
{
	struct v4l2_subdev_format fmt = {
		.which = V4L2_SUBDEV_FORMAT_TRY,
		.format = {
			.width = imx230_modes[0].width,
			.height = imx230_modes[0].height,
		},
	};

	return imx230_set_pad_format(sd, state, &fmt);
}

static const struct v4l2_subdev_video_ops imx230_video_ops = {
	.s_stream = imx230_s_stream,
};

static const struct v4l2_subdev_pad_ops imx230_pad_ops = {
	.init_cfg = imx230_init_cfg,
	.enum_mbus_code = imx230_enum_mbus_code,
	.enum_frame_size = imx230_enum_frame_size,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = imx230_set_pad_format,
	.get_selection = imx230_get_selection,
};

static const struct v4l2_subdev_ops imx230_subdev_ops = {
	.video = &imx230_video_ops,
	.pad = &imx230_pad_ops,
};

static const struct media_entity_operations imx230_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static int imx230_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct imx230 *imx230 = to_imx230(sd);
	int ret;

	ret = regulator_bulk_enable(IMX230_NUM_SUPPLIES, imx230->supplies);
	if (ret)
		return ret;

	ret = clk_prepare_enable(imx230->xclk);
	if (ret) {
		regulator_bulk_disable(IMX230_NUM_SUPPLIES, imx230->supplies);
		return ret;
	}

	/* Vendor sequence: XCLR released 3 ms after the rails and INCK. */
	usleep_range(1000, 1500);
	gpiod_set_value_cansleep(imx230->reset_gpio, 0);
	usleep_range(3000, 4000);

	return 0;
}

static int imx230_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct imx230 *imx230 = to_imx230(sd);

	gpiod_set_value_cansleep(imx230->reset_gpio, 1);
	clk_disable_unprepare(imx230->xclk);
	regulator_bulk_disable(IMX230_NUM_SUPPLIES, imx230->supplies);

	return 0;
}

static int imx230_identify(struct imx230 *imx230)
{
	u64 val;
	int ret;

	ret = cci_read(imx230->regmap, IMX230_REG_CHIP_ID, &val, NULL);
	if (ret)
		return dev_err_probe(imx230->dev, ret, "failed to read chip id\n");

	if (val != IMX230_CHIP_ID)
		return dev_err_probe(imx230->dev, -ENODEV,
				     "unexpected chip id %#llx\n", val);

	return 0;
}

static int imx230_check_hwcfg(struct imx230 *imx230)
{
	struct v4l2_fwnode_endpoint ep = { .bus_type = V4L2_MBUS_CSI2_DPHY };
	struct fwnode_handle *endpoint;
	unsigned int i, j;
	int ret;

	endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(imx230->dev), NULL);
	if (!endpoint)
		return dev_err_probe(imx230->dev, -EINVAL, "missing endpoint\n");

	ret = v4l2_fwnode_endpoint_alloc_parse(endpoint, &ep);
	fwnode_handle_put(endpoint);
	if (ret)
		return ret;

	if (ep.bus.mipi_csi2.num_data_lanes != 4) {
		ret = dev_err_probe(imx230->dev, -EINVAL,
				    "only 4 data lanes are supported\n");
		goto out;
	}

	for (i = 0; i < ARRAY_SIZE(imx230_link_freqs); i++) {
		for (j = 0; j < ep.nr_of_link_frequencies; j++)
			if (ep.link_frequencies[j] == imx230_link_freqs[i])
				break;
		if (j == ep.nr_of_link_frequencies) {
			ret = dev_err_probe(imx230->dev, -EINVAL,
					    "link frequency %lld not in DT\n",
					    imx230_link_freqs[i]);
			goto out;
		}
	}

out:
	v4l2_fwnode_endpoint_free(&ep);
	return ret;
}

static int imx230_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct imx230 *imx230;
	unsigned int i;
	u32 xclk_freq;
	int ret;

	imx230 = devm_kzalloc(dev, sizeof(*imx230), GFP_KERNEL);
	if (!imx230)
		return -ENOMEM;

	imx230->dev = dev;
	v4l2_i2c_subdev_init(&imx230->sd, client, &imx230_subdev_ops);

	ret = imx230_check_hwcfg(imx230);
	if (ret)
		return ret;

	imx230->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(imx230->regmap))
		return PTR_ERR(imx230->regmap);

	imx230->xclk = devm_clk_get(dev, NULL);
	if (IS_ERR(imx230->xclk))
		return dev_err_probe(dev, PTR_ERR(imx230->xclk),
				     "failed to get xclk\n");

	xclk_freq = clk_get_rate(imx230->xclk);
	if (xclk_freq != IMX230_XCLK_FREQ)
		return dev_err_probe(dev, -EINVAL,
				     "xclk %u Hz unsupported\n", xclk_freq);

	for (i = 0; i < IMX230_NUM_SUPPLIES; i++)
		imx230->supplies[i].supply = imx230_supply_names[i];
	ret = devm_regulator_bulk_get(dev, IMX230_NUM_SUPPLIES,
				      imx230->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get regulators\n");

	imx230->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(imx230->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(imx230->reset_gpio),
				     "failed to get reset gpio\n");

	ret = imx230_power_on(dev);
	if (ret)
		return ret;

	ret = imx230_identify(imx230);
	if (ret)
		goto err_power_off;

	imx230->mode = &imx230_modes[0];

	ret = imx230_init_controls(imx230);
	if (ret)
		goto err_power_off;

	imx230->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	imx230->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	imx230->sd.entity.ops = &imx230_entity_ops;
	imx230->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&imx230->sd.entity, 1, &imx230->pad);
	if (ret)
		goto err_ctrls;

	imx230->sd.state_lock = imx230->ctrls.lock;
	ret = v4l2_subdev_init_finalize(&imx230->sd);
	if (ret)
		goto err_entity;

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);

	ret = v4l2_async_register_subdev_sensor(&imx230->sd);
	if (ret)
		goto err_pm;

	return 0;

err_pm:
	pm_runtime_disable(dev);
	pm_runtime_set_suspended(dev);
	v4l2_subdev_cleanup(&imx230->sd);
err_entity:
	media_entity_cleanup(&imx230->sd.entity);
err_ctrls:
	v4l2_ctrl_handler_free(&imx230->ctrls);
err_power_off:
	imx230_power_off(dev);
	return ret;
}

static void imx230_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx230 *imx230 = to_imx230(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&imx230->ctrls);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		imx230_power_off(&client->dev);
	pm_runtime_set_suspended(&client->dev);
}

static DEFINE_RUNTIME_DEV_PM_OPS(imx230_pm_ops, imx230_power_off,
				 imx230_power_on, NULL);

static const struct of_device_id imx230_of_match[] = {
	{ .compatible = "sony,imx230" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx230_of_match);

static struct i2c_driver imx230_i2c_driver = {
	.driver = {
		.name = "imx230",
		.of_match_table = imx230_of_match,
		.pm = pm_ptr(&imx230_pm_ops),
	},
	.probe = imx230_probe,
	.remove = imx230_remove,
};
module_i2c_driver(imx230_i2c_driver);

MODULE_DESCRIPTION("Sony IMX230 sensor driver");
MODULE_LICENSE("GPL");
