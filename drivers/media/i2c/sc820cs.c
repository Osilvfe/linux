// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2026 Osilvfe <osilvfe@gmail.com>
/*
 * SmartSens SC820CS driver for OnePlus Pad Pro (caihong).
 *
 * The default probe path only powers the sensor long enough to identify it.
 * Streaming is enabled through the normal V4L2 s_stream callback using the
 * Caihong downstream initialization sequence recovered from the vendor tree.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/nvmem-provider.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>

#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#include "sc820cs-caihong-regs.h"

#define SC820CS_XVCLK_FREQ		24000000

#define SC820CS_REG_CHIP_ID_H		0x3107
#define SC820CS_REG_CHIP_ID_L		0x3108
#define SC820CS_CHIP_ID			0xd154

#define SC820CS_NATIVE_WIDTH		3264
#define SC820CS_NATIVE_HEIGHT		2448
#define SC820CS_NUM_DATA_LANES		4
#define SC820CS_LINK_FREQ		457500000ULL
#define SC820CS_PIXEL_RATE		291600000ULL

#define SC820CS_HTS			3888
#define SC820CS_VTS			2500
#define SC820CS_VTS_MAX			0xffff
#define SC820CS_EXPOSURE_MIN		1
#define SC820CS_EXPOSURE_MARGIN		6
#define SC820CS_EXPOSURE_MAX		(SC820CS_VTS - SC820CS_EXPOSURE_MARGIN)
#define SC820CS_EXPOSURE_DEFAULT	750
#define SC820CS_ANALOGUE_GAIN_MIN	1024
#define SC820CS_ANALOGUE_GAIN_MAX	16384
#define SC820CS_ANALOGUE_GAIN_DEFAULT	1024

#define SC820CS_OTP_SIZE		0x729
#define SC820CS_OTP_GROUPS		2
#define SC820CS_OTP_PAGES_PER_GROUP	5
#define SC820CS_OTP_PAGE_SIZE		390
#define SC820CS_OTP_LAST_PAGE_SIZE	273
#define SC820CS_OTP_READ_CHUNK		12

static const unsigned int sc820cs_supply_loads[] = {
	300000, /* DOVDD / L4B */
	600000, /* AVDD / L16B */
	600000, /* DVDD / L2G */
};

struct sc820cs {
	struct device *dev;
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct regmap *regmap;
	struct clk *xvclk;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[3];
	struct v4l2_ctrl_handler ctrls;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *hblank;
	struct v4l2_mbus_framefmt active_fmt;
	u8 *otp_data;
	struct mutex mutex;
	bool otp_valid;
	bool powered;
	bool streaming;
};

static const char * const sc820cs_supply_names[] = {
	"dovdd",
	"avdd",
	"dvdd",
};

static const s64 sc820cs_link_freq_menu[] = {
	SC820CS_LINK_FREQ,
};

struct sc820cs_gain_step {
	u32 gain;
	u8 coarse;
};

static const struct sc820cs_gain_step sc820cs_gain_steps[] = {
	{ 1024, 0x00 },
	{ 2048, 0x08 },
	{ 4096, 0x09 },
	{ 8192, 0x0b },
	{ 16384, 0x0f },
};

static const u8 sc820cs_otp_init_modes[] = {
	0x38, 0x18, 0x58,
};

struct sc820cs_otp_section {
	u16 flag;
	u16 start;
	u16 end;
	u16 checksum;
};

/* Module data, serial, two AWB records and lens-shading calibration. */
static const struct sc820cs_otp_section sc820cs_otp_sections[] = {
	{ 0x0000, 0x0000, 0x0007, 0x0008 },
	{ 0x0009, 0x0009, 0x0021, 0x0022 },
	{ 0x0023, 0x0023, 0x002f, 0x0030 },
	{ 0x0031, 0x0031, 0x003d, 0x003e },
	{ 0x003f, 0x003f, 0x0727, 0x0728 },
};

static inline struct sc820cs *to_sc820cs(struct v4l2_subdev *sd)
{
	return container_of(sd, struct sc820cs, sd);
}

static int sc820cs_power_on(struct sc820cs *sc820cs)
{
	unsigned int i;
	int ret;

	if (sc820cs->powered)
		return 0;

	ret = pm_runtime_resume_and_get(sc820cs->dev);
	if (ret < 0)
		return dev_err_probe(sc820cs->dev, ret,
				     "failed to enable camera power domain\n");

	/* Keep the sensor in reset while rails and the input clock settle. */
	gpiod_set_value_cansleep(sc820cs->reset_gpio, 1);

	for (i = 0; i < ARRAY_SIZE(sc820cs->supplies); i++) {
		ret = regulator_set_load(sc820cs->supplies[i].consumer,
					 sc820cs_supply_loads[i]);
		if (ret) {
			dev_err_probe(sc820cs->dev, ret,
				      "failed to set %s load to %u uA\n",
				      sc820cs_supply_names[i],
				      sc820cs_supply_loads[i]);
			goto put_power_domain;
		}
	}

	ret = regulator_bulk_enable(ARRAY_SIZE(sc820cs->supplies),
				    sc820cs->supplies);
	if (ret) {
		dev_err_probe(sc820cs->dev, ret, "failed to enable supplies\n");
		goto put_power_domain;
	}

	ret = clk_prepare_enable(sc820cs->xvclk);
	if (ret) {
		dev_err(sc820cs->dev, "failed to enable xvclk: %d\n", ret);
		goto disable_supplies;
	}

	usleep_range(5000, 6000);
	gpiod_set_value_cansleep(sc820cs->reset_gpio, 0);
	usleep_range(10000, 12000);

	dev_dbg(sc820cs->dev, "powered: reset released, xvclk=%lu Hz\n",
		clk_get_rate(sc820cs->xvclk));
	sc820cs->powered = true;

	return 0;

disable_supplies:
	regulator_bulk_disable(ARRAY_SIZE(sc820cs->supplies),
			       sc820cs->supplies);

put_power_domain:
	pm_runtime_put_sync(sc820cs->dev);
	return ret;
}

static void sc820cs_power_off(struct sc820cs *sc820cs)
{
	if (!sc820cs->powered)
		return;

	gpiod_set_value_cansleep(sc820cs->reset_gpio, 1);
	clk_disable_unprepare(sc820cs->xvclk);
	regulator_bulk_disable(ARRAY_SIZE(sc820cs->supplies),
				       sc820cs->supplies);
	pm_runtime_put_sync(sc820cs->dev);
	sc820cs->powered = false;
}

static int sc820cs_identify(struct sc820cs *sc820cs)
{
	u64 value;
	int ret;

	/* The upstream SmartSens driver reads the two-byte ID in one CCI transfer. */
	ret = cci_read(sc820cs->regmap, CCI_REG16(SC820CS_REG_CHIP_ID_H),
		       &value, NULL);
	if (ret)
		return dev_err_probe(sc820cs->dev, ret,
				     "failed to read chip ID\n");

	if ((u16)value != SC820CS_CHIP_ID) {
		dev_err(sc820cs->dev, "chip ID mismatch: got 0x%04x, expected 0x%04x\n",
			(u16)value, SC820CS_CHIP_ID);
		return -ENODEV;
	}

	dev_info(sc820cs->dev, "SC820CS detected, chip ID 0x%04x\n",
		 (u16)value);
	return 0;
}

static int sc820cs_otp_write_init(struct sc820cs *sc820cs, u8 mode)
{
	const struct cci_reg_sequence regs[] = {
		{ CCI_REG8(0x36b0), 0x48 },
		{ CCI_REG8(0x36b1), mode },
		{ CCI_REG8(0x36b2), 0x41 },
	};

	return cci_multi_reg_write(sc820cs->regmap, regs,
				   ARRAY_SIZE(regs), NULL);
}

static int sc820cs_otp_read_page(struct sc820cs *sc820cs,
				 unsigned int page, u8 *data, size_t size)
{
	unsigned int base = 0x8200 + page * 0x200;
	const struct cci_reg_sequence regs[] = {
		{ CCI_REG8(0x4408), base >> 8 },
		{ CCI_REG8(0x4409), base & 0xff },
		{ CCI_REG8(0x440a), (base + 0x1ff) >> 8 },
		{ CCI_REG8(0x440b), 0xff },
		{ CCI_REG8(0x4401), 0x13 },
		{ CCI_REG8(0x4412), 0x03 + page * 2 },
		{ CCI_REG8(0x4407), 0x00 },
		{ CCI_REG8(0x4400), 0x11 },
	};
	unsigned int offset;
	int ret;

	ret = cci_multi_reg_write(sc820cs->regmap, regs,
				  ARRAY_SIZE(regs), NULL);
	if (ret)
		return ret;

	usleep_range(10000, 11000);
	for (offset = 0; offset < size; offset += SC820CS_OTP_READ_CHUNK) {
		size_t count = min_t(size_t, size - offset,
				     SC820CS_OTP_READ_CHUNK);

		ret = regmap_bulk_read(sc820cs->regmap, base + 0x7a + offset,
				       data + offset, count);
		if (ret)
			return ret;
	}

	return 0;
}

static int sc820cs_otp_read_group(struct sc820cs *sc820cs,
				  unsigned int group, u8 *data)
{
	unsigned int offset = 0;
	unsigned int page;
	int ret;

	memset(data, 0, SC820CS_OTP_SIZE);
	for (page = 0; page < SC820CS_OTP_PAGES_PER_GROUP; page++) {
		size_t size = page == SC820CS_OTP_PAGES_PER_GROUP - 1 ?
			      SC820CS_OTP_LAST_PAGE_SIZE :
			      SC820CS_OTP_PAGE_SIZE;

		ret = sc820cs_otp_read_page(sc820cs,
					    group * SC820CS_OTP_PAGES_PER_GROUP + page,
					    data + offset, size);
		if (ret)
			return ret;

		offset += size;
	}

	return offset == SC820CS_OTP_SIZE ? 0 : -EIO;
}

static int sc820cs_otp_validate(const u8 *data)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(sc820cs_otp_sections); i++) {
		const struct sc820cs_otp_section *section =
			&sc820cs_otp_sections[i];
		unsigned int offset;
		unsigned int sum = 0;

		if (data[section->flag] != 0x01)
			return -EBADMSG;

		for (offset = section->start; offset <= section->end; offset++)
			sum += data[offset];

		if (data[section->checksum] != sum % 255 + 1)
			return -EBADMSG;
	}

	return 0;
}

static int sc820cs_otp_load(struct sc820cs *sc820cs)
{
	bool was_powered = sc820cs->powered;
	unsigned int attempt;
	unsigned int group;
	int ret;
	int last_error = -EBADMSG;

	if (sc820cs->streaming)
		return -EBUSY;

	ret = sc820cs_power_on(sc820cs);
	if (ret)
		return ret;

	ret = sc820cs_identify(sc820cs);
	if (ret) {
		last_error = ret;
		goto out_power;
	}

	for (attempt = 0; attempt < ARRAY_SIZE(sc820cs_otp_init_modes);
	     attempt++) {
		ret = sc820cs_otp_write_init(sc820cs,
					     sc820cs_otp_init_modes[attempt]);
		if (ret) {
			last_error = ret;
			continue;
		}

		for (group = 0; group < SC820CS_OTP_GROUPS; group++) {
			ret = sc820cs_otp_read_group(sc820cs, group,
						     sc820cs->otp_data);
			if (!ret)
				ret = sc820cs_otp_validate(sc820cs->otp_data);
			if (!ret) {
				sc820cs->otp_valid = true;
				dev_info(sc820cs->dev,
					 "validated OTP calibration group %u\n",
					 group);
				last_error = 0;
				goto out_power;
			}

			last_error = ret;
		}
	}

out_power:
	if (!was_powered)
		sc820cs_power_off(sc820cs);

	return last_error;
}

static int sc820cs_otp_nvmem_read(void *priv, unsigned int offset,
				  void *value, size_t bytes)
{
	struct sc820cs *sc820cs = priv;
	int ret = 0;

	if (offset > SC820CS_OTP_SIZE || bytes > SC820CS_OTP_SIZE - offset)
		return -EINVAL;

	mutex_lock(&sc820cs->mutex);
	if (!sc820cs->otp_valid)
		ret = sc820cs_otp_load(sc820cs);
	if (!ret)
		memcpy(value, sc820cs->otp_data + offset, bytes);
	else
		dev_err_ratelimited(sc820cs->dev,
				    "failed to load OTP calibration: %d\n", ret);
	mutex_unlock(&sc820cs->mutex);

	return ret;
}

static int sc820cs_register_otp_nvmem(struct sc820cs *sc820cs)
{
	struct nvmem_config config = {
		.dev = sc820cs->dev,
		.name = "sc820cs-otp",
		.id = NVMEM_DEVID_NONE,
		.owner = THIS_MODULE,
		.type = NVMEM_TYPE_OTP,
		.read_only = true,
		.root_only = false,
		.reg_read = sc820cs_otp_nvmem_read,
		.size = SC820CS_OTP_SIZE,
		.word_size = 1,
		.stride = 1,
		.priv = sc820cs,
	};
	struct nvmem_device *nvmem;

	sc820cs->otp_data = devm_kmalloc(sc820cs->dev, SC820CS_OTP_SIZE,
					 GFP_KERNEL);
	if (!sc820cs->otp_data)
		return -ENOMEM;

	nvmem = devm_nvmem_register(sc820cs->dev, &config);
	return PTR_ERR_OR_ZERO(nvmem);
}

static int sc820cs_write_mode(struct sc820cs *sc820cs)
{
	int ret;

	/* The vendor sequence starts with a software reset; leave it time to settle. */
	ret = cci_write(sc820cs->regmap,
			sc820cs_caihong_3264x2448_regs[0].reg,
			sc820cs_caihong_3264x2448_regs[0].val, NULL);
	if (ret)
		return ret;
	usleep_range(5000, 6000);

	ret = cci_multi_reg_write(sc820cs->regmap,
			&sc820cs_caihong_3264x2448_regs[1],
			ARRAY_SIZE(sc820cs_caihong_3264x2448_regs) - 1, NULL);
	if (ret || sc820cs->active_fmt.width == SC820CS_NATIVE_WIDTH)
		return ret;

	return cci_multi_reg_write(sc820cs->regmap,
			sc820cs_caihong_1632x1224_regs,
			ARRAY_SIZE(sc820cs_caihong_1632x1224_regs), NULL);
}

static int sc820cs_write_exposure(struct sc820cs *sc820cs,
				    unsigned int exposure)
{
	unsigned int encoded = exposure * 2;
	const struct cci_reg_sequence regs[] = {
		{ CCI_REG8(0x3e20), (encoded >> 20) & 0x0f },
		{ CCI_REG8(0x3e00), (encoded >> 12) & 0xff },
		{ CCI_REG8(0x3e01), (encoded >> 4) & 0xff },
		{ CCI_REG8(0x3e02), (exposure & 0x07) << 5 },
	};

	return cci_multi_reg_write(sc820cs->regmap, regs, ARRAY_SIZE(regs), NULL);
}

static int sc820cs_write_gain(struct sc820cs *sc820cs, unsigned int gain)
{
	const struct sc820cs_gain_step *step = &sc820cs_gain_steps[0];
	struct cci_reg_sequence regs[2];
	unsigned int fine;
	unsigned int i;

	for (i = 1; i < ARRAY_SIZE(sc820cs_gain_steps); i++) {
		if (gain < sc820cs_gain_steps[i].gain)
			break;
		step = &sc820cs_gain_steps[i];
	}

	fine = (gain * 128 + step->gain / 2) / step->gain;
	regs[0].reg = CCI_REG8(0x3e08);
	regs[0].val = step->coarse;
	regs[1].reg = CCI_REG8(0x3e07);
	regs[1].val = clamp_val(fine, 0, 0xff);

	return cci_multi_reg_write(sc820cs->regmap, regs, ARRAY_SIZE(regs), NULL);
}

static int sc820cs_write_vblank(struct sc820cs *sc820cs,
				unsigned int vblank)
{
	return cci_write(sc820cs->regmap, CCI_REG16(0x320e),
			 sc820cs->active_fmt.height + vblank, NULL);
}

static int sc820cs_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct sc820cs *sc820cs = container_of(ctrl->handler,
					       struct sc820cs, ctrls);
	unsigned int exposure_max;

	if (ctrl->id == V4L2_CID_VBLANK) {
		exposure_max = sc820cs->active_fmt.height + ctrl->val -
			       SC820CS_EXPOSURE_MARGIN;
		__v4l2_ctrl_modify_range(sc820cs->exposure,
					 SC820CS_EXPOSURE_MIN,
					 exposure_max, 1,
					 min(SC820CS_EXPOSURE_DEFAULT,
					     exposure_max));
	}

	if (!sc820cs->powered)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		return sc820cs_write_exposure(sc820cs, ctrl->val);
	case V4L2_CID_ANALOGUE_GAIN:
		return sc820cs_write_gain(sc820cs, ctrl->val);
	case V4L2_CID_VBLANK:
		return sc820cs_write_vblank(sc820cs, ctrl->val);
	default:
		return 0;
	}
}

static const struct v4l2_ctrl_ops sc820cs_ctrl_ops = {
	.s_ctrl = sc820cs_set_ctrl,
};

static int sc820cs_init_controls(struct sc820cs *sc820cs)
{
	struct v4l2_fwnode_device_properties props;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *hblank;
	int ret;

	ret = v4l2_ctrl_handler_init(&sc820cs->ctrls, 8);
	if (ret)
		return ret;

	sc820cs->ctrls.lock = &sc820cs->mutex;
	link_freq = v4l2_ctrl_new_int_menu(&sc820cs->ctrls, NULL,
					   V4L2_CID_LINK_FREQ, 0, 0,
					   sc820cs_link_freq_menu);
	pixel_rate = v4l2_ctrl_new_std(&sc820cs->ctrls, NULL,
				       V4L2_CID_PIXEL_RATE,
				       SC820CS_PIXEL_RATE, SC820CS_PIXEL_RATE,
				       1, SC820CS_PIXEL_RATE);
	hblank = v4l2_ctrl_new_std(&sc820cs->ctrls, NULL, V4L2_CID_HBLANK,
				   SC820CS_HTS - SC820CS_NATIVE_WIDTH,
				   SC820CS_HTS - SC820CS_NATIVE_WIDTH, 1,
				   SC820CS_HTS - SC820CS_NATIVE_WIDTH);
	sc820cs->vblank =
		v4l2_ctrl_new_std(&sc820cs->ctrls, &sc820cs_ctrl_ops,
				  V4L2_CID_VBLANK,
		SC820CS_VTS - SC820CS_NATIVE_HEIGHT,
		SC820CS_VTS_MAX - SC820CS_NATIVE_HEIGHT, 1,
		SC820CS_VTS - SC820CS_NATIVE_HEIGHT);
	sc820cs->exposure = v4l2_ctrl_new_std(&sc820cs->ctrls,
					      &sc820cs_ctrl_ops,
					      V4L2_CID_EXPOSURE,
					      SC820CS_EXPOSURE_MIN,
					      SC820CS_EXPOSURE_MAX, 1,
					      SC820CS_EXPOSURE_DEFAULT);
	v4l2_ctrl_new_std(&sc820cs->ctrls, &sc820cs_ctrl_ops,
			  V4L2_CID_ANALOGUE_GAIN, SC820CS_ANALOGUE_GAIN_MIN,
			  SC820CS_ANALOGUE_GAIN_MAX, 1,
			  SC820CS_ANALOGUE_GAIN_DEFAULT);
	ret = v4l2_fwnode_device_parse(sc820cs->dev, &props);
	if (ret)
		goto free_ctrls;

	ret = v4l2_ctrl_new_fwnode_properties(&sc820cs->ctrls,
					      &sc820cs_ctrl_ops, &props);
	if (ret)
		goto free_ctrls;

	if (sc820cs->ctrls.error) {
		ret = sc820cs->ctrls.error;
		goto free_ctrls;
	}

	link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	pixel_rate->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	sc820cs->hblank = hblank;
	sc820cs->sd.ctrl_handler = &sc820cs->ctrls;
	return 0;

free_ctrls:
	v4l2_ctrl_handler_free(&sc820cs->ctrls);
	return ret;
}

static void sc820cs_fill_format(struct v4l2_mbus_framefmt *fmt)
{
	fmt->width = SC820CS_NATIVE_WIDTH;
	fmt->height = SC820CS_NATIVE_HEIGHT;
	fmt->code = MEDIA_BUS_FMT_SBGGR10_1X10;
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_RAW;
}

static int sc820cs_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index)
		return -EINVAL;

	code->code = MEDIA_BUS_FMT_SBGGR10_1X10;
	return 0;
}

static int sc820cs_enum_frame_size(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *state,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index > 1 || fse->code != MEDIA_BUS_FMT_SBGGR10_1X10)
		return -EINVAL;

	fse->min_width = SC820CS_NATIVE_WIDTH / (fse->index + 1);
	fse->max_width = fse->min_width;
	fse->min_height = SC820CS_NATIVE_HEIGHT / (fse->index + 1);
	fse->max_height = fse->min_height;
	return 0;
}

static int sc820cs_get_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_state *state,
			   struct v4l2_subdev_format *fmt)
{
	struct sc820cs *sc820cs = to_sc820cs(sd);

	if (fmt->pad)
		return -EINVAL;

	mutex_lock(&sc820cs->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		fmt->format = *v4l2_subdev_state_get_format(state, fmt->pad);
		if (!fmt->format.width)
			sc820cs_fill_format(&fmt->format);
	} else {
		fmt->format = sc820cs->active_fmt;
	}
	mutex_unlock(&sc820cs->mutex);
	return 0;
}

static int sc820cs_set_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_state *state,
			   struct v4l2_subdev_format *fmt)
{
	struct sc820cs *sc820cs = to_sc820cs(sd);
	unsigned int divisor = fmt->format.width <= SC820CS_NATIVE_WIDTH / 2 &&
		fmt->format.height <= SC820CS_NATIVE_HEIGHT / 2 ? 2 : 1;
	unsigned int vblank;
	int ret = 0;

	if (fmt->pad)
		return -EINVAL;

	sc820cs_fill_format(&fmt->format);
	fmt->format.width /= divisor;
	fmt->format.height /= divisor;
	mutex_lock(&sc820cs->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		*v4l2_subdev_state_get_format(state, fmt->pad) = fmt->format;
	} else if (sc820cs->streaming) {
		ret = -EBUSY;
	} else {
		sc820cs->active_fmt = fmt->format;
		vblank = SC820CS_VTS - fmt->format.height;
		__v4l2_ctrl_modify_range(sc820cs->hblank,
			SC820CS_HTS - fmt->format.width,
			SC820CS_HTS - fmt->format.width, 1,
			SC820CS_HTS - fmt->format.width);
		__v4l2_ctrl_modify_range(sc820cs->vblank, vblank,
			SC820CS_VTS_MAX - fmt->format.height, 1, vblank);
		ret = __v4l2_ctrl_s_ctrl(sc820cs->vblank, vblank);
	}
	mutex_unlock(&sc820cs->mutex);
	return ret;
}

static int sc820cs_get_selection(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_selection *sel)
{
	if (sel->pad)
		return -EINVAL;

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = SC820CS_NATIVE_WIDTH;
		sel->r.height = SC820CS_NATIVE_HEIGHT;
		return 0;
	default:
		return -EINVAL;
	}
}

static int sc820cs_get_mbus_config(struct v4l2_subdev *sd, unsigned int pad,
				   struct v4l2_mbus_config *config)
{
	unsigned int i;

	if (pad)
		return -EINVAL;

	config->type = V4L2_MBUS_CSI2_DPHY;
	config->link_freq = SC820CS_LINK_FREQ;
	/* Zero flags means the sensor uses a continuous CSI-2 clock. */
	config->bus.mipi_csi2.flags = 0;
	config->bus.mipi_csi2.clock_lane = 7;
	config->bus.mipi_csi2.num_data_lanes = SC820CS_NUM_DATA_LANES;
	for (i = 0; i < SC820CS_NUM_DATA_LANES; i++)
		config->bus.mipi_csi2.data_lanes[i] = i;

	return 0;
}

static int sc820cs_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct sc820cs *sc820cs = to_sc820cs(sd);
	int ret = 0;

	mutex_lock(&sc820cs->mutex);
	if (!enable) {
		if (sc820cs->streaming)
			ret = cci_write(sc820cs->regmap, CCI_REG8(0x0100), 0x00,
					NULL);
		sc820cs->streaming = false;
		sc820cs_power_off(sc820cs);
		goto unlock;
	}

	if (sc820cs->streaming)
		goto unlock;

	ret = sc820cs_power_on(sc820cs);
	if (ret)
		goto unlock;

	ret = sc820cs_identify(sc820cs);
	if (ret)
		goto power_off;

	ret = sc820cs_write_mode(sc820cs);
	if (ret)
		goto power_off;

	ret = __v4l2_ctrl_handler_setup(&sc820cs->ctrls);
	if (ret)
		goto power_off;

	ret = cci_write(sc820cs->regmap, CCI_REG8(0x0100), 0x01, NULL);
	if (ret)
		goto power_off;

	sc820cs->streaming = true;
	dev_info(sc820cs->dev, "SC820CS streaming %ux%u RAW10\n",
		 sc820cs->active_fmt.width, sc820cs->active_fmt.height);
	goto unlock;

power_off:
	sc820cs_power_off(sc820cs);
unlock:
	mutex_unlock(&sc820cs->mutex);
	return ret;
}

static const struct v4l2_subdev_video_ops sc820cs_video_ops = {
	.s_stream = sc820cs_set_stream,
};

static const struct v4l2_subdev_pad_ops sc820cs_pad_ops = {
	.enum_mbus_code = sc820cs_enum_mbus_code,
	.enum_frame_size = sc820cs_enum_frame_size,
	.get_fmt = sc820cs_get_fmt,
	.set_fmt = sc820cs_set_fmt,
	.get_selection = sc820cs_get_selection,
	.get_mbus_config = sc820cs_get_mbus_config,
};

static const struct v4l2_subdev_ops sc820cs_subdev_ops = {
	.video = &sc820cs_video_ops,
	.pad = &sc820cs_pad_ops,
};

static const struct media_entity_operations sc820cs_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static int sc820cs_get_resources(struct sc820cs *sc820cs)
{
	struct v4l2_fwnode_endpoint ep = {
		.bus_type = V4L2_MBUS_CSI2_DPHY,
	};
	struct fwnode_handle *endpoint;
	unsigned long rate;
	unsigned int i;
	int ret;

	sc820cs->xvclk = devm_clk_get(sc820cs->dev, "xvclk");
	if (IS_ERR(sc820cs->xvclk))
		return dev_err_probe(sc820cs->dev, PTR_ERR(sc820cs->xvclk),
				     "failed to get xvclk\n");

	ret = clk_set_rate(sc820cs->xvclk, SC820CS_XVCLK_FREQ);
	if (ret)
		return dev_err_probe(sc820cs->dev, ret, "failed to set xvclk\n");
	rate = clk_get_rate(sc820cs->xvclk);
	if (rate != SC820CS_XVCLK_FREQ)
		return dev_err_probe(sc820cs->dev, -EINVAL,
				     "unsupported xvclk rate %lu\n", rate);

	sc820cs->reset_gpio = devm_gpiod_get_optional(sc820cs->dev, "reset",
						       GPIOD_OUT_HIGH);
	if (IS_ERR(sc820cs->reset_gpio))
		return dev_err_probe(sc820cs->dev,
				     PTR_ERR(sc820cs->reset_gpio),
				     "failed to get reset GPIO\n");

	for (i = 0; i < ARRAY_SIZE(sc820cs_supply_names); i++)
		sc820cs->supplies[i].supply = sc820cs_supply_names[i];

	ret = devm_regulator_bulk_get(sc820cs->dev,
				       ARRAY_SIZE(sc820cs->supplies),
				       sc820cs->supplies);
	if (ret)
		return dev_err_probe(sc820cs->dev, ret,
				     "failed to get supplies\n");

	endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(sc820cs->dev), NULL);
	if (!endpoint)
		return dev_err_probe(sc820cs->dev, -EINVAL,
				     "missing CSI-2 endpoint\n");

	ret = v4l2_fwnode_endpoint_parse(endpoint, &ep);
	fwnode_handle_put(endpoint);
	if (ret)
		return dev_err_probe(sc820cs->dev, ret,
				     "failed to parse CSI-2 endpoint\n");

	if (ep.bus.mipi_csi2.num_data_lanes != SC820CS_NUM_DATA_LANES)
		return dev_err_probe(sc820cs->dev, -EINVAL,
				     "expected %u CSI-2 lanes, DT has %u\n",
				     SC820CS_NUM_DATA_LANES,
				     ep.bus.mipi_csi2.num_data_lanes);

	return 0;
}

static int sc820cs_probe(struct i2c_client *client)
{
	struct sc820cs *sc820cs;
	int ret;

	sc820cs = devm_kzalloc(&client->dev, sizeof(*sc820cs), GFP_KERNEL);
	if (!sc820cs)
		return -ENOMEM;

	sc820cs->dev = &client->dev;
	sc820cs_fill_format(&sc820cs->active_fmt);
	mutex_init(&sc820cs->mutex);
	sc820cs->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(sc820cs->regmap))
		return dev_err_probe(&client->dev, PTR_ERR(sc820cs->regmap),
				     "failed to initialize CCI regmap\n");

	ret = sc820cs_get_resources(sc820cs);
	if (ret)
		return ret;

	ret = devm_pm_runtime_enable(sc820cs->dev);
	if (ret)
		return dev_err_probe(sc820cs->dev, ret,
				     "failed to enable runtime PM\n");

	v4l2_i2c_subdev_init(&sc820cs->sd, client, &sc820cs_subdev_ops);

	ret = sc820cs_power_on(sc820cs);
	if (ret)
		return ret;

	ret = sc820cs_identify(sc820cs);
	sc820cs_power_off(sc820cs);
	if (ret)
		return ret;

	ret = sc820cs_register_otp_nvmem(sc820cs);
	if (ret)
		dev_warn(sc820cs->dev,
			 "failed to register OTP NVMEM provider: %d\n", ret);

	ret = sc820cs_init_controls(sc820cs);
	if (ret)
		return dev_err_probe(&client->dev, ret,
				     "failed to initialize front-camera controls\n");

	sc820cs->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sc820cs->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	sc820cs->sd.entity.ops = &sc820cs_entity_ops;
	sc820cs->pad.flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&sc820cs->sd.entity, 1, &sc820cs->pad);
	if (ret) {
		v4l2_ctrl_handler_free(&sc820cs->ctrls);
		sc820cs_power_off(sc820cs);
		return dev_err_probe(sc820cs->dev, ret,
				     "failed to initialize media entity\n");
	}

	ret = v4l2_async_register_subdev_sensor(&sc820cs->sd);
	if (ret) {
		media_entity_cleanup(&sc820cs->sd.entity);
		v4l2_ctrl_handler_free(&sc820cs->ctrls);
		sc820cs_power_off(sc820cs);
		return dev_err_probe(sc820cs->dev, ret,
				     "failed to register V4L2 subdevice\n");
	}

	dev_info(sc820cs->dev,
		 "V4L2 subdevice registered; streaming uses Caihong 3264x2448 mode\n");
	return 0;
}

static void sc820cs_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sc820cs *sc820cs = to_sc820cs(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_ctrl_handler_free(&sc820cs->ctrls);
	media_entity_cleanup(&sd->entity);
	if (sc820cs->streaming)
		cci_write(sc820cs->regmap, CCI_REG8(0x0100), 0x00, NULL);
	sc820cs->streaming = false;
	sc820cs_power_off(sc820cs);
}

static const struct of_device_id sc820cs_of_match[] = {
	{ .compatible = "smartsens,sc820cs" },
	{ }
};
MODULE_DEVICE_TABLE(of, sc820cs_of_match);

static struct i2c_driver sc820cs_i2c_driver = {
	.driver = {
		.name = "sc820cs",
		.of_match_table = sc820cs_of_match,
	},
	.probe = sc820cs_probe,
	.remove = sc820cs_remove,
};
module_i2c_driver(sc820cs_i2c_driver);

MODULE_DESCRIPTION("SmartSens SC820CS V4L2 sensor driver for Caihong");
MODULE_LICENSE("GPL");
