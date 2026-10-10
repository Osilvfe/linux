// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2026 Osilvfe <osilvfe@gmail.com>
/*
 * Giantec GT9772 voice-coil motor driver
 *
 * Register programming and the DAC protocol are derived from Qualcomm's
 * GT9772 actuator description.  Caihong places this actuator on CCI0 master 1
 * at downstream 8-bit slave address 0x18 (Linux 7-bit address 0x0c).
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>

#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>

#define GT9772_REG_DAC			CCI_REG16(0x03)
#define GT9772_REG_CFG_ED		CCI_REG8(0xed)
#define GT9772_REG_CFG_06		CCI_REG8(0x06)
#define GT9772_REG_CFG_07		CCI_REG8(0x07)
#define GT9772_REG_CFG_08		CCI_REG8(0x08)

#define GT9772_CFG_ED_VALUE		0xab
#define GT9772_CFG_06_VALUE		0x84
#define GT9772_CFG_07_VALUE		0x01
#define GT9772_CFG_08_VALUE		0x55

#define GT9772_MAX_FOCUS_POS		1023
#define GT9772_DEFAULT_FOCUS_POS	336
#define GT9772_PARK_FOCUS_POS		40
#define GT9772_RAMP_STEP		16
#define GT9772_RAMP_DELAY_US		7000
#define GT9772_POWER_SETTLE_US		10000
#define GT9772_INIT_WRITE_DELAY_US	100
#define GT9772_POWER_DOWN_DELAY_US	1000

struct gt9772 {
	struct device *dev;
	struct regmap *regmap;
	struct regulator_bulk_data supplies[2];
	struct v4l2_subdev sd;
	struct v4l2_ctrl_handler ctrls;
	struct v4l2_ctrl *focus;
};

static const char * const gt9772_supply_names[] = {
	"vio",
	"vaf",
};

static const struct cci_reg_sequence gt9772_init_regs[] = {
	{ GT9772_REG_CFG_ED, GT9772_CFG_ED_VALUE },
	{ GT9772_REG_CFG_06, GT9772_CFG_06_VALUE },
	{ GT9772_REG_CFG_07, GT9772_CFG_07_VALUE },
	{ GT9772_REG_CFG_08, GT9772_CFG_08_VALUE },
};

static inline struct gt9772 *to_gt9772(struct v4l2_subdev *sd)
{
	return container_of(sd, struct gt9772, sd);
}

static int gt9772_write_focus(struct gt9772 *gt9772, unsigned int value)
{
	return cci_write(gt9772->regmap, GT9772_REG_DAC,
			 min(value, (unsigned int)GT9772_MAX_FOCUS_POS), NULL);
}

static int gt9772_hw_init(struct gt9772 *gt9772)
{
	unsigned int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(gt9772_init_regs); i++) {
		ret = cci_write(gt9772->regmap, gt9772_init_regs[i].reg,
				gt9772_init_regs[i].val, NULL);
		if (ret)
			return ret;

		/* Qualcomm's GT9772 table specifies 100 us after each write. */
		usleep_range(GT9772_INIT_WRITE_DELAY_US,
			     GT9772_INIT_WRITE_DELAY_US + 100);
	}

	return 0;
}

static int gt9772_power_on(struct gt9772 *gt9772)
{
	int ret;

	/*
	 * Caihong exposes both VIO and VAF to the actuator node.  The generic
	 * Qualcomm GT9772 tuning sequence explicitly waits 10 ms after VAF-on;
	 * enabling the two board rails together here preserves the board DT
	 * requirement while keeping that settle time.
	 */
	ret = regulator_bulk_enable(ARRAY_SIZE(gt9772->supplies),
				    gt9772->supplies);
	if (ret)
		return ret;

	usleep_range(GT9772_POWER_SETTLE_US, GT9772_POWER_SETTLE_US + 500);

	ret = gt9772_hw_init(gt9772);
	if (ret) {
		dev_err(gt9772->dev, "failed to initialize VCM: %d\n", ret);
		regulator_bulk_disable(ARRAY_SIZE(gt9772->supplies),
				       gt9772->supplies);
	}

	return ret;
}

static int gt9772_power_off(struct gt9772 *gt9772)
{
	int ret;

	ret = regulator_bulk_disable(ARRAY_SIZE(gt9772->supplies),
				     gt9772->supplies);
	usleep_range(GT9772_POWER_DOWN_DELAY_US,
		     GT9772_POWER_DOWN_DELAY_US + 500);

	return ret;
}

static int gt9772_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct gt9772 *gt9772 = container_of(ctrl->handler,
					     struct gt9772, ctrls);
	int ret = 0;

	if (!pm_runtime_get_if_in_use(gt9772->dev))
		return 0;

	if (ctrl->id == V4L2_CID_FOCUS_ABSOLUTE)
		ret = gt9772_write_focus(gt9772, ctrl->val);
	else
		ret = -EINVAL;

	pm_runtime_put(gt9772->dev);
	return ret;
}

static const struct v4l2_ctrl_ops gt9772_ctrl_ops = {
	.s_ctrl = gt9772_set_ctrl,
};

static int gt9772_move_toward(struct gt9772 *gt9772, int from, int to)
{
	int direction = from < to ? 1 : -1;
	int value = from;
	int ret;

	while (value != to) {
		int remaining = abs(to - value);
		int step = min(remaining, GT9772_RAMP_STEP);

		value += direction * step;
		ret = gt9772_write_focus(gt9772, value);
		if (ret)
			return ret;

		usleep_range(GT9772_RAMP_DELAY_US,
			     GT9772_RAMP_DELAY_US + 500);
	}

	return 0;
}

static int gt9772_suspend(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct gt9772 *gt9772 = to_gt9772(sd);
	int ret;

	ret = gt9772_move_toward(gt9772, gt9772->focus->val,
				 GT9772_PARK_FOCUS_POS);
	if (ret)
		dev_warn(dev, "failed to park lens: %d\n", ret);

	return gt9772_power_off(gt9772);
}

static int gt9772_resume(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct gt9772 *gt9772 = to_gt9772(sd);
	int ret;

	ret = gt9772_power_on(gt9772);
	if (ret)
		return ret;

	ret = gt9772_write_focus(gt9772, GT9772_PARK_FOCUS_POS);
	if (ret)
		goto err_power_off;

	ret = gt9772_move_toward(gt9772, GT9772_PARK_FOCUS_POS,
				 gt9772->focus->val);
	if (ret)
		goto err_power_off;

	return 0;

err_power_off:
	gt9772_power_off(gt9772);
	return ret;
}

static int gt9772_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	return pm_runtime_resume_and_get(sd->dev);
}

static int gt9772_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	pm_runtime_put_autosuspend(sd->dev);
	return 0;
}

static const struct v4l2_subdev_internal_ops gt9772_internal_ops = {
	.open = gt9772_open,
	.close = gt9772_close,
};

static const struct v4l2_subdev_ops gt9772_subdev_ops = { };

static int gt9772_probe(struct i2c_client *client)
{
	struct gt9772 *gt9772;
	unsigned int i;
	int ret;

	gt9772 = devm_kzalloc(&client->dev, sizeof(*gt9772), GFP_KERNEL);
	if (!gt9772)
		return -ENOMEM;

	gt9772->dev = &client->dev;
	gt9772->regmap = devm_cci_regmap_init_i2c(client, 8);
	if (IS_ERR(gt9772->regmap))
		return PTR_ERR(gt9772->regmap);

	for (i = 0; i < ARRAY_SIZE(gt9772_supply_names); i++)
		gt9772->supplies[i].supply = gt9772_supply_names[i];

	ret = devm_regulator_bulk_get(&client->dev,
				       ARRAY_SIZE(gt9772->supplies),
				       gt9772->supplies);
	if (ret)
		return dev_err_probe(&client->dev, ret,
				     "failed to get VCM supplies\n");

	v4l2_i2c_subdev_init(&gt9772->sd, client, &gt9772_subdev_ops);
	gt9772->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	gt9772->sd.internal_ops = &gt9772_internal_ops;

	v4l2_ctrl_handler_init(&gt9772->ctrls, 1);
	gt9772->focus = v4l2_ctrl_new_std(&gt9772->ctrls, &gt9772_ctrl_ops,
					  V4L2_CID_FOCUS_ABSOLUTE,
					  0, GT9772_MAX_FOCUS_POS, 1,
					  GT9772_DEFAULT_FOCUS_POS);
	if (gt9772->ctrls.error) {
		ret = gt9772->ctrls.error;
		goto err_free_ctrls;
	}
	gt9772->sd.ctrl_handler = &gt9772->ctrls;

	ret = media_entity_pads_init(&gt9772->sd.entity, 0, NULL);
	if (ret)
		goto err_free_ctrls;
	gt9772->sd.entity.function = MEDIA_ENT_F_LENS;

	/*
	 * Power once at probe so an absent/mis-addressed device fails through
	 * the initialization writes instead of registering a dead lens node.
	 */
	ret = gt9772_power_on(gt9772);
	if (ret)
		goto err_cleanup_media;

	ret = gt9772_write_focus(gt9772, GT9772_DEFAULT_FOCUS_POS);
	if (ret) {
		dev_err(&client->dev, "failed initial focus write: %d\n", ret);
		gt9772_power_off(gt9772);
		goto err_cleanup_media;
	}

	pm_runtime_set_active(&client->dev);
	pm_runtime_get_noresume(&client->dev);
	pm_runtime_enable(&client->dev);

	ret = v4l2_async_register_subdev(&gt9772->sd);
	if (ret)
		goto err_pm_runtime;

	pm_runtime_set_autosuspend_delay(&client->dev, 1000);
	pm_runtime_use_autosuspend(&client->dev);
	pm_runtime_put_autosuspend(&client->dev);

	dev_info(&client->dev, "GT9772 V4L2 lens actuator registered\n");
	return 0;

err_pm_runtime:
	pm_runtime_disable(&client->dev);
	pm_runtime_put_noidle(&client->dev);
	gt9772_power_off(gt9772);
err_cleanup_media:
	media_entity_cleanup(&gt9772->sd.entity);
err_free_ctrls:
	v4l2_ctrl_handler_free(&gt9772->ctrls);
	return ret;
}

static void gt9772_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct gt9772 *gt9772 = to_gt9772(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_ctrl_handler_free(&gt9772->ctrls);
	media_entity_cleanup(&gt9772->sd.entity);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		gt9772_power_off(gt9772);
	pm_runtime_set_suspended(&client->dev);
}

static const struct of_device_id gt9772_of_match[] = {
	{ .compatible = "giantec,gt9772" },
	{ }
};
MODULE_DEVICE_TABLE(of, gt9772_of_match);

static const struct i2c_device_id gt9772_i2c_ids[] = {
	{ "gt9772" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, gt9772_i2c_ids);

static DEFINE_RUNTIME_DEV_PM_OPS(gt9772_pm_ops,
				 gt9772_suspend, gt9772_resume, NULL);

static struct i2c_driver gt9772_i2c_driver = {
	.driver = {
		.name = "gt9772",
		.pm = pm_sleep_ptr(&gt9772_pm_ops),
		.of_match_table = gt9772_of_match,
	},
	.probe = gt9772_probe,
	.remove = gt9772_remove,
	.id_table = gt9772_i2c_ids,
};
module_i2c_driver(gt9772_i2c_driver);

MODULE_DESCRIPTION("Giantec GT9772 V4L2 lens actuator driver");
MODULE_LICENSE("GPL");
