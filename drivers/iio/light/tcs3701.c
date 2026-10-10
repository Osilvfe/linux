// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2026 Osilvfe <osilvfe@gmail.com>
/*
 * ams OSRAM TCS3701 ALS/color and proximity sensor
 *
 * Optional direct-I2C IIO driver for Linux v7.2 on Caihong.
 * The downstream Qualcomm sensor stack owns this device through SSC; this
 * driver intentionally exposes the physical chip directly through standard
 * Linux IIO instead of importing the vendor sensor framework.
 *
 * Datasheet: TCS3701-DS000624, v2-00, 2022-12-15.
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/iio/iio.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/regmap.h>
#include <linux/unaligned.h>

#define TCS3701_REG_ENABLE		0x80
#define TCS3701_REG_ATIME		0x81
#define TCS3701_REG_PTIME		0x82
#define TCS3701_REG_ID			0x92
#define TCS3701_REG_ASTATUS		0x94
#define TCS3701_REG_ADATA0_L		0x95
#define TCS3701_REG_ADATA1_L		0x97
#define TCS3701_REG_ADATA2_L		0x99
#define TCS3701_REG_ADATA3_L		0x9b
#define TCS3701_REG_PDATA_L		0xa1
#define TCS3701_REG_STATUS2		0xa3
#define TCS3701_REG_STATUS6		0xa7
#define TCS3701_REG_CFG1		0xaa
#define TCS3701_REG_PCFG2		0xb9
#define TCS3701_REG_PCFG4		0xbb
#define TCS3701_REG_PCFG5		0xbc
#define TCS3701_REG_ASTEP_L		0xca
#define TCS3701_REG_ASTEP_H		0xcb

#define TCS3701_ENABLE_PEN		BIT(2)
#define TCS3701_ENABLE_AEN		BIT(1)
#define TCS3701_ENABLE_PON		BIT(0)

#define TCS3701_STATUS2_AVALID		BIT(6)
#define TCS3701_STATUS2_PVALID		BIT(5)
#define TCS3701_STATUS6_INIT_BUSY	BIT(0)

#define TCS3701_CFG1_AGAIN_MASK		GENMASK(4, 0)
#define TCS3701_PCFG2_PLDRIVE0_MASK	GENMASK(6, 0)
#define TCS3701_PCFG4_PGAIN_MASK	GENMASK(1, 0)
#define TCS3701_PCFG5_PPULSE_LEN_MASK	GENMASK(7, 6)
#define TCS3701_PCFG5_PPULSE_MASK	GENMASK(5, 0)

#define TCS3701_ID_VALUE		0x18

/* Datasheet recommended ALS setup: ASTEP=599, ATIME=29 => 50 ms. */
#define TCS3701_ASTEP_VALUE		599
#define TCS3701_ATIME_VALUE		29
#define TCS3701_ALS_INT_TIME_US	50000
#define TCS3701_ALS_TIMEOUT_US		120000
#define TCS3701_PROX_TIMEOUT_US	20000

/* Datasheet characterization setup: 4 mA, 4x, 8 us, 8 pulses. */
#define TCS3701_PROX_LED_4MA		0
#define TCS3701_PROX_GAIN_4X		2
#define TCS3701_PROX_PULSE_8US		1
#define TCS3701_PROX_PULSES_8		7

struct tcs3701_data {
	struct regmap *regmap;
	struct mutex lock;
};

struct tcs3701_gain {
	int val;
	int val2;
	u8 code;
};

static const struct tcs3701_gain tcs3701_als_gains[] = {
	{ 0, 500000, 0 },
	{ 1,      0, 1 },
	{ 2,      0, 2 },
	{ 4,      0, 3 },
	{ 8,      0, 4 },
	{ 16,     0, 5 },
	{ 32,     0, 6 },
	{ 64,     0, 7 },
	{ 128,    0, 8 },
	{ 256,    0, 9 },
	{ 512,    0, 10 },
	{ 1024,   0, 11 },
};

#define TCS3701_INTENSITY_CHANNEL(_modifier, _address) { \
	.type = IIO_INTENSITY, \
	.modified = 1, \
	.channel2 = (_modifier), \
	.address = (_address), \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW), \
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_CALIBSCALE) | \
				    BIT(IIO_CHAN_INFO_INT_TIME), \
}

static const struct iio_chan_spec tcs3701_channels[] = {
	TCS3701_INTENSITY_CHANNEL(IIO_MOD_LIGHT_CLEAR, TCS3701_REG_ADATA0_L),
	TCS3701_INTENSITY_CHANNEL(IIO_MOD_LIGHT_RED, TCS3701_REG_ADATA1_L),
	TCS3701_INTENSITY_CHANNEL(IIO_MOD_LIGHT_GREEN, TCS3701_REG_ADATA2_L),
	TCS3701_INTENSITY_CHANNEL(IIO_MOD_LIGHT_BLUE, TCS3701_REG_ADATA3_L),
	{
		.type = IIO_PROXIMITY,
		.address = TCS3701_REG_PDATA_L,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
	},
};

static const struct regmap_config tcs3701_regmap_config = {
	.name = "tcs3701",
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xfa,
	.cache_type = REGCACHE_NONE,
};

static int tcs3701_wait_status(struct tcs3701_data *data, unsigned int mask,
			       unsigned int timeout_us)
{
	unsigned int status;

	return regmap_read_poll_timeout(data->regmap, TCS3701_REG_STATUS2,
					status, status & mask, 1000, timeout_us);
}

static int tcs3701_start_function(struct tcs3701_data *data, unsigned int enable)
{
	int ret;

	/* Datasheet requires PON before AEN/PEN. */
	ret = regmap_write(data->regmap, TCS3701_REG_ENABLE,
			   TCS3701_ENABLE_PON);
	if (ret)
		return ret;

	/* INIT_BUSY lasts roughly 300 us after power-on. */
	usleep_range(500, 1000);

	return regmap_write(data->regmap, TCS3701_REG_ENABLE,
			    TCS3701_ENABLE_PON | enable);
}

static int tcs3701_read_als(struct tcs3701_data *data, unsigned int reg,
			    int *val)
{
	u8 raw[2];
	unsigned int astatus;
	int ret, stop_ret;

	ret = tcs3701_start_function(data, TCS3701_ENABLE_AEN);
	if (ret)
		return ret;

	ret = tcs3701_wait_status(data, TCS3701_STATUS2_AVALID,
				  TCS3701_ALS_TIMEOUT_US);
	if (ret)
		goto stop;

	/* Reading ASTATUS latches all ALS data bytes to one conversion. */
	ret = regmap_read(data->regmap, TCS3701_REG_ASTATUS, &astatus);
	if (ret)
		goto stop;

	ret = regmap_bulk_read(data->regmap, reg, raw, sizeof(raw));
	if (!ret)
		*val = get_unaligned_le16(raw);

stop:
	stop_ret = regmap_write(data->regmap, TCS3701_REG_ENABLE, 0);
	if (!ret)
		ret = stop_ret;

	return ret;
}

static int tcs3701_read_proximity(struct tcs3701_data *data, int *val)
{
	u8 raw[2];
	int ret, stop_ret;

	ret = tcs3701_start_function(data, TCS3701_ENABLE_PEN);
	if (ret)
		return ret;

	ret = tcs3701_wait_status(data, TCS3701_STATUS2_PVALID,
				  TCS3701_PROX_TIMEOUT_US);
	if (ret)
		goto stop;

	/* PDATA is 14-bit; reading the low byte first latches the high byte. */
	ret = regmap_bulk_read(data->regmap, TCS3701_REG_PDATA_L,
			       raw, sizeof(raw));
	if (!ret)
		*val = get_unaligned_le16(raw) & GENMASK(13, 0);

stop:
	stop_ret = regmap_write(data->regmap, TCS3701_REG_ENABLE, 0);
	if (!ret)
		ret = stop_ret;

	return ret;
}

static int tcs3701_read_gain(struct tcs3701_data *data, int *val, int *val2)
{
	unsigned int reg;
	unsigned int code;
	int ret;

	ret = regmap_read(data->regmap, TCS3701_REG_CFG1, &reg);
	if (ret)
		return ret;

	code = FIELD_GET(TCS3701_CFG1_AGAIN_MASK, reg);
	if (code >= ARRAY_SIZE(tcs3701_als_gains))
		return -EINVAL;

	*val = tcs3701_als_gains[code].val;
	*val2 = tcs3701_als_gains[code].val2;

	return IIO_VAL_INT_PLUS_MICRO;
}

static int tcs3701_read_raw(struct iio_dev *indio_dev,
			    const struct iio_chan_spec *chan,
			    int *val, int *val2, long mask)
{
	struct tcs3701_data *data = iio_priv(indio_dev);
	int ret;

	mutex_lock(&data->lock);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		if (chan->type == IIO_INTENSITY)
			ret = tcs3701_read_als(data, chan->address, val);
		else if (chan->type == IIO_PROXIMITY)
			ret = tcs3701_read_proximity(data, val);
		else
			ret = -EINVAL;

		if (!ret)
			ret = IIO_VAL_INT;
		break;

	case IIO_CHAN_INFO_CALIBSCALE:
		if (chan->type != IIO_INTENSITY) {
			ret = -EINVAL;
			break;
		}
		ret = tcs3701_read_gain(data, val, val2);
		break;

	case IIO_CHAN_INFO_INT_TIME:
		if (chan->type != IIO_INTENSITY) {
			ret = -EINVAL;
			break;
		}
		*val = 0;
		*val2 = TCS3701_ALS_INT_TIME_US;
		ret = IIO_VAL_INT_PLUS_MICRO;
		break;

	default:
		ret = -EINVAL;
		break;
	}

	mutex_unlock(&data->lock);
	return ret;
}

static int tcs3701_write_raw(struct iio_dev *indio_dev,
			     const struct iio_chan_spec *chan,
			     int val, int val2, long mask)
{
	struct tcs3701_data *data = iio_priv(indio_dev);
	int ret = -EINVAL;
	unsigned int i;

	if (mask != IIO_CHAN_INFO_CALIBSCALE || chan->type != IIO_INTENSITY)
		return -EINVAL;

	mutex_lock(&data->lock);

	for (i = 0; i < ARRAY_SIZE(tcs3701_als_gains); i++) {
		if (val != tcs3701_als_gains[i].val ||
		    val2 != tcs3701_als_gains[i].val2)
			continue;

		ret = regmap_update_bits(data->regmap, TCS3701_REG_CFG1,
					 TCS3701_CFG1_AGAIN_MASK,
					 tcs3701_als_gains[i].code);
		break;
	}

	mutex_unlock(&data->lock);
	return ret;
}

static int tcs3701_write_raw_get_fmt(struct iio_dev *indio_dev,
				      const struct iio_chan_spec *chan,
				      long mask)
{
	if (mask == IIO_CHAN_INFO_CALIBSCALE && chan->type == IIO_INTENSITY)
		return IIO_VAL_INT_PLUS_MICRO;

	return -EINVAL;
}

static const struct iio_info tcs3701_info = {
	.read_raw = tcs3701_read_raw,
	.write_raw = tcs3701_write_raw,
	.write_raw_get_fmt = tcs3701_write_raw_get_fmt,
};

static int tcs3701_chip_init(struct tcs3701_data *data)
{
	unsigned int id, status6;
	int ret;

	/* Keep all engines off while changing configuration. */
	ret = regmap_write(data->regmap, TCS3701_REG_ENABLE, 0);
	if (ret)
		return ret;

	ret = regmap_read_poll_timeout(data->regmap, TCS3701_REG_STATUS6,
					status6,
					!(status6 & TCS3701_STATUS6_INIT_BUSY),
					100, 5000);
	if (ret)
		return ret;

	ret = regmap_read(data->regmap, TCS3701_REG_ID, &id);
	if (ret)
		return ret;
	if (id != TCS3701_ID_VALUE)
		return -ENODEV;

	ret = regmap_write(data->regmap, TCS3701_REG_ATIME,
			   TCS3701_ATIME_VALUE);
	if (ret)
		return ret;

	ret = regmap_write(data->regmap, TCS3701_REG_ASTEP_L,
			   TCS3701_ASTEP_VALUE & 0xff);
	if (ret)
		return ret;

	ret = regmap_write(data->regmap, TCS3701_REG_ASTEP_H,
			   TCS3701_ASTEP_VALUE >> 8);
	if (ret)
		return ret;

	/* Start ALS at the datasheet reset gain of 256x. */
	ret = regmap_update_bits(data->regmap, TCS3701_REG_CFG1,
				 TCS3701_CFG1_AGAIN_MASK, 9);
	if (ret)
		return ret;

	ret = regmap_write(data->regmap, TCS3701_REG_PTIME, 0);
	if (ret)
		return ret;

	ret = regmap_update_bits(data->regmap, TCS3701_REG_PCFG2,
				 TCS3701_PCFG2_PLDRIVE0_MASK,
				 TCS3701_PROX_LED_4MA);
	if (ret)
		return ret;

	ret = regmap_update_bits(data->regmap, TCS3701_REG_PCFG4,
				 TCS3701_PCFG4_PGAIN_MASK,
				 TCS3701_PROX_GAIN_4X);
	if (ret)
		return ret;

	return regmap_write(data->regmap, TCS3701_REG_PCFG5,
			    FIELD_PREP(TCS3701_PCFG5_PPULSE_LEN_MASK,
				       TCS3701_PROX_PULSE_8US) |
			    FIELD_PREP(TCS3701_PCFG5_PPULSE_MASK,
				       TCS3701_PROX_PULSES_8));
}

static int tcs3701_probe(struct i2c_client *client)
{
	struct tcs3701_data *data;
	struct iio_dev *indio_dev;
	struct regmap *regmap;
	int ret;

	regmap = devm_regmap_init_i2c(client, &tcs3701_regmap_config);
	if (IS_ERR(regmap))
		return dev_err_probe(&client->dev, PTR_ERR(regmap),
				     "regmap init failed\n");

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	data = iio_priv(indio_dev);
	data->regmap = regmap;
	mutex_init(&data->lock);

	ret = tcs3701_chip_init(data);
	if (ret)
		return dev_err_probe(&client->dev, ret,
				     "TCS3701 initialization failed\n");

	indio_dev->name = "tcs3701";
	indio_dev->info = &tcs3701_info;
	indio_dev->channels = tcs3701_channels;
	indio_dev->num_channels = ARRAY_SIZE(tcs3701_channels);
	indio_dev->modes = INDIO_DIRECT_MODE;

	return devm_iio_device_register(&client->dev, indio_dev);
}

static const struct of_device_id tcs3701_of_match[] = {
	{ .compatible = "ams,tcs3701" },
	{ }
};
MODULE_DEVICE_TABLE(of, tcs3701_of_match);

static const struct i2c_device_id tcs3701_i2c_ids[] = {
	{ "tcs3701" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tcs3701_i2c_ids);

static struct i2c_driver tcs3701_driver = {
	.driver = {
		.name = "tcs3701",
		.of_match_table = tcs3701_of_match,
	},
	.probe = tcs3701_probe,
	.id_table = tcs3701_i2c_ids,
};
module_i2c_driver(tcs3701_driver);

MODULE_AUTHOR("Osilvfe");
MODULE_DESCRIPTION("ams OSRAM TCS3701 ALS/color and proximity IIO driver");
MODULE_LICENSE("GPL");
