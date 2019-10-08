// SPDX-License-Identifier: GPL-2.0-only
/*
 * bma400_core.c - Core IIO driver for Bosch BMA400 triaxial acceleration
 *                 sensor. Used by bma400-i2c.
 *
 * Copyright 2019 Dan Robertson <dan@dlrobertson.com>
 *
 * TODO:
 *  - Support for power management
 *  - Support events and interrupts
 *  - Create channel the step count
 *  - Create channel for sensor time
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/bitops.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

#include "bma400.h"

/*
 * The G-range selection may be one of 2g, 4g, 8, or 16g. The scale may
 * be selected with the acc_range bits of the ACC_CONFIG1 register.
 */
static const int bma400_scale_table[] = {
	0, 38344,
	0, 76590,
	0, 153277,
	0, 306457
};

static const int bma400_osr_table[] = { 0, 1, 3 };

/* See the ACC_CONFIG1 section of the datasheet */
static const int bma400_sample_freqs[] = {
	12,  500000,
	25,  0,
	50,  0,
	100, 0,
	200, 0,
	400, 0,
	800, 0,
};

/* See the ACC_CONFIG0 section of the datasheet */
enum bma400_power_mode {
	POWER_MODE_SLEEP   = 0x00,
	POWER_MODE_LOW     = 0x01,
	POWER_MODE_NORMAL  = 0x02,
	POWER_MODE_INVALID = 0x03,
};

struct bma400_data {
	struct device *dev;
	struct mutex mutex; /* data register lock */
	struct iio_mount_matrix orientation;
	struct regmap *regmap;
	enum bma400_power_mode power_mode;
	const int *sample_freq;
	int oversampling_ratio;
	int scale;
};

static bool bma400_is_writable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case BMA400_CHIP_ID_REG:
	case BMA400_ERR_REG:
	case BMA400_STATUS_REG:
	case BMA400_X_AXIS_LSB_REG:
	case BMA400_X_AXIS_MSB_REG:
	case BMA400_Y_AXIS_LSB_REG:
	case BMA400_Y_AXIS_MSB_REG:
	case BMA400_Z_AXIS_LSB_REG:
	case BMA400_Z_AXIS_MSB_REG:
	case BMA400_SENSOR_TIME0:
	case BMA400_SENSOR_TIME1:
	case BMA400_SENSOR_TIME2:
	case BMA400_EVENT_REG:
	case BMA400_INT_STAT0_REG:
	case BMA400_INT_STAT1_REG:
	case BMA400_INT_STAT2_REG:
	case BMA400_TEMP_DATA_REG:
	case BMA400_FIFO_LENGTH0_REG:
	case BMA400_FIFO_LENGTH1_REG:
	case BMA400_FIFO_DATA_REG:
	case BMA400_STEP_CNT0_REG:
	case BMA400_STEP_CNT1_REG:
	case BMA400_STEP_CNT3_REG:
	case BMA400_STEP_STAT_REG:
		return false;
	default:
		return true;
	}
}

static bool bma400_is_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case BMA400_ERR_REG:
	case BMA400_STATUS_REG:
	case BMA400_X_AXIS_LSB_REG:
	case BMA400_X_AXIS_MSB_REG:
	case BMA400_Y_AXIS_LSB_REG:
	case BMA400_Y_AXIS_MSB_REG:
	case BMA400_Z_AXIS_LSB_REG:
	case BMA400_Z_AXIS_MSB_REG:
	case BMA400_SENSOR_TIME0:
	case BMA400_SENSOR_TIME1:
	case BMA400_SENSOR_TIME2:
	case BMA400_EVENT_REG:
	case BMA400_INT_STAT0_REG:
	case BMA400_INT_STAT1_REG:
	case BMA400_INT_STAT2_REG:
	case BMA400_TEMP_DATA_REG:
	case BMA400_FIFO_LENGTH0_REG:
	case BMA400_FIFO_LENGTH1_REG:
	case BMA400_FIFO_DATA_REG:
	case BMA400_STEP_CNT0_REG:
	case BMA400_STEP_CNT1_REG:
	case BMA400_STEP_CNT3_REG:
	case BMA400_STEP_STAT_REG:
		return true;
	default:
		return false;
	}
}

const struct regmap_config bma400_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = BMA400_CMD_REG,
	.cache_type = REGCACHE_RBTREE,
	.writeable_reg = bma400_is_writable_reg,
	.volatile_reg = bma400_is_volatile_reg,
};
EXPORT_SYMBOL(bma400_regmap_config);

static const struct iio_mount_matrix *
bma400_accel_get_mount_matrix(const struct iio_dev *indio_dev,
			      const struct iio_chan_spec *chan)
{
	struct bma400_data *data = iio_priv(indio_dev);

	return &data->orientation;
}

static const struct iio_chan_spec_ext_info bma400_ext_info[] = {
	IIO_MOUNT_MATRIX(IIO_SHARED_BY_DIR, bma400_accel_get_mount_matrix),
	{ }
};

#define BMA400_ACC_CHANNEL(_axis) { \
	.type = IIO_ACCEL, \
	.modified = 1, \
	.channel2 = IIO_MOD_##_axis, \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW), \
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ) | \
		BIT(IIO_CHAN_INFO_SCALE) | \
		BIT(IIO_CHAN_INFO_OVERSAMPLING_RATIO), \
	.info_mask_shared_by_type_available = BIT(IIO_CHAN_INFO_SAMP_FREQ) | \
		BIT(IIO_CHAN_INFO_SCALE) | \
		BIT(IIO_CHAN_INFO_OVERSAMPLING_RATIO), \
	.ext_info = bma400_ext_info, \
}

static const struct iio_chan_spec bma400_channels[] = {
	BMA400_ACC_CHANNEL(X),
	BMA400_ACC_CHANNEL(Y),
	BMA400_ACC_CHANNEL(Z),
	{
		.type = IIO_TEMP,
		.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ),
	},
};

static int bma400_get_temp_reg(struct bma400_data *data, int *val, int *val2)
{
	int ret;
	int host_temp;
	unsigned int raw_temp;

	if (data->power_mode == POWER_MODE_SLEEP)
		return -EBUSY;

	ret = regmap_read(data->regmap, BMA400_TEMP_DATA_REG, &raw_temp);

	if (ret < 0)
		return ret;

	host_temp = sign_extend32(raw_temp, 7);
	/*
	 * The formula for the TEMP_DATA register in the datasheet
	 * is: x * 0.5 + 23
	 */
	*val = (host_temp >> 1) + 23;
	*val2 = (host_temp & 0x1) * 500000;
	return IIO_VAL_INT_PLUS_MICRO;
}

static int bma400_get_accel_reg(struct bma400_data *data,
				const struct iio_chan_spec *chan,
				int *val)
{
	int ret;
	int lsb_reg;
	__le16 raw_accel;

	if (data->power_mode == POWER_MODE_SLEEP)
		return -EBUSY;

	switch (chan->channel2) {
	case IIO_MOD_X:
		lsb_reg = BMA400_X_AXIS_LSB_REG;
		break;
	case IIO_MOD_Y:
		lsb_reg = BMA400_Y_AXIS_LSB_REG;
		break;
	case IIO_MOD_Z:
		lsb_reg = BMA400_Z_AXIS_LSB_REG;
		break;
	default:
		dev_err(data->dev, "invalid axis channel modifier");
		return -EINVAL;
	}

	/* bulk read two registers, with the base being the LSB register */
	ret = regmap_bulk_read(data->regmap, lsb_reg, &raw_accel,
			       sizeof(raw_accel));
	if (ret < 0)
		return ret;

	*val = sign_extend32(le16_to_cpu(raw_accel), 11);
	return IIO_VAL_INT;
}

static int bma400_ready_for_cmd(struct bma400_data *data)
{
	unsigned int val;
	int ret = regmap_read(data->regmap, BMA400_STATUS_REG, &val);

	if (ret < 0)
		return 0;

	return (val & BMA400_CMD_RDY_MASK) ? 1 : 0;
}

static int bma400_softreset(struct bma400_data *data)
{
	int ret;

	if (!bma400_ready_for_cmd(data))
		return -EAGAIN;

	ret = regmap_write(data->regmap, BMA400_CMD_REG,
			   BMA400_SOFTRESET_CMD);
	if (ret < 0)
		return ret;

	/* a softreset restores registers to their defaults */
	data->power_mode = POWER_MODE_SLEEP;
	data->sample_freq = NULL;
	data->oversampling_ratio = -1;
	data->scale = bma400_scale_table[1];
	return 0;
}

static int bma400_get_accel_output_data_rate(struct bma400_data *data)
{
	int ret;
	unsigned int val;
	unsigned int odr;
	int idx;

	switch (data->power_mode) {
	case POWER_MODE_LOW:
		/*
		 * Runs at a fixed rate in low-power mode. See section 4.3
		 * in the datasheet.
		 */
		data->sample_freq = &bma400_sample_freqs[2];
		return 0;
	case POWER_MODE_NORMAL:
		/*
		 * In normal mode the ODR can be found in the ACC_CONFIG1
		 * register.
		 */
		ret = regmap_read(data->regmap, BMA400_ACC_CONFIG1_REG, &val);
		if (ret < 0) {
			data->sample_freq = NULL;
			return ret;
		}

		odr = (val & BMA400_ACC_ODR_MASK);
		if (odr < 0x05) {
			dev_err(data->dev, "invalid ODR=%x", odr);
			data->sample_freq = NULL;
			return -EINVAL;
		}

		idx = (odr - 0x05) * 2;

		if (idx + 1 >= ARRAY_SIZE(bma400_sample_freqs)) {
			dev_err(data->dev, "sample freq index is too high");
			return -EINVAL;
		}

		data->sample_freq = &bma400_sample_freqs[idx];
		return 0;
	default:
		data->sample_freq = NULL;
		return 0;
	}
}

static int bma400_get_accel_output_data_rate_idx(struct bma400_data *data,
						 int hz, int uhz)
{
	int i;
	for (i = 0; i + 1 < ARRAY_SIZE(bma400_sample_freqs); i += 2) {
		if (bma400_sample_freqs[i] == hz &&
		    bma400_sample_freqs[i + 1] == uhz)
			return i / 2;
	}

	return -EINVAL;
}

static int bma400_set_accel_output_data_rate(struct bma400_data *data,
					     int hz, int uhz)
{
	int ret;
	unsigned int odr;
	unsigned int val;
	int idx;

	idx = bma400_get_accel_output_data_rate_idx(data, hz, uhz);

	if (idx < 0)
		return idx;

	ret = regmap_read(data->regmap, BMA400_ACC_CONFIG1_REG, &val);

	if (ret < 0)
		return ret;

	/* preserve the range and normal mode osr */
	odr = (~BMA400_ACC_ODR_MASK & val) | (idx + 0x5);

	ret = regmap_write(data->regmap, BMA400_ACC_CONFIG1_REG, odr);
	if (ret < 0)
		return ret;

	data->sample_freq = &bma400_sample_freqs[idx * 2];
	return 0;

}

static int bma400_get_accel_oversampling_ratio(struct bma400_data *data)
{
	unsigned int val;
	int ret;

	/*
	 * The oversampling ratio is stored in a different register
	 * based on the power-mode. In normal mode the OSR is stored
	 * in ACC_CONFIG1. In low-power mode it is stored in
	 * ACC_CONFIG0.
	 */
	switch (data->power_mode) {
	case POWER_MODE_LOW:
		ret = regmap_read(data->regmap, BMA400_ACC_CONFIG0_REG, &val);
		if (ret < 0) {
			data->oversampling_ratio = -1;
			return ret;
		}

		data->oversampling_ratio = (val & BMA400_LP_OSR_MASK) >> 5;
		return 0;
	case POWER_MODE_NORMAL:
		ret = regmap_read(data->regmap, BMA400_ACC_CONFIG1_REG, &val);
		if (ret < 0) {
			data->oversampling_ratio = -1;
			return ret;
		}

		data->oversampling_ratio = (val & BMA400_NP_OSR_MASK) >> 4;
		return 0;
	default:
		data->oversampling_ratio = -1;
		return 0;
	}
}

static int bma400_set_accel_oversampling_ratio(struct bma400_data *data,
					       int val)
{
	int ret;
	unsigned int acc_config;

	if (val & ~BMA400_TWO_BITS_MASK)
		return -EINVAL;

	/*
	 * The oversampling ratio is stored in a different register
	 * based on the power-mode.
	 */
	switch (data->power_mode) {
	case POWER_MODE_LOW:
		ret = regmap_read(data->regmap, BMA400_ACC_CONFIG0_REG,
				  &acc_config);
		if (acc_config < 0)
			return acc_config;

		ret = regmap_write(data->regmap, BMA400_ACC_CONFIG0_REG,
				   (acc_config & ~BMA400_LP_OSR_MASK) |
				   (val << 5));
		if (ret < 0) {
			dev_err(data->dev, "Failed to write out OSR");
			return ret;
		}

		data->oversampling_ratio = val;
		return 0;
	case POWER_MODE_NORMAL:
		ret = regmap_read(data->regmap, BMA400_ACC_CONFIG1_REG,
				  &acc_config);
		if (ret < 0)
			return ret;

		ret = regmap_write(data->regmap, BMA400_ACC_CONFIG1_REG,
				   (acc_config & ~BMA400_NP_OSR_MASK) |
				   (val << 4));
		if (ret < 0) {
			dev_err(data->dev, "Failed to write out OSR");
			return ret;
		}

		data->oversampling_ratio = val;
		return 0;
	default:
		return -EINVAL;
	}
	return ret;
}

static int bma400_get_accel_scale(struct bma400_data *data)
{
	int idx;
	int ret;
	unsigned int val;

	ret = regmap_read(data->regmap, BMA400_ACC_CONFIG1_REG, &val);
	if (ret < 0)
		return ret;

	idx = (((val & BMA400_ACC_RANGE_MASK) >> 6) * 2) + 1;
	if (idx >= ARRAY_SIZE(bma400_scale_table))
		return -EINVAL;

	data->scale = bma400_scale_table[idx];

	return 0;
}

static int bma400_get_accel_scale_idx(struct bma400_data *data, int val)
{
	int i;

	for (i = 1; i < ARRAY_SIZE(bma400_scale_table); i += 2) {
		if (bma400_scale_table[i] == val)
			return (i - 1) / 2;
	}
	return -EINVAL;
}

static int bma400_set_accel_scale(struct bma400_data *data, unsigned int val)
{
	int ret;
	int idx;
	unsigned int acc_config;

	ret = regmap_read(data->regmap, BMA400_ACC_CONFIG1_REG, &acc_config);
	if (ret < 0)
		return ret;

	idx = bma400_get_accel_scale_idx(data, val);

	if (idx < 0)
		return idx;

	ret = regmap_write(data->regmap, BMA400_ACC_CONFIG1_REG,
			   (acc_config & ~BMA400_ACC_RANGE_MASK) | (idx << 6));
	if (ret < 0)
		return ret;

	data->scale = val;
	return 0;
}

static int bma400_get_power_mode(struct bma400_data *data)
{
	int ret;
	unsigned int val;

	ret = regmap_read(data->regmap, BMA400_STATUS_REG, &val);
	if (ret < 0) {
		dev_err(data->dev, "Failed to read status register");
		return ret;
	}

	data->power_mode = (val >> 1) & BMA400_TWO_BITS_MASK;

	return 0;
}

static int bma400_set_power_mode(struct bma400_data *data,
				 enum bma400_power_mode mode)
{
	int ret;
	unsigned int val;

	ret = regmap_read(data->regmap, BMA400_ACC_CONFIG0_REG, &val);
	if (ret < 0)
		return ret;

	if (data->power_mode == mode)
		return 0;

	if (mode == POWER_MODE_INVALID)
		return -EINVAL;

	/* Preserve the low-power oversample ratio etc */
	ret = regmap_write(data->regmap, BMA400_ACC_CONFIG0_REG,
			   mode | (val & ~BMA400_TWO_BITS_MASK));

	if (ret < 0) {
		dev_err(data->dev, "Failed to write to power-mode");
		return ret;
	}

	data->power_mode = mode;

	/*
	 * Update our cached osr and odr based on the new
	 * power-mode.
	 */
	bma400_get_accel_output_data_rate(data);
	bma400_get_accel_oversampling_ratio(data);

	return 0;
}

static int bma400_init(struct bma400_data *data)
{
	int ret;
	unsigned int val;

	/* Try to read chip_id register. It must return 0x90. */
	ret = regmap_read(data->regmap, BMA400_CHIP_ID_REG, &val);

	if (ret < 0) {
		dev_err(data->dev, "Failed to read chip id register: %x!", ret);
		return ret;
	} else if (val != BMA400_ID_REG_VAL) {
		dev_err(data->dev, "CHIP ID MISMATCH: %x!", ret);
		return -ENODEV;
	}

	ret = bma400_get_power_mode(data);
	if (ret < 0) {
		dev_err(data->dev, "Failed to get the initial power-mode!");
		return ret;
	}

	if (data->power_mode != POWER_MODE_NORMAL) {
		ret = bma400_set_power_mode(data, POWER_MODE_NORMAL);
		if (ret < 0) {
			dev_err(data->dev, "Failed to wake up the device!");
			return ret;
		}
		/*
		 * TODO: The datasheet waits 1500us here in the example, but
		 * lists 2/ODR as the wakeup time.
		 */
		usleep_range(1500, 20000);
	}

	ret = bma400_get_accel_output_data_rate(data);
	if (ret < 0)
		return ret;

	ret = bma400_get_accel_oversampling_ratio(data);
	if (ret < 0)
		return ret;

	ret = bma400_get_accel_scale(data);
	if (ret < 0)
		return ret;

	/*
	 * Once the interrupt engine is supported we might use the
	 * data_src_reg, but for now ensure this is set to the
	 * variable ODR filter selectable by the sample frequency
	 * channel.
	 */
	return regmap_write(data->regmap, BMA400_ACC_CONFIG2_REG, 0x00);
}

static struct attribute *bma400_attributes[] = {
	NULL,
};

static const struct attribute_group bma400_attrs_group = {
	.attrs = bma400_attributes,
};

static int bma400_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan, int *val,
			   int *val2, long mask)
{
	struct bma400_data *data = iio_priv(indio_dev);
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_PROCESSED:
		mutex_lock(&data->mutex);
		ret = bma400_get_temp_reg(data, val, val2);
		mutex_unlock(&data->mutex);
		return ret;
	case IIO_CHAN_INFO_RAW:
		mutex_lock(&data->mutex);
		ret = bma400_get_accel_reg(data, chan, val);
		mutex_unlock(&data->mutex);
		return ret;
	case IIO_CHAN_INFO_SAMP_FREQ:
		switch (chan->type) {
		case IIO_ACCEL:
			if (!data->sample_freq)
				return -EINVAL;

			*val = data->sample_freq[0];
			*val2 = data->sample_freq[1];
			return IIO_VAL_INT_PLUS_MICRO;
		case IIO_TEMP:
			/*
			 * Runs at a fixed sampling frequency. See Section 4.4
			 * of the datasheet.
			 */
			*val = 6;
			*val2 = 250000;
			return IIO_VAL_INT_PLUS_MICRO;
		default:
			return -EINVAL;
		}
	case IIO_CHAN_INFO_SCALE:
		*val = 0;
		*val2 = data->scale;
		return IIO_VAL_INT_PLUS_MICRO;
	case IIO_CHAN_INFO_OVERSAMPLING_RATIO:
		/*
		 * TODO: We could avoid this logic and returning -EINVAL here if
		 * we set both the low-power and normal mode OSR registers when
		 * we configure the device.
		 */
		if (data->oversampling_ratio < 0)
			return -EINVAL;

		*val = data->oversampling_ratio;
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static int bma400_read_avail(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     const int **vals, int *type, int *length,
			     long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		*type = IIO_VAL_INT_PLUS_MICRO;
		*vals = bma400_scale_table;
		*length = ARRAY_SIZE(bma400_scale_table);
		return IIO_AVAIL_LIST;
	case IIO_CHAN_INFO_OVERSAMPLING_RATIO:
		*type = IIO_VAL_INT;
		*vals = bma400_osr_table;
		*length = ARRAY_SIZE(bma400_osr_table);
		return IIO_AVAIL_RANGE;
	case IIO_CHAN_INFO_SAMP_FREQ:
		*type = IIO_VAL_INT_PLUS_MICRO;
		*vals = bma400_sample_freqs;
		*length = ARRAY_SIZE(bma400_sample_freqs);
		return IIO_AVAIL_LIST;
	default:
		return -EINVAL;
	}
}

static int bma400_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan, int val, int val2,
			    long mask)
{
	int ret;
	struct bma400_data *data = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		/*
		 * The sample frequency is readonly for the temperature
		 * register and a fixed value in low-power mode.
		 */
		if (chan->type != IIO_ACCEL)
			return -EINVAL;

		mutex_lock(&data->mutex);
		ret = bma400_set_accel_output_data_rate(data, val, val2);
		mutex_unlock(&data->mutex);
		return ret;
	case IIO_CHAN_INFO_SCALE:
		if (val != 0)
			return -EINVAL;

		mutex_lock(&data->mutex);
		ret = bma400_set_accel_scale(data, val2);
		mutex_unlock(&data->mutex);
		return ret;
	case IIO_CHAN_INFO_OVERSAMPLING_RATIO:
		mutex_lock(&data->mutex);
		ret = bma400_set_accel_oversampling_ratio(data, val);
		mutex_unlock(&data->mutex);
		return ret;
	default:
		return -EINVAL;
	}
}

static int bma400_write_raw_get_fmt(struct iio_dev *indio_dev,
				    struct iio_chan_spec const *chan,
				    long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		return IIO_VAL_INT_PLUS_MICRO;
	case IIO_CHAN_INFO_SCALE:
		return IIO_VAL_INT_PLUS_MICRO;
	case IIO_CHAN_INFO_OVERSAMPLING_RATIO:
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static const struct iio_info bma400_info = {
	.attrs             = &bma400_attrs_group,
	.read_raw          = bma400_read_raw,
	.read_avail        = bma400_read_avail,
	.write_raw         = bma400_write_raw,
	.write_raw_get_fmt = bma400_write_raw_get_fmt,
};

int bma400_probe(struct device *dev,
		 struct regmap *regmap,
		 const char *name)
{
	int ret;
	struct bma400_data *data;
	struct iio_dev *indio_dev;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	data = iio_priv(indio_dev);
	data->regmap = regmap;
	data->dev = dev;

	ret = bma400_init(data);
	if (ret < 0)
		return ret;

	ret = iio_read_mount_matrix(dev, "mount-matrix", &data->orientation);
	if (ret)
		return ret;

	mutex_init(&data->mutex);
	indio_dev->dev.parent = dev;
	indio_dev->name = name;
	indio_dev->info = &bma400_info;
	indio_dev->channels = bma400_channels;
	indio_dev->num_channels = ARRAY_SIZE(bma400_channels);
	indio_dev->modes = INDIO_DIRECT_MODE;

	dev_set_drvdata(dev, indio_dev);

	return iio_device_register(indio_dev);
}
EXPORT_SYMBOL(bma400_probe);

int bma400_remove(struct device *dev)
{
	int ret;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct bma400_data *data = iio_priv(indio_dev);

	mutex_lock(&data->mutex);
	ret = bma400_softreset(data);
	if (ret < 0) {
		/*
		 * If the softreset failed, try to put the device in
		 * sleep mode, but still report the error.
		 */
		dev_err(data->dev, "Failed to reset the device");
		bma400_set_power_mode(data, POWER_MODE_SLEEP);
	}
	mutex_unlock(&data->mutex);

	iio_device_unregister(indio_dev);

	return ret;
}
EXPORT_SYMBOL(bma400_remove);

MODULE_AUTHOR("Dan Robertson <dan@dlrobertson.com>");
MODULE_DESCRIPTION("Bosch BMA400 triaxial acceleration sensor");
MODULE_LICENSE("GPL");
