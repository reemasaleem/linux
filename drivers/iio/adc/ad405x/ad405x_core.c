// SPDX-License-Identifier: GPL-2.0-only
/*
 * AD405x ADC driver
 *
 * Copyright (c) 2023 Analog Devices Inc.
 *
 * Jorge Marques <jorge.marques@analog.com>
 */

#include <linux/bitfield.h>
#include <linux/module.h>
#include <linux/regmap.h>

#include <linux/i3c/ccc.h>

#include "ad405x.h"

const struct ad405x_chip_info ad405x_chip_info[] = {
	[AD4053] = {
		.name = "ad4053",
		.type = AD4053,
		.interface = AD405X_I3C,
		.soft_reset = false,
	},
	[AD4054] = {
		.name = "ad4054",
		.type = AD4054,
		.interface = AD405X_I3C,
		.soft_reset = false,
	},
	[AD4055] = {
		.name = "ad4055",
		.type = AD4055,
		.interface = AD405X_I3C,
		.soft_reset = false,
	}
};
EXPORT_SYMBOL_NS_GPL(ad405x_chip_info, IIO_AD405X);

static const struct regmap_range ad405x_i3c_readable_reg_range[] = {
	regmap_reg_range(AD405X_REG_ADC_MODES, AD405X_REG_ADC_IBI_EN),
	regmap_reg_range(AD405X_REG_FUSE_CRC, AD405X_REG_IBI_STATUS),
	regmap_reg_range(AD405X_REG_CONV_READ(0), AD405X_REG_CONV_TRIGGER(3))
};

const struct regmap_access_table ad405x_i3c_readable_regs_table = {
	.yes_ranges = ad405x_i3c_readable_reg_range,
	.n_yes_ranges = ARRAY_SIZE(ad405x_i3c_readable_reg_range),
};
EXPORT_SYMBOL_NS_GPL(ad405x_i3c_readable_regs_table, IIO_AD405X);

static const struct regmap_range ad405x_i3c_writable_reg_range[] = {
	regmap_reg_range(AD405X_REG_ADC_MODES, AD405X_REG_MIN_HYST),
	regmap_reg_range(AD405X_REG_INTERFACE_IBI_EN, AD405X_REG_ADC_IBI_EN),
	regmap_reg_range(AD405X_REG_FUSE_CRC, AD405X_REG_IBI_STATUS)
};

static int ad405x_read_reg_raw(struct ad405x_data *data){
	int ret;

	mutex_lock(&data->lock);

	switch (data->chip_info->interface) {
	case AD405X_I3C:
		ret = regmap_bulk_read(data->regmap,
			       AD405X_REG_CONV_READ(0),
			       &data->transf_buf, 4);
		break;
	default:
		return -EINVAL;
	}

	if (ret)
		goto unlock_ret;

	ret = le16_to_cpu(data->transf_buf);

unlock_ret:
	mutex_unlock(&data->lock);
	return ret;

}

static int ad405x_set_some_reg(struct ad405x_data *data, int val)
{
	return regmap_write(data->regmap,
			    AD405X_REG_TIMER_CONFIG,
			    val);
}



static int ad405x_set_samp_freq(struct ad405x_data *data, int freq)
{
	// TODO
	int ret;

	data->samp_freq = freq;

	ret = 0;
	return ret;
}

static int ad405x_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val, int *val2, long mask)
{
	struct ad405x_data *data = iio_priv(indio_dev);
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		ret = ad405x_read_reg_raw(data);
		if (ret < 0)
			return ret;

		*val = ret;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = data->samp_freq;
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static int ad405x_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int val, int val2, long mask)
{
	struct ad405x_data *data = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		return ad405x_set_some_reg(data, val);
	case IIO_CHAN_INFO_SAMP_FREQ:
		return ad405x_set_samp_freq(data, val);
	default:
		return -EINVAL;
	}
}

static const struct iio_info ad405x_info = {
	.read_raw	= ad405x_read_raw,
	.write_raw	= ad405x_write_raw
};

static const struct iio_chan_spec chan_template = {
	.type = IIO_VOLTAGE,
	.indexed = 1,
	.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_RAW),
	.scan_type.sign = 'u',
	.scan_type.storagebits = 32,
};

static const struct iio_chan_spec ad405x_channels[] = {
	chan_template,
};

static int ad405x_check_id(struct device *dev,
			    struct ad405x_data *data)
{
	// TODO

	return 0;
}

static int ad405x_setup(struct device *dev, struct ad405x_data *data,
			 int (*setup)(struct device *, struct regmap *))
{
	int ret;
	int val;

	if (data->chip_info->soft_reset) {
		switch (data->chip_info->interface) {
		// TODO
		case AD405X_I3C:
			ret = regmap_raw_read(data->regmap,
				       I3C_CCC_RSTDAA(0),
				       &val, 0);
		}

		if (ret)
			return ret;
	}

	if (setup) {
		ret = setup(dev, data->regmap);
		if (ret)
			return ret;
	}

	ret = ad405x_check_id(dev, data);
	if (ret)
		return ret;

	/* Enable ADC MODE (...) TODO */

	return ret;
}


int ad405x_core_probe(struct device *dev,
		       struct regmap *regmap,
		       const struct ad405x_chip_info *chip_info,
		       int (*setup)(struct device *, struct regmap *))
{
	struct ad405x_data *data;
	struct iio_dev *indio_dev;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	data = iio_priv(indio_dev);
	data->regmap = regmap;
	data->chip_info = chip_info;

	mutex_init(&data->lock);

	indio_dev->name = chip_info->name;
	indio_dev->info = &ad405x_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = ad405x_channels;
	indio_dev->num_channels = ARRAY_SIZE(ad405x_channels);

	ret = ad405x_setup(dev, data, setup);
	if (ret) {
		dev_err(dev, "AD405x setup failed\n");
		return ret;
	}

	return devm_iio_device_register(dev, indio_dev);
}
EXPORT_SYMBOL_NS_GPL(ad405x_core_probe, IIO_AD405X);

MODULE_AUTHOR("Jorge Marques <jorge.marques@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD4050, AD4051, AD4052, AD4053, AD4054, AD4055 ADC");
MODULE_LICENSE("GPL v2");
