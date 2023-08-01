// SPDX-License-Identifier: GPL-2.0-only
/*
 * AD405x ADC driver
 *
 * Copyright (c) 2023 Analog Devices Inc.
 *
 * Jorge Marques <jorge.marques@analog.com>
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i3c/device.h>
#include <linux/i3c/master.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/regmap.h>

#include "ad405x.h"

static const struct i3c_device_id ad405x_i3c_ids[] = {
	I3C_DEVICE(0x0177, 0x0070, (void *)AD4053),
	I3C_DEVICE(0x0177, 0x0071, (void *)AD4054),
	I3C_DEVICE(0x0177, 0x0072, (void *)AD4055),
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(i3c, ad405x_i3c_ids);

static int ad405x_i3c_probe(struct i3c_device *i3cdev)
{
	struct regmap_config ad405x_i3c_regmap_config = {
		.reg_bits = 8,
		.val_bits = 8,
	};
	const struct i3c_device_id *id = i3c_device_match_id(i3cdev,
							    ad405x_i3c_ids);
	struct regmap *regmap;

	regmap = devm_regmap_init_i3c(i3cdev, &ad405x_i3c_regmap_config);

	if (!id)
		return PTR_ERR(id);

	if (IS_ERR(regmap)) {
		dev_err(&i3cdev->dev, "Failed to register i3c regmap %d\n",
			(int)PTR_ERR(regmap));
		return PTR_ERR(regmap);
	}

	return ad405x_core_probe(&i3cdev->dev, regmap, id->data, NULL);
}

static struct i3c_driver ad405x_driver = {
	.driver = {
		.name = "ad405x_i3c"
	},
	.probe = ad405x_i3c_probe,
	.id_table = ad405x_i3c_ids,
};
module_i3c_driver(ad405x_driver);

MODULE_AUTHOR("Jorge Marques <jorge.marques@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD4050, AD4051, AD4052, AD4053, AD4054, AD4055 ADC");
MODULE_LICENSE("GPL v2");
