// SPDX-License-Identifier: GPL-2.0-only
/*
 * bma400-i2c.c - I2C IIO driver for Bosch BMA400 triaxial acceleration sensor.
 *
 * Copyright 2019 Dan Robertson <dan@dlrobertson.com>
 *
 * I2C address is either 0x14 or 0x15 depending on SDO
 *
 */
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/acpi.h>
#include <linux/of.h>
#include <linux/regmap.h>

#include "bma400.h"

static int bma400_i2c_probe(struct i2c_client *client,
			    const struct i2c_device_id *id)
{
	struct regmap *regmap;

	regmap = devm_regmap_init_i2c(client,
				      &bma400_regmap_config);

	return bma400_probe(&client->dev, regmap, id->name);
}

static int bma400_i2c_remove(struct i2c_client *client)
{
	return bma400_remove(&client->dev);
}

static const struct i2c_device_id bma400_i2c_ids[] = {
	{ "bma400", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, bma400_i2c_ids);

static struct i2c_driver bma400_i2c_driver = {
	.driver = {
		.name = "bma400",
	},
	.probe    = bma400_i2c_probe,
	.remove   = bma400_i2c_remove,
	.id_table = bma400_i2c_ids,
};

module_i2c_driver(bma400_i2c_driver);

MODULE_AUTHOR("Dan Robertson <dan@dlrobertson.com>");
MODULE_DESCRIPTION("Bosch BMA400 triaxial acceleration sensor");
MODULE_LICENSE("GPL");
