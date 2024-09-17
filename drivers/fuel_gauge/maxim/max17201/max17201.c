/*
 * Copyright (c) 2024, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT maxim_max17201

#include <zephyr/logging/log.h>

#include "max17201.h"

LOG_MODULE_REGISTER(MAX17201, CONFIG_FUEL_GAUGE_LOG_LEVEL);

static int max17201_get_property(const struct device *dev, fuel_gauge_prop_t prop,
				 union fuel_gauge_prop_val *val)
{
	return 0;
}

static int max17201_init(const struct device *dev)
{
	const struct max17201_config *config = dev->config;
	struct max17201_data *data = dev->data;

	return 0;
}

static const struct fuel_gauge_driver_api max17201_driver_api = {
	.get_property = &max17201_get_property,
};

#define MAX17201_INIT(n)                                                                           \
	static struct max17201_config max17201_config_##n = {};                                    \
	static struct max17201_data max17201_data_##n;                                             \
	DEVICE_DT_INST_DEFINE(n, max17201_init, NULL, &max17201_data_##n, &max17201_config_##n,    \
			      POST_KERNEL, CONFIG_FUEL_GAUGE_INIT_PRIORITY, &max17201_driver_api);

DT_INST_FOREACH_STATUS_OKAY(MAX17201_INIT)
