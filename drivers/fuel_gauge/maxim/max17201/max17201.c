/*
 * Copyright (c) 2024, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT maxim_max17201

#include <zephyr/logging/log.h>

#include "max17201.h"

LOG_MODULE_REGISTER(MAX17201, CONFIG_FUEL_GAUGE_LOG_LEVEL);

static int max17201_set_property(const struct device *dev, fuel_gauge_prop_t prop,
				 union fuel_gauge_prop_val val)
{
	return 0;
}

static int max17201_get_property(const struct device *dev, fuel_gauge_prop_t prop,
				 union fuel_gauge_prop_val *val)
{
	return 0;
}

static int max17201_init(const struct device *dev)
{
	const struct max17201_config *config = dev->config;
	struct max17201_data *data = dev->data;

	if (!i2c_is_ready_dt(&config->i2c_bus)) {
		LOG_ERR("I2C bus not ready!!");
		return -ENODEV;
	}

	LOG_INF("M5 Addr: [0x%02X]", config->m5_addr);
	LOG_INF("SBS Addr: [0x%02X]", config->sbs_addr);

	return 0;
}

static const struct fuel_gauge_driver_api max17201_driver_api = {
	.set_property = &max17201_set_property,
	.get_property = &max17201_get_property,
};

#define MAX17201_INIT(n)                                                                           \
	static const struct max17201_config max17201_config_##n = {                                \
		.i2c_bus = I2C_DT_SPEC_INST_GET(n),                                                \
		.m5_addr = DT_REG_ADDR(DT_DRV_INST(n)),                                            \
		.sbs_addr = DT_INST_PROP(n, sbs),                                                  \
	};                                                                                         \
	static struct max17201_data max17201_data_##n;                                             \
	DEVICE_DT_INST_DEFINE(n, max17201_init, NULL, &max17201_data_##n, &max17201_config_##n,    \
			      POST_KERNEL, CONFIG_FUEL_GAUGE_INIT_PRIORITY, &max17201_driver_api);

DT_INST_FOREACH_STATUS_OKAY(MAX17201_INIT)
