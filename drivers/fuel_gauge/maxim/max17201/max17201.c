/*
 * Copyright (c) 2024, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT maxim_max17201

#include <zephyr/logging/log.h>

#include "max17201.h"

LOG_MODULE_REGISTER(MAX17201, CONFIG_FUEL_GAUGE_LOG_LEVEL);

int max17201_i2c_read_dt(const struct device *dev, uint8_t reg_addr, uint16_t *result)
{
	const struct max17201_config *config = dev->config;
	uint8_t reg[2];
	int err;

	err = i2c_write_read_dt(&config->i2c_bus, &reg_addr, sizeof(reg_addr), reg, sizeof(reg));
	if (err != 0) {
		LOG_ERR("[ER] Unable to read register, error %d", err);
		return err;
	}

	*result = ((reg[1] << 8) | (reg[0] << 0));
	LOG_DBG("Read register 0x%03X: [0x%04X]", reg_addr, *result);
	return 0;
}

int max17201_i2c_write_dt(const struct device *dev, uint8_t reg_addr, uint16_t value)
{
	const struct max17201_config *config = dev->config;
	uint8_t reg[2];
	int err;

	reg[0] = value & 0xFF;
	reg[1] = (value >> 8) & 0xFF;
	err = i2c_burst_write_dt(&config->i2c_bus, reg_addr, reg, sizeof(reg));
	if (err != 0) {
		LOG_ERR("[EW] Unable to write register, error %d", err);
		return err;
	}

	LOG_DBG("Write register 0x%03X: [0x%02X][0x%02X]", reg_addr, reg[0], reg[1]);
	return 0;
}

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
	int err;

	if (!i2c_is_ready_dt(&config->i2c_bus)) {
		LOG_ERR("[EI_1] I2C bus not ready!!");
		return -ENODEV;
	}

	LOG_INF("M5 ADDR: [0x%02X]", config->m5_addr);
	LOG_INF("SB ADDR: [0x%02X]", config->sbs_addr);

	uint16_t reg;
	err = max17201_i2c_read_dt(dev, 0x21, &reg);
	if (err < 0) {
		LOG_ERR("[EI_2] Unable to read MAX_ID, error %d", err);
		return err;
	}

	if (!(reg & 0x0F)) {
		LOG_ERR("[EI_3] Device Name NULL | reg[0x%04X]", reg);
		return -1;
	}

	if ((reg & 0x0F) == MAX1720X_DEVICE_NAME_17201) {
		LOG_INF("DEVICE: [MAX17201]");
	} else if ((reg & 0x0F) == MAX1720X_DEVICE_NAME_17205) {
		LOG_INF("DEVICE: [MAX17205]");
	} else {
		LOG_WRN("DEVICE: [UKNOWN]");
	}
	LOG_INF("REVISION: [%d]", reg >> 4);
	LOG_INF("CELLS NB: [%d]", config->nb_cell);
	LOG_INF("R SHUNT: [%d mOhms]", config->rshunt);
	LOG_INF("CAPACITY: [%d mAh]", config->capacity);
	LOG_INF("EMPTY VOLTAGE: [%d mV]", config->empty_voltage);
	LOG_INF("EXT THERMISTOR: [%s][%s]", config->ext_thermistor1 ? "x" : " ",
		config->ext_thermistor2 ? "x" : " ");

	/* 750 mAh capacity is default -> MAX17201 need configuration */
	if (MAX1720X_COMPUTE_ZEPHYR_CAPACITY_MAH(reg, config->rshunt) == 750) {
		LOG_INF("MAX17201 need to be configured!");
		err = max17201_i2c_write_dt(dev, 0x60, 0x000F);
		if (err < 0) {
			LOG_ERR("[EI_4] Unable to read MAX_ID, error %d", err);
			return err;
		}
		k_sleep(K_MSEC(10));
	}
	LOG_INF("MAX17201 configured!");

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
		.nb_cell = DT_INST_PROP(n, nb_cell),                                               \
		.rshunt = DT_INST_PROP(n, rshunt),                                                 \
		.capacity = DT_INST_PROP(n, capacity),                                             \
		.empty_voltage = DT_INST_PROP(n, empty_voltage),                                   \
		.ext_thermistor1 = DT_INST_PROP(n, external_thermistor1),                          \
		.ext_thermistor2 = DT_INST_PROP(n, external_thermistor2),                          \
	};                                                                                         \
	static struct max17201_data max17201_data_##n;                                             \
	DEVICE_DT_INST_DEFINE(n, max17201_init, NULL, &max17201_data_##n, &max17201_config_##n,    \
			      POST_KERNEL, CONFIG_FUEL_GAUGE_INIT_PRIORITY, &max17201_driver_api);

DT_INST_FOREACH_STATUS_OKAY(MAX17201_INIT)
