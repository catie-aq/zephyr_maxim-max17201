/*
 * Copyright (c) 2024, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_MAX17201_MAX17201_H_
#define ZEPHYR_DRIVERS_SENSOR_MAX17201_MAX17201_H_

#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/fuel_gauge.h>

struct max17201_config {
	struct i2c_dt_spec i2c_bus;
	uint8_t m5_addr;
	uint8_t sbs_addr;

	int nb_cell;
	int rshunt;
	int capacity;
	int empty_voltage;
	bool ext_thermistor1;
	bool ext_thermistor2;
};

/* MAX17201 CONFIGURATION */
#define MAX1720X_DEVICE_NAME_17201 0x01U
#define MAX1720X_DEVICE_NAME_17205 0x05U

struct max17201_data {
};

#endif /* ZEPHYR_DRIVERS_SENSOR_MAX17201_MAX17201_H_ */
