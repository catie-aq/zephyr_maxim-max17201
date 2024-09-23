/*
 * Copyright (c) 2024, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT maxim_max17201

#include <zephyr/logging/log.h>

#include "max17201.h"

LOG_MODULE_REGISTER(MAX17201, CONFIG_FUEL_GAUGE_LOG_LEVEL);

static int max17201_i2c_read(const struct device *dev, uint16_t reg_addr, uint16_t *result)
{
	const struct max17201_config *config = dev->config;
	uint16_t addr;
	uint8_t reg[2];
	int err;

	if (MAX1720X_REGISTER_IS_SBS(reg_addr)) {
		addr = config->sbs_addr;
	} else {
		addr = config->m5_addr;
	}
	reg_addr = reg_addr & 0x00FFU;
	const struct i2c_dt_spec i2c = config->i2c_bus;

	err = i2c_write_read(i2c.bus, addr, &reg_addr, sizeof(reg_addr), reg, sizeof(reg));
	if (err != 0) {
		LOG_ERR("[ER] Unable to read register, error %d", err);
		return err;
	}

	*result = ((reg[1] << 8) | (reg[0] << 0));
	LOG_DBG("Read register 0x%03X: [0x%04X]", reg_addr, *result);
	return 0;
}

static int max17201_i2c_write(const struct device *dev, uint16_t reg_addr, uint16_t value)
{
	const struct max17201_config *config = dev->config;
	uint16_t addr;
	uint8_t reg[2];
	int err;

	if (MAX1720X_REGISTER_IS_SBS(reg_addr)) {
		addr = config->sbs_addr;
	} else {
		addr = config->m5_addr;
	}
	reg_addr = reg_addr & 0x00FFU;
	const struct i2c_dt_spec i2c = config->i2c_bus;
	reg[0] = value & 0x00FFU;
	reg[1] = (value >> 8) & 0x00FFU;

	err = i2c_burst_write(i2c.bus, addr, reg_addr, reg, sizeof(reg));
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

static int max17201_configure_thermistor(const struct device *dev)
{
	const struct max17201_config *config = dev->config;
	int err;

	err = max17201_i2c_write(dev, MAX1720X_REGISTER_N_T_GAIN,
				 ntc_gain[config->ntc_thermistors]);
	if (err < 0) {
		LOG_ERR("[ET_1] Unable to write nTGain, error %d", err);
		return err;
	}
	err = max17201_i2c_write(dev, MAX1720X_REGISTER_N_T_OFF,
				 ntc_offfset[config->ntc_thermistors]);
	if (err < 0) {
		LOG_ERR("[ET_2] Unable to write nTOff, error %d", err);
		return err;
	}
	err = max17201_i2c_write(dev, MAX1720X_REGISTER_N_T_CURVE,
				 ntc_curve[config->ntc_thermistors]);
	if (err < 0) {
		LOG_ERR("[ET_3] Unable to write nTCurve, error %d", err);
		return err;
	}

	return 0;
}

static int max17201_configuration(const struct device *dev)
{
	const struct max17201_config *config = dev->config;
	struct max17201_data *data = dev->data;
	uint16_t config_reg;
	int err;

	/* nNVCfg 0 configuration */
	config_reg = (MAX1720X_DISABLE << 15) | (MAX1720X_DISABLE << 14) | (0 << 13) | (0 << 12) |
		     (MAX1720X_DISABLE << 11) | (MAX1720X_DISABLE << 10) | (MAX1720X_DISABLE << 9) |
		     (MAX1720X_ENABLE << 8) | (MAX1720X_DISABLE << 7) | (MAX1720X_DISABLE << 6) |
		     (MAX1720X_ENABLE << 5) | (MAX1720X_ENABLE << 4) | (MAX1720X_DISABLE << 3) |
		     (MAX1720X_DISABLE << 2) | (MAX1720X_DISABLE << 1) | (MAX1720X_DISABLE << 0);
	err = max17201_i2c_write(dev, MAX1720X_REGISTER_N_NV_CFG_0, config_reg);
	if (err < 0) {
		LOG_ERR("[EC_1] Unable to write nNVCfg_0, error %d", err);
		return err;
	}

	/* nNVCfg 1 configuration */
	config_reg = (MAX1720X_DISABLE << 15) | (MAX1720X_DISABLE << 14) |
		     (MAX1720X_DISABLE << 13) | (MAX1720X_DISABLE << 12) |
		     (MAX1720X_DISABLE << 11) | (0 << 10) | (0 << 9) | (0 << 8) | (0 << 7) |
		     (0 << 6) | (0 << 5) | (MAX1720X_DISABLE << 4) | (MAX1720X_DISABLE << 3) |
		     (MAX1720X_ENABLE << 2) | (MAX1720X_ENABLE << 1) | (0 << 0);
	err = max17201_i2c_write(dev, MAX1720X_REGISTER_N_NV_CFG_1, config_reg);
	if (err < 0) {
		LOG_ERR("[EC_2] Unable to write nNVCfg_1, error %d", err);
		return err;
	}

	// 0 < cycles < 0x7FU max 64 cycles
	uint8_t cycles = 5;
	/* nNVCfg 2 configuration */
	config_reg = (MAX1720X_ENABLE << 15) | (MAX1720X_ENABLE << 14) | (MAX1720X_ENABLE << 13) |
		     (MAX1720X_ENABLE << 12) | (MAX1720X_ENABLE << 11) | (MAX1720X_ENABLE << 10) |
		     (MAX1720X_ENABLE << 9) | (MAX1720X_ENABLE << 8) | (MAX1720X_DISABLE << 7) |
		     ((((cycles << 2) - 1) & 0x7FU) << 0);
	err = max17201_i2c_write(dev, MAX1720X_REGISTER_N_NV_CFG_2, config_reg);
	if (err < 0) {
		LOG_ERR("[EC_3] Unable to write nNVCfg_2, error %d", err);
		return err;
	}

	/* Config configuration */
	config_reg = (0 << 15) | (MAX1720X_DISABLE << 14) | (MAX1720X_DISABLE << 13) |
		     (MAX1720X_DISABLE << 12) | (MAX1720X_DISABLE << 11) |
		     (MAX1720X_DISABLE << 10) | (MAX1720X_ENABLE << 9) | (MAX1720X_DISABLE << 8) |
		     (MAX1720X_DISABLE << 7) | (MAX1720X_DISABLE << 6) | (MAX1720X_DISABLE << 5) |
		     (1 << 4) | (MAX1720X_DISABLE << 3) | (MAX1720X_DISABLE << 2) |
		     (MAX1720X_DISABLE << 1) | (MAX1720X_DISABLE << 0);
	err = max17201_i2c_write(dev, MAX1720X_REGISTER_N_CONFIG, config_reg);
	if (err < 0) {
		LOG_ERR("[EC_4] Unable to write nConfig, error %d", err);
		return err;
	}

	if ((config->nb_cell <= 0x00U) || (config->nb_cell > 0x0FU)) {
		LOG_ERR("[EC_5] NB CELLS not valid!");
		return -EINVAL;
	}
	if ((data->device_type == MAX1720X_DEVICE_NAME_17201) &&
	    (config->nb_cell > MAX17201_MAX_CELLS)) {
		LOG_ERR("[EC_6] NB CELLS not valid!");
		return -EINVAL;
	}

	uint8_t internal_temp = MAX1720X_DISABLE;
#if defined(CONFIG_MAX17201_INTERNAL_TEMP)
	internal_temp = MAX1720X_ENABLE;
#endif
	uint8_t fgt = ((config->ext_thermistor1 ? (config->ext_thermistor2 ? 0x0U : 0x1U) : 0x0U));
	/* nPackCfg configuration */
	config_reg = (fgt << 15) | (0 << 14) | ((config->ext_thermistor1 ? 0x1U : 0x0U) << 13) |
		     ((config->ext_thermistor2 ? 0x1U : 0x0U) << 12) | (internal_temp << 11) |
		     (MAX1720X_ENABLE << 10) | (MAX1720X_DISABLE << 9) | (MAX1720X_DISABLE << 8) |
		     ((MAX17201_CELL_BALANCING & 0x07U) << 5) | (0 << 4) | (config->nb_cell << 0);
	err = max17201_i2c_write(dev, MAX1720X_REGISTER_N_PACK_CFG, config_reg);
	if (err < 0) {
		LOG_ERR("[EC_7] Unable to write nPackCfg, error %d", err);
		return err;
	}

	// EXT Thermistors configuration
	err = max17201_configure_thermistor(dev);
	if (err < 0) {
		LOG_ERR("[EC_8] Unable to configure thermistors, error %d", err);
		return err;
	}

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
	err = max17201_i2c_read(dev, 0x21, &reg);
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
		data->device_type = MAX1720X_DEVICE_NAME_17201;
	} else if ((reg & 0x0F) == MAX1720X_DEVICE_NAME_17205) {
		LOG_INF("DEVICE: [MAX17205]");
		data->device_type = MAX1720X_DEVICE_NAME_17205;
	} else {
		LOG_ERR("[EI_4] DEVICE: [UKNOWN]");
	}
	LOG_INF("REVISION: [%d]", reg >> 4);
	LOG_INF("CELLS NB: [%d]", config->nb_cell);
	LOG_INF("R SHUNT: [%d mOhms]", config->rshunt);
	LOG_INF("CAPACITY: [%d mAh]", config->capacity);
	LOG_INF("EMPTY VOLTAGE: [%d mV]", config->empty_voltage);
	LOG_INF("EXT THERMISTOR: [%s][%s]", config->ext_thermistor1 ? "x" : " ",
		config->ext_thermistor2 ? "x" : " ");
	LOG_INF("NTC THERMISTOR: [%d idx]", config->ntc_thermistors);
	char internal_temp = ' ';
#if defined(CONFIG_MAX17201_INTERNAL_TEMP)
	internal_temp = 'x';
#endif
	LOG_INF("INTERNAL TEMP: [%c]", internal_temp);

	err = max17201_i2c_read(dev, MAX1720X_REGISTER_DESIGN_CAP, &reg);
	if (err < 0) {
		LOG_ERR("[EI_5] Unable to read DesignCap, error %d", err);
		return err;
	}

	/* 750 mAh capacity is default -> MAX17201 need configuration */
	if (MAX1720X_COMPUTE_ZEPHYR_CAPACITY_MAH(reg, config->rshunt) == 750) {
		LOG_INF("MAX17201 Configuration...");
		LOG_DBG("Restoring MAX17201 non-volatile memory");
		err = max17201_i2c_write(dev, MAX1720X_REGISTER_COMMAND,
					 MAX1720X_COMMAND_HARDWARE_RESET);
		if (err < 0) {
			LOG_ERR("[EI_6] Unable to write COMMAND, error %d", err);
			return err;
		}
		k_sleep(K_MSEC(MAX1720X_TIMING_POWER_ON_RESET_MS));

		err = max17201_configuration(dev);
		if (err < 0) {
			LOG_ERR("[EI_7] Unable to configure DEVICE, error %d", err);
			return err;
		}
	}
	LOG_INF("MAX17201 configured!");

	err = max17201_i2c_read(dev, MAX1720X_REGISTER_AGE, &reg);
	if (err < 0) {
		LOG_ERR("[EI_7] Unable to read Age, error %d", err);
		return err;
	}
	LOG_INF("AGE: [%d%%]", MAX1720X_COMPUTE_PERCENTAGE(reg));

	err = max17201_i2c_read(dev, MAX1720X_REGISTER_TEMP, &reg);
	if (err < 0) {
		LOG_ERR("[EI_8] Unable to read Temp, error %d", err);
		return err;
	}
	LOG_INF("TEMP: [%d C]", MAX1720X_COMPUTE_TEMPERATURE(reg));

	err = max17201_i2c_read(dev, MAX1720X_REGISTER_TTE, &reg);
	if (err < 0) {
		LOG_ERR("[EI_9] Unable to read TTE, error %d", err);
		return err;
	}
	LOG_INF("TIME TO EMPTY: [%d s]", MAX1720X_COMPUTE_TIME(reg));

	err = max17201_i2c_read(dev, MAX1720X_REGISTER_DESIGN_CAP, &reg);
	if (err < 0) {
		LOG_ERR("[EI_A] Unable to read DesignCap, error %d", err);
		return err;
	}
	LOG_INF("DESIGN CAP: [%d mAh]", MAX1720X_COMPUTE_ZEPHYR_CAPACITY_MAH(reg, config->rshunt));

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
		.ntc_thermistors = DT_INST_ENUM_IDX(n, ntc_thermistors),                           \
	};                                                                                         \
	static struct max17201_data max17201_data_##n;                                             \
	DEVICE_DT_INST_DEFINE(n, max17201_init, NULL, &max17201_data_##n, &max17201_config_##n,    \
			      POST_KERNEL, CONFIG_FUEL_GAUGE_INIT_PRIORITY, &max17201_driver_api);

DT_INST_FOREACH_STATUS_OKAY(MAX17201_INIT)
