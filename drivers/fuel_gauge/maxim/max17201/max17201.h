/*
 * Copyright (c) 2024, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_MAX17201_MAX17201_H_
#define ZEPHYR_DRIVERS_SENSOR_MAX17201_MAX17201_H_

#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/fuel_gauge.h>

#define MAX1720X_NTC_MODE_MURATA 0x00U
#define MAX1720X_NTC_MODE_FENWAL 0x01U
#define MAX1720X_NTC_MODE_TDK    0x02U

static const uint16_t ntc_gain[3] = {
	0xEE56,
	0xF49A,
	0xF284,
};

static const uint16_t ntc_offfset[3] = {
	0x1DA4,
	0x16A1,
	0x18E8,
};

static const uint16_t ntc_curve[3] = {
	0x0025,
	0x0064,
	0x0035,
};

enum thermistor_ntc_mode {
	MURATA = MAX1720X_NTC_MODE_MURATA,
	FENWAL = MAX1720X_NTC_MODE_FENWAL,
	TDK = MAX1720X_NTC_MODE_TDK,
};

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
	enum thermistor_ntc_mode ntc_thermistors;
};

/* MAX17201 CONFIGURATION */
#define MAX1720X_DEVICE_NAME_17201     0x01U
#define MAX1720X_DEVICE_NAME_17205     0x05U
#define MAX1720X_ENABLE                0x1U
#define MAX1720X_DISABLE               0x0U
#define MAX1720X_V_EMPTY_HYSTERESIS    500
#define MAX1720X_V_EMPTY_CONVERSION_VE (1 / 10)
#define MAX1720X_V_EMPTY_CONVERSION_VR (1 / 40)

#define MAX17201_MAX_CELLS      0x01U
#define MAX17201_CELL_BALANCING 0x00U

#if defined(CONFIG_MAX17201_FG_INPUT_INTERNAL)
#define MAX17201_TEMP_INPUT MAX17201_TEMP_INPUT_INTERNAL
#elif defined(CONFIG_MAX17201_FG_INPUT_EXTERNAL)
#define MAX17201_TEMP_INPUT MAX17201_TEMP_INPUT_EXTERNAL
#elif defined(CONFIG_MAX17201_FG_INPUT_REGISTER)
#define MAX17201_TEMP_INPUT MAX17201_TEMP_INPUT_REGISTER
#endif

/* MAX1720X Timings */
#define MAX1720X_TIMING_POWER_ON_RESET_MS 10

/* REGISTERS Address Control */
#define MAX1720X_REGISTER_PAGE_ONE_LOW  0x000U
#define MAX1720X_REGISTER_PAGE_ONE_HIGH 0x0FFU
#define MAX1720X_REGISTER_PAGE_TWO_LOW  0x180U
#define MAX1720X_REGISTER_PAGE_TWO_HIGH 0x1FFU

#define MAX1720X_REGISTER_SBS_LOW  0x100U
#define MAX1720X_REGISTER_SBS_HIGH 0x17FU

#define MAX1720X_REGISTER_IS_SBS(reg)                                                              \
	((MAX1720X_REGISTER_PAGE_ONE_HIGH < reg) && (reg < MAX1720X_REGISTER_PAGE_TWO_LOW))

/* MAX1720X REGISTERS */
#define MAX1720X_REGISTER_STATUS      0x000U
#define MAX1720X_REGISTER_V_ALR_TH    0x001U
#define MAX1720X_REGISTER_T_ALR_TH    0x002U
#define MAX1720X_REGISTER_S_ALR_TH    0x003U
#define MAX1720X_REGISTER_AT_RATE     0x004U
#define MAX1720X_REGISTER_REP_CAP     0x005U
#define MAX1720X_REGISTER_REP_SOC     0x006U
#define MAX1720X_REGISTER_AGE         0x007U
#define MAX1720X_REGISTER_TEMP        0x008U
#define MAX1720X_REGISTER_V_CELL      0x009U
#define MAX1720X_REGISTER_CURRENT     0x00AU
#define MAX1720X_REGISTER_AVG_CURRENT 0x00BU
#define MAX1720X_REGISTER_Q_RESIDUAL  0x00CU
#define MAX1720X_REGISTER_MIX_SOC     0x00DU
#define MAX1720X_REGISTER_AV_SOC      0x00EU
#define MAX1720X_REGISTER_MIX_CAP     0x00FU

#define MAX1720X_REGISTER_FULL_CAP     0x010U
#define MAX1720X_REGISTER_TTE          0x011U
#define MAX1720X_REGISTER_QR_TABLE_00  0x012U
#define MAX1720X_REGISTER_FULL_SOC_THR 0x013U
#define MAX1720X_REGISTER_R_CELL       0x014U
#define MAX1720X_REGISTER_R_FAST       0x015U
#define MAX1720X_REGISTER_AVG_TA       0x016U
#define MAX1720X_REGISTER_CYCLES       0x017U
#define MAX1720X_REGISTER_DESIGN_CAP   0x018U
#define MAX1720X_REGISTER_AVG_V_CELL   0x019U
#define MAX1720X_REGISTER_MAX_MIN_TEMP 0x01AU
#define MAX1720X_REGISTER_MAX_MIN_VOLT 0x01BU
#define MAX1720X_REGISTER_MAX_MIN_CURR 0x01CU
#define MAX1720X_REGISTER_CONFIG       0x01DU
#define MAX1720X_REGISTER_I_CHG_TERM   0x01EU
#define MAX1720X_REGISTER_AV_CAP       0x01FU

#define MAX1720X_REGISTER_TTF          0x020U
#define MAX1720X_REGISTER_DEV_NAME     0x021U
#define MAX1720X_REGISTER_QR_TABLE_10  0x022U
#define MAX1720X_REGISTER_FULL_CAP_NOM 0x023U
#define MAX1720X_REGISTER_AIN_0        0x027U
#define MAX1720X_REGISTER_LEARN_CFG    0x028U
#define MAX1720X_REGISTER_FILTER_CFG   0x029U
#define MAX1720X_REGISTER_RELAX_CFG    0x02AU
#define MAX1720X_REGISTER_MISC_CFG     0x02BU
#define MAX1720X_REGISTER_T_GAIN       0x02CU
#define MAX1720X_REGISTER_T_OFF        0x02DU
#define MAX1720X_REGISTER_C_GAIN       0x02EU
#define MAX1720X_REGISTER_C_OFF        0x02FU

#define MAX1720X_REGISTER_QR_TABLE_20  0x032U
#define MAX1720X_REGISTER_FULL_CAP_REP 0x035U
#define MAX1720X_REGISTER_I_AVG_EMPTY  0x036U
#define MAX1720X_REGISTER_R_COMP_0     0x038U
#define MAX1720X_REGISTER_TEMP_CO      0x039U
#define MAX1720X_REGISTER_V_EMPTY      0x03AU
#define MAX1720X_REGISTER_F_STAT       0x03DU
#define MAX1720X_REGISTER_C_G_TEMP_CO  0x03EU
#define MAX1720X_REGISTER_SHDN_TIMER   0x03FU

#define MAX1720X_REGISTER_QR_TABLE_30 0x042U
#define MAX1720X_REGISTER_D_Q_ACC     0x045U
#define MAX1720X_REGISTER_D_P_ACC     0x046U
#define MAX1720X_REGISTER_V_F_REM_CAP 0x04AU
#define MAX1720X_REGISTER_QH          0x04DU

#define MAX1720X_REGISTER_STATUS_2     0x0B0U
#define MAX1720X_REGISTER_I_ALRT_TH    0x0B4U
#define MAX1720X_REGISTER_V_SHDN_CFG   0x0B8U
#define MAX1720X_REGISTER_AGE_FORECAST 0x0B9U
#define MAX1720X_REGISTER_HIB_CFG      0x0BAU
#define MAX1720X_REGISTER_CONFIG_2     0x0BBU
#define MAX1720X_REGISTER_V_RIPPLE     0x0BCU
#define MAX1720X_REGISTER_PACK_CFG     0x0BDU
#define MAX1720X_REGISTER_TIMER_H      0x0BEU

#define MAX1720X_REGISTER_AVG_CELL_4    0x0D1U
#define MAX1720X_REGISTER_AVG_CELL_3    0x0D2U
#define MAX1720X_REGISTER_AVG_CELL_2    0x0D3U
#define MAX1720X_REGISTER_AVG_CELL_1    0x0D4U
#define MAX1720X_REGISTER_CELL_4        0x0D5U
#define MAX1720X_REGISTER_CELL_3        0x0D6U
#define MAX1720X_REGISTER_CELL_2        0x0D7U
#define MAX1720X_REGISTER_CELL_1        0x0D8U
#define MAX1720X_REGISTER_CELL_X        0x0D9U
#define MAX1720X_REGISTER_BATT          0x0DAU
#define MAX1720X_REGISTER_AT_Q_RESIDUAL 0x0DCU
#define MAX1720X_REGISTER_AT_TTE        0x0DDU
#define MAX1720X_REGISTER_AT_AV_SOC     0x0DEU
#define MAX1720X_REGISTER_AT_AV_CAP     0x0DFU

/* MAX1720X Command REGISTERS */
#define MAX1720X_REGISTER_COMMAND   0x060U
#define MAX1720X_REGISTER_COMM_STAT 0x061U

/* REGISTERS NON-VOLATILE */
#define MAX1720X_REGISTER_N_X_TABLE_0  0x180U
#define MAX1720X_REGISTER_N_X_TABLE_1  0x181U
#define MAX1720X_REGISTER_N_X_TABLE_2  0x182U
#define MAX1720X_REGISTER_N_X_TABLE_3  0x183U
#define MAX1720X_REGISTER_N_X_TABLE_4  0x184U
#define MAX1720X_REGISTER_N_X_TABLE_5  0x185U
#define MAX1720X_REGISTER_N_X_TABLE_6  0x186U
#define MAX1720X_REGISTER_N_X_TABLE_7  0x187U
#define MAX1720X_REGISTER_N_X_TABLE_8  0x188U
#define MAX1720X_REGISTER_N_X_TABLE_9  0x189U
#define MAX1720X_REGISTER_N_X_TABLE_10 0x18AU
#define MAX1720X_REGISTER_N_X_TABLE_11 0x18BU
#define MAX1720X_REGISTER_N_USER_18C   0x18CU
#define MAX1720X_REGISTER_N_USER_18D   0x18DU
#define MAX1720X_REGISTER_N_OD_SC_TH   0x18EU
#define MAX1720X_REGISTER_N_OD_SC_CFG  0x18FU

#define MAX1720X_REGISTER_N_OCV_TABLE_0  0x190U
#define MAX1720X_REGISTER_N_OCV_TABLE_1  0x191U
#define MAX1720X_REGISTER_N_OCV_TABLE_2  0x192U
#define MAX1720X_REGISTER_N_OCV_TABLE_3  0x193U
#define MAX1720X_REGISTER_N_OCV_TABLE_4  0x194U
#define MAX1720X_REGISTER_N_OCV_TABLE_5  0x195U
#define MAX1720X_REGISTER_N_OCV_TABLE_6  0x196U
#define MAX1720X_REGISTER_N_OCV_TABLE_7  0x197U
#define MAX1720X_REGISTER_N_OCV_TABLE_8  0x198U
#define MAX1720X_REGISTER_N_OCV_TABLE_9  0x199U
#define MAX1720X_REGISTER_N_OCV_TABLE_10 0x19AU
#define MAX1720X_REGISTER_N_OCV_TABLE_11 0x19BU
#define MAX1720X_REGISTER_N_I_CHG_TEM    0x19CU
#define MAX1720X_REGISTER_N_FILTER_CFG   0x19DU
#define MAX1720X_REGISTER_N_V_EMPTY      0x19EU
#define MAX1720X_REGISTER_N_LEARN_CFG    0x19FU

#define MAX1720X_REGISTER_N_QR_TABLE_00  0x1A0U
#define MAX1720X_REGISTER_N_QR_TABLE_10  0x1A1U
#define MAX1720X_REGISTER_N_QR_TABLE_20  0x1A2U
#define MAX1720X_REGISTER_N_QR_TABLE_30  0x1A3U
#define MAX1720X_REGISTER_N_CYCLES       0x1A4U
#define MAX1720X_REGISTER_N_FULL_CAP_NOM 0x1A5U
#define MAX1720X_REGISTER_N_R_COMP_0     0x1A6U
#define MAX1720X_REGISTER_N_TEMP_CO      0x1A7U
#define MAX1720X_REGISTER_N_I_AVG_EMPTY  0x1A8U
#define MAX1720X_REGISTER_N_FULL_CAP_REP 0x1A9U
#define MAX1720X_REGISTER_N_VOLT_TEMP    0x1AAU
#define MAX1720X_REGISTER_N_MAX_MIN_CURR 0x1ABU
#define MAX1720X_REGISTER_N_MAX_MIN_VOLT 0x1ACU
#define MAX1720X_REGISTER_N_MAX_MIN_TEMP 0x1ADU
#define MAX1720X_REGISTER_N_SOC          0x1AEU
#define MAX1720X_REGISTER_N_TIMER_H      0x1AFU

#define MAX1720X_REGISTER_N_CONFIG     0x1B0U
#define MAX1720X_REGISTER_N_RIPPLE_CFG 0x1B1U
#define MAX1720X_REGISTER_N_MISC_CFG   0x1B2U
#define MAX1720X_REGISTER_N_DESIGN_CAP 0x1B3U
#define MAX1720X_REGISTER_N_HIB_CFG    0x1B4U
#define MAX1720X_REGISTER_N_PACK_CFG   0x1B5U
#define MAX1720X_REGISTER_N_RELAX_CFG  0x1B6U
#define MAX1720X_REGISTER_N_CONVG_CFG  0x1B7U
#define MAX1720X_REGISTER_N_NV_CFG_0   0x1B8U
#define MAX1720X_REGISTER_N_NV_CFG_1   0x1B9U
#define MAX1720X_REGISTER_N_NV_CFG_2   0x1BAU
#define MAX1720X_REGISTER_N_ROM_ID_0   0x1BCU
#define MAX1720X_REGISTER_N_ROM_ID_1   0x1BDU
#define MAX1720X_REGISTER_N_ROM_ID_2   0x1BEU
#define MAX1720X_REGISTER_N_ROM_ID_3   0x1BFU

#define MAX1720X_REGISTER_N_V_ALRT_TH       0x1C0U
#define MAX1720X_REGISTER_N_T_ALRT_TH       0x1C1U
#define MAX1720X_REGISTER_N_S_ALRT_TH       0x1C2U
#define MAX1720X_REGISTER_N_I_ALRT_TH       0x1C3U
#define MAX1720X_REGISTER_N_USER_1C4        0x1C4U
#define MAX1720X_REGISTER_N_USER_1C5        0x1C5U
#define MAX1720X_REGISTER_N_FULL_SOC_THR    0x1C6U
#define MAX1720X_REGISTER_N_TTF_CFG         0x1C7U
#define MAX1720X_REGISTER_N_C_GAIN          0x1C8U
#define MAX1720X_REGISTER_N_T_CURVE         0x1C9U
#define MAX1720X_REGISTER_N_T_GAIN          0x1CAU
#define MAX1720X_REGISTER_N_T_OFF           0x1CBU
#define MAX1720X_REGISTER_N_MANIFCTR_NAME_0 0x1CCU
#define MAX1720X_REGISTER_N_MANIFCTR_NAME_1 0x1CDU
#define MAX1720X_REGISTER_N_MANIFCTR_NAME_2 0x1CEU
#define MAX1720X_REGISTER_N_R_SENSE         0x1CFU

#define MAX1720X_REGISTER_N_USER_1D0        0x1D0U
#define MAX1720X_REGISTER_N_USER_1D1        0x1D1U
#define MAX1720X_REGISTER_N_AGE_FC_CFG      0x1D2U
#define MAX1720X_REGISTER_N_DESIGN_VOLTAGE  0x1D3U
#define MAX1720X_REGISTER_N_USER_1D4        0x1D4U
#define MAX1720X_REGISTER_N_R_FAST_V_SHDN   0x1D5U
#define MAX1720X_REGISTER_N_MANUFCTR_DATE   0x1D6U
#define MAX1720X_REGISTER_N_FIRST_USED      0x1D7U
#define MAX1720X_REGISTER_N_SERIAL_NUMBER_0 0x1D8U
#define MAX1720X_REGISTER_N_SERIAL_NUMBER_1 0x1D9U
#define MAX1720X_REGISTER_N_SERIAL_NUMBER_2 0x1DAU
#define MAX1720X_REGISTER_N_DEVICE_NUMBER_0 0x1DBU
#define MAX1720X_REGISTER_N_DEVICE_NUMBER_1 0x1DCU
#define MAX1720X_REGISTER_N_DEVICE_NUMBER_2 0x1DDU
#define MAX1720X_REGISTER_N_DEVICE_NUMBER_3 0x1DEU
#define MAX1720X_REGISTER_N_DEVICE_NUMBER_4 0x1DFU

/* MAX1720X Additionnal REGISTERS */
#define MAX1720X_REGISTER_TIMER       0x03EU
#define MAX1720X_REGISTER_D_P_ACC     0x046U
#define MAX1720X_REGISTER_V_F_SOC     0x0FFU
#define MAX1720X_REGISTER_V_F_OCV     0x0FBU
#define MAX1720X_REGISTER_V_F_REM_CAP 0x04AU

/* MAX1720X Status REGISTERS */
#define MAX1720X_REGISTER_TEMP_1       0134U
#define MAX1720X_REGISTER_TEMP_2       0x13BU
#define MAX1720X_REGISTER_INT_TEMP     0x135U
#define MAX1720X_REGISTER_AVG_TEMP_1   0x137U
#define MAX1720X_REGISTER_AVG_TEMP_2   0x139U
#define MAX1720X_REGISTER_AVG_INT_TEMP 0x138U

#define MAX1720X_REGISTER_N_OD_SC_TH  0x18EU
#define MAX1720X_REGISTER_N_OD_SC_CFG 0x18FU

/* MAX1720X SBS REGISTERS */
#define MAX1720X_REGISTER_S_CELL_4       0x13CU
#define MAX1720X_REGISTER_S_CELL_3       0x13DU
#define MAX1720X_REGISTER_S_CELL_2       0x13EU
#define MAX1720X_REGISTER_S_CELL_1       0x13FU
#define MAX1720X_REGISTER_S_AVG_CELL_4   0x14CU
#define MAX1720X_REGISTER_S_AVG_CELL_3   0x14DU
#define MAX1720X_REGISTER_S_AVG_CELL_2   0x14EU
#define MAX1720X_REGISTER_S_AVG_CELL_1   0x14FU
#define MAX1720X_REGISTER_S_AV_CAP       0x167U
#define MAX1720X_REGISTER_S_MIX_CAP      0x168U
#define MAX1720X_REGISTER_S_MAN_FCT_INFO 0x170U
#define MAX1720X_REGISTER_N_SBS_CFG      0x1BBU

/* MAX1720X COMMANDS */
#define MAX1720X_COMMAND_HARDWARE_RESET 0x000FU

/* MAX1720X Units Compute */
#define MAX1720X_COMPUTE_CAPACITY(reg, rshunt) (reg * 5 / rshunt)
#define MAX1720X_COMPUTE_PERCENTAGE(reg)       (int)((reg >> 8) / 2.56)
#define MAX1720X_COMPUTE_VOLTAGE(reg)          (reg / 12.8)
#define MAX1720X_COMPUTE_CURRENT(reg, rshunt)  ((signed int)(reg)*1.5625 / rshunt)
#define MAX1720X_COMPUTE_TEMPERATURE(reg)      (((signed int)(reg)) >> 8)
#define MAX1720X_COMPUTE_RESISTENCE(reg)       (reg / 4096)
#define MAX1720X_COMPUTE_TIME(reg)             (int)(reg * 5.625)

/* ZEPHYR Units Compute */
#define MAX1720X_ZEPHYR_CAPACITY_UAH(val) (uint32_t)(val * 1000) // uAh
#define MAX1720X_ZEPHYR_CAPACITY_MAH(val) (uint16_t)(val)        // mAh

#define MAX1720X_COMPUTE_ZEPHYR_CAPACITY_UAH(reg, rshunt)                                          \
	(MAX1720X_ZEPHYR_CAPACITY_UAH(MAX1720X_COMPUTE_CAPACITY(reg, rshunt)))
#define MAX1720X_COMPUTE_ZEPHYR_CAPACITY_MAH(reg, rshunt)                                          \
	(MAX1720X_ZEPHYR_CAPACITY_MAH(MAX1720X_COMPUTE_CAPACITY(reg, rshunt)))

struct max17201_data {
	uint8_t device_type;
};

#endif /* ZEPHYR_DRIVERS_SENSOR_MAX17201_MAX17201_H_ */
