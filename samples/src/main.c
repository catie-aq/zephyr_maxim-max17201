/*
 * Copyright (c) 2024, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/fuel_gauge.h>
#include <zephyr/drivers/gpio.h>

#include "max17201.h"

static const struct device *fg_dev = DEVICE_DT_GET(DT_NODELABEL(max172010));
static const struct gpio_dt_spec alert_gpio =
	GPIO_DT_SPEC_GET(DT_NODELABEL(max172010), alert_gpios);

struct gpio_callback gpio_cb;
struct k_work gpio_work;

static void gpio_callback_handler(const struct device *p_port, struct gpio_callback *p_cb,
				  gpio_port_pins_t pins)
{
	ARG_UNUSED(p_port);
	ARG_UNUSED(p_cb);
	ARG_UNUSED(pins);

	k_work_submit(&gpio_work); // Using work queue to exit isr context
}

static void gpio_worker(struct k_work *p_work)
{
	ARG_UNUSED(p_work);

	union fuel_gauge_prop_val value;
	int err;

	err = fuel_gauge_get_prop(fg_dev, FUEL_GAUGE_STATUS, &value);
	if (err < 0) {
		printk("IRQ GPIO worker read FLAGS error\n");
		return;
	}

	if (value.fg_status & MAX1720X_FLAGS_ALERT_CURR_MIN) {
		value.fg_status &= ~MAX1720X_FLAGS_ALERT_CURR_MIN;
		k_work_submit(&current_min_work);
	}
	if (value.fg_status & MAX1720X_FLAGS_ALERT_CURR_MAX) {
		value.fg_status &= ~MAX1720X_FLAGS_ALERT_CURR_MAX;
		k_work_submit(&current_max_work);
	}
	if (value.fg_status & MAX1720X_FLAGS_ALERT_SOC_PERCENT) {
		value.fg_status &= ~MAX1720X_FLAGS_ALERT_SOC_PERCENT;
		k_work_submit(&soc_percent_work);
	}
	if (value.fg_status & MAX1720X_FLAGS_ALERT_VOLT_MIN) {
		value.fg_status &= ~MAX1720X_FLAGS_ALERT_VOLT_MIN;
		k_work_submit(&voltage_min_work);
	}
	if (value.fg_status & MAX1720X_FLAGS_ALERT_VOLT_MAX) {
		value.fg_status &= ~MAX1720X_FLAGS_ALERT_VOLT_MAX;
		k_work_submit(&voltage_max_work);
	}
	if (value.fg_status & MAX1720X_FLAGS_ALERT_TEMP_MIN) {
		value.fg_status &= ~MAX1720X_FLAGS_ALERT_TEMP_MIN;
		k_work_submit(&temp_min_work);
	}
	if (value.fg_status & MAX1720X_FLAGS_ALERT_TEMP_MAX) {
		value.fg_status &= ~MAX1720X_FLAGS_ALERT_TEMP_MAX;
		k_work_submit(&temp_max_work);
	}

	err = fuel_gauge_set_prop(fg_dev, FUEL_GAUGE_STATUS, value);
	if (err < 0) {
		printk("Alert GPIO worker read FLAGS error\n");
		return;
	}
}

static void gpio_callback_handler(const struct device *p_port, struct gpio_callback *p_cb,
				  gpio_port_pins_t pins)
{
	ARG_UNUSED(p_port);
	ARG_UNUSED(p_cb);
	ARG_UNUSED(pins);

	k_work_submit(&gpio_work); /* Using work queue to exit isr context */
}

int main(void)
{
	union fuel_gauge_prop_val value;
	int err;

	if (!device_is_ready(fg_dev)) {
		printk("Device %s is not ready\n", fg_dev->name);
		return 1;
	}

	if (!gpio_is_ready_dt(&alert_gpio)) {
		printk("GPIO not ready!!");
		return -ENODEV;
	}

	printk("GPIO configure\n");
	err = gpio_pin_configure_dt(&alert_gpio, GPIO_INPUT);
	if (err < 0) {
		printk("Failed to configure GPIO!!");
		return err;
	}

	err = gpio_pin_interrupt_configure_dt(&alert_gpio, GPIO_INT_EDGE_FALLING);
	if (err < 0) {
		printk("Failed to configure interrupt!!");
		return err;
	}

	gpio_init_callback(&gpio_cb, gpio_callback_handler, BIT(alert_gpio.pin));

	err = gpio_add_callback_dt(&alert_gpio, &gpio_cb);
	if (err < 0) {
		printk("Failed to add GPIO callback!!");
		return err;
	}

	gpio_work.handler = gpio_worker;
	current_min_work.handler = current_min_worker;
	current_max_work.handler = current_max_worker;
	soc_percent_work.handler = soc_percent_worker;
	voltage_min_work.handler = voltage_min_worker;
	voltage_max_work.handler = voltage_max_worker;
	temp_min_work.handler = temp_min_worker;
	temp_max_work.handler = temp_max_worker;

	/* Setup alert for State Of Charge thresholds value (0xFF00 = no thresholds)*/
	value.flags = 0xFF00U; /* [Mx(8 bits) | Mn(8 bits)]  with 1% LSB resolution */
	err = fuel_gauge_set_prop(fg_dev, MAX1720X_FUEL_GAUGE_SOC_THRESHOLDS, value);
	if (err < 0) {
		return err;
	}

	/* Enable Alert irq */
	value.flags = MAX1720X_ALERT_ENABLE_THRESHOLDS | MAX1720X_ALERT_ENABLE_SOC_PERCENT;
	err = fuel_gauge_set_prop(fg_dev, MAX1720X_FUEL_GAUGE_ALERT_ENABLE, value);
	if (err < 0) {
		return err;
	}

	/* Clear Alert Flags */
	value.flags = 0x0000U;
	err = fuel_gauge_set_prop(fg_dev, FUEL_GAUGE_STATUS, value);
	if (err < 0) {
		return err;
	}

	while (1) {
		/* Example for fuel gauge driver */
		printk("----------------\n");
		err = fuel_gauge_get_prop(fg_dev, FUEL_GAUGE_AVG_CURRENT, &value);
		if (err < 0) {
			return err;
		}
		printk("Avg Current: %d uA\n", value.avg_current);
		err = fuel_gauge_get_prop(fg_dev, FUEL_GAUGE_CURRENT, &value);
		if (err < 0) {
			return err;
		}
		printk("Current: %d uA\n", value.current);
		err = fuel_gauge_get_prop(fg_dev, FUEL_GAUGE_CYCLE_COUNT, &value);
		if (err < 0) {
			return err;
		}
		printk("Cycles: %d%%\n", value.cycle_count);
		err = fuel_gauge_get_prop(fg_dev, FUEL_GAUGE_FLAGS, &value);
		if (err < 0) {
			return err;
		}
		printk("FLAGS: 0x%08X\n", value.flags);
		err = fuel_gauge_get_prop(fg_dev, FUEL_GAUGE_FULL_CHARGE_CAPACITY, &value);
		if (err < 0) {
			return err;
		}
		printk("Full Charge Cap: %d uAh\n", value.full_charge_capacity);
		err = fuel_gauge_get_prop(fg_dev, FUEL_GAUGE_PRESENT_STATE, &value);
		if (err < 0) {
			return err;
		}
		printk("Battery present: %d\n", value.fg_status);
		err = fuel_gauge_get_prop(fg_dev, FUEL_GAUGE_REMAINING_CAPACITY, &value);
		if (err < 0) {
			return err;
		}
		printk("Remaining Cap: %d uAh\n", value.remaining_capacity);
		err = fuel_gauge_get_prop(fg_dev, FUEL_GAUGE_RUNTIME_TO_EMPTY, &value);
		if (err < 0) {
			return err;
		}
		printk("TTE: %d minutes\n", value.runtime_to_empty);
		err = fuel_gauge_get_prop(fg_dev, FUEL_GAUGE_RUNTIME_TO_FULL, &value);
		if (err < 0) {
			return err;
		}
		printk("TTF: %d minutes\n", value.runtime_to_full);
		err = fuel_gauge_get_prop(fg_dev, FUEL_GAUGE_ABSOLUTE_STATE_OF_CHARGE, &value);
		if (err < 0) {
			return err;
		}
		printk("Abs SOC: %d%%\n", value.absolute_state_of_charge);
		err = fuel_gauge_get_prop(fg_dev, FUEL_GAUGE_RELATIVE_STATE_OF_CHARGE, &value);
		if (err < 0) {
			return err;
		}
		printk("Rel SOC: %d%%\n", value.relative_state_of_charge);
		err = fuel_gauge_get_prop(fg_dev, FUEL_GAUGE_TEMPERATURE, &value);
		if (err < 0) {
			return err;
		}
		printk("Temp: %d 0.1K\n", value.temperature);
		err = fuel_gauge_get_prop(fg_dev, FUEL_GAUGE_VOLTAGE, &value);
		if (err < 0) {
			return err;
		}
		printk("Voltage: %d uV\n", value.voltage);
		err = fuel_gauge_get_prop(fg_dev, FUEL_GAUGE_STATUS, &value);
		if (err < 0) {
			return err;
		}
		printk("STATUS: 0x%04X\n", value.fg_status);
		err = fuel_gauge_get_prop(fg_dev, FUEL_GAUGE_DESIGN_CAPACITY, &value);
		if (err < 0) {
			return err;
		}
		printk("DesignCap: %d mAh\n", value.design_cap);

		k_sleep(K_MSEC(100));
	}
}
