/*
 * Copyright (c) 2024, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/fuel_gauge.h>

int main(void)
{
	const struct device *const dev = DEVICE_DT_GET(DT_NODELABEL(max172010));
	int err;

	if (!device_is_ready(dev)) {
		printk("Device %s is not ready\n", dev->name);
		return 1;
	}

	while (1) {
		// Example for fuel gauge driver
		union fuel_gauge_prop_val value;
		err = fuel_gauge_get_prop(dev, FUEL_GAUGE_AVG_CURRENT, &value);
		if (err < 0) {
			return err;
		}
		printk("Avg Current: %d\n", value.avg_current);
		err = fuel_gauge_get_prop(dev, FUEL_GAUGE_CURRENT, &value);
		if (err < 0) {
			return err;
		}
		printk("Current: %d\n", value.current);
		err = fuel_gauge_get_prop(dev, FUEL_GAUGE_CYCLE_COUNT, &value);
		if (err < 0) {
			return err;
		}
		printk("Cycles: %d\n", value.cycle_count);
		err = fuel_gauge_get_prop(dev, FUEL_GAUGE_FLAGS, &value);
		if (err < 0) {
			return err;
		}
		printk("FLAGS: 0x%08X\n", value.flags);

		k_sleep(K_MSEC(1000));
	}
}
