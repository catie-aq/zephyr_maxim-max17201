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

	if (!device_is_ready(dev)) {
		printk("Device %s is not ready\n", dev->name);
		return 1;
	}

	while (1) {
		// Example for fuel gauge driver
		union fuel_gauge_prop_val value;
		fuel_gauge_get_prop(dev, FUEL_GAUGE_AVG_CURRENT, &value);
		printk("Avg Current: %d\n", value.avg_current);

		k_sleep(K_MSEC(1000));
	}
}
