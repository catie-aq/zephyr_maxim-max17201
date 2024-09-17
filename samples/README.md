# Overview

This sample application provides an example usage of the Maxim MAX17201 Fuel Gauge.

- Steps

> [!NOTE]

# Requirements

- Board allowing I2C communication.
- Available IRQ GPIO for Alert interrupts.
- `CONFIG_FUEL_GAUGE=y` in prj.conf to use Fuel Gauge API.

# References

- [RV-8803-C7 Component](https://www.maximintegrated.com/en/products/power/battery-management/MAX17201.html/storefront/storefront.html).
- [RV-8803-C7 Datasheet](https://www.mouser.fr/datasheet/2/609/MAX17201_MAX17215-3469373.pdf).

# Building and Running

```shell
cd <driver_directory>
west build -p always -b <BOARD> samples/ -- -D DTC_OVERLAY_FILE=sixtron_bus.overlay
west flash
```

# Sample Output

```shell
*** Booting Zephyr OS build v3.7.0 ***

...

<repeats endlessly>
```
