# Overview

This sample application provides an example usage of the Maxim MAX17201 Fuel Gauge.

It configures the fuel gauge for the following battery settings from devicetree:

- `nb-cell`: Battery cell number.
- `rshunt`: Shunt resistor in mOmhs.
- `capacity`: Battery theorical capacity in mAh.
- `empty-voltage`: Empty battery voltage in mV.
- `external-thermistor1`: Presence of external thermistor on AIN1.
- `external-thermistor2`: Presence of external thermistor on AIN2.
- `alert-gpios`: GPIO used for alert interrupt signal.

And displays each second the following property:

- Average Current (uA)
- Current (uA)
- Cycles (1/100th)
- Flags (Refer to `max17201.h` for flags indexes)
- Full Charge Capacity (uAh)
- Battery presence
- Remaining Capaciy (uAh)
- Time To Empty (minutes)
- Time To Full (minutes)
- Absolute State Of Charge
- Relative State Of Charge
- Temperature (0.1K)
- Cell Voltage (uV)
- Status (Refer to `max17201.h` for flags indexes)
- Design Capacity (mAh)

> [!NOTE]
>
> Don't forget to specify MAX17201 configuration in the devicetree:
>
> ```
> max172010: max17201@36 {
> 	compatible = "maxim,max17201";
> 	reg = <0x36>;
> 	sbs = <0x0b>;
> 	nb-cell = <1>; /* Battery cell number */
> 	rshunt = <20>; /* Shunt resistor value in mOhms */
> 	capacity = <800>; /* Battery capacity in mA·h */
> 	empty-voltage = <3050>; /* Empty battery voltage in mV */
> 	external-thermistor1; /* Presence of external thermistor on AIN1 */
> 	external-thermistor2; /* Presence of external thermistor on AIN2 */
> 	alert-gpios = <&sixtron_connector DIO1 (GPIO_ACTIVE_LOW | GPIO_PULL_UP)>;
> };
> ```

# Requirements

- Power supply: fuel gauge requires a battery to monitor.
- Board allowing I2C communication.
- Available GPIOs for alert interrupts.

# References

- [MAX17201 Component](https://www.maximintegrated.com/en/products/power/battery-management/MAX17201.html/storefront/storefront.html).
- [MAX17201 Datasheet](https://www.mouser.fr/datasheet/2/609/MAX17201_MAX17215-3469373.pdf).

# Building and Running

> [!NOTE]
>
> `CONFIG_FUEL_GAUGE=y` in prj.conf to use fuel gauge API.

```shell
cd <driver_directory>
west build -p always -b <BOARD> samples/ -- -D DTC_OVERLAY_FILE=sixtron_bus.overlay
west flash
```

# Sample Output

```shell
[00:00:00.000,000] <inf> MAX17201: CONF: M5 ADDR: [0x36]
[00:00:00.000,000] <inf> MAX17201: CONF: SB ADDR: [0x0B]
[00:00:00.000,000] <inf> MAX17201: CONF: DEVICE: [MAX17201]
[00:00:00.000,000] <inf> MAX17201: CONF: REVISION: [272]
[00:00:00.000,000] <inf> MAX17201: CONF: CELLS NB: [1]
[00:00:00.000,000] <inf> MAX17201: CONF: R SHUNT: [20 mOhms]
[00:00:00.000,000] <inf> MAX17201: CONF: CAPACITY: [800 mAh]
[00:00:00.000,000] <inf> MAX17201: CONF: EMPTY VOLTAGE: [3050 mV]
[00:00:00.000,000] <inf> MAX17201: CONF: EXT THERMISTOR: [x][x]
[00:00:00.000,000] <inf> MAX17201: CONF: NTC THERMISTOR: [0 idx]
[00:00:00.000,000] <inf> MAX17201: CONF: INTERNAL TEMP: [ ]
[00:00:00.001,000] <wrn> MAX17201: MAX17201 Configuration...		# POR ONLY #
[00:00:00.128,000] <inf> MAX17201: MAX17201 configured!
*** Booting Zephyr OS build v3.7.0 ***
----------------
Avg Current: -23281
Current: -23906
Cycles: 0
FLAGS: 0x00000080
Full Charge Cap: 800000 uAh
Battery present: 1
Remaining Cap: 595000 uAh
TTE: 1495 m
TTF: 6143 m
Abs SOC: 74%
Rel SOC: 74%
Temp: 3001 0.1K
Voltage: 3943750 uV
STATUS: 0x0080
DesignCap: 800 mAh
----------------
Avg Current: -23281
Current: -23906
Cycles: 0
FLAGS: 0x00000080
Full Charge Cap: 800000 uAh
Battery present: 1
Remaining Cap: 595000 uAh
TTE: 1495 m
TTF: 6143 m
Abs SOC: 74%
Rel SOC: 74%
Temp: 3001 0.1K
Voltage: 3943750 uV
STATUS: 0x0080
DesignCap: 800 mAh

<repeats endlessly>
```
