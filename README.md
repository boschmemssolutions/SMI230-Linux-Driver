# SMI230 Input Sensor-API and Sensor Driver

## Table of Contents
 - [Introduction](#Intro)
 - [License](#License)
 - [Sensor interfaces](#interfaces)
 - [Architecture](#Architecture)
 - [Continuous Integration](#Continuous-Integration)
 - [Operation examples](#examples)

## Introduction <a name=Intro></a>

SMI230 is a system-in-package inertial measurement unit which offers accurate acceleration and angular rate measurements.
Due to system-in-package approach of SMI230 (two sensors in single package), the gyroscope and acceleration data is acquired in a non-synchronized manner.
However, synchronization between accelerometer and gyroscope can be achieved:
The software modules in this repository are provided as reference for SMI230 users and shall demonstrate exemplarily the usage of the following features:
- Data synchronization.
- Data collection from FIFO.

_Note: SMI230 is rather an Industrial I/O (IIO) than an input device. Therefore, we now implement SMI230 as an IIO driver (see [GitHub](https://github.com/boschmemssolutions/SMI230-Linux-Driver-IIO)). Further development will only happen on the IIO driver, not on the input driver anymore._

_Note: The sensor driver utilizes a sensor API, which is following BMI08x's sensor API, available on [GitHub](https://github.com/BoschSensortec/BMI08x-Sensor-API/releases/tag/bmi08x_v1.4.4)._

_Note: The data synchronization feature utilizes the sensor's configuration, which is following BMI08x sensor configuration, available on [GitHub](https://github.com/BoschSensortec/BMI08x-Sensor-API/releases/tag/bmi08x_v1.2.0)._

## License <a name=License></a>
See [LICENSE](drivers/input/sensors/smi230/LICENSE.md) file

## Sensor interfaces <a name=interfaces></a>
* I2C
* SPI

## Architecture <a name=Architecture></a>
```
                  User space
-------------------------------------------------------
                 |          |
               sysfs       dev
                 \          /
               input-subsystem
                      |
sensor_API <-- smi230_driver --> smi230_SPI/I2C_driver
                                           |
                                      SPI/I2C_bus
                                           |
-------------------------------------------------------
                  Hardware
```
## Operation examples <a name=examples></a>
1. Userspace
The driver exposes a device file node under /dev/input/event*, which can be read as a normal Linux file. Tools like evtest can also be used for read data out. E.g.:
```
sudo evtest /dev/input/event0
```
The data will be displayed on the console with timestamp.

2. Sysfs
The driver also exposes a set of sysfs nodes under `/sys/devices/virtual/input/input*`, where users can get information about the sensor and also control the sensor. E.g.:
```
# read the acc power config
cat /sys/devices/virtual/input/SMI230ACC/pwr_cfg

# set the acc power config active, this command is needed if acc needs to be fully fuctional.
echo 0 > /sys/devices/virtual/input/SMI230ACC/pwr_cfg

# set the acc power config suspend
echo 3 > /sys/devices/virtual/input/SMI230ACC/pwr_cfg

# read the gyro power config
cat /sys/devices/virtual/input/SMI230GYRO/pwr_cfg

# set the gyro power config active, this command is needed if gyro needs to be fully fuctional.
echo 0 > /sys/devices/virtual/input/SMI230GYRO/pwr_cfg

# set the gyro power config suspend
echo 3 > /sys/devices/virtual/input/SMI230GYRO/pwr_cfg

# read the chip id
cat /sys/devices/virtual/input/SMI230GYRO/chip_id

# read the synced acc data
cat /sys/devices/virtual/input/SMI230ACC/data_sync

# read the asynced acc data
cat /sys/devices/virtual/input/SMI230ACC/acc_value

# read the gyro data
cat /sys/devices/virtual/input/SMI230GYRO/gyro_value

```
