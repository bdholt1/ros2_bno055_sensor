# ros2_bno055_sensor
A ROS2 driver for the [BNO055 IMU sensor](https://cdn-shop.adafruit.com/datasheets/BST_BNO055_DS000_12.pdf) (Bosch Sensortec) 

This ROS2 driver communicates with the sensor over I2C and relies only on `libi2c-dev`.

## Building from source
This driver depends on the [Bosch Sensortec API](https://github.com/BoschSensortec/BNO055_driver), but that is unfortunately C only, so a patch is required to support C linkage with the C++ ROS2 driver.

```
$ git clone --recurse-submodules https://github.com/bdholt1/ros2_bno055_sensor.git
$ cd ros2_bno055_sensor/thirdparty/BNO055_driver/
$ git apply ../../bno055.h.patch
$ cd ../../../
$ colcon build
```

## Usage
Publish IMU messages using default parameters
```
$ ros2 run bno055_sensor bno055_sensor_node
```

# Nodes

## bno055_sensor_node

## Published topics
* `/imu/raw` - `sensor_msgs/Imu`

Raw gyro and linear acceleration data

* `/imu/data` - `sensor_msgs/Imu`

Fusion data (requires NDOF mode)

* `/mag` - `sensor_msgs/MagneticField`

Magnetic field


* `temp` -  `sensor_msgs/Temperature`

Temperature

## Parameters
* `device` - `string`

The path to the i2c device (default is /dev/i2c-3)

* `address` - `string`

The i2c address of the BNO055 (default is 0x28)

# Why another BNO055 driver?  
Bosch Sensortec have released [a package](https://github.com/BoschSensortec/BNO055_driver) for communicating with the BNO055. This abstracts away the exact register addresses and communiction details, requiring only that the user supply 3 functions (I2C read, I2c write, and delay). No other ROS drivers make use of this package.

There is [already](https://github.com/flynneva/bno055) a ROS2 Driver for the BN055 but I2C is not supported, only UART.

Reference ROS1 BNO055 drivers:
* https://github.com/mdrwiega/bosch_imu_driver (UART only)
* https://github.com/dheera/ros-imu-bno055 (I2C only, uses SMBus).
* https://github.com/joeyjyyang/ros_bno055 (I2C only, uses SMBus).

Other drivers:
https://github.com/fm4dd/pi-bno055

# Usage on Raspberry Pi

You will need user access to the i2c devices. Do this by adding your user to the i2c group `sudo usermod -aG i2c ${USER}`.

All Raspberry Pi devices have a [clock-stretching bug](http://www.advamation.com/knowhow/raspberrypi/rpi-i2c-bug.html) that affects hardware i2c. To resolve this, either [slow down the clock](https://learn.adafruit.com/circuitpython-on-raspberrypi-linux/i2c-clock-stretching) or [switch to software i2c](https://github.com/fivdi/i2c-bus/blob/master/doc/raspberry-pi-software-i2c.md). Software i2c will increase the CPU utilisation of this driver.

Note that the advice provided is targeted to Rasbian, editing `/boot/config.txt` as the [boot configuration file](https://www.raspberrypi.org/documentation/configuration/config-txt/README.md).  This driver is targeted at ROS2 on Ubuntu which uses `/boot/firmware/config.txt` for the [boot configuration file](https://wiki.ubuntu.com/ARM/RaspberryPi), however it is recommended to use `/boot/firmware/usercfg.txt` for [user config](https://www.raspberrypi.org/forums/viewtopic.php?t=241485#p1828300) instead.

In summary, to enable software i2c on `/dev/i2c-3`, add the following line to `/boot/firmware/usercfg.txt` on the Raspberry Pi and reboot.

```
dtoverlay=i2c-gpio,bus=3
```

Test your i2c setup as follows

```
$ sudo apt install i2c-tools
$ i2cdetect -y 3
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:          -- -- -- -- -- -- -- -- -- -- -- -- -- 
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
20: -- -- -- -- -- -- -- -- 28 -- -- -- -- -- -- -- 
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
70: -- -- -- -- -- -- -- --
```



