# ros2_bno055_sensor
A ROS2 driver for the [BNO055 IMU sensor](https://cdn-shop.adafruit.com/datasheets/BST_BNO055_DS000_12.pdf) (Bosch Sensortec) 

This ROS2 driver communicates with the sensor over I2C and relies only on `libi2c-dev`.

## Building from source
This driver depends on the [Bosch Sensortec API](https://github.com/BoschSensortec/BNO055_driver), but that is unfortunately C only, so a patch is required to support C linkage with the C++ ROS2 driver.

```
$ git clone --recurse-submodules https://github.com/bdholt1/ros2_bno055_sensor.git
$ cd thirdparty/BNO055_driver/
$ git apply ../../bno055.h.patch
$ cd ../../
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
The path to the i2c device (default is /dev/i2c-1)

* `address` - `string`
The i2c addres of the BNO055 (default is 0x28)

# Why another BNO055 driver?  
Bosch Sensortec have released [an API](https://github.com/BoschSensortec/BNO055_driver) for communicating with the BNO055. This abstracts away the exact register addresses and communiction details, requiring only that the user supply 3 functions (I2C read, I2c write, and delay). No other ROS drivers make use of this API.

There is [already](https://github.com/flynneva/bno055) a ROS2 Driver for the BN055 but I2C is not supported, only UART.

Reference ROS1 BNO055 drivers:
https://github.com/mdrwiega/bosch_imu_driver (UART only)
https://github.com/dheera/ros-imu-bno055 (I2C only, uses SMBus). 
https://github.com/joeyjyyang/ros_bno055 (I2C only, uses SMBus).

Other drivers:
https://github.com/fm4dd/pi-bno055 (low level I2C using unix read()/write() on the /dev/i2c-1 file descriptor). 
Really [fast](https://github.com/fm4dd/pi-bno055/pull/4), doesn't seem to exhibit the RPi hardware clock stretching problem (not sure why). 

Useful references
[Linux Kernel on I2C in userspace](https://www.kernel.org/doc/html/latest/i2c/dev-interface.html)



