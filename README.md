# ros2_bno055_sensor
A ROS2 driver for the BNO055 sensor 

[Datasheet](https://cdn-shop.adafruit.com/datasheets/BST_BNO055_DS000_12.pdf)

Why is this required?  
There is already a ROS2 Driver for the BN055: https://github.com/flynneva/bno055 but it's in python and I2C is not supported.

Reference ROS1 BNO055 drivers:
https://github.com/mdrwiega/bosch_imu_driver (UART only)
https://github.com/dheera/ros-imu-bno055 (I2C only, uses SMBus). 
https://github.com/joeyjyyang/ros_bno055 (I2C only, uses SMBus).

Other drivers:
https://github.com/fm4dd/pi-bno055 (low level I2C using unix read()/write() on the /dev/i2c-1 file descriptor). 
Really [fast](https://github.com/fm4dd/pi-bno055/pull/4), doesn't seem to exhibit the RPi hardware clock stretching problem (not sure why). 

https://github.com/BoschSensortec/BNO055_driver: the official Bosch Sensortec driver, requires that the user implement their own I2C communication.

Useful references
[Linux Kernel on I2C in userspace](https://www.kernel.org/doc/html/latest/i2c/dev-interface.html)
