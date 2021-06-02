// Copyright 2021 Brian Holt
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "bno055_sensor/bno055_sensor.hpp"

#include "bno055_sensor/bno055_i2c.h"

using namespace std::chrono_literals;

namespace bno055_sensor
{

BNO055Sensor::BNO055Sensor(rclcpp::NodeOptions const & options)
:  Node("bno055_sensor_node"), count_(0)
{

  imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
  timer_ = this->create_wall_timer(
  500ms, std::bind(&BNO055Sensor::timer_callback, this));

  std::string i2c_addr("/dev/i2c-1");
  std::string dev_addr("0x28");
  int retval = init_i2cbus(i2c_addr.c_str(), dev_addr.c_str());
  sensor_.bus_read = BNO055_I2C_bus_read;
  sensor_.bus_write = BNO055_I2C_bus_write;
  sensor_.delay_msec = BNO055_delay_msek;
  sensor_.dev_addr = BNO055_I2C_ADDR1;

  //TODO(brian): check the result and print an error
  bno055_init(&sensor_);

  /* set the power mode as NORMAL*/
  bno055_set_power_mode(BNO055_POWER_MODE_NORMAL);
  
  bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF);
}
  
BNO055Sensor::~BNO055Sensor()
{
    /*  For de - initializing the BNO sensor it is required
     * to the operation mode of the sensor as SUSPEND
     * Suspend mode can set from the register
     * Page - page0
     * register - 0x3E
     * bit positions - 0 and 1*/
    u8 power_mode = BNO055_POWER_MODE_SUSPEND;

    /* set the power mode as SUSPEND*/
    bno055_set_power_mode(power_mode);

    close_i2cbus();
}

void BNO055Sensor::timer_callback()
{
  RCLCPP_INFO(this->get_logger(), "timer_callback");
  
  u8 mag_calib_status;
  u8 accel_calib_status;
  u8 gyro_calib_status;
  u8 sys_calib_status;

  bno055_get_mag_calib_stat(&mag_calib_status);  
  bno055_get_accel_calib_stat(&accel_calib_status);
  bno055_get_gyro_calib_stat(&gyro_calib_status);
  bno055_get_sys_calib_stat(&sys_calib_status);

  bno055_euler_t euler_hrp;
  bno055_read_euler_hrp(&euler_hrp);

  bno055_quaternion_t quaternion;
  bno055_read_quaternion(&quaternion);  
  

  //auto message = std_msgs::msg::String();
  //message.data = " " + std::to_string(sys_calib_status) +
  //" " + std::to_string(gyro_calib_status) +
  //" " + std::to_string(accel_calib_status) +
  //" " + std::to_string(mag_calib_status) +
  //" " + std::to_string(euler_hrp.h) +
  //" " + std::to_string(euler_hrp.r) +
  //" " + std::to_string(euler_hrp.p);
  //RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());

  auto imu_msg = sensor_msgs::msg::Imu();
  imu_msg.header.stamp = now();
  imu_msg.header.frame_id = std::string("imu_base_link");

  imu_msg.orientation.x = bno055_driver_.data_.qua_x_; 
  imu_msg.orientation.y = bno055_driver_.data_.qua_y_;
  imu_msg.orientation.z = bno055_driver_.data_.qua_z_;
  imu_msg.orientation.w = bno055_driver_.data_.qua_w_;
  imu_msg.angular_velocity.x = bno055_driver_.data_.gyr_x_;
  imu_msg.angular_velocity.y = bno055_driver_.data_.gyr_y_;
  imu_msg.angular_velocity.z = bno055_driver_.data_.gyr_z_;
  imu_msg.linear_acceleration.x = bno055_driver_.data_.acc_x_;
  imu_msg.linear_acceleration.y = bno055_driver_.data_.acc_y_;
  imu_msg.linear_acceleration.z = bno055_driver_.data_.acc_z_;
    
  //  mag_msg_.header.stamp = time_stamp;
  //  mag_msg_.magnetic_field.x = bno055_driver_.data_.mag_x_;
  //  mag_msg_.magnetic_field.y = bno055_driver_.data_.mag_y_;
  //  mag_msg_.magnetic_field.z = bno055_driver_.data_.mag_z_;

  //  temp_msg_.header.stamp = time_stamp;
  //  temp_msg_.temperature = bno055_driver_.data_.temp_;
  
  
  imu_publisher_->publish(message);  
}

} // namespace bno055_sensor  
