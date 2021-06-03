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

#include <cmath>

using namespace std::chrono_literals;

namespace bno055_sensor
{

BNO055Sensor::BNO055Sensor(rclcpp::NodeOptions const & options)
:  Node("bno055_sensor_node"), count_(0)
{

  imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
  mag_publisher_ = this->create_publisher<sensor_msgs::msg::MagneticField>("mag", 10);
  temp_publisher_ = this->create_publisher<sensor_msgs::msg::Temperature>("temp", 10);
  timer_ = this->create_wall_timer(
  10ms, std::bind(&BNO055Sensor::timer_callback, this));

  std::string i2c_addr("/dev/i2c-1");
  std::string dev_addr("0x28");
  int retval = init_i2cbus(i2c_addr.c_str(), dev_addr.c_str());
  sensor_.bus_read = BNO055_I2C_bus_read;
  sensor_.bus_write = BNO055_I2C_bus_write;
  sensor_.delay_msec = BNO055_delay_msek;
  sensor_.dev_addr = BNO055_I2C_ADDR1;

  //TODO(brian): check the result and print an error
  s32 comres = BNO055_SUCCESS;
  comres += bno055_init(&sensor_);

  // set the power mode as NORMAL
  comres += bno055_set_power_mode(BNO055_POWER_MODE_NORMAL);

  // set operation mode as NDOF
  comres += bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF);

  if (comres != 0)
  {
    RCLCPP_WARN(this->get_logger(), "Error setting up BNO055 sensor");
  }
  
}
  
BNO055Sensor::~BNO055Sensor()
{
  // set the power mode as SUSPEND
  bno055_set_power_mode(BNO055_POWER_MODE_SUSPEND);

  close_i2cbus();
}

void BNO055Sensor::timer_callback()
{
  s32 comres = BNO055_SUCCESS;
  
  u8 mag_calib_status;
  u8 accel_calib_status;
  u8 gyro_calib_status;
  u8 sys_calib_status;

  bno055_quaternion_t quaternion_wxyz;
  bno055_gyro_double_t d_gyro_xyz;
  bno055_euler_double_t d_euler_hpr;
  bno055_linear_accel_double_t d_linear_accel_xyz;
  bno055_gravity_double_t d_gravity_xyz;
  bno055_mag_double_t d_mag_xyz;
  double d_temp;

  // get the status variables
  comres += bno055_get_mag_calib_stat(&mag_calib_status);
  comres += bno055_get_accel_calib_stat(&accel_calib_status);
  comres += bno055_get_gyro_calib_stat(&gyro_calib_status);
  comres += bno055_get_sys_calib_stat(&sys_calib_status);

  /*
  comres += bno055_read_gyro_xyz(&gyro_xyz);
  comres += bno055_read_euler_hrp(&euler_hrp);
  comres += bno055_read_linear_accel_xyz(&linear_accel_xyz);
  comres += bno055_read_gravity_xyz(&gravity_xyz);
  */

  // read the fusion data
  comres += bno055_read_quaternion_wxyz(&quaternion_wxyz);
  
  // read fusion data and convert the data into SI units
  comres += bno055_convert_double_gyro_xyz_dps(&d_gyro_xyz);
  comres += bno055_convert_double_euler_hpr_deg(&d_euler_hpr);
  comres += bno055_convert_double_linear_accel_xyz_msq(&d_linear_accel_xyz);
  comres += bno055_convert_double_gravity_xyz_msq(&d_gravity_xyz);
  comres += bno055_convert_double_mag_xyz_uT(&d_mag_xyz);
  comres += bno055_convert_double_temp_celsius(&d_temp);

  if (comres != 0)
  {
    RCLCPP_WARN(this->get_logger(), "Error reading BNO055 data");
  }

  double quaternion_norm = std::sqrt(
    quaternion_wxyz.w * quaternion_wxyz.w +
    quaternion_wxyz.x * quaternion_wxyz.x +
    quaternion_wxyz.y * quaternion_wxyz.y +
    quaternion_wxyz.z * quaternion_wxyz.z);

  auto time_stamp = now();

  auto imu_msg = sensor_msgs::msg::Imu();
  imu_msg.header.stamp = time_stamp;
  imu_msg.header.frame_id = std::string("imu_base_link");
  imu_msg.orientation.x = quaternion_wxyz.x / quaternion_norm;
  imu_msg.orientation.y = quaternion_wxyz.y / quaternion_norm;
  imu_msg.orientation.z = quaternion_wxyz.z / quaternion_norm;
  imu_msg.orientation.w = quaternion_wxyz.w / quaternion_norm;
  imu_msg.angular_velocity.x = d_gyro_xyz.x;
  imu_msg.angular_velocity.y = d_gyro_xyz.y;
  imu_msg.angular_velocity.z = d_gyro_xyz.z;
  imu_msg.linear_acceleration.x = d_linear_accel_xyz.x;
  imu_msg.linear_acceleration.y = d_linear_accel_xyz.y;
  imu_msg.linear_acceleration.z = d_linear_accel_xyz.z;

  auto mag_msg = sensor_msgs::msg::MagneticField();
  mag_msg.header.stamp = time_stamp;
  mag_msg.header.frame_id = std::string("imu_base_link");
  mag_msg.magnetic_field.x = d_mag_xyz.x;
  mag_msg.magnetic_field.y = d_mag_xyz.y;
  mag_msg.magnetic_field.z = d_mag_xyz.z;

  auto temp_msg = sensor_msgs::msg::Temperature();
  temp_msg.header.stamp = time_stamp;
  temp_msg.header.frame_id = std::string("imu_base_link");
  temp_msg.temperature = d_temp;
  
  imu_publisher_->publish(imu_msg);
  mag_publisher_->publish(mag_msg);
  temp_publisher_->publish(temp_msg);

  count_++;
}

} // namespace bno055_sensor  