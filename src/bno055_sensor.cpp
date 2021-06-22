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
  imu_raw_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/raw", 10);
  imu_data_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
  gravity_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("gravity", 10);
  mag_publisher_ = this->create_publisher<sensor_msgs::msg::MagneticField>("mag", 10);
  temp_publisher_ = this->create_publisher<sensor_msgs::msg::Temperature>("temp", 10);
  diagnostics_publisher_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("diagnostics", 10);
  data_timer_ = this->create_wall_timer(10ms, std::bind(&BNO055Sensor::publish_data, this));
  diagnostics_timer_ = this->create_wall_timer(1000ms, std::bind(&BNO055Sensor::publish_diagnostics, this));

  this->declare_parameter<std::string>("i2c_address", "/dev/i2c-3");
  this->declare_parameter<std::string>("device_address", "0x28");
  this->declare_parameter<std::string>("frame_id", "imu_link");

  sensor_.bus_read = BNO055_I2C_bus_read;
  sensor_.bus_write = BNO055_I2C_bus_write;
  sensor_.delay_msec = BNO055_delay_msek;

  initialise();
}

BNO055Sensor::~BNO055Sensor()
{
  // set the power mode as SUSPEND
  bno055_set_power_mode(BNO055_POWER_MODE_SUSPEND);

  close_i2cbus();
}

void BNO055Sensor::initialise()
{
  std::string i2c_addr;
  this->get_parameter("i2c_address", i2c_addr);

  std::string dev_addr;
  this->get_parameter("device_address", dev_addr);

  sensor_.dev_addr = BNO055_I2C_ADDR1; //TODO convert dev_addr from string to byte
  int retval = init_i2cbus(i2c_addr.c_str(), dev_addr.c_str());

  s32 comres = BNO055_SUCCESS;
  comres += bno055_init(&sensor_);

  // set the power mode as NORMAL
  comres += bno055_set_power_mode(BNO055_POWER_MODE_NORMAL);

  // set operation mode as NDOF
  comres += bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF);

  if (comres != 0)
  {
    RCLCPP_FATAL(this->get_logger(), "Error setting up BNO055 sensor");
  }
}

void BNO055Sensor::publish_data()
{
  s32 comres = BNO055_SUCCESS;

  u8 system_status;
  u8 sys_calib_status;
  
  bno055_gyro_t gyro_xyz;
  bno055_linear_accel_t linear_accel_xyz;

  bno055_quaternion_t quaternion_wxyz;
  bno055_gyro_double_t d_gyro_xyz;
  bno055_euler_double_t d_euler_hpr;
  bno055_linear_accel_double_t d_linear_accel_xyz;
  bno055_gravity_double_t d_gravity_xyz;
  bno055_mag_double_t d_mag_xyz;
  double d_temp;

  comres += bno055_get_sys_stat_code(&system_status);
  comres += bno055_get_sys_calib_stat(&sys_calib_status);

  // read the raw data
  comres += bno055_read_gyro_xyz(&gyro_xyz);
  comres += bno055_read_linear_accel_xyz(&linear_accel_xyz);
  comres += bno055_convert_double_temp_celsius(&d_temp);

  // read the fusion data
  comres += bno055_read_quaternion_wxyz(&quaternion_wxyz);
  
  // read fusion data and convert the data into SI units
  comres += bno055_convert_double_gyro_xyz_dps(&d_gyro_xyz);
  comres += bno055_convert_double_linear_accel_xyz_msq(&d_linear_accel_xyz);
  comres += bno055_convert_double_gravity_xyz_msq(&d_gravity_xyz);
  comres += bno055_convert_double_mag_xyz_uT(&d_mag_xyz);


  if (comres != 0 || system_status == 1)
  {
    RCLCPP_FATAL(this->get_logger(), "Error reading BNO055 data");
    return;
  }

  double quaternion_norm = std::sqrt(
    quaternion_wxyz.w * quaternion_wxyz.w +
    quaternion_wxyz.x * quaternion_wxyz.x +
    quaternion_wxyz.y * quaternion_wxyz.y +
    quaternion_wxyz.z * quaternion_wxyz.z);

  auto time_stamp = now();

  std::string frame_id;
  this->get_parameter("frame_id", frame_id);

  auto imu_raw_msg = sensor_msgs::msg::Imu();
  imu_raw_msg.header.stamp = time_stamp;
  imu_raw_msg.header.frame_id = frame_id;
  imu_raw_msg.orientation.x = 0;
  imu_raw_msg.orientation.y = 0;
  imu_raw_msg.orientation.z = 0;
  imu_raw_msg.orientation.w = 0;
  imu_raw_msg.angular_velocity.x = gyro_xyz.x;
  imu_raw_msg.angular_velocity.y = gyro_xyz.y;
  imu_raw_msg.angular_velocity.z = gyro_xyz.z;
  imu_raw_msg.linear_acceleration.x = linear_accel_xyz.x;
  imu_raw_msg.linear_acceleration.y = linear_accel_xyz.y;
  imu_raw_msg.linear_acceleration.z = linear_accel_xyz.z;

  auto temp_msg = sensor_msgs::msg::Temperature();
  temp_msg.header.stamp = time_stamp;
  temp_msg.header.frame_id = frame_id;
  temp_msg.temperature = d_temp;

  imu_raw_publisher_->publish(imu_raw_msg);
  temp_publisher_->publish(temp_msg);

  if (sys_calib_status == 0)
  {
    RCLCPP_WARN(this->get_logger(), "Fusion data is not reliable as system is not calibrated");
    return;
  }

  auto imu_data_msg = sensor_msgs::msg::Imu();
  imu_data_msg.header.stamp = time_stamp;
  imu_data_msg.header.frame_id = frame_id;
  imu_data_msg.orientation.x = quaternion_wxyz.x / quaternion_norm;
  imu_data_msg.orientation.y = quaternion_wxyz.y / quaternion_norm;
  imu_data_msg.orientation.z = quaternion_wxyz.z / quaternion_norm;
  imu_data_msg.orientation.w = quaternion_wxyz.w / quaternion_norm;
  imu_data_msg.angular_velocity.x = d_gyro_xyz.x;
  imu_data_msg.angular_velocity.y = d_gyro_xyz.y;
  imu_data_msg.angular_velocity.z = d_gyro_xyz.z;
  imu_data_msg.linear_acceleration.x = d_linear_accel_xyz.x;
  imu_data_msg.linear_acceleration.y = d_linear_accel_xyz.y;
  imu_data_msg.linear_acceleration.z = d_linear_accel_xyz.z;

  auto gravity_msg = geometry_msgs::msg::Vector3Stamped();
  gravity_msg.header.stamp = time_stamp;
  gravity_msg.header.frame_id = frame_id;
  gravity_msg.vector.x = d_gravity_xyz.x;
  gravity_msg.vector.y = d_gravity_xyz.y;
  gravity_msg.vector.z = d_gravity_xyz.z;

  auto mag_msg = sensor_msgs::msg::MagneticField();
  mag_msg.header.stamp = time_stamp;
  mag_msg.header.frame_id = frame_id;
  mag_msg.magnetic_field.x = d_mag_xyz.x;
  mag_msg.magnetic_field.y = d_mag_xyz.y;
  mag_msg.magnetic_field.z = d_mag_xyz.z;

  imu_data_publisher_->publish(imu_data_msg);
  gravity_publisher_->publish(gravity_msg);
  mag_publisher_->publish(mag_msg);

  count_++;
}

std::string BNO055Sensor::system_status_as_string(u8 system_status)
{
  std::string retval;
  switch (system_status)
  {
    case 0: retval = "System idle"; break;
    case 1: retval = "System Error"; break;
    case 2: retval = "Initializing peripherals"; break;
    case 3: retval = "System Initialization"; break;
    case 4: retval = "Executing Selftest"; break;
    case 5: retval = "Sensor fusion algorithm running"; break;
    case 6: retval = "System running without fusion algorithm"; break;
  }
  return retval;
}

std::string BNO055Sensor::system_error_as_string(u8 system_error)
{
  std::string retval;
  switch (system_error)
  {
    case 0: retval = "No Error"; break;
    case 1: retval = "Peripheral initialization error"; break;
    case 2: retval = "System initialization error"; break;
    case 3: retval = "Selftest result failed"; break;
    case 4: retval = "Register map value out of range"; break;
    case 5: retval = "Register map address out of range"; break;
    case 6: retval = "Register map write error"; break;
    case 7: retval = "BNO low power mode not  available for selected operation mode"; break;
    case 8: retval = "Accelerometer power mode not available"; break;
    case 9: retval = "Fusion algorithm configuration error"; break;
  }
  return retval;
}

void BNO055Sensor::publish_diagnostics()
{
  try
  {
    u8 system_status;
    u8 system_error;
    u8 sys_calib_status;
    u8 gyro_calib_status;
    u8 accel_calib_status;
    u8 mag_calib_status;

    // get the status variables
    s32 comres = BNO055_SUCCESS;

    auto diagnostic_msg = diagnostic_msgs::msg::DiagnosticStatus();
    diagnostic_msg.name = "BNO055";

    comres += bno055_get_sys_stat_code(&system_status);
    comres += bno055_get_sys_error_code(&system_error);
    comres += bno055_get_sys_calib_stat(&sys_calib_status);
    comres += bno055_get_gyro_calib_stat(&gyro_calib_status);
    comres += bno055_get_accel_calib_stat(&accel_calib_status);
    comres += bno055_get_mag_calib_stat(&mag_calib_status);

    if (comres != BNO055_SUCCESS)
    {
      diagnostic_msg.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      diagnostic_msg.message = "Disconnected";
      diagnostic_msg.values.resize(0);
    }
    else if (system_status == 1)
    {
      diagnostic_msg.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      diagnostic_msg.message = "BNO055 System Error";
      diagnostic_msg.values.resize(1);
      diagnostic_msg.values[0].key = "System Error";
      diagnostic_msg.values[0].value = system_error_as_string(system_error);
    }
    else
    {
      diagnostic_msg.values.resize(5);
      diagnostic_msg.values[0].key = "System Status";
      diagnostic_msg.values[0].value = system_status_as_string(system_status);
      diagnostic_msg.values[1].key = "Calibration (SYS)";
      diagnostic_msg.values[1].value = std::to_string(sys_calib_status);
      diagnostic_msg.values[2].key = "Calibration (GYR)";
      diagnostic_msg.values[2].value = std::to_string(gyro_calib_status);
      diagnostic_msg.values[3].key = "Calibration (ACC)";
      diagnostic_msg.values[3].value = std::to_string(accel_calib_status);
      diagnostic_msg.values[4].key = "Calibration (MAG)";
      diagnostic_msg.values[4].value = std::to_string(mag_calib_status);

      if (sys_calib_status < 1 || gyro_calib_status < 1 || accel_calib_status < 1 || mag_calib_status < 1)
      {
	diagnostic_msg.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
	diagnostic_msg.message = "Poorly Calibrated";
      }
      else
      {
	diagnostic_msg.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
	diagnostic_msg.message = "Operating Normally";
      }
    }

    diagnostics_publisher_->publish(diagnostic_msg);

  } catch (std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to read sensor and publish diagnostics: %s", e.what());
  }
}

} // namespace bno055_sensor  
