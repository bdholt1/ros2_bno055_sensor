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

#ifndef BNO055_SENSOR__BNO055_SENSOR_HPP_
#define BNO055_SENSOR__BNO055_SENSOR_HPP_

#include "bno055.h"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"
//#include "sensor_msgs/msg/magnetic_field.hpp"
//#include "sensor_msgs/msg/temperature.hpp"

namespace bno055_sensor
{

class BNO055Sensor : public rclcpp::Node
{
public:
  explicit BNO055Sensor(rclcpp::NodeOptions const & options);

  virtual ~BNO055Sensor();

private:
  void timer_callback();

  bno055_t sensor_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  size_t count_;  
};

}  // namespace bno055_sensor


#endif // BNO055_SENSOR__BNO055_SENSOR_HPP_
