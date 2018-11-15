// Copyright 2016 Open Source Robotics Foundation, Inc.
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
//#define _USE_MATH_DEFINES
//#include <cmath>
//#include <iostream>

#ifndef RVIZ_DEFAULT_PLUGINS__PUBLISHERS__POINT_CLOUD2_PUBLISHER_HPP_
#define RVIZ_DEFAULT_PLUGINS__PUBLISHERS__POINT_CLOUD2_PUBLISHER_HPP_

#include <string>

#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "std_msgs/msg/header.hpp"

#include "../pointcloud_messages.hpp"

using namespace std::chrono_literals; //NOLINT

namespace nodes {

sensor_msgs::msg::Temperature createTemperature(float temp, float var)
{
  sensor_msgs::msg::Temperature temperature;
  temperature.temperature = temp;
  temperature.variance = var;

  return temperature;
}

class TemperaturePublisher : public rclcpp::Node
{
public:
    TemperaturePublisher()
    : Node("temperature_publisher"),
      count_(0)
    {
      publisher_ = this->create_publisher<sensor_msgs::msg::Temperature>("temperature");

      auto timer_callback =
              [this]() -> void {
                  auto message = sensor_msgs::msg::Temperature();

                  message.header = std_msgs::msg::Header();
                  message.header.frame_id = "temperature_frame";
                  message.header.stamp = rclcpp::Clock().now();

                  message.temperature = 20.0;
                  message.variance = 1.0;

                  //RCLCPP_INFO(this->get_logger(), "Temperature Test");
                  this->publisher_->publish(message);
              };
      timer_ = this->create_wall_timer(500ms, timer_callback);
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr publisher_;
    size_t count_;
};

} // namespace nodes

#endif  // RVIZ_DEFAULT_PLUGINS__PUBLISHERS__POINT_CLOUD2_PUBLISHER_HPP_