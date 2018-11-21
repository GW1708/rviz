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
#define _USE_MATH_DEFINES
#include <cmath>

#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "std_msgs/msg/header.hpp"



using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses a fancy C++11 lambda
 * function to shorten the callback syntax, at the expense of making the
 * code somewhat more difficult to understand at first glance. */

class TemperaturePublisher : public rclcpp::Node
{
public:
    TemperaturePublisher() : Node("temperature_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<sensor_msgs::msg::Temperature>("temperature_topic");

      auto timer_callback =
              [this]() -> void {
                  auto message = sensor_msgs::msg::Temperature();

                  message.header = std_msgs::msg::Header();
                  message.header.frame_id = "point_frame";
                  message.header.stamp = rclcpp::Clock().now();

                  message.temperature = counter_;
                  message.variance = 1.0;

                  RCLCPP_INFO(this->get_logger(), "Temperature Test");
                  this->publisher_->publish(message);
              };
      timer_ = this->create_wall_timer(750ms, timer_callback);
      counter_++;
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr publisher_;
    double counter_ = 1.;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TemperaturePublisher>());
  rclcpp::shutdown();
  return 0;
}