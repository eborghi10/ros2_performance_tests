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

#include "throughput_test/talker_component.hpp"

#include <chrono>
#include <iostream>
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace std::chrono_literals;

namespace throughput_test
{
Talker::Talker(const rclcpp::NodeOptions & options)
: Node("talker", options)
{
  for (size_t i = 0; i < cameras_; i++)
  {
    // auto color_pub = create_publisher<sensor_msgs::msg::Image>("image_raw_" + i, rclcpp::SensorDataQoS());
    // color_publishers_.push_back(color_pub);
    // auto depth_pub = create_publisher<sensor_msgs::msg::Image>("depth_raw_" + i, rclcpp::SensorDataQoS());
    // depth_publishers_.push_back(depth_pub);
    color_publishers_.push_back(create_publisher<sensor_msgs::msg::Image>("image_" + std::to_string(i), rclcpp::SensorDataQoS()));
    depth_publishers_.push_back(create_publisher<sensor_msgs::msg::Image>("depth_" + std::to_string(i), rclcpp::SensorDataQoS()));
  }

  auto color_image_callback =
    [this](sensor_msgs::msg::Image::ConstSharedPtr msg) -> void
    {
      for (size_t i = 0; i < cameras_; i++)
      {
        // color_publishers_[i]->publish(std::move(msg));
        color_publishers_[i]->publish(*msg);
      }
    };

  auto depth_image_callback =
    [this](sensor_msgs::msg::Image::ConstSharedPtr msg) -> void
    {
      for (size_t i = 0; i < cameras_; i++)
      {
        // depth_publishers_[i]->publish(std::move(msg));
        depth_publishers_[i]->publish(*msg);
      }
    };

  color_image_sub_ = create_subscription<sensor_msgs::msg::Image>("image_raw", rclcpp::SensorDataQoS(), color_image_callback);
  depth_image_sub_ = create_subscription<sensor_msgs::msg::Image>("depth_raw", rclcpp::SensorDataQoS(), depth_image_callback);
}

}  // namespace throughput_test

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(throughput_test::Talker)
