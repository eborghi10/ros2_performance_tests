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

#ifndef COMPOSITION__TALKER_COMPONENT_HPP_
#define COMPOSITION__TALKER_COMPONENT_HPP_

#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace throughput_test
{
class Talker : public rclcpp::Node
{
public:
  explicit Talker(const rclcpp::NodeOptions & options);

protected:
  void on_timer();

private:
  const size_t cameras_{ 5 };
  std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> color_publishers_;
  std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> depth_publishers_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_sub_;
};

}  // namespace throughput_test

#endif  // COMPOSITION__TALKER_COMPONENT_HPP_
