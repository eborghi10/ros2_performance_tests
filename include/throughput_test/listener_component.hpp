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

#ifndef COMPOSITION__LISTENER_COMPONENT_HPP_
#define COMPOSITION__LISTENER_COMPONENT_HPP_

#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/subscription_options.hpp"
#include "realsense2_camera_msgs/msg/extrinsics.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"


namespace throughput_test
{

class Listener : public rclcpp::Node
{
public:
  explicit Listener(const rclcpp::NodeOptions & options);
  ~Listener();

private:
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> color_image_sub_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> depth_image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr color_camera_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr depth_camera_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
  rclcpp::Subscription<realsense2_camera_msgs::msg::Extrinsics>::SharedPtr extrinsics_sub_;

  void color_image_callback(sensor_msgs::msg::Image::ConstSharedPtr msg, const std::size_t i);
  void depth_image_callback(sensor_msgs::msg::Image::ConstSharedPtr msg, const std::size_t i);

  double to_milliseconds(const rclcpp::Duration& d){ return d.nanoseconds() * 1e-6; };
  double to_microseconds(const rclcpp::Duration& d){ return d.nanoseconds() * 1e-3; };

  void generate_histogram(const std::vector<double>& data, const std::string& label, const double max, const std::size_t bins = 10, const double min = 0.0);

  const std::size_t CAMERAS_{ 5 };
  rclcpp::Time color_last_time_, depth_last_time_;
  std::vector<std::vector<double>> color_latency_data_, depth_latency_data_;
  std::vector<std::vector<double>> color_throughput_data_, depth_throughput_data_;
  std::mutex color_mutex_, depth_mutex_;
};

}  // namespace throughput_test

#endif  // COMPOSITION__LISTENER_COMPONENT_HPP_
