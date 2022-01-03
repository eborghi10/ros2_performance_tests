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

#include "throughput_test/listener_component.hpp"

#include <boost/format.hpp>    // only needed for printing
#include <boost/histogram.hpp>
#include <cassert>
#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "realsense2_camera_msgs/msg/extrinsics.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"


namespace throughput_test
{
namespace bh = boost::histogram;
using namespace bh::literals;  //enables _c suffix

Listener::Listener(const rclcpp::NodeOptions & options)
: Node("listener", options)
, color_last_time_(now())
, depth_last_time_(now())
, color_latency_data_(CAMERAS_)
, depth_latency_data_(CAMERAS_)
, color_throughput_data_(CAMERAS_)
, depth_throughput_data_(CAMERAS_)
{
  // manually enable topic statistics via options
  auto sub_options = rclcpp::SubscriptionOptions();
  sub_options.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;
  // configure the collection window and publish period (default 1s)
  sub_options.topic_stats_options.publish_period = std::chrono::seconds(10);

  auto color_camera_info_callback =
    [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr /*msg*/) -> void
    {};

  auto depth_camera_info_callback =
    [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr /*msg*/) -> void
    {};

  auto point_cloud_callback =
    [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr /*msg*/) -> void
    {};

  auto extrinsics_callback =
    [this](realsense2_camera_msgs::msg::Extrinsics::ConstSharedPtr /*msg*/) -> void
    {};

  for (std::size_t i = 0; i < CAMERAS_; i++)
  {
    std::function<void(sensor_msgs::msg::Image::ConstSharedPtr msg)> image_cb_fcn =
        std::bind(&Listener::color_image_callback, this, std::placeholders::_1, i);
    std::function<void(sensor_msgs::msg::Image::ConstSharedPtr msg)> depth_cb_fcn =
        std::bind(&Listener::depth_image_callback, this, std::placeholders::_1, i);

    color_image_sub_.push_back(create_subscription<sensor_msgs::msg::Image>(
        "image_" + std::to_string(i), rclcpp::SensorDataQoS(), image_cb_fcn));
    depth_image_sub_.push_back(create_subscription<sensor_msgs::msg::Image>(
        "depth_" + std::to_string(i), rclcpp::SensorDataQoS(), depth_cb_fcn));
  }
  color_camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>("color_camera_info", rclcpp::SensorDataQoS(), color_camera_info_callback);
  depth_camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>("depth_camera_info", rclcpp::SensorDataQoS(), depth_camera_info_callback);
  point_cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>("pointcloud", rclcpp::SensorDataQoS(), point_cloud_callback);
  extrinsics_sub_ = create_subscription<realsense2_camera_msgs::msg::Extrinsics>("extrinsics", rclcpp::SensorDataQoS(), extrinsics_callback);
}

void Listener::generate_histogram(const std::vector<double>& data, const std::string& label,
                                  const double max, const std::size_t bins, const double min)
{
  auto h = bh::make_histogram(bh::axis::regular<double>(bins, min, max, label));
  std::for_each(data.begin(), data.end(), std::ref(h));
  std::ostringstream header;
  header << "[" << label << "][" << get_name() << "]---------------------------------";
  std::cout << header.str() << std::endl;
  std::ostringstream os;
  for (auto&& x : bh::indexed(h, bh::coverage::all)) {
    os << boost::format("bin %2i [%4.1f, %4.1f): %i\n")
          % x.index() % x.bin().lower() % x.bin().upper() % *x;
  }
  std::cout << os.str() << std::flush;
}

void Listener::color_image_callback(sensor_msgs::msg::Image::ConstSharedPtr msg, const std::size_t i)
{
  std::lock_guard<std::mutex> lock{ color_mutex_ };
  const rclcpp::Time current_time = now();
  const double latency = to_milliseconds(current_time - msg->header.stamp);
  color_latency_data_[i].push_back(latency);
  // RCLCPP_INFO_STREAM(this->get_logger(), "Latency: " << latency << " [ms]");

  const double throughput = to_milliseconds(current_time - color_last_time_);
  color_last_time_ = current_time;
  color_throughput_data_[i].push_back(throughput);
  // RCLCPP_INFO_STREAM(this->get_logger(), "Throughput: " << throughput << " [ms]");

  // std::flush(std::cout);
}

void Listener::depth_image_callback(sensor_msgs::msg::Image::ConstSharedPtr msg, const std::size_t i)
{
  std::lock_guard<std::mutex> lock{ depth_mutex_ };
  const rclcpp::Time current_time = now();
  const double latency = to_milliseconds(current_time - msg->header.stamp);
  depth_latency_data_[i].push_back(latency);

  const double throughput = to_milliseconds(current_time - depth_last_time_);
  depth_last_time_ = current_time;
  depth_throughput_data_[i].push_back(throughput);
}

Listener::~Listener()
{
  for (std::size_t i = 0; i < CAMERAS_; i++)
  {
    {
      std::lock_guard<std::mutex> lock{ color_mutex_ };
      generate_histogram(color_latency_data_[i], std::to_string(i) + " - Color Latency", 300.0);
      generate_histogram(color_throughput_data_[i], std::to_string(i) + " - Color Throughput", 100.0);
    }
    {
      std::lock_guard<std::mutex> lock{ depth_mutex_ };
      generate_histogram(depth_latency_data_[i], std::to_string(i) + " - Depth Latency", 300.0);
      generate_histogram(depth_throughput_data_[i], std::to_string(i) + " - Depth Throughput", 100.0);
    }
  }
}

}  // namespace throughput_test

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(throughput_test::Listener)
