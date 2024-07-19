// Copyright 2024 Enrico Piazza, Università degli Studi di Milano
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

#ifndef ASB_LIDAR_FILTER_ASB_LIDAR_FILTER_H
#define ASB_LIDAR_FILTER_ASB_LIDAR_FILTER_H

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

using namespace std::chrono_literals;

class ASBLidarFilter : public rclcpp::Node
{
public:
  ASBLidarFilter();

private:

  void points_in_callback_xyz(const sensor_msgs::msg::PointCloud2::SharedPtr points_in_msg);
  void points_in_callback_os(const sensor_msgs::msg::PointCloud2::SharedPtr points_in_msg);

  bool transform_box(const sensor_msgs::msg::PointCloud2::SharedPtr points_in_msg,
                     geometry_msgs::msg::PointStamped* p_min_t,
                     geometry_msgs::msg::PointStamped* p_max_t);

  std::string point_type_, frame_id_;
  bool apply_range_min_filter_, apply_range_max_filter_, apply_box_filter_;
  float range_min_, range_max_;
  double x_min_, x_max_, y_min_, y_max_, z_min_, z_max_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_in_subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_out_publisher_;
};


#endif //ASB_LIDAR_FILTER_ASB_LIDAR_FILTER_H
