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

#include "asb_lidar_filter/asb_lidar_filter.h"

using std::placeholders::_1;

ASBLidarFilter::ASBLidarFilter() : Node("asb_lidar_filter") {

  points_in_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "points_in", rclcpp::SensorDataQoS().durability_volatile().best_effort(),
    std::bind(&ASBLidarFilter::points_in_callback, this, _1));

  points_out_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "points_out", rclcpp::SensorDataQoS().durability_volatile().best_effort());

  //  TODO get footprint box

}

void ASBLidarFilter::points_in_callback(const sensor_msgs::msg::PointCloud2::SharedPtr points_in_msg)
{
  RCLCPP_INFO(this->get_logger(), "received point cloud. frame_id: %s", points_in_msg->header.frame_id.c_str());

//  TODO transform footprint box
//  TODO apply box filter

  points_out_publisher_->publish(*points_in_msg);
}
