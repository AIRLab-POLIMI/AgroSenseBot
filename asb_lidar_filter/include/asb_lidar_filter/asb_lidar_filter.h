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
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

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

  void points_in_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_in_subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_out_publisher_;
};


#endif //ASB_LIDAR_FILTER_ASB_LIDAR_FILTER_H
