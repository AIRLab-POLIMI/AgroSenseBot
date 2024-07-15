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

#ifndef ASB_SPRAYING_TASK_ASB_SPRAYING_TASK_H
#define ASB_SPRAYING_TASK_ASB_SPRAYING_TASK_H

#include "rclcpp/rclcpp.hpp"
#include "octomap/OcTree.h"
#include "asb_msgs/msg/canopy_data.hpp"
#include "octomap_msgs/msg/octomap.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

using namespace std::chrono_literals;
using octomap::OcTree;
using octomap::AbstractOcTree;
using asb_msgs::msg::CanopyData;
using octomap_msgs::msg::Octomap;
using std_msgs::msg::Header;
using visualization_msgs::msg::MarkerArray;
using visualization_msgs::msg::Marker;

class ASBSprayingTask : public rclcpp::Node
{
public:
  ASBSprayingTask();

private:

  void add_viz_marker(size_t marker_id, Header header, double size, double x, double y_min, double y_max, double z);
  void octomap_callback(const Octomap::SharedPtr octomap_msg);

  MarkerArray viz_marker_array_ = MarkerArray();

  rclcpp::Subscription<Octomap>::SharedPtr octomap_subscriber_;
  rclcpp::Publisher<CanopyData>::SharedPtr canopy_data_publisher_;
  rclcpp::Publisher<MarkerArray>::SharedPtr viz_publisher_;

};


#endif //ASB_SPRAYING_TASK_ASB_SPRAYING_TASK_H
