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
#include "asb_msgs/msg/canopy_density.hpp"
#include "octomap_msgs/msg/octomap.hpp"
#include "octomap/OcTree.h"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

using namespace std::chrono_literals;
using octomap::OcTree;
using octomap::AbstractOcTree;
using octomap_msgs::msg::Octomap;
using asb_msgs::msg::CanopyDensity;

class ASBSprayingTask : public rclcpp::Node
{
public:
  ASBSprayingTask();

private:

  void octomap_callback(const Octomap::SharedPtr octomap_msg);

  rclcpp::Subscription<Octomap>::SharedPtr octomap_subscriber_;
  rclcpp::Publisher<CanopyDensity>::SharedPtr canopy_density_publisher_;

  float canopy_density_res_ = 0.05;
};


#endif //ASB_SPRAYING_TASK_ASB_SPRAYING_TASK_H
