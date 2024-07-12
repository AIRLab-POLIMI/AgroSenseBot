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

#include "asb_spraying_task/asb_spraying_task.h"
#include "octomap_msgs/conversions.h"

using std::placeholders::_1;

ASBSprayingTask::ASBSprayingTask() : Node("asb_spraying_task") {

  octomap_subscriber_ = this->create_subscription<Octomap>(
    "octomap_full", rclcpp::SensorDataQoS().reliable().transient_local(),
    std::bind(&ASBSprayingTask::octomap_callback, this, _1));

  canopy_density_publisher_ = this->create_publisher<CanopyDensity>(
    "canopy_density", rclcpp::SensorDataQoS().reliable().transient_local());

}

void ASBSprayingTask::octomap_callback(const octomap_msgs::msg::Octomap::SharedPtr octomap_msg) {
  std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

  auto* abstract_octree = octomap_msgs::msgToMap(*octomap_msg);
  if(!abstract_octree) {
    RCLCPP_ERROR(this->get_logger(), "Error creating octree from received message.");
    return;
  }

  auto* octree = dynamic_cast<OcTree*>(abstract_octree);
  if (!octree){
    RCLCPP_ERROR(this->get_logger(), "Error creating octree from received message.");
    return;
  }

  double x_min, y_min, z_min, x_max, y_max, z_max;
  octree->getMetricMin(x_min, y_min, z_min);
  octree->getMetricMax(x_max, y_max, z_max);
  int i_min = floor(x_min/canopy_density_res_);
  int i_max = ceil(x_max/canopy_density_res_);

  RCLCPP_INFO(
    this->get_logger(), "Map received, %zu nodes, %.2f m res, x_min: %f, x_max: %f, y_min: %f, y_max: %f, z_min: %f, z_max: %f, i_min: %i, i_max: %i",
    octree->size(),
    octree->getResolution(),
    x_min,
    x_max,
    y_min,
    y_max,
    z_min,
    z_max,
    i_min,
    i_max
    );

  float res = (float) octree->getResolution();
  double leaf_volume = pow(res, 3);

  CanopyDensity canopy_density_msg;
  canopy_density_msg.header = octomap_msg->header;
  for(int i = i_min; i <= i_max; i++) {
    float x_1 = (float)i * res;
    float x_2 = x_1 + res;
    float x = x_1 + res/2;

    auto bbx_min = octomap::point3d(x_1, (float)y_min, (float)z_min);
    auto bbx_max = octomap::point3d(x_2, (float)y_max, (float)z_max);

    if(octree->begin() == octree->end()) {
      RCLCPP_INFO(this->get_logger(), "slice is empty");
    }

    long n = 0;
    double v = 0.0;
    for (auto it = octree->begin_leafs_bbx(bbx_min, bbx_max), end = octree->end_leafs_bbx(); it != end; ++it) {
      if(octree->isNodeOccupied(*it)) n++;
      v += it->getOccupancy() * leaf_volume;
    }
    canopy_density_msg.x_array.emplace_back(x);
    canopy_density_msg.y_array.emplace_back(v);
  }

  delete octree;

  canopy_density_publisher_->publish(canopy_density_msg);

  std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
  RCLCPP_INFO(this->get_logger(), "%f", (std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count()) / 1000000.0);
}
