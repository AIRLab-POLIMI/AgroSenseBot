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

  canopy_data_publisher_ = this->create_publisher<CanopyData>(
    "canopy_data", rclcpp::SensorDataQoS().reliable().transient_local());

  viz_publisher_ = this->create_publisher<MarkerArray>(
    "canopy_visualization_markers", rclcpp::SensorDataQoS().reliable().transient_local());

  octomap_subscriber_ = this->create_subscription<Octomap>(
    "octomap_full", rclcpp::SensorDataQoS().reliable().transient_local(),
    std::bind(&ASBSprayingTask::octomap_callback, this, _1));

}

void ASBSprayingTask::add_viz_marker(size_t marker_id, Header header, double size, double x, double y_min, double y_max, double z) {
  viz_marker_array_.markers[marker_id].pose.position.x = x;
  viz_marker_array_.markers[marker_id].pose.position.y = (y_min + y_max) / 2;
  viz_marker_array_.markers[marker_id].pose.position.z = z;
  viz_marker_array_.markers[marker_id].pose.orientation.w = 1;
  viz_marker_array_.markers[marker_id].header = header;
  viz_marker_array_.markers[marker_id].ns = "canopy";
  viz_marker_array_.markers[marker_id].id = (int)marker_id;
  viz_marker_array_.markers[marker_id].type = Marker::CUBE;
  viz_marker_array_.markers[marker_id].action = Marker::ADD;
  viz_marker_array_.markers[marker_id].scale.x = size;
  viz_marker_array_.markers[marker_id].scale.y = size + y_max - y_min;
  viz_marker_array_.markers[marker_id].scale.z = size;
  viz_marker_array_.markers[marker_id].color.r = 0.1;
  viz_marker_array_.markers[marker_id].color.g = 0.7;
  viz_marker_array_.markers[marker_id].color.b = 0.1;
  viz_marker_array_.markers[marker_id].color.a = 1.0;
}

void ASBSprayingTask::octomap_callback(const Octomap::SharedPtr octomap_msg) {

  bool publish_viz_marker_array = viz_publisher_->get_subscription_count() > 0;

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

  // expand the tree to make sure all leaf nodes are the same size (resolution)
  octree->expand();

  CanopyData canopy_data_msg;
  canopy_data_msg.header = octomap_msg->header;
  canopy_data_msg.resolution = (float)octree->getResolution();

  double bb_x_min, bb_y_min, bb_z_min, bb_x_max, bb_y_max, bb_z_max;
  octree->getMetricMin(bb_x_min, bb_y_min, bb_z_min);
  octree->getMetricMax(bb_x_max, bb_y_max, bb_z_max);

//  bb_x_min = 1.0; // TODO get region of interest from somewhere
//  bb_x_max = 3.0; // TODO get region of interest from somewhere

  // collect y values for each x, z coordinate
  auto bbx_min = octomap::point3d((float)bb_x_min, (float)bb_y_min, (float)bb_z_min);
  auto bbx_max = octomap::point3d((float)bb_x_max, (float)bb_y_max, (float)bb_z_max);
  std::map<std::pair<double, double>, std::vector<double>> y_vector_map;
  for (auto v = octree->begin_leafs_bbx(bbx_min, bbx_max), end = octree->end_leafs_bbx(); v != end; ++v) {
    if(octree->isNodeOccupied(*v)) {
      auto x_z = std::pair(v.getX(), v.getZ());
      y_vector_map[x_z].emplace_back(v.getY());
    }
  }

  if(y_vector_map.size() > viz_marker_array_.markers.size()) {
    viz_marker_array_.markers.resize(y_vector_map.size());
  }

  // compute y_depth for each x, z coordinate
  // Note: adding voxel_length to y_depth because y_depth is computed from the centroids of the voxels (otherwise the
  // volume of a 1-voxel deep region would be 0 m^3)
  double voxel_length = octree->getResolution();
  std::map<std::pair<double, double>, double> y_depth_map;
  int marker_id = 0;
  for(auto [x_z, y_vector]: y_vector_map) {
    auto const &[x, z] = x_z;
    const auto [y_vector_min, y_vector_max] = std::minmax_element(std::begin(y_vector), std::end(y_vector));
    y_depth_map[x_z] = voxel_length + *y_vector_max - *y_vector_min;

    canopy_data_msg.depth_x_array.emplace_back(x);
    canopy_data_msg.depth_y_array.emplace_back(y_depth_map[x_z]);
    canopy_data_msg.depth_z_array.emplace_back(z);

    if(publish_viz_marker_array) {
      add_viz_marker(marker_id, octomap_msg->header, voxel_length, x, *y_vector_min, *y_vector_max, z);
      marker_id++;
    }
  }

  if(publish_viz_marker_array) {
    for(size_t further_marker_id = marker_id; further_marker_id < viz_marker_array_.markers.size(); further_marker_id++) {
      viz_marker_array_.markers[further_marker_id].action = Marker::DELETE;
    }
    viz_publisher_->publish(viz_marker_array_);
  }

  // compute volume for each x coordinate
  double voxel_area = pow(voxel_length, 2);
  std::map<double, double> x_volume_map;
  for(auto [x_z, y_depth]: y_depth_map) {
    auto const &x = x_z.first;
    x_volume_map[x] += y_depth * voxel_area;
  }

  for(auto [x, volume]: x_volume_map) {
    canopy_data_msg.volume_x_array.emplace_back(x);
    canopy_data_msg.volume_y_array.emplace_back(volume);
  }

  delete octree;
  canopy_data_publisher_->publish(canopy_data_msg);

}
