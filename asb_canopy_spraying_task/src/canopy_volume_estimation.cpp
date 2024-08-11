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

#include "canopy_volume_estimation/canopy_volume_estimation.h"
#include "pcl_conversions/pcl_conversions.h"
#include "octomap_msgs/conversions.h"
#include "octomap_ros/conversions.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

CanopyVolumeEstimation::CanopyVolumeEstimation() : Node("canopy_volume_estimation") {

  res_ = declare_parameter("resolution", 0.05);
  max_range_ = declare_parameter("max_range", 10.0);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  points_in_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "points_in", rclcpp::SensorDataQoS().durability_volatile().best_effort(),
    std::bind(&CanopyVolumeEstimation::points_in_callback, this, _1));

  initialize_canopy_region_service_ = this->create_service<InitializeCanopyRegion>(
    "initialize_canopy_region", std::bind(&CanopyVolumeEstimation::initialize_canopy_region, this, _1, _2));

  canopy_data_publisher_ = this->create_publisher<CanopyData>(
    "canopy_data", rclcpp::SensorDataQoS().reliable().transient_local());

  viz_publisher_ = this->create_publisher<MarkerArray>(
    "canopy_visualization_markers", rclcpp::SensorDataQoS().reliable().transient_local());

}

void CanopyVolumeEstimation::add_viz_marker(size_t marker_id, Header header, double size, double x, double y_min, double y_max, double z) {
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

bool CanopyVolumeEstimation::transform_region_of_interest(Header target_header) {

  if(roi_.frame_id.empty()) {
    RCLCPP_WARN(this->get_logger(), "No region of interest was received yet");
    return false;
  }

  PointStamped p_1;
  p_1.header.frame_id = roi_.frame_id;
  p_1.header.stamp = target_header.stamp;
  p_1.point.x = roi_.x_1;

  PointStamped p_2;
  p_2.header.frame_id = roi_.frame_id;
  p_2.header.stamp = target_header.stamp;
  p_2.point.x = roi_.x_2;

  PointStamped p_1_transformed, p_2_transformed;
  p_1_transformed.header = target_header;
  p_2_transformed.header = target_header;

  try {
    tf2::doTransform(p_1, p_1_transformed, tf_buffer_->lookupTransform(
      p_1_transformed.header.frame_id, p_1.header.frame_id, tf2::TimePointZero));
    tf2::doTransform(p_2, p_2_transformed, tf_buffer_->lookupTransform(
      p_2_transformed.header.frame_id, p_2.header.frame_id, tf2::TimePointZero));
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "Transform Exception: %s", ex.what());
    return false;
  } catch (std::exception &ex) {
    RCLCPP_WARN(this->get_logger(), "Exception: %s", ex.what());
    return false;
  }

  roi_transformed_.frame_id = target_header.frame_id;
  roi_transformed_.x_1 = p_1_transformed.point.x;
  roi_transformed_.x_2 = p_2_transformed.point.x;

  return true;
}

void CanopyVolumeEstimation::initialize_canopy_region(const std::shared_ptr<InitializeCanopyRegion::Request> request, std::shared_ptr<InitializeCanopyRegion::Response> response) {
  canopy_id_ = request->canopy_id;
  canopy_frame_id_ = request->canopy_frame_id;
  point_cloud_min_x_ = request->min_x;
  point_cloud_max_x_ = request->max_x;
  point_cloud_min_y_ = request->min_y;
  point_cloud_max_y_ = request->max_y;
  point_cloud_min_z_ = request->min_z;
  point_cloud_max_z_ = request->max_z;
  roi_ = request->roi;
  octree_ = std::make_unique<OcTree>(res_);

  response->result = true;
}

void CanopyVolumeEstimation::points_in_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud) {
//  const auto start_time = rclcpp::Clock{}.now();

  if(canopy_id_.empty()) {
    return;
  }

  PCLPointCloud pc;
  pcl::fromROSMsg(*cloud, pc);

  geometry_msgs::msg::TransformStamped sensor_to_canopy_transform_stamped;
  try {
    sensor_to_canopy_transform_stamped = tf_buffer_->lookupTransform(
      canopy_frame_id_, cloud->header.frame_id, cloud->header.stamp,
      rclcpp::Duration::from_seconds(0.1));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "%s", ex.what());
    return;
  }

  // set up filter for height range, also removes NANs:
  pcl::PassThrough<PCLPoint> pass_x;
  pass_x.setFilterFieldName("x");
  pass_x.setFilterLimits((float)point_cloud_min_x_, (float)point_cloud_max_x_);
  pcl::PassThrough<PCLPoint> pass_y;
  pass_y.setFilterFieldName("y");
  pass_y.setFilterLimits((float)point_cloud_min_y_, (float)point_cloud_max_y_);
  pcl::PassThrough<PCLPoint> pass_z;
  pass_z.setFilterFieldName("z");
  pass_z.setFilterLimits((float)point_cloud_min_z_, (float)point_cloud_max_z_);

  // directly transform to canopy frame:
  pcl_ros::transformPointCloud(pc, pc, sensor_to_canopy_transform_stamped);

  // just filter height range:
  pass_x.setInputCloud(pc.makeShared());
  pass_x.filter(pc);
  pass_y.setInputCloud(pc.makeShared());
  pass_y.filter(pc);
  pass_z.setInputCloud(pc.makeShared());
  pass_z.filter(pc);

  const auto & t = sensor_to_canopy_transform_stamped.transform.translation;
  tf2::Vector3 sensor_origin_tf{t.x, t.y, t.z};
  const auto sensor_origin = octomap::pointTfToOctomap(sensor_origin_tf);
  for (auto& it : pc) {
    octomap::point3d point(it.x, it.y, it.z);
    if ((point - sensor_origin).norm() <= max_range_) {
      octomap::OcTreeKey key;
      if (octree_->coordToKeyChecked(point, key)) octree_->updateNode(key, true);
    }
  }

//  double total_elapsed = (rclcpp::Clock{}.now() - start_time).seconds();
//  RCLCPP_INFO(get_logger(), "inserted %zu points in %.3f s", pc.size(), total_elapsed);

  update_canopy_volume(cloud->header.stamp);

}

void CanopyVolumeEstimation::update_canopy_volume(const rclcpp::Time & ros_time) {

  bool publish_viz_marker_array = viz_publisher_->get_subscription_count() > 0;

  // expand the tree to make sure all leaf nodes are the same size (resolution)
  octree_->expand();

  CanopyData canopy_data_msg;
  canopy_data_msg.header.frame_id = canopy_frame_id_;
  canopy_data_msg.header.stamp = ros_time;
  canopy_data_msg.resolution = (float)octree_->getResolution();

  double bb_x_min, bb_y_min, bb_z_min, bb_x_max, bb_y_max, bb_z_max;
  octree_->getMetricMin(bb_x_min, bb_y_min, bb_z_min);
  octree_->getMetricMax(bb_x_max, bb_y_max, bb_z_max);

  Header roi_header = Header();
  roi_header.frame_id = canopy_frame_id_;
  roi_header.stamp = ros_time;
  bool roi_result = transform_region_of_interest(roi_header);
  if(!roi_result) return;

  canopy_data_msg.roi = roi_transformed_;

  // sort the region of interest x_1, x_2 values so that x_min <= x_max, otherwise the bounding box will be considered empty
  bb_x_min = std::min(roi_transformed_.x_1, roi_transformed_.x_2);
  bb_x_max = std::max(roi_transformed_.x_1, roi_transformed_.x_2);

  // collect y values for each x, z coordinate
  auto bbx_min = octomap::point3d((float)bb_x_min, (float)bb_y_min, (float)bb_z_min);
  auto bbx_max = octomap::point3d((float)bb_x_max, (float)bb_y_max, (float)bb_z_max);
  std::map<std::pair<double, double>, std::vector<double>> y_vector_map;
  for (auto v = octree_->begin_leafs_bbx(bbx_min, bbx_max), end = octree_->end_leafs_bbx(); v != end; ++v) {
    if(octree_->isNodeOccupied(*v)) {
      auto x_z = std::pair(v.getX(), v.getZ());
      y_vector_map[x_z].emplace_back(v.getY());
    }
  }

  if(publish_viz_marker_array) {
    if (y_vector_map.size() > viz_marker_array_.markers.size()) {
      viz_marker_array_.markers.resize(y_vector_map.size());
    }
  }

  // compute y_depth for each x, z coordinate
  // Note: adding voxel_length to y_depth because y_depth is computed from the centroids of the voxels (otherwise the
  // volume of a 1-voxel deep region would be 0 m^3)
  double voxel_length = octree_->getResolution();
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
      add_viz_marker(marker_id, roi_header, voxel_length, x, *y_vector_min, *y_vector_max, z);
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

  canopy_data_publisher_->publish(canopy_data_msg);

}
