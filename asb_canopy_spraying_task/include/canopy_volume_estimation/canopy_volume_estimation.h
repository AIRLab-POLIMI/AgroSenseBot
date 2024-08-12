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

#ifndef CANOPY_VOLUME_ESTIMATION_CANOPY_VOLUME_ESTIMATION_H
#define CANOPY_VOLUME_ESTIMATION_CANOPY_VOLUME_ESTIMATION_H

#include "rclcpp/rclcpp.hpp"
#include "asb_msgs/msg/canopy_data.hpp"
#include "asb_msgs/msg/canopy_data_array.hpp"
#include "asb_msgs/msg/canopy_region_of_interest.hpp"
#include "asb_msgs/srv/initialize_canopy_region.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/transforms.hpp"

#include "octomap/OcTree.h"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

using namespace std::chrono_literals;
using PCLPoint = pcl::PointXYZ;
using PCLPointCloud = pcl::PointCloud<pcl::PointXYZ>;
using octomap::OcTree;
using std_msgs::msg::Header;
using asb_msgs::msg::CanopyData;
using asb_msgs::msg::CanopyDataArray;
using asb_msgs::msg::CanopyRegionOfInterest;
using asb_msgs::srv::InitializeCanopyRegion;
using geometry_msgs::msg::PointStamped;
using sensor_msgs::msg::PointCloud2;
using visualization_msgs::msg::MarkerArray;
using visualization_msgs::msg::Marker;

class CanopyMap {
public:
  std::string canopy_id;
  std::string canopy_frame_id;
  double point_cloud_min_x, point_cloud_max_x;
  double point_cloud_min_y, point_cloud_max_y;
  double point_cloud_min_z, point_cloud_max_z;
  CanopyRegionOfInterest roi, roi_transformed;
  std::unique_ptr<OcTree> octree;
  MarkerArray viz_marker_array;
};

class CanopyVolumeEstimation : public rclcpp::Node
{
public:
  CanopyVolumeEstimation();

private:

  void initialize_canopy_region(const std::shared_ptr<InitializeCanopyRegion::Request> request, std::shared_ptr<InitializeCanopyRegion::Response> response);
  void points_in_callback(const sensor_msgs::msg::PointCloud2::SharedPtr points_in_msg);

  bool transform_region_of_interest(const CanopyRegionOfInterest& roi, const Header& target_header, CanopyRegionOfInterest& roi_transformed);
  void update_canopy_volume(CanopyMap& canopy_map, CanopyData& canopy_data_msg, const rclcpp::Time & ros_time);
  static void add_viz_marker(CanopyMap& canopy_map, size_t marker_id, Header header, double size, double x, double y_min, double y_max, double z);

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  rclcpp::Subscription<PointCloud2>::SharedPtr points_in_subscriber_;

  rclcpp::Service<InitializeCanopyRegion>::SharedPtr initialize_canopy_region_service_;

  rclcpp::Publisher<CanopyDataArray>::SharedPtr canopy_data_array_publisher_;
  rclcpp::Publisher<MarkerArray>::SharedPtr viz_publisher_;

  // node parameters
  double res_;
  double max_range_;
  int hit_count_threshold_;
  bool print_timing_;

  std::vector<CanopyMap> canopy_maps;

};


#endif //CANOPY_VOLUME_ESTIMATION_CANOPY_VOLUME_ESTIMATION_H
