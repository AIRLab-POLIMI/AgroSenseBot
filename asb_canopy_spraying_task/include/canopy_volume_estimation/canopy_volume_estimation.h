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
#include "asb_msgs/msg/canopy_region_of_interest.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "octomap_msgs/msg/octomap.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "octomap/OcTree.h"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

using namespace std::chrono_literals;
using octomap::OcTree;
using octomap::AbstractOcTree;
using std_msgs::msg::Header;
using asb_msgs::msg::CanopyData;
using asb_msgs::msg::CanopyRegionOfInterest;
using geometry_msgs::msg::PointStamped;
using octomap_msgs::msg::Octomap;
using visualization_msgs::msg::MarkerArray;
using visualization_msgs::msg::Marker;

class CanopyVolumeEstimation : public rclcpp::Node
{
public:
  CanopyVolumeEstimation();

private:

  void add_viz_marker(size_t marker_id, Header header, double size, double x, double y_min, double y_max, double z);
  bool transform_region_of_interest(Header target_header);

  void canopy_region_of_interest_callback(const CanopyRegionOfInterest::SharedPtr roi_msg);
  void octomap_callback(const Octomap::SharedPtr octomap_msg);

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  CanopyRegionOfInterest roi_, roi_transformed_;
  MarkerArray viz_marker_array_ = MarkerArray();

  rclcpp::Publisher<CanopyData>::SharedPtr canopy_data_publisher_;
  rclcpp::Publisher<MarkerArray>::SharedPtr viz_publisher_;
  rclcpp::Subscription<Octomap>::SharedPtr octomap_subscriber_;
  rclcpp::Subscription<CanopyRegionOfInterest>::SharedPtr roi_subscriber_;

};


#endif //CANOPY_VOLUME_ESTIMATION_CANOPY_VOLUME_ESTIMATION_H
