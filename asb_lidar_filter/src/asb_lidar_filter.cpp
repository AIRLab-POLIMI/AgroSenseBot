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

#include "rclcpp/duration.hpp"
#include "asb_lidar_filter/asb_lidar_filter.h"
#include <pcl/point_types.h>

#include "pcl/filters/crop_box.h"
#include "pcl/filters/conditional_removal.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/transforms.hpp"

#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "sensor_msgs/point_cloud2_iterator.hpp"

using std::placeholders::_1;

ASBLidarFilter::ASBLidarFilter() : Node("asb_lidar_filter") {

  this->declare_parameter("base_frame_id", "base_footprint");
  base_frame_id_ = this->get_parameter("base_frame_id").as_string();

  // pointcloud filter params
  this->declare_parameter("x_min", 0.0);
  this->declare_parameter("x_max", 0.0);
  this->declare_parameter("y_min", 0.0);
  this->declare_parameter("y_max", 0.0);
  this->declare_parameter("z_min", 0.0);
  this->declare_parameter("z_max", 0.0);
  x_min_ = this->get_parameter("x_min").as_double();
  x_max_ = this->get_parameter("x_max").as_double();
  y_min_ = this->get_parameter("y_min").as_double();
  y_max_ = this->get_parameter("y_max").as_double();
  z_min_ = this->get_parameter("z_min").as_double();
  z_max_ = this->get_parameter("z_max").as_double();

  // scan filter params
  this->declare_parameter("scan_min_height", 0.0);
  scan_min_height_ = this->get_parameter("scan_min_height").as_double();
  this->declare_parameter("scan_max_height", 0.0);
  scan_max_height_ = this->get_parameter("scan_max_height").as_double();

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  points_in_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "points_in", rclcpp::SensorDataQoS().durability_volatile().reliable(),
    std::bind(&ASBLidarFilter::points_in_callback, this, _1));

  points_out_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "points_out", rclcpp::SensorDataQoS().durability_volatile().reliable());

  scan_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
    "scan_out", rclcpp::SensorDataQoS().durability_volatile().reliable());

}

void ASBLidarFilter::points_in_callback(const sensor_msgs::msg::PointCloud2::SharedPtr points_in_msg) {
  typedef pcl::PointXYZ PointType;

  auto points_out = std::make_shared<pcl::PointCloud<PointType>>();
  pcl::fromROSMsg(*points_in_msg, *points_out);

  geometry_msgs::msg::TransformStamped sensor_to_base_transform_stamped;
  try {
    tf_buffer_->canTransform(base_frame_id_, points_in_msg->header.frame_id, points_in_msg->header.stamp, rclcpp::Duration::from_seconds(0.05));
    sensor_to_base_transform_stamped = tf_buffer_->lookupTransform(base_frame_id_, points_in_msg->header.frame_id, points_in_msg->header.stamp, rclcpp::Duration::from_seconds(0.05));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "Transform Exception: %s", ex.what());
  }
  pcl_ros::transformPointCloud(*points_out, *points_out, sensor_to_base_transform_stamped);
  points_out->header.frame_id = base_frame_id_;

  // filter the input pointcloud with a box modelling the space occupied by the robot (remove points inside the box)
  pcl::CropBox<PointType> crop_box_filter;
  crop_box_filter.setNegative(true);
  crop_box_filter.setInputCloud(points_out);
  crop_box_filter.setMin(Eigen::Vector4f(
    (float)std::min(x_min_, x_max_),
    (float)std::min(y_min_, y_max_),
    (float)std::min(z_min_, z_max_),
    1.0
  ));
  crop_box_filter.setMax(Eigen::Vector4f(
    (float)std::max(x_min_, x_max_),
    (float)std::max(y_min_, y_max_),
    (float)std::max(z_min_, z_max_),
    1.0
  ));
  crop_box_filter.filter(*points_out);

  sensor_msgs::msg::PointCloud2::SharedPtr points_out_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
  points_out_msg->header = points_in_msg->header;
  pcl::toROSMsg(*points_out, *points_out_msg);
  points_out_publisher_->publish(*points_out_msg);

  // Create LaserScan message
  auto scan_msg = std::make_unique<sensor_msgs::msg::LaserScan>();
  scan_msg->header = points_out_msg->header;

  scan_msg->angle_min = -M_PI;
  scan_msg->angle_max = M_PI;
  scan_msg->angle_increment = M_PI / (5 * 180.0);
  scan_msg->time_increment = 0.0;
  scan_msg->scan_time = 1.0 / 30.0;
  scan_msg->range_min = 0.0;
  scan_msg->range_max = 30.0;

  uint32_t ranges_size = std::ceil((scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment);
  scan_msg->ranges.assign(ranges_size, std::numeric_limits<float>::infinity());

  for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*points_out_msg, "x"), iter_y(*points_out_msg, "y"), iter_z(*points_out_msg, "z"); iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z)) continue;
    if (*iter_z > scan_max_height_ || *iter_z < scan_min_height_) continue;

    float range = hypot(*iter_x, *iter_y);
    double angle = atan2(*iter_y, *iter_x);
    int i = (int)std::round((angle - scan_msg->angle_min) / scan_msg->angle_increment);

    if (range < scan_msg->ranges[i]) scan_msg->ranges[i] = range;
  }

  scan_publisher_->publish(std::move(scan_msg));

}
