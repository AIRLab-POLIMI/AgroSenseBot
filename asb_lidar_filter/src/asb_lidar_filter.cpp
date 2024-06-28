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
#include "asb_lidar_filter/os_point.h"

#include "pcl/filters/crop_box.h"
#include "pcl/filters/conditional_removal.h"
#include "pcl_conversions/pcl_conversions.h"

#include "geometry_msgs/msg/point_stamped.hpp"

#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using std::placeholders::_1;

ASBLidarFilter::ASBLidarFilter() : Node("asb_lidar_filter") {

  this->declare_parameter("apply_range_min_filter", false);
  apply_range_min_filter_ = this->get_parameter("apply_range_min_filter").as_bool();
  this->declare_parameter("range_min", -1.0);
  range_min_ = (float)this->get_parameter("range_min").as_double();

  this->declare_parameter("apply_range_max_filter", false);
  apply_range_max_filter_ = this->get_parameter("apply_range_max_filter").as_bool();
  this->declare_parameter("range_max", -1.0);
  range_max_ = (float)this->get_parameter("range_max").as_double();

  this->declare_parameter("apply_box_filter", false);
  apply_box_filter_ = this->get_parameter("apply_box_filter").as_bool();
  this->declare_parameter("frame_id", "base_footprint");
  frame_id_ = this->get_parameter("frame_id").as_string();
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

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  points_in_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "points_in", rclcpp::SensorDataQoS().durability_volatile().best_effort(),
    std::bind(&ASBLidarFilter::points_in_callback, this, _1));

  points_out_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "points_out", rclcpp::SensorDataQoS().durability_volatile().best_effort());

}

void ASBLidarFilter::points_in_callback(const sensor_msgs::msg::PointCloud2::SharedPtr points_in_msg) {

  // it is also possible to use pcl::PointXYZ from pcl/point_types.h, but the additional fields will be discarded
  typedef asb_ouster_ros::Point PointType;

  auto points_out = std::make_shared<pcl::PointCloud<PointType>>();
  pcl::fromROSMsg(*points_in_msg, *points_out);

  if(apply_box_filter_) {
    // Transform the min and max box points into the point cloud frame
    geometry_msgs::msg::PointStamped p_min;
    p_min.header.frame_id = frame_id_;
    p_min.header.stamp = points_in_msg->header.stamp;
    p_min.point.x = x_min_;
    p_min.point.y = y_min_;
    p_min.point.z = z_min_;

    geometry_msgs::msg::PointStamped p_max;
    p_max.header.frame_id = frame_id_;
    p_max.header.stamp = points_in_msg->header.stamp;
    p_max.point.x = x_max_;
    p_max.point.y = y_max_;
    p_max.point.z = z_max_;

    geometry_msgs::msg::PointStamped p_min_t;
    geometry_msgs::msg::PointStamped p_max_t;
    try {
      tf_buffer_->transform(p_min, p_min_t, points_in_msg->header.frame_id);
      tf_buffer_->transform(p_max, p_max_t, points_in_msg->header.frame_id);
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "Transform Exception: %s", ex.what());
      return;
    }

    pcl::CropBox<PointType> crop_box_filter;
    crop_box_filter.setNegative(true);
    crop_box_filter.setInputCloud(points_out);
    crop_box_filter.setMin(Eigen::Vector4f(
      (float)std::min(p_min_t.point.x, p_max_t.point.x),
      (float)std::min(p_min_t.point.y, p_max_t.point.y),
      (float)std::min(p_min_t.point.z, p_max_t.point.z),
      1.0
    ));
    crop_box_filter.setMax(Eigen::Vector4f(
      (float)std::max(p_min_t.point.x, p_max_t.point.x),
      (float)std::max(p_min_t.point.y, p_max_t.point.y),
      (float)std::max(p_min_t.point.z, p_max_t.point.z),
      1.0
    ));
    crop_box_filter.filter(*points_out);
  }

  if(apply_range_min_filter_){
    pcl::ConditionAnd<PointType>::Ptr range_min_field_condition(new pcl::ConditionAnd<PointType> ());
    range_min_field_condition->addComparison (pcl::FieldComparison<PointType>::ConstPtr (new pcl::FieldComparison<PointType> ("range", pcl::ComparisonOps::GT, (uint32_t) (range_min_*1000))));
    pcl::ConditionalRemoval<PointType> range_min_filter;
    range_min_filter.setInputCloud(points_out);
    range_min_filter.setCondition(range_min_field_condition);
    range_min_filter.filter(*points_out);
  }

  if(apply_range_max_filter_){
    pcl::ConditionAnd<PointType>::Ptr range_max_field_condition(new pcl::ConditionAnd<PointType> ());
    range_max_field_condition->addComparison (pcl::FieldComparison<PointType>::ConstPtr (new pcl::FieldComparison<PointType> ("range", pcl::ComparisonOps::LT, (uint32_t) (range_max_*1000))));
    pcl::ConditionalRemoval<PointType> range_min_filter;
    range_min_filter.setInputCloud(points_out);
    range_min_filter.setCondition(range_max_field_condition);
    range_min_filter.filter(*points_out);
  }

  sensor_msgs::msg::PointCloud2::SharedPtr points_out_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
  points_out_msg->header = points_in_msg->header;
  pcl::toROSMsg(*points_out, *points_out_msg);
  points_out_publisher_->publish(*points_out_msg);
}
