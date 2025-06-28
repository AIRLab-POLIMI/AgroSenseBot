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
#include "rclcpp/duration.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/exceptions.h"
#include "tf2_eigen/tf2_eigen/tf2_eigen.hpp"

#include "std_srvs/srv/empty.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "asb_msgs/msg/duration_stamped.hpp"

#include "sensor_msgs/point_cloud2_iterator.hpp"

#include <pcl/point_types.h>
#include "pcl/filters/crop_box.h"
#include "pcl/filters/conditional_removal.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/transforms.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <fstream>
#include <cstdint>

using namespace std::chrono_literals;
using std_srvs::srv::Empty;

class ASBLidarFilter : public rclcpp::Node {
public:
    ASBLidarFilter();

private:

    void points_in_callback(const sensor_msgs::msg::PointCloud2::SharedPtr points_in_msg);

    void create_mask_service_callback(const std::shared_ptr<Empty::Request> request, std::shared_ptr<Empty::Response> response);

    template<typename PointT>
    void add_point_cloud_to_mask(const std::shared_ptr<pcl::PointCloud<PointT>> & points_in_pcl, const std_msgs::msg::Header & points_in_header);

    template<typename PointT>
    bool transform_point_cloud_to_base_frame(const std::shared_ptr<pcl::PointCloud<PointT>> & points_in_pcl, const std_msgs::msg::Header & points_in_header, std::shared_ptr<pcl::PointCloud<PointT>> & points_transformed_pcl);

    template<typename PointT>
    bool transform_point_cloud_to_sensor_frame(const std::shared_ptr<pcl::PointCloud<PointT>> & points_in_pcl, const std_msgs::msg::Header & points_in_header, std::shared_ptr<pcl::PointCloud<PointT>> & points_transformed_pcl);

    void save_mask_as_pbm(const std::string& filename, const std::vector<bool> & mask, std::uint32_t width, std::uint32_t height);

    bool load_mask_from_pbm(const std::string& filename, std::vector<bool> & mask, std::uint32_t & width, std::uint32_t & height);

    std::string base_frame_id_;

    // mask filter params
    std::string mask_file_path_;
    int mask_filter_size_;
    unsigned int mask_filter_count_;
    double x_min_, x_max_, y_min_, y_max_, z_min_, z_max_;

    // range and layer limit params
    float min_range_, min_range_2_;
    int min_layer_from_bottom_, max_layer_from_bottom_;

    // scan params
    double scan_min_height_, scan_max_height_;

    // run time variables
    bool reset_mask_ = false;
    bool create_mask_ = false;
    unsigned int create_mask_count_ = 0;
    std::vector<bool> mask_;
    std::uint32_t mask_width_, mask_height_;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_in_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_out_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_out_no_ground_ceiling_publisher_;
    rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr heartbeat_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_publisher_;
    rclcpp::Publisher<asb_msgs::msg::DurationStamped>::SharedPtr benchmarking_execution_duration_publisher_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr create_mask_service_;
};


#endif //ASB_LIDAR_FILTER_ASB_LIDAR_FILTER_H
