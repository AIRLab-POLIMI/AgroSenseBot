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

using std::placeholders::_1;
using std::placeholders::_2;

CanopyVolumeEstimation::CanopyVolumeEstimation() : Node("canopy_volume_estimation") {

    this->declare_parameter("resolution", rclcpp::ParameterType::PARAMETER_DOUBLE);
    res_ = this->get_parameter("resolution").as_double();

    this->declare_parameter("max_range", rclcpp::ParameterType::PARAMETER_DOUBLE);
    max_range_ = this->get_parameter("max_range").as_double();

    this->declare_parameter("hit_count_threshold", rclcpp::ParameterType::PARAMETER_INTEGER);
    hit_count_threshold_ = this->get_parameter("hit_count_threshold").as_int();

    this->declare_parameter("print_timing", rclcpp::ParameterType::PARAMETER_BOOL);
    print_timing_ = this->get_parameter("print_timing").as_bool();

    this->declare_parameter("enable_canopy_estimation", rclcpp::ParameterType::PARAMETER_BOOL);
    enable_canopy_estimation_ = this->get_parameter("enable_canopy_estimation").as_bool();

    this->declare_parameter("canopy_data_dir_path", rclcpp::ParameterType::PARAMETER_STRING);
    canopy_data_dir_path_ = this->get_parameter("canopy_data_dir_path").as_string();

    this->declare_parameter("enable_viz_topics", rclcpp::ParameterType::PARAMETER_BOOL);
    enable_viz_topics_ = this->get_parameter("enable_viz_topics").as_bool();

    this->declare_parameter("octomap_viz_publish_period", rclcpp::ParameterType::PARAMETER_DOUBLE);
    octomap_viz_publish_period_ = this->get_parameter("octomap_viz_publish_period").as_double();

    if (!fs::exists(canopy_data_dir_path_)) {
        if (!fs::create_directories(canopy_data_dir_path_)) {
            RCLCPP_FATAL(this->get_logger(), "failed to create canopy data directory: %s", canopy_data_dir_path_.c_str());
            throw std::runtime_error("failed to create directory: " + canopy_data_dir_path_.string());
        }
    } else if (!fs::is_directory(canopy_data_dir_path_)) {
        RCLCPP_FATAL(this->get_logger(), "failed to create canopy data directory, path exists but is not a directory: %s", canopy_data_dir_path_.c_str());
        throw std::runtime_error("path exists but is not a directory: " + canopy_data_dir_path_.string());
    }

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    if (enable_canopy_estimation_) {
        points_in_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "points_in", rclcpp::SensorDataQoS().durability_volatile().reliable().keep_last(1),
                std::bind(&CanopyVolumeEstimation::points_in_callback, this, _1));
    } else {
        publish_canopy_data_timer_ = this->create_timer(std::chrono::milliseconds(100), std::bind(&CanopyVolumeEstimation::publish_canopy_data_timer_callback, this));
    }

    initialize_canopy_region_service_ = this->create_service<InitializeCanopyRegion>(
            "initialize_canopy_region", std::bind(&CanopyVolumeEstimation::initialize_canopy_region, this, _1, _2));

    suspend_canopy_region_service_ = this->create_service<SuspendCanopyRegion>(
            "suspend_canopy_region", std::bind(&CanopyVolumeEstimation::suspend_canopy_region, this, _1, _2));

    canopy_data_array_publisher_ = this->create_publisher<CanopyDataArray>(
            "canopy_data", rclcpp::SensorDataQoS().reliable().transient_local());

    benchmarking_execution_duration_publisher_ = this->create_publisher<asb_msgs::msg::ExecutionDurationStamped>("~/benchmarking/execution_duration", rclcpp::SensorDataQoS().transient_local().reliable().keep_last(10));

    canopy_viz_publisher_ = this->create_publisher<MarkerArray>(
            "canopy_visualization_markers", rclcpp::SensorDataQoS().reliable().transient_local());

}

void CanopyVolumeEstimation::add_roi_depth_viz_marker(CanopyStruct & canopy_struct, size_t marker_id, Header header, double size, double x, double y_min, double y_max, double z) {

    canopy_struct.roi_depth_viz_marker_array.markers[marker_id].pose.position.x = x;
    canopy_struct.roi_depth_viz_marker_array.markers[marker_id].pose.position.y = (y_min + y_max) / 2;
    canopy_struct.roi_depth_viz_marker_array.markers[marker_id].pose.position.z = z;
    canopy_struct.roi_depth_viz_marker_array.markers[marker_id].pose.orientation.w = 1;
    canopy_struct.roi_depth_viz_marker_array.markers[marker_id].header = header;
    canopy_struct.roi_depth_viz_marker_array.markers[marker_id].ns = canopy_struct.canopy_id + "/roi_depth";
    canopy_struct.roi_depth_viz_marker_array.markers[marker_id].id = (int) marker_id;
    canopy_struct.roi_depth_viz_marker_array.markers[marker_id].type = Marker::CUBE;
    canopy_struct.roi_depth_viz_marker_array.markers[marker_id].action = Marker::ADD;
    canopy_struct.roi_depth_viz_marker_array.markers[marker_id].scale.x = size;
    canopy_struct.roi_depth_viz_marker_array.markers[marker_id].scale.y = size + y_max - y_min;
    canopy_struct.roi_depth_viz_marker_array.markers[marker_id].scale.z = size;
    canopy_struct.roi_depth_viz_marker_array.markers[marker_id].color.r = 0.1;
    canopy_struct.roi_depth_viz_marker_array.markers[marker_id].color.g = 0.7;
    canopy_struct.roi_depth_viz_marker_array.markers[marker_id].color.b = 0.1;
    canopy_struct.roi_depth_viz_marker_array.markers[marker_id].color.a = 1.0;
}

bool CanopyVolumeEstimation::transform_region_of_interest(const CanopyRegionOfInterest & roi, const Header & target_header, CanopyRegionOfInterest & roi_transformed) {

    PointStamped p_1;
    p_1.header.frame_id = roi.frame_id;
    p_1.header.stamp = target_header.stamp;
    p_1.point.x = roi.x_1;

    PointStamped p_2;
    p_2.header.frame_id = roi.frame_id;
    p_2.header.stamp = target_header.stamp;
    p_2.point.x = roi.x_2;

    PointStamped p_1_transformed, p_2_transformed;
    p_1_transformed.header = target_header;
    p_2_transformed.header = target_header;

    try {
        tf2::doTransform(p_1, p_1_transformed, tf_buffer_->lookupTransform(p_1_transformed.header.frame_id, p_1.header.frame_id, tf2::TimePointZero));
        tf2::doTransform(p_2, p_2_transformed, tf_buffer_->lookupTransform(p_2_transformed.header.frame_id, p_2.header.frame_id, tf2::TimePointZero));
    } catch (tf2::TransformException & ex) {
        RCLCPP_WARN(this->get_logger(), "Transform Exception: %s", ex.what());
        return false;
    } catch (std::exception & ex) {
        RCLCPP_WARN(this->get_logger(), "Exception: %s", ex.what());
        return false;
    }

    roi_transformed.frame_id = target_header.frame_id;
    roi_transformed.x_1 = p_1_transformed.point.x;
    roi_transformed.x_2 = p_2_transformed.point.x;

    return true;
}

void CanopyVolumeEstimation::initialize_canopy_region(const std::shared_ptr<InitializeCanopyRegion::Request> request, std::shared_ptr<InitializeCanopyRegion::Response> response) {

    if (canopy_structs.contains(request->canopy_id)) {
        RCLCPP_INFO(this->get_logger(), "received initialize canopy region request, row_id: %s (was suspended, re-enabling)", request->canopy_id.c_str());
        canopy_structs[request->canopy_id].suspended = false;
    } else {
        RCLCPP_INFO(this->get_logger(), "received initialize canopy region request, row_id: %s (did not exist, initializing)", request->canopy_id.c_str());
        canopy_structs[request->canopy_id] = CanopyStruct();
        canopy_structs[request->canopy_id].canopy_id = request->canopy_id;
        canopy_structs[request->canopy_id].suspended = false;
        canopy_structs[request->canopy_id].canopy_frame_id = request->canopy_frame_id;
        canopy_structs[request->canopy_id].point_cloud_min_x = request->min_x;
        canopy_structs[request->canopy_id].point_cloud_max_x = request->max_x;
        canopy_structs[request->canopy_id].point_cloud_min_y = request->min_y;
        canopy_structs[request->canopy_id].point_cloud_max_y = request->max_y;
        canopy_structs[request->canopy_id].point_cloud_min_z = request->min_z;
        canopy_structs[request->canopy_id].point_cloud_max_z = request->max_z;
        canopy_structs[request->canopy_id].roi = request->roi;
        canopy_structs[request->canopy_id].roi_depth_viz_marker_array = MarkerArray();
        canopy_structs[request->canopy_id].octomap_viz_marker_array = MarkerArray();
        canopy_structs[request->canopy_id].octomap_viz_marker_array.markers.resize(1);
        canopy_structs[request->canopy_id].octree = std::make_unique<OcTree>(res_);

        if (!enable_canopy_estimation_) {
            fs::path octree_filename(replace_substring(request->canopy_id, "/", "__") + ".bt");
            fs::path octree_file_path = canopy_data_dir_path_ / octree_filename;

            auto read_octree_start = std::chrono::high_resolution_clock::now();
            if (canopy_structs[request->canopy_id].octree->readBinary(octree_file_path)) {
                std::chrono::duration<double, std::milli> read_octree_duration_ms = std::chrono::high_resolution_clock::now() - read_octree_start;
                RCLCPP_INFO(this->get_logger(), "read octree from file [row_id: %s]. It took %.1f ms", request->canopy_id.c_str(), read_octree_duration_ms.count());
            } else {
                RCLCPP_ERROR(this->get_logger(), "failed to read octree from file: %s [row_id: %s]", octree_file_path.c_str(), request->canopy_id.c_str());
                response->result = false;
                return;
            }
        }

        // compute the occupancy probability threshold such that nodes are considered occupied after the n-th hit
        double p = 0.51;
        long n = hit_count_threshold_;
        double th = pow(p / (1 - p), n) / (1 + pow(p / (1 - p), n));
        double th_clamp = pow(p / (1 - p), n + 1) / (1 + pow(p / (1 - p), n + 1));
        canopy_structs[request->canopy_id].octree->setProbHit(p);
        canopy_structs[request->canopy_id].octree->setOccupancyThres(th);
        canopy_structs[request->canopy_id].octree->setClampingThresMax(th_clamp);
    }

    response->result = true;
}

void CanopyVolumeEstimation::suspend_canopy_region(const std::shared_ptr<SuspendCanopyRegion::Request> request, std::shared_ptr<SuspendCanopyRegion::Response> response) {

    if (canopy_structs.contains(request->canopy_id)) {
        if (!canopy_structs[request->canopy_id].suspended) {
            RCLCPP_INFO(this->get_logger(), "received suspend canopy region request, row_id: %s", request->canopy_id.c_str());
            canopy_structs[request->canopy_id].suspended = true;

            if (enable_canopy_estimation_) {
                fs::path octree_filename(replace_substring(request->canopy_id, "/", "__") + ".bt");
                fs::path octree_file_path = canopy_data_dir_path_ / octree_filename;

                auto write_octree_start = std::chrono::high_resolution_clock::now();
                if (canopy_structs[request->canopy_id].octree->writeBinaryConst(octree_file_path)) {
                    std::chrono::duration<double, std::milli> write_octree_duration_ms = std::chrono::high_resolution_clock::now() - write_octree_start;
                    RCLCPP_INFO(this->get_logger(), "written octree to file [row_id: %s]. It took %.1f ms", request->canopy_id.c_str(), write_octree_duration_ms.count());
                } else {
                    RCLCPP_ERROR(this->get_logger(), "failed to write octree to file: %s [row_id: %s]", octree_file_path.c_str(), request->canopy_id.c_str());
                }
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "received suspend canopy region request [row_id: %s] but it was already suspended", request->canopy_id.c_str());
        }

        response->result = true;
        return;
    } else {
        RCLCPP_ERROR(this->get_logger(), "received suspend canopy region request, row_id: %s (did not exist)", request->canopy_id.c_str());
        response->result = false;
        return;
    }
}

void CanopyVolumeEstimation::points_in_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud) {

    auto execution_start = std::chrono::high_resolution_clock::now();

    CanopyDataArray canopy_data_array_msg = CanopyDataArray();

    for (auto & [canopy_id, canopy_struct]: canopy_structs) {
        if (canopy_struct.suspended) continue;
        const auto start_time = rclcpp::Clock{}.now();

        PCLPointCloud pc;
        pcl::fromROSMsg(*cloud, pc);

        geometry_msgs::msg::TransformStamped sensor_to_canopy_transform_stamped;
        try {
            sensor_to_canopy_transform_stamped = tf_buffer_->lookupTransform(canopy_struct.canopy_frame_id, cloud->header.frame_id, cloud->header.stamp, rclcpp::Duration::from_seconds(0.1));
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN(this->get_logger(), "%s", ex.what());
            return;
        }

        // set up filter for height range, also removes NANs:
        pcl::PassThrough<PCLPoint> pass_x;
        pass_x.setFilterFieldName("x");
        pass_x.setFilterLimits((float) canopy_struct.point_cloud_min_x, (float) canopy_struct.point_cloud_max_x);
        pcl::PassThrough<PCLPoint> pass_y;
        pass_y.setFilterFieldName("y");
        pass_y.setFilterLimits((float) canopy_struct.point_cloud_min_y, (float) canopy_struct.point_cloud_max_y);
        pcl::PassThrough<PCLPoint> pass_z;
        pass_z.setFilterFieldName("z");
        pass_z.setFilterLimits((float) canopy_struct.point_cloud_min_z, (float) canopy_struct.point_cloud_max_z);

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
        for (auto & it: pc) {
            octomap::point3d point(it.x, it.y, it.z);
            if ((point - sensor_origin).norm() <= max_range_) {
                octomap::OcTreeKey key;
                if (canopy_struct.octree->coordToKeyChecked(point, key)) canopy_struct.octree->updateNode(key, true);
            }
        }

        canopy_data_array_msg.canopy_data_array.emplace_back();
        update_canopy_volume(canopy_struct, canopy_data_array_msg.canopy_data_array.back(), cloud->header.stamp);

        if (print_timing_) RCLCPP_INFO(get_logger(), "%s:\t %zu points,\t %.3f s", canopy_struct.canopy_id.c_str(), pc.size(), (rclcpp::Clock{}.now() - start_time).seconds());
    }

    canopy_data_array_publisher_->publish(canopy_data_array_msg);

    std::chrono::duration<double> execution_duration_s = std::chrono::high_resolution_clock::now() - execution_start;
    if (print_timing_) RCLCPP_INFO(get_logger(), "execution_duration_s: %.2f ms", execution_duration_s.count() * 1000);

    asb_msgs::msg::ExecutionDurationStamped execution_duration;
    execution_duration.stamp = this->get_clock()->now();
    execution_duration.execution_duration = rclcpp::Duration::from_seconds((execution_duration_s).count());
    execution_duration.label = "";
    benchmarking_execution_duration_publisher_->publish(execution_duration);
}

void CanopyVolumeEstimation::publish_canopy_data_timer_callback() {

    CanopyDataArray canopy_data_array_msg = CanopyDataArray();

    for (auto & [canopy_id, canopy_struct]: canopy_structs) {
        if (canopy_struct.suspended) continue;
        canopy_data_array_msg.canopy_data_array.emplace_back();
        update_canopy_volume(canopy_struct, canopy_data_array_msg.canopy_data_array.back(), this->get_clock()->now());
    }

    canopy_data_array_publisher_->publish(canopy_data_array_msg);

}

void CanopyVolumeEstimation::update_canopy_volume(CanopyStruct & canopy_struct, CanopyData & canopy_data_msg, const rclcpp::Time & ros_time) {

    auto octomap_marker_array_create_duration_ms = std::chrono::duration<double, std::milli>(0);

    bool publish_canopy_viz_ = enable_viz_topics_ && (canopy_viz_publisher_->get_subscription_count() + canopy_viz_publisher_->get_intra_process_subscription_count() > 0);

    // expand the tree to make sure all leaf nodes are the same size (resolution)
    canopy_struct.octree->expand();

    canopy_data_msg.canopy_id = canopy_struct.canopy_id;
    canopy_data_msg.header.frame_id = canopy_struct.canopy_frame_id;
    canopy_data_msg.header.stamp = ros_time;
    canopy_data_msg.resolution = (float) canopy_struct.octree->getResolution();

    double bb_x_min, bb_y_min, bb_z_min, bb_x_max, bb_y_max, bb_z_max;
    canopy_struct.octree->getMetricMin(bb_x_min, bb_y_min, bb_z_min);
    canopy_struct.octree->getMetricMax(bb_x_max, bb_y_max, bb_z_max);

    Header roi_header = Header();
    roi_header.frame_id = canopy_struct.canopy_frame_id;
    roi_header.stamp = ros_time;
    bool roi_result = transform_region_of_interest(canopy_struct.roi, roi_header, canopy_struct.roi_transformed);
    if (!roi_result) return;

    canopy_data_msg.roi = canopy_struct.roi_transformed;

    // sort the region of interest x_1, x_2 values so that x_min <= x_max, otherwise the bounding box will be considered empty
    bb_x_min = std::min(canopy_struct.roi_transformed.x_1, canopy_struct.roi_transformed.x_2);
    bb_x_max = std::max(canopy_struct.roi_transformed.x_1, canopy_struct.roi_transformed.x_2);

    // collect y values for each x, z coordinate
    auto bbx_min = octomap::point3d((float) bb_x_min, (float) bb_y_min, (float) bb_z_min);
    auto bbx_max = octomap::point3d((float) bb_x_max, (float) bb_y_max, (float) bb_z_max);
    std::map<std::pair<double, double>, std::vector<double>> y_vector_map;
    for (auto it = canopy_struct.octree->begin_leafs_bbx(bbx_min, bbx_max), end = canopy_struct.octree->end_leafs_bbx(); it != end; ++it) {
        if (canopy_struct.octree->isNodeOccupied(*it)) {

            double x = it.getX();
            double y = it.getY();
            double z = it.getZ();

            auto x_z = std::pair(x, z);
            y_vector_map[x_z].emplace_back(y);

            if (publish_canopy_viz_) {
                auto octomap_marker_array_create_start = std::chrono::high_resolution_clock::now();
                add_octomap_marker_array_voxel(x, y, z, canopy_struct);
                octomap_marker_array_create_duration_ms += std::chrono::high_resolution_clock::now() - octomap_marker_array_create_start;

            }
        }
    }

    if (print_timing_) RCLCPP_INFO(get_logger(), "octomap_marker_array_create_duration_ms: %.2f ms   %s", octomap_marker_array_create_duration_ms.count(), canopy_struct.canopy_id.c_str());

    if (publish_canopy_viz_) {
        if (y_vector_map.size() > canopy_struct.roi_depth_viz_marker_array.markers.size()) {
            canopy_struct.roi_depth_viz_marker_array.markers.resize(y_vector_map.size());
        }
    }

    // compute y_depth for each x, z coordinate
    // Note: adding voxel_length to y_depth because y_depth is computed from the centroids of the voxels (otherwise the
    // volume of a 1-voxel deep region would be 0 m^3)
    double voxel_length = canopy_struct.octree->getResolution();
    std::map<std::pair<double, double>, double> y_depth_map;
    int marker_id = 0;
    for (auto [x_z, y_vector]: y_vector_map) {
        auto const & [x, z] = x_z;
        const auto [y_vector_min, y_vector_max] = std::minmax_element(std::begin(y_vector), std::end(y_vector));
        y_depth_map[x_z] = voxel_length + *y_vector_max - *y_vector_min;

        canopy_data_msg.depth_x_array.emplace_back(x);
        canopy_data_msg.depth_y_array.emplace_back(y_depth_map[x_z]);
        canopy_data_msg.depth_z_array.emplace_back(z);

        if (publish_canopy_viz_) {
            add_roi_depth_viz_marker(canopy_struct, marker_id, roi_header, voxel_length, x, *y_vector_min, *y_vector_max, z);
            marker_id++;
        }
    }

    if (publish_canopy_viz_) {
        for (size_t further_marker_id = marker_id; further_marker_id < canopy_struct.roi_depth_viz_marker_array.markers.size(); further_marker_id++) {
            canopy_struct.roi_depth_viz_marker_array.markers[further_marker_id].action = Marker::DELETE;
        }
        canopy_viz_publisher_->publish(canopy_struct.roi_depth_viz_marker_array);

        publish_octomap_marker_array(ros_time, canopy_struct);
    }

    // compute volume for each x coordinate
    double voxel_area = pow(voxel_length, 2);
    std::map<double, double> x_volume_map;
    for (auto [x_z, y_depth]: y_depth_map) {
        auto const & x = x_z.first;
        x_volume_map[x] += y_depth * voxel_area;
    }

    for (auto [x, volume]: x_volume_map) {
        canopy_data_msg.volume_x_array.emplace_back(x);
        canopy_data_msg.volume_y_array.emplace_back(volume);
    }

}

void CanopyVolumeEstimation::add_octomap_marker_array_voxel(const double & x, const double & y, const double & z, CanopyStruct & canopy_struct) {

    const size_t octomap_size = canopy_struct.octree->size();
    if (octomap_size > 1) {

        geometry_msgs::msg::Point cube_center;
        cube_center.x = x;
        cube_center.y = y;
        cube_center.z = z;

        canopy_struct.octomap_viz_marker_array.markers[0].points.push_back(cube_center);
        double min_x, min_y, min_z, max_x, max_y, max_z;
        canopy_struct.octree->getMetricMin(min_x, min_y, min_z);
        canopy_struct.octree->getMetricMax(max_x, max_y, max_z);

        double color_factor_ = 0.8;
        double h = (1.0 - std::clamp((cube_center.z - min_z) / (max_z - min_z), 0.0, 1.0)) * color_factor_;
        canopy_struct.octomap_viz_marker_array.markers[0].colors.push_back(height_color_map(h));
    }
}

void CanopyVolumeEstimation::publish_octomap_marker_array(const rclcpp::Time & rostime, CanopyStruct & canopy_struct) {

    auto & last_msg_stamp = canopy_struct.octomap_viz_marker_array.markers[0].header.stamp;
    double last_msg_s = last_msg_stamp.sec + last_msg_stamp.nanosec / 1E9;
    double now_s = rostime.seconds();

    if (now_s - last_msg_s < octomap_viz_publish_period_) {
        return;
    }

    canopy_struct.octomap_viz_marker_array.markers[0].header.frame_id = canopy_struct.canopy_frame_id;
    canopy_struct.octomap_viz_marker_array.markers[0].header.stamp = rostime;
    canopy_struct.octomap_viz_marker_array.markers[0].ns = canopy_struct.canopy_id + "/octomap";
    canopy_struct.octomap_viz_marker_array.markers[0].id = 0;
    canopy_struct.octomap_viz_marker_array.markers[0].type = visualization_msgs::msg::Marker::CUBE_LIST;
    canopy_struct.octomap_viz_marker_array.markers[0].scale.x = res_;
    canopy_struct.octomap_viz_marker_array.markers[0].scale.y = res_;
    canopy_struct.octomap_viz_marker_array.markers[0].scale.z = res_;

    if (!canopy_struct.octomap_viz_marker_array.markers[0].points.empty()) {
        canopy_struct.octomap_viz_marker_array.markers[0].action = visualization_msgs::msg::Marker::ADD;
    } else {
        canopy_struct.octomap_viz_marker_array.markers[0].action = visualization_msgs::msg::Marker::DELETE;
    }

    auto publish_octomap_marker_array_start = std::chrono::high_resolution_clock::now();

    canopy_viz_publisher_->publish(canopy_struct.octomap_viz_marker_array);

    std::chrono::duration<double, std::milli> publish_octomap_marker_array_duration_ms = std::chrono::high_resolution_clock::now() - publish_octomap_marker_array_start;
    if (print_timing_) RCLCPP_INFO(get_logger(), "publish_octomap_marker_array_duration_ms: %.2f ms   %s", publish_octomap_marker_array_duration_ms.count(), canopy_struct.canopy_id.c_str());
}


ColorRGBA CanopyVolumeEstimation::height_color_map(double h) {

    ColorRGBA color;
    color.a = 1.0;
    // blend over HSV-values (more colors)

    const double s = 1.0;
    const double v = 1.0;

    h -= floor(h);
    h *= 6;
    int i;
    double m;
    double n;
    double f;

    i = floor(h);
    f = h - i;
    if (!(i & 1)) {
        // if i is even
        f = 1 - f;
    }
    m = v * (1.0 - s);
    n = v * (1.0 - s * f);

    switch (i) {
        case 6:
        case 0:
            color.r = static_cast<float>(v);
            color.g = static_cast<float>(n);
            color.b = static_cast<float>(m);
            break;
        case 1:
            color.r = static_cast<float>(n);
            color.g = static_cast<float>(v);
            color.b = static_cast<float>(m);
            break;
        case 2:
            color.r = static_cast<float>(m);
            color.g = static_cast<float>(v);
            color.b = static_cast<float>(n);
            break;
        case 3:
            color.r = static_cast<float>(m);
            color.g = static_cast<float>(n);
            color.b = static_cast<float>(v);
            break;
        case 4:
            color.r = static_cast<float>(n);
            color.g = static_cast<float>(m);
            color.b = static_cast<float>(v);
            break;
        case 5:
            color.r = static_cast<float>(v);
            color.g = static_cast<float>(m);
            color.b = static_cast<float>(n);
            break;
        default:
            color.r = 1;
            color.g = 0.5;
            color.b = 0.5;
            break;
    }

    return color;
}

std::string CanopyVolumeEstimation::replace_substring(const std::string & str, const std::string & from, const std::string & to) {

    std::string res = str;
    size_t start_pos = 0;
    while ((start_pos = str.find(from, start_pos)) != std::string::npos) {
        res.replace(start_pos, from.length(), to);
        start_pos += to.length(); // Advance past the replacement
    }
    return res;
}
