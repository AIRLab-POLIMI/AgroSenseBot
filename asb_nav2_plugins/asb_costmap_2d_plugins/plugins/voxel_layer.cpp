/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2025, Enrico Piazza, Università degli Studi di Milano
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/

#include "asb_costmap_2d_plugins/voxel_layer.hpp"

#define VOXEL_BITS 16
PLUGINLIB_EXPORT_CLASS(asb_costmap_2d_plugins::VoxelLayer, nav2_costmap_2d::Layer)

using nav2_costmap_2d::NO_INFORMATION;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::FREE_SPACE;
using rcl_interfaces::msg::ParameterType;

namespace asb_costmap_2d_plugins {

void VoxelLayer::onInitialize() {

    nav2_costmap_2d::ObstacleLayer::onInitialize();

    declareParameter("enabled", rclcpp::ParameterValue(true));
    declareParameter("footprint_clearing_enabled", rclcpp::ParameterValue(true));
    declareParameter("max_obstacle_height", rclcpp::ParameterValue(2.0));
    declareParameter("z_voxels", rclcpp::ParameterValue(10));
    declareParameter("origin_z", rclcpp::ParameterValue(0.0));
    declareParameter("z_resolution", rclcpp::ParameterValue(0.2));
    declareParameter("unknown_threshold", rclcpp::ParameterValue(15));
    declareParameter("mark_threshold", rclcpp::ParameterValue(0));
    declareParameter("combination_method", rclcpp::ParameterValue(1));
    declareParameter("publish_voxel_map", rclcpp::ParameterValue(false));

    auto node = node_.lock();
    if (!node) {
        throw std::runtime_error{"Failed to lock node"};
    }

    node->get_parameter(name_ + "." + "enabled", enabled_);
    node->get_parameter(name_ + "." + "footprint_clearing_enabled", footprint_clearing_enabled_);
    node->get_parameter(name_ + "." + "max_obstacle_height", max_obstacle_height_);
    node->get_parameter(name_ + "." + "z_voxels", size_z_);
    node->get_parameter(name_ + "." + "origin_z", origin_z_);
    node->get_parameter(name_ + "." + "z_resolution", z_resolution_);
    node->get_parameter(name_ + "." + "unknown_threshold", unknown_threshold_);
    node->get_parameter(name_ + "." + "mark_threshold", mark_threshold_);
    node->get_parameter(name_ + "." + "combination_method", combination_method_);
    node->get_parameter(name_ + "." + "publish_voxel_map", publish_voxel_);

    auto qos_reliable_transient_local_depth_1 = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    auto qos_reliable_transient_local_depth_10 = rclcpp::QoS(rclcpp::KeepLast(10)).transient_local().reliable();

    if (publish_voxel_) {
        voxel_pub_ = node->create_publisher<nav2_msgs::msg::VoxelGrid>("voxel_grid", qos_reliable_transient_local_depth_1);
        voxel_pub_->on_activate();
    }

    clearing_endpoints_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("clearing_endpoints", qos_reliable_transient_local_depth_1);
    clearing_endpoints_pub_->on_activate();

    benchmarking_execution_duration_publisher_ = node->create_publisher<asb_msgs::msg::ExecutionDurationStamped>("~/benchmarking/execution_duration", qos_reliable_transient_local_depth_10);
    benchmarking_execution_duration_publisher_->on_activate();

    unknown_threshold_ += (VOXEL_BITS - size_z_);
    matchSize();

    // Add callback for dynamic parameters
    dyn_params_handler_ = node->add_on_set_parameters_callback(std::bind(&VoxelLayer::dynamicParametersCallback, this, std::placeholders::_1));

    RCLCPP_INFO(logger_, "ASB voxel_layer initialized");

}

VoxelLayer::~VoxelLayer() {

    dyn_params_handler_.reset();
}

void VoxelLayer::matchSize() {

    std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
    nav2_costmap_2d::ObstacleLayer::matchSize();
    voxel_grid_.resize(size_x_, size_y_, size_z_);
    assert(voxel_grid_.sizeX() == size_x_ && voxel_grid_.sizeY() == size_y_);
}

void VoxelLayer::reset() {
    // Call the base class method before adding our own functionality
    nav2_costmap_2d::ObstacleLayer::reset();
    resetMaps();
}

void VoxelLayer::resetMaps() {
    // Call the base class method before adding our own functionality
    // Note: at the time this was written, ObstacleLayer doesn't implement
    // resetMaps so this goes to the next layer down Costmap2DLayer which also
    // doesn't implement this, so it actually goes all the way to Costmap2D
    nav2_costmap_2d::ObstacleLayer::resetMaps();
    voxel_grid_.reset();
}

void VoxelLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double *min_x, double *min_y, double *max_x, double *max_y) {

    std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
    auto update_bounds_start = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> raytracing_duration_s(0);

    if (rolling_window_) {
        updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
    }
    if (!enabled_) {
        return;
    }
    useExtraBounds(min_x, min_y, max_x, max_y);

//    bool current = true;
//    std::vector<nav2_costmap_2d::Observation> marking_observations, clearing_observations;

    std::vector<nav2_costmap_2d::Observation> observations;
    bool current = getObservations(observations);

//    // get the marking observations
//    current = getMarkingObservations(marking_observations) && current;
//
//    // get the clearing observations
//    current = getClearingObservations(clearing_observations) && current;

    // update the global current status
    current_ = current;

    for (unsigned int i = 0; i < observations.size(); ++i) {
        const nav2_costmap_2d::Observation &obs = observations[i];
        voxel_grid_.reset();

        //**********************************************//
        //                                              //
        //                mark obstacles                //
        //                                              //
        //**********************************************//

        const sensor_msgs::msg::PointCloud2 &cloud = *(obs.cloud_);

        double sq_obstacle_max_range = obs.obstacle_max_range_ * obs.obstacle_max_range_;
        double sq_obstacle_min_range = obs.obstacle_min_range_ * obs.obstacle_min_range_;

        sensor_msgs::PointCloud2ConstIterator<float> iter_m_x(cloud, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_m_y(cloud, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_m_z(cloud, "z");

        for (; iter_m_x != iter_m_x.end(); ++iter_m_x, ++iter_m_y, ++iter_m_z) {
            // if the obstacle is too high or too far away from the robot we won't add it
            if (*iter_m_z > max_obstacle_height_) {
                continue;
            }

            // compute the squared distance from the hit point to the point cloud's origin
            double sq_dist = (*iter_m_x - obs.origin_.x) * (*iter_m_x - obs.origin_.x) + (*iter_m_y - obs.origin_.y) * (*iter_m_y - obs.origin_.y) + (*iter_m_z - obs.origin_.z) * (*iter_m_z - obs.origin_.z);

            // if the point is far enough away... we won't consider it
            if (sq_dist >= sq_obstacle_max_range) {
                continue;
            }

            // If the point is too close, do not consider it
            if (sq_dist < sq_obstacle_min_range) {
                continue;
            }

            // now we need to compute the map coordinates for the observation
            unsigned int mx, my, mz;
            if (*iter_m_z < origin_z_) {
                if (!worldToMap3D(*iter_m_x, *iter_m_y, origin_z_, mx, my, mz)) {
                    continue;
                }
            } else if (!worldToMap3D(*iter_m_x, *iter_m_y, *iter_m_z, mx, my, mz)) {
                continue;
            }

            // mark the cell in the voxel grid and check if we should also mark it in the costmap
            if (voxel_grid_.markVoxelInMap(mx, my, mz, mark_threshold_)) {
                unsigned int index = getIndex(mx, my);

                costmap_[index] = LETHAL_OBSTACLE;
                touch(static_cast<double>(*iter_m_x), static_cast<double>(*iter_m_y), min_x, min_y, max_x, max_y);
            }
        }

        //**********************************************//
        //                     end                      //
        //                mark obstacles                //
        //                                              //
        //**********************************************//

        //**********************************************//
        //                                              //
        //              raytrace freespace              //
        //                                              //
        //**********************************************//

        auto raytracing_start = std::chrono::high_resolution_clock::now();

        auto clearing_endpoints_ = std::make_unique<sensor_msgs::msg::PointCloud2>();

        if (obs.cloud_->height == 0 || obs.cloud_->width == 0) {
            continue;
        }

        double sensor_x, sensor_y, sensor_z;
        double ox = obs.origin_.x;
        double oy = obs.origin_.y;
        double oz = obs.origin_.z;

        if (!worldToMap3DFloat(ox, oy, oz, sensor_x, sensor_y, sensor_z)) {
            RCLCPP_WARN(logger_, "Sensor origin at (%.2f, %.2f %.2f) is out of map bounds "
                                 "(%.2f, %.2f, %.2f) to (%.2f, %.2f, %.2f). "
                                 "The costmap cannot raytrace for it.", ox, oy, oz, origin_x_, origin_y_, origin_z_, origin_x_ + getSizeInMetersX(), origin_y_ + getSizeInMetersY(), origin_z_ + getSizeInMetersZ());

            continue;
        }

        bool publish_clearing_points;

        {
            auto node = node_.lock();
            if (!node) {
                throw std::runtime_error{"Failed to lock node"};
            }
            publish_clearing_points = (node->count_subscribers("clearing_endpoints") > 0);
        }

        clearing_endpoints_->data.clear();
        clearing_endpoints_->width = obs.cloud_->width;
        clearing_endpoints_->height = obs.cloud_->height;
        clearing_endpoints_->is_dense = true;
        clearing_endpoints_->is_bigendian = false;

        sensor_msgs::PointCloud2Modifier modifier(*clearing_endpoints_);
        modifier.setPointCloud2Fields(3, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1, sensor_msgs::msg::PointField::FLOAT32, "z", 1, sensor_msgs::msg::PointField::FLOAT32);

        sensor_msgs::PointCloud2Iterator<float> clearing_endpoints_iter_x(*clearing_endpoints_, "x");
        sensor_msgs::PointCloud2Iterator<float> clearing_endpoints_iter_y(*clearing_endpoints_, "y");
        sensor_msgs::PointCloud2Iterator<float> clearing_endpoints_iter_z(*clearing_endpoints_, "z");

        // we can pre-compute the end points of the map outside the inner loop... we'll need these later
        double map_end_x = origin_x_ + getSizeInMetersX();
        double map_end_y = origin_y_ + getSizeInMetersY();
        double map_end_z = origin_z_ + getSizeInMetersZ();

        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*(obs.cloud_), "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*(obs.cloud_), "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*(obs.cloud_), "z");

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
            double wpx = *iter_x;
            double wpy = *iter_y;
            double wpz = *iter_z;

            double distance = dist(ox, oy, oz, wpx, wpy, wpz);
            double scaling_fact = 1.0;
            scaling_fact = std::max(std::min(scaling_fact, (distance - 2 * resolution_) / distance), 0.0);
            wpx = scaling_fact * (wpx - ox) + ox;
            wpy = scaling_fact * (wpy - oy) + oy;
            wpz = scaling_fact * (wpz - oz) + oz;

            double a = wpx - ox;
            double b = wpy - oy;
            double c = wpz - oz;
            double t = 1.0;

            // rescale the endpoint if z is outside of map boundaries
            if (wpz < origin_z_) {
                t = std::min(t, (origin_z_ - oz) / c);
            } else if (wpz > map_end_z) {
                t = std::max(0.0, std::min(t, (map_end_z - 0.01 - oz) / c));
            }

            // rescale the endpoint if x is outside of map boundaries
            if (wpx < origin_x_) {
                t = std::min(t, (origin_x_ - ox) / a);
            } else if (wpx > map_end_x) {
                t = std::min(t, (map_end_x - ox) / a);
            }

            // rescale the endpoint if y is outside of map boundaries
            if (wpy < origin_y_) {
                t = std::min(t, (origin_y_ - oy) / b);
            } else if (wpy > map_end_y) {
                t = std::min(t, (map_end_y - oy) / b);
            }

            double wpx_scaled = ox + a * t;
            double wpy_scaled = oy + b * t;
            double wpz_scaled = oz + c * t;

            double point_x, point_y, point_z;
            if (worldToMap3DFloat(wpx_scaled, wpy_scaled, wpz_scaled, point_x, point_y, point_z)) {
                unsigned int cell_raytrace_max_range = cellDistance(obs.raytrace_max_range_);
                unsigned int cell_raytrace_min_range = cellDistance(obs.raytrace_min_range_);

                unsigned int mx = static_cast<unsigned int>(point_x), my = static_cast<unsigned int>(point_y), mz = static_cast<unsigned int>(point_z);  // raytracing endpoint

                // do not ray trace if the raytracing endpoint has already been cleared (a ray has already been traced between this observation's origin and approximately in the same direction)
                if(!voxel_grid_.isVoxelCleared(mx ,my, mz)){
                    voxel_grid_.clearVoxelLine(sensor_x, sensor_y, sensor_z, point_x, point_y, point_z, cell_raytrace_max_range, cell_raytrace_min_range);
                }

                updateRaytraceBounds(ox, oy, wpx_scaled, wpy_scaled, obs.raytrace_max_range_, obs.raytrace_min_range_, min_x, min_y, max_x, max_y);

                if (publish_clearing_points) {
                    *clearing_endpoints_iter_x = wpx_scaled;
                    *clearing_endpoints_iter_y = wpy_scaled;
                    *clearing_endpoints_iter_z = wpz_scaled;

                    ++clearing_endpoints_iter_x;
                    ++clearing_endpoints_iter_y;
                    ++clearing_endpoints_iter_z;
                }
            }
        }

        if (publish_clearing_points) {
            clearing_endpoints_->header.frame_id = global_frame_;
            clearing_endpoints_->header.stamp = obs.cloud_->header.stamp;

            clearing_endpoints_pub_->publish(std::move(clearing_endpoints_));
        }

        raytracing_duration_s += std::chrono::high_resolution_clock::now() - raytracing_start;

        //**********************************************//
        //                     end                      //
        //              raytrace freespace              //
        //                                              //
        //**********************************************//

//        auto transfer_start = std::chrono::high_resolution_clock::now();
        voxel_grid_.transferToCostmap(LETHAL_OBSTACLE, FREE_SPACE, NO_INFORMATION, unknown_threshold_, mark_threshold_, costmap_);
//        std::chrono::duration<double, std::milli> transfer_duration = std::chrono::high_resolution_clock::now() - transfer_start;
//        RCLCPP_INFO(logger_, "transfer_duration: %.3f ms", transfer_duration.count());

    }

    if (publish_voxel_) {
        auto grid_msg = std::make_unique<nav2_msgs::msg::VoxelGrid>();
        unsigned int size = voxel_grid_.sizeX() * voxel_grid_.sizeY();
        grid_msg->size_x = voxel_grid_.sizeX();
        grid_msg->size_y = voxel_grid_.sizeY();
        grid_msg->size_z = voxel_grid_.sizeZ();
        grid_msg->data.resize(size);
        memcpy(&grid_msg->data[0], voxel_grid_.getData(), size * sizeof(unsigned int));

        grid_msg->origin.x = origin_x_;
        grid_msg->origin.y = origin_y_;
        grid_msg->origin.z = origin_z_;

        grid_msg->resolutions.x = resolution_;
        grid_msg->resolutions.y = resolution_;
        grid_msg->resolutions.z = z_resolution_;
        grid_msg->header.frame_id = global_frame_;
        grid_msg->header.stamp = clock_->now();

        voxel_pub_->publish(std::move(grid_msg));
    }

    updateFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);

    std::chrono::duration<double> update_bounds_duration_s = std::chrono::high_resolution_clock::now() - update_bounds_start;
    auto node = node_.lock();
    if (!node) {
        throw std::runtime_error{"Failed to lock node"};
    }
    auto now = node->get_clock()->now();

    asb_msgs::msg::ExecutionDurationStamped raytracing_duration_msg;
    raytracing_duration_msg.stamp = now;
    raytracing_duration_msg.execution_duration = rclcpp::Duration::from_seconds((raytracing_duration_s).count());
    raytracing_duration_msg.label = "raytracing";
    benchmarking_execution_duration_publisher_->publish(raytracing_duration_msg);

    asb_msgs::msg::ExecutionDurationStamped update_bounds_duration_msg;
    update_bounds_duration_msg.stamp = node->get_clock()->now();
    update_bounds_duration_msg.execution_duration = rclcpp::Duration::from_seconds((update_bounds_duration_s).count());
    update_bounds_duration_msg.label = "update_bounds";
    benchmarking_execution_duration_publisher_->publish(update_bounds_duration_msg);
}

void VoxelLayer::updateOrigin(double new_origin_x, double new_origin_y) {
    // project the new origin into the grid
    int cell_ox, cell_oy;
    cell_ox = static_cast<int>((new_origin_x - origin_x_) / resolution_);
    cell_oy = static_cast<int>((new_origin_y - origin_y_) / resolution_);

    // compute the associated world coordinates for the origin cell
    // because we want to keep things grid-aligned
    double new_grid_ox, new_grid_oy;
    new_grid_ox = origin_x_ + cell_ox * resolution_;
    new_grid_oy = origin_y_ + cell_oy * resolution_;

    // To save casting from unsigned int to int a bunch of times
    int size_x = size_x_;
    int size_y = size_y_;

    // we need to compute the overlap of the new and existing windows
    int lower_left_x, lower_left_y, upper_right_x, upper_right_y;
    lower_left_x = std::min(std::max(cell_ox, 0), size_x);
    lower_left_y = std::min(std::max(cell_oy, 0), size_y);
    upper_right_x = std::min(std::max(cell_ox + size_x, 0), size_x);
    upper_right_y = std::min(std::max(cell_oy + size_y, 0), size_y);

    unsigned int cell_size_x = upper_right_x - lower_left_x;
    unsigned int cell_size_y = upper_right_y - lower_left_y;

    // we need a map to store the obstacles in the window temporarily
    unsigned char *local_map = new unsigned char[cell_size_x * cell_size_y];
    unsigned int *local_voxel_map = new unsigned int[cell_size_x * cell_size_y];
    unsigned int *voxel_map = voxel_grid_.getData();

    // copy the local window in the costmap to the local map
    copyMapRegion(costmap_, lower_left_x, lower_left_y, size_x_, local_map, 0, 0, cell_size_x, cell_size_x, cell_size_y);
    copyMapRegion(voxel_map, lower_left_x, lower_left_y, size_x_, local_voxel_map, 0, 0, cell_size_x, cell_size_x, cell_size_y);

    // we'll reset our maps to unknown space if appropriate
    resetMaps();

    // update the origin with the appropriate world coordinates
    origin_x_ = new_grid_ox;
    origin_y_ = new_grid_oy;

    // compute the starting cell location for copying data back in
    int start_x = lower_left_x - cell_ox;
    int start_y = lower_left_y - cell_oy;

    // now we want to copy the overlapping information back into the map, but in its new location
    copyMapRegion(local_map, 0, 0, cell_size_x, costmap_, start_x, start_y, size_x_, cell_size_x, cell_size_y);
    copyMapRegion(local_voxel_map, 0, 0, cell_size_x, voxel_map, start_x, start_y, size_x_, cell_size_x, cell_size_y);

    // make sure to clean up
    delete[] local_map;
    delete[] local_voxel_map;
}

/**
  * @brief Callback executed when a parameter change is detected
  * @param event ParameterEvent message
  */
rcl_interfaces::msg::SetParametersResult VoxelLayer::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters) {

    std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
    rcl_interfaces::msg::SetParametersResult result;
    bool resize_map_needed = false;

    for (auto parameter: parameters) {
        const auto &param_type = parameter.get_type();
        const auto &param_name = parameter.get_name();

        if (param_type == ParameterType::PARAMETER_DOUBLE) {
            if (param_name == name_ + "." + "max_obstacle_height") {
                max_obstacle_height_ = parameter.as_double();
            } else if (param_name == name_ + "." + "origin_z") {
                origin_z_ = parameter.as_double();
                resize_map_needed = true;
            } else if (param_name == name_ + "." + "z_resolution") {
                z_resolution_ = parameter.as_double();
                resize_map_needed = true;
            }
        } else if (param_type == ParameterType::PARAMETER_BOOL) {
            if (param_name == name_ + "." + "enabled") {
                enabled_ = parameter.as_bool();
                current_ = false;
            } else if (param_name == name_ + "." + "footprint_clearing_enabled") {
                footprint_clearing_enabled_ = parameter.as_bool();
            } else if (param_name == name_ + "." + "publish_voxel_map") {
                RCLCPP_WARN(logger_, "publish voxel map is not a dynamic parameter "
                                     "cannot be changed while running. Rejecting parameter update.");
                continue;
            }

        } else if (param_type == ParameterType::PARAMETER_INTEGER) {
            if (param_name == name_ + "." + "z_voxels") {
                size_z_ = parameter.as_int();
                resize_map_needed = true;
            } else if (param_name == name_ + "." + "unknown_threshold") {
                unknown_threshold_ = parameter.as_int() + (VOXEL_BITS - size_z_);
            } else if (param_name == name_ + "." + "mark_threshold") {
                mark_threshold_ = parameter.as_int();
            } else if (param_name == name_ + "." + "combination_method") {
                combination_method_ = parameter.as_int();
            }
        }
    }

    if (resize_map_needed) {
        matchSize();
    }

    result.successful = true;
    return result;
}

}  // namespace asb_costmap_2d_plugins