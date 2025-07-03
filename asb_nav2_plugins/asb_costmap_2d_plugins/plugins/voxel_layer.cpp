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
    declareParameter("volatile_update", rclcpp::ParameterValue(false));
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

    node->get_parameter(name_ + ".enabled", enabled_);
    node->get_parameter(name_ + ".publish_voxel_map", publish_voxel_);
    node->get_parameter(name_ + ".footprint_clearing_enabled", footprint_clearing_enabled_);
    node->get_parameter(name_ + ".max_obstacle_height", max_obstacle_height_);
    node->get_parameter(name_ + ".z_voxels", size_z_);
    node->get_parameter(name_ + ".origin_z", origin_z_);
    node->get_parameter(name_ + ".z_resolution", z_resolution_);
    node->get_parameter(name_ + ".unknown_threshold", unknown_threshold_);
    node->get_parameter(name_ + ".mark_threshold", mark_threshold_);
    node->get_parameter(name_ + ".combination_method", combination_method_);
    node->get_parameter(name_ + ".volatile_update", volatile_update_);

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
    dyn_params_handler_ = node->add_on_set_parameters_callback(std::bind(&VoxelLayer::voxelLayerDynamicParametersCallback, this, std::placeholders::_1));

    RCLCPP_INFO(logger_, "ASB voxel_layer initialized");

}

VoxelLayer::~VoxelLayer() {

    dyn_params_handler_.reset();
}

void VoxelLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double * min_x, double * min_y, double * max_x, double * max_y) {

    std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
    auto update_bounds_start = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> raytracing_duration_s(0);

    if (rolling_window_ && !volatile_update_) {
        updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
    }
    if (!enabled_) {
        return;
    }
    useExtraBounds(min_x, min_y, max_x, max_y);

    std::vector<nav2_costmap_2d::Observation> observations;
    bool current = getObservations(observations);

    // update the global current status
    current_ = current;

    // resize the voxel grid (this also resets it)
    voxel_grid_.resize(size_x_, size_y_, size_z_, volatile_update_);

    for (unsigned int i = 0; i < observations.size(); ++i) {
        const nav2_costmap_2d::Observation & obs = observations[i];
        asb_voxel_grid::VoxelGrid voxel_grid_obs(voxel_grid_.sizeX(), voxel_grid_.sizeY(), voxel_grid_.sizeZ());

        //**********************************************//
        //                mark obstacles                //
        //**********************************************//

        const sensor_msgs::msg::PointCloud2 & cloud = *(obs.cloud_);

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
            if (voxel_grid_obs.markVoxel(mx, my, mz, mark_threshold_)) {
                unsigned int index = getIndex(mx, my);

                costmap_[index] = LETHAL_OBSTACLE;
                touch(static_cast<double>(*iter_m_x), static_cast<double>(*iter_m_y), min_x, min_y, max_x, max_y);
            }
        }

        if (!volatile_update_) {

            //**********************************************//
            //              raytrace freespace              //
            //**********************************************//

            auto raytracing_start = std::chrono::high_resolution_clock::now();

            auto clearing_endpoints_ = std::make_unique<sensor_msgs::msg::PointCloud2>();

            if (obs.cloud_->height == 0 || obs.cloud_->width == 0) {
                continue;
            }

            double sensor_origin_x = obs.origin_.x;
            double sensor_origin_y = obs.origin_.y;
            double sensor_origin_z = obs.origin_.z;

            double map_end_x = origin_x_ + getSizeInMetersX();
            double map_end_y = origin_y_ + getSizeInMetersY();
            double map_end_z = origin_z_ + getSizeInMetersZ();

            double sensor_origin_m_x, sensor_origin_m_y, sensor_origin_m_z;
            if (!worldToMap3DFloat(sensor_origin_x, sensor_origin_y, sensor_origin_z, sensor_origin_m_x, sensor_origin_m_y, sensor_origin_m_z)) {
                RCLCPP_WARN(logger_,
                        "Sensor origin at (%.2f, %.2f %.2f) is out of map bounds "
                        "(%.2f, %.2f, %.2f) to (%.2f, %.2f, %.2f). "
                        "The costmap cannot raytrace for it.",
                        sensor_origin_x, sensor_origin_y, sensor_origin_z,
                        origin_x_, origin_y_, origin_z_,
                        map_end_x, map_end_y, map_end_z);

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

            unsigned int raytrace_max_range_m = cellDistance(obs.raytrace_max_range_);
            unsigned int raytrace_min_range_m = cellDistance(obs.raytrace_min_range_);

            sensor_msgs::PointCloud2ConstIterator<float> iter_x(*(obs.cloud_), "x");
            sensor_msgs::PointCloud2ConstIterator<float> iter_y(*(obs.cloud_), "y");
            sensor_msgs::PointCloud2ConstIterator<float> iter_z(*(obs.cloud_), "z");
            for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
                // raytracing endpoint in world coordinates
                double endpoint_x = *iter_x;
                double endpoint_y = *iter_y;
                double endpoint_z = *iter_z;

                double distance = dist(sensor_origin_x, sensor_origin_y, sensor_origin_z, endpoint_x, endpoint_y, endpoint_z);
                double scaling_fact = 1.0;
                scaling_fact = std::max(std::min(scaling_fact, (distance - 2 * resolution_) / distance), 0.0);
                endpoint_x = scaling_fact * (endpoint_x - sensor_origin_x) + sensor_origin_x;
                endpoint_y = scaling_fact * (endpoint_y - sensor_origin_y) + sensor_origin_y;
                endpoint_z = scaling_fact * (endpoint_z - sensor_origin_z) + sensor_origin_z;

                double a = endpoint_x - sensor_origin_x;
                double b = endpoint_y - sensor_origin_y;
                double c = endpoint_z - sensor_origin_z;
                double t = 1.0;

                // rescale the endpoint if z is outside of map boundaries
                if (endpoint_z < origin_z_) {
                    t = std::min(t, (origin_z_ - sensor_origin_z) / c);
                } else if (endpoint_z > map_end_z) {
                    t = std::max(0.0, std::min(t, (map_end_z - 0.01 - sensor_origin_z) / c));
                }

                // rescale the endpoint if x is outside of map boundaries
                if (endpoint_x < origin_x_) {
                    t = std::min(t, (origin_x_ - sensor_origin_x) / a);
                } else if (endpoint_x > map_end_x) {
                    t = std::min(t, (map_end_x - sensor_origin_x) / a);
                }

                // rescale the endpoint if y is outside of map boundaries
                if (endpoint_y < origin_y_) {
                    t = std::min(t, (origin_y_ - sensor_origin_y) / b);
                } else if (endpoint_y > map_end_y) {
                    t = std::min(t, (map_end_y - sensor_origin_y) / b);
                }

                endpoint_x = sensor_origin_x + a * t;
                endpoint_y = sensor_origin_y + b * t;
                endpoint_z = sensor_origin_z + c * t;

                double endpoint_m_x, endpoint_m_y, endpoint_m_z;
                if (worldToMap3DFloat(endpoint_x, endpoint_y, endpoint_z, endpoint_m_x, endpoint_m_y, endpoint_m_z)) {

                    // do not ray trace if the raytracing endpoint has already been cleared (a ray has already been traced between this observation's origin and approximately in the same direction)
                    if (!voxel_grid_obs.isVoxelCleared(static_cast<unsigned int>(endpoint_m_x), static_cast<unsigned int>(endpoint_m_y), static_cast<unsigned int>(endpoint_m_z))) {
                        voxel_grid_obs.clearVoxelLine(sensor_origin_m_x, sensor_origin_m_y, sensor_origin_m_z, endpoint_m_x, endpoint_m_y, endpoint_m_z, raytrace_max_range_m, raytrace_min_range_m);
                    }

                    updateRaytraceBounds(sensor_origin_x, sensor_origin_y, endpoint_x, endpoint_y, obs.raytrace_max_range_, obs.raytrace_min_range_, min_x, min_y, max_x, max_y);

                    if (publish_clearing_points) {
                        *clearing_endpoints_iter_x = static_cast<float>(endpoint_x);
                        *clearing_endpoints_iter_y = static_cast<float>(endpoint_y);
                        *clearing_endpoints_iter_z = static_cast<float>(endpoint_z);

                        ++clearing_endpoints_iter_x;
                        ++clearing_endpoints_iter_y;
                        ++clearing_endpoints_iter_z;
                    }
                }
            }

            raytracing_duration_s += std::chrono::high_resolution_clock::now() - raytracing_start;

            //**********************************************//
            //           end raytrace freespace             //
            //**********************************************//

            if (publish_clearing_points) {
                clearing_endpoints_->header.frame_id = global_frame_;
                clearing_endpoints_->header.stamp = obs.cloud_->header.stamp;

                clearing_endpoints_pub_->publish(std::move(clearing_endpoints_));
            }

        }
        voxel_grid_obs.transferTo(voxel_grid_);

    }

    voxel_grid_.transferToCostmap(LETHAL_OBSTACLE, FREE_SPACE, NO_INFORMATION, unknown_threshold_, mark_threshold_, costmap_);

    if (publish_voxel_) {
        auto grid_msg = std::make_unique<nav2_msgs::msg::VoxelGrid>();
        unsigned int size = voxel_grid_.sizeX() * voxel_grid_.sizeY();
        grid_msg->size_x = voxel_grid_.sizeX();
        grid_msg->size_y = voxel_grid_.sizeY();
        grid_msg->size_z = voxel_grid_.sizeZ();
        grid_msg->data.resize(size);
        memcpy(&grid_msg->data[0], voxel_grid_.getData(), size * sizeof(unsigned int));

        grid_msg->origin.x = static_cast<float>(origin_x_);
        grid_msg->origin.y = static_cast<float>(origin_y_);
        grid_msg->origin.z = static_cast<float>(origin_z_);

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

/**
  * @brief Callback executed when a parameter change is detected
  * @param event ParameterEvent message
  */
rcl_interfaces::msg::SetParametersResult VoxelLayer::voxelLayerDynamicParametersCallback(const std::vector<rclcpp::Parameter> & parameters) {

    std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
    rcl_interfaces::msg::SetParametersResult result;
    bool resize_map_needed = false;

    for (const auto & parameter: parameters) {
        const auto & param_type = parameter.get_type();
        const auto & param_name = parameter.get_name();

        if (param_type == ParameterType::PARAMETER_DOUBLE) {
            if (param_name == name_ + ".origin_z") {
                origin_z_ = parameter.as_double();
                resize_map_needed = true;
            } else if (param_name == name_ + ".z_resolution") {
                z_resolution_ = parameter.as_double();
                resize_map_needed = true;
            }
        } else if (param_type == ParameterType::PARAMETER_BOOL) {
            if (param_name == name_ + ".volatile_update") {
                volatile_update_ = parameter.as_bool();
            } else if (param_name == name_ + ".publish_voxel_map") {
                RCLCPP_WARN(logger_, "publish voxel map is not a dynamic parameter "
                                     "cannot be changed while running. Rejecting parameter update.");
                continue;
            }

        } else if (param_type == ParameterType::PARAMETER_INTEGER) {
            if (param_name == name_ + ".z_voxels") {
                size_z_ = static_cast<int>(parameter.as_int());
                resize_map_needed = true;
            } else if (param_name == name_ + ".unknown_threshold") {
                unknown_threshold_ = static_cast<int>(parameter.as_int()) + (VOXEL_BITS - size_z_);
            } else if (param_name == name_ + ".mark_threshold") {
                mark_threshold_ = static_cast<int>(parameter.as_int());
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