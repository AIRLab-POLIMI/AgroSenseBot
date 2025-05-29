/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2024 Università degli Studi di Milano, Enrico Piazza
 *  Copyright (c) 2017, Locus Robotics
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
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <memory>
#include <string>
#include <limits>
#include <vector>
#include <cmath>
#include "asb_nav2_plugins/plugins/asb_rotate_to_path_goal_checker.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "angles/angles.h"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"

#include "tf2/utils.h"

#pragma GCC diagnostic pop

using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace asb_nav2_plugins {

ASBRotateToPathGoalChecker::ASBRotateToPathGoalChecker() : xy_goal_tolerance_(0.25), yaw_goal_tolerance_(0.25), forward_(true), in_goal_proximity_(false), xy_goal_tolerance_sq_(0.0625) {
}

void ASBRotateToPathGoalChecker::initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, const std::string &plugin_name, const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>/*costmap_ros*/) {
    plugin_name_ = plugin_name;
    auto node = parent.lock();

    nav2_util::declare_parameter_if_not_declared(node, plugin_name + ".xy_goal_tolerance", rclcpp::ParameterValue(0.25));
    nav2_util::declare_parameter_if_not_declared(node, plugin_name + ".yaw_goal_tolerance", rclcpp::ParameterValue(0.25));

    node->get_parameter(plugin_name + ".xy_goal_tolerance", xy_goal_tolerance_);
    node->get_parameter(plugin_name + ".yaw_goal_tolerance", yaw_goal_tolerance_);

    xy_goal_tolerance_sq_ = xy_goal_tolerance_ * xy_goal_tolerance_;

    // Add callback for dynamic parameters
    dyn_params_handler_ = node->add_on_set_parameters_callback(std::bind(&ASBRotateToPathGoalChecker::dynamicParametersCallback, this, _1));

    RCLCPP_INFO(node->get_logger(), "ASBRotateToPathGoalChecker initialized");
}

void ASBRotateToPathGoalChecker::reset() {
    in_goal_proximity_ = false;
}

bool ASBRotateToPathGoalChecker::isGoalReached(const geometry_msgs::msg::Pose &query_pose, const geometry_msgs::msg::Pose &goal_pose, const geometry_msgs::msg::Twist &) {
    // Extract goal pose orientation and compute forward x-axis unit vector
    tf2::Quaternion goal_q(goal_pose.orientation.x, goal_pose.orientation.y, goal_pose.orientation.z, goal_pose.orientation.w);

    tf2::Vector3 goal_x_axis = tf2::quatRotate(goal_q, tf2::Vector3(1, 0, 0));

    // Compute start of goal segment (goal_pose position - xy_goal_tolerance_ * x-axis)
    tf2::Vector3 goal_position(goal_pose.position.x, goal_pose.position.y, 0);
    tf2::Vector3 start_position = goal_position - xy_goal_tolerance_ * goal_x_axis;

    // Get query_pose x-axis vector (from orientation)
    tf2::Quaternion query_q(query_pose.orientation.x, query_pose.orientation.y, query_pose.orientation.z, query_pose.orientation.w);

    tf2::Vector3 query_x_axis = tf2::quatRotate(query_q, tf2::Vector3(1, 0, 0));
    tf2::Vector3 query_position(query_pose.position.x, query_pose.position.y, 0);

    // Check if query_x_axis intersects the segment [start_position, goal_position]
    // We treat query_pose's x-axis as a ray: p = query_position + t * query_x_axis

    // Let the segment be from A to B:
    tf2::Vector3 A = start_position;
    tf2::Vector3 B = goal_position;

    tf2::Vector3 v = query_x_axis;         // direction of the ray
    tf2::Vector3 w = B - A;                // direction of the segment
    tf2::Vector3 u = A - query_position;   // vector from ray origin to segment start

    double denom = v.x() * w.y() - v.y() * w.x();

    // Check if they are parallel
    if (std::abs(denom) < 1e-6)
        return false;

    // Compute intersection parameters
    double s = (u.x() * w.y() - u.y() * w.x()) / denom;
    double t = (u.x() * v.y() - u.y() * v.x()) / denom;

    // Ray (s >= 0), segment (0 <= t <= 1)
    if (s >= 0.0 && t >= 0.0 && t <= 1.0)
        return true;

    return false;
}

bool ASBRotateToPathGoalChecker::getTolerances(geometry_msgs::msg::Pose &pose_tolerance, geometry_msgs::msg::Twist &vel_tolerance) {
    double invalid_field = std::numeric_limits<double>::lowest();

    pose_tolerance.position.x = xy_goal_tolerance_;
    pose_tolerance.position.y = xy_goal_tolerance_;
    pose_tolerance.position.z = invalid_field;
    pose_tolerance.orientation = nav2_util::geometry_utils::orientationAroundZAxis(yaw_goal_tolerance_);

    vel_tolerance.linear.x = invalid_field;
    vel_tolerance.linear.y = invalid_field;
    vel_tolerance.linear.z = invalid_field;

    vel_tolerance.angular.x = invalid_field;
    vel_tolerance.angular.y = invalid_field;
    vel_tolerance.angular.z = invalid_field;

    return true;
}

rcl_interfaces::msg::SetParametersResult ASBRotateToPathGoalChecker::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    for (auto &parameter: parameters) {
        const auto &type = parameter.get_type();
        const auto &name = parameter.get_name();

        if (type == ParameterType::PARAMETER_DOUBLE) {
            if (name == plugin_name_ + ".xy_goal_tolerance") {
                xy_goal_tolerance_ = parameter.as_double();
                xy_goal_tolerance_sq_ = xy_goal_tolerance_ * xy_goal_tolerance_;
            } else if (name == plugin_name_ + ".yaw_goal_tolerance") {
                yaw_goal_tolerance_ = parameter.as_double();
            }
        }
    }
    result.successful = true;
    return result;
}

}  // namespace asb_nav2_plugins

PLUGINLIB_EXPORT_CLASS(asb_nav2_plugins::ASBRotateToPathGoalChecker, nav2_core::GoalChecker)
