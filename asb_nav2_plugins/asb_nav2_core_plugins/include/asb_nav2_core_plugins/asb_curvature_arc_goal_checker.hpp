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

#ifndef ASB_NAV2_CORE_PLUGINS__ASB_CURVATURE_ARC_GOAL_CHECKER_HPP_
#define ASB_NAV2_CORE_PLUGINS__ASB_CURVATURE_ARC_GOAL_CHECKER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_core/goal_checker.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Twist;

namespace asb_nav2_core_plugins {

/**
 * @class ASBCurvatureArcGoalChecker
 * @brief Goal Checker plugin that considers the goal reached if the robot pose is such that the robot can follow a target straight path
 * starting the goal pose's x-axis with a minimum turning radius without straying from the target path more than a tolerance.
 */
class ASBCurvatureArcGoalChecker : public nav2_core::GoalChecker {
public:
    ASBCurvatureArcGoalChecker();

    // Standard GoalChecker Interface
    void initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, const std::string &plugin_name, const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

    void reset() override;

    bool isGoalReached(const Pose &query_pose, const Pose &goal_pose, const Twist &velocity) override;

    bool getTolerances(Pose &pose_tolerance, Twist &vel_tolerance) override;

protected:
    double xy_goal_tolerance_, lookahead_dist_, path_constraint_x_, path_constraint_y_;

    rclcpp::Logger logger_{rclcpp::get_logger("ASBCurvatureArcGoalChecker")};

    // Dynamic parameters handler
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
    std::string plugin_name_;

    static tf2::Transform getRobotToGoalTransform(const Pose &goal_pose, const Pose &robot_pose);

    /**
     * @brief Find the intersection points between a circle with center in the origin and radius r, and a line that projects from the pose p (only forward)
     * @return true if an intersection exists, false otherwise
     */
    static bool findRadiusPoseIntersection(const Pose &p, const double &r, Point &p_int);

    Point getExtendedLookaheadPoint(const Pose &path_pose, bool &valid_solution) const;

    double getLookaheadCurvature(Point lookahead_point) const;

    static Pose get_pose_c_to_r(const Point &point_in_c, const tf2::Transform &tf_r_to_c);

    /**
     * @brief Callback executed when a parameter change is detected
     * @param parameters list of changed parameters
     */
    rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);
};

}  // namespace asb_nav2_core_plugins

#endif  // ASB_NAV2_CORE_PLUGINS__ASB_CURVATURE_ARC_GOAL_CHECKER_HPP_
