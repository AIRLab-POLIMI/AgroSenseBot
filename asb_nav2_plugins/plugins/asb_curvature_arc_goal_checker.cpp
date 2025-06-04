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
#include "asb_nav2_plugins/plugins/asb_curvature_arc_goal_checker.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "angles/angles.h"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "tf2/utils.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"


#pragma GCC diagnostic pop

using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace asb_nav2_plugins {

ASBCurvatureArcGoalChecker::ASBCurvatureArcGoalChecker() : xy_goal_tolerance_(0.25), lookahead_dist_(2.0), path_constraint_x_(1.0), path_constraint_y_(0.1) {
}

void ASBCurvatureArcGoalChecker::initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, const std::string &plugin_name, const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>/*costmap_ros*/) {
    plugin_name_ = plugin_name;
    auto node = parent.lock();

    nav2_util::declare_parameter_if_not_declared(node, plugin_name + ".xy_goal_tolerance", rclcpp::ParameterValue(0.25));
    nav2_util::declare_parameter_if_not_declared(node, plugin_name + ".yaw_goal_tolerance", rclcpp::ParameterValue(0.25));

    node->get_parameter(plugin_name + ".xy_goal_tolerance", xy_goal_tolerance_);
    node->get_parameter(plugin_name + ".lookahead_dist", lookahead_dist_);
    node->get_parameter(plugin_name + ".path_constraint_x", path_constraint_x_);
    node->get_parameter(plugin_name + ".path_constraint_y", path_constraint_y_);

    // Add callback for dynamic parameters
    dyn_params_handler_ = node->add_on_set_parameters_callback(std::bind(&ASBCurvatureArcGoalChecker::dynamicParametersCallback, this, _1));

    logger_ = node->get_logger();

    RCLCPP_INFO(node->get_logger(), "ASBCurvatureArcGoalChecker initialized");
}

void ASBCurvatureArcGoalChecker::reset() {
}

bool ASBCurvatureArcGoalChecker::isGoalReached(const Pose &query_pose, const Pose &goal_pose, const Twist &) {

    // compute the goal pose in the robot frame (the frame of reference defined by query_pose)
    tf2::Transform tf_r_to_g = getRobotToGoalTransform(goal_pose, query_pose);
    Pose g_in_r;
    toMsg(tf_r_to_g, g_in_r);
    RCLCPP_INFO(logger_, "query in fixed frame  x: %+.3f  y: %+.3f  theta: %+.3f", query_pose.position.x, query_pose.position.y, tf2::getYaw(query_pose.orientation));
    RCLCPP_INFO(logger_, "goal in fixed frame   x: %+.3f  y: %+.3f  theta: %+.3f", goal_pose.position.x, goal_pose.position.y, tf2::getYaw(goal_pose.orientation));
    RCLCPP_INFO(logger_, "goal in robot frame   x: %+.3f  y: %+.3f  theta: %+.3f", g_in_r.position.x, g_in_r.position.y, tf2::getYaw(g_in_r.orientation));

    if (std::hypot(g_in_r.position.x, g_in_r.position.y) > xy_goal_tolerance_) {
        RCLCPP_INFO(logger_, "GOAL NOT REACHED\n\n");
        return false;
    }

    // get the lookahead point for the lookahead distance and the straight path defined by the goal (same as carrot point in RPP)
    bool valid_solution;
    Point lookahead_point = getExtendedLookaheadPoint(g_in_r, valid_solution);
    if (!valid_solution) {
        RCLCPP_INFO(logger_, "GOAL NOT REACHED\n\n");
        return false;
    }

    double c = getLookaheadCurvature(lookahead_point);

    RCLCPP_INFO(logger_, "curvature   c: %+.3f  ", c);

    tf2::Transform tf_g_to_clx(tf2::Quaternion(tf2::Vector3(0, 0, 1), 0), tf2::Vector3(path_constraint_x_, path_constraint_y_, 0));
    tf2::Transform tf_g_to_cly(tf2::Quaternion(tf2::Vector3(0, 0, 1), M_PI / 2), tf2::Vector3(path_constraint_x_, path_constraint_y_, 0));
    tf2::Transform tf_g_to_crx(tf2::Quaternion(tf2::Vector3(0, 0, 1), 0), tf2::Vector3(path_constraint_x_, -path_constraint_y_, 0));
    tf2::Transform tf_g_to_cry(tf2::Quaternion(tf2::Vector3(0, 0, 1), -M_PI / 2), tf2::Vector3(path_constraint_x_, -path_constraint_y_, 0));

    tf2::Transform tf_g_to_clf(tf2::Quaternion(tf2::Vector3(0, 0, 1), 0), tf2::Vector3(path_constraint_x_ * 4, path_constraint_y_, 0));
    tf2::Transform tf_g_to_crf(tf2::Quaternion(tf2::Vector3(0, 0, 1), -M_PI / 2), tf2::Vector3(path_constraint_x_ * 4, -path_constraint_y_, 0));

    Pose p_clx_in_r, p_cly_in_r, p_crx_in_r, p_cry_in_r, p_clf_in_r, p_crf_in_r;  // constraint poses in robot frame
    tf2::Transform tf_r_to_clx = tf_r_to_g * tf_g_to_clx;
    tf2::Transform tf_r_to_cly = tf_r_to_g * tf_g_to_cly;
    tf2::Transform tf_r_to_crx = tf_r_to_g * tf_g_to_crx;
    tf2::Transform tf_r_to_cry = tf_r_to_g * tf_g_to_cry;

    tf2::Transform tf_r_to_clf = tf_r_to_g * tf_g_to_clf;
    tf2::Transform tf_r_to_crf = tf_r_to_g * tf_g_to_crf;

    toMsg(tf_r_to_clx, p_clx_in_r);
    toMsg(tf_r_to_cly, p_cly_in_r);
    toMsg(tf_r_to_crx, p_crx_in_r);
    toMsg(tf_r_to_cry, p_cry_in_r);

    toMsg(tf_r_to_crf, p_crf_in_r);
    toMsg(tf_r_to_clf, p_clf_in_r);

    Pose p_c_in_r;  // center of rotation in robot frame
    p_c_in_r.position.y = 1 / c;
    p_c_in_r.orientation.w = 1;
    tf2::Transform tf_r_to_c;
    tf2::fromMsg(p_c_in_r, tf_r_to_c);
    tf2::Transform tf_c_to_r = tf_r_to_c.inverse();  // transform from center of rotation to the robot frame

    Pose p_l_in_r;  // lookahead pose in robot frame
    p_l_in_r.position = lookahead_point;
    p_l_in_r.orientation.w = 1;
    Pose p_l_in_c;  // lookahead pose in center of rotation frame
    tf2::Transform tf_r_to_l;
    tf2::fromMsg(p_l_in_r, tf_r_to_l);
    tf2::Transform tf_c_to_l = tf_c_to_r * tf_r_to_l;
    toMsg(tf_c_to_l, p_l_in_c);

    if (c == 0.0) {
        RCLCPP_INFO(logger_, "GOAL NOT REACHED\n\n");
        return false;  // in the extremely rare case in which the curvature is exactly 0, return
    }

    double r = std::fabs(1 / c);  // curvature radius
    if (r > 100) {
        RCLCPP_WARN(logger_, "radius: %+.3f", r);
    } else {
        RCLCPP_INFO(logger_, "radius: %+.3f", r);
    }


    Pose p_clx_in_c, p_cly_in_c, p_crx_in_c, p_cry_in_c;  // constraint poses in center of rotation frame
    tf2::Transform tf_c_to_clx = tf_c_to_r * tf_r_to_g * tf_g_to_clx;
    tf2::Transform tf_c_to_cly = tf_c_to_r * tf_r_to_g * tf_g_to_cly;
    tf2::Transform tf_c_to_crx = tf_c_to_r * tf_r_to_g * tf_g_to_crx;
    tf2::Transform tf_c_to_cry = tf_c_to_r * tf_r_to_g * tf_g_to_cry;
    toMsg(tf_c_to_clx, p_clx_in_c);
    toMsg(tf_c_to_cly, p_cly_in_c);
    toMsg(tf_c_to_crx, p_crx_in_c);
    toMsg(tf_c_to_cry, p_cry_in_c);

    double theta_1, theta_2;
    double theta_o = c > 0 ? -M_PI/2 : M_PI/2;
    double theta_l = std::atan2(p_l_in_c.position.y, p_l_in_c.position.x);
    if (c > 0) {
        theta_1 = theta_o;
        theta_2 = theta_l;
    } else {
        theta_1 = theta_l;
        theta_2 = theta_o;
    }
    RCLCPP_INFO(logger_, "int angles   theta_1: %+.3f    theta_2: %+.3f\n", theta_1, theta_2);

    // find the intersection points of the curvature arc with the constraint segments (in the center of rotation frame)
    Point p_clx_int_in_c;
    if (findRadiusPoseIntersection(p_clx_in_c, r, p_clx_int_in_c)) {
        // find intersection angle
        double theta_int = std::atan2(p_clx_int_in_c.y, p_clx_int_in_c.x);
        if (theta_1 < theta_int && theta_int < theta_2) {
            RCLCPP_INFO(logger_, "int   clx   theta_int: %+.3f   x: %+.3f y: %+.3f", theta_int, p_clx_int_in_c.x, p_clx_int_in_c.y);
            RCLCPP_INFO(logger_, "GOAL NOT REACHED\n\n");
            return false;
        } else {
            RCLCPP_INFO(logger_, "int   clx");
        }
    } else {
        RCLCPP_INFO(logger_, "int   clx");
    }

    Point p_cly_int_in_c;
    if (findRadiusPoseIntersection(p_cly_in_c, r, p_cly_int_in_c)) {
        // find intersection angle
        double theta_int = std::atan2(p_cly_int_in_c.y, p_cly_int_in_c.x);
        if (theta_1 < theta_int && theta_int < theta_2) {
            RCLCPP_INFO(logger_, "int   cly   theta_int: %+.3f   x: %+.3f y: %+.3f", theta_int, p_cly_int_in_c.x, p_cly_int_in_c.y);
            RCLCPP_INFO(logger_, "GOAL NOT REACHED\n\n");
            return false;
        } else {
            RCLCPP_INFO(logger_, "int   cly");
        }
    } else {
        RCLCPP_INFO(logger_, "int   cly");
    }

    Point p_crx_int_in_c;
    if (findRadiusPoseIntersection(p_crx_in_c, r, p_crx_int_in_c)) {
        // find intersection angle
        double theta_int = std::atan2(p_crx_int_in_c.y, p_crx_int_in_c.x);
        if (theta_1 < theta_int && theta_int < theta_2) {
            RCLCPP_INFO(logger_, "int   crx   theta_int: %+.3f   x: %+.3f y: %+.3f", theta_int, p_crx_int_in_c.x, p_crx_int_in_c.y);
            RCLCPP_INFO(logger_, "GOAL NOT REACHED\n\n");
            return false;
        } else {
            RCLCPP_INFO(logger_, "int   crx");
        }
    } else {
        RCLCPP_INFO(logger_, "int   crx");
    }

    Point p_cry_int_in_c;
    if (findRadiusPoseIntersection(p_cry_in_c, r, p_cry_int_in_c)) {
        // find intersection angle
        double theta_int = std::atan2(p_cry_int_in_c.y, p_cry_int_in_c.x);
        if (theta_1 < theta_int && theta_int < theta_2) {
            RCLCPP_INFO(logger_, "int   cry   theta_int: %+.3f   x: %+.3f y: %+.3f", theta_int, p_cry_int_in_c.x, p_cry_int_in_c.y);
            RCLCPP_INFO(logger_, "GOAL NOT REACHED\n\n");
            return false;
        } else {
            RCLCPP_INFO(logger_, "int   cry");
        }
    } else {
        RCLCPP_INFO(logger_, "int   cry");
    }

    RCLCPP_INFO(logger_, "*********************  GOAL REACHED  *********************");
    RCLCPP_INFO(logger_, "\n");
    return true;
}

tf2::Transform ASBCurvatureArcGoalChecker::getRobotToGoalTransform(const Pose &goal_pose, const Pose &robot_pose) {
    // Convert both poses to tf2::Transform
    tf2::Transform tf_robot;
    tf2::fromMsg(robot_pose, tf_robot);
    tf2::Transform tf_goal;
    tf2::fromMsg(goal_pose, tf_goal);

    // Transform goal_pose into the robot_pose frame
    tf2::Transform tf_result = tf_robot.inverse() * tf_goal;
    return tf_result;
}

bool ASBCurvatureArcGoalChecker::findRadiusPoseIntersection(const Pose &p, const double &r, Point &p_int) {
    // Find the intersection points by solving the quadratic equation a * D_e^2 + b * D_e + c = 0; where D_e (extension distance) is the distance of the intersection points from p.
    // Note that the quadratic formula variable a is 1.

    double x_p = p.position.x;
    double y_p = p.position.y;
    double theta_p = tf2::getYaw(p.orientation);

    double b = 2 * (x_p * std::cos(theta_p) + y_p * std::sin(theta_p));
    double c = std::pow(x_p, 2) + std::pow(y_p, 2) - std::pow(r, 2);

    double discriminant = std::pow(b, 2) - 4 * c;
    if (discriminant < 0) {  // No intersection
        return false;
    }

    double sqrt_discriminant = std::sqrt(discriminant);
    double d_1 = (-b + sqrt_discriminant) / 2;
    double d_2 = (-b - sqrt_discriminant) / 2;

    double d_min = std::min(d_1, d_2);
    double d_max = std::max(d_1, d_2);

    if (d_max < 0) {  // both solutions are negative
        return false;
    } else {
        if(d_min > 0) {  // both solutions are positive, return the smallest
            p_int.x = x_p + d_min * std::cos(theta_p);
            p_int.y = y_p + d_min * std::sin(theta_p);
            return true;
        } else {  // one solution is negative and the other is positive, return the positive one
            p_int.x = x_p + d_max * std::cos(theta_p);
            p_int.y = y_p + d_max * std::sin(theta_p);
            return true;
        }
    }

}

Point ASBCurvatureArcGoalChecker::getExtendedLookaheadPoint(const Pose &path_pose, bool &valid_solution) const {
    // Find the pose which is at a lookahead_dist distance from the origin (robot pose) and lies on the projection of next_stop_pose
    // by solving the quadratic equation a * D_e^2 + b * D_e + c = 0; where D_e (extension distance) is the distance of the found pose from next_stop_pose.
    // Note that the quadratic formula variable a is 1.

    double x_p = path_pose.position.x;
    double y_p = path_pose.position.y;
    double theta_p = tf2::getYaw(path_pose.orientation);

    double b = 2 * (x_p * std::cos(theta_p) + y_p * std::sin(theta_p));
    double c = std::pow(x_p, 2) + std::pow(y_p, 2) - std::pow(lookahead_dist_, 2);

    double discriminant = std::pow(b, 2) - 4 * c;

    if (discriminant < 0) {
        // No real solution, this happens when the path's closest point to the robot position is higher than lookahead_dist_
        valid_solution = false;
        return Point();
    }

    double sqrt_discriminant = std::sqrt(discriminant);
    double D_e1 = (-b + sqrt_discriminant) / 2;
    double D_e2 = (-b - sqrt_discriminant) / 2;

    // Choose the maximum extension distance (the one that produces the point furthest along on the forward direction of the path defined by the goal pose)
    double D_e = std::max(D_e1, D_e2);

    Point lookahead_point;
    lookahead_point.x = x_p + D_e * std::cos(theta_p);
    lookahead_point.y = y_p + D_e * std::sin(theta_p);
    valid_solution = true;
    return lookahead_point;
}

double ASBCurvatureArcGoalChecker::getLookaheadCurvature(Point lookahead_point) const {
    return 2 * lookahead_point.y / std::pow(lookahead_dist_, 2);
}

bool ASBCurvatureArcGoalChecker::getTolerances(Pose &pose_tolerance, Twist &vel_tolerance) {
    double invalid_field = std::numeric_limits<double>::lowest();

    pose_tolerance.position.x = xy_goal_tolerance_;
    pose_tolerance.position.y = xy_goal_tolerance_;
    pose_tolerance.position.z = invalid_field;
    pose_tolerance.orientation.x = invalid_field;
    pose_tolerance.orientation.y = invalid_field;
    pose_tolerance.orientation.z = invalid_field;
    pose_tolerance.orientation.w = invalid_field;

    vel_tolerance.linear.x = invalid_field;
    vel_tolerance.linear.y = invalid_field;
    vel_tolerance.linear.z = invalid_field;
    vel_tolerance.angular.x = invalid_field;
    vel_tolerance.angular.y = invalid_field;
    vel_tolerance.angular.z = invalid_field;

    return true;
}

rcl_interfaces::msg::SetParametersResult ASBCurvatureArcGoalChecker::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    for (auto &parameter: parameters) {
        const auto &type = parameter.get_type();
        const auto &name = parameter.get_name();

        if (type == ParameterType::PARAMETER_DOUBLE) {
            if (name == plugin_name_ + ".xy_goal_tolerance") {
                xy_goal_tolerance_ = parameter.as_double();
            } else if (name == plugin_name_ + ".lookahead_dist") {
                lookahead_dist_ = parameter.as_double();
            } else if (name == plugin_name_ + ".path_constraint_x") {
                path_constraint_x_ = parameter.as_double();
            } else if (name == plugin_name_ + ".path_constraint_y") {
                path_constraint_y_ = parameter.as_double();
            }
        }
    }
    result.successful = true;
    return result;
}

}  // namespace asb_nav2_plugins

PLUGINLIB_EXPORT_CLASS(asb_nav2_plugins::ASBCurvatureArcGoalChecker, nav2_core::GoalChecker)
