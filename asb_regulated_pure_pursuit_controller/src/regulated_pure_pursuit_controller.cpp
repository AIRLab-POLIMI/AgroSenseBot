// Copyright (c) 2024 Università degli Studi di Milano, Enrico Piazza
// Copyright (c) 2020 Shrijit Singh
// Copyright (c) 2020 Samsung Research America
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

#include <algorithm>
#include <string>
#include <memory>
#include <vector>
#include <utility>
#include <cmath>

#include "asb_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp"
#include "nav2_core/controller_exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"

using std::hypot;
using std::min;
using std::max;
using std::abs;
using namespace nav2_costmap_2d;  // NOLINT

namespace asb_regulated_pure_pursuit_controller {

void RegulatedPurePursuitController::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name, std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {
    auto node = parent.lock();
    node_ = parent;
    if (!node) {
        throw nav2_core::ControllerException("Unable to lock node!");
    }
    last_call_time_ = node->now();  // Initialize with current time

    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();
    tf_ = tf;
    plugin_name_ = name;
    logger_ = node->get_logger();

    // Handles storage and dynamic configuration of parameters.
    // Returns pointer to data current param settings.
    param_handler_ = std::make_unique<ParameterHandler>(node, plugin_name_, logger_, costmap_->getSizeInMetersX());
    params_ = param_handler_->getParams();

    // Handles global path transformations
    path_handler_ = std::make_unique<PathHandler>(tf2::durationFromSec(params_->transform_tolerance), tf_, costmap_ros_);

    // Checks for imminent collisions
    collision_checker_ = std::make_unique<CollisionChecker>(node, costmap_ros_, params_);

    double control_frequency = 20.0;
    goal_dist_tol_ = 0.25;  // reasonable default before first update
    in_goal_proximity_ = false;
    forward_ = true;

    node->get_parameter("controller_frequency", control_frequency);
    control_duration_ = 1.0 / control_frequency;

    global_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("~/received_global_plan", 1);
    carrot_pose_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>("~/lookahead_pose", 1);
    angle_lookahead_pose_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>("~/angle_lookahead_pose", 1);
    goal_pose_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>("~/goal_pose", 1);
    stop_pose_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>("~/stop_pose", 1);
    lookahead_circle_pub_ = node->create_publisher<geometry_msgs::msg::PolygonStamped>("~/lookahead_circle", 1);
    cost_gradient_descent_poses_pub_ = node->create_publisher<geometry_msgs::msg::PoseArray>("~/cost_gradient_descent_poses", 1);
    constraint_intersection_poses_pub_ = node->create_publisher<geometry_msgs::msg::PointStamped>("~/constraint_intersection_poses", 1);
    constraints_pub_ = node->create_publisher<geometry_msgs::msg::PolygonStamped>("~/constraints", 1);
    lookahead_arc_pub_ = node->create_publisher<nav_msgs::msg::Path>("~/lookahead_arc", 1);
    path_lookahead_arc_pub_ = node->create_publisher<nav_msgs::msg::Path>("~/path_lookahead_arc", 1);
    angle_priority_arc_pub_ = node->create_publisher<nav_msgs::msg::Path>("~/angle_lookahead_arc", 1);
    goal_checker_arc_pub_ = node->create_publisher<nav_msgs::msg::Path>("~/goal_checker_arc", 1);
    goal_checker_intersection_arc_pub_ = node->create_publisher<nav_msgs::msg::Path>("~/goal_checker_intersection_arc", 1);
    lookahead_curvature_pub_ = node->create_publisher<std_msgs::msg::Float64>("lookahead_curvature", 1);
    min_curvature_pub_ = node->create_publisher<std_msgs::msg::Float64>("min_curvature", 1);
    max_curvature_pub_ = node->create_publisher<std_msgs::msg::Float64>("max_curvature", 1);

    RCLCPP_INFO(logger_, "ASB RPP configured.");
}

void RegulatedPurePursuitController::cleanup() {
    RCLCPP_INFO(logger_, "Cleaning up controller: %s of type"
                         " regulated_pure_pursuit_controller::RegulatedPurePursuitController", plugin_name_.c_str());
    global_path_pub_.reset();
    carrot_pose_pub_.reset();
    angle_lookahead_pose_pub_.reset();
    goal_pose_pub_.reset();
    stop_pose_pub_.reset();
    lookahead_circle_pub_.reset();
    constraint_intersection_poses_pub_.reset();
    cost_gradient_descent_poses_pub_.reset();
    constraints_pub_.reset();
    lookahead_arc_pub_.reset();
    path_lookahead_arc_pub_.reset();
    angle_priority_arc_pub_.reset();
    goal_checker_arc_pub_.reset();
    goal_checker_intersection_arc_pub_.reset();
    lookahead_curvature_pub_.reset();
    min_curvature_pub_.reset();
    max_curvature_pub_.reset();
}

void RegulatedPurePursuitController::activate() {
    RCLCPP_INFO(logger_, "Activating controller: %s of type "
                         "regulated_pure_pursuit_controller::RegulatedPurePursuitController", plugin_name_.c_str());
    global_path_pub_->on_activate();
    carrot_pose_pub_->on_activate();
    angle_lookahead_pose_pub_->on_activate();
    goal_pose_pub_->on_activate();
    stop_pose_pub_->on_activate();
    lookahead_circle_pub_->on_activate();
    constraint_intersection_poses_pub_->on_activate();
    cost_gradient_descent_poses_pub_->on_activate();
    constraints_pub_->on_activate();
    lookahead_arc_pub_->on_activate();
    path_lookahead_arc_pub_->on_activate();
    angle_priority_arc_pub_->on_activate();
    goal_checker_arc_pub_->on_activate();
    goal_checker_intersection_arc_pub_->on_activate();
    lookahead_curvature_pub_->on_activate();
    min_curvature_pub_->on_activate();
    max_curvature_pub_->on_activate();
}

void RegulatedPurePursuitController::deactivate() {
    RCLCPP_INFO(logger_, "Deactivating controller: %s of type "
                         "regulated_pure_pursuit_controller::RegulatedPurePursuitController", plugin_name_.c_str());
    global_path_pub_->on_deactivate();
    carrot_pose_pub_->on_deactivate();
    angle_lookahead_pose_pub_->on_deactivate();
    goal_pose_pub_->on_deactivate();
    stop_pose_pub_->on_deactivate();
    lookahead_circle_pub_->on_deactivate();
    constraint_intersection_poses_pub_->on_deactivate();
    cost_gradient_descent_poses_pub_->on_deactivate();
    constraints_pub_->on_deactivate();
    lookahead_arc_pub_->on_deactivate();
    path_lookahead_arc_pub_->on_deactivate();
    angle_priority_arc_pub_->on_deactivate();
    goal_checker_arc_pub_->on_deactivate();
    goal_checker_intersection_arc_pub_->on_deactivate();
    lookahead_curvature_pub_->on_deactivate();
    min_curvature_pub_->on_deactivate();
    max_curvature_pub_->on_deactivate();
}

std::unique_ptr<nav_msgs::msg::Path> RegulatedPurePursuitController::createLookAheadArcMsgFromCurvature(const geometry_msgs::msg::PoseStamped &robot_pose, const double &curvature, const double &distance, const double &sign) {

    auto arc_pts_msg = std::make_unique<nav_msgs::msg::Path>();
    arc_pts_msg->header.frame_id = "base_link";
    arc_pts_msg->header.stamp = robot_pose.header.stamp;
    geometry_msgs::msg::PoseStamped pose_msg;

    pose_msg.header.frame_id = arc_pts_msg->header.frame_id;
    pose_msg.header.stamp = arc_pts_msg->header.stamp;
    pose_msg.pose.position.z = 0.005;

    if (std::fabs(curvature) == 0 ) return arc_pts_msg;

    for (int i = 0; i <= 100; i++) {
        double d = i * distance / 100;
        pose_msg.pose.position.x = sign * std::sqrt(std::pow(d, 2) - std::pow(curvature, 2) * std::pow(d, 4) / 4);
        pose_msg.pose.position.y = curvature * std::pow(d, 2) / 2;
        arc_pts_msg->poses.push_back(pose_msg);
    }

    return arc_pts_msg;
}

std::unique_ptr<geometry_msgs::msg::PolygonStamped> RegulatedPurePursuitController::createLookAheadCircleMsg(const double &lookahead_dist, const builtin_interfaces::msg::Time &stamp) {
    int num_points = 100;
    auto polygon_msg = std::make_unique<geometry_msgs::msg::PolygonStamped>();
    polygon_msg->header.frame_id = costmap_ros_->getBaseFrameID();
    polygon_msg->header.stamp = stamp;
    polygon_msg->polygon.points.resize(num_points);
    for (int i = 0; i < num_points; i++) {
        polygon_msg->polygon.points[i].x = (float) lookahead_dist * (float) std::cos(2 * M_PI * (float) i / num_points);
        polygon_msg->polygon.points[i].y = (float) lookahead_dist * (float) std::sin(2 * M_PI * (float) i / num_points);
        polygon_msg->polygon.points[i].z = 0.01;  // publish above the map to stand out
    }
    return polygon_msg;
}

std::unique_ptr<std_msgs::msg::Float64> RegulatedPurePursuitController::createCurvatureMsg(double curvature) {
    auto curvature_msg = std::make_unique<std_msgs::msg::Float64>();
    curvature_msg->data = curvature;
    return curvature_msg;
}

double calculateCurvature(geometry_msgs::msg::Point lookahead_point) {
    // Find distance^2 to look ahead point (carrot) in robot base frame
    // This is the chord length of the circle
    const double carrot_dist2 = (lookahead_point.x * lookahead_point.x) + (lookahead_point.y * lookahead_point.y);

    // Find curvature of circle (k = 1 / R)
    if (carrot_dist2 > 0.001) {
        return 2.0 * lookahead_point.y / carrot_dist2;
    } else {
        return 0.0;
    }
}

void RegulatedPurePursuitController::setPlan(const nav_msgs::msg::Path &path) {
    RCLCPP_INFO(logger_, "setPlan");
    path_handler_->setPlan(path);

    auto node = node_.lock();
    if (!node) {
        throw nav2_core::ControllerException("Unable to lock node!");
    }
    rclcpp::Time now = node->now();
    last_call_time_ = now;
    travelled_distance_ = 0.0;
}

void RegulatedPurePursuitController::setSpeedLimit(const double &speed_limit, const bool &percentage) {
    std::lock_guard<std::mutex> lock_reinit(param_handler_->getMutex());

    if (speed_limit == nav2_costmap_2d::NO_SPEED_LIMIT) {
        // Restore default value
        params_->desired_linear_vel = params_->base_desired_linear_vel;
    } else {
        if (percentage) {
            // Speed limit is expressed in % from maximum speed of robot
            params_->desired_linear_vel = params_->base_desired_linear_vel * speed_limit / 100.0;
        } else {
            // Speed limit is expressed in absolute value
            params_->desired_linear_vel = speed_limit;
        }
    }
}

geometry_msgs::msg::TwistStamped RegulatedPurePursuitController::computeVelocityCommands(const geometry_msgs::msg::PoseStamped &pose, const geometry_msgs::msg::Twist &speed, nav2_core::GoalChecker *goal_checker) {
    std::lock_guard<std::mutex> lock_reinit(param_handler_->getMutex());

    nav2_costmap_2d::Costmap2D *costmap = costmap_ros_->getCostmap();
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));

    // Update for the current goal checker's state
    geometry_msgs::msg::Pose pose_tolerance;
    geometry_msgs::msg::Twist vel_tolerance;
    if (!goal_checker->getTolerances(pose_tolerance, vel_tolerance)) {
        RCLCPP_WARN(logger_, "Unable to retrieve goal checker's tolerances!");
    } else {
        goal_dist_tol_ = pose_tolerance.position.x;
        goal_yaw_tol_ = tf2::getYaw(pose_tolerance.orientation);
    }

    auto node = node_.lock();
    if (!node) {
        throw nav2_core::ControllerException("Unable to lock node!");
    }
    rclcpp::Time now = node->now();
    double dt = (now - last_call_time_).seconds();
    travelled_distance_ += speed.linear.x * dt;
    last_call_time_ = now;

    // Transform path to robot base frame
    auto transformed_plan = path_handler_->transformGlobalPlan(pose, params_->max_robot_pose_search_dist, !params_->use_angular_approach);
    global_path_pub_->publish(transformed_plan);

    auto goal_pose = transformed_plan.poses.back();
    goal_pose_pub_->publish(goal_pose);

    geometry_msgs::msg::Pose goal_pose_fixed_frame = path_handler_->getGoalInFixedFrame();
    goal_checker_isGoalReached(pose, pose.pose, goal_pose_fixed_frame, Twist());

    if(!params_->use_angular_approach && transformed_plan.poses.size() == 1) {
        double theta = tf2::getYaw(goal_pose.pose.orientation);  // goal angle in robot frame, -PI < theta < PI
        if(theta > M_PI) theta -= 2 * M_PI;
        double g_x = goal_pose.pose.position.x;

        bool goal_forward = std::fabs(theta) < M_PI/2;
        bool goal_behind = g_x < 0.0;

        if(goal_forward == goal_behind) {  // robot moves forward and goal is already behind, or robot moves backward and goal is already in front
            if(std::fabs(speed.linear.x) > 0) {
                // return zero velocity command
                geometry_msgs::msg::TwistStamped cmd_vel;
                cmd_vel.header = pose.header;
                return cmd_vel;
            } else {
                RCLCPP_INFO(logger_, "Reached end of plan, but goal is out of tolerance");
                throw nav2_core::FailedToMakeProgress("Reached end of plan");
            }
        }

    }

    double robot_path_distance = std::hypot(transformed_plan.poses[0].pose.position.x, transformed_plan.poses[0].pose.position.y);
    if (robot_path_distance > params_->max_robot_path_dist) {
        throw nav2_core::InvalidPath("RegulatedPurePursuitController robot too far from global plan!");
    }

    double lookahead_dist = params_->lookahead_dist;

    // Compute lookahead distance based on velocity
    if (params_->use_velocity_scaled_lookahead_dist) {
        lookahead_dist = fabs(speed.linear.x) * params_->lookahead_time;
        lookahead_dist = std::clamp(lookahead_dist, params_->min_lookahead_dist, params_->max_lookahead_dist);
    }

    // Compute lookahead distance based on path distance, which only increases the lookahead distance when the
    // lookahead margin is not respected
    if (params_->use_adaptive_lookahead_dist && (lookahead_dist < robot_path_distance + params_->adaptive_lookahead_path_distance_margin)) {
        lookahead_dist = robot_path_distance + params_->adaptive_lookahead_path_distance_margin;
    }
    lookahead_circle_pub_->publish(createLookAheadCircleMsg(lookahead_dist, pose.header.stamp));

    auto next_stop_pose = findStopPose(transformed_plan);
    stop_pose_pub_->publish(next_stop_pose);

    double next_stop_dist = std::hypot(next_stop_pose.pose.position.x, next_stop_pose.pose.position.y);

    // Get the particular point on the path at the lookahead distance.
    // If the lookahead distance is further than the next stop pose (cusp or goal), then over-extend the lookahead pose.
    geometry_msgs::msg::PoseStamped carrot_pose;
    if (next_stop_dist < lookahead_dist) {
        carrot_pose = getExtendedLookaheadPose(next_stop_pose, lookahead_dist);
    } else {
        carrot_pose = getLookAheadPoint(lookahead_dist, transformed_plan);
    }
    carrot_pose_pub_->publish(carrot_pose);

    // Setting the velocity direction
    double sign = 1.0;
    if (params_->allow_reversing) {
        sign = carrot_pose.pose.position.x >= 0.0 ? 1.0 : -1.0;
    }

    if(params_->use_cost_gradient_descent) {

        std::string global_frame_id = costmap_ros_->getGlobalFrameID();

        geometry_msgs::msg::PoseArray cost_gradient_descent_poses;
        cost_gradient_descent_poses.header = carrot_pose.header;
//        cost_gradient_descent_poses.header.stamp = carrot_pose.header.stamp;
//        cost_gradient_descent_poses.header.frame_id = global_frame_id;
        double res = costmap->getResolution();

        // find the pose shifting the lookahead pose that minimizes the cost within a window, among the poses that minimize the cost, pick the one closest to the lookahead pose
        int min_d = std::numeric_limits<int>::max();
//        uint8_t min_c = std::numeric_limits<uint8_t>::max();
        geometry_msgs::msg::PoseStamped min_p = carrot_pose;

        RCLCPP_INFO(logger_, "costmap res: %f, costmap frame_id: %s", res, costmap_ros_->getGlobalFrameID().c_str());
        for (int i=-10; i<11; i++){
            geometry_msgs::msg::PoseStamped shifted_carrot_pose = carrot_pose;
            shifted_carrot_pose.pose.position.y += i*res;

//            geometry_msgs::msg::PoseStamped p;
//            if (!path_handler_->transformPose(global_frame_id, shifted_carrot_pose, p)) {
//                throw nav2_core::ControllerTFError("Unable to transform robot pose into global plan's frame");
//            }
            cost_gradient_descent_poses.poses.emplace_back(shifted_carrot_pose.pose);

            double shifted_carrot_pose_dist = std::hypot(shifted_carrot_pose.pose.position.x, shifted_carrot_pose.pose.position.y);
            double lc = calculateCurvature(shifted_carrot_pose.pose.position);
            double angular_vel = params_->desired_linear_vel * lc;
            bool collision = collision_checker_->isCollisionImminent(pose, params_->desired_linear_vel, angular_vel, shifted_carrot_pose_dist, false);
//            unsigned int mx, my;
//            costmap->worldToMap(p.pose.position.x, p.pose.position.y, mx, my);
//            uint8_t c = costmap->getCost(mx, my);
            int d = std::abs(i);

            if (collision){
                RCLCPP_INFO(logger_, "i: %+i  COLLISION ****************", i);
            } else {
                RCLCPP_INFO(logger_, "i: %+i  no collision", i);
            }

            if(!collision && d < min_d) {
                min_d = d;
                min_p = shifted_carrot_pose;
            }

        }
        cost_gradient_descent_poses_pub_->publish(cost_gradient_descent_poses);
        RCLCPP_INFO(logger_, "min_d: %i, min_p x: %+0.2f, min_p y: %+0.2f", min_d, min_p.pose.position.x, min_p.pose.position.y);
        carrot_pose = min_p;
    }

    double lookahead_curvature = calculateCurvature(carrot_pose.pose.position);
    if (params_->use_angular_approach) {
        sign = -1.0;
        lookahead_curvature = -lookahead_curvature;
    }

    double linear_vel, angular_vel;
    linear_vel = params_->desired_linear_vel;

    // Make sure we're in compliance with basic constraints
    double angle_to_heading;
    if (shouldRotateToGoalHeading(carrot_pose)) {
        double angle_to_goal = tf2::getYaw(transformed_plan.poses.back().pose.orientation);
        rotateToHeading(linear_vel, angular_vel, angle_to_goal, speed);
    } else if (shouldRotateToPath(carrot_pose, angle_to_heading)) {
        rotateToHeading(linear_vel, angular_vel, angle_to_heading, speed);
    } else {
        const double pose_cost = collision_checker_->costAtPose(pose.pose.position.x, pose.pose.position.y);
        applyLinearVelocityConstraints(lookahead_curvature, pose_cost, next_stop_dist, sign, linear_vel);

        // Apply curvature to angular velocity after constraining linear velocity
        double constrained_lookahead_curvature;
        if (params_->min_turning_radius > 0.001) {
            double max_curvature = 1.0 / params_->min_turning_radius;
            constrained_lookahead_curvature = std::clamp(lookahead_curvature, -max_curvature, max_curvature);
            min_curvature_pub_->publish(createCurvatureMsg(-max_curvature));
            max_curvature_pub_->publish(createCurvatureMsg(max_curvature));
        } else {
            constrained_lookahead_curvature = lookahead_curvature;
        }

        // When inverting the linear velocity direction (we want to start going backward while still going forward, and
        // vice versa), send a zero linear velocity (hence zero angular velocity as well, since it's computed from the
        // linear velocity and the curvature) until the robot has stopped
        if ((speed.linear.x > 0 && linear_vel < 0) || (speed.linear.x < 0 && linear_vel > 0)) {
            linear_vel = 0.0;
        }

        angular_vel = linear_vel * constrained_lookahead_curvature;

        lookahead_curvature_pub_->publish(createCurvatureMsg(constrained_lookahead_curvature));
    }

    const double &carrot_dist = hypot(carrot_pose.pose.position.x, carrot_pose.pose.position.y);
    lookahead_arc_pub_->publish(createLookAheadArcMsgFromCurvature(pose, lookahead_curvature, carrot_dist, sign));

    if (params_->use_angular_approach) {
        angle_priority_arc_pub_->publish(createLookAheadArcMsgFromCurvature(pose, -lookahead_curvature, carrot_dist, -sign));
    }

    // Collision checking on this velocity heading
    if (params_->use_collision_detection && collision_checker_->isCollisionImminent(pose, linear_vel, angular_vel, carrot_dist, true)) {
        throw nav2_core::NoValidControl("RegulatedPurePursuitController detected collision ahead!");
    }

    // populate and return message
    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header = pose.header;
    cmd_vel.twist.linear.x = linear_vel;
    cmd_vel.twist.angular.z = angular_vel;
    return cmd_vel;
}

bool RegulatedPurePursuitController::shouldRotateToPath(const geometry_msgs::msg::PoseStamped &carrot_pose, double &angle_to_path) {
    // Whether we should rotate robot to rough path heading
    angle_to_path = atan2(carrot_pose.pose.position.y, carrot_pose.pose.position.x);
    return params_->use_rotate_to_heading && fabs(angle_to_path) > params_->rotate_to_heading_min_angle;
}

bool RegulatedPurePursuitController::shouldRotateToGoalHeading(const geometry_msgs::msg::PoseStamped &carrot_pose) {
    // Whether we should rotate robot to goal heading
    double dist_to_goal = std::hypot(carrot_pose.pose.position.x, carrot_pose.pose.position.y);
    return params_->use_rotate_to_heading && dist_to_goal < goal_dist_tol_;
}

void RegulatedPurePursuitController::rotateToHeading(double &linear_vel, double &angular_vel, const double &angle_to_path, const geometry_msgs::msg::Twist &curr_speed) {
    // Rotate in place using max angular velocity / acceleration possible
    linear_vel = 0.0;
    const double sign = angle_to_path > 0.0 ? 1.0 : -1.0;
    angular_vel = sign * params_->rotate_to_heading_angular_vel;

    const double &dt = control_duration_;
    const double min_feasible_angular_speed = curr_speed.angular.z - params_->max_angular_accel * dt;
    const double max_feasible_angular_speed = curr_speed.angular.z + params_->max_angular_accel * dt;
    angular_vel = std::clamp(angular_vel, min_feasible_angular_speed, max_feasible_angular_speed);
}

geometry_msgs::msg::Point RegulatedPurePursuitController::circleSegmentIntersection(const geometry_msgs::msg::Point &p1, const geometry_msgs::msg::Point &p2, double r) {
    // Formula for intersection of a line with a circle centered at the origin,
    // modified to always return the point that is on the segment between the two points.
    // https://mathworld.wolfram.com/Circle-LineIntersection.html
    // This works because the poses are transformed into the robot frame.
    // This can be derived from solving the system of equations of a line and a circle
    // which results in something that is just a reformulation of the quadratic formula.
    // Interactive illustration in doc/circle-segment-intersection.ipynb as well as at
    // https://www.desmos.com/calculator/td5cwbuocd
    double x1 = p1.x;
    double x2 = p2.x;
    double y1 = p1.y;
    double y2 = p2.y;

    double dx = x2 - x1;
    double dy = y2 - y1;
    double dr2 = dx * dx + dy * dy;
    double D = x1 * y2 - x2 * y1;

    // Augmentation to only return point within segment
    double d1 = x1 * x1 + y1 * y1;
    double d2 = x2 * x2 + y2 * y2;
    double dd = d2 - d1;

    geometry_msgs::msg::Point p;
    double sqrt_term = std::sqrt(r * r * dr2 - D * D);
    p.x = (D * dy + std::copysign(1.0, dd) * dx * sqrt_term) / dr2;
    p.y = (-D * dx + std::copysign(1.0, dd) * dy * sqrt_term) / dr2;
    return p;
}

geometry_msgs::msg::PoseStamped RegulatedPurePursuitController::getLookAheadPoint(const double &lookahead_dist, const nav_msgs::msg::Path &transformed_plan) {
    // Find the first pose which is at a distance greater than the lookahead distance
    auto goal_pose_it = std::find_if(transformed_plan.poses.begin(), transformed_plan.poses.end(), [&](const auto &ps) {
        return hypot(ps.pose.position.x, ps.pose.position.y) >= lookahead_dist;
    });

    // If all poses are inside the lookahead circle, take the last pose (the plan goal)
    if (goal_pose_it == transformed_plan.poses.end()) {
        goal_pose_it = std::prev(transformed_plan.poses.end());
    } else if (params_->use_interpolation && goal_pose_it != transformed_plan.poses.begin()) {
        // Find the point on the line segment between the two poses
        // that is exactly the lookahead distance away from the robot pose (the origin)
        // This can be found with a closed form for the intersection of a segment and a circle
        // Because of the way we did the std::find_if, prev_pose is guaranteed to be inside the circle,
        // and goal_pose is guaranteed to be outside the circle.
        auto prev_pose_it = std::prev(goal_pose_it);
        auto point = circleSegmentIntersection(prev_pose_it->pose.position, goal_pose_it->pose.position, lookahead_dist);
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = prev_pose_it->header.frame_id;
        pose.header.stamp = goal_pose_it->header.stamp;
        pose.pose.position = point;
        pose.pose.orientation = goal_pose_it->pose.orientation;
        return pose;
    }

    return *goal_pose_it;
}

geometry_msgs::msg::PoseStamped RegulatedPurePursuitController::getExtendedLookaheadPose(const geometry_msgs::msg::PoseStamped &next_stop_pose, const double lookahead_dist) {
    // Find the pose which is at a lookahead_dist distance from the origin (robot pose) and lies on the projection of next_stop_pose
    // by solving the quadratic equation a * D_e^2 + b * D_e + c = 0; where D_e (extension distance) is the distance of the found pose from next_stop_pose.
    // Note that the quadratic formula variable a is 1.

    double x_p = next_stop_pose.pose.position.x;
    double y_p = next_stop_pose.pose.position.y;
    double theta_p = tf2::getYaw(next_stop_pose.pose.orientation);

    double b = 2 * (x_p * std::cos(theta_p) + y_p * std::sin(theta_p));
    double c = std::pow(x_p, 2) + std::pow(y_p, 2) - std::pow(lookahead_dist, 2);

    double discriminant = std::pow(b, 2) - 4 * c;

    if (discriminant < 0) {
        // No real solution
        throw nav2_core::ControllerException("Negative discriminant in getExtendedLookaheadPose!");
    }

    double sqrt_discriminant = std::sqrt(discriminant);
    double D_e1 = (-b + sqrt_discriminant) / 2;
    double D_e2 = (-b - sqrt_discriminant) / 2;

    // Choose the extension distance with the same sign as x_p
    double D_e;
    if (x_p >= 0) {
        D_e = (D_e1 >= 0) ? D_e1 : D_e2;
    } else {
        D_e = (D_e1 <= 0) ? D_e1 : D_e2;
    }

    geometry_msgs::msg::PoseStamped lookahead_pose;
    lookahead_pose.header = next_stop_pose.header;
    lookahead_pose.pose.position.x = x_p + D_e * std::cos(theta_p);
    lookahead_pose.pose.position.y = y_p + D_e * std::sin(theta_p);
    lookahead_pose.pose.orientation = next_stop_pose.pose.orientation;
    return lookahead_pose;

}

void RegulatedPurePursuitController::applyLinearVelocityConstraints(const double &curvature, const double &pose_cost, const double &stop_dist, const double &sign, double &linear_vel) {
    double curvature_vel = linear_vel, cost_vel = linear_vel;

    // limit the linear velocity by curvature
    if (params_->use_regulated_linear_velocity_scaling) {
        curvature_vel = heuristics::curvatureConstraint(linear_vel, curvature, params_->regulated_linear_scaling_min_radius);
    }

    // limit the linear velocity by proximity to obstacles
    if (params_->use_cost_regulated_linear_velocity_scaling) {
        cost_vel = heuristics::costConstraint(linear_vel, pose_cost, costmap_ros_, params_);
    }

    // Use the lowest of the 2 constraints, but above the minimum translational speed
    linear_vel = std::min(cost_vel, curvature_vel);
    linear_vel = std::max(linear_vel, params_->regulated_linear_scaling_min_speed);  // TODO only apply if some param is true?

    // Apply constraint to reduce speed on departure
    double departure_scaling_factor = std::clamp(travelled_distance_ / params_->departure_velocity_scaling_dist, 0.0, 1.0);
    double departure_vel = std::max(linear_vel * departure_scaling_factor, params_->min_departure_linear_velocity);

    // Apply constraint to reduce speed on approach to the final goal pose and to the next cusp
    double approach_scaling_factor = std::clamp(stop_dist / params_->approach_velocity_scaling_dist, 0.0, 1.0);
    double approach_vel = std::max(linear_vel * approach_scaling_factor, params_->min_approach_linear_velocity);

    // Use the lowest between departure and approach velocity constraints
    linear_vel = std::min(linear_vel, departure_vel);
    linear_vel = std::min(linear_vel, approach_vel);

    // Limit linear velocities to be valid
    linear_vel = std::clamp(fabs(linear_vel), 0.0, params_->desired_linear_vel);
    linear_vel = sign * linear_vel;
}

geometry_msgs::msg::PoseStamped RegulatedPurePursuitController::findStopPose(const nav_msgs::msg::Path &transformed_plan) {
    // Iterating through the transformed global path to determine the position of the cusp
    for (unsigned int i = 1; i < transformed_plan.poses.size() - 1; ++i) {
        // We have two vectors for the dot product OA and AB. Determining the vectors.
        double oa_x = transformed_plan.poses[i].pose.position.x - transformed_plan.poses[i - 1].pose.position.x;
        double oa_y = transformed_plan.poses[i].pose.position.y - transformed_plan.poses[i - 1].pose.position.y;
        double ab_x = transformed_plan.poses[i + 1].pose.position.x - transformed_plan.poses[i].pose.position.x;
        double ab_y = transformed_plan.poses[i + 1].pose.position.y - transformed_plan.poses[i].pose.position.y;

        // Checking for the existence of cusp in the path, using the dot product.
        if ((oa_x * ab_x) + (oa_y * ab_y) < 0.0) {
            // returning the distance if there is a cusp
            // The transformed path is in the robots frame, so robot is at the origin
            return transformed_plan.poses[i];
        }
    }

    // If there is no cusp in the path return the last pose (goal pose)
    return transformed_plan.poses.back();
}

bool RegulatedPurePursuitController::goal_checker_isGoalReached(const geometry_msgs::msg::PoseStamped &robot_pose, const Pose &query_pose, const Pose &goal_pose, const Twist &) {
    double path_constraint_x_ = 1.0, path_constraint_y_ = 0.05;

    // compute the goal pose in the robot frame (the frame of reference defined by query_pose)
    tf2::Transform tf_r_to_g = goal_checker_getRobotToGoalTransform(goal_pose, query_pose);
    Pose g_in_r;
    toMsg(tf_r_to_g, g_in_r);
//    RCLCPP_INFO(logger_, "query in fixed frame  x: %+.3f  y: %+.3f  theta: %+.3f", query_pose.position.x, query_pose.position.y, tf2::getYaw(query_pose.orientation));
//    RCLCPP_INFO(logger_, "goal in fixed frame   x: %+.3f  y: %+.3f  theta: %+.3f", goal_pose.position.x, goal_pose.position.y, tf2::getYaw(goal_pose.orientation));
//    RCLCPP_INFO(logger_, "goal in robot frame   x: %+.3f  y: %+.3f  theta: %+.3f", g_in_r.position.x, g_in_r.position.y, tf2::getYaw(g_in_r.orientation));

    if (std::hypot(g_in_r.position.x, g_in_r.position.y) > goal_dist_tol_) {
//        nav_msgs::msg::Path empty_path;
//        empty_path.header.stamp = robot_pose.header.stamp;
//        empty_path.header.frame_id = "base_footprint";
//        angle_priority_arc_pub_->publish(empty_path);

//        geometry_msgs::msg::PolygonStamped polygon_msg;
//        polygon_msg.header.stamp = robot_pose.header.stamp;
//        polygon_msg.header.frame_id = "base_footprint";
//        constraints_pub_->publish(polygon_msg);

//        geometry_msgs::msg::PoseArray constraint_intersections_msg;
//        constraint_intersections_msg.header.stamp = robot_pose.header.stamp;
//        constraint_intersections_msg.header.frame_id = "base_footprint";
//        constraint_intersection_poses_pub_->publish(constraint_intersections_msg);

//        RCLCPP_INFO(logger_, "GOAL NOT REACHED\n\n");
        return false;
    }

    // get the lookahead point for the lookahead distance and the straight path defined by the goal (same as carrot point in RPP)
    bool valid_solution;
    Point lookahead_point = goal_checker_getExtendedLookaheadPoint(g_in_r, valid_solution);
    if (!valid_solution) {
//        nav_msgs::msg::Path empty_path;
//        empty_path.header.stamp = robot_pose.header.stamp;
//        empty_path.header.frame_id = "base_footprint";
//        angle_priority_arc_pub_->publish(empty_path);

//        geometry_msgs::msg::PolygonStamped polygon_msg;
//        polygon_msg.header.stamp = robot_pose.header.stamp;
//        polygon_msg.header.frame_id = "base_footprint";
//        constraints_pub_->publish(polygon_msg);

//        geometry_msgs::msg::PoseArray constraint_intersections_msg;
//        constraint_intersections_msg.header.stamp = robot_pose.header.stamp;
//        constraint_intersections_msg.header.frame_id = "base_footprint";
//        constraint_intersection_poses_pub_->publish(constraint_intersections_msg);

//        RCLCPP_INFO(logger_, "GOAL NOT REACHED\n\n");
        return false;
    }

    double c = goal_checker_getLookaheadCurvature(lookahead_point);

//    RCLCPP_INFO(logger_, "curvature   c: %+.3f  ", c);

    tf2::Transform tf_g_to_clx(tf2::Quaternion(tf2::Vector3(0, 0, 1), 0), tf2::Vector3(path_constraint_x_, path_constraint_y_, 0));
    tf2::Transform tf_g_to_cly(tf2::Quaternion(tf2::Vector3(0, 0, 1), M_PI / 2), tf2::Vector3(path_constraint_x_, path_constraint_y_, 0));
    tf2::Transform tf_g_to_crx(tf2::Quaternion(tf2::Vector3(0, 0, 1), 0), tf2::Vector3(path_constraint_x_, -path_constraint_y_, 0));
    tf2::Transform tf_g_to_cry(tf2::Quaternion(tf2::Vector3(0, 0, 1), -M_PI / 2), tf2::Vector3(path_constraint_x_, -path_constraint_y_, 0));

    tf2::Transform tf_g_to_clw(tf2::Quaternion(tf2::Vector3(0, 0, 1), 0), tf2::Vector3(path_constraint_x_, path_constraint_y_ * 10, 0));
    tf2::Transform tf_g_to_crw(tf2::Quaternion(tf2::Vector3(0, 0, 1), -M_PI / 2), tf2::Vector3(path_constraint_x_, -path_constraint_y_ * 10, 0));
    tf2::Transform tf_g_to_clf(tf2::Quaternion(tf2::Vector3(0, 0, 1), 0), tf2::Vector3(path_constraint_x_ * 4, path_constraint_y_, 0));
    tf2::Transform tf_g_to_crf(tf2::Quaternion(tf2::Vector3(0, 0, 1), -M_PI / 2), tf2::Vector3(path_constraint_x_ * 4, -path_constraint_y_, 0));

    Pose p_clx_in_r, p_cly_in_r, p_crx_in_r, p_cry_in_r, p_clw_in_r, p_crw_in_r, p_clf_in_r, p_crf_in_r;  // constraint poses in robot frame
    tf2::Transform tf_r_to_clx = tf_r_to_g * tf_g_to_clx;
    tf2::Transform tf_r_to_cly = tf_r_to_g * tf_g_to_cly;
    tf2::Transform tf_r_to_crx = tf_r_to_g * tf_g_to_crx;
    tf2::Transform tf_r_to_cry = tf_r_to_g * tf_g_to_cry;

    tf2::Transform tf_r_to_clw = tf_r_to_g * tf_g_to_clw;
    tf2::Transform tf_r_to_crw = tf_r_to_g * tf_g_to_crw;
    tf2::Transform tf_r_to_clf = tf_r_to_g * tf_g_to_clf;
    tf2::Transform tf_r_to_crf = tf_r_to_g * tf_g_to_crf;

    toMsg(tf_r_to_clx, p_clx_in_r);
    toMsg(tf_r_to_cly, p_cly_in_r);
    toMsg(tf_r_to_crx, p_crx_in_r);
    toMsg(tf_r_to_cry, p_cry_in_r);

    toMsg(tf_r_to_clw, p_clw_in_r);
    toMsg(tf_r_to_crw, p_crw_in_r);
    toMsg(tf_r_to_clf, p_clf_in_r);
    toMsg(tf_r_to_crf, p_crf_in_r);

    geometry_msgs::msg::PolygonStamped polygon_msg;
    polygon_msg.header.stamp = robot_pose.header.stamp;
    polygon_msg.header.frame_id = "base_footprint";
    polygon_msg.polygon.points.resize(6);
    polygon_msg.polygon.points[0].x = (float) p_clw_in_r.position.x;
    polygon_msg.polygon.points[0].y = (float) p_clw_in_r.position.y;

    polygon_msg.polygon.points[1].x = (float) p_cly_in_r.position.x;
    polygon_msg.polygon.points[1].y = (float) p_cly_in_r.position.y;

    polygon_msg.polygon.points[2].x = (float) p_clf_in_r.position.x;
    polygon_msg.polygon.points[2].y = (float) p_clf_in_r.position.y;

    polygon_msg.polygon.points[3].x = (float) p_crf_in_r.position.x;
    polygon_msg.polygon.points[3].y = (float) p_crf_in_r.position.y;

    polygon_msg.polygon.points[4].x = (float) p_cry_in_r.position.x;
    polygon_msg.polygon.points[4].y = (float) p_cry_in_r.position.y;

    polygon_msg.polygon.points[5].x = (float) p_crw_in_r.position.x;
    polygon_msg.polygon.points[5].y = (float) p_crw_in_r.position.y;

    polygon_msg.polygon.points.insert(polygon_msg.polygon.points.end(), polygon_msg.polygon.points.rbegin(), polygon_msg.polygon.points.rend());
    constraints_pub_->publish(polygon_msg);

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
//        RCLCPP_INFO(logger_, "GOAL NOT REACHED\n\n");
        return false;  // in the extremely rare case in which the curvature is exactly 0, return
    }

    double r = std::fabs(1 / c);  // curvature radius

    Pose p_clx_in_c, p_cly_in_c, p_crx_in_c, p_cry_in_c;  // constraint poses in center of rotation frame
    tf2::Transform tf_c_to_clx = tf_c_to_r * tf_r_to_g * tf_g_to_clx;
    tf2::Transform tf_c_to_cly = tf_c_to_r * tf_r_to_g * tf_g_to_cly;
    tf2::Transform tf_c_to_crx = tf_c_to_r * tf_r_to_g * tf_g_to_crx;
    tf2::Transform tf_c_to_cry = tf_c_to_r * tf_r_to_g * tf_g_to_cry;
    toMsg(tf_c_to_clx, p_clx_in_c);
    toMsg(tf_c_to_cly, p_cly_in_c);
    toMsg(tf_c_to_crx, p_crx_in_c);
    toMsg(tf_c_to_cry, p_cry_in_c);

    PointStamped constraint_intersections_msg;
    constraint_intersections_msg.header.stamp = robot_pose.header.stamp;
    constraint_intersections_msg.header.frame_id = "base_footprint";
//    constraint_intersections_msg.poses.push_back(goal_checker_get_pose_c_to_r(p_l_in_c.position, tf_r_to_c));

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
//    RCLCPP_INFO(logger_, "int angles   theta_1: %+.3f    theta_2: %+.3f\n", theta_1, theta_2);

    // find the intersection points of the curvature arc with the constraint segments (in the center of rotation frame)
    Point p_clx_int_in_c;
    if (goal_checker_findRadiusPoseIntersection(p_clx_in_c, r, p_clx_int_in_c)) {
        // find intersection angle
        double theta_int = std::atan2(p_clx_int_in_c.y, p_clx_int_in_c.x);
        if (theta_1 < theta_int && theta_int < theta_2) {
//            RCLCPP_INFO(logger_, "int   clx   theta_int: %+.3f   x: %+.3f y: %+.3f", theta_int, p_clx_int_in_c.x, p_clx_int_in_c.y);
            constraint_intersections_msg.point = goal_checker_get_pose_c_to_r(p_clx_int_in_c, tf_r_to_c).position;
            constraint_intersection_poses_pub_->publish(constraint_intersections_msg);
            goal_checker_intersection_arc_pub_->publish(createLookAheadArcMsgFromCurvature(robot_pose, c, std::hypot(lookahead_point.x, lookahead_point.y), 1));
//            RCLCPP_INFO(logger_, "GOAL NOT REACHED\n\n");
            return false;
        } else {
//            RCLCPP_INFO(logger_, "int   clx");
        }
    } else {
//        RCLCPP_INFO(logger_, "int   clx");
    }

    Point p_cly_int_in_c;
    if (goal_checker_findRadiusPoseIntersection(p_cly_in_c, r, p_cly_int_in_c)) {
        // find intersection angle
        double theta_int = std::atan2(p_cly_int_in_c.y, p_cly_int_in_c.x);
        if (theta_1 < theta_int && theta_int < theta_2) {
//            RCLCPP_INFO(logger_, "int   cly   theta_int: %+.3f   x: %+.3f y: %+.3f", theta_int, p_cly_int_in_c.x, p_cly_int_in_c.y);
            constraint_intersections_msg.point = goal_checker_get_pose_c_to_r(p_cly_int_in_c, tf_r_to_c).position;
            constraint_intersection_poses_pub_->publish(constraint_intersections_msg);
            goal_checker_intersection_arc_pub_->publish(createLookAheadArcMsgFromCurvature(robot_pose, c, std::hypot(lookahead_point.x, lookahead_point.y), 1));
//            RCLCPP_INFO(logger_, "GOAL NOT REACHED\n\n");
            return false;
        } else {
//            RCLCPP_INFO(logger_, "int   cly");
        }
    } else {
//        RCLCPP_INFO(logger_, "int   cly");
    }

    Point p_crx_int_in_c;
    if (goal_checker_findRadiusPoseIntersection(p_crx_in_c, r, p_crx_int_in_c)) {
        // find intersection angle
        double theta_int = std::atan2(p_crx_int_in_c.y, p_crx_int_in_c.x);
        if (theta_1 < theta_int && theta_int < theta_2) {
//            RCLCPP_INFO(logger_, "int   crx   theta_int: %+.3f   x: %+.3f y: %+.3f", theta_int, p_crx_int_in_c.x, p_crx_int_in_c.y);
            constraint_intersections_msg.point = goal_checker_get_pose_c_to_r(p_crx_int_in_c, tf_r_to_c).position;
            constraint_intersection_poses_pub_->publish(constraint_intersections_msg);
            goal_checker_intersection_arc_pub_->publish(createLookAheadArcMsgFromCurvature(robot_pose, c, std::hypot(lookahead_point.x, lookahead_point.y), 1));
//            RCLCPP_INFO(logger_, "GOAL NOT REACHED\n\n");
            return false;
        } else {
//            RCLCPP_INFO(logger_, "int   crx");
        }
    } else {
//        RCLCPP_INFO(logger_, "int   crx");
    }

    Point p_cry_int_in_c;
    if (goal_checker_findRadiusPoseIntersection(p_cry_in_c, r, p_cry_int_in_c)) {
        // find intersection angle
        double theta_int = std::atan2(p_cry_int_in_c.y, p_cry_int_in_c.x);
        if (theta_1 < theta_int && theta_int < theta_2) {
//            RCLCPP_INFO(logger_, "int   cry   theta_int: %+.3f   x: %+.3f y: %+.3f", theta_int, p_cry_int_in_c.x, p_cry_int_in_c.y);
            constraint_intersections_msg.point = goal_checker_get_pose_c_to_r(p_cry_int_in_c, tf_r_to_c).position;
            constraint_intersection_poses_pub_->publish(constraint_intersections_msg);
            goal_checker_intersection_arc_pub_->publish(createLookAheadArcMsgFromCurvature(robot_pose, c, std::hypot(lookahead_point.x, lookahead_point.y), 1));
//            RCLCPP_INFO(logger_, "GOAL NOT REACHED\n\n");
            return false;
        } else {
//            RCLCPP_INFO(logger_, "int   cry");
        }
    } else {
//        RCLCPP_INFO(logger_, "int   cry");
    }

//    constraint_intersection_poses_pub_->publish(constraint_intersections_msg);

//    RCLCPP_INFO(logger_, "*********************  GOAL REACHED  *********************");
//    RCLCPP_INFO(logger_, "\n");
    goal_checker_arc_pub_->publish(createLookAheadArcMsgFromCurvature(robot_pose, c, std::hypot(lookahead_point.x, lookahead_point.y), 1));
    return true;
}

Pose RegulatedPurePursuitController::goal_checker_get_pose_c_to_r(const Point &point_in_c, const tf2::Transform &tf_r_to_c) {

    Pose p_in_c;
    p_in_c.position = point_in_c;
    p_in_c.orientation.w = 1;
    tf2::Transform tf_c_to_p;
    tf2::fromMsg(p_in_c, tf_c_to_p);
    tf2::Transform tf_r_to_p = tf_r_to_c * tf_c_to_p;

    Pose p_in_r;
    toMsg(tf_r_to_p, p_in_r);
    return p_in_r;
}

tf2::Transform RegulatedPurePursuitController::goal_checker_getRobotToGoalTransform(const Pose &goal_pose, const Pose &robot_pose) {
    // Convert both poses to tf2::Transform
    tf2::Transform tf_robot;
    tf2::fromMsg(robot_pose, tf_robot);
    tf2::Transform tf_goal;
    tf2::fromMsg(goal_pose, tf_goal);

    // Transform goal_pose into the robot_pose frame
    tf2::Transform tf_result = tf_robot.inverse() * tf_goal;
    return tf_result;
}

bool RegulatedPurePursuitController::goal_checker_findRadiusPoseIntersection(const Pose &p, const double &r, Point &p_int) {
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

Point RegulatedPurePursuitController::goal_checker_getExtendedLookaheadPoint(const Pose &path_pose, bool &valid_solution) const {
    // Find the pose which is at a lookahead_dist distance from the origin (robot pose) and lies on the projection of next_stop_pose
    // by solving the quadratic equation a * D_e^2 + b * D_e + c = 0; where D_e (extension distance) is the distance of the found pose from next_stop_pose.
    // Note that the quadratic formula variable a is 1.

    double lookahead_dist_ = 2.8;

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

double RegulatedPurePursuitController::goal_checker_getLookaheadCurvature(Point lookahead_point) const {
    double lookahead_dist_ = 2.8;
    return 2 * lookahead_point.y / std::pow(lookahead_dist_, 2);
}




}  // namespace asb_regulated_pure_pursuit_controller

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(asb_regulated_pure_pursuit_controller::RegulatedPurePursuitController, nav2_core::Controller)
