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
#include <limits>
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

namespace asb_regulated_pure_pursuit_controller
{

void RegulatedPurePursuitController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  node_ = parent;
  if (!node) {
    throw nav2_core::ControllerException("Unable to lock node!");
  }

  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  tf_ = tf;
  plugin_name_ = name;
  logger_ = node->get_logger();

  // Handles storage and dynamic configuration of parameters.
  // Returns pointer to data current param settings.
  param_handler_ = std::make_unique<ParameterHandler>(
    node, plugin_name_, logger_, costmap_->getSizeInMetersX());
  params_ = param_handler_->getParams();

  // Handles global path transformations
  path_handler_ = std::make_unique<PathHandler>(
    tf2::durationFromSec(params_->transform_tolerance), tf_, costmap_ros_);

  // Checks for imminent collisions
  collision_checker_ = std::make_unique<CollisionChecker>(node, costmap_ros_, params_);

  double control_frequency = 20.0;
  goal_dist_tol_ = 0.25;  // reasonable default before first update

  node->get_parameter("controller_frequency", control_frequency);
  control_duration_ = 1.0 / control_frequency;

  global_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("~/received_global_plan", 1);
  carrot_pub_ = node->create_publisher<geometry_msgs::msg::PointStamped>("~/lookahead_point", 1);
  carrot_pose_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>("~/lookahead_pose", 1);
  goal_pose_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>("~/goal_pose", 1);
  angle_goal_pose_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>("~/angle_goal_pose", 1);
  lookahead_circle_pub_ = node->create_publisher<geometry_msgs::msg::PolygonStamped>("~/lookahead_circle", 1);
  lookahead_arc_pub_ = node->create_publisher<nav_msgs::msg::Path>("~/lookahead_arc", 1);
  lookahead_curvature_pub_ = node->create_publisher<std_msgs::msg::Float64>("lookahead_curvature", 1);
  min_curvature_pub_ = node->create_publisher<std_msgs::msg::Float64>("min_curvature", 1);
  max_curvature_pub_ = node->create_publisher<std_msgs::msg::Float64>("max_curvature", 1);
  robot_path_distance_pub_ = node->create_publisher<std_msgs::msg::Float64>("robot_global_path_distance", 1);

  RCLCPP_INFO(logger_, "ASB RPP configured.");
}

void RegulatedPurePursuitController::cleanup()
{
  RCLCPP_INFO(
    logger_,
    "Cleaning up controller: %s of type"
    " regulated_pure_pursuit_controller::RegulatedPurePursuitController",
    plugin_name_.c_str());
  global_path_pub_.reset();
  carrot_pub_.reset();
  carrot_pose_pub_.reset();
  goal_pose_pub_.reset();
  angle_goal_pose_pub_.reset();
  lookahead_circle_pub_.reset();
  lookahead_arc_pub_.reset();
  lookahead_curvature_pub_.reset();
  min_curvature_pub_.reset();
  max_curvature_pub_.reset();
  robot_path_distance_pub_.reset();
}

void RegulatedPurePursuitController::activate()
{
  RCLCPP_INFO(
    logger_,
    "Activating controller: %s of type "
    "regulated_pure_pursuit_controller::RegulatedPurePursuitController",
    plugin_name_.c_str());
  global_path_pub_->on_activate();
  carrot_pub_->on_activate();
  carrot_pose_pub_->on_activate();
  goal_pose_pub_->on_activate();
  angle_goal_pose_pub_->on_activate();
  lookahead_circle_pub_->on_activate();
  lookahead_arc_pub_->on_activate();
  lookahead_curvature_pub_->on_activate();
  min_curvature_pub_->on_activate();
  max_curvature_pub_->on_activate();
  robot_path_distance_pub_->on_activate();
}

void RegulatedPurePursuitController::deactivate()
{
  RCLCPP_INFO(
    logger_,
    "Deactivating controller: %s of type "
    "regulated_pure_pursuit_controller::RegulatedPurePursuitController",
    plugin_name_.c_str());
  global_path_pub_->on_deactivate();
  carrot_pub_->on_deactivate();
  carrot_pose_pub_->on_deactivate();
  goal_pose_pub_->on_deactivate();
  angle_goal_pose_pub_->on_deactivate();
  lookahead_circle_pub_->on_deactivate();
  lookahead_arc_pub_->on_deactivate();
  lookahead_curvature_pub_->on_deactivate();
  min_curvature_pub_->on_deactivate();
  max_curvature_pub_->on_deactivate();
  robot_path_distance_pub_->on_deactivate();
}

std::unique_ptr<geometry_msgs::msg::PointStamped> RegulatedPurePursuitController::createCarrotMsg(
  const geometry_msgs::msg::PoseStamped & carrot_pose)
{
  auto carrot_msg = std::make_unique<geometry_msgs::msg::PointStamped>();
  carrot_msg->header = carrot_pose.header;
  carrot_msg->point.x = carrot_pose.pose.position.x;
  carrot_msg->point.y = carrot_pose.pose.position.y;
  carrot_msg->point.z = 0.01;  // publish above the map to stand out
  return carrot_msg;
}

std::unique_ptr<nav_msgs::msg::Path> RegulatedPurePursuitController::createLookAheadArcMsg(
  const geometry_msgs::msg::PoseStamped & robot_pose, const double & linear_vel,
  const double & angular_vel, const double & carrot_dist)
{
  // Note: robot_pose is in odom frame

  auto arc_pts_msg = std::make_unique<nav_msgs::msg::Path>();
  arc_pts_msg->header.frame_id = costmap_ros_->getGlobalFrameID();
  arc_pts_msg->header.stamp = robot_pose.header.stamp;
  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header.frame_id = arc_pts_msg->header.frame_id;
  pose_msg.header.stamp = arc_pts_msg->header.stamp;
  pose_msg.pose.position.z = 0.005;

  double projection_time;
  if (fabs(linear_vel) < 0.01 && fabs(angular_vel) > 0.01) {
    // rotating to heading at goal or toward path
    // Equation finds the angular distance required for the largest
    // part of the robot radius to move to another costmap cell:
    // theta_min = 2.0 * sin ((res/2) / r_max)
    // via isosceles triangle r_max-r_max-resolution,
    // dividing by angular_velocity gives us a timestep.
    double max_radius = costmap_ros_->getLayeredCostmap()->getCircumscribedRadius();
    projection_time = 2.0 * sin((costmap_->getResolution() / 2) / max_radius) / fabs(angular_vel);
  } else {
    // Normal path tracking
    projection_time = costmap_->getResolution() / fabs(linear_vel);
  }

  const geometry_msgs::msg::Point & robot_xy = robot_pose.pose.position;
  geometry_msgs::msg::Pose2D curr_pose;
  curr_pose.x = robot_pose.pose.position.x;
  curr_pose.y = robot_pose.pose.position.y;
  curr_pose.theta = tf2::getYaw(robot_pose.pose.orientation);
  pose_msg.pose.position.x = curr_pose.x;
  pose_msg.pose.position.y = curr_pose.y;
  arc_pts_msg->poses.push_back(pose_msg);

  // only forward simulate arc poses within max arc duration for visualization
  int i = 1;
  while (i * projection_time < 10.0) {
    i++;

    // apply velocity at curr_pose over distance
    curr_pose.x += projection_time * (linear_vel * cos(curr_pose.theta));
    curr_pose.y += projection_time * (linear_vel * sin(curr_pose.theta));
    curr_pose.theta += projection_time * angular_vel;

    if (hypot(curr_pose.x - robot_xy.x, curr_pose.y - robot_xy.y) > carrot_dist) {
      break;
    }

    pose_msg.pose.position.x = curr_pose.x;
    pose_msg.pose.position.y = curr_pose.y;
    arc_pts_msg->poses.push_back(pose_msg);

  }

  return arc_pts_msg;
}

std::unique_ptr<geometry_msgs::msg::PolygonStamped> RegulatedPurePursuitController::createLookAheadCircleMsg(
  const double & lookahead_dist, const builtin_interfaces::msg::Time & stamp)
{
  int num_points = 100;
  auto polygon_msg = std::make_unique<geometry_msgs::msg::PolygonStamped>();
  polygon_msg->header.frame_id = costmap_ros_->getBaseFrameID();
  polygon_msg->header.stamp = stamp;
  polygon_msg->polygon.points.resize(num_points);
  for(int i = 0; i < num_points; i++)
  {
    polygon_msg->polygon.points[i].x = (float)lookahead_dist * (float)std::cos(2 * M_PI * (float)i / num_points);
    polygon_msg->polygon.points[i].y = (float)lookahead_dist * (float)std::sin(2 * M_PI * (float)i / num_points);
    polygon_msg->polygon.points[i].z = 0.01;  // publish above the map to stand out
  }
  return polygon_msg;
}

std::unique_ptr<std_msgs::msg::Float64> RegulatedPurePursuitController::createCurvatureMsg(double curvature)
{
  auto curvature_msg = std::make_unique<std_msgs::msg::Float64>();
  curvature_msg->data = curvature;
  return curvature_msg;
}

//double RegulatedPurePursuitController::getLookAheadDistance(
//  const geometry_msgs::msg::Twist & speed)
//{
//  // If using velocity-scaled look ahead distances, find and clamp the dist
//  // Else, use the static look ahead distance
//  double lookahead_dist = params_->lookahead_dist;
//  if (params_->use_velocity_scaled_lookahead_dist) {
//    lookahead_dist = fabs(speed.linear.x) * params_->lookahead_time;
//    lookahead_dist = std::clamp(
//      lookahead_dist, params_->min_lookahead_dist, params_->max_lookahead_dist);
//  }
//
//  return lookahead_dist;
//}

double calculateCurvature(geometry_msgs::msg::Point lookahead_point)
{
  // Find distance^2 to look ahead point (carrot) in robot base frame
  // This is the chord length of the circle
  const double carrot_dist2 =
    (lookahead_point.x * lookahead_point.x) +
    (lookahead_point.y * lookahead_point.y);

  // Find curvature of circle (k = 1 / R)
  if (carrot_dist2 > 0.001) {
    return 2.0 * lookahead_point.y / carrot_dist2;
  } else {
    return 0.0;
  }
}

geometry_msgs::msg::TwistStamped RegulatedPurePursuitController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & speed,
  nav2_core::GoalChecker * goal_checker)
{
  std::lock_guard<std::mutex> lock_reinit(param_handler_->getMutex());

  nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));

  // Update for the current goal checker's state
  geometry_msgs::msg::Pose pose_tolerance;
  geometry_msgs::msg::Twist vel_tolerance;
  if (!goal_checker->getTolerances(pose_tolerance, vel_tolerance)) {
    RCLCPP_WARN(logger_, "Unable to retrieve goal checker's tolerances!");
  } else {
    goal_dist_tol_ = pose_tolerance.position.x;
  }

  // Transform path to robot base frame
  auto transformed_plan = path_handler_->transformGlobalPlan(pose, params_->max_robot_pose_search_dist);
  global_path_pub_->publish(transformed_plan);

  double robot_path_distance = std::hypot(transformed_plan.poses[0].pose.position.x, transformed_plan.poses[0].pose.position.y);
  std_msgs::msg::Float64 robot_path_distance_msg = std_msgs::msg::Float64();
  robot_path_distance_msg.data = robot_path_distance;
  robot_path_distance_pub_->publish(robot_path_distance_msg);
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

  // Check for reverse driving
  if (params_->allow_reversing) {
    // Cusp check
    const double dist_to_cusp = findVelocitySignChange(transformed_plan);

    // if the lookahead distance is further than the cusp, use the cusp distance instead
    if (dist_to_cusp < lookahead_dist) {
      lookahead_dist = dist_to_cusp;
    }
  }

  auto goal_pose = transformed_plan.poses.back();

  // Get the particular point on the path at the lookahead distance
  auto carrot_pose = getLookAheadPoint(lookahead_dist, transformed_plan);
  carrot_pub_->publish(createCarrotMsg(carrot_pose));
  carrot_pose_pub_->publish(carrot_pose);
  goal_pose_pub_->publish(goal_pose);
  lookahead_circle_pub_->publish(createLookAheadCircleMsg(lookahead_dist, pose.header.stamp));

  double lookahead_curvature = calculateCurvature(carrot_pose.pose.position);

  if (params_->use_goal_angle_approach) {
    const double remaining_distance = nav2_util::geometry_utils::calculate_path_length(transformed_plan);
    double dist_to_cusp = findVelocitySignChange(transformed_plan);
    if (remaining_distance < params_->goal_angle_approach_dist && remaining_distance < dist_to_cusp) {
      double x_g = goal_pose.pose.position.x, y_g = goal_pose.pose.position.y, t = tf2::getYaw(goal_pose.pose.orientation);
      lookahead_curvature = tan(t) / (x_g + y_g * tan(t));

      auto angle_goal_pose = goal_pose;
      angle_goal_pose.pose.position.x = y_g * sin(t) + x_g * cos(t);
      angle_goal_pose.pose.position.y = x_g * cos(t) * tan(t/2) + y_g * (1 - cos(t));
      angle_goal_pose_pub_->publish(angle_goal_pose);

      double angle_goal_dist = std::hypot(angle_goal_pose.pose.position.x - x_g, angle_goal_pose.pose.position.y - y_g);
      if (angle_goal_dist > pose_tolerance.position.y) {
        RCLCPP_WARN(logger_, "angle_pose_goal > tolerance: dist = %.3f tolerance = %.3f", angle_goal_dist, pose_tolerance.position.y);
        throw nav2_core::InvalidPath("RegulatedPurePursuitController can not execute angle goal approach within tolerance");
      }
    }
  }

  double regulation_curvature = lookahead_curvature;
  if (params_->use_fixed_curvature_lookahead) {
    auto curvature_lookahead_pose = getLookAheadPoint(params_->curvature_lookahead_dist, transformed_plan);
    regulation_curvature = calculateCurvature(curvature_lookahead_pose.pose.position);
  }

  // Setting the velocity direction
  double sign = 1.0;
  if (params_->allow_reversing) {
    sign = carrot_pose.pose.position.x >= 0.0 ? 1.0 : -1.0;
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
    applyConstraints(
      regulation_curvature, speed,
      collision_checker_->costAtPose(pose.pose.position.x, pose.pose.position.y), transformed_plan,
      linear_vel, sign);

    // Apply curvature to angular velocity after constraining linear velocity
    double constrained_lookahead_curvature;
    if(params_->min_turning_radius > 0.001)
    {
      double max_curvature = 1.0/params_->min_turning_radius;
      constrained_lookahead_curvature = std::clamp(lookahead_curvature, -max_curvature, max_curvature);
      min_curvature_pub_->publish(createCurvatureMsg(-max_curvature));
      max_curvature_pub_->publish(createCurvatureMsg(max_curvature));
    }
    else
    {
      constrained_lookahead_curvature = lookahead_curvature;
    }

    // When inverting the linear velocity direction (we want to start going backward while still going forward, and
    // vice versa), send a zero linear velocity (hence zero angular velocity as well, since it's computed from the
    // linear velocity and the curvature) until the robot has stopped
    if ((speed.linear.x > 0 && linear_vel < 0) || (speed.linear.x < 0 && linear_vel > 0))
    {
      linear_vel = 0.0;
    }

    angular_vel = linear_vel * constrained_lookahead_curvature;

    lookahead_curvature_pub_->publish(createCurvatureMsg(constrained_lookahead_curvature));
  }

  const double & carrot_dist = hypot(carrot_pose.pose.position.x, carrot_pose.pose.position.y);
  lookahead_arc_pub_->publish(createLookAheadArcMsg(pose, linear_vel, angular_vel, carrot_dist));

  // Collision checking on this velocity heading
  if (params_->use_collision_detection && collision_checker_->isCollisionImminent(pose, linear_vel, angular_vel, carrot_dist))
  {
    throw nav2_core::NoValidControl("RegulatedPurePursuitController detected collision ahead!");
  }


  // populate and return message
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header = pose.header;
  cmd_vel.twist.linear.x = linear_vel;
  cmd_vel.twist.angular.z = angular_vel;
  return cmd_vel;
}

bool RegulatedPurePursuitController::shouldRotateToPath(
  const geometry_msgs::msg::PoseStamped & carrot_pose, double & angle_to_path)
{
  // Whether we should rotate robot to rough path heading
  angle_to_path = atan2(carrot_pose.pose.position.y, carrot_pose.pose.position.x);
  return params_->use_rotate_to_heading && fabs(angle_to_path) > params_->rotate_to_heading_min_angle;
}

bool RegulatedPurePursuitController::shouldRotateToGoalHeading(
  const geometry_msgs::msg::PoseStamped & carrot_pose)
{
  // Whether we should rotate robot to goal heading
  double dist_to_goal = std::hypot(carrot_pose.pose.position.x, carrot_pose.pose.position.y);
  return params_->use_rotate_to_heading && dist_to_goal < goal_dist_tol_;
}

void RegulatedPurePursuitController::rotateToHeading(
  double & linear_vel, double & angular_vel,
  const double & angle_to_path, const geometry_msgs::msg::Twist & curr_speed)
{
  // Rotate in place using max angular velocity / acceleration possible
  linear_vel = 0.0;
  const double sign = angle_to_path > 0.0 ? 1.0 : -1.0;
  angular_vel = sign * params_->rotate_to_heading_angular_vel;

  const double & dt = control_duration_;
  const double min_feasible_angular_speed = curr_speed.angular.z - params_->max_angular_accel * dt;
  const double max_feasible_angular_speed = curr_speed.angular.z + params_->max_angular_accel * dt;
  angular_vel = std::clamp(angular_vel, min_feasible_angular_speed, max_feasible_angular_speed);
}

geometry_msgs::msg::Point RegulatedPurePursuitController::circleSegmentIntersection(
  const geometry_msgs::msg::Point & p1,
  const geometry_msgs::msg::Point & p2,
  double r)
{
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

geometry_msgs::msg::PoseStamped RegulatedPurePursuitController::getLookAheadPoint(
  const double & lookahead_dist,
  const nav_msgs::msg::Path & transformed_plan)
{
  // Find the first pose which is at a distance greater than the lookahead distance
  auto goal_pose_it = std::find_if(
    transformed_plan.poses.begin(), transformed_plan.poses.end(), [&](const auto & ps) {
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
    auto point = circleSegmentIntersection(
      prev_pose_it->pose.position,
      goal_pose_it->pose.position, lookahead_dist);
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = prev_pose_it->header.frame_id;
    pose.header.stamp = goal_pose_it->header.stamp;
    pose.pose.position = point;
    pose.pose.orientation = goal_pose_it->pose.orientation;
    return pose;
  }

  return *goal_pose_it;
}

void RegulatedPurePursuitController::applyConstraints(
  const double & curvature, const geometry_msgs::msg::Twist & /*curr_speed*/,
  const double & pose_cost, const nav_msgs::msg::Path & path, double & linear_vel, double & sign)
{
  double curvature_vel = linear_vel, cost_vel = linear_vel;

  // limit the linear velocity by curvature
  if (params_->use_regulated_linear_velocity_scaling) {
    curvature_vel = heuristics::curvatureConstraint(
      linear_vel, curvature, params_->regulated_linear_scaling_min_radius);
  }

  // limit the linear velocity by proximity to obstacles
  if (params_->use_cost_regulated_linear_velocity_scaling) {
    cost_vel = heuristics::costConstraint(linear_vel, pose_cost, costmap_ros_, params_);
  }

  // Use the lowest of the 2 constraints, but above the minimum translational speed
  linear_vel = std::min(cost_vel, curvature_vel);
  linear_vel = std::max(linear_vel, params_->regulated_linear_scaling_min_speed);  // TODO only apply if some param is true?

  // Apply constraint to reduce speed on approach to the final goal pose
  linear_vel = heuristics::approachVelocityConstraint(
    linear_vel, path, params_->min_approach_linear_velocity, params_->approach_velocity_scaling_dist);

  // Limit linear velocities to be valid
  linear_vel = std::clamp(fabs(linear_vel), 0.0, params_->desired_linear_vel);
  linear_vel = sign * linear_vel;
}

void RegulatedPurePursuitController::setPlan(const nav_msgs::msg::Path & path)
{
  path_handler_->setPlan(path);
}

void RegulatedPurePursuitController::setSpeedLimit(
  const double & speed_limit,
  const bool & percentage)
{
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

double RegulatedPurePursuitController::findVelocitySignChange(
  const nav_msgs::msg::Path & transformed_plan)
{
  // Iterating through the transformed global path to determine the position of the cusp
  for (unsigned int pose_id = 1; pose_id < transformed_plan.poses.size() - 1; ++pose_id) {
    // We have two vectors for the dot product OA and AB. Determining the vectors.
    double oa_x = transformed_plan.poses[pose_id].pose.position.x - transformed_plan.poses[pose_id - 1].pose.position.x;
    double oa_y = transformed_plan.poses[pose_id].pose.position.y - transformed_plan.poses[pose_id - 1].pose.position.y;
    double ab_x = transformed_plan.poses[pose_id + 1].pose.position.x - transformed_plan.poses[pose_id].pose.position.x;
    double ab_y = transformed_plan.poses[pose_id + 1].pose.position.y - transformed_plan.poses[pose_id].pose.position.y;

    /* Checking for the existence of cusp, in the path, using the dot product
    and determine its distance from the robot. If there is no cusp in the path,
    then just determine the distance to the goal location. */
    if ( (oa_x * ab_x) + (oa_y * ab_y) < 0.0) {
      // returning the distance if there is a cusp
      // The transformed path is in the robots frame, so robot is at the origin
      return hypot(
        transformed_plan.poses[pose_id].pose.position.x,
        transformed_plan.poses[pose_id].pose.position.y);
    }
  }

  return std::numeric_limits<double>::max();
}
}  // namespace asb_regulated_pure_pursuit_controller

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(
  asb_regulated_pure_pursuit_controller::RegulatedPurePursuitController,
  nav2_core::Controller)
