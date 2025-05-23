// Copyright (c) 2022 Samsung Research America
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

#ifndef ASB_REGULATED_PURE_PURSUIT_CONTROLLER__REGULATION_FUNCTIONS_HPP_
#define ASB_REGULATED_PURE_PURSUIT_CONTROLLER__REGULATION_FUNCTIONS_HPP_

#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "asb_regulated_pure_pursuit_controller/parameter_handler.hpp"

namespace asb_regulated_pure_pursuit_controller {

namespace heuristics {

/**
 * @brief apply curvature constraint regulation on the linear velocity
 * @param raw_linear_velocity Raw linear velocity desired
 * @param curvature Curvature of the current command to follow the path
 * @param min_radius Minimum path radius to apply the heuristic
 * @return Velocity after applying the curvature constraint
 */
inline double curvatureConstraint(const double raw_linear_vel, const double curvature, const double min_radius) {
    const double radius = fabs(1.0 / curvature);
    if (radius < min_radius) {
        return raw_linear_vel * (1.0 - (fabs(radius - min_radius) / min_radius));
    } else {
        return raw_linear_vel;
    }
}

/**
 * @brief apply cost constraint regulation on the linear velocity
 * @param raw_linear_velocity Raw linear velocity desired
 * @param pose_cost Cost at the robot pose
 * @param costmap_ros Costmap object to query
 * @param params Parameters
 * @return Velocity after applying the curvature constraint
 */
inline double costConstraint(const double raw_linear_vel, const double pose_cost, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros, Parameters *params) {
    using namespace nav2_costmap_2d;  // NOLINT

    if (pose_cost != static_cast<double>(NO_INFORMATION) && pose_cost != static_cast<double>(FREE_SPACE)) {
        const double &inscribed_radius = costmap_ros->getLayeredCostmap()->getInscribedRadius();

        const double min_distance_to_obstacle = (params->inflation_cost_scaling_factor * inscribed_radius - log(pose_cost) + log(253.0f)) / params->inflation_cost_scaling_factor;

        if (min_distance_to_obstacle < params->cost_scaling_dist) {
            return raw_linear_vel * (params->cost_scaling_gain * min_distance_to_obstacle / params->cost_scaling_dist);
        }
    }

    return raw_linear_vel;
}

}  // namespace heuristics

}  // namespace asb_regulated_pure_pursuit_controller

#endif  // ASB_REGULATED_PURE_PURSUIT_CONTROLLER__REGULATION_FUNCTIONS_HPP_
