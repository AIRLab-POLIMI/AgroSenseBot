// Copyright (c) 2020, Samsung Research America
// Copyright (c) 2023, Open Navigation LLC
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
// limitations under the License. Reserved.

#include <string>
#include <memory>
#include <vector>
#include <algorithm>
#include <limits>

#include "Eigen/Core"
#include "asb_smac_planner/smac_planner_hybrid.hpp"

// #define BENCHMARK_TESTING

namespace asb_smac_planner {

using namespace std::chrono;  // NOLINT
using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

SmacPlannerHybrid::SmacPlannerHybrid() : _a_star(nullptr), _collision_checker(nullptr, 1, nullptr), _smoother(nullptr), _costmap(nullptr), _costmap_downsampler(nullptr) {
}

SmacPlannerHybrid::~SmacPlannerHybrid() {
    RCLCPP_INFO(_logger, "Destroying plugin %s of type ASB SmacPlannerHybrid", _name.c_str());
}

void SmacPlannerHybrid::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name, std::shared_ptr<tf2_ros::Buffer>/*tf*/, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {
    _node = parent;
    auto node = parent.lock();
    _logger = node->get_logger();
    _clock = node->get_clock();
    _costmap = costmap_ros->getCostmap();
    _costmap_ros = costmap_ros;
    _name = name;
    _global_frame = costmap_ros->getGlobalFrameID();

    RCLCPP_INFO(_logger, "Configuring %s of type ASB SmacPlannerHybrid", name.c_str());

    int angle_quantizations;
    double analytic_expansion_max_length_m;
    bool smooth_path;

    // General planner params
    nav2_util::declare_parameter_if_not_declared(node, name + ".downsample_costmap", rclcpp::ParameterValue(false));
    node->get_parameter(name + ".downsample_costmap", _downsample_costmap);
    nav2_util::declare_parameter_if_not_declared(node, name + ".downsampling_factor", rclcpp::ParameterValue(1));
    node->get_parameter(name + ".downsampling_factor", _downsampling_factor);

    nav2_util::declare_parameter_if_not_declared(node, name + ".angle_quantization_bins", rclcpp::ParameterValue(72));
    node->get_parameter(name + ".angle_quantization_bins", angle_quantizations);
    _angle_bin_size = 2.0 * M_PI / angle_quantizations;
    _angle_quantizations = static_cast<unsigned int>(angle_quantizations);

    nav2_util::declare_parameter_if_not_declared(node, name + ".tolerance", rclcpp::ParameterValue(0.25));
    _tolerance = static_cast<float>(node->get_parameter(name + ".tolerance").as_double());
    nav2_util::declare_parameter_if_not_declared(node, name + ".allow_unknown", rclcpp::ParameterValue(true));
    node->get_parameter(name + ".allow_unknown", _allow_unknown);
    nav2_util::declare_parameter_if_not_declared(node, name + ".max_iterations", rclcpp::ParameterValue(1000000));
    node->get_parameter(name + ".max_iterations", _max_iterations);
    nav2_util::declare_parameter_if_not_declared(node, name + ".max_on_approach_iterations", rclcpp::ParameterValue(1000));
    node->get_parameter(name + ".max_on_approach_iterations", _max_on_approach_iterations);
    nav2_util::declare_parameter_if_not_declared(node, name + ".smooth_path", rclcpp::ParameterValue(true));
    node->get_parameter(name + ".smooth_path", smooth_path);

    nav2_util::declare_parameter_if_not_declared(node, name + ".minimum_turning_radius", rclcpp::ParameterValue(0.4));
    node->get_parameter(name + ".minimum_turning_radius", _minimum_turning_radius_global_coords);
    nav2_util::declare_parameter_if_not_declared(node, name + ".cache_obstacle_heuristic", rclcpp::ParameterValue(false));
    node->get_parameter(name + ".cache_obstacle_heuristic", _search_info.cache_obstacle_heuristic);
    nav2_util::declare_parameter_if_not_declared(node, name + ".reverse_penalty", rclcpp::ParameterValue(2.0));
    node->get_parameter(name + ".reverse_penalty", _search_info.reverse_penalty);
    nav2_util::declare_parameter_if_not_declared(node, name + ".cusp_penalty", rclcpp::ParameterValue(1.0));
    node->get_parameter(name + ".cusp_penalty", _search_info.cusp_penalty);
    nav2_util::declare_parameter_if_not_declared(node, name + ".change_penalty", rclcpp::ParameterValue(0.0));
    node->get_parameter(name + ".change_penalty", _search_info.change_penalty);
    nav2_util::declare_parameter_if_not_declared(node, name + ".non_straight_penalty", rclcpp::ParameterValue(1.2));
    node->get_parameter(name + ".non_straight_penalty", _search_info.non_straight_penalty);
    nav2_util::declare_parameter_if_not_declared(node, name + ".cost_penalty", rclcpp::ParameterValue(2.0));
    node->get_parameter(name + ".cost_penalty", _search_info.cost_penalty);
    nav2_util::declare_parameter_if_not_declared(node, name + ".retrospective_penalty", rclcpp::ParameterValue(0.015));
    node->get_parameter(name + ".retrospective_penalty", _search_info.retrospective_penalty);
    nav2_util::declare_parameter_if_not_declared(node, name + ".analytic_expansion_ratio", rclcpp::ParameterValue(3.5));
    node->get_parameter(name + ".analytic_expansion_ratio", _search_info.analytic_expansion_ratio);
    nav2_util::declare_parameter_if_not_declared(node, name + ".analytic_expansion_max_length", rclcpp::ParameterValue(3.0));
    node->get_parameter(name + ".analytic_expansion_max_length", analytic_expansion_max_length_m);
    _search_info.analytic_expansion_max_length = analytic_expansion_max_length_m / _costmap->getResolution();

    nav2_util::declare_parameter_if_not_declared(node, name + ".max_planning_time", rclcpp::ParameterValue(5.0));
    node->get_parameter(name + ".max_planning_time", _max_planning_time);
    nav2_util::declare_parameter_if_not_declared(node, name + ".lookup_table_size", rclcpp::ParameterValue(20.0));
    node->get_parameter(name + ".lookup_table_size", _lookup_table_size);
    nav2_util::declare_parameter_if_not_declared(node, name + ".visualizations_edges_line_size", rclcpp::ParameterValue(0.001));
    node->get_parameter(name + ".visualizations_edges_line_size", _visualizations_edges_line_size);
    nav2_util::declare_parameter_if_not_declared(node, name + ".visualizations_path_footprint_line_size", rclcpp::ParameterValue(0.01));
    node->get_parameter(name + ".visualizations_path_footprint_line_size", _visualizations_path_footprint_line_size);

    nav2_util::declare_parameter_if_not_declared(node, name + ".debug_visualizations", rclcpp::ParameterValue(false));
    node->get_parameter(name + ".debug_visualizations", _debug_visualizations);

    nav2_util::declare_parameter_if_not_declared(node, name + ".motion_model_for_search", rclcpp::ParameterValue(std::string("DUBIN")));
    node->get_parameter(name + ".motion_model_for_search", _motion_model_for_search);
    _motion_model = fromString(_motion_model_for_search);
    if (_motion_model == MotionModel::UNKNOWN) {
        RCLCPP_WARN(_logger, "Unable to get MotionModel search type. Given '%s', "
                             "valid options are MOORE, VON_NEUMANN, DUBIN, REEDS_SHEPP, STATE_LATTICE.", _motion_model_for_search.c_str());
    }

    nav2_util::declare_parameter_if_not_declared(node, name + ".motion_model_for_approach", rclcpp::ParameterValue(std::string("DUBIN")));
    node->get_parameter(name + ".motion_model_for_approach", _motion_model_for_approach);
    _approach_motion_model = fromString(_motion_model_for_approach);
    if (_approach_motion_model == MotionModel::UNKNOWN) {
        RCLCPP_WARN(_logger, "Unable to get approach MotionModel search type. Given '%s', "
                             "valid options are DUBIN, REEDS_SHEPP.", _motion_model_for_approach.c_str());
    }

    if (_max_on_approach_iterations <= 0) {
        RCLCPP_INFO(_logger, "On approach iteration selected as <= 0, "
                             "disabling tolerance and on approach iterations.");
        _max_on_approach_iterations = std::numeric_limits<int>::max();
    }

    if (_max_iterations <= 0) {
        RCLCPP_INFO(_logger, "maximum iteration selected as <= 0, "
                             "disabling maximum iterations.");
        _max_iterations = std::numeric_limits<int>::max();
    }

    // convert to grid coordinates
    if (!_downsample_costmap) {
        _downsampling_factor = 1;
    }
    _search_info.minimum_turning_radius = _minimum_turning_radius_global_coords / (_costmap->getResolution() * _downsampling_factor);
    _lookup_table_dim = static_cast<float>(_lookup_table_size) / static_cast<float>(_costmap->getResolution() * _downsampling_factor);

    // Make sure it is a whole number
    _lookup_table_dim = static_cast<float>(static_cast<int>(_lookup_table_dim));

    // Make sure it is an odd number
    if (static_cast<int>(_lookup_table_dim) % 2 == 0) {
        RCLCPP_INFO(_logger, "Even sized heuristic lookup table size set %f, increasing size by 1 to make odd", _lookup_table_dim);
        _lookup_table_dim += 1.0;
    }

    // Initialize collision checker
    _collision_checker = GridCollisionChecker(_costmap, _angle_quantizations, node);
    _collision_checker.setFootprint(_costmap_ros->getRobotFootprint(), _costmap_ros->getUseRadius(), findCircumscribedCost(_costmap_ros));

    // Initialize A* template
    _a_star = std::make_unique<AStarAlgorithm<NodeHybrid>>(_motion_model, _approach_motion_model, _search_info);
    _a_star->initialize(_allow_unknown, _max_iterations, _max_on_approach_iterations, _max_planning_time, _lookup_table_dim, _angle_quantizations);

    // Initialize path smoother
    if (smooth_path) {
        SmootherParams params;
        params.get(node, name);
        _smoother = std::make_unique<Smoother>(params);
        _smoother->initialize(_minimum_turning_radius_global_coords);
    }

    // Initialize costmap downsampler
    if (_downsample_costmap && _downsampling_factor > 1) {
        _costmap_downsampler = std::make_unique<CostmapDownsampler>();
        std::string topic_name = "downsampled_costmap";
        _costmap_downsampler->on_configure(node, _global_frame, topic_name, _costmap, _downsampling_factor);
    }

    _raw_plan_publisher = node->create_publisher<nav_msgs::msg::Path>("unsmoothed_plan", 1);

    if (_debug_visualizations) {
        _expansions_publisher = node->create_publisher<visualization_msgs::msg::MarkerArray>("expansions", 1);
    }
    _planned_footprints_publisher = node->create_publisher<visualization_msgs::msg::MarkerArray>("planned_footprints", 1);

    RCLCPP_INFO(_logger, "Configured plugin %s of type ASB SmacPlannerHybrid with "
                         "maximum iterations %i, max on approach iterations %i, and %s. Tolerance %.2f."
                         "Using search motion model: %s. Using approach search motion model: %s.", _name.c_str(), _max_iterations, _max_on_approach_iterations, _allow_unknown ? "allowing unknown traversal" : "not allowing unknown traversal", _tolerance, toString(_motion_model).c_str(), toString(_approach_motion_model).c_str());
}

void SmacPlannerHybrid::activate() {
    RCLCPP_INFO(_logger, "Activating plugin %s of type ASB SmacPlannerHybrid", _name.c_str());
    _raw_plan_publisher->on_activate();
    if (_debug_visualizations) {
        _expansions_publisher->on_activate();
    }
    _planned_footprints_publisher->on_activate();
    if (_costmap_downsampler) {
        _costmap_downsampler->on_activate();
    }
    auto node = _node.lock();
    // Add callback for dynamic parameters
    _dyn_params_handler = node->add_on_set_parameters_callback(std::bind(&SmacPlannerHybrid::dynamicParametersCallback, this, _1));
}

void SmacPlannerHybrid::deactivate() {
    RCLCPP_INFO(_logger, "Deactivating plugin %s of type ASB SmacPlannerHybrid", _name.c_str());
    _raw_plan_publisher->on_deactivate();
    if (_debug_visualizations) {
        _expansions_publisher->on_deactivate();
    }
    _planned_footprints_publisher->on_deactivate();
    if (_costmap_downsampler) {
        _costmap_downsampler->on_deactivate();
    }
    _dyn_params_handler.reset();
}

void SmacPlannerHybrid::cleanup() {
    RCLCPP_INFO(_logger, "Cleaning up plugin %s of type ASB SmacPlannerHybrid", _name.c_str());
    _a_star.reset();
    _smoother.reset();
    if (_costmap_downsampler) {
        _costmap_downsampler->on_cleanup();
        _costmap_downsampler.reset();
    }
    _raw_plan_publisher.reset();
    _expansions_publisher.reset();
    _planned_footprints_publisher.reset();
}

nav_msgs::msg::Path SmacPlannerHybrid::createPlan(const geometry_msgs::msg::PoseStamped &start, const geometry_msgs::msg::PoseStamped &goal) {
    std::lock_guard<std::mutex> lock_reinit(_mutex);
    steady_clock::time_point a = steady_clock::now();

    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(_costmap->getMutex()));

    // Downsample costmap, if required
    nav2_costmap_2d::Costmap2D *costmap = _costmap;
    if (_costmap_downsampler) {
        costmap = _costmap_downsampler->downsample(_downsampling_factor);
        _collision_checker.setCostmap(costmap);
    }

    // Set collision checker and costmap information
    _collision_checker.setFootprint(_costmap_ros->getRobotFootprint(), _costmap_ros->getUseRadius(), findCircumscribedCost(_costmap_ros));
    _a_star->setCollisionChecker(&_collision_checker);

    // Set starting point, in A* bin search coordinates
    unsigned int mx, my;
    if (!costmap->worldToMap(start.pose.position.x, start.pose.position.y, mx, my)) {
        throw nav2_core::StartOutsideMapBounds("Start Coordinates of(" + std::to_string(start.pose.position.x) + ", " + std::to_string(start.pose.position.y) + ") was outside bounds");
    }

    double orientation_bin = tf2::getYaw(start.pose.orientation) / _angle_bin_size;
    while (orientation_bin < 0.0) {
        orientation_bin += static_cast<float>(_angle_quantizations);
    }
    // This is needed to handle precision issues
    if (orientation_bin >= static_cast<float>(_angle_quantizations)) {
        orientation_bin -= static_cast<float>(_angle_quantizations);
    }
    unsigned int orientation_bin_id = static_cast<unsigned int>(floor(orientation_bin));
    _a_star->setStart(mx, my, orientation_bin_id);

    // Set goal point, in A* bin search coordinates
    if (!costmap->worldToMap(goal.pose.position.x, goal.pose.position.y, mx, my)) {
        throw nav2_core::GoalOutsideMapBounds("Goal Coordinates of(" + std::to_string(goal.pose.position.x) + ", " + std::to_string(goal.pose.position.y) + ") was outside bounds");
    }
    orientation_bin = tf2::getYaw(goal.pose.orientation) / _angle_bin_size;
    while (orientation_bin < 0.0) {
        orientation_bin += static_cast<float>(_angle_quantizations);
    }
    // This is needed to handle precision issues
    if (orientation_bin >= static_cast<float>(_angle_quantizations)) {
        orientation_bin -= static_cast<float>(_angle_quantizations);
    }
    orientation_bin_id = static_cast<unsigned int>(floor(orientation_bin));
    _a_star->setGoal(mx, my, orientation_bin_id);

    // Setup message
    nav_msgs::msg::Path plan;
    plan.header.stamp = _clock->now();
    plan.header.frame_id = _global_frame;
    geometry_msgs::msg::PoseStamped pose;
    pose.header = plan.header;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;

    // Compute plan
    NodeHybrid::CoordinateVector path;
    int num_iterations = 0;
    std::string error;
    std::unique_ptr<std::vector<std::tuple<float, float, float, float, float, bool>>> expansions = nullptr;
    if (_debug_visualizations && (_expansions_publisher->get_subscription_count() > 0)) {
        expansions = std::make_unique<std::vector<std::tuple<float, float, float, float, float, bool>>>();
    }
    // Note: All exceptions thrown are handled by the planner server and returned to the action
    if (!_a_star->createPath(path, num_iterations, _tolerance / static_cast<float>(costmap->getResolution()), expansions.get())) {
        if (num_iterations < _a_star->getMaxIterations()) {
            throw nav2_core::NoValidPathCouldBeFound("no valid path found");
        } else {
            throw nav2_core::PlannerTimedOut("exceeded maximum iterations");
        }
    }

    // Convert to world coordinates
    plan.poses.reserve(path.size());
    for (int i = path.size() - 1; i >= 0; --i) {
        pose.pose = getWorldCoords(path[i].x, path[i].y, costmap);
        pose.pose.orientation = i>0 ? getWorldOrientation(path[i].theta) : goal.pose.orientation; // set the orientation as the goal orientation to the last pose, ASB RPP will use it when it enters its lookahead distance
        plan.poses.push_back(pose);
    }

    // Publish raw path for debug
    if (_raw_plan_publisher->get_subscription_count() > 0) {
        _raw_plan_publisher->publish(plan);
    }

    // Publish expansions for debug
    if (_debug_visualizations && expansions && (_expansions_publisher->get_subscription_count() > 0)) {
        auto edges_marker = std::make_unique<visualization_msgs::msg::MarkerArray>();
        edges_marker->markers.resize(2);
        fillExpansionsMarker(edges_marker->markers[0], "expanded_search_edges", _visualizations_edges_line_size, 1.0f, 1.0f, 0.0f, 1.0f, _global_frame, _clock->now());
        fillExpansionsMarker(edges_marker->markers[1], "analytic_edges", _visualizations_edges_line_size, 1.0f, 0.0f, 0.0f, 1.0f, _global_frame, _clock->now());

        for (auto &e: *expansions) {
            bool analytic_expansion = std::get<5>(e);
            geometry_msgs::msg::Pose edge_pose_1 = getWorldCoords(std::get<0>(e), std::get<1>(e), costmap);
            edge_pose_1.position.z = 0.01;
            geometry_msgs::msg::Pose edge_pose_2 = getWorldCoords(std::get<2>(e), std::get<3>(e), costmap);
            edge_pose_1.position.z = 0.01;

            if (analytic_expansion) {
                edges_marker->markers[1].points.push_back(edge_pose_1.position);
                edges_marker->markers[1].points.push_back(edge_pose_2.position);
            } else {
                edges_marker->markers[0].points.push_back(edge_pose_1.position);
                edges_marker->markers[0].points.push_back(edge_pose_2.position);
            }
        }

        _expansions_publisher->publish(std::move(edges_marker));
    }

    // plot footprint path planned for debug
    if (_planned_footprints_publisher->get_subscription_count() > 0) {
        auto marker_array = std::make_unique<visualization_msgs::msg::MarkerArray>();
        for (size_t i = 0; i < plan.poses.size(); i++) {
            const std::vector<geometry_msgs::msg::Point> edge = transformFootprintToEdges(plan.poses[i].pose, _costmap_ros->getRobotFootprint());
            marker_array->markers.push_back(createMarker(edge, _visualizations_path_footprint_line_size, i, _global_frame, _clock->now()));
        }

        if (marker_array->markers.empty()) {
            visualization_msgs::msg::Marker clear_all_marker;
            clear_all_marker.action = visualization_msgs::msg::Marker::DELETEALL;
            marker_array->markers.push_back(clear_all_marker);
        }
        _planned_footprints_publisher->publish(std::move(marker_array));
    }

    steady_clock::time_point b = steady_clock::now();
    duration<double> time_span = duration_cast<duration<double>>(b - a);
    RCLCPP_INFO(_logger, "Created plan with ASB SmacPlannerHybrid in %.3f s", static_cast<double>(time_span.count()));

    // Find how much time we have left to do smoothing
    double time_remaining = _max_planning_time - static_cast<double>(time_span.count());

#ifdef BENCHMARK_TESTING
    std::cout << "It took " << time_span.count() * 1000 <<
      " milliseconds with " << num_iterations << " iterations." << std::endl;
#endif

    // Smooth plan
    if (_smoother && num_iterations > 1) {
        _smoother->smooth(plan, costmap, time_remaining);
    }

#ifdef BENCHMARK_TESTING
    steady_clock::time_point c = steady_clock::now();
    duration<double> time_span2 = duration_cast<duration<double>>(c - b);
    std::cout << "It took " << time_span2.count() * 1000 <<
      " milliseconds to smooth path." << std::endl;
#endif

    return plan;
}

rcl_interfaces::msg::SetParametersResult SmacPlannerHybrid::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    std::lock_guard<std::mutex> lock_reinit(_mutex);

    bool reinit_collision_checker = false;
    bool reinit_a_star = false;
    bool reinit_downsampler = false;
    bool reinit_smoother = false;

    for (auto parameter: parameters) {
        const auto &type = parameter.get_type();
        const auto &name = parameter.get_name();

        if (type == ParameterType::PARAMETER_DOUBLE) {
            if (name == _name + ".max_planning_time") {
                reinit_a_star = true;
                _max_planning_time = parameter.as_double();
            } else if (name == _name + ".tolerance") {
                _tolerance = static_cast<float>(parameter.as_double());
            } else if (name == _name + ".lookup_table_size") {
                reinit_a_star = true;
                _lookup_table_size = parameter.as_double();
            } else if (name == _name + ".visualizations_edges_line_size") {
                reinit_a_star = true;
                _visualizations_edges_line_size = parameter.as_double();
            } else if (name == _name + ".minimum_turning_radius") {
                reinit_a_star = true;
                if (_smoother) {
                    reinit_smoother = true;
                }
                _minimum_turning_radius_global_coords = static_cast<float>(parameter.as_double());
            } else if (name == _name + ".reverse_penalty") {
                reinit_a_star = true;
                _search_info.reverse_penalty = static_cast<float>(parameter.as_double());
            } else if (name == _name + ".cusp_penalty") {
                reinit_a_star = true;
                _search_info.cusp_penalty = static_cast<float>(parameter.as_double());
            } else if (name == _name + ".change_penalty") {
                reinit_a_star = true;
                _search_info.change_penalty = static_cast<float>(parameter.as_double());
            } else if (name == _name + ".non_straight_penalty") {
                reinit_a_star = true;
                _search_info.non_straight_penalty = static_cast<float>(parameter.as_double());
            } else if (name == _name + ".cost_penalty") {
                reinit_a_star = true;
                _search_info.cost_penalty = static_cast<float>(parameter.as_double());
            } else if (name == _name + ".analytic_expansion_ratio") {
                reinit_a_star = true;
                _search_info.analytic_expansion_ratio = static_cast<float>(parameter.as_double());
            } else if (name == _name + ".analytic_expansion_max_length") {
                reinit_a_star = true;
                _search_info.analytic_expansion_max_length = static_cast<float>(parameter.as_double()) / _costmap->getResolution();
            }
        } else if (type == ParameterType::PARAMETER_BOOL) {
            if (name == _name + ".downsample_costmap") {
                reinit_downsampler = true;
                _downsample_costmap = parameter.as_bool();
            } else if (name == _name + ".allow_unknown") {
                reinit_a_star = true;
                _allow_unknown = parameter.as_bool();
            } else if (name == _name + ".cache_obstacle_heuristic") {
                reinit_a_star = true;
                _search_info.cache_obstacle_heuristic = parameter.as_bool();
            } else if (name == _name + ".smooth_path") {
                if (parameter.as_bool()) {
                    reinit_smoother = true;
                } else {
                    _smoother.reset();
                }
            }
        } else if (type == ParameterType::PARAMETER_INTEGER) {
            if (name == _name + ".downsampling_factor") {
                reinit_a_star = true;
                reinit_downsampler = true;
                _downsampling_factor = parameter.as_int();
            } else if (name == _name + ".max_iterations") {
                reinit_a_star = true;
                _max_iterations = parameter.as_int();
                if (_max_iterations <= 0) {
                    RCLCPP_INFO(_logger, "maximum iteration selected as <= 0, "
                                         "disabling maximum iterations.");
                    _max_iterations = std::numeric_limits<int>::max();
                }
            } else if (name == _name + ".max_on_approach_iterations") {
                reinit_a_star = true;
                _max_on_approach_iterations = parameter.as_int();
                if (_max_on_approach_iterations <= 0) {
                    RCLCPP_INFO(_logger, "On approach iteration selected as <= 0, "
                                         "disabling tolerance and on approach iterations.");
                    _max_on_approach_iterations = std::numeric_limits<int>::max();
                }
            } else if (name == _name + ".angle_quantization_bins") {
                reinit_collision_checker = true;
                reinit_a_star = true;
                int angle_quantizations = parameter.as_int();
                _angle_bin_size = 2.0 * M_PI / angle_quantizations;
                _angle_quantizations = static_cast<unsigned int>(angle_quantizations);
            }
        } else if (type == ParameterType::PARAMETER_STRING) {
            if (name == _name + ".motion_model_for_search") {
                reinit_a_star = true;
                _motion_model = fromString(parameter.as_string());
                if (_motion_model == MotionModel::UNKNOWN) {
                    RCLCPP_WARN(_logger, "Unable to get MotionModel search type. Given '%s', "
                                         "valid options are MOORE, VON_NEUMANN, DUBIN, REEDS_SHEPP.", _motion_model_for_search.c_str());
                }
            } else if (name == _name + ".motion_model_for_approach") {
                reinit_a_star = true;
                _approach_motion_model = fromString(parameter.as_string());
                if (_approach_motion_model == MotionModel::UNKNOWN) {
                    RCLCPP_WARN(_logger, "Unable to get approach MotionModel search type. Given '%s', "
                                         "valid options are MOORE, VON_NEUMANN, DUBIN, REEDS_SHEPP.", _motion_model_for_approach.c_str());
                }
            }
        }
    }

    // Re-init if needed with mutex lock (to avoid re-init while creating a plan)
    if (reinit_a_star || reinit_downsampler || reinit_collision_checker || reinit_smoother) {
        // convert to grid coordinates
        if (!_downsample_costmap) {
            _downsampling_factor = 1;
        }
        _search_info.minimum_turning_radius = _minimum_turning_radius_global_coords / (_costmap->getResolution() * _downsampling_factor);
        _lookup_table_dim = static_cast<float>(_lookup_table_size) / static_cast<float>(_costmap->getResolution() * _downsampling_factor);

        // Make sure it is a whole number
        _lookup_table_dim = static_cast<float>(static_cast<int>(_lookup_table_dim));

        // Make sure it is an odd number
        if (static_cast<int>(_lookup_table_dim) % 2 == 0) {
            RCLCPP_INFO(_logger, "Even sized heuristic lookup table size set %f, increasing size by 1 to make odd", _lookup_table_dim);
            _lookup_table_dim += 1.0;
        }

        auto node = _node.lock();

        // Re-Initialize A* template
        if (reinit_a_star) {
            _a_star = std::make_unique<AStarAlgorithm<NodeHybrid>>(_motion_model, _approach_motion_model, _search_info);
            _a_star->initialize(_allow_unknown, _max_iterations, _max_on_approach_iterations, _max_planning_time, _lookup_table_dim, _angle_quantizations);
        }

        // Re-Initialize costmap downsampler
        if (reinit_downsampler) {
            if (_downsample_costmap && _downsampling_factor > 1) {
                std::string topic_name = "downsampled_costmap";
                _costmap_downsampler = std::make_unique<CostmapDownsampler>();
                _costmap_downsampler->on_configure(node, _global_frame, topic_name, _costmap, _downsampling_factor);
            }
        }

        // Re-Initialize collision checker
        if (reinit_collision_checker) {
            _collision_checker = GridCollisionChecker(_costmap, _angle_quantizations, node);
            _collision_checker.setFootprint(_costmap_ros->getRobotFootprint(), _costmap_ros->getUseRadius(), findCircumscribedCost(_costmap_ros));
        }

        // Re-Initialize smoother
        if (reinit_smoother) {
            SmootherParams params;
            params.get(node, _name);
            _smoother = std::make_unique<Smoother>(params);
            _smoother->initialize(_minimum_turning_radius_global_coords);
        }
    }
    result.successful = true;
    return result;
}

}  // namespace asb_smac_planner

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(asb_smac_planner::SmacPlannerHybrid, nav2_core::GlobalPlanner)
