// Copyright 2023 TIER IV, Inc.
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

#ifndef PATH_OPTIMIZER__PATH_OPTIMIZER_TYPES_HPP_
#define PATH_OPTIMIZER__PATH_OPTIMIZER_TYPES_HPP_

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace autoware::path_optimizer
{

// Basic geometric types
struct Point {
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct Quaternion {
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double w{1.0};
};

struct Pose {
  Point position;
  Quaternion orientation;
};

// PathPoint - represents a point on the reference path
struct PathPoint {
  Pose pose;
  double longitudinal_velocity_mps{0.0};
  double lateral_velocity_mps{0.0};
  double heading_rate_rps{0.0};
};

// TrajectoryPoint - optimized trajectory point with additional fields
struct TrajectoryPoint {
  Pose pose;
  double longitudinal_velocity_mps{0.0};
  double lateral_velocity_mps{0.0};
  double heading_rate_rps{0.0};
  double acceleration_mps2{0.0};
  double front_wheel_angle_rad{0.0};
  double rear_wheel_angle_rad{0.0};
};

// Vehicle information
struct VehicleInfo {
  double wheel_base{2.79};
  double front_overhang{0.96};
  double rear_overhang{1.02};
  double vehicle_width{1.92};
  double vehicle_length{4.77};
  double max_steer_angle{0.7};
  double max_steer_angle_rad{0.7};  // alias
};

// Bounds for optimization constraints
struct Bounds {
  double lower_bound{0.0};
  double upper_bound{0.0};
};

// Kinematic state for optimization
struct KinematicState {
  double lat{0.0};   // lateral error
  double yaw{0.0};   // yaw error
};

// Reference point for MPT optimization
struct ReferencePoint {
  Pose pose;
  double longitudinal_velocity_mps{0.0};
  
  // Optimization data
  double curvature{0.0};
  double delta_arc_length{0.0};
  double alpha{0.0};  // Curvature angle for optimization center offset
  double normalized_avoidance_cost{0.0};  // [0,1] for adaptive weight interpolation
  Bounds bounds{};
  
  // Optimization results
  std::optional<KinematicState> fixed_kinematic_state{std::nullopt};
  KinematicState optimized_kinematic_state{};
  double optimized_input{0.0};
};

// MPT (Model Predictive Trajectory) parameters
struct MPTParam {
  // State equation
  int num_curvature_sampling_points{5};
  double delta_arc_length_for_mpt_points{1.0};
  
  // Optimization
  int num_points{100};
  double max_optimization_time_ms{50.0};
  
  // Objective weights
  double l_inf_weight{1.0};
  double lat_error_weight{1.0};
  double weight_lat_error{1.0};  // alias for lat_error_weight
  double yaw_error_weight{0.0};
  double yaw_error_rate_weight{0.0};
  double steer_input_weight{1.0};
  double weight_steer_input{0.1};  // alias for steer_input_weight
  double steer_rate_weight{1.0};
  
  // Adaptive weights for terminal and goal points (ROS2 compatibility)
  double terminal_lat_error_weight{100.0};  // Strong tracking at terminal point
  double terminal_yaw_error_weight{0.0};
  double goal_lat_error_weight{1000.0};  // Very strong tracking at goal
  double goal_yaw_error_weight{0.0};
  
  // Optimization center offset (ROS2 compatibility)
  double optimization_center_offset{0.0};  // Default 0, typically wheelbase * 0.8
  
  // Constraints
  double max_steer_rad{0.7};
  double max_steer_rate_rad_per_s{0.5};
  
  // Collision avoidance
  bool enable_avoidance{true};
  double avoidance_precision{0.5};
  double soft_collision_free_weight{1000.0};
  
  // Terminal condition
  bool enable_terminal_constraint{true};
  double terminal_lat_error_threshold{0.3};
  double terminal_yaw_error_threshold{0.1};
};

// Trajectory parameters
struct TrajectoryParam {
  double output_delta_arc_length{0.5};
  double output_backward_traj_length{2.0};
  int num_sampling_points{100};
};

// Ego nearest parameters
struct EgoNearestParam {
  double dist_threshold{3.0};
  double yaw_threshold{1.046};  // ~60 degrees
};

// Replan checker parameters
struct ReplanCheckerParam {
  double max_path_shape_change_dist{0.5};
  double max_ego_moving_dist{5.0};
  double max_delta_time_sec{2.0};
};

// Main path optimizer parameters
struct PathOptimizerParam {
  TrajectoryParam trajectory;
  EgoNearestParam ego_nearest;
  MPTParam mpt;
  ReplanCheckerParam replan_checker;
  
  bool enable_outside_drivable_area_stop{true};
  double vehicle_stop_margin_outside_drivable_area{0.5};
  bool enable_skip_optimization{false};
  bool enable_reset_prev_optimization{true};
};

// Planner data structure
struct PlannerData {
  std::vector<TrajectoryPoint> traj_points;
  std::vector<Point> left_bound;
  std::vector<Point> right_bound;
  
  Pose ego_pose;
  double ego_vel{0.0};
};

// Optimization result
struct OptimizationResult {
  std::vector<TrajectoryPoint> trajectory;
  std::vector<ReferencePoint> reference_points;
  bool success{false};
  std::string error_message;
  double computation_time_ms{0.0};
};

}  // namespace autoware::path_optimizer

#endif  // PATH_OPTIMIZER__PATH_OPTIMIZER_TYPES_HPP_
