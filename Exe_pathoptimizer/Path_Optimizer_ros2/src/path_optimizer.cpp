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

#include "path_optimizer.hpp"
#include "mpt_optimizer.hpp"
#include "replan_checker.hpp"

#include <chrono>
#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <iostream>

namespace autoware::path_optimizer
{

PathOptimizer::PathOptimizer(
  const PathOptimizerParam & param,
  const VehicleInfo & vehicle_info)
: param_(param)
, vehicle_info_(vehicle_info)
{
  mpt_optimizer_ = std::make_unique<MPTOptimizer>(param_.mpt, vehicle_info_);
  replan_checker_ = std::make_unique<ReplanChecker>(param_.replan_checker);
  
  std::cout << "[PathOptimizer] Initialized with:" << std::endl;
  std::cout << "  - Vehicle wheelbase: " << vehicle_info_.wheel_base << " m" << std::endl;
  std::cout << "  - Max steer angle: " << vehicle_info_.max_steer_angle << " rad" << std::endl;
  std::cout << "  - MPT num points: " << param_.mpt.num_points << std::endl;
}

PathOptimizer::~PathOptimizer() = default;

std::vector<TrajectoryPoint> PathOptimizer::optimizePath(
  const std::vector<PathPoint> & path_points,
  const std::vector<Point> & left_bound,
  const std::vector<Point> & right_bound,
  const Pose & ego_pose,
  const double ego_velocity)
{
  auto result = optimizePathWithDebug(
    path_points, left_bound, right_bound, ego_pose, ego_velocity);
  
  return result.trajectory;
}

OptimizationResult PathOptimizer::optimizePathWithDebug(
  const std::vector<PathPoint> & path_points,
  const std::vector<Point> & left_bound,
  const std::vector<Point> & right_bound,
  const Pose & ego_pose,
  const double ego_velocity)
{
  OptimizationResult result;
  
  auto start_time = std::chrono::high_resolution_clock::now();
  
  // 1. Create planner data
  auto planner_data = createPlannerData(
    path_points, left_bound, right_bound, ego_pose, ego_velocity);
  
  if (planner_data.traj_points.empty()) {
    result.error_message = "No trajectory points to optimize";
    std::cerr << "[PathOptimizer] " << result.error_message << std::endl;
    return result;
  }
  
  // 2. Check if replan is required
  bool need_replan = true;
  if (!prev_optimized_traj_.empty()) {
    need_replan = replan_checker_->isReplanRequired(
      planner_data.traj_points, ego_pose, 0.0);  // time=0 for standalone
  }
  
  std::cout << "[PathOptimizer] Replan required: " 
            << (need_replan ? "YES" : "NO") << std::endl;
  
  // 3. Optimize trajectory
  std::optional<std::vector<TrajectoryPoint>> optimized_traj;
  
  if (need_replan || prev_optimized_traj_.empty()) {
    optimized_traj = mpt_optimizer_->optimize(
      planner_data.traj_points,
      planner_data.left_bound,
      planner_data.right_bound,
      planner_data.ego_pose,
      planner_data.ego_vel);
      
    if (optimized_traj.has_value()) {
      prev_optimized_traj_ = optimized_traj.value();
      replan_checker_->updatePreviousData(prev_optimized_traj_, ego_pose, 0.0);
      result.success = true;
    } else {
      result.error_message = "Optimization failed";
      std::cerr << "[PathOptimizer] " << result.error_message << std::endl;
      
      // Use previous trajectory if available
      if (!prev_optimized_traj_.empty()) {
        optimized_traj = prev_optimized_traj_;
        result.success = true;
        result.error_message = "Using previous trajectory";
      }
    }
  } else {
    // Use previous optimized trajectory
    optimized_traj = prev_optimized_traj_;
    result.success = true;
  }
  
  // 4. Apply input velocity
  if (optimized_traj.has_value()) {
    result.trajectory = optimized_traj.value();
    applyInputVelocity(result.trajectory, path_points, ego_pose);
    
    // 5. Resample trajectory with output_delta_arc_length
    if (param_.trajectory.output_delta_arc_length > 0.0) {
      result.trajectory = resampleTrajectory(
        result.trajectory, param_.trajectory.output_delta_arc_length);
    }
    
    // 6. Calculate additional control fields (heading_rate, front_wheel_angle)
    calculateControlFields(result.trajectory);
  }
  
  // 6. Get reference points for debug
  result.reference_points = mpt_optimizer_->getReferencePoints();
  
  auto end_time = std::chrono::high_resolution_clock::now();
  result.computation_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    end_time - start_time).count();
  
  std::cout << "[PathOptimizer] Optimization completed in " 
            << result.computation_time_ms << " ms" << std::endl;
  std::cout << "  - Input points: " << path_points.size() << std::endl;
  std::cout << "  - Output points: " << result.trajectory.size() << std::endl;
  
  return result;
}

PlannerData PathOptimizer::createPlannerData(
  const std::vector<PathPoint> & path_points,
  const std::vector<Point> & left_bound,
  const std::vector<Point> & right_bound,
  const Pose & ego_pose,
  const double ego_velocity) const
{
  PlannerData data;
  
  data.traj_points = convertPathToTrajectory(path_points);
  data.left_bound = left_bound;
  data.right_bound = right_bound;
  data.ego_pose = ego_pose;
  data.ego_vel = ego_velocity;
  
  return data;
}

std::vector<TrajectoryPoint> PathOptimizer::convertPathToTrajectory(
  const std::vector<PathPoint> & path_points) const
{
  std::vector<TrajectoryPoint> traj_points;
  traj_points.reserve(path_points.size());
  
  for (const auto & path_point : path_points) {
    TrajectoryPoint traj_point;
    traj_point.pose = path_point.pose;
    traj_point.longitudinal_velocity_mps = path_point.longitudinal_velocity_mps;
    traj_point.lateral_velocity_mps = path_point.lateral_velocity_mps;
    traj_point.heading_rate_rps = path_point.heading_rate_rps;
    
    traj_points.push_back(traj_point);
  }
  
  return traj_points;
}

void PathOptimizer::applyInputVelocity(
  std::vector<TrajectoryPoint> & output_traj,
  const std::vector<PathPoint> & input_path,
  const Pose & ego_pose) const
{
  if (output_traj.empty() || input_path.empty()) {
    return;
  }
  
  // Simple velocity application - copy from input path
  for (size_t i = 0; i < output_traj.size() && i < input_path.size(); ++i) {
    output_traj[i].longitudinal_velocity_mps = input_path[i].longitudinal_velocity_mps;
  }
}

bool PathOptimizer::checkIfInsideDrivableArea(
  const TrajectoryPoint & point,
  const std::vector<Point> & left_bound,
  const std::vector<Point> & right_bound) const
{
  // Simplified check - implement proper polygon check if needed
  return true;
}

std::vector<TrajectoryPoint> PathOptimizer::resampleTrajectory(
  const std::vector<TrajectoryPoint> & trajectory,
  const double delta_arc_length) const
{
  if (trajectory.empty() || delta_arc_length <= 0.0) {
    return trajectory;
  }
  
  // Calculate total path length
  double total_length = 0.0;
  for (size_t i = 1; i < trajectory.size(); ++i) {
    const auto & p1 = trajectory[i - 1].pose.position;
    const auto & p2 = trajectory[i].pose.position;
    total_length += std::hypot(p2.x - p1.x, p2.y - p1.y);
  }
  
  // Calculate number of output points
  const size_t num_output_points = static_cast<size_t>(
    std::ceil(total_length / delta_arc_length)) + 1;
  
  std::vector<TrajectoryPoint> resampled;
  resampled.reserve(num_output_points);
  
  // Always add first point
  resampled.push_back(trajectory.front());
  
  // Resample trajectory
  double accumulated_length = 0.0;
  double target_length = delta_arc_length;
  
  for (size_t i = 1; i < trajectory.size(); ++i) {
    const auto & prev_point = trajectory[i - 1];
    const auto & curr_point = trajectory[i];
    
    const auto & p1 = prev_point.pose.position;
    const auto & p2 = curr_point.pose.position;
    
    const double segment_length = std::hypot(p2.x - p1.x, p2.y - p1.y);
    
    while (target_length <= accumulated_length + segment_length && 
           resampled.size() < num_output_points) {
      // Interpolate point at target_length
      const double ratio = (target_length - accumulated_length) / segment_length;
      
      TrajectoryPoint interp_point;
      
      // Interpolate position
      interp_point.pose.position.x = p1.x + ratio * (p2.x - p1.x);
      interp_point.pose.position.y = p1.y + ratio * (p2.y - p1.y);
      interp_point.pose.position.z = p1.z + ratio * (p2.z - p1.z);
      
      // Interpolate orientation (simplified quaternion interpolation)
      const auto & q1 = prev_point.pose.orientation;
      const auto & q2 = curr_point.pose.orientation;
      
      interp_point.pose.orientation.x = q1.x + ratio * (q2.x - q1.x);
      interp_point.pose.orientation.y = q1.y + ratio * (q2.y - q1.y);
      interp_point.pose.orientation.z = q1.z + ratio * (q2.z - q1.z);
      interp_point.pose.orientation.w = q1.w + ratio * (q2.w - q1.w);
      
      // Normalize quaternion
      const double norm = std::sqrt(
        interp_point.pose.orientation.x * interp_point.pose.orientation.x +
        interp_point.pose.orientation.y * interp_point.pose.orientation.y +
        interp_point.pose.orientation.z * interp_point.pose.orientation.z +
        interp_point.pose.orientation.w * interp_point.pose.orientation.w);
      
      if (norm > 1e-6) {
        interp_point.pose.orientation.x /= norm;
        interp_point.pose.orientation.y /= norm;
        interp_point.pose.orientation.z /= norm;
        interp_point.pose.orientation.w /= norm;
      }
      
      // Interpolate velocity
      interp_point.longitudinal_velocity_mps = 
        prev_point.longitudinal_velocity_mps + 
        ratio * (curr_point.longitudinal_velocity_mps - prev_point.longitudinal_velocity_mps);
      
      interp_point.lateral_velocity_mps = 
        prev_point.lateral_velocity_mps + 
        ratio * (curr_point.lateral_velocity_mps - prev_point.lateral_velocity_mps);
      
      interp_point.acceleration_mps2 = 
        prev_point.acceleration_mps2 + 
        ratio * (curr_point.acceleration_mps2 - prev_point.acceleration_mps2);
      
      interp_point.heading_rate_rps = 
        prev_point.heading_rate_rps + 
        ratio * (curr_point.heading_rate_rps - prev_point.heading_rate_rps);
      
      resampled.push_back(interp_point);
      target_length += delta_arc_length;
    }
    
    accumulated_length += segment_length;
  }
  
  // Add last point if not already added
  if (resampled.empty() || 
      std::abs(resampled.back().pose.position.x - trajectory.back().pose.position.x) > 1e-6 ||
      std::abs(resampled.back().pose.position.y - trajectory.back().pose.position.y) > 1e-6) {
    resampled.push_back(trajectory.back());
  }
  
  return resampled;
}

void PathOptimizer::calculateControlFields(
  std::vector<TrajectoryPoint> & trajectory) const
{
  if (trajectory.size() < 2) {
    return;
  }
  
  const double dt = 0.1;  // Assumed time step (100ms)
  const double wheelbase = vehicle_info_.wheel_base;
  
  // Calculate heading_rate and front_wheel_angle for each point
  for (size_t i = 0; i < trajectory.size(); ++i) {
    auto & point = trajectory[i];
    
    // Get current yaw
    double curr_yaw = 2.0 * std::atan2(
      point.pose.orientation.z, point.pose.orientation.w);
    
    // Calculate heading_rate (yaw rate)
    if (i < trajectory.size() - 1) {
      const auto & next_point = trajectory[i + 1];
      double next_yaw = 2.0 * std::atan2(
        next_point.pose.orientation.z, next_point.pose.orientation.w);
      
      // Handle angle wrap-around
      double dyaw = next_yaw - curr_yaw;
      while (dyaw > M_PI) dyaw -= 2.0 * M_PI;
      while (dyaw < -M_PI) dyaw += 2.0 * M_PI;
      
      point.heading_rate_rps = dyaw / dt;
    } else if (i > 0) {
      // Use previous point's heading rate for last point
      point.heading_rate_rps = trajectory[i - 1].heading_rate_rps;
    }
    
    // Calculate front_wheel_angle using Bicycle model
    if (i < trajectory.size() - 1) {
      const auto & next_point = trajectory[i + 1];
      
      // Calculate distance
      const double dx = next_point.pose.position.x - point.pose.position.x;
      const double dy = next_point.pose.position.y - point.pose.position.y;
      const double ds = std::hypot(dx, dy);
      
      if (ds > 1e-6) {
        double next_yaw = 2.0 * std::atan2(
          next_point.pose.orientation.z, next_point.pose.orientation.w);
        
        // Handle angle wrap-around
        double dyaw = next_yaw - curr_yaw;
        while (dyaw > M_PI) dyaw -= 2.0 * M_PI;
        while (dyaw < -M_PI) dyaw += 2.0 * M_PI;
        
        // Curvature
        double curvature = dyaw / ds;
        
        // Steering angle from Bicycle model: δ = atan(L * κ)
        point.front_wheel_angle_rad = std::atan(wheelbase * curvature);
        
        // Clamp to vehicle limits
        const double max_steer = vehicle_info_.max_steer_angle;
        point.front_wheel_angle_rad = std::clamp(
          point.front_wheel_angle_rad, -max_steer, max_steer);
      }
    } else if (i > 0) {
      // Use previous point's steering angle for last point
      point.front_wheel_angle_rad = trajectory[i - 1].front_wheel_angle_rad;
    }
    
    // lateral_velocity_mps and rear_wheel_angle_rad remain 0.0 (default)
  }
}

}  // namespace autoware::path_optimizer
