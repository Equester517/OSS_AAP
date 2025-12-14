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

#include "replan_checker.hpp"

#include <cmath>
#include <iostream>

namespace autoware::path_optimizer
{

ReplanChecker::ReplanChecker(const ReplanCheckerParam & param)
: param_(param)
{
  // std::cout << "[ReplanChecker] Initialized with:" << std::endl;
  // std::cout << "  - Max path shape change: " << param_.max_path_shape_change_dist << " m" << std::endl;
  // std::cout << "  - Max ego moving dist: " << param_.max_ego_moving_dist << " m" << std::endl;
  // std::cout << "  - Max delta time: " << param_.max_delta_time_sec << " s" << std::endl;
}

bool ReplanChecker::isReplanRequired(
  const std::vector<TrajectoryPoint> & current_trajectory,
  const Pose & current_ego_pose,
  const double current_time_sec) const
{
  // If no previous data, replan is required
  if (!prev_traj_points_.has_value() || 
      !prev_ego_pose_.has_value() || 
      !prev_replanned_time_sec_.has_value()) {
    // std::cout << "[ReplanChecker] No previous data - replan required" << std::endl;
    return true;
  }
  
  // Check time threshold
  double delta_time = current_time_sec - prev_replanned_time_sec_.value();
  if (delta_time > param_.max_delta_time_sec) {
    // std::cout << "[ReplanChecker] Time threshold exceeded (" 
    // << delta_time << " > " << param_.max_delta_time_sec 
    // << ") - replan required" << std::endl;
    return true;
  }
  
  // Check ego moving distance
  double ego_moving_dist = calculateDistance(current_ego_pose, prev_ego_pose_.value());
  if (ego_moving_dist > param_.max_ego_moving_dist) {
    // std::cout << "[ReplanChecker] Ego moving distance exceeded (" 
    // << ego_moving_dist << " > " << param_.max_ego_moving_dist 
    // << ") - replan required" << std::endl;
    return true;
  }
  
  // Check path shape change
  double path_shape_change = calculatePathShapeChange(
    current_trajectory, prev_traj_points_.value());
  if (path_shape_change > param_.max_path_shape_change_dist) {
    // std::cout << "[ReplanChecker] Path shape change exceeded (" 
    // << path_shape_change << " > " << param_.max_path_shape_change_dist 
    // << ") - replan required" << std::endl;
    return true;
  }
  
  // std::cout << "[ReplanChecker] No replan required" << std::endl;
  return false;
}

void ReplanChecker::updatePreviousData(
  const std::vector<TrajectoryPoint> & traj_points,
  const Pose & ego_pose,
  const double current_time_sec)
{
  prev_traj_points_ = traj_points;
  prev_ego_pose_ = ego_pose;
  prev_replanned_time_sec_ = current_time_sec;
  
  // std::cout << "[ReplanChecker] Updated previous data (time=" 
  // << current_time_sec << ")" << std::endl;
}

void ReplanChecker::reset()
{
  prev_traj_points_ = std::nullopt;
  prev_ego_pose_ = std::nullopt;
  prev_replanned_time_sec_ = std::nullopt;
  
  // std::cout << "[ReplanChecker] Reset" << std::endl;
}

double ReplanChecker::calculatePathShapeChange(
  const std::vector<TrajectoryPoint> & traj1,
  const std::vector<TrajectoryPoint> & traj2) const
{
  if (traj1.empty() || traj2.empty()) {
    return 0.0;
  }
  
  // Calculate maximum distance between corresponding points
  double max_dist = 0.0;
  size_t min_size = std::min(traj1.size(), traj2.size());
  
  for (size_t i = 0; i < min_size; ++i) {
    double dx = traj1[i].pose.position.x - traj2[i].pose.position.x;
    double dy = traj1[i].pose.position.y - traj2[i].pose.position.y;
    double dist = std::hypot(dx, dy);
    
    if (dist > max_dist) {
      max_dist = dist;
    }
  }
  
  return max_dist;
}

double ReplanChecker::calculateDistance(const Pose & pose1, const Pose & pose2) const
{
  double dx = pose1.position.x - pose2.position.x;
  double dy = pose1.position.y - pose2.position.y;
  double dz = pose1.position.z - pose2.position.z;
  
  return std::hypot(dx, dy, dz);
}

}  // namespace autoware::path_optimizer
