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

#ifndef PATH_OPTIMIZER__PATH_OPTIMIZER_HPP_
#define PATH_OPTIMIZER__PATH_OPTIMIZER_HPP_

#include "path_optimizer_types.hpp"

#include <memory>
#include <vector>

namespace autoware::path_optimizer
{

// Forward declarations
class MPTOptimizer;
class ReplanChecker;

class PathOptimizer
{
public:
  PathOptimizer(
    const PathOptimizerParam & param,
    const VehicleInfo & vehicle_info);

  ~PathOptimizer();

  // Main optimization function
  std::vector<TrajectoryPoint> optimizePath(
    const std::vector<PathPoint> & path_points,
    const std::vector<Point> & left_bound,
    const std::vector<Point> & right_bound,
    const Pose & ego_pose,
    const double ego_velocity);

  // Optimization with detailed result
  OptimizationResult optimizePathWithDebug(
    const std::vector<PathPoint> & path_points,
    const std::vector<Point> & left_bound,
    const std::vector<Point> & right_bound,
    const Pose & ego_pose,
    const double ego_velocity);

private:
  PathOptimizerParam param_;
  VehicleInfo vehicle_info_;
  
  std::unique_ptr<MPTOptimizer> mpt_optimizer_;
  std::unique_ptr<ReplanChecker> replan_checker_;
  
  // Previous optimization data
  std::vector<TrajectoryPoint> prev_optimized_traj_;
  
  // Helper functions
  PlannerData createPlannerData(
    const std::vector<PathPoint> & path_points,
    const std::vector<Point> & left_bound,
    const std::vector<Point> & right_bound,
    const Pose & ego_pose,
    const double ego_velocity) const;
    
  std::vector<TrajectoryPoint> convertPathToTrajectory(
    const std::vector<PathPoint> & path_points) const;
    
  void applyInputVelocity(
    std::vector<TrajectoryPoint> & output_traj,
    const std::vector<PathPoint> & input_path,
    const Pose & ego_pose) const;
  
  // Trajectory resampling
  std::vector<TrajectoryPoint> resampleTrajectory(
    const std::vector<TrajectoryPoint> & trajectory,
    const double delta_arc_length) const;
  
  // Calculate control fields (heading_rate, front_wheel_angle)
  void calculateControlFields(
    std::vector<TrajectoryPoint> & trajectory) const;
    
  bool checkIfInsideDrivableArea(
    const TrajectoryPoint & point,
    const std::vector<Point> & left_bound,
    const std::vector<Point> & right_bound) const;
};

}  // namespace autoware::path_optimizer

#endif  // PATH_OPTIMIZER__PATH_OPTIMIZER_HPP_
