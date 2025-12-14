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

#ifndef PATH_OPTIMIZER__MPT_OPTIMIZER_HPP_
#define PATH_OPTIMIZER__MPT_OPTIMIZER_HPP_

#include "path_optimizer_types.hpp"
#include "state_equation_generator.hpp"

#include <Eigen/Core>
#include <Eigen/Sparse>

#include <memory>
#include <optional>
#include <vector>

namespace autoware::path_optimizer
{

class MPTOptimizer
{
public:
  explicit MPTOptimizer(
    const MPTParam & param,
    const VehicleInfo & vehicle_info);

  ~MPTOptimizer();

  // Main optimization function
  std::optional<std::vector<TrajectoryPoint>> optimize(
    const std::vector<TrajectoryPoint> & traj_points,
    const std::vector<Point> & left_bound,
    const std::vector<Point> & right_bound,
    const Pose & ego_pose,
    const double ego_velocity);

  // Get reference points (for debugging)
  const std::vector<ReferencePoint> & getReferencePoints() const {
    return ref_points_;
  }

private:
  MPTParam param_;
  VehicleInfo vehicle_info_;
  
  std::vector<ReferencePoint> ref_points_;
  std::unique_ptr<StateEquationGenerator> state_equation_generator_;
  
  // ⭐ Warm start mechanism (ROS2 compatibility)
  std::vector<double> prev_optimized_solution_;  // Previous OSQP solution (U vector)
  std::vector<ReferencePoint> prev_ref_points_;  // Previous reference points for fixed point
  bool has_prev_solution_{false};
  
  // Helper functions
  std::vector<ReferencePoint> generateReferencePoints(
    const std::vector<TrajectoryPoint> & traj_points) const;
    
  void updateFixedPoint(
    std::vector<ReferencePoint> & ref_points);
    
  std::vector<Bounds> calculateBounds(
    const std::vector<ReferencePoint> & ref_points,
    const std::vector<Point> & left_bound,
    const std::vector<Point> & right_bound) const;
    
  bool solveQP(
    std::vector<ReferencePoint> & ref_points,
    const std::vector<Bounds> & bounds,
    const KinematicState & ego_state);
    
  std::vector<TrajectoryPoint> convertToTrajectory(
    const std::vector<ReferencePoint> & ref_points) const;
};

}  // namespace autoware::path_optimizer

#endif  // PATH_OPTIMIZER__MPT_OPTIMIZER_HPP_
