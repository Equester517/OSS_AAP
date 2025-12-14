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

#ifndef PATH_OPTIMIZER__REPLAN_CHECKER_HPP_
#define PATH_OPTIMIZER__REPLAN_CHECKER_HPP_

#include "path_optimizer_types.hpp"

#include <optional>
#include <vector>

namespace autoware::path_optimizer
{

class ReplanChecker
{
public:
  explicit ReplanChecker(const ReplanCheckerParam & param);

  // Check if replanning is required
  bool isReplanRequired(
    const std::vector<TrajectoryPoint> & current_trajectory,
    const Pose & current_ego_pose,
    const double current_time_sec) const;

  // Update previous data
  void updatePreviousData(
    const std::vector<TrajectoryPoint> & traj_points,
    const Pose & ego_pose,
    const double current_time_sec);
    
  // Reset previous data
  void reset();

private:
  ReplanCheckerParam param_;
  
  // Previous data
  std::optional<std::vector<TrajectoryPoint>> prev_traj_points_{std::nullopt};
  std::optional<Pose> prev_ego_pose_{std::nullopt};
  std::optional<double> prev_replanned_time_sec_{std::nullopt};
  
  // Helper functions
  double calculatePathShapeChange(
    const std::vector<TrajectoryPoint> & traj1,
    const std::vector<TrajectoryPoint> & traj2) const;
    
  double calculateDistance(const Pose & pose1, const Pose & pose2) const;
};

}  // namespace autoware::path_optimizer

#endif  // PATH_OPTIMIZER__REPLAN_CHECKER_HPP_
