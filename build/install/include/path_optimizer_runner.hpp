// Thin callable wrapper extracted from Path_Optimizer_ros2/src/main.cpp
#ifndef PATH_OPTIMIZER_RUNNER_HPP
#define PATH_OPTIMIZER_RUNNER_HPP

#include <vector>
#include "path_optimizer_types.hpp"

namespace path_optimizer_runner
{
    // Run the path optimizer once with in-memory data from AUTOSAR interfaces
    void run_once(
        const std::vector<autoware::path_optimizer::PathPoint>& path_points,
        const autoware::path_optimizer::Pose& ego_pose,
        double ego_velocity,
        const std::vector<autoware::path_optimizer::Point>& left_bound = {},
        const std::vector<autoware::path_optimizer::Point>& right_bound = {});
}

#endif // PATH_OPTIMIZER_RUNNER_HPP
