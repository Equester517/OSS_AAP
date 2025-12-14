#ifndef BEHAVIOR_PATH_PLANNER_RUNNER_HPP
#define BEHAVIOR_PATH_PLANNER_RUNNER_HPP

#include "common/common_types.hpp"
#include <optional>
#include <string>

namespace behavior_path_planner_runner
{

/**
 * @brief Run BehaviorPathPlanner once with given route and odometry
 * @param osm_path Path to OSM map file
 * @param route Route message from MissionPlanner
 * @param odometry Current vehicle odometry state
 * @return PathWithLaneId result, or nullopt on failure
 */
std::optional<autoware::common_types::PathWithLaneId> run_once(
    const std::string& osm_path,
    const autoware::common_types::LaneletRoute& route,
    const autoware::common_types::Odometry& odometry);

} // namespace behavior_path_planner_runner

#endif // BEHAVIOR_PATH_PLANNER_RUNNER_HPP
