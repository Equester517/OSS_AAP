// Thin callable wrapper extracted from previous ros2/main.cpp
#ifndef MISSION_PLANNER_RUNNER_HPP
#define MISSION_PLANNER_RUNNER_HPP

#include <string>
#include <optional>
#include "mission_planner_types.hpp"

namespace mission_planner_runner
{
    // Run the mission planner once using the provided OSM file path.
    // Returns an optional LaneletRoute if a route was created.
    std::optional<autoware::mission_planner_universe::LaneletRoute> run_once(
        const std::string &osm_file,
        const autoware::mission_planner_universe::Pose &start,
        const autoware::mission_planner_universe::Pose &goal);
}

#endif // MISSION_PLANNER_RUNNER_HPP
