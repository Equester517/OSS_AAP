#pragma once

#include "../../include/ros2/mission_planner_types.hpp"

#include "oss/type/geometry_msgs/msg/impl_type_pose.h"
#include "oss/type/autoware_planning_msgs/msg/impl_type_laneletroute.h"
#include "oss/type/std_msgs/msg/impl_type_header.h"
#include "oss/type/unique_identifier_msgs/msg/impl_type_uuid.h"

#include <vector>
#include <array>
#include <string>

namespace ros_autosar_adapter
{
using ::autoware::mission_planner_universe::Pose;
using ::autoware::mission_planner_universe::LaneletRoute;

// Pose conversions
autoware::mission_planner_universe::Pose FromOssPose(const oss::type::geometry_msgs::msg::Pose &src);
oss::type::geometry_msgs::msg::Pose ToOssPose(const autoware::mission_planner_universe::Pose &src);

// LaneletRoute conversions
autoware::mission_planner_universe::LaneletRoute FromOssLaneletRoute(const oss::type::autoware_planning_msgs::msg::LaneletRoute &src);
oss::type::autoware_planning_msgs::msg::LaneletRoute ToOssLaneletRoute(const autoware::mission_planner_universe::LaneletRoute &src);

} // namespace ros_autosar_adapter
