#pragma once

#include "common/common_types.hpp"

#include "oss/type/autoware_planning_msgs/msg/impl_type_laneletroute.h"
#include "oss/type/nav_msgs/msg/impl_type_odometry.h"
#include "oss/type/tier4_planning_msgs/msg/impl_type_pathwithlaneid.h"

namespace ros_autosar_adapter
{

// LaneletRoute conversions
autoware::common_types::LaneletRoute FromOssLaneletRoute(const oss::type::autoware_planning_msgs::msg::LaneletRoute &src);
oss::type::autoware_planning_msgs::msg::LaneletRoute ToOssLaneletRoute(const autoware::common_types::LaneletRoute &src);

// Odometry conversions
autoware::common_types::Odometry FromOssOdometry(const oss::type::nav_msgs::msg::Odometry &src);
oss::type::nav_msgs::msg::Odometry ToOssOdometry(const autoware::common_types::Odometry &src);

// PathWithLaneId conversions
autoware::common_types::PathWithLaneId FromOssPathWithLaneId(const oss::type::tier4_planning_msgs::msg::PathWithLaneId &src);
oss::type::tier4_planning_msgs::msg::PathWithLaneId ToOssPathWithLaneId(const autoware::common_types::PathWithLaneId &src);

} // namespace ros_autosar_adapter
