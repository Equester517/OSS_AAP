#pragma once

#include "common/common_types.hpp"

#include "oss/type/autoware_planning_msgs/msg/impl_type_path.h"
#include "oss/type/nav_msgs/msg/impl_type_odometry.h"

namespace ros_autosar_adapter
{

// Odometry conversions
autoware::common_types::Odometry FromOssOdometry(const oss::type::nav_msgs::msg::Odometry &src);
oss::type::nav_msgs::msg::Odometry ToOssOdometry(const autoware::common_types::Odometry &src);

// Path conversions (for PathOptimizer)
autoware::common_types::PathWithLaneId FromOssPath(const oss::type::autoware_planning_msgs::msg::Path &src);
oss::type::autoware_planning_msgs::msg::Path ToOssPath(const autoware::common_types::PathWithLaneId &src);

} // namespace ros_autosar_adapter
