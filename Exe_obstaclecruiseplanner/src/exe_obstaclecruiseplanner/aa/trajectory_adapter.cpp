///////////////////////////////////////////////////////////////////////////////////////////////////////////
///
/// Trajectory Adapter Implementation
/// Convert OSS_TEST_HELPER Trajectory to AUTOSAR Trajectory
///
///////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "exe_obstaclecruiseplanner/aa/trajectory_adapter.hpp"

// OSS_TEST_HELPER types
#include "oss_test_helper/types.hpp"
#include "oss_test_helper/jsonl_reader.hpp"

// AUTOSAR generated types
#include "oss/type/autoware_planning_msgs/msg/impl_type_trajectory.h"
#include "oss/type/autoware_planning_msgs/msg/impl_type_trajectorypoint.h"
#include "oss/type/std_msgs/msg/impl_type_header.h"
#include "oss/type/geometry_msgs/msg/impl_type_pose.h"
#include "oss/type/geometry_msgs/msg/impl_type_point.h"
#include "oss/type/geometry_msgs/msg/impl_type_quaternion.h"
#include "oss/type/builtin_interfaces/msg/impl_type_duration.h"
#include "oss/type/builtin_interfaces/msg/impl_type_time.h"

namespace exe_obstaclecruiseplanner
{
namespace aa
{

oss::type::autoware_planning_msgs::msg::Trajectory TrajectoryAdapter::convertTrajectory(
    const oss_test_helper::Trajectory& internal_trajectory)
{
    oss::type::autoware_planning_msgs::msg::Trajectory autosar_trajectory;
    
    // Convert header
    autosar_trajectory.header = convertHeader(internal_trajectory.header);
    
    // Convert all trajectory points
    autosar_trajectory.points.reserve(internal_trajectory.points.size());
    for (const auto& internal_point : internal_trajectory.points) {
        autosar_trajectory.points.push_back(convertTrajectoryPoint(internal_point));
    }
    
    return autosar_trajectory;
}

oss::type::autoware_planning_msgs::msg::TrajectoryPoint TrajectoryAdapter::convertTrajectoryPoint(
    const oss_test_helper::TrajectoryPoint& internal_point)
{
    oss::type::autoware_planning_msgs::msg::TrajectoryPoint autosar_point;
    
    // Convert time from start
    autosar_point.time_from_start = convertDuration(internal_point.time_from_start);
    
    // Convert pose
    autosar_point.pose = convertPose(internal_point.pose);
    
    // Copy scalar velocity and acceleration values directly
    autosar_point.longitudinal_velocity_mps = internal_point.longitudinal_velocity_mps;
    autosar_point.lateral_velocity_mps = internal_point.lateral_velocity_mps;
    autosar_point.acceleration_mps2 = internal_point.acceleration_mps2;
    autosar_point.heading_rate_rps = internal_point.heading_rate_rps;
    autosar_point.front_wheel_angle_rad = internal_point.front_wheel_angle_rad;
    autosar_point.rear_wheel_angle_rad = internal_point.rear_wheel_angle_rad;
    
    return autosar_point;
}

oss::type::std_msgs::msg::Header TrajectoryAdapter::convertHeader(
    const oss_test_helper::Header& internal_header)
{
    oss::type::std_msgs::msg::Header autosar_header;
    
    autosar_header.seq = internal_header.seq;
    autosar_header.frame_id = internal_header.frame_id;
    autosar_header.stamp = convertTimestamp(internal_header.stamp);
    
    return autosar_header;
}

oss::type::geometry_msgs::msg::Pose TrajectoryAdapter::convertPose(
    const oss_test_helper::Pose& internal_pose)
{
    oss::type::geometry_msgs::msg::Pose autosar_pose;
    
    autosar_pose.position = convertPosition(internal_pose.position);
    autosar_pose.orientation = convertOrientation(internal_pose.orientation);
    
    return autosar_pose;
}

oss::type::geometry_msgs::msg::Point TrajectoryAdapter::convertPosition(
    const oss_test_helper::Position& internal_position)
{
    oss::type::geometry_msgs::msg::Point autosar_point;
    
    autosar_point.x = internal_position.x;
    autosar_point.y = internal_position.y;
    autosar_point.z = internal_position.z;
    
    return autosar_point;
}

oss::type::geometry_msgs::msg::Quaternion TrajectoryAdapter::convertOrientation(
    const oss_test_helper::Orientation& internal_orientation)
{
    oss::type::geometry_msgs::msg::Quaternion autosar_quaternion;
    
    autosar_quaternion.x = internal_orientation.x;
    autosar_quaternion.y = internal_orientation.y;
    autosar_quaternion.z = internal_orientation.z;
    autosar_quaternion.w = internal_orientation.w;
    
    return autosar_quaternion;
}

oss::type::builtin_interfaces::msg::Duration TrajectoryAdapter::convertDuration(
    const oss_test_helper::Duration& internal_duration)
{
    oss::type::builtin_interfaces::msg::Duration autosar_duration;
    
    autosar_duration.sec = internal_duration.sec;
    autosar_duration.nanosec = internal_duration.nanosec;
    
    return autosar_duration;
}

oss::type::builtin_interfaces::msg::Time TrajectoryAdapter::convertTimestamp(
    const oss_test_helper::TimeStamp& internal_timestamp)
{
    oss::type::builtin_interfaces::msg::Time autosar_time;
    
    autosar_time.sec = internal_timestamp.sec;
    autosar_time.nanosec = internal_timestamp.nanosec;
    
    return autosar_time;
}

} // namespace aa
} // namespace exe_obstaclecruiseplanner
