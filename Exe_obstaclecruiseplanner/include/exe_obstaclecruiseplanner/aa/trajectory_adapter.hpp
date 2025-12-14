///////////////////////////////////////////////////////////////////////////////////////////////////////////
/// 
/// Trajectory Adapter: Convert OSS_TEST_HELPER Trajectory to AUTOSAR Trajectory
/// 
///////////////////////////////////////////////////////////////////////////////////////////////////////////
#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <memory>

// Forward declarations - types from oss_test_helper
namespace oss_test_helper
{
struct TimeStamp;
struct Duration;
struct Header;
struct Position;
struct Orientation;
struct Pose;
struct TrajectoryPoint;
struct Trajectory;
}

namespace exe_obstaclecruiseplanner
{
namespace aa
{

class TrajectoryAdapter
{
public:
    /// Convert internal OSS_TEST_HELPER trajectory to AUTOSAR trajectory type
    /// This converts the complete trajectory structure including all header and point data
    static oss::type::autoware_planning_msgs::msg::Trajectory convertTrajectory(
        const oss_test_helper::Trajectory& internal_trajectory);
    
    /// Convert a single trajectory point
    static oss::type::autoware_planning_msgs::msg::TrajectoryPoint convertTrajectoryPoint(
        const oss_test_helper::TrajectoryPoint& internal_point);
    
    /// Convert header information
    static oss::type::std_msgs::msg::Header convertHeader(
        const oss_test_helper::Header& internal_header);
    
    /// Convert pose (position + orientation)
    static oss::type::geometry_msgs::msg::Pose convertPose(
        const oss_test_helper::Pose& internal_pose);
    
    /// Convert position
    static oss::type::geometry_msgs::msg::Point convertPosition(
        const oss_test_helper::Position& internal_position);
    
    /// Convert orientation (quaternion)
    static oss::type::geometry_msgs::msg::Quaternion convertOrientation(
        const oss_test_helper::Orientation& internal_orientation);
    
    /// Convert duration
    static oss::type::builtin_interfaces::msg::Duration convertDuration(
        const oss_test_helper::Duration& internal_duration);
    
    /// Convert timestamp
    static oss::type::builtin_interfaces::msg::Time convertTimestamp(
        const oss_test_helper::TimeStamp& internal_timestamp);

private:
    // Private helper methods can be added here as needed
};

} // namespace aa
} // namespace exe_obstaclecruiseplanner

#endif // EXE_OBSTACLECRUISEPLANNER_AA_TRAJECTORY_ADAPTER_HPP
