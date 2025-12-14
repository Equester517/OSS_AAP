#include "ros2/ros_autosar_adapter.hpp"

#include "oss/type/autoware_planning_msgs/msg/impl_type_pathpoint.h"
#include "oss/type/builtin_interfaces/msg/impl_type_time.h"
#include "oss/type/geometry_msgs/msg/impl_type_point.h"
#include "oss/type/geometry_msgs/msg/impl_type_pose.h"
#include "oss/type/geometry_msgs/msg/impl_type_posewithcovariance.h"
#include "oss/type/geometry_msgs/msg/impl_type_quaternion.h"
#include "oss/type/geometry_msgs/msg/impl_type_twist.h"
#include "oss/type/geometry_msgs/msg/impl_type_twistwithcovariance.h"
#include "oss/type/geometry_msgs/msg/impl_type_vector3.h"
#include "oss/type/impl_type_double_36.h"
#include "oss/type/impl_type_pathpoint_vector.h"
#include "oss/type/impl_type_point_vector.h"
#include "oss/type/std_msgs/msg/impl_type_header.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace ros_autosar_adapter
{
namespace
{
using autoware::common_types::Header;
using autoware::common_types::Odometry;
using autoware::common_types::PathPoint;
using autoware::common_types::PathPointWithLaneId;
using autoware::common_types::PathWithLaneId;
using autoware::common_types::Point;
using autoware::common_types::PointXYZ;
using autoware::common_types::Position;
using autoware::common_types::Pose;
using autoware::common_types::PoseWithCovariance;
using autoware::common_types::Quaternion;
using autoware::common_types::TimeStamp;
using autoware::common_types::Twist;
using autoware::common_types::TwistWithCovariance;
using autoware::common_types::Vector3;

std::string OssStringToStd(const oss::type::string &src)
{
  return std::string(src.c_str(), src.size());
}

oss::type::string StdToOssString(const std::string &src)
{
  return oss::type::string(src.c_str(), src.size());
}

TimeStamp FromOssTime(const oss::type::builtin_interfaces::msg::Time &src)
{
  TimeStamp dst;
  dst.sec = static_cast<std::int64_t>(src.sec);
  dst.nsec = static_cast<std::int64_t>(src.nanosec);
  return dst;
}

oss::type::builtin_interfaces::msg::Time ToOssTime(const TimeStamp &src)
{
  oss::type::builtin_interfaces::msg::Time dst;
  dst.sec = static_cast<std::int32_t>(src.sec);
  dst.nanosec = static_cast<std::uint32_t>(src.nsec);
  return dst;
}

Header FromOssHeader(const oss::type::std_msgs::msg::Header &src)
{
  Header dst;
  dst.seq = 0;
  dst.stamp = FromOssTime(src.stamp);
  dst.frame_id = OssStringToStd(src.frame_id);
  return dst;
}

oss::type::std_msgs::msg::Header ToOssHeader(const Header &src)
{
  oss::type::std_msgs::msg::Header dst;
  dst.stamp = ToOssTime(src.stamp);
  dst.frame_id = StdToOssString(src.frame_id);
  return dst;
}

Position FromOssPoint(const oss::type::geometry_msgs::msg::Point &src)
{
  Position dst;
  dst.x = src.x;
  dst.y = src.y;
  dst.z = src.z;
  return dst;
}

PointXYZ FromOssPointXYZ(const oss::type::geometry_msgs::msg::Point &src)
{
  PointXYZ dst;
  dst.x = src.x;
  dst.y = src.y;
  dst.z = src.z;
  return dst;
}

oss::type::geometry_msgs::msg::Point ToOssPoint(const Position &src)
{
  oss::type::geometry_msgs::msg::Point dst;
  dst.x = src.x;
  dst.y = src.y;
  dst.z = src.z;
  return dst;
}

oss::type::geometry_msgs::msg::Point ToOssPointXYZ(const PointXYZ &src)
{
  oss::type::geometry_msgs::msg::Point dst;
  dst.x = src.x;
  dst.y = src.y;
  dst.z = src.z;
  return dst;
}

Quaternion FromOssQuaternion(const oss::type::geometry_msgs::msg::Quaternion &src)
{
  Quaternion dst;
  dst.x = src.x;
  dst.y = src.y;
  dst.z = src.z;
  dst.w = src.w;
  return dst;
}

oss::type::geometry_msgs::msg::Quaternion ToOssQuaternion(const Quaternion &src)
{
  oss::type::geometry_msgs::msg::Quaternion dst;
  dst.x = src.x;
  dst.y = src.y;
  dst.z = src.z;
  dst.w = src.w;
  return dst;
}

Pose FromOssPose(const oss::type::geometry_msgs::msg::Pose &src)
{
  Pose dst;
  dst.position = FromOssPoint(src.position);
  dst.orientation = FromOssQuaternion(src.orientation);
  return dst;
}

oss::type::geometry_msgs::msg::Pose ToOssPose(const Pose &src)
{
  oss::type::geometry_msgs::msg::Pose dst;
  dst.position = ToOssPoint(src.position);
  dst.orientation = ToOssQuaternion(src.orientation);
  return dst;
}

Vector3 FromOssVector3(const oss::type::geometry_msgs::msg::Vector3 &src)
{
  Vector3 dst;
  dst.x = src.x;
  dst.y = src.y;
  dst.z = src.z;
  return dst;
}

oss::type::geometry_msgs::msg::Vector3 ToOssVector3(const Vector3 &src)
{
  oss::type::geometry_msgs::msg::Vector3 dst;
  dst.x = src.x;
  dst.y = src.y;
  dst.z = src.z;
  return dst;
}

std::array<double, 36> FromOssCovariance(const oss::type::double_36 &src)
{
  std::array<double, 36> dst{};
  std::copy(src.begin(), src.end(), dst.begin());
  return dst;
}

oss::type::double_36 ToOssCovariance(const std::array<double, 36> &src)
{
  oss::type::double_36 dst{};
  std::copy(src.begin(), src.end(), dst.begin());
  return dst;
}

PoseWithCovariance FromOssPoseWithCovariance(const oss::type::geometry_msgs::msg::PoseWithCovariance &src)
{
  PoseWithCovariance dst;
  dst.pose = FromOssPose(src.pose);
  dst.covariance = FromOssCovariance(src.covariance);
  return dst;
}

oss::type::geometry_msgs::msg::PoseWithCovariance ToOssPoseWithCovariance(const PoseWithCovariance &src)
{
  oss::type::geometry_msgs::msg::PoseWithCovariance dst;
  dst.pose = ToOssPose(src.pose);
  dst.covariance = ToOssCovariance(src.covariance);
  return dst;
}

Twist FromOssTwist(const oss::type::geometry_msgs::msg::Twist &src)
{
  Twist dst;
  dst.linear = FromOssVector3(src.linear);
  dst.angular = FromOssVector3(src.angular);
  return dst;
}

oss::type::geometry_msgs::msg::Twist ToOssTwist(const Twist &src)
{
  oss::type::geometry_msgs::msg::Twist dst;
  dst.linear = ToOssVector3(src.linear);
  dst.angular = ToOssVector3(src.angular);
  return dst;
}

TwistWithCovariance FromOssTwistWithCovariance(const oss::type::geometry_msgs::msg::TwistWithCovariance &src)
{
  TwistWithCovariance dst;
  dst.linear = FromOssVector3(src.twist.linear);
  dst.angular = FromOssVector3(src.twist.angular);
  dst.covariance = FromOssCovariance(src.covariance);
  return dst;
}

oss::type::geometry_msgs::msg::TwistWithCovariance ToOssTwistWithCovariance(const TwistWithCovariance &src)
{
  oss::type::geometry_msgs::msg::TwistWithCovariance dst;
  dst.twist.linear = ToOssVector3(src.linear);
  dst.twist.angular = ToOssVector3(src.angular);
  dst.covariance = ToOssCovariance(src.covariance);
  return dst;
}

PathPoint FromOssPathPoint(const oss::type::autoware_planning_msgs::msg::PathPoint &src)
{
  PathPoint dst;
  dst.pose = FromOssPose(src.pose);
  dst.longitudinal_velocity_mps = static_cast<double>(src.longitudinal_velocity_mps);
  dst.lateral_velocity_mps = static_cast<double>(src.lateral_velocity_mps);
  dst.heading_rate_rps = static_cast<double>(src.heading_rate_rps);
  return dst;
}

oss::type::autoware_planning_msgs::msg::PathPoint ToOssPathPoint(const PathPoint &src)
{
  oss::type::autoware_planning_msgs::msg::PathPoint dst;
  dst.pose = ToOssPose(src.pose);
  dst.longitudinal_velocity_mps = static_cast<float>(src.longitudinal_velocity_mps);
  dst.lateral_velocity_mps = static_cast<float>(src.lateral_velocity_mps);
  dst.heading_rate_rps = static_cast<float>(src.heading_rate_rps);
  return dst;
}

} // namespace

// Odometry conversions
Odometry FromOssOdometry(const oss::type::nav_msgs::msg::Odometry &src)
{
  Odometry dst;
  dst.header = FromOssHeader(src.header);
  dst.child_frame_id = OssStringToStd(src.child_frame_id);
  dst.pose = FromOssPoseWithCovariance(src.pose);
  dst.twist = FromOssTwistWithCovariance(src.twist);
  return dst;
}

oss::type::nav_msgs::msg::Odometry ToOssOdometry(const Odometry &src)
{
  oss::type::nav_msgs::msg::Odometry dst;
  dst.header = ToOssHeader(src.header);
  dst.child_frame_id = StdToOssString(src.child_frame_id);
  dst.pose = ToOssPoseWithCovariance(src.pose);
  dst.twist = ToOssTwistWithCovariance(src.twist);
  return dst;
}

// Path conversions (for PathOptimizer)
PathWithLaneId FromOssPath(const oss::type::autoware_planning_msgs::msg::Path &src)
{
  PathWithLaneId dst;
  dst.header = FromOssHeader(src.header);
  
  dst.points.clear();
  dst.points.reserve(src.points.size());
  for (const auto &pt : src.points)
  {
    PathPointWithLaneId lane_pt;
    lane_pt.point = FromOssPathPoint(pt);
    lane_pt.lane_ids.clear(); // Path doesn't have lane IDs
    dst.points.push_back(lane_pt);
  }
  
  dst.left_bound.clear();
  dst.left_bound.reserve(src.left_bound.size());
  for (const auto &pt : src.left_bound)
  {
    dst.left_bound.push_back(FromOssPointXYZ(pt));
  }
  
  dst.right_bound.clear();
  dst.right_bound.reserve(src.right_bound.size());
  for (const auto &pt : src.right_bound)
  {
    dst.right_bound.push_back(FromOssPointXYZ(pt));
  }
  
  return dst;
}

oss::type::autoware_planning_msgs::msg::Path ToOssPath(const PathWithLaneId &src)
{
  oss::type::autoware_planning_msgs::msg::Path dst;
  dst.header = ToOssHeader(src.header);
  
  dst.points.clear();
  for (const auto &lane_pt : src.points)
  {
    dst.points.push_back(ToOssPathPoint(lane_pt.point));
  }
  
  dst.left_bound.clear();
  for (const auto &pt : src.left_bound)
  {
    dst.left_bound.push_back(ToOssPointXYZ(pt));
  }
  
  dst.right_bound.clear();
  for (const auto &pt : src.right_bound)
  {
    dst.right_bound.push_back(ToOssPointXYZ(pt));
  }
  
  return dst;
}

} // namespace ros_autosar_adapter
