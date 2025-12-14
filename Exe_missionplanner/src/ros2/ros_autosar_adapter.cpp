#include "ros_autosar_adapter.hpp"

#include "oss/type/geometry_msgs/msg/impl_type_point.h"
#include "oss/type/geometry_msgs/msg/impl_type_quaternion.h"
#include "oss/type/std_msgs/msg/impl_type_header.h"
#include "oss/type/builtin_interfaces/msg/impl_type_time.h"
#include "oss/type/unique_identifier_msgs/msg/impl_type_uuid.h"
#include "oss/type/autoware_planning_msgs/msg/impl_type_laneletsegment.h"
#include "oss/type/impl_type_laneletsegment_vector.h"
#include "oss/type/autoware_planning_msgs/msg/impl_type_laneletprimitive.h"
#include "oss/type/impl_type_laneletprimitive_vector.h"

#include <algorithm>

namespace ros_autosar_adapter
{

autoware::mission_planner_universe::Pose FromOssPose(const oss::type::geometry_msgs::msg::Pose &src)
{
  autoware::mission_planner_universe::Pose dst;
  dst.position.x = src.position.x;
  dst.position.y = src.position.y;
  dst.position.z = src.position.z;
  dst.orientation.x = src.orientation.x;
  dst.orientation.y = src.orientation.y;
  dst.orientation.z = src.orientation.z;
  dst.orientation.w = src.orientation.w;
  return dst;
}

oss::type::geometry_msgs::msg::Pose ToOssPose(const autoware::mission_planner_universe::Pose &src)
{
  oss::type::geometry_msgs::msg::Pose dst;
  dst.position.x = src.position.x;
  dst.position.y = src.position.y;
  dst.position.z = src.position.z;
  dst.orientation.x = src.orientation.x;
  dst.orientation.y = src.orientation.y;
  dst.orientation.z = src.orientation.z;
  dst.orientation.w = src.orientation.w;
  return dst;
}

// Helper: convert oss::type::string -> std::string
static std::string OssStringToStd(const oss::type::string &s)
{
  return std::string(s.data(), s.size());
}

// Helper: convert std::string -> oss::type::string
static oss::type::string StdToOssString(const std::string &s)
{
  return oss::type::string(s.data(), s.size());
}

autoware::mission_planner_universe::LaneletRoute FromOssLaneletRoute(const oss::type::autoware_planning_msgs::msg::LaneletRoute &src)
{
  autoware::mission_planner_universe::LaneletRoute dst;

  // Header
  dst.header.stamp.sec = static_cast<std::int64_t>(src.header.stamp.sec);
  dst.header.stamp.nsec = static_cast<std::int64_t>(src.header.stamp.nanosec);
  dst.header.frame_id = OssStringToStd(src.header.frame_id);

  // Start / goal poses
  dst.start_pose = FromOssPose(src.start_pose);
  dst.goal_pose = FromOssPose(src.goal_pose);

  // UUID
  {
    const auto &oss_uuid = src.uuid.uuid;
    for (std::size_t i = 0; i < dst.uuid.size(); ++i) {
      dst.uuid[i] = oss_uuid[i];
    }
  }

  dst.allow_modification = src.allow_modification;

  // segments
  dst.segments.clear();
  for (const auto &oss_seg : src.segments) {
    autoware::mission_planner_universe::LaneletSegment seg;
    // primitive_type -> preferred_primitive
    seg.preferred_primitive.id = oss_seg.primitive_type.id;
    seg.preferred_primitive.primitive_type = OssStringToStd(oss_seg.primitive_type.primitive_type);

    // primitives vector
    seg.primitives.clear();
    for (const auto &oss_prim : oss_seg.primitives) {
      autoware::mission_planner_universe::LaneletPrimitive prim;
      prim.id = oss_prim.id;
      prim.primitive_type = OssStringToStd(oss_prim.primitive_type);
      seg.primitives.push_back(std::move(prim));
    }

    dst.segments.push_back(std::move(seg));
  }

  return dst;
}

oss::type::autoware_planning_msgs::msg::LaneletRoute ToOssLaneletRoute(const autoware::mission_planner_universe::LaneletRoute &src)
{
  oss::type::autoware_planning_msgs::msg::LaneletRoute dst;

  // Header
  dst.header.stamp.sec = static_cast<std::int32_t>(src.header.stamp.sec);
  dst.header.stamp.nanosec = static_cast<std::uint32_t>(src.header.stamp.nsec);
  dst.header.frame_id = StdToOssString(src.header.frame_id);

  // Poses
  dst.start_pose = ToOssPose(src.start_pose);
  dst.goal_pose = ToOssPose(src.goal_pose);

  // UUID
  for (std::size_t i = 0; i < src.uuid.size(); ++i) {
    dst.uuid.uuid[i] = src.uuid[i];
  }

  dst.allow_modification = src.allow_modification;

  // segments
  dst.segments.clear();
  for (const auto &seg : src.segments) {
    oss::type::autoware_planning_msgs::msg::LaneletSegment oss_seg;
    // preferred_primitive -> primitive_type
    oss_seg.primitive_type.id = seg.preferred_primitive.id;
    oss_seg.primitive_type.primitive_type = StdToOssString(seg.preferred_primitive.primitive_type);

    // primitives
    oss_seg.primitives.clear();
    for (const auto &prim : seg.primitives) {
      oss::type::autoware_planning_msgs::msg::LaneletPrimitive oss_prim;
      oss_prim.id = prim.id;
      oss_prim.primitive_type = StdToOssString(prim.primitive_type);
      oss_seg.primitives.push_back(std::move(oss_prim));
    }

    dst.segments.push_back(std::move(oss_seg));
  }

  return dst;
}

} // namespace ros_autosar_adapter
