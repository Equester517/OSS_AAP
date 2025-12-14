#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace autoware::common_types
{
// --------------------------------------------------------------------------
// 1) Time / Header
// --------------------------------------------------------------------------
struct TimeStamp
{
  std::int64_t sec{0};
  std::int64_t nsec{0};
};

struct Header
{
  std::uint32_t seq{0};
  TimeStamp stamp{};
  std::string frame_id;
};

// --------------------------------------------------------------------------
// 2) 기본 공간/자세 타입
// --------------------------------------------------------------------------
struct Point
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct Position
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct Orientation
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double w{1.0};
};

using Quaternion = Orientation;

struct Pose
{
  Position position;
  Orientation orientation;
};

struct PointXYZ
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

// --------------------------------------------------------------------------
// 3) 벡터/속도 관련
// --------------------------------------------------------------------------
struct Vector3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct Twist
{
  Vector3 linear;
  Vector3 angular;
};

// --------------------------------------------------------------------------
// 4) Path 계열 타입
// --------------------------------------------------------------------------
struct PathPoint
{
  Pose pose;
  double longitudinal_velocity_mps{0.0};
  double lateral_velocity_mps{0.0};
  double heading_rate_rps{0.0};
  bool is_final{false};
};

struct PathPointWithLaneId
{
  PathPoint point;
  std::vector<std::int64_t> lane_ids;
};

struct PathWithLaneId
{
  Header header;
  std::vector<PathPointWithLaneId> points;
  std::vector<PointXYZ> left_bound;
  std::vector<PointXYZ> right_bound;
};

struct Path
{
  Header header;
  std::vector<PathPoint> points;
  std::vector<PointXYZ> left_bound;
  std::vector<PointXYZ> right_bound;
};

// --------------------------------------------------------------------------
// 5) Covariance 포함 타입
// --------------------------------------------------------------------------
struct PoseWithCovariance
{
  Pose pose;
  std::array<double, 36> covariance{}; // 6x6 matrix
};

struct TwistWithCovariance
{
  Vector3 linear;                 // m/s
  Vector3 angular;                // rad/s
  std::array<double, 36> covariance{}; // 6x6 matrix
};

// --------------------------------------------------------------------------
// 6) Odometry
// --------------------------------------------------------------------------
struct Odometry
{
  Header header;
  std::string child_frame_id;
  PoseWithCovariance pose;
  TwistWithCovariance twist;
};

}  // namespace autoware::common_types
