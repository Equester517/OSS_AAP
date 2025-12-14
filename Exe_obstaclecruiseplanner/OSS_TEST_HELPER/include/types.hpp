#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace oss_test_helper
{
struct TimeStamp
{
  std::int64_t sec{0};
  std::int64_t nanosec{0};
};

struct Duration
{
  std::int32_t sec{0};
  std::uint32_t nanosec{0};
};

struct Header
{
  std::uint32_t seq{0};
  TimeStamp stamp{};
  std::string frame_id;
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

struct Pose
{
  Position position;
  Orientation orientation;
};

struct TrajectoryPoint
{
  Duration time_from_start{};
  Pose pose;
  double longitudinal_velocity_mps{0.0};
  double lateral_velocity_mps{0.0};
  double acceleration_mps2{0.0};
  double heading_rate_rps{0.0};
  double front_wheel_angle_rad{0.0};
  double rear_wheel_angle_rad{0.0};
};

struct Trajectory
{
  Header header;
  std::vector<TrajectoryPoint> points;
};
}  // namespace oss_test_helper
