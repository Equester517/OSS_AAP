#pragma once

#include <vector>

namespace roless
{
struct Point
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct PathPoint
{
  Point position;
  double yaw{0.0};
};

using Path = std::vector<PathPoint>;

struct DrivableAreaParams
{
  double left_offset{3.5};
  double right_offset{3.5};
  double backward_extend{5.0};
  double forward_extend{20.0};
  double resample_interval{1.0};
  double smoothing_window{0.0};
};

struct DrivableArea
{
  std::vector<Point> left_bound;
  std::vector<Point> right_bound;
  std::vector<Point> polygon;
};

DrivableArea generateDrivableArea(const Path & center_line, const DrivableAreaParams & params);

}  // namespace roless

