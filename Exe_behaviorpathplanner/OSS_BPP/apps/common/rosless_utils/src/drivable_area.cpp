#include "rosless/drivable_area.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

namespace roless
{
namespace
{
double distanceXY(const Point & a, const Point & b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return std::hypot(dx, dy);
}

std::vector<double> cumulativeLength(const Path & path)
{
  std::vector<double> s;
  s.reserve(path.size());
  double accum = 0.0;
  for (size_t i = 0; i < path.size(); ++i) {
    if (i > 0) {
      accum += distanceXY(path[i - 1].position, path[i].position);
    }
    s.push_back(accum);
  }
  return s;
}

Path extendPath(const Path & path, double backward, double forward)
{
  if (path.size() < 2) {
    return path;
  }

  Path extended;
  extended.reserve(path.size() + 2);

  const auto & first = path.front();
  const auto & second = path[1];
  const double dx0 = second.position.x - first.position.x;
  const double dy0 = second.position.y - first.position.y;
  const double len0 = std::hypot(dx0, dy0);
  PathPoint start = first;
  if (len0 > 1e-6) {
    start.position.x -= dx0 / len0 * backward;
    start.position.y -= dy0 / len0 * backward;
  }
  extended.push_back(start);
  extended.insert(extended.end(), path.begin(), path.end());

  const auto & last = path.back();
  const auto & prev = path[path.size() - 2];
  const double dx1 = last.position.x - prev.position.x;
  const double dy1 = last.position.y - prev.position.y;
  const double len1 = std::hypot(dx1, dy1);
  PathPoint end = last;
  if (len1 > 1e-6) {
    end.position.x += dx1 / len1 * forward;
    end.position.y += dy1 / len1 * forward;
  }
  extended.push_back(end);
  return extended;
}

Path resamplePath(const Path & path, double interval)
{
  if (path.size() < 2 || interval <= 0.0) {
    return path;
  }

  const auto arc = cumulativeLength(path);
  const double total = arc.back();
  const size_t samples = std::max<size_t>(1U, static_cast<size_t>(std::floor(total / interval)));

  Path resampled;
  resampled.reserve(samples + 1);

  auto interpolate = [&](double target_s) {
    if (target_s <= 0.0) {
      return path.front();
    }
    if (target_s >= total) {
      return path.back();
    }
    auto it = std::lower_bound(arc.begin(), arc.end(), target_s);
    const size_t idx = static_cast<size_t>(std::distance(arc.begin(), it));
    if (*it == target_s || idx == 0) {
      return path[idx];
    }
    const size_t prev_idx = idx - 1;
    const double seg_len = arc[idx] - arc[prev_idx];
    const double ratio = seg_len < 1e-6 ? 0.0 : (target_s - arc[prev_idx]) / seg_len;
    PathPoint out;
    out.position.x =
      path[prev_idx].position.x + (path[idx].position.x - path[prev_idx].position.x) * ratio;
    out.position.y =
      path[prev_idx].position.y + (path[idx].position.y - path[prev_idx].position.y) * ratio;
    out.position.z =
      path[prev_idx].position.z + (path[idx].position.z - path[prev_idx].position.z) * ratio;
    out.yaw = path[prev_idx].yaw + (path[idx].yaw - path[prev_idx].yaw) * ratio;
    return out;
  };

  for (size_t i = 0; i <= samples; ++i) {
    const double s = std::min(total, interval * static_cast<double>(i));
    resampled.push_back(interpolate(s));
  }
  return resampled;
}

Point offsetPoint(const PathPoint & base, double lateral)
{
  const double c = std::cos(base.yaw);
  const double s = std::sin(base.yaw);
  Point out = base.position;
  out.x -= s * lateral;
  out.y += c * lateral;
  return out;
}

void removeDuplicates(std::vector<Point> & bound)
{
  if (bound.empty()) {
    return;
  }
  constexpr double thresh2 = 1e-4;
  std::vector<Point> filtered;
  filtered.reserve(bound.size());
  filtered.push_back(bound.front());
  for (size_t i = 1; i < bound.size(); ++i) {
    const double dx = bound[i].x - filtered.back().x;
    const double dy = bound[i].y - filtered.back().y;
    if (dx * dx + dy * dy > thresh2) {
      filtered.push_back(bound[i]);
    }
  }
  bound.swap(filtered);
}

void smoothZ(std::vector<Point> & bound, double window)
{
  if (bound.size() < 3 || window <= 0.0) {
    return;
  }
  std::vector<Point> original = bound;
  for (size_t i = 0; i < bound.size(); ++i) {
    double weighted = 0.0;
    double weight_sum = 0.0;
    for (size_t j = 0; j < bound.size(); ++j) {
      const double dist = distanceXY(original[i], original[j]);
      if (dist > window) {
        continue;
      }
      const double w = std::max(1e-6, window - dist);
      weighted += w * original[j].z;
      weight_sum += w;
    }
    if (weight_sum > 0.0) {
      bound[i].z = weighted / weight_sum;
    }
  }
}

std::vector<Point> buildPolygon(const std::vector<Point> & left, const std::vector<Point> & right)
{
  std::vector<Point> polygon;
  polygon.reserve(left.size() + right.size() + 1);
  polygon.insert(polygon.end(), left.begin(), left.end());
  for (auto it = right.rbegin(); it != right.rend(); ++it) {
    polygon.push_back(*it);
  }
  if (!polygon.empty()) {
    polygon.push_back(polygon.front());
  }
  return polygon;
}
}  // namespace

DrivableArea generateDrivableArea(const Path & center_line, const DrivableAreaParams & params)
{
  DrivableArea area;
  if (center_line.size() < 2) {
    return area;
  }

  const Path extended = extendPath(center_line, params.backward_extend, params.forward_extend);
  const Path resampled = resamplePath(extended, params.resample_interval);

  area.left_bound.reserve(resampled.size());
  area.right_bound.reserve(resampled.size());
  for (const auto & pt : resampled) {
    area.left_bound.push_back(offsetPoint(pt, params.left_offset));
    area.right_bound.push_back(offsetPoint(pt, -params.right_offset));
  }

  removeDuplicates(area.left_bound);
  removeDuplicates(area.right_bound);
  smoothZ(area.left_bound, params.smoothing_window);
  smoothZ(area.right_bound, params.smoothing_window);
  area.polygon = buildPolygon(area.left_bound, area.right_bound);
  return area;
}
}  // namespace roless

