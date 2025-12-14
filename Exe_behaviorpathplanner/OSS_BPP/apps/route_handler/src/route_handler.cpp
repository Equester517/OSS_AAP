#include <autoware/route_handler/route_handler.hpp>
#include <common/common_types.hpp>

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/primitives/LaneletSequence.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/RoutingGraphContainer.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <boost/geometry.hpp>

#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <iterator>
#include <limits>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace autoware::route_handler
{
namespace
{
using autoware::common_types::LaneletPrimitive;
using autoware::common_types::LaneletRoute;
using autoware::common_types::LaneletSegment;
using autoware::common_types::PathPointWithLaneId;
using autoware::common_types::PathWithLaneId;
using autoware::common_types::Point;
using autoware::common_types::Position;
using autoware::common_types::Pose;
using autoware::common_types::Quaternion;
using autoware::common_types::UUID;

// ======================
//  Geometry helpers
// ======================
Quaternion createQuaternionFromYaw(double yaw)
{
  Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

double normalizeRadian(double a)
{
  constexpr double pi = 3.14159265358979323846;
  while (a > pi) {
    a -= 2.0 * pi;
  }
  while (a < -pi) {
    a += 2.0 * pi;
  }
  return a;
}

double calcDistance2d(
  const Point & p1, const Point & p2)
{
  const double dx = p1.x - p2.x;
  const double dy = p1.y - p2.y;
  return std::sqrt(dx * dx + dy * dy);
}

double calcDistance3d(const Point & p1, const Point & p2)
{
  const double dx = p1.x - p2.x;
  const double dy = p1.y - p2.y;
  const double dz = p1.z - p2.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

Point toPoint(const Position & pos)
{
  Point p;
  p.x = pos.x;
  p.y = pos.y;
  p.z = pos.z;
  return p;
}

Position toPosition(const Point & point)
{
  Position pos;
  pos.x = point.x;
  pos.y = point.y;
  pos.z = point.z;
  return pos;
}

double calcDistance3d(const Position & p1, const Position & p2)
{
  return calcDistance3d(toPoint(p1), toPoint(p2));
}

double calcDistance3d(const Position & p1, const Point & p2)
{
  return calcDistance3d(toPoint(p1), p2);
}

double calcDistance3d(const Point & p1, const Position & p2)
{
  return calcDistance3d(p1, toPoint(p2));
}

double calcDistance2d(const lanelet::ConstPoint3d & lanelet_point, const Point & point)
{
  const double dx = lanelet_point.x() - point.x;
  const double dy = lanelet_point.y() - point.y;
  return std::sqrt(dx * dx + dy * dy);
}

double calcDistancePointToSegment2d(
  const Point & point, const lanelet::ConstPoint3d & segment_start,
  const lanelet::ConstPoint3d & segment_end)
{
  const double sx = segment_start.x();
  const double sy = segment_start.y();
  const double ex = segment_end.x();
  const double ey = segment_end.y();

  const double seg_dx = ex - sx;
  const double seg_dy = ey - sy;
  const double point_dx = point.x - sx;
  const double point_dy = point.y - sy;

  const double seg_length_sq = seg_dx * seg_dx + seg_dy * seg_dy;
  double ratio = 0.0;
  if (seg_length_sq > std::numeric_limits<double>::epsilon()) {
    ratio = (point_dx * seg_dx + point_dy * seg_dy) / seg_length_sq;
    ratio = std::clamp(ratio, 0.0, 1.0);
  }

  const double closest_x = sx + ratio * seg_dx;
  const double closest_y = sy + ratio * seg_dy;
  const double dx = point.x - closest_x;
  const double dy = point.y - closest_y;

  return std::sqrt(dx * dx + dy * dy);
}

double calcDistanceToLaneletCenterline2d(
  const lanelet::ConstLanelet & lanelet, const Point & point)
{
  const auto & centerline = lanelet.centerline();
  if (centerline.empty()) {
    return std::numeric_limits<double>::max();
  }

  if (centerline.size() == 1) {
    return calcDistance2d(centerline.front(), point);
  }

  double min_distance = std::numeric_limits<double>::max();
  for (size_t i = 0; i + 1 < centerline.size(); ++i) {
    const double dist = calcDistancePointToSegment2d(point, centerline[i], centerline[i + 1]);
    if (dist < min_distance) {
      min_distance = dist;
    }
  }

  return min_distance;
}

double getCenterlineLength2d(const lanelet::ConstLanelet & lanelet)
{
  const auto & centerline = lanelet.centerline();
  if (centerline.size() < 2) {
    return 0.0;
  }

  double length = 0.0;
  auto prev_it = centerline.begin();
  auto it = std::next(prev_it);
  for (; it != centerline.end(); ++it, ++prev_it) {
    const double dx = it->x() - prev_it->x();
    const double dy = it->y() - prev_it->y();
    length += std::hypot(dx, dy);
  }
  return length;
}

bool isSameUuid(const UUID & lhs, const UUID & rhs)
{
  return std::equal(std::begin(lhs.bytes), std::end(lhs.bytes), std::begin(rhs.bytes));
}

Point toPoint(const lanelet::ConstPoint3d & pt)
{
  Point p;
  p.x = pt.x();
  p.y = pt.y();
  p.z = pt.z();
  return p;
}

double calcLaneletYawAtPoint(const lanelet::ConstLanelet & lanelet, const Point & point)
{
  const auto & centerline = lanelet.centerline();
  if (centerline.size() < 2) {
    return 0.0;
  }

  size_t best_idx = 0;
  double min_dist = std::numeric_limits<double>::max();
  for (size_t i = 0; i + 1 < centerline.size(); ++i) {
    const double dist = calcDistancePointToSegment2d(point, centerline[i], centerline[i + 1]);
    if (dist < min_dist) {
      min_dist = dist;
      best_idx = i;
    }
  }

  const auto & start = centerline[best_idx];
  const auto & end = centerline[std::min(best_idx + 1, centerline.size() - 1)];
  return std::atan2(end.y() - start.y(), end.x() - start.x());
}

ReferencePoint makeReferencePoint(const lanelet::ConstPoint3d & pt)
{
  ReferencePoint ref;
  ref.is_waypoint = false;
  ref.point = toPoint(pt);
  return ref;
}

ReferencePoint makeReferencePoint(const Point & pt)
{
  ReferencePoint ref;
  ref.is_waypoint = true;
  ref.point = pt;
  return ref;
}

template <typename T>
bool isIndexWithinVector(const std::vector<T> & vec, const int index)
{
  return 0 <= index && index < static_cast<int>(vec.size());
}

template <typename T>
void removeIndicesFromVector(std::vector<T> & vec, std::vector<size_t> indices)
{
  std::sort(indices.begin(), indices.end(), std::greater<size_t>());
  for (const auto idx : indices) {
    if (idx < vec.size()) {
      vec.erase(vec.begin() + idx);
    }
  }
}

bool isClose(const Point & p1, const Point & p2, const double epsilon)
{
  return std::abs(p1.x - p2.x) < epsilon && std::abs(p1.y - p2.y) < epsilon;
}

PiecewiseReferencePoints convertWaypointsToReferencePoints(
  const std::vector<Point> & piecewise_waypoints)
{
  PiecewiseReferencePoints refs;
  for (const auto & waypoint : piecewise_waypoints) {
    refs.push_back(makeReferencePoint(waypoint));
  }
  return refs;
}

lanelet::ArcCoordinates calcArcCoordinates(
  const lanelet::ConstLanelet & lanelet, const Point & point)
{
  const auto center2d = lanelet::utils::to2D(lanelet.centerline());
  lanelet::BasicPoint2d target{point.x, point.y};
  return lanelet::geometry::toArcCoordinates(center2d, target);
}

}  // namespace


Header RouteHandler::getRouteHeader() const
{
  if (!route_ptr_) {
    //RCLCPP_WARN(logger_, "[Route Handler] getRouteHeader: Route has not been set yet");
    return Header();
  }
  return route_ptr_->header;
}

RouteHandler::RouteHandler() = default;

RouteHandler::RouteHandler(
  const LaneletMapPtr & lanelet_map, const RoutingGraphPtr & routing_graph)
: lanelet_map_ptr_(lanelet_map), routing_graph_ptr_(routing_graph)
{
  traffic_rules_ptr_ = lanelet::traffic_rules::TrafficRulesFactory::create(
    lanelet::Locations::Germany,
    lanelet::Participants::Vehicle
  );

  if (!routing_graph_ptr_) {
  routing_graph_ptr_ = lanelet::routing::RoutingGraph::build(
    *lanelet_map_ptr_, *traffic_rules_ptr_);
  }
  
}
/*
void RouteHandler::setMap(
  const LaneletMapPtr & lanelet_map, const RoutingGraphPtr & routing_graph)
{
  lanelet_map_ptr_ = lanelet_map;
  routing_graph_ptr_ = routing_graph;

  if (lanelet_map_ptr_) {
    // TrafficRulesFactory 인스턴스 생성 X
    traffic_rules_ptr_ = lanelet::traffic_rules::TrafficRulesFactory::create(
      lanelet::Locations::Germany,
      lanelet::Participants::Vehicle
    );

    // routing_graph 이 안 들어왔으면 여기서 생성해줌
    if (!routing_graph_ptr_ && traffic_rules_ptr_) {
      routing_graph_ptr_ = lanelet::routing::RoutingGraph::build(
        *lanelet_map_ptr_, *traffic_rules_ptr_);
    }
  }

  is_handler_ready_ = static_cast<bool>(lanelet_map_ptr_ && routing_graph_ptr_);
}
*/
void RouteHandler::setRoute(const LaneletRoute & route_msg)
{
  if (!lanelet_map_ptr_) {
    std::cerr << "[RouteHandler] Cannot set route: lanelet map is not set.\n";
    is_handler_ready_ = false;
    return;
  }

  if (isRouteLooped(route_msg.segments)) {
    std::cerr << "[RouteHandler] Loop detected within route. Previous route remains active.\n";
    return;
  }

  if (!route_ptr_ || !isSameUuid(route_ptr_->uuid, route_msg.uuid)) {
    original_start_pose_ = route_msg.start_pose;
    original_goal_pose_ = route_msg.goal_pose;
  }

  route_ptr_ = std::make_shared<LaneletRoute>(route_msg);
  setLaneletsFromRouteMsg();
}

bool RouteHandler::isRouteLooped(const RouteSections & route_sections) const
{
  std::unordered_set<lanelet::Id> visited_lanelets;
  for (const auto & section : route_sections) {
    for (const auto & primitive : section.primitives) {
      const auto lanelet_id = static_cast<lanelet::Id>(primitive.id);
      if (!visited_lanelets.insert(lanelet_id).second) {
        return true;
      }
    }
  }
  return false;
}

void RouteHandler::setLaneletsFromRouteMsg()
{
  route_lanelets_.clear();
  preferred_lanelets_.clear();
  start_lanelets_.clear();
  goal_lanelets_.clear();

  if (!route_ptr_ || !lanelet_map_ptr_) {
    is_handler_ready_ = false;
    return;
  }

  const auto & segments = route_ptr_->segments;
  if (segments.empty()) {
    is_handler_ready_ = false;
    return;
  }

  const auto segment_count = segments.size();
  for (size_t segment_index = 0; segment_index < segment_count; ++segment_index) {
    const auto & segment = segments.at(segment_index);
    for (const auto & primitive : segment.primitives) {
      const auto lanelet_id = static_cast<lanelet::Id>(primitive.id);
      if (!lanelet_map_ptr_->laneletLayer.exists(lanelet_id)) {
        std::cerr << "[RouteHandler] lanelet id " << lanelet_id << " not found in map.\n";
        continue;
      }
      const auto & lanelet = lanelet_map_ptr_->laneletLayer.get(lanelet_id);
      route_lanelets_.push_back(lanelet);
      if (primitive.id == segment.preferred_primitive.id) {
        preferred_lanelets_.push_back(lanelet);
      }
      if (segment_index == 0) {
        start_lanelets_.push_back(lanelet);
      }
      if (segment_index + 1 == segment_count) {
        goal_lanelets_.push_back(lanelet);
      }
    }
  }

  is_handler_ready_ = !route_lanelets_.empty();
}

void RouteHandler::clearRoute()
{
  route_lanelets_.clear();
  preferred_lanelets_.clear();
  start_lanelets_.clear();
  goal_lanelets_.clear();
  route_ptr_.reset();
  is_handler_ready_ = false;
}

Pose RouteHandler::getGoalPose() const
{
  if (route_ptr_) return route_ptr_->goal_pose;
  return Pose{};
}

Pose RouteHandler::getStartPose() const
{
  if (route_ptr_) return route_ptr_->start_pose;
  return Pose{};
}

Pose RouteHandler::getOriginalGoalPose() const { return original_goal_pose_; }
Pose RouteHandler::getOriginalStartPose() const { return original_start_pose_; }


lanelet::ConstPoint3d get3DPointFrom2DArcLength(
  const lanelet::ConstLanelets & lanelet_sequence, const double s)
{
  using lanelet::utils::to2D;
  double accumulated = 0.0;
  for (const auto & lanelet : lanelet_sequence) {
    const auto & centerline = lanelet.centerline();
    if (centerline.empty()) {
      continue;
    }
    auto prev_pt = centerline.front();
    for (const auto & pt : centerline) {
      const double distance =
        lanelet::geometry::distance2d(to2D(prev_pt), to2D(pt));
      if (accumulated + distance > s) {
        const double ratio = (s - accumulated) / distance;
        const auto interpolated = prev_pt.basicPoint() * (1 - ratio) + pt.basicPoint() * ratio;
        return lanelet::ConstPoint3d{
          lanelet::InvalId, interpolated.x(), interpolated.y(), interpolated.z()};
      }
      accumulated += distance;
      prev_pt = pt;
    }
  }
  return lanelet::ConstPoint3d{};
}

PathWithLaneId removeOverlappingPoints(const PathWithLaneId & input_path)
{
  PathWithLaneId filtered;
  filtered.left_bound = input_path.left_bound;
  filtered.right_bound = input_path.right_bound;
  filtered.header = input_path.header;

  for (const auto & pt : input_path.points) {
    if (filtered.points.empty()) {
      filtered.points.push_back(pt);
      continue;
    }
    constexpr double min_dist = 1.0e-3;
    auto & back = filtered.points.back();
    if (calcDistance3d(back.point.pose.position, pt.point.pose.position) < min_dist) {
      back.lane_ids.insert(back.lane_ids.end(), pt.lane_ids.begin(), pt.lane_ids.end());
      back.point.longitudinal_velocity_mps =
        std::min(back.point.longitudinal_velocity_mps, pt.point.longitudinal_velocity_mps);
    } else {
      filtered.points.push_back(pt);
    }
  }
  return filtered;
}

std::vector<Waypoints> RouteHandler::calcWaypointsVector(
  const lanelet::ConstLanelets & lanelet_sequence) const
{
  std::vector<Waypoints> waypoints_vec;
  if (!lanelet_map_ptr_) {
    return waypoints_vec;
  }

  for (const auto & lanelet : lanelet_sequence) {
    if (!lanelet.hasAttribute("waypoints")) {
      continue;
    }
    const auto attr = lanelet.attribute("waypoints").asId();
    if (!attr) {
      continue;
    }

    PiecewiseWaypoints piecewise_waypoints;
    piecewise_waypoints.lanelet_id = lanelet.id();
    const auto & ls = lanelet_map_ptr_->lineStringLayer.get(attr.value());
    for (const auto & waypoint : ls) {
      piecewise_waypoints.piecewise_waypoints.push_back(toPoint(waypoint));
    }
    if (piecewise_waypoints.piecewise_waypoints.empty()) {
      continue;
    }

    if (
      !waypoints_vec.empty() &&
      isClose(
        waypoints_vec.back().back().piecewise_waypoints.back(),
        piecewise_waypoints.piecewise_waypoints.front(), 1.0)) {
      waypoints_vec.back().push_back(piecewise_waypoints);
    } else {
      Waypoints new_waypoints;
      new_waypoints.push_back(piecewise_waypoints);
      waypoints_vec.push_back(new_waypoints);
    }
  }

  return waypoints_vec;
}

void RouteHandler::removeOverlappedCenterlineWithWaypoints(
  std::vector<PiecewiseReferencePoints> & piecewise_ref_points_vec,
  const std::vector<Point> & piecewise_waypoints,
  const lanelet::ConstLanelets & lanelet_sequence,
  const size_t piecewise_waypoints_lanelet_sequence_index,
  const bool is_removing_direction_forward) const
{
  const double margin_ratio = 10.0;
  const auto & target_lanelet = lanelet_sequence.at(piecewise_waypoints_lanelet_sequence_index);
  const auto front_arc = calcArcCoordinates(target_lanelet, piecewise_waypoints.front());
  const auto back_arc = calcArcCoordinates(target_lanelet, piecewise_waypoints.back());
  const double lanelet_length =
    lanelet::geometry::length(target_lanelet.centerline().basicLineString());

  const double front_threshold =
    -lanelet_length + front_arc.length - std::abs(front_arc.distance) * margin_ratio;
  const double back_threshold =
    back_arc.length + std::abs(back_arc.distance) * margin_ratio;

  double offset_arc_length = 0.0;
  int target_index = static_cast<int>(piecewise_waypoints_lanelet_sequence_index);
  while (isIndexWithinVector(lanelet_sequence, target_index)) {
    auto & target_points = piecewise_ref_points_vec.at(target_index);
    const auto & lanelet = lanelet_sequence.at(target_index);
    const double lanelet_arc_length =
      lanelet::geometry::length(lanelet.centerline().basicLineString());

    std::vector<size_t> overlapped_indices;
    bool finished = false;
    for (size_t i = 0; i < target_points.size(); ++i) {
      const size_t idx =
        is_removing_direction_forward ? i : target_points.size() - 1 - i;
      const auto & ref_point = target_points.at(idx);
      if (ref_point.is_waypoint) {
        if (target_index == static_cast<int>(piecewise_waypoints_lanelet_sequence_index)) {
          overlapped_indices.clear();
        }
        continue;
      }

      const double ref_arc =
        (is_removing_direction_forward ? 0.0 : -lanelet_arc_length) +
        calcArcCoordinates(lanelet, ref_point.point).length;

      if (is_removing_direction_forward) {
        if (back_threshold < offset_arc_length + ref_arc) {
          finished = true;
          break;
        }
      } else if (offset_arc_length + ref_arc < front_threshold) {
        finished = true;
        break;
      }

      overlapped_indices.push_back(idx);
    }

    removeIndicesFromVector(target_points, overlapped_indices);
    if (finished) {
      break;
    }

    target_index += is_removing_direction_forward ? 1 : -1;
    offset_arc_length = (is_removing_direction_forward ? 1.0 : -1.0) * lanelet_arc_length;
  }
}

PathWithLaneId RouteHandler::getCenterLinePath(
  const lanelet::ConstLanelets & lanelet_sequence) const
{
  return getCenterLinePath(
    lanelet_sequence, 0.0, std::numeric_limits<double>::max(), true);
}

PathWithLaneId RouteHandler::getCenterLinePath(
  const lanelet::ConstLanelets & lanelet_sequence, const double s_start, const double s_end,
  bool use_exact) const
{
  using lanelet::utils::to2D;
  PathWithLaneId reference_path{};
  if (lanelet_sequence.empty()) {
    return reference_path;
  }

  std::vector<PiecewiseReferencePoints> piecewise_ref_points_vec;
  piecewise_ref_points_vec.reserve(lanelet_sequence.size());

  double s = 0.0;
  for (const auto & lanelet : lanelet_sequence) {
    piecewise_ref_points_vec.emplace_back();
    const auto & centerline = lanelet.centerline();
    if (centerline.empty()) {
      continue;
    }

    for (size_t i = 0; i < centerline.size(); ++i) {
      const auto & pt = centerline[i];
      const auto next_pt = (i + 1 < centerline.size()) ? centerline[i + 1] : centerline[i];
      const double distance = lanelet::geometry::distance2d(to2D(pt), to2D(next_pt));

      if (s < s_start && s + distance > s_start) {
        const auto p = use_exact ? get3DPointFrom2DArcLength(lanelet_sequence, s_start) : pt;
        piecewise_ref_points_vec.back().push_back(makeReferencePoint(p));
      }
      if (s >= s_start && s <= s_end) {
        piecewise_ref_points_vec.back().push_back(makeReferencePoint(pt));
      }
      if (s < s_end && s + distance > s_end) {
        const auto p = use_exact ? get3DPointFrom2DArcLength(lanelet_sequence, s_end) : next_pt;
        piecewise_ref_points_vec.back().push_back(makeReferencePoint(p));
      }
      s += distance;
    }
  }

  const auto waypoints_vec = calcWaypointsVector(lanelet_sequence);
  for (const auto & waypoints : waypoints_vec) {
    for (auto it = waypoints.begin(); it != waypoints.end(); ++it) {
      const auto & piecewise_waypoints = it->piecewise_waypoints;
      const auto lanelet_id = it->lanelet_id;
      const auto lanelet_it = std::find_if(
        lanelet_sequence.begin(), lanelet_sequence.end(),
        [&](const auto & lanelet) { return lanelet.id() == lanelet_id; });
      if (lanelet_it == lanelet_sequence.end()) {
        continue;
      }
      const size_t lanelet_index = std::distance(lanelet_sequence.begin(), lanelet_it);
      const auto ref_points_by_waypoints = convertWaypointsToReferencePoints(piecewise_waypoints);

      const bool is_first = it == waypoints.begin();
      const bool is_last = it == waypoints.end() - 1;
      if (is_first || is_last) {
        const auto original_points = piecewise_ref_points_vec.at(lanelet_index);
        auto & current_points = piecewise_ref_points_vec.at(lanelet_index);
        current_points = ref_points_by_waypoints;
        if (is_first) {
          current_points.insert(current_points.begin(), original_points.begin(), original_points.end());
          removeOverlappedCenterlineWithWaypoints(
            piecewise_ref_points_vec, piecewise_waypoints, lanelet_sequence, lanelet_index, false);
        }
        if (is_last) {
          current_points.insert(current_points.end(), original_points.begin(), original_points.end());
          removeOverlappedCenterlineWithWaypoints(
            piecewise_ref_points_vec, piecewise_waypoints, lanelet_sequence, lanelet_index, true);
        }
      } else {
        piecewise_ref_points_vec.at(lanelet_index) = ref_points_by_waypoints;
      }
    }
  }

  for (size_t lanelet_idx = 0; lanelet_idx < lanelet_sequence.size(); ++lanelet_idx) {
    const auto & lanelet = lanelet_sequence.at(lanelet_idx);
    const float speed_limit =
      traffic_rules_ptr_
        ? static_cast<float>(traffic_rules_ptr_->speedLimit(lanelet).speedLimit.value())
        : 0.0F;
    const auto & ref_points = piecewise_ref_points_vec.at(lanelet_idx);
    for (const auto & ref_point : ref_points) {
      PathPointWithLaneId path_point{};
      path_point.point.pose.position = toPosition(ref_point.point);
      path_point.point.longitudinal_velocity_mps = speed_limit;
      path_point.lane_ids.push_back(lanelet.id());
      reference_path.points.push_back(path_point);
    }
  }

  reference_path = removeOverlappingPoints(reference_path);

  if (reference_path.points.size() == 1 && lanelet_map_ptr_) {
    const auto lane_id = reference_path.points.front().lane_ids.front();
    const auto lanelet = lanelet_map_ptr_->laneletLayer.get(lane_id);
    const auto point = reference_path.points.front().point.pose.position;
    const auto yaw = calcLaneletYawAtPoint(lanelet, toPoint(point));
    PathPointWithLaneId extra{};
    extra.lane_ids.push_back(lane_id);
    constexpr double ds = 0.1;
    extra.point.pose.position.x = point.x + ds * std::cos(yaw);
    extra.point.pose.position.y = point.y + ds * std::sin(yaw);
    extra.point.pose.position.z = point.z;
    reference_path.points.push_back(extra);
  }

  for (size_t i = 0; i < reference_path.points.size(); ++i) {
    double yaw = 0.0;
    if (i + 1 < reference_path.points.size()) {
      yaw = std::atan2(
        reference_path.points[i + 1].point.pose.position.y -
          reference_path.points[i].point.pose.position.y,
        reference_path.points[i + 1].point.pose.position.x -
          reference_path.points[i].point.pose.position.x);
    } else if (i > 0) {
      yaw = std::atan2(
        reference_path.points[i].point.pose.position.y -
          reference_path.points[i - 1].point.pose.position.y,
        reference_path.points[i].point.pose.position.x -
          reference_path.points[i - 1].point.pose.position.x);
    }
    reference_path.points[i].point.pose.orientation = createQuaternionFromYaw(yaw);
  }

  return reference_path;
}

bool RouteHandler::getClosestLaneletWithinRoute(
  const Pose & search_pose, lanelet::ConstLanelet * closest_lanelet) const
{
  if (!closest_lanelet || !route_ptr_ || !lanelet_map_ptr_) {
    return false;
  }

  const auto & segments = route_ptr_->segments;
  if (segments.empty()) {
    return false;
  }

  const Point search_point{
    search_pose.position.x, search_pose.position.y, search_pose.position.z};

  std::unordered_set<lanelet::Id> visited_lanelets;
  double min_distance = std::numeric_limits<double>::max();
  lanelet::ConstLanelet best_lanelet;
  bool found_lanelet = false;

  auto consider_lanelet_id = [&](const std::int64_t lanelet_id_raw) {
    const auto lanelet_id = static_cast<lanelet::Id>(lanelet_id_raw);
    if (!lanelet_map_ptr_->laneletLayer.exists(lanelet_id)) {
      return;
    }
    if (!visited_lanelets.insert(lanelet_id).second) {
      return;
    }

    const auto candidate = lanelet_map_ptr_->laneletLayer.get(lanelet_id);
    const double distance = calcDistanceToLaneletCenterline2d(candidate, search_point);
    if (distance < min_distance) {
      min_distance = distance;
      best_lanelet = candidate;
      found_lanelet = true;
    }
  };

  for (const auto & segment : segments) {
    consider_lanelet_id(segment.preferred_primitive.id);
    for (const auto & primitive : segment.primitives) {
      consider_lanelet_id(primitive.id);
    }
  }

  if (!found_lanelet) {
    return false;
  }

  *closest_lanelet = best_lanelet;
  return true;
}

lanelet::ConstLanelet RouteHandler::getLaneletsFromId(const lanelet::Id id) const
{
  return lanelet_map_ptr_->laneletLayer.get(id);
}

bool RouteHandler::isShoulderLanelet(const lanelet::ConstLanelet & lanelet) const
{
  return lanelet.hasAttribute(lanelet::AttributeName::Subtype) &&
         lanelet.attribute(lanelet::AttributeName::Subtype) == "road_shoulder";
}

bool RouteHandler::isRouteLanelet(const lanelet::ConstLanelet & lanelet) const
{
  return isLaneletPartOfRoute(lanelet);
}

lanelet::ConstLanelets RouteHandler::getLaneletSequenceAfter(
  const lanelet::ConstLanelet & lanelet, const double min_length, const bool only_route_lanes) const
{
  lanelet::ConstLanelets lanelet_sequence_forward;
  if (only_route_lanes && !isLaneletPartOfRoute(lanelet)) {
    return lanelet_sequence_forward;
  }
  if (only_route_lanes) {
    const bool is_current_goal = std::any_of(
      goal_lanelets_.begin(), goal_lanelets_.end(),
      [&](const lanelet::ConstLanelet & goal_lanelet) {
        return goal_lanelet.id() == lanelet.id();
      });
    if (is_current_goal) {
      return lanelet_sequence_forward;
    }
  }
  if (!routing_graph_ptr_) {
    return lanelet_sequence_forward;
  }

  double length = 0;
  lanelet::ConstLanelet current_lanelet = lanelet;
  std::unordered_set<lanelet::Id> visited_lanelets;
  visited_lanelets.insert(lanelet.id());
  while (length < min_length) {
    lanelet::ConstLanelet next_lanelet;
    bool found_next = false;
    if (only_route_lanes) {
      found_next = getNextLaneletWithinRoute(current_lanelet, &next_lanelet);
    } else {
      const auto next_lanes = getNextLanelets(current_lanelet);
      if (!next_lanes.empty()) {
        next_lanelet = next_lanes.front();
        found_next = true;
      }
    }
    if (!found_next) {
      break;
    }
    if (!visited_lanelets.insert(next_lanelet.id()).second) {
      break;
    }
    lanelet_sequence_forward.push_back(next_lanelet);
    if (only_route_lanes) {
      const bool is_next_goal = std::any_of(
        goal_lanelets_.begin(), goal_lanelets_.end(),
        [&](const lanelet::ConstLanelet & goal_lanelet) {
          return goal_lanelet.id() == next_lanelet.id();
        });
      if (is_next_goal) {
        break;
      }
    }
    current_lanelet = next_lanelet;
    length += getCenterlineLength2d(next_lanelet);
  }

  return lanelet_sequence_forward;
}
//경량버전
lanelet::ConstLanelets RouteHandler::getLaneletSequence(
  const lanelet::ConstLanelet & lanelet,
  const double backward_distance,
  const double forward_distance,
  const bool only_route_lanes) const
{
  lanelet::ConstLanelets lanelet_sequence;

  // only_route_lanes 인데 현재 lanelet 이 route 에 없으면 빈 시퀀스 반환
  if (only_route_lanes && !isLaneletPartOfRoute(lanelet)) {
    return lanelet_sequence;
  }

  // 전방 방향 lanelet 시퀀스
  lanelet::ConstLanelets lanelet_sequence_forward;
  if (only_route_lanes) {
    lanelet_sequence_forward =
      getLaneletSequenceAfter(lanelet, forward_distance, only_route_lanes);
  } else if (isShoulderLanelet(lanelet)) {
    lanelet_sequence_forward =
      getShoulderLaneletSequenceAfter(lanelet, forward_distance);
  }

  // 후방 방향 lanelet 시퀀스
  lanelet::ConstLanelets lanelet_sequence_backward;

  // 원래 코드는
  //   current_pose.position = centerline().front();
  //   arc_coordinate = getArcCoordinates({lanelet}, current_pose);
  //   if (arc_coordinate.length < backward_distance) ...
  // 였는데, centerline().front() 기준이면 arc length == 0 이라
  // 조건은 사실상 (backward_distance > 0) 과 동일합니다.
  const double arc_length = 0.0;
  if (arc_length < backward_distance) {  // backward_distance > 0 인 경우만 확장
    if (only_route_lanes) {
      lanelet_sequence_backward =
        getLaneletSequenceUpTo(lanelet, backward_distance, only_route_lanes);
    } else if (isShoulderLanelet(lanelet)) {
      lanelet_sequence_backward =
        getShoulderLaneletSequenceUpTo(lanelet, backward_distance);
    }
  }

  // loop check (중복 lanelet 제거)
  if (!lanelet_sequence_forward.empty() && !lanelet_sequence_backward.empty()) {
    if (lanelet_sequence_backward.back().id() == lanelet_sequence_forward.front().id()) {
      // 뒤/앞 시퀀스가 같은 lanelet 에서 만나는 경우, 중복 없이 forward 쪽으로만 반환
      return lanelet_sequence_forward;
    }
  }

  // backward 시퀀스 + 현재 lanelet + forward 시퀀스 이어 붙이기
  lanelet_sequence.insert(
    lanelet_sequence.end(),
    lanelet_sequence_backward.begin(),
    lanelet_sequence_backward.end());

  lanelet_sequence.push_back(lanelet);

  lanelet_sequence.insert(
    lanelet_sequence.end(),
    lanelet_sequence_forward.begin(),
    lanelet_sequence_forward.end());

  return lanelet_sequence;
}

lanelet::ConstLanelets RouteHandler::getLaneletSequenceUpTo(
  const lanelet::ConstLanelet & lanelet,
  const double min_length,
  const bool only_route_lanes) const
{
  lanelet::ConstLanelets lanelet_sequence_backward;
  if (only_route_lanes && !isLaneletPartOfRoute(lanelet)) {
    return lanelet_sequence_backward;
  }
  if (!routing_graph_ptr_) {
    return lanelet_sequence_backward;
  }

  lanelet::ConstLanelet current_lanelet = lanelet;
  double length = 0.0;
  std::unordered_set<lanelet::Id> visited_lanelets;
  visited_lanelets.insert(lanelet.id());

  while (length < min_length) {
    lanelet::ConstLanelets previous_lanelets;
    bool has_previous = false;
    if (only_route_lanes) {
      has_previous = getPreviousLaneletsWithinRoute(current_lanelet, &previous_lanelets);
    } else {
      previous_lanelets = getPreviousLanelets(current_lanelet);
      has_previous = !previous_lanelets.empty();
    }

    if (!has_previous) {
      break;
    }

    const auto candidate_it = std::find_if(
      previous_lanelets.begin(), previous_lanelets.end(),
      [&](const lanelet::ConstLanelet & candidate) {
        return visited_lanelets.count(candidate.id()) == 0;
      });

    if (candidate_it == previous_lanelets.end()) {
      break;
    }

    lanelet_sequence_backward.insert(lanelet_sequence_backward.begin(), *candidate_it);
    visited_lanelets.insert(candidate_it->id());
    length += getCenterlineLength2d(*candidate_it);
    current_lanelet = *candidate_it;
  }

  return lanelet_sequence_backward;
}

lanelet::ConstLanelets RouteHandler::getShoulderLaneletSequenceAfter(
  const lanelet::ConstLanelet & lanelet,
  const double min_length) const
{
  lanelet::ConstLanelets lanelet_sequence_forward;
  if (!isShoulderLanelet(lanelet) || !lanelet_map_ptr_) {
    return lanelet_sequence_forward;
  }

  double length = 0.0;
  lanelet::ConstLanelet current_lanelet = lanelet;
  std::unordered_set<lanelet::Id> visited_lanelets;
  visited_lanelets.insert(lanelet.id());

  while (length < min_length) {
    const auto next_lanelet_opt = getFollowingShoulderLanelet(current_lanelet);
    if (!next_lanelet_opt) {
      break;
    }
    const auto & next_lanelet = *next_lanelet_opt;
    if (!visited_lanelets.insert(next_lanelet.id()).second) {
      break;
    }
    lanelet_sequence_forward.push_back(next_lanelet);
    current_lanelet = next_lanelet;
    length += getCenterlineLength2d(next_lanelet);
  }

  return lanelet_sequence_forward;
}

lanelet::ConstLanelets RouteHandler::getShoulderLaneletSequenceUpTo(
  const lanelet::ConstLanelet & lanelet,
  const double min_length) const
{
  lanelet::ConstLanelets lanelet_sequence_backward;
  if (!isShoulderLanelet(lanelet) || !lanelet_map_ptr_) {
    return lanelet_sequence_backward;
  }

  double length = 0.0;
  lanelet::ConstLanelet current_lanelet = lanelet;
  std::unordered_set<lanelet::Id> visited_lanelets;
  visited_lanelets.insert(lanelet.id());

  while (length < min_length) {
    const auto prev_lanelet_opt = getPreviousShoulderLanelet(current_lanelet);
    if (!prev_lanelet_opt) {
      break;
    }
    const auto & prev_lanelet = *prev_lanelet_opt;
    if (!visited_lanelets.insert(prev_lanelet.id()).second) {
      break;
    }
    lanelet_sequence_backward.insert(lanelet_sequence_backward.begin(), prev_lanelet);
    current_lanelet = prev_lanelet;
    length += getCenterlineLength2d(prev_lanelet);
  }

  return lanelet_sequence_backward;
}

lanelet::ConstLanelets RouteHandler::getNextLanelets(const lanelet::ConstLanelet & lanelet) const
{
  if (!routing_graph_ptr_) {
    return {};
  }
  return routing_graph_ptr_->following(lanelet);
}

lanelet::ConstLanelets RouteHandler::getPreviousLanelets(const lanelet::ConstLanelet & lanelet) const
{
  if (!routing_graph_ptr_) {
    return {};
  }
  return routing_graph_ptr_->previous(lanelet);
}

bool RouteHandler::getNextLaneletWithinRoute(
  const lanelet::ConstLanelet & lanelet,
  lanelet::ConstLanelet * next_lanelet) const
{
  if (!next_lanelet || !routing_graph_ptr_ || !route_ptr_) {
    return false;
  }

  const auto candidates = getNextLanelets(lanelet);
  for (const auto & candidate : candidates) {
    if (isLaneletPartOfRoute(candidate)) {
      *next_lanelet = candidate;
      return true;
    }
  }

  return false;
}

bool RouteHandler::getPreviousLaneletsWithinRoute(
  const lanelet::ConstLanelet & lanelet,
  lanelet::ConstLanelets * previous_lanelets) const
{
  if (!previous_lanelets || !routing_graph_ptr_ || !route_ptr_) {
    return false;
  }

  previous_lanelets->clear();
  const auto candidates = getPreviousLanelets(lanelet);
  for (const auto & candidate : candidates) {
    if (isLaneletPartOfRoute(candidate)) {
      previous_lanelets->push_back(candidate);
    }
  }

  return !previous_lanelets->empty();
}

std::optional<lanelet::ConstLanelet> RouteHandler::getFollowingShoulderLanelet(
  const lanelet::ConstLanelet & lanelet) const
{
  if (!lanelet_map_ptr_) {
    return std::nullopt;
  }

  for (const auto & candidate : lanelet_map_ptr_->laneletLayer) {
    if (!isShoulderLanelet(candidate)) {
      continue;
    }
    if (candidate.id() == lanelet.id()) {
      continue;
    }
    if (lanelet::geometry::follows(lanelet, candidate)) {
      return candidate;
    }
  }

  return std::nullopt;
}

std::optional<lanelet::ConstLanelet> RouteHandler::getPreviousShoulderLanelet(
  const lanelet::ConstLanelet & lanelet) const
{
  if (!lanelet_map_ptr_) {
    return std::nullopt;
  }

  for (const auto & candidate : lanelet_map_ptr_->laneletLayer) {
    if (!isShoulderLanelet(candidate)) {
      continue;
    }
    if (candidate.id() == lanelet.id()) {
      continue;
    }
    if (lanelet::geometry::follows(candidate, lanelet)) {
      return candidate;
    }
  }

  return std::nullopt;
}

bool RouteHandler::isLaneletPartOfRoute(const lanelet::ConstLanelet & lanelet) const
{
  if (!route_ptr_) {
    return false;
  }

  const auto lanelet_id = static_cast<std::int64_t>(lanelet.id());
  for (const auto & segment : route_ptr_->segments) {
    if (segment.preferred_primitive.id == lanelet_id) {
      return true;
    }
    for (const auto & primitive : segment.primitives) {
      if (primitive.id == lanelet_id) {
        return true;
      }
    }
  }

  return false;
}

}
