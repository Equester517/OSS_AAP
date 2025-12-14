#include "mission_planner_runner.hpp"

#include "mission_planner.hpp"
#include "default_planner.hpp"
#include "arrival_checker.hpp"

#include <iostream>
#include <cmath>
#include <filesystem>
#include <optional>

using namespace autoware::mission_planner_universe;

namespace mission_planner_runner
{
std::optional<autoware::mission_planner_universe::LaneletRoute> run_once(
    const std::string &osm_file,
    const autoware::mission_planner_universe::Pose &start,
    const autoware::mission_planner_universe::Pose &goal)
{
  // Check OSM file exists before proceeding
  if (osm_file.empty()) {
    std::cerr << "mission_planner_runner: no osm file path provided\n";
    return std::nullopt;
  }

  std::filesystem::path map_path(osm_file);
  if (!std::filesystem::exists(map_path)) {
    std::cerr << "mission_planner_runner: OSM file not found: " << osm_file << " (cwd=" << std::filesystem::current_path() << ")\n";
    return std::nullopt;
  }

  // 1) DefaultPlanner 준비
  DefaultPlannerParam param;
  SimpleVehicleInfo vehicle_info;
  auto planner = std::make_shared<DefaultPlanner>(param, vehicle_info);

  // 2) OSM 파일 path (인자로 전달)
  std::string map_path_str = osm_file;

  // 3) origin hard-code (map_config.yaml 스크립트에서 가져옴)
  lanelet::GPSPoint origin_gps;
  origin_gps.lat = 35.23808753540768;
  origin_gps.lon = 139.9009591876285;
  origin_gps.ele = 0.0;

  lanelet::Origin origin(origin_gps);
  planner->loadOsmMap(map_path_str, origin);

  // 4) pose 변환 함수 (여기서는 맵 좌표 쓰므로 아무처리 없이 그대로 반환)
  TransformPoseFn tf_fn = [](const Pose & p, const Header &) {
    return p;
  };

  // 5) MissionPlanner 생성
  MissionPlanner core("map", planner, tf_fn);

  // 6) header, uuid 준비 (start/goal are now passed as parameters)
  Header h;
  h.frame_id = "map";

  UUID uuid{};
  uuid[0] = 1;

  std::vector<Pose> waypoints;

  // 8) Route 생성
  auto route = core.makeRouteFromWaypoints(h, waypoints, start, goal, uuid, true);

  // std::cout << "Route created: start=("
  //           << route.start_pose.position.x << ", "
  //           << route.start_pose.position.y << "), goal=("
  //           << route.goal_pose.position.x << ", "
  //           << route.goal_pose.position.y << ")\n";

  // std::cout << "Route segments size = " << route.segments.size() << std::endl;
  // Commented detailed per-segment output to reduce log noise
  // for (std::size_t i = 0; i < route.segments.size(); ++i) {
  //     const auto & seg = route.segments[i];
  //     std::cout << "  seg " << i;
  //
  //     if (!seg.primitives.empty()) {
  //       std::cout << " lanelet_id=" << seg.primitives.front().id;
  //     } else {
  //       std::cout << " lanelet_id=<none>";
  //     }
  //
  //     std::cout << " primitives=" << seg.primitives.size() << std::endl;
  // }

  // ArrivalChecker 테스트
  ArrivalChecker arrival_checker(
    /*angle_rad*/ 5.0 * M_PI / 180.0,
    /*distance*/ 1.0,
    /*duration_sec*/ 2.0,
    /*stop_checker*/ [](double) {
      // 예제: 항상 정지했다고 가정
      return true;
    });

  Pose2D goal2d{"map", goal.position.x, goal.position.y, 0.0};
  arrival_checker.set_goal(goal2d);

  Pose2D current{"map", goal.position.x + 0.3, goal.position.y + 0.2, 0.01};
  bool arrived = arrival_checker.is_arrived(current);
  // Keep arrival result output
  std::cout << "Arrived? " << (arrived ? "YES" : "NO") << std::endl;

  // Return the created route so the caller (SWC) can publish/convert it.
  return route;
}

} // namespace mission_planner_runner
