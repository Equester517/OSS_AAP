#include "common/common_types.hpp"
#include "autoware/route_handler/route_handler.hpp"
#include "autoware/behavior_path_planner_common/data_manager.hpp"
#include "planner_manager.hpp"

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace
{
using autoware::behavior_path_planner::BehaviorModuleOutput;
using autoware::behavior_path_planner::PlannerData;
using autoware::common_types::KinematicState;
using autoware::common_types::LaneletRoute;
using autoware::common_types::LaneletSegment;
using autoware::common_types::PathWithLaneId;
using autoware::common_types::Pose;
using autoware::common_types::TimeStamp;
using autoware::route_handler::RouteHandler;

struct MissionData
{
  Pose start_pose;
  Pose goal_pose;
  std::vector<std::int64_t> lanelet_ids;
};

std::string trim(const std::string & str)
{
  const auto first = str.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return {};
  }
  const auto last = str.find_last_not_of(" \t\r\n");
  return str.substr(first, last - first + 1);
}

std::optional<double> parseDoubleFromLine(const std::string & line)
{
  const auto pos = line.find(':');
  if (pos == std::string::npos) {
    return std::nullopt;
  }
  try {
    return std::stod(trim(line.substr(pos + 1)));
  } catch (...) {
    return std::nullopt;
  }
}

std::optional<std::int64_t> parseInt64FromLine(const std::string & line)
{
  const auto pos = line.find(':');
  if (pos == std::string::npos) {
    return std::nullopt;
  }
  try {
    return std::stoll(trim(line.substr(pos + 1)));
  } catch (...) {
    return std::nullopt;
  }
}

std::optional<std::string> parseStringFromLine(const std::string & line)
{
  const auto pos = line.find(':');
  if (pos == std::string::npos) {
    return std::nullopt;
  }
  return trim(line.substr(pos + 1));
}

bool parsePoseBlock(std::ifstream & ifs, Pose & pose)
{
  std::streampos last_pos = ifs.tellg();
  std::string line;
  int values = 0;
  while (std::getline(ifs, line)) {
    const auto stripped = trim(line);
    if (stripped.empty()) {
      last_pos = ifs.tellg();
      continue;
    }
    if (stripped.find("position:") != std::string::npos ||
        stripped.find("orientation:") != std::string::npos) {
      last_pos = ifs.tellg();
      continue;
    }
    if (stripped.front() == '-') {
      ifs.seekg(last_pos);
      break;
    }
    if (stripped.rfind("x:", 0) == 0) {
      const auto val = parseDoubleFromLine(stripped);
      if (!val) return false;
      if (values < 3) {
        pose.position.x = *val;
      } else {
        pose.orientation.x = *val;
      }
      ++values;
    } else if (stripped.rfind("y:", 0) == 0) {
      const auto val = parseDoubleFromLine(stripped);
      if (!val) return false;
      if (values < 3) {
        pose.position.y = *val;
      } else {
        pose.orientation.y = *val;
      }
      ++values;
    } else if (stripped.rfind("z:", 0) == 0) {
      const auto val = parseDoubleFromLine(stripped);
      if (!val) return false;
      if (values < 3) {
        pose.position.z = *val;
      } else {
        pose.orientation.z = *val;
      }
      ++values;
    } else if (stripped.rfind("w:", 0) == 0) {
      const auto val = parseDoubleFromLine(stripped);
      if (!val) return false;
      pose.orientation.w = *val;
      ++values;
    } else {
      ifs.seekg(last_pos);
      break;
    }
    last_pos = ifs.tellg();
  }
  return true;
}

bool parseMissionPlannerFile(const std::string & path, MissionData & mission)
{
  std::ifstream ifs(path);
  if (!ifs.is_open()) {
    std::cerr << "[Mission] Failed to open file: " << path << "\n";
    return false;
  }

  std::string line;
  bool expect_preferred_id = false;
  while (std::getline(ifs, line)) {
    const auto stripped = trim(line);
    if (stripped.empty()) {
      continue;
    }
    if (stripped == "start_pose:") {
      if (!parsePoseBlock(ifs, mission.start_pose)) {
        std::cerr << "[Mission] Failed to parse start_pose.\n";
        return false;
      }
      continue;
    }
    if (stripped == "goal_pose:") {
      if (!parsePoseBlock(ifs, mission.goal_pose)) {
        std::cerr << "[Mission] Failed to parse goal_pose.\n";
        return false;
      }
      continue;
    }
    if (stripped.find("preferred_primitive") != std::string::npos) {
      expect_preferred_id = true;
      continue;
    }
    if (expect_preferred_id && stripped.rfind("id:", 0) == 0) {
      const auto id = parseInt64FromLine(stripped);
      if (!id) {
        std::cerr << "[Mission] Invalid preferred_primitive id.\n";
        return false;
      }
      mission.lanelet_ids.push_back(*id);
      expect_preferred_id = false;
    }
  }

  if (mission.lanelet_ids.empty()) {
    std::cerr << "[Mission] No lanelet IDs were found.\n";
    return false;
  }
  return true;
}

bool parseLocalizationFile(const std::string & path, KinematicState & ego_state)
{
  std::ifstream ifs(path);
  if (!ifs.is_open()) {
    std::cerr << "[Localization] Failed to open file: " << path << "\n";
    return false;
  }

  enum class Context { None, HeaderStamp, PosePosition, PoseOrientation, TwistLinear, TwistAngular };
  Context context = Context::None;
  bool has_position = false;
  bool has_orientation = false;

  std::string line;
  while (std::getline(ifs, line)) {
    const auto stripped = trim(line);
    if (stripped.empty()) {
      continue;
    }

    if (stripped == "stamp:") {
      context = Context::HeaderStamp;
      continue;
    }
    if (stripped == "position:") {
      context = Context::PosePosition;
      continue;
    }
    if (stripped == "orientation:") {
      context = Context::PoseOrientation;
      continue;
    }
    if (stripped == "linear:") {
      context = Context::TwistLinear;
      continue;
    }
    if (stripped == "angular:") {
      context = Context::TwistAngular;
      continue;
    }
    if (
      stripped == "header:" || stripped == "pose:" || stripped == "twist:" ||
      stripped == "covariance:")
    {
      context = Context::None;
      continue;
    }

    if (stripped.rfind("frame_id:", 0) == 0) {
      if (const auto val = parseStringFromLine(stripped)) {
        ego_state.header.frame_id = *val;
      }
      continue;
    }

    if (context == Context::HeaderStamp) {
      if (stripped.rfind("sec:", 0) == 0) {
        const auto sec = parseInt64FromLine(stripped);
        if (sec) {
          ego_state.header.stamp.sec = *sec;
        }
        continue;
      }
      if (stripped.rfind("nanosec:", 0) == 0) {
        const auto nsec = parseInt64FromLine(stripped);
        if (nsec) {
          ego_state.header.stamp.nsec = *nsec;
        }
        continue;
      }
    }

    const auto assign_position = [&](double value, char axis) {
      switch (axis) {
        case 'x':
          ego_state.pose.position.x = value;
          break;
        case 'y':
          ego_state.pose.position.y = value;
          break;
        case 'z':
          ego_state.pose.position.z = value;
          break;
        default:
          break;
      }
    };

    const auto assign_orientation = [&](double value, char axis) {
      switch (axis) {
        case 'x':
          ego_state.pose.orientation.x = value;
          break;
        case 'y':
          ego_state.pose.orientation.y = value;
          break;
        case 'z':
          ego_state.pose.orientation.z = value;
          break;
        case 'w':
          ego_state.pose.orientation.w = value;
          break;
        default:
          break;
      }
    };

    if (context == Context::PosePosition && stripped.size() > 2 && stripped[1] == ':') {
      const auto val = parseDoubleFromLine(stripped);
      if (val) {
        assign_position(*val, stripped.front());
        has_position = true;
      }
      continue;
    }
    if (context == Context::PoseOrientation && stripped.size() > 2 && stripped[1] == ':') {
      const auto val = parseDoubleFromLine(stripped);
      if (val) {
        assign_orientation(*val, stripped.front());
        if (stripped.front() == 'w') {
          has_orientation = true;
        }
      }
      continue;
    }
    if (context == Context::TwistLinear && stripped.size() > 2 && stripped[1] == ':') {
      const auto val = parseDoubleFromLine(stripped);
      if (val) {
        if (stripped.front() == 'x') ego_state.twist.linear_x = *val;
        if (stripped.front() == 'y') ego_state.twist.linear_y = *val;
        if (stripped.front() == 'z') ego_state.twist.linear_z = *val;
      }
      continue;
    }
    if (context == Context::TwistAngular && stripped.size() > 2 && stripped[1] == ':') {
      const auto val = parseDoubleFromLine(stripped);
      if (val) {
        if (stripped.front() == 'x') ego_state.twist.angular_x = *val;
        if (stripped.front() == 'y') ego_state.twist.angular_y = *val;
        if (stripped.front() == 'z') ego_state.twist.angular_z = *val;
      }
      continue;
    }
  }

  if (!has_position || !has_orientation) {
    std::cerr << "[Localization] Failed to parse pose data from file: " << path << "\n";
    return false;
  }

  return true;
}

struct MapInfo
{
  double min_x{std::numeric_limits<double>::infinity()};
  double max_x{-std::numeric_limits<double>::infinity()};
  double min_y{std::numeric_limits<double>::infinity()};
  double max_y{-std::numeric_limits<double>::infinity()};
  size_t lanelet_count{0};
};

MapInfo computeMapInfo(const lanelet::LaneletMapPtr & map)
{
  MapInfo info;
  if (!map) {
    return info;
  }
  for (const auto & lanelet : map->laneletLayer) {
    ++info.lanelet_count;
    for (const auto & pt : lanelet.centerline()) {
      const auto & p = pt.basicPoint();
      info.min_x = std::min(info.min_x, p.x());
      info.max_x = std::max(info.max_x, p.x());
      info.min_y = std::min(info.min_y, p.y());
      info.max_y = std::max(info.max_y, p.y());
    }
  }
  return info;
}

void printYaml(const PathWithLaneId & path)
{
  std::cout << std::fixed << std::setprecision(6);
  std::cout << "header:\n";
  std::cout << "  stamp:\n";
  std::cout << "    sec: " << path.header.stamp.sec << "\n";
  std::cout << "    nanosec: " << path.header.stamp.nsec << "\n";
  std::cout << "  frame_id: " << path.header.frame_id << "\n";
  std::cout << "points:\n";
  for (const auto & point : path.points) {
    std::cout << "- point:\n";
    std::cout << "    pose:\n";
    std::cout << "      position:\n";
    std::cout << "        x: " << point.point.pose.position.x << "\n";
    std::cout << "        y: " << point.point.pose.position.y << "\n";
    std::cout << "        z: " << point.point.pose.position.z << "\n";
    std::cout << "      orientation:\n";
    std::cout << "        x: " << point.point.pose.orientation.x << "\n";
    std::cout << "        y: " << point.point.pose.orientation.y << "\n";
    std::cout << "        z: " << point.point.pose.orientation.z << "\n";
    std::cout << "        w: " << point.point.pose.orientation.w << "\n";
    std::cout << "    longitudinal_velocity_mps: " << point.point.longitudinal_velocity_mps << "\n";
    std::cout << "    lateral_velocity_mps: " << point.point.lateral_velocity_mps << "\n";
    std::cout << "    heading_rate_rps: " << point.point.heading_rate_rps << "\n";
    std::cout << "    is_final: " << (point.point.is_final ? "true" : "false") << "\n";
    std::cout << "  lane_ids:\n";
    if (point.lane_ids.empty()) {
      std::cout << "  - \n";
    } else {
      for (const auto & id : point.lane_ids) {
        std::cout << "  - " << id << "\n";
      }
    }
  }

  auto printBound = [](const std::vector<autoware::common_types::PointXYZ> & bound,
                       const std::string & name) {
    std::cout << name << ":\n";
    if (bound.empty()) {
      std::cout << "  []\n";
      return;
    }
    for (const auto & pt : bound) {
      std::cout << "- x: " << pt.x << "\n";
      std::cout << "  y: " << pt.y << "\n";
      std::cout << "  z: " << pt.z << "\n";
    }
  };

  printBound(path.left_bound, "left_bound");
  printBound(path.right_bound, "right_bound");
}

std::shared_ptr<RouteHandler> createRouteHandler(const lanelet::LaneletMapPtr & map)
{
  auto traffic_rules = lanelet::traffic_rules::TrafficRulesFactory::create(
    lanelet::Locations::Germany, lanelet::Participants::Vehicle);
  auto routing_graph_unique = lanelet::routing::RoutingGraph::build(*map, *traffic_rules);
  lanelet::routing::RoutingGraphPtr routing_graph(routing_graph_unique.release());
  return std::make_shared<RouteHandler>(map, routing_graph);
}

void applyVelocityProfile(PathWithLaneId & path, const KinematicState * ego_state)
{
  if (path.points.empty()) {
    return;
  }
  if (path.points.size() == 1) {
    path.points.front().point.longitudinal_velocity_mps = 0.0;
    return;
  }

  const double lane_speed = path.points.front().point.longitudinal_velocity_mps;
  const double ego_speed =
    ego_state ? std::abs(ego_state->twist.linear_x) : lane_speed;
  const double cruise_speed = std::max(lane_speed, ego_speed);

  for (size_t i = 0; i + 1 < path.points.size(); ++i) {
    path.points[i].point.longitudinal_velocity_mps = cruise_speed;
  }
  path.points.back().point.longitudinal_velocity_mps = 0.0;
}

LaneletRoute buildRouteMessage(const MissionData & mission)
{
  LaneletRoute route_msg;
  route_msg.header.frame_id = "map";
  route_msg.start_pose = mission.start_pose;
  route_msg.goal_pose = mission.goal_pose;
  route_msg.allow_modification = false;
  route_msg.segments.reserve(mission.lanelet_ids.size());
  for (const auto id : mission.lanelet_ids) {
    LaneletSegment segment;
    segment.preferred_primitive.id = id;
    segment.preferred_primitive.primitive_type = "lane";
    segment.primitives.push_back(segment.preferred_primitive);
    route_msg.segments.push_back(segment);
  }
  return route_msg;
}

PathWithLaneId runBehaviorPlanner(
  const std::shared_ptr<RouteHandler> & route_handler, const MissionData & mission,
  const Pose & fallback_pose, const KinematicState * ego_state)
{
  auto planner_data = std::make_shared<PlannerData>();
  autoware::behavior_path_planner::VehicleInfo vehicle_info{};
  autoware::behavior_path_planner::BehaviorPathPlannerConfig config{};
  planner_data->init_parameters(vehicle_info, config);
  planner_data->route_handler = route_handler;
  auto odometry = std::make_shared<autoware::common_types::Odometry>();
  const Pose & selected_pose = ego_state ? ego_state->pose : fallback_pose;
  odometry->pose.pose = selected_pose;
  if (ego_state) {
    odometry->header = ego_state->header;
    odometry->twist.linear.x = ego_state->twist.linear_x;
    odometry->twist.linear.y = ego_state->twist.linear_y;
    odometry->twist.linear.z = ego_state->twist.linear_z;
    odometry->twist.angular.x = ego_state->twist.angular_x;
    odometry->twist.angular.y = ego_state->twist.angular_y;
    odometry->twist.angular.z = ego_state->twist.angular_z;
  }
  planner_data->self_odometry = odometry;

  autoware::behavior_path_planner::PlannerManager manager;
  const BehaviorModuleOutput output = manager.run(planner_data);
  return output.reference_path;
}

TimeStamp currentTimeStamp()
{
  using namespace std::chrono;
  const auto now = system_clock::now();
  const auto secs = time_point_cast<seconds>(now);
  const auto nsec = duration_cast<nanoseconds>(now - secs);
  TimeStamp stamp;
  stamp.sec = secs.time_since_epoch().count();
  stamp.nsec = static_cast<std::int64_t>(nsec.count());
  return stamp;
}

}  // namespace

int main(int argc, char ** argv)
{
  // if (argc < 4) {
  //   std::cerr << "Usage: " << argv[0] << " <map.osm> <mission_planner.txt> <localization_kinematicstate.txt>\n";
  //   return 1;
  // }

  // const std::string osm_path = argv[1];
  // const std::string mission_path = argv[2];
  // const std::string localization_path = argv[3];
  (void)argc;
  (void)argv;
  const std::string osm_path = "../../../sample-map-planning/lanelet2_map.osm";
  const std::string mission_path = "../../../reference_IO/mission_planner.txt";
  const std::string localization_path = "../../../reference_IO/localization_kinematicstate.txt";
  namespace fs = std::filesystem;
  if (!fs::exists(osm_path)) {
    std::cerr << "[Lanelet2] Map file not found: " << osm_path << "\n";
    return 1;
  }

  lanelet::ErrorMessages errors;
  lanelet::projection::UtmProjector projector(lanelet::Origin({35.23808753540768, 139.9009591876285}));
  auto lanelet_map_unique = lanelet::load(osm_path, projector, &errors);
  if (!lanelet_map_unique) {
    std::cerr << "[Lanelet2] Failed to load map: " << osm_path << "\n";
    return 1;
  }
  lanelet::LaneletMapPtr lanelet_map(lanelet_map_unique.release());

  const auto map_info = computeMapInfo(lanelet_map);
  std::cerr << "[Lanelet2] map loaded. lanelet count = " << map_info.lanelet_count << "\n";
  if (map_info.lanelet_count > 0) {
    std::cerr << "[Lanelet2] map XY range: x in [" << map_info.min_x << ", " << map_info.max_x
              << "], y in [" << map_info.min_y << ", " << map_info.max_y << "]\n";
  }

  MissionData mission;
  if (!parseMissionPlannerFile(mission_path, mission)) {
    return 1;
  }

  KinematicState ego_state{};
  const bool localization_available = parseLocalizationFile(localization_path, ego_state);
  if (!localization_available) {
    std::cerr << "[Localization] Using mission start pose as fallback ego state.\n";
  }

  const auto route_handler = createRouteHandler(lanelet_map);
  const auto route_msg = buildRouteMessage(mission);
  route_handler->setRoute(route_msg);
  std::cerr << "[RouteHandler] Route initialized with " << mission.lanelet_ids.size()
            << " lanelets.\n";

  const KinematicState * ego_state_ptr = localization_available ? &ego_state : nullptr;
  PathWithLaneId path = runBehaviorPlanner(route_handler, mission, mission.start_pose, ego_state_ptr);
  if (path.points.empty()) {
    std::cerr << "[BPP] Behavior Path Planner returned an empty path.\n";
    return 1;
  }
  path.header.frame_id = "map";
  path.header.stamp = currentTimeStamp();
  applyVelocityProfile(path, ego_state_ptr);

  printYaml(path);
  return 0;
}
