#include "behavior_path_planner_runner.hpp"
#include "autoware/route_handler/route_handler.hpp"
#include "autoware/behavior_path_planner_common/data_manager.hpp"
#include "planner_manager.hpp"

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <filesystem>
#include <iostream>
#include <memory>

#ifndef BPP_RUNNER_MAIN_GUARD
namespace behavior_path_planner_runner
{

using autoware::behavior_path_planner::BehaviorModuleOutput;
using autoware::behavior_path_planner::PlannerData;
using autoware::common_types::LaneletRoute;
using autoware::common_types::Odometry;
using autoware::common_types::PathWithLaneId;
using autoware::route_handler::RouteHandler;

namespace
{

std::shared_ptr<RouteHandler> createRouteHandler(const lanelet::LaneletMapPtr& map)
{
    auto traffic_rules = lanelet::traffic_rules::TrafficRulesFactory::create(
        lanelet::Locations::Germany, lanelet::Participants::Vehicle);
    auto routing_graph_unique = lanelet::routing::RoutingGraph::build(*map, *traffic_rules);
    lanelet::routing::RoutingGraphPtr routing_graph(routing_graph_unique.release());
    return std::make_shared<RouteHandler>(map, routing_graph);
}

PathWithLaneId runBehaviorPlanner(
    const std::shared_ptr<RouteHandler>& route_handler,
    const std::shared_ptr<Odometry>& odometry)
{
    auto planner_data = std::make_shared<PlannerData>();
    autoware::behavior_path_planner::VehicleInfo vehicle_info{};
    autoware::behavior_path_planner::BehaviorPathPlannerConfig config{};
    planner_data->init_parameters(vehicle_info, config);
    planner_data->route_handler = route_handler;
    planner_data->self_odometry = odometry;

    autoware::behavior_path_planner::PlannerManager manager;
    const BehaviorModuleOutput output = manager.run(planner_data);
    return output.reference_path;
}

} // anonymous namespace

std::optional<PathWithLaneId> run_once(
    const std::string& osm_path,
    const LaneletRoute& route,
    const Odometry& odometry)
{
    namespace fs = std::filesystem;
    
    // Check if map file exists
    if (!fs::exists(osm_path)) {
        std::cerr << "[BPP Runner] Map file not found: " << osm_path << "\n";
        return std::nullopt;
    }

    // Load Lanelet2 map
    lanelet::ErrorMessages errors;
    lanelet::projection::UtmProjector projector(lanelet::Origin({35.23808753540768, 139.9009591876285}));
    auto lanelet_map_unique = lanelet::load(osm_path, projector, &errors);
    if (!lanelet_map_unique) {
        std::cerr << "[BPP Runner] Failed to load map: " << osm_path << "\n";
        return std::nullopt;
    }
    lanelet::LaneletMapPtr lanelet_map(lanelet_map_unique.release());
    std::cout << "[BPP Runner] Lanelet2 map loaded successfully\n";

    // Create route handler and set route
    const auto route_handler = createRouteHandler(lanelet_map);
    route_handler->setRoute(route);
    std::cout << "[BPP Runner] Route initialized with " << route.segments.size() << " segments\n";

    // Prepare odometry
    auto odometry_ptr = std::make_shared<Odometry>(odometry);

    // Run behavior path planner
    PathWithLaneId path = runBehaviorPlanner(route_handler, odometry_ptr);
    
    if (path.points.empty()) {
        std::cerr << "[BPP Runner] Behavior Path Planner returned an empty path\n";
        return std::nullopt;
    }

    std::cout << "[BPP Runner] Path generated with " << path.points.size() << " points\n";
    return path;
}

} // namespace behavior_path_planner_runner
#endif // BPP_RUNNER_MAIN_GUARD
