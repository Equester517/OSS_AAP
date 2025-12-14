#include "path_optimizer/path_optimizer_runner.hpp"

#include "path_optimizer.hpp"
#include "path_optimizer_types.hpp"

#include <iostream>
#include <vector>

using namespace autoware::path_optimizer;

namespace path_optimizer_runner
{

void run_once(
    const std::vector<PathPoint>& path_points,
    const Pose& ego_pose,
    double ego_velocity,
    const std::vector<Point>& left_bound,
    const std::vector<Point>& right_bound)
{
  if (path_points.empty()) {
    std::cerr << "path_optimizer_runner: No path points provided" << std::endl;
    return;
  }

  std::cout << "path_optimizer_runner: Starting optimization with " << path_points.size() 
            << " path points" << std::endl;

  // Setup parameters (same defaults as before)
  PathOptimizerParam param;
  param.trajectory.output_delta_arc_length = 0.5;
  param.trajectory.num_sampling_points = 100;

  VehicleInfo vehicle_info;
  vehicle_info.wheel_base = 2.79;
  vehicle_info.vehicle_width = 1.92;
  vehicle_info.vehicle_length = 4.77;
  vehicle_info.max_steer_angle = 0.7;

  param.mpt.max_steer_rad = 0.7;
  param.mpt.lat_error_weight = 1.0;
  param.mpt.steer_input_weight = 1.0;
  param.mpt.steer_rate_weight = 1.0;
  param.mpt.enable_avoidance = true;
  param.mpt.num_points = 100;

  param.mpt.terminal_lat_error_weight = 100.0;
  param.mpt.goal_lat_error_weight = 1000.0;
  param.mpt.terminal_yaw_error_weight = 0.0;
  param.mpt.goal_yaw_error_weight = 0.0;

  param.mpt.optimization_center_offset = vehicle_info.wheel_base * 0.8;

  param.replan_checker.max_path_shape_change_dist = 0.5;
  param.replan_checker.max_ego_moving_dist = 5.0;
  param.replan_checker.max_delta_time_sec = 2.0;

  std::cout << "path_optimizer_runner: Parameters configured" << std::endl;

  PathOptimizer optimizer(param, vehicle_info);

  std::cout << "path_optimizer_runner: Ego state: pos=(" << ego_pose.position.x
            << ", " << ego_pose.position.y << "), vel=" << ego_velocity << " m/s" << std::endl;

  // Run optimization with provided data
  OptimizationResult result = optimizer.optimizePathWithDebug(
      path_points, left_bound, right_bound, ego_pose, ego_velocity);

  std::cout << "path_optimizer_runner: Success: " << (result.success ? "YES" : "NO") << std::endl;
  std::cout << "path_optimizer_runner: Computation time: " << result.computation_time_ms << " ms" << std::endl;
  std::cout << "path_optimizer_runner: Output points: " << result.trajectory.size() << std::endl;
}

} // namespace path_optimizer_runner
