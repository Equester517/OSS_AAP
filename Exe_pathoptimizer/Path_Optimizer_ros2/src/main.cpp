#include "path_optimizer.hpp"
#include "path_optimizer_types.hpp"
#include "path_optimizer/path_optimizer_runner.hpp"

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>

using namespace autoware::path_optimizer;

// Expose standalone main only when explicitly requested by build.
#ifdef BUILD_STANDALONE_PATH_OPTIMIZER
// Only build the full standalone main when explicitly requested by the build
#ifdef BUILD_STANDALONE_PATH_OPTIMIZER
int main(int argc, char ** argv)
{
  std::string path_file = (argc > 1) ? argv[1] : "test_path.csv";
  path_optimizer_runner::run_once(path_file);
  return 0;
}
#endif // BUILD_STANDALONE_PATH_OPTIMIZER
#endif

// ... rest of old code commented out for reference (not compiled unless BUILD_STANDALONE_PATH_OPTIMIZER)
struct OdometryData {
  Pose pose;
  double linear_velocity_x;  // m/s
  double linear_velocity_y;  // m/s
  double angular_velocity_z; // rad/s
};

OdometryData loadOdometryFromCSV(const std::string & filename)
{
  OdometryData odom;
  // Default values
  odom.pose.position.x = 0.0;
  odom.pose.position.y = 0.0;
  odom.pose.position.z = 0.0;
  odom.pose.orientation.x = 0.0;
  odom.pose.orientation.y = 0.0;
  odom.pose.orientation.z = 0.0;
  odom.pose.orientation.w = 1.0;
  odom.linear_velocity_x = 5.0;  // Default 5.0 m/s
  odom.linear_velocity_y = 0.0;
  odom.angular_velocity_z = 0.0;
  
  std::ifstream file(filename);
  
  if (!file.is_open()) {
    std::cerr << "Failed to open odometry file: " << filename << std::endl;
    std::cerr << "Using default values: pos=(0,0), vel=5.0 m/s" << std::endl;
    return odom;
  }
  
  std::string line;
  // Skip header
  std::getline(file, line);
  
  // Read data line
  if (std::getline(file, line)) {
    std::istringstream iss(line);
    std::string token;
    std::vector<std::string> tokens;
    
    while (std::getline(iss, token, ',')) {
      tokens.push_back(token);
    }
    
    // Expected format: x,y,z,qx,qy,qz,qw,vx,vy,vz,wx,wy,wz
    if (tokens.size() >= 13) {
      odom.pose.position.x = std::stod(tokens[0]);
      odom.pose.position.y = std::stod(tokens[1]);
      odom.pose.position.z = std::stod(tokens[2]);
      odom.pose.orientation.x = std::stod(tokens[3]);
      odom.pose.orientation.y = std::stod(tokens[4]);
      odom.pose.orientation.z = std::stod(tokens[5]);
      odom.pose.orientation.w = std::stod(tokens[6]);
      odom.linear_velocity_x = std::stod(tokens[7]);
      odom.linear_velocity_y = std::stod(tokens[8]);
      // tokens[9] is vz (not used for ground vehicle)
      // tokens[10] is wx (not used)
      // tokens[11] is wy (not used)
      odom.angular_velocity_z = std::stod(tokens[12]);
      
      std::cout << "Loaded odometry from " << filename << std::endl;
      std::cout << "  Position: (" << odom.pose.position.x << ", " 
                << odom.pose.position.y << ")" << std::endl;
      std::cout << "  Velocity: " << odom.linear_velocity_x << " m/s" << std::endl;
    } else {
      std::cerr << "Invalid odometry format (expected 13 columns)" << std::endl;
    }
  }
  
  return odom;
}

// Load path points from CSV
std::vector<PathPoint> loadPathFromCSV(const std::string & filename)
{
  std::vector<PathPoint> points;
  std::ifstream file(filename);
  
  if (!file.is_open()) {
    std::cerr << "Failed to open file: " << filename << std::endl;
    return points;
  }
  
  std::string line;
  // Skip header
  std::getline(file, line);
  
  while (std::getline(file, line)) {
    std::istringstream iss(line);
    std::string token;
    std::vector<std::string> tokens;
    
    while (std::getline(iss, token, ',')) {
      tokens.push_back(token);
    }
    
    if (tokens.size() >= 5) {
      PathPoint point;
      point.pose.position.x = std::stod(tokens[0]);
      point.pose.position.y = std::stod(tokens[1]);
      point.pose.position.z = std::stod(tokens[2]);
      
      double yaw = std::stod(tokens[3]);
      point.pose.orientation.w = std::cos(yaw / 2.0);
      point.pose.orientation.z = std::sin(yaw / 2.0);
      
      point.longitudinal_velocity_mps = std::stod(tokens[4]);
      
      points.push_back(point);
    } else if (tokens.size() >= 4) {
      // Backward compatibility: x,y,yaw,velocity (no z)
      PathPoint point;
      point.pose.position.x = std::stod(tokens[0]);
      point.pose.position.y = std::stod(tokens[1]);
      point.pose.position.z = 0.0;
      
      double yaw = std::stod(tokens[2]);
      point.pose.orientation.w = std::cos(yaw / 2.0);
      point.pose.orientation.z = std::sin(yaw / 2.0);
      
      point.longitudinal_velocity_mps = std::stod(tokens[3]);
      
      points.push_back(point);
    }
  }
  
  std::cout << "Loaded " << points.size() << " path points from " << filename << std::endl;
  return points;
}

// Load bound points from CSV
std::vector<Point> loadBoundFromCSV(const std::string & filename)
{
  std::vector<Point> points;
  std::ifstream file(filename);
  
  if (!file.is_open()) {
    std::cerr << "Failed to open file: " << filename << std::endl;
    return points;
  }
  
  std::string line;
  // Skip header
  std::getline(file, line);
  
  while (std::getline(file, line)) {
    std::istringstream iss(line);
    std::string token;
    std::vector<std::string> tokens;
    
    while (std::getline(iss, token, ',')) {
      tokens.push_back(token);
    }
    
    if (tokens.size() >= 3) {
      Point point;
      point.x = std::stod(tokens[0]);
      point.y = std::stod(tokens[1]);
      point.z = std::stod(tokens[2]);
      
      points.push_back(point);
    } else if (tokens.size() >= 2) {
      // Backward compatibility: x,y (no z)
      Point point;
      point.x = std::stod(tokens[0]);
      point.y = std::stod(tokens[1]);
      point.z = 0.0;
      
      points.push_back(point);
    }
  }
  
  std::cout << "Loaded " << points.size() << " bound points from " << filename << std::endl;
  return points;
}

// Save trajectory to CSV
void saveTrajectoryToCSV(
  const std::string & filename,
  const std::vector<TrajectoryPoint> & trajectory)
{
  std::ofstream file(filename);
  
  if (!file.is_open()) {
    std::cerr << "Failed to open file for writing: " << filename << std::endl;
    return;
  }
  
  // CSV header matching ROS2 TrajectoryPoint format
  file << "x,y,z,qx,qy,qz,qw,longitudinal_velocity_mps,lateral_velocity_mps,"
       << "acceleration_mps2,heading_rate_rps,front_wheel_angle_rad,rear_wheel_angle_rad\n";
  
  for (const auto & point : trajectory) {
    file << point.pose.position.x << ","
         << point.pose.position.y << ","
         << point.pose.position.z << ","
         << point.pose.orientation.x << ","
         << point.pose.orientation.y << ","
         << point.pose.orientation.z << ","
         << point.pose.orientation.w << ","
         << point.longitudinal_velocity_mps << ","
         << point.lateral_velocity_mps << ","
         << point.acceleration_mps2 << ","
         << point.heading_rate_rps << ","
         << point.front_wheel_angle_rad << ","
         << point.rear_wheel_angle_rad << "\n";
  }
  
  std::cout << "Saved " << trajectory.size() << " trajectory points to " << filename << std::endl;
}

// Generate simple test data
void generateTestData()
{
  // Generate straight path
  std::ofstream path_file("test_path.csv");
  path_file << "x,y,yaw,velocity\n";
  
  for (int i = 0; i < 50; ++i) {
    double x = i * 2.0;
    double y = 0.0;
    double yaw = 0.0;
    double velocity = 10.0;
    
    path_file << x << "," << y << "," << yaw << "," << velocity << "\n";
  }
  path_file.close();
  
  // Generate left bound
  std::ofstream left_file("test_left_bound.csv");
  left_file << "x,y\n";
  
  for (int i = 0; i < 50; ++i) {
    double x = i * 2.0;
    double y = 2.0;
    
    left_file << x << "," << y << "\n";
  }
  left_file.close();
  
  // Generate right bound
  std::ofstream right_file("test_right_bound.csv");
  right_file << "x,y\n";
  
  for (int i = 0; i < 50; ++i) {
    double x = i * 2.0;
    double y = -2.0;
    
    right_file << x << "," << y << "\n";
  }
  right_file.close();
  
  std::cout << "Generated test data files" << std::endl;
}

int main(int argc, char ** argv)
{
  std::cout << "=== Path Optimizer Standalone ===" << std::endl;
  
  // Check arguments
  if (argc < 2) {
    std::cout << "Generating test data..." << std::endl;
    generateTestData();
    
    std::cout << "\nUsage: " << argv[0] << " <path.csv> <odometry.csv> [left_bound.csv] [right_bound.csv]" << std::endl;
    std::cout << "Using generated test data..." << std::endl;
    
    argc = 4;
    char* test_argv[] = {
      argv[0],
      const_cast<char*>("test_path.csv"),
      const_cast<char*>("test_left_bound.csv"),
      const_cast<char*>("test_right_bound.csv")
    };
    argv = test_argv;
  }
  
  // 1. Load data
  std::cout << "\n=== Loading Data ===" << std::endl;
  
  std::string path_file = argv[1];
  std::string odometry_file = argc > 2 ? argv[2] : "test_odometry.csv";
  std::string left_bound_file = argc > 3 ? argv[3] : "test_left_bound.csv";
  std::string right_bound_file = argc > 4 ? argv[4] : "test_right_bound.csv";
  
  auto path_points = loadPathFromCSV(path_file);
  auto odom_data = loadOdometryFromCSV(odometry_file);
  auto left_bound = loadBoundFromCSV(left_bound_file);
  auto right_bound = loadBoundFromCSV(right_bound_file);
  
  if (path_points.empty()) {
    std::cerr << "No path points loaded!" << std::endl;
    return 1;
  }
  
  // 2. Setup parameters
  std::cout << "\n=== Setting Up Parameters ===" << std::endl;
  
  PathOptimizerParam param;
  param.trajectory.output_delta_arc_length = 0.5;
  param.trajectory.num_sampling_points = 100;
  
  // Define vehicle info first (needed for optimization_center_offset calculation)
  VehicleInfo vehicle_info;
  vehicle_info.wheel_base = 2.79;
  vehicle_info.vehicle_width = 1.92;
  vehicle_info.vehicle_length = 4.77;
  vehicle_info.max_steer_angle = 0.7;
  
  param.mpt.max_steer_rad = 0.7;
  param.mpt.lat_error_weight = 1.0;  // Normal tracking weight
  param.mpt.steer_input_weight = 1.0;  // Will be increased to min 0.5 for stability
  param.mpt.steer_rate_weight = 1.0;  // ROS2 default for smooth steering
  param.mpt.enable_avoidance = true;
  param.mpt.num_points = 100;  // Match ROS2: 100 points for full dynamics
  
  // Adaptive weight parameters (ROS2 compatibility)
  param.mpt.terminal_lat_error_weight = 100.0;  // Strong tracking at path end
  param.mpt.goal_lat_error_weight = 1000.0;  // Very strong tracking at goal
  param.mpt.terminal_yaw_error_weight = 0.0;
  param.mpt.goal_yaw_error_weight = 0.0;
  
  // Optimization center offset (ROS2 compatibility)
  // Set to wheelbase * 0.8 as per ROS2 default logic
  // This makes the trajectory start ahead of ego position
  param.mpt.optimization_center_offset = vehicle_info.wheel_base * 0.8;  // = 2.79 * 0.8 = 2.232m
  
  param.replan_checker.max_path_shape_change_dist = 0.5;
  param.replan_checker.max_ego_moving_dist = 5.0;
  param.replan_checker.max_delta_time_sec = 2.0;
  
  std::cout << "Parameters configured" << std::endl;
  
  // 3. Create optimizer
  std::cout << "\n=== Creating Path Optimizer ===" << std::endl;
  
  PathOptimizer optimizer(param, vehicle_info);
  
  // 4. Set ego state from odometry file
  Pose ego_pose = odom_data.pose;
  double ego_velocity = odom_data.linear_velocity_x;  // m/s (from test_odometry.csv)
  
  std::cout << "\n=== Ego State (from odometry file) ===" << std::endl;
  std::cout << "Ego pose: (" << ego_pose.position.x << ", " 
            << ego_pose.position.y << ")" << std::endl;
  std::cout << "Ego velocity: " << ego_velocity << " m/s" << std::endl;
  
  // Calculate yaw from quaternion for display
  double ego_yaw = 2.0 * std::atan2(ego_pose.orientation.z, ego_pose.orientation.w);
  std::cout << "Ego yaw: " << ego_yaw << " rad (" << (ego_yaw * 180.0 / M_PI) << " deg)" << std::endl;
  
  // 5. Optimize path with ITERATIVE REFINEMENT (ROS2 방식)
  std::cout << "\n=== Optimizing Path (Iterative Refinement) ===" << std::endl;
  
  // ⭐⭐⭐ IMPORTANT: Single-shot optimization for ROS2 compatibility
  // ROS2는 한 번의 최적화만 수행하므로, Standalone도 동일하게 1회만 수행합니다.
  // Multiple iterations (10회)는 수렴 시뮬레이션 목적이지만, ROS2 비교를 위해서는 1회만 필요합니다.
  const int num_iterations = 1;  // ROS2 compatibility: single-shot optimization
  OptimizationResult result;
  
  // ego_pose는 매 iteration마다 업데이트됩니다 (vehicle이 움직이는 시뮬레이션)
  Pose current_ego_pose = ego_pose;
  
  for (int iter = 0; iter < num_iterations; ++iter) {
    std::cout << "\n--- Iteration " << (iter + 1) << "/" << num_iterations << " ---" << std::endl;
    std::cout << "  Current ego: (" << current_ego_pose.position.x 
              << ", " << current_ego_pose.position.y << ")" << std::endl;
    
    result = optimizer.optimizePathWithDebug(
      path_points, left_bound, right_bound, current_ego_pose, ego_velocity);
    
    if (!result.success) {
      std::cerr << "Optimization failed at iteration " << (iter + 1) << std::endl;
      if (iter > 0) {
        // 이전 결과 사용
        std::cout << "Using previous iteration result" << std::endl;
      }
      break;
    }
    
    std::cout << "  Computation time: " << result.computation_time_ms << " ms" << std::endl;
    std::cout << "  Output points: " << result.trajectory.size() << std::endl;
    
    // 첫 몇 점의 lateral offset 확인 (수렴 모니터링)
    if (!result.reference_points.empty()) {
      std::cout << "  First point lat_offset: " 
                << result.reference_points[0].optimized_kinematic_state.lat << " m" << std::endl;
      
      if (result.reference_points.size() > 4) {
        std::cout << "  Point[3] lat_offset: " 
                  << result.reference_points[3].optimized_kinematic_state.lat << " m" << std::endl;
      }
      
      // First optimized point position (global coordinates)
      const auto & first_opt_pt = result.trajectory.front();
      std::cout << "  First optimized point global: (" 
                << first_opt_pt.pose.position.x << ", " 
                << first_opt_pt.pose.position.y << ")" << std::endl;
    }
    
    // 다음 iteration을 위해: optimized trajectory를 PathPoint로 변환하여 다음 입력으로
    // ⭐ ROS2의 핵심: ego_pose도 첫 번째 optimized point로 업데이트!
    if (iter < num_iterations - 1 && !result.trajectory.empty()) {
      // TrajectoryPoint -> PathPoint 변환
      path_points.clear();
      for (const auto & traj_pt : result.trajectory) {
        PathPoint path_pt;
        path_pt.pose = traj_pt.pose;
        path_pt.longitudinal_velocity_mps = traj_pt.longitudinal_velocity_mps;
        path_points.push_back(path_pt);
      }
      
      // ⭐⭐⭐ 이것이 핵심! Ego pose를 optimized trajectory의 첫 점으로 업데이트
      // ROS2는 vehicle이 조금씩 움직이면서 재계획하므로, 
      // 이전 optimization 결과의 몇 번째 점(예: 1번째)으로 이동했다고 시뮬레이션
      const size_t ego_advance_steps = 1;  // ⭐ 3 -> 1: 더 작은 step으로 수렴
      if (result.trajectory.size() > ego_advance_steps) {
        current_ego_pose = result.trajectory[ego_advance_steps].pose;
      } else {
        current_ego_pose = result.trajectory.front().pose;
      }
      
      std::cout << "  -> Using optimized result as next input (converging...)" << std::endl;
      std::cout << "  -> Ego advanced to step " << ego_advance_steps 
                << ": (" << current_ego_pose.position.x 
                << ", " << current_ego_pose.position.y << ")" << std::endl;
    }
  }
  
  // 6. Print results
  std::cout << "\n=== Optimization Results ===" << std::endl;
  std::cout << "Success: " << (result.success ? "YES" : "NO") << std::endl;
  std::cout << "Computation time: " << result.computation_time_ms << " ms" << std::endl;
  std::cout << "Output points: " << result.trajectory.size() << std::endl;
  
  if (!result.error_message.empty()) {
    std::cout << "Message: " << result.error_message << std::endl;
  }
  
  if (!result.trajectory.empty()) {
    // 7. Save results
    saveTrajectoryToCSV("optimized_trajectory.csv", result.trajectory);
    
    // Print reference points (optimized before resampling)
    if (!result.reference_points.empty()) {
      std::cout << "\nFirst 5 reference points (optimized, before resampling):" << std::endl;
      for (size_t i = 0; i < std::min(size_t(5), result.reference_points.size()); ++i) {
        const auto & p = result.reference_points[i];
        double yaw = 2.0 * std::atan2(p.pose.orientation.z, p.pose.orientation.w);
        
        std::cout << "  [" << i << "] x=" << p.pose.position.x 
                  << ", y=" << p.pose.position.y
                  << ", lat_offset=" << p.optimized_kinematic_state.lat
                  << ", yaw_error=" << p.optimized_kinematic_state.yaw
                  << ", v=" << p.longitudinal_velocity_mps << " m/s" << std::endl;
      }
    }
    
    // Print first few points
    std::cout << "\nFirst 5 optimized points (final, after resampling):" << std::endl;
    for (size_t i = 0; i < std::min(size_t(5), result.trajectory.size()); ++i) {
      const auto & p = result.trajectory[i];
      double yaw = 2.0 * std::atan2(p.pose.orientation.z, p.pose.orientation.w);
      
      std::cout << "  [" << i << "] x=" << p.pose.position.x 
                << ", y=" << p.pose.position.y
                << ", yaw=" << yaw
                << ", v=" << p.longitudinal_velocity_mps << " m/s" << std::endl;
    }
  }
  
  std::cout << "\n=== Done ===" << std::endl;
  
  return 0;
}
