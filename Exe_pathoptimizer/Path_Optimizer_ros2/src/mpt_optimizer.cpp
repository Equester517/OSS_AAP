// Copyright 2023 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "mpt_optimizer.hpp"

#ifdef USE_OSQP
#include "osqp_interface.hpp"
#endif

#include "cubic_spline.hpp"

#include <cmath>
#include <iostream>

namespace autoware::path_optimizer
{

namespace
{
// Helper function to get yaw from quaternion
double getYaw(const Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

// Helper function to calculate distance
double calcDistance(const Point & p1, const Point & p2)
{
  double dx = p1.x - p2.x;
  double dy = p1.y - p2.y;
  return std::hypot(dx, dy);
}

// Helper function to calculate curvature
double calculateCurvature(
  const Pose & prev_pose, const Pose & curr_pose, const Pose & next_pose)
{
  double prev_yaw = getYaw(prev_pose.orientation);
  double curr_yaw = getYaw(curr_pose.orientation);
  double next_yaw = getYaw(next_pose.orientation);
  
  double dyaw = (next_yaw - prev_yaw) / 2.0;
  double ds = calcDistance(prev_pose.position, next_pose.position) / 2.0;
  
  if (ds < 1e-6) return 0.0;
  
  return dyaw / ds;
}
}  // namespace

MPTOptimizer::MPTOptimizer(
  const MPTParam & param,
  const VehicleInfo & vehicle_info)
: param_(param)
, vehicle_info_(vehicle_info)
, state_equation_generator_(std::make_unique<StateEquationGenerator>(
    vehicle_info.wheel_base, vehicle_info.max_steer_angle))
{
  // std::cout << "[MPTOptimizer] Initialized with state equation generator" << std::endl;
}

MPTOptimizer::~MPTOptimizer() = default;

std::optional<std::vector<TrajectoryPoint>> MPTOptimizer::optimize(
  const std::vector<TrajectoryPoint> & traj_points,
  const std::vector<Point> & left_bound,
  const std::vector<Point> & right_bound,
  const Pose & ego_pose,
  const double ego_velocity)
{
  // std::cout << "[MPTOptimizer] Starting optimization..." << std::endl;
  
  // 1. Generate reference points
  ref_points_ = generateReferencePoints(traj_points);
  
  if (ref_points_.empty()) {
    // std::cerr << "[MPTOptimizer] No reference points generated" << std::endl;
    return std::nullopt;
  }
  
  // std::cout << "  - Reference points: " << ref_points_.size() << std::endl;
  
  // ⭐ 1.5. Apply fixed point mechanism (ROS2 temporal consistency)
  updateFixedPoint(ref_points_);
  
  // 2. Calculate bounds from drivable area
  auto bounds = calculateBounds(ref_points_, left_bound, right_bound);
  
  if (bounds.empty()) {
    // std::cerr << "[MPTOptimizer] Failed to calculate bounds" << std::endl;
    return std::nullopt;
  }
  
  // 3. Calculate initial ego state relative to reference path
  KinematicState ego_state;
  
  // Calculate ego's lateral offset and yaw error from first reference point
  if (!ref_points_.empty()) {
    const auto & ref_point = ref_points_[0];
    
    // Calculate lateral offset: perpendicular distance from reference
    double dx = ego_pose.position.x - ref_point.pose.position.x;
    double dy = ego_pose.position.y - ref_point.pose.position.y;
    double ref_yaw = getYaw(ref_point.pose.orientation);
    
    // Transform to Frenet frame: lateral = perpendicular distance
    ego_state.lat = -dx * std::sin(ref_yaw) + dy * std::cos(ref_yaw);
    
    // Yaw error: difference between ego yaw and reference yaw
    double ego_yaw = getYaw(ego_pose.orientation);
    ego_state.yaw = ego_yaw - ref_yaw;
    
    // Normalize yaw error to [-pi, pi]
    while (ego_state.yaw > M_PI) ego_state.yaw -= 2.0 * M_PI;
    while (ego_state.yaw < -M_PI) ego_state.yaw += 2.0 * M_PI;
    
    // std::cout << "  - Ego initial state: lat=" << ego_state.lat 
    // << ", yaw=" << ego_state.yaw << " rad" << std::endl;
  } else {
    ego_state.lat = 0.0;
    ego_state.yaw = 0.0;
  }
  
  // 4. Solve QP problem
  bool success = solveQP(ref_points_, bounds, ego_state);
  
  if (!success) {
    // std::cerr << "[MPTOptimizer] QP optimization failed" << std::endl;
    return std::nullopt;
  }
  
  // 5. Convert to trajectory
  auto optimized_traj = convertToTrajectory(ref_points_);
  
  // ⭐ 6. Save results for warm start and fixed point in next iteration
  prev_ref_points_ = ref_points_;
  has_prev_solution_ = true;
  // std::cout << "[MPTOptimizer] Saved previous solution for warm start" << std::endl;
  
  // std::cout << "[MPTOptimizer] Optimization successful" << std::endl;
  
  return optimized_traj;
}

std::vector<ReferencePoint> MPTOptimizer::generateReferencePoints(
  const std::vector<TrajectoryPoint> & traj_points) const
{
  std::vector<ReferencePoint> ref_points;
  
  if (traj_points.size() < 2) {
    return ref_points;
  }
  
  // ========================================
  // Step 1: Arc-length parameterization 계산
  // ========================================
  // 각 waypoint까지의 누적 거리 계산
  std::vector<double> s_vec;  // arc-length
  std::vector<double> x_vec, y_vec, z_vec;
  
  s_vec.push_back(0.0);
  x_vec.push_back(traj_points[0].pose.position.x);
  y_vec.push_back(traj_points[0].pose.position.y);
  z_vec.push_back(traj_points[0].pose.position.z);
  
  for (size_t i = 1; i < traj_points.size(); ++i) {
    double dx = traj_points[i].pose.position.x - traj_points[i-1].pose.position.x;
    double dy = traj_points[i].pose.position.y - traj_points[i-1].pose.position.y;
    double ds = std::hypot(dx, dy);
    s_vec.push_back(s_vec.back() + ds);
    x_vec.push_back(traj_points[i].pose.position.x);
    y_vec.push_back(traj_points[i].pose.position.y);
    z_vec.push_back(traj_points[i].pose.position.z);
  }
  
  const double total_length = s_vec.back();
  
  // ========================================
  // Step 2: Cubic Spline 생성 (ROS2와 동일)
  // ========================================
  // s를 매개변수로 하여 x(s), y(s), z(s)의 spline 생성
  // 이렇게 하면 arc-length parameterization이 되어 smooth한 경로 생성
  CubicSpline spline_x, spline_y, spline_z;
  spline_x.calcSplineCoefficients(s_vec, x_vec);
  spline_y.calcSplineCoefficients(s_vec, y_vec);
  spline_z.calcSplineCoefficients(s_vec, z_vec);
  
  // ========================================
  // Step 3: Uniform spacing으로 reference points 생성
  // ========================================
  const size_t num_points = static_cast<size_t>(param_.num_points);
  ref_points.reserve(num_points);
  
  for (size_t i = 0; i < num_points; ++i) {
    // Uniform arc-length sampling
    double s = (i * total_length) / (num_points - 1);
    if (i == num_points - 1) s = total_length;  // 마지막 점은 정확히 끝
    
    ReferencePoint ref_point;
    
    // Spline에서 위치 계산 (x, y, z 모두 보간)
    ref_point.pose.position.x = spline_x.interpolate(s);
    ref_point.pose.position.y = spline_y.interpolate(s);
    ref_point.pose.position.z = spline_z.interpolate(s);
    
    // Spline 미분으로 방향(yaw) 계산
    // yaw = atan2(dy/ds, dx/ds)
    double dx_ds = spline_x.derivative(s);
    double dy_ds = spline_y.derivative(s);
    double yaw = std::atan2(dy_ds, dx_ds);
    
    // ⭐ 첫 점 특수 처리: Forward difference로 yaw 계산 (ROS2 호환)
    // Spline derivative at s=0는 부정확할 수 있으므로 다음 점 방향 사용
    if (i == 0 && num_points > 1) {
      // 다음 샘플 포인트로 방향 계산
      double s_next = (1.0 * total_length) / (num_points - 1);
      double x_next = spline_x.interpolate(s_next);
      double y_next = spline_y.interpolate(s_next);
      double dx_forward = x_next - ref_point.pose.position.x;
      double dy_forward = y_next - ref_point.pose.position.y;
      yaw = std::atan2(dy_forward, dx_forward);
    }
    
    // Quaternion으로 변환
    ref_point.pose.orientation.w = std::cos(yaw / 2.0);
    ref_point.pose.orientation.x = 0.0;
    ref_point.pose.orientation.y = 0.0;
    ref_point.pose.orientation.z = std::sin(yaw / 2.0);
    
    // Velocity 보간 (arc-length 기반)
    // 간단히 선형 보간
    double ratio = s / total_length;
    size_t idx = 0;
    for (size_t j = 0; j < s_vec.size() - 1; ++j) {
      if (s_vec[j] <= s && s <= s_vec[j + 1]) {
        idx = j;
        break;
      }
    }
    if (idx >= traj_points.size() - 1) idx = traj_points.size() - 2;
    
    double local_ratio = (s_vec[idx + 1] - s_vec[idx]) > 1e-6 ?
      (s - s_vec[idx]) / (s_vec[idx + 1] - s_vec[idx]) : 0.0;
    
    ref_point.longitudinal_velocity_mps = 
      traj_points[idx].longitudinal_velocity_mps * (1 - local_ratio) +
      traj_points[idx + 1].longitudinal_velocity_mps * local_ratio;
    
    // Arc length to previous point
    if (i > 0) {
      double dx = ref_point.pose.position.x - ref_points[i-1].pose.position.x;
      double dy = ref_point.pose.position.y - ref_points[i-1].pose.position.y;
      ref_point.delta_arc_length = std::hypot(dx, dy);
    } else {
      ref_point.delta_arc_length = 0.0;
    }
    
    // Curvature 계산 (2차 미분 사용)
    // κ = |x'y'' - y'x''| / (x'^2 + y'^2)^(3/2)
    double d2x_ds2 = spline_x.secondDerivative(s);
    double d2y_ds2 = spline_y.secondDerivative(s);
    
    double numerator = std::abs(dx_ds * d2y_ds2 - dy_ds * d2x_ds2);
    double denominator = std::pow(dx_ds * dx_ds + dy_ds * dy_ds, 1.5);
    
    if (denominator > 1e-6) {
      ref_point.curvature = numerator / denominator;
    } else {
      ref_point.curvature = 0.0;
    }
    
    // Alpha 계산 (optimization center offset용)
    ref_point.alpha = ref_point.curvature * ref_point.delta_arc_length;
    
    // Normalized avoidance cost (장애물 없음)
    ref_point.normalized_avoidance_cost = 0.0;
    
    ref_points.push_back(ref_point);
  }
  
  // std::cout << "  [Spline] Generated " << ref_points.size() << " reference points" << std::endl;
  // std::cout << "  [Spline] Total path length: " << total_length << " m" << std::endl;
  // std::cout << "  [Spline] Average curvature (first 5): ";
  for (size_t i = 0; i < std::min(size_t(5), ref_points.size()); ++i) {
    // std::cout << ref_points[i].curvature << " ";
  }
  // std::cout << std::endl;
  
  return ref_points;
}

std::vector<Bounds> MPTOptimizer::calculateBounds(
  const std::vector<ReferencePoint> & ref_points,
  const std::vector<Point> & left_bound,
  const std::vector<Point> & right_bound) const
{
  std::vector<Bounds> bounds;
  bounds.reserve(ref_points.size());
  
  // Calculate bounds based on drivable area
  for (const auto & ref_point : ref_points) {
    Bounds bound;
    
    // Calculate distance to left boundary (positive direction)
    double min_left_dist = 1e6;
    for (const auto & left_pt : left_bound) {
      const double dx = left_pt.x - ref_point.pose.position.x;
      const double dy = left_pt.y - ref_point.pose.position.y;
      const double dist = std::hypot(dx, dy);
      min_left_dist = std::min(min_left_dist, dist);
    }
    
    // Calculate distance to right boundary (negative direction)
    double min_right_dist = 1e6;
    for (const auto & right_pt : right_bound) {
      const double dx = right_pt.x - ref_point.pose.position.x;
      const double dy = right_pt.y - ref_point.pose.position.y;
      const double dist = std::hypot(dx, dy);
      min_right_dist = std::min(min_right_dist, dist);
    }
    
    // Reference path yaw
    double yaw = getYaw(ref_point.pose.orientation);
    
    // Calculate lateral distance to boundaries
    // Left is positive, right is negative
    double left_lat_dist = min_left_dist;
    double right_lat_dist = min_right_dist;
    
    // Set bounds with margin
    const double margin = 0.3;  // 30cm safety margin
    bound.upper_bound = left_lat_dist - margin;    // Left side (positive)
    bound.lower_bound = -(right_lat_dist - margin);  // Right side (negative)
    
    // Ensure bounds are reasonable
    bound.upper_bound = std::min(bound.upper_bound, 2.0);  // Max 2m deviation
    bound.lower_bound = std::max(bound.lower_bound, -2.0);
    
    bounds.push_back(bound);
  }
  
  // Debug: Print first few bounds
  // std::cout << "  - Bounds for first 3 points:" << std::endl;
  for (size_t i = 0; i < std::min(size_t(3), bounds.size()); ++i) {
    // std::cout << "    [" << i << "] lower=" << bounds[i].lower_bound 
    // << ", upper=" << bounds[i].upper_bound << std::endl;
  }
  
  return bounds;
}

bool MPTOptimizer::solveQP(
  std::vector<ReferencePoint> & ref_points,
  const std::vector<Bounds> & bounds,
  const KinematicState & ego_state)
{
#ifdef USE_SIMPLE_OPTIMIZATION
  // ================================================================
  // ITERATIVE GRADIENT DESCENT OPTIMIZATION (No OSQP)
  // ================================================================
  // Based on autoware_path_optimizer QP formulation:
  // min: tracking_error + collision_avoidance + smoothness + center_bias
  // Solved with gradient descent instead of QP solver
  
  // std::cout << "[MPTOptimizer] Using ITERATIVE GRADIENT DESCENT optimization..." << std::endl;
  
  const size_t N = ref_points.size();
  
  // Print first few bounds for debugging
  // std::cout << "  - Bounds for first 3 points:" << std::endl;
  for (size_t i = 0; i < std::min(size_t(3), N); ++i) {
    // std::cout << "    [" << i << "] lower=" << bounds[i].lower_bound 
    // << ", upper=" << bounds[i].upper_bound << std::endl;
  }
  
  // std::cout << "  - Ego initial state: lat=" << ego_state.lat 
  // << ", yaw=" << ego_state.yaw << " rad" << std::endl;
  
  // Decision variables: lateral offsets for each point
  std::vector<double> lateral_offsets(N);
  
  // Initialize with lane center (better than lat=0 which might violate bounds)
  lateral_offsets[0] = ego_state.lat;
  for (size_t i = 1; i < N; ++i) {
    // Start at geometric center of drivable area for better convergence
    double center = (bounds[i].lower_bound + bounds[i].upper_bound) / 2.0;
    lateral_offsets[i] = center;
  }
  
  // Cost function weights (based on ROS2 autoware_path_optimizer)
  const double w_tracking = param_.lat_error_weight;          // Track reference path (lat=0)
  const double w_collision = 50.0;                            // Stay within bounds
  const double w_smoothness = param_.steer_rate_weight * 5.0; // Smooth trajectory
  const double w_center_bias = 3.0;                           // Bias towards lane center in narrow lanes
  
  // std::cout << "  - Weights: tracking=" << w_tracking 
  // << ", collision=" << w_collision 
  // << ", smoothness=" << w_smoothness 
  // << ", center_bias=" << w_center_bias << std::endl;
  
  // Gradient descent parameters
  const int max_iterations = 150;
  const double learning_rate = 0.02;
  const double convergence_threshold = 1e-4;
  
  double prev_cost = std::numeric_limits<double>::max();
  
  for (int iter = 0; iter < max_iterations; ++iter) {
    // Compute gradient for each variable
    std::vector<double> gradients(N, 0.0);
    double total_cost = 0.0;
    
    for (size_t i = 0; i < N; ++i) {
      double grad = 0.0;
      double lat = lateral_offsets[i];
      
      // 1. Tracking cost: ∂/∂lat (lat - 0)^2 = 2 * lat
      grad += w_tracking * 2.0 * lat;
      total_cost += w_tracking * lat * lat;
      
      // 2. Collision avoidance: Soft quadratic penalty near bounds
      const double margin = 0.20;  // Start penalizing 20cm from bound
      
      double dist_to_lower = lat - bounds[i].lower_bound;
      if (dist_to_lower < margin) {
        double violation = margin - dist_to_lower;
        double penalty = w_collision * violation * violation;  // Quadratic penalty
        grad -= w_collision * 2.0 * violation;  // Gradient
        total_cost += penalty;
      }
      
      double dist_to_upper = bounds[i].upper_bound - lat;
      if (dist_to_upper < margin) {
        double violation = margin - dist_to_upper;
        double penalty = w_collision * violation * violation;
        grad += w_collision * 2.0 * violation;
        total_cost += penalty;
      }
      
      // 3. Smoothness: Penalize acceleration (second derivative)
      if (i > 0 && i < N - 1) {
        double accel = lateral_offsets[i - 1] - 2.0 * lat + lateral_offsets[i + 1];
        grad += w_smoothness * 2.0 * (-2.0) * accel;
        total_cost += w_smoothness * accel * accel;
      } else if (i > 0) {
        double diff = lat - lateral_offsets[i - 1];
        grad += w_smoothness * 2.0 * diff;
        total_cost += w_smoothness * diff * diff;
      }
      
      // 4. Lane center bias (especially in narrow lanes)
      const double lane_width = bounds[i].upper_bound - bounds[i].lower_bound;
      const double center = (bounds[i].lower_bound + bounds[i].upper_bound) / 2.0;
      
      if (lane_width < 3.5) {
        double center_error = lat - center;
        double bias_weight = w_center_bias * (1.0 - lane_width / 3.5);
        grad += bias_weight * 2.0 * center_error;
        total_cost += bias_weight * center_error * center_error;
      }
      
      gradients[i] = grad;
    }
    
    // Check convergence
    double cost_change = std::abs(total_cost - prev_cost);
    if (iter % 10 == 0) {
      // std::cout << "  - Iteration " << iter << ": cost=" << total_cost 
      // << ", change=" << cost_change << std::endl;
    }
    
    if (cost_change < convergence_threshold && iter > 10) {
      // std::cout << "  - Converged at iteration " << iter << std::endl;
      break;
    }
    
    prev_cost = total_cost;
    
    // Update variables (except first point - fixed by ego)
    for (size_t i = 1; i < N; ++i) {
      lateral_offsets[i] -= learning_rate * gradients[i];
      
      // Project to feasible region
      lateral_offsets[i] = std::max(bounds[i].lower_bound + 0.05, 
                                    std::min(bounds[i].upper_bound - 0.05, lateral_offsets[i]));
    }
  }
  
  // Apply results to reference points
  for (size_t i = 0; i < N; ++i) {
    ref_points[i].optimized_kinematic_state.lat = lateral_offsets[i];
    ref_points[i].optimized_kinematic_state.yaw = 0.0;
    ref_points[i].optimized_input = 0.0;
  }
  
  // Debug: print first few optimized values
  // std::cout << "  - First 5 optimized lateral offsets:" << std::endl;
  for (size_t i = 0; i < std::min(size_t(5), N); ++i) {
    // std::cout << "    [" << i << "] lat=" << lateral_offsets[i] 
    // << ", bounds=[" << bounds[i].lower_bound << ", " << bounds[i].upper_bound << "]"
    // << std::endl;
  }
  
  // std::cout << "[MPTOptimizer] Iterative optimization successful" << std::endl;
  return true;
  
#elif defined(USE_OSQP)
  // std::cout << "[MPTOptimizer] Solving QP problem with OSQP..." << std::endl;
  
  const size_t N = ref_points.size();
  const size_t D_x = state_equation_generator_->getDimX();  // 2
  const size_t D_u = state_equation_generator_->getDimU();  // 1
  
  // Calculate state equation matrices: X = B*U + W
  auto mpt_mat = state_equation_generator_->calcMatrix(ref_points);
  
  // Debug: Check delta_arc_length values
  // std::cout << "  - Delta arc lengths (first 3): ";
  for (size_t i = 0; i < std::min(size_t(3), ref_points.size()); ++i) {
    // std::cout << ref_points[i].delta_arc_length << " ";
  }
  // std::cout << std::endl;
  
  const size_t N_x = N * D_x;      // Total state dimension
  const size_t N_u = (N - 1) * D_u;  // Total input dimension
  
  // Build QP objective: min 0.5 * U' * H * U + g' * U
  // We want to minimize: (X - X_ref)' Q (X - X_ref) + U' R U + ΔU' R_rate ΔU
  // where X = B*U + W, ΔU = diff(U) = D*U
  // 
  // Tracking cost: (X - X_ref)' Q (X - X_ref) = (B*U + W - X_ref)' Q (B*U + W - X_ref)
  //              = U' (B'QB) U + 2*(W - X_ref)' Q B U + constant
  //
  // Control cost: U' R U
  //
  // Smoothness cost: (D*U)' R_rate (D*U) = U' (D' R_rate D) U
  //
  // Total Hessian: H = B'QB + R + D' R_rate D
  // Total gradient: g = B' Q (W - X_ref)
  
  // Set X_ref = 0 (ROS2 default: minimize deviation from reference path)
  Eigen::VectorXd X_ref = Eigen::VectorXd::Zero(N_x);
  
  // Create sparse Q matrix (state cost) with ADAPTIVE WEIGHTS (ROS2 compatibility)
  std::vector<Eigen::Triplet<double>> Q_triplets;
  
  // Check if last point matches the goal (full path optimization)
  const bool is_goal_contained = (N > 0);  // Simplified: assume always optimizing toward goal
  
  for (size_t i = 0; i < N; ++i) {
    // Adaptive weight calculation based on point position and avoidance cost
    double adaptive_lat_weight, adaptive_yaw_weight;
    
    if (i == N - 1) {
      // Terminal point: use very strong tracking weights
      if (is_goal_contained) {
        adaptive_lat_weight = param_.goal_lat_error_weight;
        adaptive_yaw_weight = param_.goal_yaw_error_weight;
      } else {
        adaptive_lat_weight = param_.terminal_lat_error_weight;
        adaptive_yaw_weight = param_.terminal_yaw_error_weight;
      }
    } else if (ref_points[i].normalized_avoidance_cost > 0.0) {
      // Avoidance region: interpolate between normal and avoidance weights
      // Higher avoidance_cost => lower lateral tracking weight (allow deviation)
      const double t = ref_points[i].normalized_avoidance_cost;  // [0, 1]
      adaptive_lat_weight = param_.lat_error_weight * (1.0 - t) + 0.0 * t;  // 0.0 = avoidance_lat_error_weight
      adaptive_yaw_weight = param_.yaw_error_weight * (1.0 - t) + 0.0 * t;
    } else {
      // Normal tracking
      adaptive_lat_weight = param_.lat_error_weight;
      adaptive_yaw_weight = param_.yaw_error_weight;
    }
    
    Q_triplets.push_back(Eigen::Triplet<double>(i * D_x, i * D_x, adaptive_lat_weight));
    Q_triplets.push_back(Eigen::Triplet<double>(i * D_x + 1, i * D_x + 1, adaptive_yaw_weight));
  }
  
  Eigen::SparseMatrix<double> Q(N_x, N_x);
  Q.setFromTriplets(Q_triplets.begin(), Q_triplets.end());
  
  // Apply OPTIMIZATION CENTER OFFSET transformation (ROS2 compatibility)
  // This shifts the optimization reference point forward by offset distance
  Eigen::SparseMatrix<double> T_sparse(N_x, N_x);
  Eigen::VectorXd T_vec = Eigen::VectorXd::Zero(N_x);
  
  if (std::abs(param_.optimization_center_offset) > 1e-6) {
    std::vector<Eigen::Triplet<double>> T_triplets;
    const double offset = param_.optimization_center_offset;
    
    for (size_t i = 0; i < N; ++i) {
      const double alpha = ref_points[i].alpha;  // Curvature angle
      
      // T matrix: transforms state considering forward offset
      // lat_transformed = cos(alpha) * lat + offset*cos(alpha) * yaw
      T_triplets.push_back(Eigen::Triplet<double>(i * D_x, i * D_x, std::cos(alpha)));
      T_triplets.push_back(Eigen::Triplet<double>(i * D_x, i * D_x + 1, offset * std::cos(alpha)));
      T_triplets.push_back(Eigen::Triplet<double>(i * D_x + 1, i * D_x + 1, 1.0));
      
      // T_vec: constant offset term\n      T_vec(i * D_x) = -offset * std::sin(alpha);
    }
    
    T_sparse.setFromTriplets(T_triplets.begin(), T_triplets.end());
    
    // Transform Q matrix: Q_transformed = T' * Q * T
    Eigen::SparseMatrix<double> Q_transformed = T_sparse.transpose() * Q * T_sparse;
    Q = Q_transformed;
  } else {
    // No transformation, use identity
    T_sparse.setIdentity();
  }
  
  // Build Hessian and gradient
  Eigen::MatrixXd B_dense = mpt_mat.B;
  Eigen::MatrixXd Q_dense = Eigen::MatrixXd(Q);
  
  // 1. Tracking term: B' Q B
  Eigen::MatrixXd H_u = B_dense.transpose() * Q_dense * B_dense;
  
  // 2. Control cost: R (diagonal) with increased weight for stability
  const double w_steer = std::max(param_.weight_steer_input, 0.5);  // Minimum 0.5 for stability
  for (size_t i = 0; i < N_u; ++i) {
    H_u(i, i) += w_steer;
  }
  
  // 2b. Tikhonov regularization for numerical stability (especially at 50+ points)
  const double tikhonov_eps = 1e-6;  // Small regularization to prevent singularity
  for (size_t i = 0; i < N_u; ++i) {
    H_u(i, i) += tikhonov_eps;
  }
  
  // 3. Smoothness cost: D' R_rate D (steering rate penalty for smooth path)
  const double w_steer_rate = param_.steer_rate_weight;
  if (w_steer_rate > 1e-9 && N_u >= 2) {
    // D'D has tridiagonal structure for difference operator
    // First diagonal element
    H_u(0, 0) += w_steer_rate;
    
    // Off-diagonal elements (symmetric)
    for (size_t i = 0; i < N_u - 1; ++i) {
      H_u(i, i + 1) += -w_steer_rate;
      H_u(i + 1, i) += -w_steer_rate;
    }
    
    // Remaining diagonal elements
    for (size_t i = 1; i < N_u; ++i) {
      H_u(i, i) += w_steer_rate;
    }
  }
  
  // Override W[0] with actual ego initial state BEFORE gradient calculation
  mpt_mat.W(0) = ego_state.lat;  // Initial lateral offset
  mpt_mat.W(1) = ego_state.yaw;  // Initial yaw error
  
  // Gradient: g = B' Q (W - X_ref)
  // If optimization center offset is used, apply T transformation to the gradient
  Eigen::VectorXd error = mpt_mat.W - X_ref;
  Eigen::VectorXd g;
  
  if (std::abs(param_.optimization_center_offset) > 1e-6) {
    // g = B' * Q * (W - X_ref) + B' * Q * T_vec
    // where Q already includes T transformation: Q = T' * Q_original * T
    Eigen::VectorXd T_vec_contrib = Eigen::VectorXd::Zero(N_x);
    const double offset = param_.optimization_center_offset;
    for (size_t i = 0; i < N; ++i) {
      T_vec_contrib(i * D_x) = -offset * std::sin(ref_points[i].alpha);
    }
    // Transform error through T_sparse (implicit in Q), add T_vec contribution
    g = B_dense.transpose() * Q_dense * (error + T_vec_contrib);
  } else {
    g = B_dense.transpose() * Q_dense * error;
  }
  
  // std::cout << "  - Ego state: lat=" << ego_state.lat << ", yaw=" << ego_state.yaw << std::endl;
  
  // Debug initial state
  // std::cout << "  - Initial W (first 5 lateral states): ";
  for (size_t i = 0; i < std::min(size_t(5), N); ++i) {
    // std::cout << mpt_mat.W(i * D_x) << " ";
  }
  // std::cout << std::endl;
  // std::cout << "  - Initial W (first 5 yaw states): ";
  for (size_t i = 0; i < std::min(size_t(5), N); ++i) {
    // std::cout << mpt_mat.W(i * D_x + 1) << " ";
  }
  // std::cout << std::endl;
  
  // Debug B matrix structure
  // std::cout << "  - B matrix size: " << mpt_mat.B.rows() << " x " << mpt_mat.B.cols() << std::endl;
  if (mpt_mat.B.rows() >= 4 && mpt_mat.B.cols() >= 1) {
    // std::cout << "  - B[2:4, 0] (X[1] from U[0]): [" << mpt_mat.B(2, 0) << ", " << mpt_mat.B(3, 0) << "]" << std::endl;
  }
  if (mpt_mat.B.rows() >= 6 && mpt_mat.B.cols() >= 2) {
    // std::cout << "  - B[4:6, 1] (X[2] from U[1]): [" << mpt_mat.B(4, 1) << ", " << mpt_mat.B(5, 1) << "]" << std::endl;
  }
  
  // Build constraints
  std::vector<Eigen::Triplet<double>> A_triplets;
  std::vector<double> lower_bounds;
  std::vector<double> upper_bounds;
  
  int constraint_idx = 0;
  
  // 1. Lateral bounds: lower <= B*U + W <= upper
  for (size_t i = 0; i < N; ++i) {
    // Extract row from B for this lateral state
    for (size_t j = 0; j < N_u; ++j) {
      double val = B_dense(i * D_x, j);
      if (std::abs(val) > 1e-9) {
        A_triplets.push_back(Eigen::Triplet<double>(constraint_idx, j, val));
      }
    }
    
    // Bounds: lower <= B*U + W[i*D_x] <= upper
    lower_bounds.push_back(bounds[i].lower_bound - mpt_mat.W(i * D_x));
    upper_bounds.push_back(bounds[i].upper_bound - mpt_mat.W(i * D_x));
    constraint_idx++;
  }
  
  // 2. Steering input bounds
  const double max_steer = vehicle_info_.max_steer_angle_rad;
  for (size_t i = 0; i < N_u; ++i) {
    A_triplets.push_back(Eigen::Triplet<double>(constraint_idx, i, 1.0));
    lower_bounds.push_back(-max_steer);
    upper_bounds.push_back(max_steer);
    constraint_idx++;
  }
  
  // Convert to sparse matrix
  Eigen::SparseMatrix<double> A_sparse(constraint_idx, N_u);
  A_sparse.setFromTriplets(A_triplets.begin(), A_triplets.end());
  
  // Convert to CSC format for OSQP
  Eigen::MatrixXd A_dense = Eigen::MatrixXd(A_sparse);
  
  CSC_Matrix P_csc = calCSCMatrixTrapezoidal(H_u);
  CSC_Matrix A_csc = calCSCMatrix(A_dense);
  
  std::vector<double> q_vec(g.data(), g.data() + g.size());
  
  // Create OSQP interface
  OSQPInterface osqp(P_csc, A_csc, q_vec, lower_bounds, upper_bounds, 1e-4);
  
  // ⭐ Apply warm start if previous solution exists (ROS2 temporal consistency)
  if (has_prev_solution_ && !prev_optimized_solution_.empty()) {
    // Check if dimensions match
    if (prev_optimized_solution_.size() == N_u) {
      // std::cout << "  - Applying warm start (size=" << prev_optimized_solution_.size() << ")" << std::endl;
      osqp.setWarmStart(prev_optimized_solution_);
    } else {
      // std::cout << "  - Warm start size mismatch: prev=" << prev_optimized_solution_.size()
      // << ", current=" << N_u << " - skip" << std::endl;
    }
  } else {
    // std::cout << "  - No warm start (first iteration)" << std::endl;
  }
  
  // Solve
  auto [solution, dual, polish, status, iter] = osqp.optimize();
  
  if (status != 1) {  // OSQP_SOLVED
    // std::cerr << "[MPTOptimizer] OSQP failed with status: " << status << std::endl;
    return false;
  }
  
  // std::cout << "  - OSQP solved in " << iter << " iterations" << std::endl;
  
  // ⭐ Save solution for warm start in next iteration
  prev_optimized_solution_ = solution;
  // std::cout << "  - Saved solution for warm start (size=" << solution.size() << ")" << std::endl;
  
  // Convert solution to states: X = B*U + W
  Eigen::VectorXd U(N_u);
  for (size_t i = 0; i < N_u; ++i) {
    U(i) = solution[i];
  }
  
  Eigen::VectorXd X = state_equation_generator_->predict(mpt_mat, U);
  
  // Apply solution to reference points
  for (size_t i = 0; i < N; ++i) {
    ref_points[i].optimized_kinematic_state.lat = X(i * D_x);
    ref_points[i].optimized_kinematic_state.yaw = X(i * D_x + 1);
    
    if (i < N - 1) {
      ref_points[i].optimized_input = solution[i];
    }
  }
  
  // Debug: Print first few optimized values
  // std::cout << "  - First 5 optimized lateral offsets:" << std::endl;
  for (size_t i = 0; i < std::min(size_t(5), N); ++i) {
    // std::cout << "    [" << i << "] lat=" << ref_points[i].optimized_kinematic_state.lat 
    // << ", yaw=" << ref_points[i].optimized_kinematic_state.yaw 
    // << ", ref_x=" << ref_points[i].pose.position.x << std::endl;
  }
  
  return true;
  
#else
  // Simplified QP solver - fallback when OSQP not available
  // std::cout << "[MPTOptimizer] Solving QP problem (simplified solver)..." << std::endl;
  // std::cout << "  - Note: OSQP not available, using simplified solver" << std::endl;
  
  for (size_t i = 0; i < ref_points.size(); ++i) {
    // Keep points within bounds
    double lat_offset = 0.0;  // Simplified - stay on reference
    
    // Clamp to bounds
    if (lat_offset < bounds[i].lower_bound) {
      lat_offset = bounds[i].lower_bound;
    }
    if (lat_offset > bounds[i].upper_bound) {
      lat_offset = bounds[i].upper_bound;
    }
    
    ref_points[i].optimized_kinematic_state.lat = lat_offset;
    ref_points[i].optimized_kinematic_state.yaw = 0.0;
    ref_points[i].optimized_input = 0.0;
  }
  
  return true;
#endif
}

std::vector<TrajectoryPoint> MPTOptimizer::convertToTrajectory(
  const std::vector<ReferencePoint> & ref_points) const
{
  std::vector<TrajectoryPoint> trajectory;
  trajectory.reserve(ref_points.size());
  
  for (size_t i = 0; i < ref_points.size(); ++i) {
    const auto & ref_point = ref_points[i];
    TrajectoryPoint traj_point;
    traj_point.longitudinal_velocity_mps = ref_point.longitudinal_velocity_mps;
    
    // Calculate path tangent direction from adjacent points
    double path_yaw = 0.0;
    if (i < ref_points.size() - 1) {
      const auto & next_point = ref_points[i + 1];
      double dx = next_point.pose.position.x - ref_point.pose.position.x;
      double dy = next_point.pose.position.y - ref_point.pose.position.y;
      path_yaw = std::atan2(dy, dx);
    } else if (i > 0) {
      const auto & prev_point = ref_points[i - 1];
      double dx = ref_point.pose.position.x - prev_point.pose.position.x;
      double dy = ref_point.pose.position.y - prev_point.pose.position.y;
      path_yaw = std::atan2(dy, dx);
    }
    
    // Apply lateral offset from optimization
    double lat_offset = ref_point.optimized_kinematic_state.lat;
    double yaw_error = ref_point.optimized_kinematic_state.yaw;
    
    // Final position = reference position + lateral offset in normal direction
    traj_point.pose.position.x = ref_point.pose.position.x + lat_offset * std::cos(path_yaw + M_PI / 2.0);
    traj_point.pose.position.y = ref_point.pose.position.y + lat_offset * std::sin(path_yaw + M_PI / 2.0);
    traj_point.pose.position.z = ref_point.pose.position.z;
    
    // Final orientation = path yaw + yaw error from optimization
    double final_yaw = path_yaw + yaw_error;
    traj_point.pose.orientation.x = 0.0;
    traj_point.pose.orientation.y = 0.0;
    traj_point.pose.orientation.z = std::sin(final_yaw / 2.0);
    traj_point.pose.orientation.w = std::cos(final_yaw / 2.0);
    
    trajectory.push_back(traj_point);
  }
  
  return trajectory;
}

void MPTOptimizer::updateFixedPoint(
  std::vector<ReferencePoint> & ref_points)
{
  // ROS2의 핵심 메커니즘: 이전 optimization 결과를 첫 점에 적용하여 temporal consistency 유지
  if (!has_prev_solution_ || prev_ref_points_.empty()) {
    // No previous data - skip fixed point
    // std::cout << "[MPTOptimizer] No previous data - skip fixed point" << std::endl;
    return;
  }
  
  if (ref_points.empty()) {
    return;
  }
  
  // ⭐ 핵심: 첫 점의 pose와 optimized state를 이전 결과로 교체
  // 이렇게 하면 trajectory가 smooth하게 이어지고, 수렴이 빨라짐
  // ROS2는 vehicle이 움직이면서 계속 replanning하므로 temporal consistency가 중요
  
  // Find closest previous point to current first point (ego position)
  size_t closest_idx = 0;
  double min_dist = std::numeric_limits<double>::max();
  
  for (size_t i = 0; i < prev_ref_points_.size(); ++i) {
    double dx = prev_ref_points_[i].pose.position.x - ref_points[0].pose.position.x;
    double dy = prev_ref_points_[i].pose.position.y - ref_points[0].pose.position.y;
    double dist = std::sqrt(dx*dx + dy*dy);
    
    if (dist < min_dist) {
      min_dist = dist;
      closest_idx = i;
    }
  }
  
  // If closest point is nearby (< 1.0m), use it as fixed point
  if (min_dist < 1.0) {
    // std::cout << "[MPTOptimizer] Fixed point: using prev[" << closest_idx 
    // << "] (dist=" << min_dist << "m)" << std::endl;
    
    // ⭐ Replace first point's pose and optimized state
    ref_points[0].pose = prev_ref_points_[closest_idx].pose;
    ref_points[0].optimized_kinematic_state = prev_ref_points_[closest_idx].optimized_kinematic_state;
    
    // std::cout << "    -> lat=" << ref_points[0].optimized_kinematic_state.lat
    // << ", yaw=" << ref_points[0].optimized_kinematic_state.yaw << std::endl;
  } else {
    // std::cout << "[MPTOptimizer] Fixed point: prev too far (" << min_dist << "m) - skip" << std::endl;
  }
}

}  // namespace autoware::path_optimizer
