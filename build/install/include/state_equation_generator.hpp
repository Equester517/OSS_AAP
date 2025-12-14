// State Equation Generator for Path Optimizer
#ifndef PATH_OPTIMIZER__STATE_EQUATION_GENERATOR_HPP_
#define PATH_OPTIMIZER__STATE_EQUATION_GENERATOR_HPP_

#include "vehicle_model.hpp"
#include "path_optimizer_types.hpp"

#include <Eigen/Dense>
#include <memory>
#include <vector>

namespace autoware::path_optimizer
{

class StateEquationGenerator
{
public:
  struct Matrix
  {
    Eigen::MatrixXd A;  // State transition matrix
    Eigen::MatrixXd B;  // Input matrix
    Eigen::VectorXd W;  // Offset vector
  };

  StateEquationGenerator() = default;
  
  StateEquationGenerator(const double wheelbase, const double max_steer_rad)
  : vehicle_model_(std::make_unique<VehicleModel>(wheelbase, max_steer_rad))
  {
  }

  int getDimX() const { return vehicle_model_->getDimX(); }
  int getDimU() const { return vehicle_model_->getDimU(); }

  // Calculate time-series state equation: X = B * U + W
  // where X is state vector for all time steps, U is input vector for all time steps
  Matrix calcMatrix(const std::vector<ReferencePoint> & ref_points) const
  {
    const size_t D_x = vehicle_model_->getDimX();
    const size_t D_u = vehicle_model_->getDimU();

    const size_t N_ref = ref_points.size();
    const size_t N_x = N_ref * D_x;
    const size_t N_u = (N_ref - 1) * D_u;

    // Matrices for whole state equation
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(N_x, N_x);
    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(N_x, N_u);
    Eigen::VectorXd W = Eigen::VectorXd::Zero(N_x);

    // Matrices for one-step state equation
    Eigen::MatrixXd Ad, Bd, Wd;

    // Initial state at X[0] will be set from ego vehicle state
    // Leave W[0] as zero for now, will be overridden in solveQP
    W.segment(0, D_x) = Eigen::VectorXd::Zero(D_x);

    // Calculate state equations for each time step using recurrence:
    // X[k+1] = Ad * X[k] + Bd * U[k] + Wd
    // 
    // In matrix form: X = A_cum * X[0] + B_cum * U + W_cum
    // where A_cum, B_cum, W_cum propagate through state transitions
    
    for (size_t i = 1; i < N_ref; ++i) {
      const auto & p = ref_points[i - 1];
      
      // Get discrete kinematics matrix Ad, Bd, Wd
      // NOTE: Using curvature = 0.0 for stability (same as ROS2 version)
      vehicle_model_->calculateStateEquationMatrix(Ad, Bd, Wd, 0.0, p.delta_arc_length);

      // Update W: W[i] = Ad * W[i-1] + Wd (cumulative propagation)
      W.segment(i * D_x, D_x) = Ad * W.segment((i - 1) * D_x, D_x) + Wd;
      
      // Update B: B[i,:] propagates previous B through Ad, plus current Bd
      // B[i, k] = Ad * B[i-1, k]  for k < i-1
      // B[i, i-1] = Bd
      for (size_t k = 0; k < i - 1; ++k) {
        B.block(i * D_x, k * D_u, D_x, D_u) = Ad * B.block((i - 1) * D_x, k * D_u, D_x, D_u);
      }
      B.block(i * D_x, (i - 1) * D_u, D_x, D_u) = Bd;
      
      // A is not used in X = B*U + W formulation (initial state absorbed into W)
      // But we keep it for reference: A[i, i-1] = Ad
      A.block(i * D_x, (i - 1) * D_x, D_x, D_x) = Ad;
    }

    return Matrix{A, B, W};
  }

  Eigen::VectorXd predict(const Matrix & mat, const Eigen::VectorXd & U) const
  {
    return mat.B * U + mat.W;
  }

private:
  std::unique_ptr<VehicleModel> vehicle_model_;
};

}  // namespace autoware::path_optimizer

#endif  // PATH_OPTIMIZER__STATE_EQUATION_GENERATOR_HPP_
