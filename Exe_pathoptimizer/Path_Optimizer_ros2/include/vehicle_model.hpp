// Vehicle Model for Path Optimizer
#ifndef PATH_OPTIMIZER__VEHICLE_MODEL_HPP_
#define PATH_OPTIMIZER__VEHICLE_MODEL_HPP_

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>

namespace autoware::path_optimizer
{

class VehicleModel
{
public:
  VehicleModel(const double wheelbase, const double steer_limit)
  : wheelbase_(wheelbase), steer_limit_(steer_limit)
  {
  }

  int getDimX() const { return 2; }  // [lateral_error, yaw_error]
  int getDimU() const { return 1; }  // [steering_angle]

  // Calculate discrete state equation: x_{t+1} = Ad * x_t + Bd * u_t + Wd
  void calculateStateEquationMatrix(
    Eigen::MatrixXd & Ad,
    Eigen::MatrixXd & Bd,
    Eigen::MatrixXd & Wd,
    const double curvature,
    const double ds) const
  {
    const double delta_r = std::atan(wheelbase_ * curvature);
    const double cropped_delta_r = std::clamp(delta_r, -steer_limit_, steer_limit_);

    // State: [lateral_error, yaw_error]
    // Input: [steering_angle]
    
    // Ad matrix (2x2)
    Ad.resize(2, 2);
    Ad << 1.0, ds,
          0.0, 1.0;

    // Bd matrix (2x1)
    Bd.resize(2, 1);
    const double cos_delta = std::cos(delta_r);
    Bd << 0.0,
          ds / wheelbase_ / (cos_delta * cos_delta);

    // Wd vector (2x1)
    Wd.resize(2, 1);
    const double tan_cropped = std::tan(cropped_delta_r);
    const double cos_cropped = std::cos(cropped_delta_r);
    Wd << 0.0,
          -ds * curvature + ds / wheelbase_ * 
          (tan_cropped - cropped_delta_r / (cos_cropped * cos_cropped));
  }

  double getWheelbase() const { return wheelbase_; }
  double getSteerLimit() const { return steer_limit_; }

private:
  double wheelbase_;
  double steer_limit_;
};

}  // namespace autoware::path_optimizer

#endif  // PATH_OPTIMIZER__VEHICLE_MODEL_HPP_
