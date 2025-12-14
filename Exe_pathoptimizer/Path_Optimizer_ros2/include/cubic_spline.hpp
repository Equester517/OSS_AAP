// Copyright 2024 TIER IV, Inc.
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

#ifndef PATH_OPTIMIZER__CUBIC_SPLINE_HPP_
#define PATH_OPTIMIZER__CUBIC_SPLINE_HPP_

#include <vector>
#include <cmath>
#include <algorithm>

namespace autoware::path_optimizer
{

/**
 * @brief Cubic Spline 보간을 위한 클래스
 * 
 * 자연 경계 조건(natural boundary condition)을 사용한 cubic spline 구현
 * s(t) = a + b*t + c*t^2 + d*t^3 형태의 3차 다항식으로 각 구간을 보간
 */
class CubicSpline
{
public:
  CubicSpline() = default;
  
  /**
   * @brief Spline 계수 계산
   * @param x x 좌표 배열 (strictly increasing이어야 함)
   * @param y 대응하는 y 값 배열
   */
  void calcSplineCoefficients(const std::vector<double> & x, const std::vector<double> & y)
  {
    const size_t n = x.size();
    if (n < 2) {
      return;
    }
    
    x_ = x;
    y_ = y;
    
    // 계수 벡터 초기화
    a_ = y;
    b_.resize(n);
    c_.resize(n);
    d_.resize(n);
    
    if (n == 2) {
      // 선형 보간
      b_[0] = (y[1] - y[0]) / (x[1] - x[0]);
      c_[0] = 0.0;
      d_[0] = 0.0;
      return;
    }
    
    // Tridiagonal 행렬 시스템 구성
    std::vector<double> h(n - 1);
    for (size_t i = 0; i < n - 1; ++i) {
      h[i] = x[i + 1] - x[i];
    }
    
    // 자연 경계 조건: 양 끝에서 2차 미분 = 0
    std::vector<double> alpha(n);
    for (size_t i = 1; i < n - 1; ++i) {
      alpha[i] = 3.0 / h[i] * (y[i + 1] - y[i]) - 3.0 / h[i - 1] * (y[i] - y[i - 1]);
    }
    
    // Thomas 알고리즘으로 tridiagonal 시스템 풀기
    std::vector<double> l(n), mu(n), z(n);
    l[0] = 1.0;
    mu[0] = 0.0;
    z[0] = 0.0;
    
    for (size_t i = 1; i < n - 1; ++i) {
      l[i] = 2.0 * (x[i + 1] - x[i - 1]) - h[i - 1] * mu[i - 1];
      mu[i] = h[i] / l[i];
      z[i] = (alpha[i] - h[i - 1] * z[i - 1]) / l[i];
    }
    
    l[n - 1] = 1.0;
    z[n - 1] = 0.0;
    c_[n - 1] = 0.0;
    
    // Back substitution
    for (int i = static_cast<int>(n) - 2; i >= 0; --i) {
      c_[i] = z[i] - mu[i] * c_[i + 1];
      b_[i] = (y[i + 1] - y[i]) / h[i] - h[i] * (c_[i + 1] + 2.0 * c_[i]) / 3.0;
      d_[i] = (c_[i + 1] - c_[i]) / (3.0 * h[i]);
    }
  }
  
  /**
   * @brief 주어진 x 값에서 spline 값 계산
   */
  double interpolate(double x) const
  {
    if (x_.empty()) return 0.0;
    if (x <= x_.front()) return y_.front();
    if (x >= x_.back()) return y_.back();
    
    // 해당 구간 찾기
    size_t i = findSegment(x);
    
    // Cubic polynomial 계산: s(t) = a + b*t + c*t^2 + d*t^3
    double dx = x - x_[i];
    return a_[i] + b_[i] * dx + c_[i] * dx * dx + d_[i] * dx * dx * dx;
  }
  
  /**
   * @brief 주어진 x 값에서 1차 미분(기울기) 계산
   */
  double derivative(double x) const
  {
    if (x_.empty()) return 0.0;
    if (x <= x_.front()) return b_.front();
    if (x >= x_.back()) return b_.back();
    
    size_t i = findSegment(x);
    double dx = x - x_[i];
    return b_[i] + 2.0 * c_[i] * dx + 3.0 * d_[i] * dx * dx;
  }
  
  /**
   * @brief 주어진 x 값에서 2차 미분(곡률) 계산
   */
  double secondDerivative(double x) const
  {
    if (x_.empty()) return 0.0;
    if (x <= x_.front() || x >= x_.back()) return 0.0;
    
    size_t i = findSegment(x);
    double dx = x - x_[i];
    return 2.0 * c_[i] + 6.0 * d_[i] * dx;
  }
  
private:
  /**
   * @brief x가 속한 구간의 인덱스 찾기 (binary search)
   */
  size_t findSegment(double x) const
  {
    auto it = std::lower_bound(x_.begin(), x_.end(), x);
    if (it == x_.end()) return x_.size() - 2;
    if (it == x_.begin()) return 0;
    return std::distance(x_.begin(), it) - 1;
  }
  
  std::vector<double> x_;  // x 좌표
  std::vector<double> y_;  // y 값
  std::vector<double> a_;  // Spline 계수
  std::vector<double> b_;
  std::vector<double> c_;
  std::vector<double> d_;
};

}  // namespace autoware::path_optimizer

#endif  // PATH_OPTIMIZER__CUBIC_SPLINE_HPP_
